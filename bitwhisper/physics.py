"""Thermal physics of the BitWhisper air-gap channel.

Models, in discrete time:
  Transmitter  -- PC chassis/exhaust temperature elevation driven by CPU load
                  (fast convective + slow conductive paths).
  Air gap      -- advective dead-time (0.35 min/cm, V.B.2) + local mixing lag,
                  then exponential attenuation vs distance.
  Receiver     -- ambient thermal sensor with lag, 1C quantization, noise,
                  and slow room-temperature drift.

All parameters below are taken from the measurements in Guri et al. (2015);
see module docstring of bitwhisper/__init__.py for the mapping.
"""

from __future__ import annotations

import math
import random
from bisect import bisect_left
from collections import deque
from dataclasses import dataclass, field


# ---------------------------------------------------------------------------
# Layout presets (Sec V.B.3-4 of the paper)
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class LayoutParams:
    name: str
    dead_base_min: float      # advective dead-time at 0 cm
    dead_per_cm_min: float    # extra dead-time per cm (V.B.2: ~0.35 min/cm,
    lag_base_min: float       # local mixing lag at 0 cm      split dead/lag)
    lag_per_cm_min: float
    atten_scale: float       # multiplier on thermal coupling
    rx_lag_min: float = 2.0  # receiver ambient-sensor thermal inertia


LAYOUTS = {
    # Parallel (side-by-side): 3 min to first +1C at ~0 cm, +4C max (V.B.2).
    "parallel": LayoutParams("parallel", dead_base_min=0.5,
                            dead_per_cm_min=0.175, lag_base_min=1.5,
                            lag_per_cm_min=0.175, atten_scale=1.0),
    # Stacked: 5 min (top heats) / 12 min (bottom heats) delay (Table 3).
    "stacked": LayoutParams("stacked", dead_base_min=2.0,
                           dead_per_cm_min=0.175, lag_base_min=3.0,
                           lag_per_cm_min=0.175, atten_scale=0.75),
    # Face-away (backs to each other): ~10 min delay, <=+1C (V.B.4, Fig 8).
    "face-away": LayoutParams("face-away", dead_base_min=4.0,
                             dead_per_cm_min=0.175, lag_base_min=6.0,
                             lag_per_cm_min=0.175, atten_scale=0.25),
    # Quadrature (exhaust aimed at receiver): 115 s delay at 6 cm (V.B.4).
    "quadrature": LayoutParams("quadrature", dead_base_min=0.5,
                              dead_per_cm_min=0.175, lag_base_min=1.0,
                              lag_per_cm_min=0.175, atten_scale=1.0),
}

# Coupling calibrated so a full-blast transmitter at 0 cm in the parallel
# layout drives the receiver's ambient sensor +4C (V.B.2: "+4 after 26 min").
_TX_EXHAUST_MAX_C = 15.0          # TX exhaust elevation at 100% CPU (C)
_RX_MAX_C_AT_ZERO_CM = 4.0       # RX ambient elevation, 0 cm, parallel
_COUPLING_0 = _RX_MAX_C_AT_ZERO_CM / _TX_EXHAUST_MAX_C
_ATTEN_LENGTH_CM = 25.0          # exp falloff; ~1C left at 30-35 cm (V.B.2)


# ---------------------------------------------------------------------------
# Transmitter: chassis/exhaust thermal model
# ---------------------------------------------------------------------------

class Transmitter:
    """Two-path thermal model of the transmitting PC.

    Heat reaches a neighbor via two paths (paper Sec V.A/V.B):
      * fast convective path (exhaust airflow): when the CPU loads, hot air
        leaves the case within ~1 min; when idle, the exhaust cools in ~1-2
        min.  Dominates the *ripple* the receiver sees per symbol.
      * slow conductive path (warm chassis radiation): tau ~8-20 min.
        Dominates the *ramp* (inter-symbol interference).

    This split is what makes the paper's two end-to-end numbers consistent:
    +1C after ~3 min (fast path) yet +4C only after ~26 min (slow buildup).
    """

    def __init__(self,
                 e_fast_c: float = 8.0, tau_fast_heat: float = 1.0,
                 tau_fast_cool: float = 1.5,
                 e_slow_c: float = 7.0, tau_slow_heat: float = 8.0,
                 tau_slow_cool: float = 18.0):
        self.e_fast_max = e_fast_c
        self.e_slow_max = e_slow_c
        self.tau_fast_heat = tau_fast_heat
        self.tau_fast_cool = tau_fast_cool
        self.tau_slow_heat = tau_slow_heat
        self.tau_slow_cool = tau_slow_cool
        self.elev_fast = 0.0
        self.elev_slow = 0.0

    @property
    def e_max(self) -> float:
        return self.e_fast_max + self.e_slow_max

    @property
    def elev(self) -> float:
        return self.elev_fast + self.elev_slow

    @staticmethod
    def _step_one(elev: float, target: float, dt_min: float,
                  tau_heat: float, tau_cool: float) -> float:
        tau = tau_heat if target > elev else tau_cool
        return elev + (target - elev) * (1.0 - math.exp(-dt_min / tau))

    def step(self, dt_min: float, load: float) -> float:
        """Advance by dt_min under CPU load in [0, 1]; return elevation (C)."""
        load = min(1.0, max(0.0, load))
        self.elev_fast = self._step_one(self.elev_fast, self.e_fast_max * load,
                                       dt_min, self.tau_fast_heat,
                                       self.tau_fast_cool)
        self.elev_slow = self._step_one(self.elev_slow, self.e_slow_max * load,
                                       dt_min, self.tau_slow_heat,
                                       self.tau_slow_cool)
        return self.elev


# ---------------------------------------------------------------------------
# Receiver: ambient thermal sensor model
# ---------------------------------------------------------------------------

class Receiver:
    """Ambient (motherboard, group-B) thermal sensor.

    First-order lag toward the air-gap input, 1C quantization (Table 1),
    additive noise, and a slow random-walk room-temperature drift.
    """

    def __init__(self, room_temp_c: float = 22.0, lag_min: float = 2.0,
                 quant_c: float = 1.0, noise_sigma_c: float = 0.15,
                 drift_sigma_c_per_h: float = 0.10, seed: int | None = None):
        self.room = room_temp_c
        self.lag = lag_min
        self.quant = quant_c
        self.noise_sigma = noise_sigma_c
        self.drift_sigma_h = drift_sigma_c_per_h
        self.rng = random.Random(seed)
        self.elev = 0.0   # sensed elevation above room temp
        self._drift = 0.0

    def step(self, dt_min: float, gap_input_c: float) -> None:
        self.elev += (gap_input_c - self.elev) * (1.0 - math.exp(-dt_min / self.lag))
        # Room-temperature random walk (office AC; small).
        self._drift += self.rng.gauss(0.0, self.drift_sigma_h * math.sqrt(dt_min / 60.0))

    def read(self) -> float:
        """Quantized sensor reading in C, as the paper's 0.5 Hz logger saw."""
        true = self.room + self._drift + self.elev
        noisy = true + self.rng.gauss(0.0, self.noise_sigma)
        if self.quant <= 0:
            return noisy
        return round(noisy / self.quant) * self.quant


# ---------------------------------------------------------------------------
# Full channel: TX -> air gap (delay + attenuation) -> RX
# ---------------------------------------------------------------------------

class ThermalChannel:
    """One direction of the half-duplex BitWhisper link (Sec V.B)."""

    def __init__(self, distance_cm: float, layout: str = "parallel",
                 room_temp_c: float = 22.0, seed: int | None = None,
                 tx: Transmitter | None = None, rx: Receiver | None = None):
        if layout not in LAYOUTS:
            raise ValueError(f"unknown layout {layout!r}; choose from {sorted(LAYOUTS)}")
        self.params = LAYOUTS[layout]
        self.distance_cm = distance_cm
        self.tx = tx or Transmitter()
        self.rx = rx or Receiver(room_temp_c=room_temp_c,
                                lag_min=self.params.rx_lag_min, seed=seed)
        self.t = 0.0  # sim time, minutes
        # Air gap: advective dead-time (delay line) + local mixing lag.
        self._gap = 0.0                       # mixing-lag state (TX units)
        self._hist: deque[tuple[float, float]] = deque([(0.0, 0.0)])

    # -- channel transfer function --------------------------------------
    def gap_dead_min(self) -> float:
        """Advective dead-time of the thermal plume (V.B.2: 0.35 min/cm)."""
        p = self.params
        return p.dead_base_min + p.dead_per_cm_min * self.distance_cm

    def gap_lag_min(self) -> float:
        """Local mixing time constant at the receiver."""
        p = self.params
        return p.lag_base_min + p.lag_per_cm_min * self.distance_cm

    def coupling(self) -> float:
        return (_COUPLING_0 * self.params.atten_scale
                * math.exp(-self.distance_cm / _ATTEN_LENGTH_CM))

    # -- simulation ------------------------------------------------------
    def step(self, dt_min: float, tx_load: float) -> None:
        """Advance the whole channel by dt_min with TX CPU load in [0,1]."""
        e_tx = self.tx.step(dt_min, tx_load)
        self.t += dt_min
        self._hist.append((self.t, e_tx))
        horizon = self.gap_dead_min() + 60.0
        while len(self._hist) > 2 and self._hist[1][0] < self.t - horizon:
            self._hist.popleft()
        # Dead-time lookup (linear interpolation on the TX history).
        src_t = self.t - self.gap_dead_min()
        times = [h[0] for h in self._hist]
        i = bisect_left(times, src_t)
        if i <= 0:
            e_delayed = self._hist[0][1]
        elif i >= len(self._hist):
            e_delayed = self._hist[-1][1]
        else:
            t0, e0 = self._hist[i - 1]
            t1, e1 = self._hist[i]
            frac = (src_t - t0) / max(1e-9, t1 - t0)
            e_delayed = e0 + frac * (e1 - e0)
        # Local mixing lag, then attenuated coupling into the RX sensor.
        tau = self.gap_lag_min()
        self._gap += (e_delayed - self._gap) * (1.0 - math.exp(-dt_min / tau))
        self.rx.step(dt_min, self.coupling() * self._gap)

    def read_sensor(self) -> float:
        return self.rx.read()

    # -- link-budget helpers ---------------------------------------------
    def steady_rx_elev_c(self, tx_load: float = 1.0) -> float:
        """Expected RX elevation if TX held `tx_load` forever (C)."""
        return self.coupling() * _TX_EXHAUST_MAX_C * tx_load

    def detectable(self, thresh_c: float = 0.75) -> bool:
        """Can a full-blast TX move the RX sensor by a measurable amount?"""
        return self.steady_rx_elev_c() >= thresh_c
