"""Real-hardware interfaces for BitWhisper.

Transmitter side (paper Sec IV.B.3): heat is generated with CPU-intensive
busy loops (the paper used prime95/FurMark); here a SIGSTOP/SIGCONT
pulse-width burner gives precise duty-cycle control without extra tools.

Receiver side: reads Linux thermal zones (hwmon/sysfs). The paper's best
receiving sensor is the motherboard *ambient* sensor (group B) -- on Linux
this is usually an `acpitz` or `nct6775`/`it87` zone; CPU core zones
(`coretemp`/`k10temp`, group A) are too jumpy for reception.

This VM exposes no thermal zones, so hw.py is validated against the
simulator; on bare metal the same Modem code can drive these directly.
"""

from __future__ import annotations

import multiprocessing as mp
import os
import signal
import time


# ---------------------------------------------------------------------------
# Receiver: thermal sensor readout
# ---------------------------------------------------------------------------

def list_sensors() -> dict[str, float | None]:
    """Return {zone_name: temp_C or None} for all Linux thermal zones."""
    out: dict[str, float | None] = {}
    base = "/sys/class/thermal"
    if not os.path.isdir(base):
        return out
    for zone in sorted(os.listdir(base)):
        if not zone.startswith("thermal_zone"):
            continue
        try:
            with open(os.path.join(base, zone, "type")) as f:
                name = f.read().strip()
            with open(os.path.join(base, zone, "temp")) as f:
                temp_c = int(f.read().strip()) / 1000.0
        except (OSError, ValueError):
            continue
        key = f"{name} ({zone})"
        # disambiguate duplicate type names
        i = 2
        while key in out:
            key = f"{name} ({zone}#{i})"
            i += 1
        out[key] = temp_c
    return out


def pick_ambient_sensor(sensors: dict[str, float | None]) -> str | None:
    """Heuristic: prefer motherboard/ambient zones over CPU core zones."""
    names = list(sensors)
    for prefer in ("acpitz", "nct", "it87", "k8temp", "atk0110"):
        for n in names:
            if prefer in n.lower():
                return n
    for avoid in ("coretemp", "k10temp", "zenpower"):
        pass
    return names[0] if names else None


class SensorReader:
    """Poll a thermal zone at a fixed rate (paper: 0.5 Hz via HWiNFO)."""

    def __init__(self, zone: str | None = None, rate_hz: float = 0.5):
        sensors = list_sensors()
        self.zone = zone or pick_ambient_sensor(sensors)
        if self.zone is None:
            raise RuntimeError("no thermal zones found (see list_sensors())")
        self.rate_hz = rate_hz

    def read(self) -> float | None:
        return list_sensors().get(self.zone)

    def record(self, minutes: float) -> list[tuple[float, float]]:
        """Return [(t_min, temp_C)] for `minutes` of wall-clock sampling."""
        out: list[tuple[float, float]] = []
        t0 = time.monotonic()
        period = 1.0 / self.rate_hz
        while (time.monotonic() - t0) < minutes * 60:
            out.append(((time.monotonic() - t0) / 60.0, self.read()))
            time.sleep(period)
        return out


# ---------------------------------------------------------------------------
# Transmitter: CPU busy-loop burner with duty-cycle control
# ---------------------------------------------------------------------------

def _burn_loop(stop: mp.Event):
    x = 0.0
    while not stop.is_set():
        # Integer-free busy work: hard for the compiler/CPU to optimize out,
        # maximal ALU/FPU switching activity -> maximal Joule heating.
        for _ in range(20000):
            x = (x * 1.000001 + 0.123456) % 1.0
    # sink the accumulator so the loop cannot be eliminated
    with open(os.devnull, "w") as dn:
        dn.write(str(x))


class CPUBurner:
    """Modulate CPU load 0..1 across `nprocs` busy-loop workers.

    Uses SIGSTOP/SIGCONT pulse-width modulation: workers run flat-out while
    un-stopped, giving precise average utilization without scheduler games.
    """

    def __init__(self, nprocs: int | None = None):
        self.nprocs = nprocs or os.cpu_count() or 1
        self._stop = mp.Event()
        self._procs: list[mp.Process] = []

    def start(self):
        if self._procs:
            return
        self._stop.clear()
        self._procs = [mp.Process(target=_burn_loop, args=(self._stop,),
                                 daemon=True) for _ in range(self.nprocs)]
        for p in self._procs:
            p.start()

    def stop(self):
        self._stop.set()
        for p in self._procs:
            p.join(timeout=5)
            if p.is_alive():
                p.terminate()
        self._procs = []

    def emit(self, load: float, minutes: float):
        """Hold average CPU `load` in [0,1] for `minutes` (wall clock)."""
        load = min(1.0, max(0.0, load))
        self.start()
        period, deadline = 1.0, time.monotonic() + minutes * 60.0
        try:
            while time.monotonic() < deadline:
                on = period * load
                for p in self._procs:          # run
                    try:
                        os.kill(p.pid, signal.SIGCONT)
                    except ProcessLookupError:
                        pass
                time.sleep(on)
                for p in self._procs:          # freeze
                    try:
                        os.kill(p.pid, signal.SIGSTOP)
                    except ProcessLookupError:
                        pass
                time.sleep(max(0.0, period - on))
        finally:
            for p in self._procs:
                try:
                    os.kill(p.pid, signal.SIGCONT)
                except ProcessLookupError:
                    pass

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, *exc):
        self.stop()


def self_heating_test(minutes: float = 2.0, load: float = 1.0,
                      zone: str | None = None) -> dict:
    """Sanity check: burn CPU and report local sensor movement.

    (On the paper's hardware this is the TX characterization of Sec V.A;
    in a VM the numbers are meaningless but the plumbing is exercised.)
    """
    reader = SensorReader(zone=zone)
    before = reader.read()
    with CPUBurner() as burner:
        burner.emit(load, minutes)
    time.sleep(2)
    after = reader.read()
    return {"sensor": reader.zone, "before_c": before, "after_c": after,
            "delta_c": (after - before) if None not in (after, before) else None}
