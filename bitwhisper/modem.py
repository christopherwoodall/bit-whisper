"""Manchester modem for the BitWhisper thermal channel.

NOTE: Section VI (communication protocol) is omitted from the public draft
of Guri et al. (2015), so the modulation below is OUR design, dimensioned
from the paper's measured channel constraints.

Why Manchester: the TX slow (chassis) path has tau ~8-18 min, so plain OOK
at 7.5-min symbols suffers crushing inter-symbol interference -- a '0'
after a '1' still reads hot. Manchester encodes each bit as two half-slots,
1 -> [heat, idle] and 0 -> [idle, heat]:

  * constant 50% duty cycle -> the slow path settles to a flat baseline
    regardless of data (zero ISI by construction);
  * the bit decision compares the two half-slots *within* one bit period,
    so baseline wander and room drift cancel out -- no adaptive threshold;
  * bit rate stays 8 bit/hour (7.5-min bit period), the paper's rate.

  * Frame = 16-bit alternating preamble (acquisition) + 8-bit length +
    payload + CRC-8, all Manchester-encoded. A 2*Ts idle guard precedes
    every frame so the receiver can settle a baseline.
"""

from __future__ import annotations

import bisect
import math
import statistics

from .physics import ThermalChannel, Transmitter, Receiver

DEFAULT_TS_MIN = 7.5      # bit period -> 8 bit/hour (paper Sec I.B)
PREAMBLE = [1, 0] * 8     # 16 alternating bits
GUARD_SLOTS = 2
_SAMPLE_HZ = 0.5          # paper's logger rate (Sec IV.B.3)


class FrameError(Exception):
    """Raised when no valid frame can be recovered from the samples."""


# ---------------------------------------------------------------------------
# CRC-8 (poly 0x07)
# ---------------------------------------------------------------------------

def crc8(data: bytes, poly: int = 0x07, init: int = 0x00) -> int:
    crc = init
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = ((crc << 1) ^ poly) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


# ---------------------------------------------------------------------------
# Bit / byte helpers (MSB first)
# ---------------------------------------------------------------------------

def bytes_to_bits(data: bytes) -> list[int]:
    bits: list[int] = []
    for byte in data:
        bits.extend((byte >> i) & 1 for i in range(7, -1, -1))
    return bits


def bits_to_bytes(bits: list[int]) -> bytes:
    if len(bits) % 8:
        raise ValueError("bit count not a multiple of 8")
    out = bytearray()
    for i in range(0, len(bits), 8):
        val = 0
        for b in bits[i:i + 8]:
            val = (val << 1) | b
        out.append(val)
    return bytes(out)


# ---------------------------------------------------------------------------
# Framing
# ---------------------------------------------------------------------------

def encode_frame(payload: bytes) -> list[int]:
    """payload -> framed bit stream (preamble | len | payload | crc8)."""
    if len(payload) > 255:
        raise ValueError("payload too long (max 255 bytes)")
    body = bytes([len(payload)]) + payload
    body += bytes([crc8(body)])
    return PREAMBLE + bytes_to_bits(body)


def decode_frame_bits(bits: list[int]) -> bytes:
    """Framed bits -> payload; validates preamble/length/CRC."""
    if len(bits) < 16 + 16:
        raise FrameError("frame too short")
    if bits[:16] != PREAMBLE:
        raise FrameError("preamble mismatch")
    body_bits = bits[16:]
    if len(body_bits) < 16 or len(body_bits) % 8:
        raise FrameError("bad body length")
    body = bits_to_bytes(body_bits)
    plen = body[0]
    if len(body) != 1 + plen + 1:
        raise FrameError(f"length field {plen} != actual {len(body) - 2}")
    if crc8(body[:-1]) != body[-1]:
        raise FrameError("CRC-8 mismatch")
    return body[1:-1]


# ---------------------------------------------------------------------------
# Modulation: bits -> Manchester heater schedule
# ---------------------------------------------------------------------------

def modulate(bits: list[int], ts_min: float = DEFAULT_TS_MIN,
             guard_slots: int = GUARD_SLOTS) -> list[tuple[float, float, float]]:
    """Return [(t_start, t_end, load)] heater schedule in minutes.

    Manchester: bit 1 -> [ON, OFF], bit 0 -> [OFF, ON] over two half-slots.
    """
    half = ts_min / 2.0
    sched: list[tuple[float, float, float]] = []
    t = 0.0
    for _ in range(guard_slots * 2):      # idle guard for RX baseline
        sched.append((t, t + half, 0.0))
        t += half
    for bit in bits:
        first, second = (1.0, 0.0) if bit else (0.0, 1.0)
        sched.append((t, t + half, first))
        t += half
        sched.append((t, t + half, second))
        t += half
    sched.append((t, t + half, 0.0))     # trailing guard
    sched.append((t + half, t + 2 * half, 0.0))
    return sched


def load_at(sched: list[tuple[float, float, float]], t: float) -> float:
    for t0, t1, load in sched:
        if t0 <= t < t1:
            return load
    return 0.0


# ---------------------------------------------------------------------------
# Demodulation: sensor samples -> payload
# ---------------------------------------------------------------------------

def _interp(samples: list[tuple[float, float]], t: float) -> float:
    ts = [s[0] for s in samples]
    i = bisect.bisect_left(ts, t)
    if i <= 0:
        return samples[0][1]
    if i >= len(samples):
        return samples[-1][1]
    t0, v0 = samples[i - 1]
    t1, v1 = samples[i]
    frac = (t - t0) / max(1e-9, t1 - t0)
    return v0 + frac * (v1 - v0)


def _detrend(vals: list[float]) -> list[float]:
    """Subtract least-squares linear fit (kills DC level and slow ramp)."""
    n = len(vals)
    if n < 2:
        return list(vals)
    sx = n * (n - 1) / 2.0
    sxx = n * (n - 1) * (2 * n - 1) / 6.0
    sy = sum(vals)
    sxy = sum(i * v for i, v in enumerate(vals))
    denom = n * sxx - sx * sx
    slope = (n * sxy - sx * sy) / denom if denom else 0.0
    intercept = (sy - slope * sx) / n
    return [v - (slope * i + intercept) for i, v in enumerate(vals)]


def _quiet_channel(distance_cm: float, layout: str) -> ThermalChannel:
    """Noiseless, unquantized channel for generating the matched template."""
    ch = ThermalChannel(distance_cm, layout=layout)
    ch.rx.noise_sigma = 0.0
    ch.rx.drift_sigma_h = 0.0
    ch.rx.quant = 0.0
    return ch


def _preamble_template(distance_cm: float, layout: str,
                       ts_min: float) -> list[tuple[float, float]]:
    """Expected detrended (t, signal) shape of the Manchester preamble,
    from the physics model. Returned relative to preamble start (t=0)."""
    ch = _quiet_channel(distance_cm, layout)
    sched = modulate(PREAMBLE, ts_min)
    t_end = sched[-1][1]
    dt = 2.0 / 60.0
    t = 0.0
    series: list[tuple[float, float]] = []
    while t <= t_end:
        ch.step(dt, load_at(sched, t))
        series.append((t, ch.read_sensor()))
        t += dt
    pre_start = GUARD_SLOTS * ts_min
    base = _interp(series, pre_start)
    step = 10.0 / 60.0
    tmpl = [(tt, _interp(series, pre_start + tt) - base)
            for tt in _frange(0.0, len(PREAMBLE) * ts_min, step)]
    d = _detrend([v for _, v in tmpl])
    return [(t, v) for (t, _), v in zip(tmpl, d)]


def _frange(a: float, b: float, step: float):
    v = a
    while v <= b:
        yield v
        v += step


def demodulate(samples: list[tuple[float, float]], ts_min: float = DEFAULT_TS_MIN,
               distance_cm: float = 0.0, layout: str = "parallel",
               max_payload: int = 255) -> bytes:
    """Recover the payload from (t_min, temp_C) sensor samples.

    Raises FrameError if no preamble is found or validation fails.
    """
    if len(samples) < 100:
        raise FrameError("not enough samples")
    samples = sorted(samples)
    ts = ts_min
    half = ts / 2.0
    t_max = samples[-1][0]

    # -- 1. rough baseline from the idle guard ---------------------------
    guard_vals = [v for t, v in samples if t < 1.5 * ts]
    if not guard_vals:
        raise FrameError("no guard samples")
    baseline0 = statistics.median(guard_vals)
    sig = [(t, v - baseline0) for t, v in samples]

    # -- 2. preamble acquisition -------------------------------------------
    # Stage A (coarse): ripple-onset energy detector. Slide a 2*Ts window
    # and watch its variance jump from the guard's noise floor.
    grid_step = 10.0 / 60.0
    grid_n = int(min(6 * ts, t_max) / grid_step)
    grid = [(i * grid_step, _interp(sig, i * grid_step))
            for i in range(grid_n + 1)]
    win_n = max(2, int(2 * ts / grid_step))
    estep = max(1, win_n // 8)

    def window_var(s: int) -> float:
        seg = [v for _, v in grid[s:s + win_n]]
        m = sum(seg) / len(seg)
        return sum((v - m) ** 2 for v in seg) / len(seg)

    idx = list(range(0, len(grid) - win_n, estep))
    energies = [window_var(i) for i in idx]
    e_times = [i * grid_step for i in idx]
    if not energies:
        raise FrameError("sample window too short for acquisition")
    noise_floor = statistics.median(energies[:max(1, len(energies) // 6)])
    e_max = max(energies)
    if e_max < 9.0 * max(noise_floor, 1e-9):
        raise FrameError("no preamble found (link may be out of range)")
    t_cross = next(t for t, e in zip(e_times, energies) if e > 0.5 * e_max)
    t_coarse = t_cross + ts

    # Stage B (fine): correlate the detrended template within ±Ts.
    tmpl = _preamble_template(distance_cm, layout, ts)
    tt = [p[0] for p in tmpl]
    tv = [p[1] for p in tmpl]
    best_c, best_s = t_coarse, -1e18
    c = max(0.0, t_coarse - ts)
    while c <= t_coarse + ts and c + tt[-1] <= t_max:
        win = _detrend([_interp(sig, c + x) for x in tt])
        s = sum(w * y for w, y in zip(win, tv))
        # Signed max: template opens with a rising edge, so true alignment
        # is the positive peak (no abs(): that admits a half-period error).
        if s > best_s:
            best_s, best_c = s, c
        c += grid_step
    t0 = best_c  # TX-time preamble start (template includes channel lag)

    # -- 3. lag calibration --------------------------------------------------
    # Received symbols lag the TX schedule; slide the grid by lag in [0, Ts)
    # to maximize the preamble's Manchester separation.
    margin = min(0.75, 0.15 * half)

    def half_mean(t0_: float, h: int) -> float:
        a, b = t0_ + h * half + margin, t0_ + (h + 1) * half - margin
        vals = [v for t, v in sig if a <= t <= b]
        if not vals:
            raise FrameError(f"no samples in half-slot {h}")
        return sum(vals) / len(vals)

    def manchester_score(t0_: float, bit_idx: int) -> float:
        d = half_mean(t0_, 2 * bit_idx) - half_mean(t0_, 2 * bit_idx + 1)
        return d if PREAMBLE[bit_idx] else -d

    def preamble_sep(t0_: float) -> float:
        return sum(manchester_score(t0_, i) for i in range(16)) / 16.0

    best_lag, best_sep = 0.0, -1e18
    lag = 0.0
    while lag < ts:
        sep = preamble_sep(t0 + lag)
        if sep > best_sep:
            best_sep, best_lag = sep, lag
        lag += ts / 8
    t0 += best_lag
    if best_sep < 0.05:
        raise FrameError("preamble levels indistinguishable")

    # -- 4. Manchester bit decisions ----------------------------------------
    # bit = 1 iff first half-slot is hotter than the second. The comparison
    # is local to the bit period: baseline wander and slow ISI cancel out.
    def decide(i: int) -> int:
        return 1 if half_mean(t0, 2 * i) > half_mean(t0, 2 * i + 1) else 0

    pre_bits = [decide(i) for i in range(16)]
    if sum(a != b for a, b in zip(pre_bits, PREAMBLE)) > 2:
        raise FrameError("preamble decode failed")
    bits: list[int] = list(PREAMBLE)  # preamble carries no information

    for i in range(16, 24):  # length byte
        if t0 + (2 * i + 2) * half > t_max:
            raise FrameError("samples end mid-frame")
        bits.append(decide(i))
    plen = int.from_bytes(bits_to_bytes(bits[16:24]), "big")
    if plen > max_payload:
        raise FrameError(f"absurd length {plen}")
    total_bits = 16 + 8 * (1 + plen + 1)
    for i in range(24, total_bits):
        if t0 + (2 * i + 2) * half > t_max:
            raise FrameError("samples end mid-frame")
        bits.append(decide(i))

    return decode_frame_bits(bits)
