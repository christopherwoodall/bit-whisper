"""Link protocol for the BitWhisper thermal channel.

Half-duplex (paper Sec I.B): each side takes turns as transmitter/receiver.

  * thermal_ping -- neighbor discovery (paper Fig 1, Sec II): the transmitter
    emits a long heat pulse; a nearby receiver that sees a sustained rise
    answers with its own pulse. Also sounds the channel (delay/attenuation).
  * HalfDuplexLink -- framed message transfer with ACK/retry.

Timing is in sim-minutes; drive with ThermalChannel.step().
"""

from __future__ import annotations

import statistics

from .modem import (DEFAULT_TS_MIN, FrameError, decode_frame_bits,
                    demodulate, encode_frame, load_at, modulate)
from .physics import ThermalChannel

SAMPLE_DT_MIN = 2.0 / 60.0   # 0.5 Hz sampling, as in the paper


def run_schedule(ch: ThermalChannel, sched, sample: bool = True):
    """Drive `sched` [(t0,t1,load)] on channel `ch`; return (t,temp) samples."""
    t_end = sched[-1][1]
    t, nxt = 0.0, 0.0
    samples: list[tuple[float, float]] = []
    while t <= t_end:
        ch.step(SAMPLE_DT_MIN, load_at(sched, t))
        if sample and t >= nxt:
            samples.append((t, ch.read_sensor()))
            nxt += 1.0 / 30.0            # 0.5 Hz
        t += SAMPLE_DT_MIN
    return samples


def _idle(ch: ThermalChannel, minutes: float):
    t = 0.0
    while t < minutes:
        ch.step(SAMPLE_DT_MIN, 0.0)
        t += SAMPLE_DT_MIN


def thermal_ping(ch: ThermalChannel, ts_min: float = DEFAULT_TS_MIN,
                 pulse_slots: int = 6, thresh_c: float = 1.5) -> dict:
    """Emit a heat pulse and listen for the thermal echo of a neighbor.

    In the real attack (Sec II) the *receiver* answers a ping with its own
    pulse; here we sound one direction: TX pulses, we watch the RX sensor.

    The rise is measured from the median of the hottest samples (not a
    single peak) so sensor noise can't fake a neighbor, and the 1.5C
    threshold needs ~2 sensor quantization steps -- drift plus rounding
    alone cannot cross it at out-of-range distances.

    Returns dict(present, delay_min, rise_c).
    """
    base = [ch.read_sensor() for _ in range(5)]
    base_med = statistics.median(base)
    pulse_min = pulse_slots * ts_min
    sched = [(0.0, pulse_min, 1.0), (pulse_min, pulse_min + 2 * ts_min, 0.0)]
    samples = run_schedule(ch, sched)
    hottest = sorted(v for _, v in samples)[-5:]
    rise = statistics.median(hottest) - base_med
    present = rise >= thresh_c
    delay = None
    if present:
        # first sustained crossing of half the rise
        for t, v in samples:
            if v - base_med >= 0.5 * rise:
                delay = t
                break
    return {"present": present, "delay_min": delay, "rise_c": rise}


class HalfDuplexLink:
    """Bidirectional link over two ThermalChannels (a->b, b->a).

    The paper's channel is asymmetric (V.B.3); each direction may have its
    own distance/layout parameters.
    """

    def __init__(self, ch_ab: ThermalChannel, ch_ba: ThermalChannel,
                 ts_min: float = DEFAULT_TS_MIN):
        self.ch_ab = ch_ab
        self.ch_ba = ch_ba
        self.ts = ts_min

    def _send_one_way(self, ch: ThermalChannel, payload: bytes):
        bits = encode_frame(payload)
        sched = modulate(bits, self.ts)
        return run_schedule(ch, sched), bits

    def _recv_one_way(self, ch: ThermalChannel, samples) -> bytes:
        return demodulate(samples, self.ts, ch.distance_cm,
                          ch.params.name)

    def ping(self) -> dict:
        """Sound the a->b direction (thermal ping)."""
        return thermal_ping(self.ch_ab, self.ts)

    def transfer(self, payload: bytes, retries: int = 2) -> dict:
        """Send payload a->b with ACK; returns transfer report."""
        attempt = 0
        while True:
            attempt += 1
            samples, tx_bits = self._send_one_way(self.ch_ab, payload)
            try:
                rx_payload = self._recv_one_way(self.ch_ab, samples)
            except FrameError as e:
                rx_payload = None
                rx_err = str(e)
            else:
                rx_err = None
            # b->a ACK
            ack = b"ACK" if rx_payload == payload else b"NACK"
            ack_samples, _ = self._send_one_way(self.ch_ba, ack)
            try:
                ack_rx = self._recv_one_way(self.ch_ba, ack_samples)
            except FrameError:
                ack_rx = None
            ok = rx_payload == payload and ack_rx == b"ACK"
            if ok or attempt > retries:
                ber = None
                if rx_payload is not None:
                    rx_bits = None  # bit-level BER needs raw bits; byte compare:
                    ber = sum(a != b for a, b in
                              zip(payload, rx_payload)) / max(1, len(payload))
                return {
                    "ok": ok,
                    "attempts": attempt,
                    "tx_bits": len(tx_bits),
                    "rx_payload": rx_payload,
                    "rx_error": rx_err,
                    "ack": ack_rx,
                    "byte_error_rate": ber,
                    "air_time_min": samples[-1][0] if samples else 0.0,
                }
