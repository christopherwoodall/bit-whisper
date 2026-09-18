"""End-to-end tests: bits -> heat -> air gap -> sensor -> bits."""

from bitwhisper.modem import (DEFAULT_TS_MIN, FrameError, demodulate,
                              encode_frame, modulate)
from bitwhisper.physics import ThermalChannel
from bitwhisper.protocol import HalfDuplexLink, run_schedule, thermal_ping


def _transfer(payload: bytes, distance_cm: float, layout: str = "parallel",
              seed: int = 7, ts: float = DEFAULT_TS_MIN):
    ch = ThermalChannel(distance_cm, layout=layout, seed=seed)
    sched = modulate(encode_frame(payload), ts)
    samples = run_schedule(ch, sched)
    rx = demodulate(samples, ts, distance_cm, layout)
    return rx, samples, ch


def test_e2e_zero_cm():
    rx, samples, ch = _transfer(b"hi", 0.0)
    assert rx == b"hi", rx
    air_h = samples[-1][0] / 60.0
    print(f"  (48 bits over {air_h:.1f} sim-hours)")


def test_e2e_10cm_with_noise_and_drift():
    # Realistic sensor: quantization + noise + room drift all enabled.
    rx, _, _ = _transfer(b"ok", 10.0, seed=42)
    assert rx == b"ok", rx


def test_e2e_out_of_range_no_frame():
    ch = ThermalChannel(50.0, layout="parallel", seed=9)
    sched = modulate(encode_frame(b"hi"), DEFAULT_TS_MIN)
    samples = run_schedule(ch, sched)
    try:
        demodulate(samples, DEFAULT_TS_MIN, 50.0, "parallel")
    except FrameError:
        return
    raise AssertionError("expected FrameError at 50 cm")


def test_thermal_ping_detects_neighbor():
    ch = ThermalChannel(5.0, layout="parallel", seed=11)
    res = thermal_ping(ch)
    assert res["present"], res
    assert res["delay_min"] is not None and res["delay_min"] < 30, res
    print(f"  (ping: delay={res['delay_min']:.1f} min, rise={res['rise_c']:.1f}C)")


def test_thermal_ping_no_neighbor_far():
    ch = ThermalChannel(60.0, layout="parallel", seed=12)
    res = thermal_ping(ch)
    assert not res["present"], res


def test_halfduplex_transfer_with_ack():
    link = HalfDuplexLink(
        ThermalChannel(5.0, layout="parallel", seed=21),
        ThermalChannel(5.0, layout="parallel", seed=22),
    )
    rep = link.transfer(b"ACK-test", retries=1)
    assert rep["ok"], rep
    assert rep["rx_payload"] == b"ACK-test"
    assert rep["ack"] == b"ACK"
    print(f"  (air time {rep['air_time_min']/60:.1f} h, "
          f"{rep['tx_bits']/ (rep['air_time_min']/60):.1f} bit/h)")


if __name__ == "__main__":
    import time
    import traceback
    fns = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    for fn in fns:
        t0 = time.time()
        try:
            fn()
            print(f"PASS {fn.__name__} ({time.time()-t0:.1f}s)")
        except Exception:
            print(f"FAIL {fn.__name__}")
            traceback.print_exc()
            raise SystemExit(1)
