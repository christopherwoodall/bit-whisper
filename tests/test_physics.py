"""Physics tests: model behavior vs. the paper's measured numbers."""

from bitwhisper.physics import ThermalChannel, Transmitter, Receiver


def _drive(ch, minutes, load, dt=2.0 / 60.0):
    t = 0.0
    while t < minutes:
        ch.step(dt, load)
        t += dt


def test_tx_heats_toward_max():
    tx = Transmitter()
    for _ in range(int(40 / (2 / 60))):
        tx.step(2.0 / 60.0, 1.0)
    assert 12.0 < tx.elev <= 15.0, tx.elev


def test_tx_cools_slower_than_it_heats():
    # The slow (chassis) path cools slower than it heats (paper V.A.1);
    # the fast (exhaust) path reacts within ~1 min both ways.
    tx = Transmitter()
    dt = 2.0 / 60.0
    for _ in range(int(10 / dt)):
        tx.step(dt, 1.0)
    slow_hot, fast_hot = tx.elev_slow, tx.elev_fast
    for _ in range(int(10 / dt)):
        tx.step(dt, 0.0)
    assert tx.elev_slow > 0.5 * slow_hot      # slow path retains heat
    assert tx.elev_fast < 0.05 * fast_hot    # fast path sheds it quickly
    assert slow_hot > fast_hot * 0.3         # both paths carry real energy


def test_zero_cm_parallel_matches_paper():
    # V.B.2: ~0 cm -> first +1C after ~3 min, +4C after ~26 min.
    ch = ThermalChannel(0.0, layout="parallel", seed=1)
    ch.rx.noise_sigma = 0.0
    ch.rx.drift_sigma_h = 0.0
    _drive(ch, 3.0, 1.0)
    assert ch.rx.elev >= 0.5, ch.rx.elev          # moving by minute 3
    _drive(ch, 23.0, 1.0)                          # total 26 min
    assert 3.0 <= ch.rx.elev <= 5.0, ch.rx.elev   # ~+4C


def test_attenuation_with_distance():
    # V.B.2: at 30-35 cm at most +1C; beyond 40 cm nothing sensed.
    near = ThermalChannel(35.0, layout="parallel", seed=2)
    near.rx.noise_sigma = 0.0
    near.rx.drift_sigma_h = 0.0
    _drive(near, 120.0, 1.0)
    assert 0.5 <= near.rx.elev <= 1.5, near.rx.elev

    far = ThermalChannel(45.0, layout="parallel", seed=3)
    assert not far.detectable()
    far.rx.noise_sigma = 0.0
    far.rx.drift_sigma_h = 0.0
    _drive(far, 180.0, 1.0)
    assert far.rx.elev < 0.75, far.rx.elev


def test_delay_grows_with_distance():
    a = ThermalChannel(0.0, layout="parallel")
    b = ThermalChannel(20.0, layout="parallel")
    assert b.gap_dead_min() + b.gap_lag_min() > a.gap_dead_min() + a.gap_lag_min()
    slope = ((b.gap_dead_min() + b.gap_lag_min())
             - (a.gap_dead_min() + a.gap_lag_min())) / 20.0
    assert abs(slope - 0.35) < 1e-9


def test_sensor_quantization():
    rx = Receiver(seed=4)
    rx.elev = 2.3
    for _ in range(20):
        v = rx.read()
        assert v == round(v), v  # whole degrees only


def test_layout_presets_differ():
    q = ThermalChannel(6.0, layout="quadrature")
    f = ThermalChannel(6.0, layout="face-away")
    assert q.gap_dead_min() + q.gap_lag_min() < f.gap_dead_min() + f.gap_lag_min()
    assert q.coupling() > f.coupling()


if __name__ == "__main__":
    for name, fn in sorted(list(globals().items())):
        if name.startswith("test_"):
            fn()
            print(f"PASS {name}")
