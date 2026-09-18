"""Modem unit tests: framing, CRC, modulation schedule (no channel)."""

try:
    import pytest
except ImportError:  # minimal fallback when pytest is unavailable
    class _Raises:
        def __init__(self, exc):
            self.exc = exc

        def __enter__(self):
            return self

        def __exit__(self, exc_type, exc, tb):
            assert exc_type is not None and issubclass(exc_type, self.exc), \
                f"expected {self.exc.__name__}, got {exc_type}"
            return True

    class pytest:  # type: ignore
        @staticmethod
        def raises(exc):
            return _Raises(exc)

from bitwhisper.modem import (
    FrameError, bits_to_bytes, bytes_to_bits, crc8,
    decode_frame_bits, encode_frame, load_at, modulate,
)


def test_crc8_vector():
    assert crc8(b"123456789") == 0xF4  # standard CRC-8 check value


def test_bits_roundtrip():
    data = bytes(range(256))
    assert bits_to_bytes(bytes_to_bits(data)) == data


def test_frame_roundtrip():
    payload = b"p@ssw0rd!"
    bits = encode_frame(payload)
    assert bits[:16] == [1, 0] * 8
    assert decode_frame_bits(bits) == payload


def test_frame_rejects_bad_crc():
    bits = encode_frame(b"abc")
    bits[-1] ^= 1
    with pytest.raises(FrameError):
        decode_frame_bits(bits)


def test_frame_rejects_bad_preamble():
    bits = encode_frame(b"abc")
    bits[0] ^= 1
    with pytest.raises(FrameError):
        decode_frame_bits(bits)


def test_modulate_schedule():
    bits = [1, 0, 1]
    sched = modulate(bits, ts_min=7.5, guard_slots=2)
    # Manchester: 2*guard half-slots + 2 per bit + 2 trailing half-slots
    assert len(sched) == 4 + 6 + 2
    assert sched[0] == (0.0, 3.75, 0.0)          # guard
    assert sched[4] == (15.0, 18.75, 1.0)        # first data bit: 1 -> [ON, OFF]
    assert sched[5] == (18.75, 22.5, 0.0)
    assert sched[6] == (22.5, 26.25, 0.0)        # second bit: 0 -> [OFF, ON]
    assert sched[7] == (26.25, 30.0, 1.0)
    assert load_at(sched, 16.0) == 1.0
    assert load_at(sched, 5.0) == 0.0
    assert load_at(sched, 999.0) == 0.0


def test_empty_payload():
    assert decode_frame_bits(encode_frame(b"")) == b""


if __name__ == "__main__":
    import traceback
    fns = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    for fn in fns:
        try:
            fn()
            print(f"PASS {fn.__name__}")
        except Exception:
            print(f"FAIL {fn.__name__}")
            traceback.print_exc()
            raise SystemExit(1)
