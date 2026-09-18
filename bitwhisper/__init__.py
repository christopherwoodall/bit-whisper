"""BitWhisper: covert thermal signaling between air-gapped computers.

Implements the channel described in Guri et al., "BitWhisper: Covert
Signaling Channel between Air-Gapped Computers using Thermal Manipulations"
(arXiv:1503.07919, 2015).

Paper facts encoded here (all time constants in minutes unless noted):
  * TX heats by CPU workload; RX senses with motherboard ambient sensors
    (sensor group B) -- accurate vs. environment, immune to local CPU spikes.
  * ~1.5-3 min of full load for the ambient sensor to move 1C (Sec V.A.1).
  * Parallel layout, ~0 cm: first +1C after ~3 min; +4C after ~26 min (V.B.2).
  * Propagation delay grows ~0.35 min per cm of separation (V.B.2).
  * 30-35 cm: at most +1C; beyond ~40 cm nothing is sensed (V.B.2).
  * Heating is faster near idle temp, cooling faster near max temp (fan
    curves) -- modelled as asymmetric heating/cooling time constants.
  * Sensor quantization: 1C; monitoring sample rate in paper: 0.5 Hz.

Section VI (modulation/protocol) is omitted from the public draft, so
modem.py documents our own design built on the measured constraints above.
"""

from .physics import (  # noqa: F401  (re-exported for convenience)
    LayoutParams, LAYOUTS, ThermalChannel, Transmitter, Receiver,
)
from .modem import (  # noqa: F401
    encode_frame, decode_frame_bits, modulate, demodulate, FrameError,
    DEFAULT_TS_MIN, crc8,
)
from .protocol import HalfDuplexLink, thermal_ping  # noqa: F401

__version__ = "0.1.0"
