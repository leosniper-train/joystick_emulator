"""Betaflight Joystick Emulator (SITL + DEVICE).

Virtual RC transmitter for Betaflight:
  - SITL mode: UDP RC to port 9004, PWM feedback on port 9001
  - DEVICE mode: MSP over a WebSocket serial bridge (default ws://127.0.0.1:5761)

Controllable with mouse (drag sticks) and keyboard (WASD + arrows + AUX keys).
"""

from __future__ import annotations

import argparse
import json
import math
import os
import socket
import struct
import sys
import threading
import time
from dataclasses import asdict, dataclass, field, fields
from typing import Any, Callable, Optional
from urllib.parse import urlparse

import pygame
from pygame import gfxdraw

try:
    import websocket
except ImportError:  # pragma: no cover - installed via requirements.txt
    websocket = None  # type: ignore

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

CONFIG_FILENAME = "config.json"
NUM_PACKET_CHANNELS = 16  # Betaflight SITL always reads a fixed 16-channel struct.
NUM_AUX_CHANNELS = 12     # CH5..CH16 are user-configurable AUX channels.
AUX_TYPES = ("2pos", "3pos", "slider")
MODES = ("sitl", "device")

# MSP v1 command IDs (Betaflight)
MSP_SERVO = 103
MSP_MOTOR = 104
MSP_STATUS_EX = 150
MSP_SET_RAW_RC = 200
MSP_SET_MOTOR = 214  # disarmed motor test / override

WINDOW_W, WINDOW_H = 880, 648
FPS_RENDER_CAP = 120  # Rendering may run faster than the RC send rate.

# ---- Palette (modern dark "avionics" theme) ----
COL_BG_TOP = (16, 19, 26)
COL_BG_BOT = (10, 12, 17)
COL_PANEL = (23, 27, 35)
COL_PANEL_LIGHT = (33, 39, 50)
COL_BORDER = (42, 49, 62)
COL_BORDER_HI = (58, 68, 86)

COL_ACCENT = (56, 189, 248)      # sky blue - primary axes
COL_ACCENT_DIM = (34, 108, 145)
COL_THROTTLE = (245, 176, 66)    # amber - throttle
COL_AUX = (167, 139, 250)        # violet - aux switches
COL_MUTED = (90, 102, 120)       # unused channels

COL_TEXT = (231, 236, 244)
COL_TEXT_DIM = (140, 149, 165)
COL_TEXT_FAINT = (96, 105, 122)

COL_GIMBAL = (18, 21, 28)
COL_GIMBAL_RING = (48, 56, 72)
COL_GIMBAL_GRID = (36, 43, 55)
COL_KNOB = (56, 189, 248)
COL_KNOB_GRAB = (125, 211, 252)

COL_GOOD = (52, 211, 153)        # armed / connected (green)
COL_BAD = (248, 113, 113)        # disarmed / error (red)
COL_WARN = (245, 176, 66)
COL_TRACK = (30, 35, 45)

# Backwards-compatible aliases used elsewhere
COL_BG = COL_BG_TOP
COL_BAR_BG = COL_TRACK
COL_BAR_FILL = COL_ACCENT


# ---------------------------------------------------------------------------
# Settings
# ---------------------------------------------------------------------------


@dataclass
class Settings:
    """All user-tunable parameters. Persisted to config.json."""

    # Core
    mode: str = "sitl"             # "sitl" | "device"
    ip: str = "127.0.0.1"
    port: int = 9004
    send_hz: int = 50

    # PWM output (motor/servo) input from SITL, port 9001
    pwm_in_enabled: bool = True
    pwm_in_port: int = 9001

    # Only transmit RC while the SITL's PWM output is being received.
    require_pwm: bool = True

    # DEVICE mode: MSP over WebSocket serial bridge
    msp_ws_url: str = "ws://127.0.0.1:5761"
    msp_poll_hz: int = 20
    require_msp: bool = True

    # PWM range
    pwm_min: int = 1000
    pwm_mid: int = 1500
    pwm_max: int = 2000

    # Stick feel
    return_speed: float = 5.0  # normalized units/sec when recentering
    key_step: float = 2.5      # initial rate (normalized units/sec) when a key is pressed
    key_accel: float = 6.0     # rate acceleration while held (units/sec^2); 0 = constant
    key_step_max: float = 12.0 # max rate after acceleration (units/sec)
    deadband: float = 0.03     # ignore tiny stick offsets (0..0.4)
    expo: float = 0.0          # 0 = linear, 1 = full cubic softening

    # Throttle
    throttle_sticky: bool = True   # throttle holds its position when released
    throttle_arm_safe: int = 1000  # throttle PWM applied on reset/failsafe

    # Channels
    channel_order: str = "AETR"    # order of the 4 main axes on CH1-4
    channel_count: int = 16        # active channels (packet is always 16)
    aux_low: int = 1000            # switch "off" / slider-min PWM value
    aux_high: int = 2000           # switch "on" / slider-max PWM value
    # Per-channel control type for CH5..CH16 (12 entries): "2pos" | "3pos" | "slider".
    aux_types: list = field(default_factory=lambda: ["2pos"] * NUM_AUX_CHANNELS)

    # ------------------------------------------------------------------
    @property
    def is_device(self) -> bool:
        return self.mode == "device"

    # ------------------------------------------------------------------
    def clamp_all(self) -> None:
        """Validate/clamp values into sane ranges."""
        mode = str(self.mode).strip().lower()
        self.mode = mode if mode in MODES else "sitl"
        self.port = int(_clamp(self.port, 1, 65535))
        self.pwm_in_port = int(_clamp(self.pwm_in_port, 1, 65535))
        self.send_hz = int(_clamp(self.send_hz, 1, 250))
        self.msp_poll_hz = int(_clamp(self.msp_poll_hz, 1, 100))
        url = str(self.msp_ws_url).strip()
        if not url:
            url = "ws://127.0.0.1:5761"
        if "://" not in url:
            url = "ws://" + url
        self.msp_ws_url = url
        self.pwm_min = int(_clamp(self.pwm_min, 500, 2500))
        self.pwm_max = int(_clamp(self.pwm_max, 500, 2500))
        if self.pwm_max < self.pwm_min:
            self.pwm_min, self.pwm_max = self.pwm_max, self.pwm_min
        self.pwm_mid = int(_clamp(self.pwm_mid, self.pwm_min, self.pwm_max))
        self.return_speed = float(_clamp(self.return_speed, 0.2, 50.0))
        self.key_step = float(_clamp(self.key_step, 0.2, 50.0))
        self.key_accel = float(_clamp(self.key_accel, 0.0, 100.0))
        self.key_step_max = float(_clamp(self.key_step_max, self.key_step, 100.0))
        self.deadband = float(_clamp(self.deadband, 0.0, 0.4))
        self.expo = float(_clamp(self.expo, 0.0, 1.0))
        self.throttle_arm_safe = int(_clamp(self.throttle_arm_safe, self.pwm_min, self.pwm_max))
        order = "".join(c for c in self.channel_order.upper() if c in "AETR")
        # keep it a valid permutation of AETR
        if sorted(order) != list("AETR"):
            order = "AETR"
        self.channel_order = order
        self.channel_count = int(_clamp(self.channel_count, 4, NUM_PACKET_CHANNELS))
        self.aux_low = int(_clamp(self.aux_low, 500, 2500))
        self.aux_high = int(_clamp(self.aux_high, 500, 2500))
        raw = list(self.aux_types) if isinstance(self.aux_types, (list, tuple)) else []
        types = [t if t in AUX_TYPES else "2pos" for t in raw][:NUM_AUX_CHANNELS]
        types += ["2pos"] * (NUM_AUX_CHANNELS - len(types))
        self.aux_types = types

    # ------------------------------------------------------------------
    @classmethod
    def load(cls, path: str) -> "Settings":
        s = cls()
        if os.path.isfile(path):
            try:
                with open(path, "r", encoding="utf-8") as fh:
                    data = json.load(fh)
                valid = {f.name for f in fields(cls)}
                for k, v in data.items():
                    if k in valid:
                        setattr(s, k, v)
            except (OSError, ValueError, json.JSONDecodeError) as exc:
                print(f"[settings] could not read {path}: {exc}; using defaults")
        s.clamp_all()
        # Always (re)write the file so it contains every current setting,
        # migrating older configs that predate newly added fields.
        s.save(path)
        return s

    def save(self, path: str) -> bool:
        self.clamp_all()
        try:
            with open(path, "w", encoding="utf-8") as fh:
                json.dump(asdict(self), fh, indent=2)
            return True
        except OSError as exc:
            print(f"[settings] could not write {path}: {exc}")
            return False


def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


# ---------------------------------------------------------------------------
# RC state
# ---------------------------------------------------------------------------

AXIS_LETTER_TO_ATTR = {"A": "roll", "E": "pitch", "T": "throttle", "R": "yaw"}


@dataclass
class RCState:
    """Normalized control state. roll/pitch/yaw in [-1, 1], throttle in [0, 1]."""

    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0
    throttle: float = 0.0

    # Normalized [0, 1] positions for CH5..CH16. How a position maps to PWM
    # depends on the per-channel type in Settings.aux_types. Index 0 is CH5,
    # which conventionally arms the craft.
    aux: list = field(default_factory=lambda: [0.0] * NUM_AUX_CHANNELS)

    # Discrete positions each control type snaps to.
    _NOTCHES = {
        "2pos": (0.0, 1.0),
        "3pos": (0.0, 0.5, 1.0),
        "slider": (0.0, 0.25, 0.5, 0.75, 1.0),
    }

    @property
    def armed(self) -> bool:
        return self.aux[0] >= 0.5

    @armed.setter
    def armed(self, value: bool) -> None:
        self.aux[0] = 1.0 if value else 0.0

    # ------------------------------------------------------------------
    def reset(self, settings: Settings) -> None:
        """Failsafe: center sticks, cut throttle to the arm-safe value, disarm."""
        self.roll = self.pitch = self.yaw = 0.0
        for i in range(len(self.aux)):
            self.aux[i] = 0.0
        span = max(1, settings.pwm_max - settings.pwm_min)
        self.throttle = _clamp((settings.throttle_arm_safe - settings.pwm_min) / span, 0.0, 1.0)

    # ------------------------------------------------------------------
    def actuate(self, i: int, aux_types: list) -> None:
        """Advance an AUX channel to its next discrete notch (wraps around)."""
        if not 0 <= i < len(self.aux):
            return
        kind = aux_types[i] if i < len(aux_types) else "2pos"
        notches = self._NOTCHES.get(kind, (0.0, 1.0))
        cur = self.aux[i]
        nearest = min(range(len(notches)), key=lambda k: abs(notches[k] - cur))
        self.aux[i] = notches[(nearest + 1) % len(notches)]

    def set_pos(self, i: int, pos: float, aux_types: list) -> None:
        """Set an AUX channel from a raw [0, 1] position, quantized by its type."""
        if not 0 <= i < len(self.aux):
            return
        pos = _clamp(pos, 0.0, 1.0)
        kind = aux_types[i] if i < len(aux_types) else "2pos"
        if kind == "2pos":
            self.aux[i] = 1.0 if pos >= 0.5 else 0.0
        elif kind == "3pos":
            self.aux[i] = 0.0 if pos < 1 / 3 else (1.0 if pos > 2 / 3 else 0.5)
        else:
            self.aux[i] = pos

    # ------------------------------------------------------------------
    def to_channels(self, settings: Settings) -> list:
        """Build the list of PWM channel values for this state."""
        chans = [settings.pwm_mid] * NUM_PACKET_CHANNELS

        # Main 4 axes, placed per channel_order.
        for idx, letter in enumerate(settings.channel_order[:4]):
            attr = AXIS_LETTER_TO_ATTR.get(letter)
            if attr is None or idx >= NUM_PACKET_CHANNELS:
                continue
            if attr == "throttle":
                chans[idx] = _throttle_to_pwm(self.throttle, settings)
            else:
                val = _shape_axis(getattr(self, attr), settings)
                chans[idx] = _axis_to_pwm(val, settings)

        # AUX channels CH5..CH16, each mapped per its configured type.
        types = settings.aux_types
        for i in range(len(self.aux)):
            ch = 4 + i  # CH5 -> index 4
            if ch >= NUM_PACKET_CHANNELS:
                break
            kind = types[i] if i < len(types) else "2pos"
            chans[ch] = _aux_to_pwm(self.aux[i], kind, settings)

        # Channels beyond the active count sit at the mid value.
        for ch in range(settings.channel_count, NUM_PACKET_CHANNELS):
            chans[ch] = settings.pwm_mid

        return [int(_clamp(c, 1000, 2000)) for c in chans]


def _shape_axis(x: float, settings: Settings) -> float:
    """Apply deadband + expo to a [-1, 1] axis."""
    x = _clamp(x, -1.0, 1.0)
    db = settings.deadband
    if abs(x) <= db:
        return 0.0
    # rescale so motion resumes smoothly just outside the deadband
    x = (abs(x) - db) / (1.0 - db) * (1 if x > 0 else -1)
    e = settings.expo
    if e > 0.0:
        x = (1.0 - e) * x + e * (x ** 3)
    return _clamp(x, -1.0, 1.0)


def _axis_to_pwm(x: float, settings: Settings) -> int:
    """Map a shaped [-1, 1] axis to PWM using an (optionally) asymmetric mid."""
    if x >= 0:
        return round(settings.pwm_mid + x * (settings.pwm_max - settings.pwm_mid))
    return round(settings.pwm_mid + x * (settings.pwm_mid - settings.pwm_min))


def _throttle_to_pwm(t: float, settings: Settings) -> int:
    t = _clamp(t, 0.0, 1.0)
    return round(settings.pwm_min + t * (settings.pwm_max - settings.pwm_min))


def _aux_to_pwm(pos: float, kind: str, settings: Settings) -> int:
    """Map an AUX position [0, 1] to PWM according to its control type."""
    pos = _clamp(pos, 0.0, 1.0)
    if kind == "3pos":
        if pos <= 0.25:
            return settings.aux_low
        if pos >= 0.75:
            return settings.aux_high
        return settings.pwm_mid
    if kind == "slider":
        return round(settings.aux_low + pos * (settings.aux_high - settings.aux_low))
    return settings.aux_high if pos >= 0.5 else settings.aux_low


# ---------------------------------------------------------------------------
# UDP sender
# ---------------------------------------------------------------------------


class RCSender:
    """Sends RC packets to the SITL over a non-blocking UDP socket."""

    def __init__(self, ip: str, port: int) -> None:
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setblocking(False)
        self.ip = ip
        self.port = port
        self.packets_sent = 0
        self.last_error: Optional[str] = None

    def retarget(self, ip: str, port: int) -> None:
        self.ip = ip
        self.port = port

    def send(self, channels: list) -> None:
        chans = list(channels[:NUM_PACKET_CHANNELS])
        if len(chans) < NUM_PACKET_CHANNELS:
            chans += [1500] * (NUM_PACKET_CHANNELS - len(chans))
        payload = struct.pack("<d", time.time()) + struct.pack(
            "<%dH" % NUM_PACKET_CHANNELS, *chans
        )
        try:
            self.sock.sendto(payload, (self.ip, self.port))
            self.packets_sent += 1
            self.last_error = None
        except OSError as exc:
            self.last_error = str(exc)

    def close(self) -> None:
        try:
            self.sock.close()
        except OSError:
            pass


# ---------------------------------------------------------------------------
# PWM output receiver (Betaflight SITL -> port 9001)
# ---------------------------------------------------------------------------


class PWMSnapshot:
    """Immutable view of the receiver state, safe to read on the main thread."""

    __slots__ = ("enabled", "bound", "error", "motor_count", "values", "last_rx", "packets")

    def __init__(self, enabled, bound, error, motor_count, values, last_rx, packets):
        self.enabled = enabled
        self.bound = bound
        self.error = error
        self.motor_count = motor_count
        self.values = values
        self.last_rx = last_rx
        self.packets = packets

    def connected(self, timeout: float) -> bool:
        return self.bound and (time.time() - self.last_rx) < timeout


class PWMReceiver:
    """Listens for the SITL's raw PWM output (servo_packet_raw) on a UDP port.

    Runs on a dedicated daemon thread that blocks on ``recvfrom`` (with a short
    timeout so it can be stopped cleanly). The most recent packet is stored under
    a lock; the render loop reads a consistent view via :meth:`snapshot`.

    Packet layout (little-endian, natural C alignment):
        uint16_t motorCount;
        <2 bytes padding>
        float    pwm_output_raw[16];   # motors first, then servos
    => 68 bytes. Some builds may emit a packed 66-byte variant.
    """

    RX_TIMEOUT = 1.5  # seconds without a packet => considered "waiting"
    _SOCK_TIMEOUT = 0.2  # recv timeout so the thread can observe the stop flag
    _S_PADDED = struct.Struct("<H2x16f")   # 68 bytes (default C alignment)
    _S_PACKED = struct.Struct("<H16f")     # 66 bytes (packed builds)

    def __init__(self, port: int, enabled: bool = True, bind_ip: str = "0.0.0.0") -> None:
        self.port = port
        self.enabled = enabled
        self.bind_ip = bind_ip

        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._sock: Optional[socket.socket] = None

        # Shared state (guarded by _lock).
        self._bound = False
        self._error: Optional[str] = None
        self._motor_count = 0
        self._values = [0.0] * 16
        self._last_rx = 0.0
        self._packets = 0

        if enabled:
            self._start()

    # ---- lifecycle ----
    def _start(self) -> None:
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            s.settimeout(self._SOCK_TIMEOUT)
            s.bind((self.bind_ip, self.port))
            self._sock = s
            with self._lock:
                self._bound = True
                self._error = None
        except OSError as exc:
            self._sock = None
            with self._lock:
                self._bound = False
                self._error = str(exc)
            return

        self._stop.clear()
        self._thread = threading.Thread(target=self._run, name="pwm-rx", daemon=True)
        self._thread.start()

    def _stop_thread(self) -> None:
        self._stop.set()
        sock = self._sock
        if sock is not None:
            try:
                sock.close()   # unblocks a pending recvfrom
            except OSError:
                pass
        thread = self._thread
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=1.0)
        self._thread = None
        self._sock = None
        with self._lock:
            self._bound = False

    def _run(self) -> None:
        while not self._stop.is_set():
            sock = self._sock
            if sock is None:
                break
            try:
                data, _addr = sock.recvfrom(2048)
            except socket.timeout:
                continue
            except OSError as exc:
                if not self._stop.is_set():
                    with self._lock:
                        self._error = str(exc)
                        self._bound = False
                break
            parsed = self._parse(data)
            if parsed is not None:
                mc, vals = parsed
                with self._lock:
                    self._motor_count = mc
                    self._values = vals
                    self._last_rx = time.time()
                    self._packets += 1

    def reconfigure(self, port: int, enabled: bool) -> None:
        if port == self.port and enabled == self.enabled:
            return
        self._stop_thread()
        self.port = port
        self.enabled = enabled
        if enabled:
            self._start()

    def _parse(self, data: bytes):
        if len(data) >= self._S_PADDED.size:
            vals = self._S_PADDED.unpack_from(data, 0)
        elif len(data) >= self._S_PACKED.size:
            vals = self._S_PACKED.unpack_from(data, 0)
        else:
            return None
        motor_count = min(int(vals[0]), 16)
        return motor_count, list(vals[1:17])

    def snapshot(self) -> PWMSnapshot:
        with self._lock:
            return PWMSnapshot(
                self.enabled,
                self._bound,
                self._error,
                self._motor_count,
                list(self._values),
                self._last_rx,
                self._packets,
            )

    def close(self) -> None:
        self._stop_thread()


# ---------------------------------------------------------------------------
# MSP over WebSocket (DEVICE mode)
# ---------------------------------------------------------------------------


class MspCodec:
    """MSP v1 encode / stream decode (`$M<` request, `$M>` / `$M!` reply)."""

    @staticmethod
    def encode_request(cmd: int, payload: bytes = b"") -> bytes:
        if len(payload) > 255:
            raise ValueError("MSP v1 payload must be <= 255 bytes")
        size = len(payload)
        checksum = size ^ (cmd & 0xFF)
        for b in payload:
            checksum ^= b
        return b"$M<" + bytes([size, cmd & 0xFF]) + payload + bytes([checksum & 0xFF])


class MspDecoder:
    """Incremental MSP v1 frame decoder for a byte stream."""

    def __init__(self) -> None:
        self._buf = bytearray()

    def reset(self) -> None:
        self._buf.clear()

    def feed(self, data: bytes) -> list:
        """Return list of (cmd, payload, ok) where ok=False means error reply (`$M!`)."""
        if not data:
            return []
        self._buf.extend(data)
        out = []
        while True:
            parsed = self._try_one()
            if parsed is None:
                break
            out.append(parsed)
        return out

    def _try_one(self):
        buf = self._buf
        # Scan for `$M`
        while True:
            idx = buf.find(b"$M")
            if idx < 0:
                # Keep last byte in case of a split `$`
                if buf and buf[-1] == ord("$"):
                    del buf[:-1]
                else:
                    buf.clear()
                return None
            if idx > 0:
                del buf[:idx]
            if len(buf) < 3:
                return None
            direction = buf[2]
            if direction not in (ord("<"), ord(">"), ord("!")):
                del buf[0]
                continue
            if len(buf) < 5:
                return None
            size = buf[3]
            cmd = buf[4]
            total = 5 + size + 1
            if len(buf) < total:
                return None
            payload = bytes(buf[5:5 + size])
            checksum = buf[5 + size]
            expect = size ^ cmd
            for b in payload:
                expect ^= b
            del buf[:total]
            if (expect & 0xFF) != checksum:
                continue
            return cmd, payload, direction != ord("!")


class MspSnapshot:
    """Immutable view of the DEVICE MSP link, safe to read on the main thread."""

    __slots__ = (
        "enabled", "connected", "error", "motors", "servos", "motor_count",
        "armed", "cycle_time", "cpu_load", "last_rx", "packets", "rc_sent",
    )

    def __init__(
        self, enabled, connected, error, motors, servos, motor_count,
        armed, cycle_time, cpu_load, last_rx, packets, rc_sent,
    ):
        self.enabled = enabled
        self.connected = connected
        self.error = error
        self.motors = motors
        self.servos = servos
        self.motor_count = motor_count
        self.armed = armed
        self.cycle_time = cycle_time
        self.cpu_load = cpu_load
        self.last_rx = last_rx
        self.packets = packets
        self.rc_sent = rc_sent

    def alive(self, timeout: float) -> bool:
        return self.connected and (time.time() - self.last_rx) < timeout


class MspDeviceLink:
    """WebSocket MSP client: SET_RAW_RC + MOTOR/SERVO/STATUS_EX polling."""

    RX_TIMEOUT = 1.5
    _RECONNECT_DELAY = 1.0

    def __init__(self, url: str, enabled: bool = True, poll_hz: int = 20) -> None:
        self.url = url
        self.enabled = enabled
        self.poll_hz = max(1, int(poll_hz))

        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._ws = None
        self._decoder = MspDecoder()

        self._connected = False
        self._error: Optional[str] = None
        self._motors = [0.0] * 16
        self._servos = [0.0] * 16
        self._motor_count = 0
        self._armed = False
        self._cycle_time = 0
        self._cpu_load = 0
        self._last_rx = 0.0
        self._packets = 0
        self._rc_sent = 0
        self._pending_rc: Optional[bytes] = None
        self._pending_motors: Optional[bytes] = None

        if enabled:
            self._start()

    def _start(self) -> None:
        if websocket is None:
            with self._lock:
                self._error = "websocket-client not installed"
                self._connected = False
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, name="msp-ws", daemon=True)
        self._thread.start()

    def _stop_thread(self) -> None:
        self._stop.set()
        ws = self._ws
        if ws is not None:
            try:
                ws.close()
            except Exception:
                pass
        thread = self._thread
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=2.0)
        self._thread = None
        self._ws = None
        with self._lock:
            self._connected = False

    def reconfigure(self, url: str, enabled: bool, poll_hz: int) -> None:
        url = url.strip()
        poll_hz = max(1, int(poll_hz))
        if url == self.url and enabled == self.enabled and poll_hz == self.poll_hz:
            return
        self._stop_thread()
        self.url = url
        self.enabled = enabled
        self.poll_hz = poll_hz
        self._decoder.reset()
        if enabled:
            self._start()

    def send_rc(self, channels: list) -> None:
        """Queue an MSP_SET_RAW_RC frame (sent by the worker thread)."""
        n = max(4, min(len(channels), NUM_PACKET_CHANNELS))
        chans = [int(_clamp(c, 1000, 2000)) for c in channels[:n]]
        if len(chans) < n:
            chans += [1500] * (n - len(chans))
        payload = struct.pack("<%dH" % n, *chans)
        frame = MspCodec.encode_request(MSP_SET_RAW_RC, payload)
        with self._lock:
            self._pending_rc = frame

    def send_motors(self, values: list) -> None:
        """Queue MSP_SET_MOTOR (disarmed motor override / test)."""
        n = max(1, min(len(values), 8))
        vals = [int(_clamp(v, 1000, 2000)) for v in values[:n]]
        payload = struct.pack("<%dH" % n, *vals)
        frame = MspCodec.encode_request(MSP_SET_MOTOR, payload)
        with self._lock:
            self._pending_motors = frame

    def idle_motors(self, count: int = 4) -> None:
        """Command all motors to min (1000)."""
        self.send_motors([1000] * max(1, min(count, 8)))

    def snapshot(self) -> MspSnapshot:
        with self._lock:
            return MspSnapshot(
                self.enabled,
                self._connected,
                self._error,
                list(self._motors),
                list(self._servos),
                self._motor_count,
                self._armed,
                self._cycle_time,
                self._cpu_load,
                self._last_rx,
                self._packets,
                self._rc_sent,
            )

    def close(self) -> None:
        self._stop_thread()

    # ---- worker ----
    def _run(self) -> None:
        while not self._stop.is_set():
            try:
                ws = websocket.create_connection(
                    self.url,
                    timeout=3,
                    enable_multithread=True,
                    # Serial bridges (incl. the local :5761 server) require this.
                    subprotocols=["binary"],
                )
                ws.settimeout(0.05)
            except Exception as exc:
                with self._lock:
                    self._connected = False
                    self._error = str(exc)
                    self._ws = None
                self._stop.wait(self._RECONNECT_DELAY)
                continue

            self._decoder.reset()
            with self._lock:
                self._ws = ws
                self._connected = True
                self._error = None

            last_poll = 0.0
            try:
                while not self._stop.is_set():
                    # Flush pending RC / motor override frames.
                    with self._lock:
                        frame = self._pending_rc
                        self._pending_rc = None
                        motor_frame = self._pending_motors
                        self._pending_motors = None
                    if frame is not None:
                        try:
                            ws.send_binary(frame)
                            with self._lock:
                                self._rc_sent += 1
                        except Exception as exc:
                            with self._lock:
                                self._error = str(exc)
                            break
                    if motor_frame is not None:
                        try:
                            ws.send_binary(motor_frame)
                        except Exception as exc:
                            with self._lock:
                                self._error = str(exc)
                            break

                    now = time.time()
                    interval = 1.0 / self.poll_hz
                    if now - last_poll >= interval:
                        last_poll = now
                        for cmd in (MSP_MOTOR, MSP_SERVO, MSP_STATUS_EX):
                            try:
                                ws.send_binary(MspCodec.encode_request(cmd))
                            except Exception as exc:
                                with self._lock:
                                    self._error = str(exc)
                                break
                        else:
                            pass

                    try:
                        raw = ws.recv()
                    except websocket.WebSocketTimeoutException:
                        continue
                    except Exception as exc:
                        with self._lock:
                            self._error = str(exc)
                        break

                    if raw is None:
                        break
                    if isinstance(raw, str):
                        data = raw.encode("latin-1", errors="ignore")
                    else:
                        data = bytes(raw)
                    for cmd, payload, ok in self._decoder.feed(data):
                        if ok:
                            self._handle_reply(cmd, payload)
            finally:
                try:
                    ws.close()
                except Exception:
                    pass
                with self._lock:
                    if self._ws is ws:
                        self._ws = None
                    self._connected = False
            if not self._stop.is_set():
                self._stop.wait(self._RECONNECT_DELAY)

    def _handle_reply(self, cmd: int, payload: bytes) -> None:
        with self._lock:
            self._last_rx = time.time()
            self._packets += 1
            if cmd == MSP_MOTOR:
                n = len(payload) // 2
                vals = list(struct.unpack("<%dH" % n, payload[: n * 2])) if n else []
                motors = [0.0] * 16
                for i, v in enumerate(vals[:16]):
                    motors[i] = float(v)
                self._motors = motors
                # Count active motors (non-zero or any reported slot)
                self._motor_count = max(1, min(len(vals), 16)) if vals else 4
            elif cmd == MSP_SERVO:
                n = len(payload) // 2
                vals = list(struct.unpack("<%dH" % n, payload[: n * 2])) if n else []
                servos = [0.0] * 16
                for i, v in enumerate(vals[:16]):
                    servos[i] = float(v)
                self._servos = servos
            elif cmd == MSP_STATUS_EX and len(payload) >= 11:
                # cycleTime u16, i2cError u16, sensor u16, flightModeFlags u32, ...
                cycle_time, _i2c, _sensor = struct.unpack_from("<HHH", payload, 0)
                flags = struct.unpack_from("<I", payload, 6)[0]
                cpu_load = 0
                if len(payload) >= 13:
                    cpu_load = struct.unpack_from("<H", payload, 11)[0]
                self._cycle_time = int(cycle_time)
                self._cpu_load = int(cpu_load)
                self._armed = bool(flags & 0x1)


class Field:
    """A single editable row in the settings panel."""

    def __init__(
        self,
        label: str,
        attr: Optional[str],
        kind: str,
        step: float = 1.0,
        big_step: float = 10.0,
        vmin: float = 0.0,
        vmax: float = 0.0,
        fmt: Optional[Callable[[Any], str]] = None,
        choices: Optional[tuple] = None,
    ) -> None:
        self.label = label
        self.attr = attr          # None => section header (not selectable)
        self.kind = kind          # "int" | "float" | "bool" | "str" | "choice" | "header"
        self.step = step
        self.big_step = big_step
        self.vmin = vmin
        self.vmax = vmax
        self.fmt = fmt
        self.choices = tuple(choices) if choices else ()

    @property
    def selectable(self) -> bool:
        return self.attr is not None and self.kind != "header"


def build_fields(mode: str = "sitl") -> list:
    mode = mode if mode in MODES else "sitl"
    rows = [
        Field("CORE", None, "header"),
        Field("Mode", "mode", "choice", choices=MODES),
        Field("Send rate (Hz)", "send_hz", "int", 1, 10, 1, 250),
    ]
    if mode == "sitl":
        rows += [
            Field("Target IP", "ip", "str"),
            Field("RC out port", "port", "int", 1, 10, 1, 65535),
            Field("PWM in port", "pwm_in_port", "int", 1, 10, 1, 65535),
            Field("PWM in enabled", "pwm_in_enabled", "bool"),
            Field("Require PWM link", "require_pwm", "bool"),
        ]
    else:
        rows += [
            Field("MSP WebSocket URL", "msp_ws_url", "str"),
            Field("MSP poll (Hz)", "msp_poll_hz", "int", 1, 5, 1, 100),
            Field("Require MSP link", "require_msp", "bool"),
        ]
    rows += [
        Field("PWM RANGE", None, "header"),
        Field("PWM min", "pwm_min", "int", 5, 50, 500, 2500),
        Field("PWM mid", "pwm_mid", "int", 5, 50, 500, 2500),
        Field("PWM max", "pwm_max", "int", 5, 50, 500, 2500),
        Field("STICK FEEL", None, "header"),
        Field("Return speed", "return_speed", "float", 0.1, 1.0, 0.2, 50.0),
        Field("Key step rate", "key_step", "float", 0.1, 1.0, 0.2, 50.0),
        Field("Key accel", "key_accel", "float", 0.5, 2.0, 0.0, 100.0),
        Field("Key max rate", "key_step_max", "float", 0.5, 2.0, 0.2, 100.0),
        Field("Deadband", "deadband", "float", 0.01, 0.05, 0.0, 0.4),
        Field("Expo", "expo", "float", 0.05, 0.1, 0.0, 1.0),
        Field("THROTTLE", None, "header"),
        Field("Sticky throttle", "throttle_sticky", "bool"),
        Field("Arm-safe throttle PWM", "throttle_arm_safe", "int", 5, 50, 500, 2500),
        Field("CHANNELS", None, "header"),
        Field("Channel order", "channel_order", "str"),
        Field("Active channels", "channel_count", "int", 1, 4, 4, NUM_PACKET_CHANNELS),
        Field("AUX low PWM", "aux_low", "int", 5, 50, 500, 2500),
        Field("AUX high PWM", "aux_high", "int", 5, 50, 500, 2500),
    ]
    return rows


class SettingsPanel:
    """Overlay UI for viewing/editing settings with live-apply."""

    def __init__(self, settings: Settings, config_path: str) -> None:
        self.settings = settings
        self.config_path = config_path
        self.fields = build_fields(settings.mode)
        self.selected = self._first_selectable()
        self.editing = False
        self.edit_buffer = ""
        self.status = ""
        self.status_time = 0.0
        self.on_change: Optional[Callable[[], None]] = None

    def refresh_fields(self) -> None:
        """Rebuild rows when mode changes; keep selection on Mode if possible."""
        prev_attr = None
        if 0 <= self.selected < len(self.fields):
            prev_attr = self.fields[self.selected].attr
        self.fields = build_fields(self.settings.mode)
        if prev_attr:
            for i, f in enumerate(self.fields):
                if f.attr == prev_attr:
                    self.selected = i
                    return
        self.selected = self._first_selectable()

    def _first_selectable(self) -> int:
        for i, f in enumerate(self.fields):
            if f.selectable:
                return i
        return 0

    def _set_status(self, msg: str) -> None:
        self.status = msg
        self.status_time = time.time()

    def _move(self, direction: int) -> None:
        i = self.selected
        n = len(self.fields)
        for _ in range(n):
            i = (i + direction) % n
            if self.fields[i].selectable:
                self.selected = i
                return

    def _adjust(self, field_obj: Field, amount: float) -> None:
        s = self.settings
        cur = getattr(s, field_obj.attr)
        if field_obj.kind == "int":
            setattr(s, field_obj.attr, int(round(cur + amount)))
        elif field_obj.kind == "float":
            setattr(s, field_obj.attr, round(float(cur) + amount, 4))
        elif field_obj.kind == "bool":
            setattr(s, field_obj.attr, not cur)
        elif field_obj.kind == "choice" and field_obj.choices:
            choices = field_obj.choices
            try:
                idx = choices.index(cur)
            except ValueError:
                idx = 0
            idx = (idx + (1 if amount >= 0 else -1)) % len(choices)
            setattr(s, field_obj.attr, choices[idx])
        else:
            return
        self._apply()

    def _apply(self) -> None:
        self.settings.clamp_all()
        # Rebuild CORE rows when mode flips between sitl / device.
        new_attrs = [f.attr for f in build_fields(self.settings.mode)]
        old_attrs = [f.attr for f in self.fields]
        if new_attrs != old_attrs:
            self.refresh_fields()
        if self.on_change:
            self.on_change()

    def _commit_edit(self, field_obj: Field) -> None:
        raw = self.edit_buffer.strip()
        try:
            if field_obj.kind == "int":
                setattr(self.settings, field_obj.attr, int(float(raw)))
            elif field_obj.kind == "float":
                setattr(self.settings, field_obj.attr, float(raw))
            elif field_obj.kind in ("str", "choice"):
                setattr(self.settings, field_obj.attr, raw)
            self._apply()
            self._set_status(f"Set {field_obj.label}")
        except ValueError:
            self._set_status("Invalid value")
        self.editing = False
        self.edit_buffer = ""

    # ------------------------------------------------------------------
    def handle_event(self, event: pygame.event.Event) -> None:
        field_obj = self.fields[self.selected]
        mods = pygame.key.get_mods()
        coarse = bool(mods & pygame.KMOD_SHIFT)

        if self.editing:
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_RETURN:
                    self._commit_edit(field_obj)
                elif event.key == pygame.K_ESCAPE:
                    self.editing = False
                    self.edit_buffer = ""
                elif event.key == pygame.K_BACKSPACE:
                    self.edit_buffer = self.edit_buffer[:-1]
                elif event.unicode and event.unicode.isprintable():
                    self.edit_buffer += event.unicode
            return

        if event.type != pygame.KEYDOWN:
            return

        ctrl = bool(mods & pygame.KMOD_CTRL)
        # Check Ctrl+S first so it isn't swallowed by other bindings.
        if ctrl and event.key == pygame.K_s:
            self._save()
            return

        if event.key == pygame.K_UP:
            self._move(-1)
        elif event.key == pygame.K_DOWN:
            self._move(1)
        elif event.key == pygame.K_LEFT:
            step = field_obj.big_step if coarse else field_obj.step
            self._adjust(field_obj, -step)
        elif event.key == pygame.K_RIGHT:
            step = field_obj.big_step if coarse else field_obj.step
            self._adjust(field_obj, step)
        elif event.key == pygame.K_RETURN:
            if field_obj.kind == "bool":
                self._adjust(field_obj, 0)
            elif field_obj.kind == "choice":
                self._adjust(field_obj, 1)
            else:
                self.editing = True
                self.edit_buffer = str(getattr(self.settings, field_obj.attr))
        elif event.key == pygame.K_d:
            self._reset_defaults()

    def _save(self) -> None:
        ok = self.settings.save(self.config_path)
        self._set_status("Saved to config.json" if ok else "Save failed")

    def _reset_defaults(self) -> None:
        defaults = Settings()
        for f in fields(Settings):
            setattr(self.settings, f.name, getattr(defaults, f.name))
        self.refresh_fields()
        self._apply()
        self._set_status("Reset to defaults")


# ---------------------------------------------------------------------------
# Application
# ---------------------------------------------------------------------------


class App:
    def __init__(self, settings: Settings, config_path: str) -> None:
        self.settings = settings
        self.config_path = config_path
        self.state = RCState()
        self.state.reset(settings)
        self.sender = RCSender(settings.ip, settings.port)
        # Only bind the SITL PWM port when actually in SITL mode.
        self.pwm_in = PWMReceiver(
            settings.pwm_in_port,
            settings.pwm_in_enabled and not settings.is_device,
        )
        self.msp = MspDeviceLink(
            settings.msp_ws_url,
            enabled=settings.is_device,
            poll_hz=settings.msp_poll_hz,
        )

        self.panel = SettingsPanel(settings, config_path)
        self.panel.on_change = self._on_settings_changed
        self.panel_open = False

        self.running = True
        self.dragging: Optional[str] = None  # "left" | "right" | ("aux", idx) | ("motor", idx) | None
        self._send_accumulator = 0.0
        self.rc_active = False  # True while actually transmitting RC
        # Seconds each stick axis key has been held (for rate acceleration).
        self._key_hold = {"roll": 0.0, "pitch": 0.0, "yaw": 0.0, "throttle": 0.0}
        # DEVICE disarmed motor test (MSP_SET_MOTOR). Servos have no live MSP set.
        self.motor_override = [1000] * 8
        self.motor_test = False
        self._motor_bar_layout: list = []
        self._was_armed = False

        pygame.init()
        pygame.display.set_caption("Betaflight Joystick Emulator")
        self.screen = pygame.display.set_mode((WINDOW_W, WINDOW_H))
        self.clock = pygame.time.Clock()

        # Typography: UI font for labels, mono font for numeric readouts.
        self.font_title = _sysfont(["Segoe UI Semibold", "Segoe UI", "Arial"], 19, bold=True)
        self.font_h = _sysfont(["Segoe UI", "Arial"], 11, bold=True)
        self.font = _sysfont(["Segoe UI", "Arial"], 14)
        self.font_small = _sysfont(["Segoe UI", "Arial"], 11)
        self.font_mono = _sysfont(["Consolas", "Courier New"], 14)
        self.font_mono_sm = _sysfont(["Consolas", "Courier New"], 11)
        self.font_arm = _sysfont(["Segoe UI Black", "Segoe UI", "Arial"], 22, bold=True)

        # Cached vertical-gradient background.
        self.bg = _make_gradient(WINDOW_W, WINDOW_H, COL_BG_TOP, COL_BG_BOT)

        # ---- Layout geometry ----
        margin = 14
        gap = 12
        top = 60
        card_w = 300
        card_h = 224
        self.left_card = pygame.Rect(margin, top, card_w, card_h)
        self.right_card = pygame.Rect(WINDOW_W - margin - card_w, top, card_w, card_h)
        self.center_card = pygame.Rect(
            self.left_card.right + gap,
            top,
            self.right_card.left - self.left_card.right - 2 * gap,
            card_h,
        )
        self.chan_card = pygame.Rect(
            margin, self.left_card.bottom + gap, WINDOW_W - 2 * margin, 200
        )
        self.motor_card = pygame.Rect(
            margin, self.chan_card.bottom + gap, WINDOW_W - 2 * margin, 110
        )

        self.gimbal_r = 70
        gy = self.left_card.y + 110
        self.left_center = (self.left_card.centerx, gy)
        self.right_center = (self.right_card.centerx, gy)

    # ------------------------------------------------------------------
    def _on_settings_changed(self) -> None:
        self.sender.retarget(self.settings.ip, self.settings.port)
        sitl = not self.settings.is_device
        if sitl and self.motor_test:
            self._stop_motor_test(send_idle=True)
        self.pwm_in.reconfigure(
            self.settings.pwm_in_port,
            self.settings.pwm_in_enabled and sitl,
        )
        self.msp.reconfigure(
            self.settings.msp_ws_url,
            enabled=self.settings.is_device,
            poll_hz=self.settings.msp_poll_hz,
        )

    # ------------------------------------------------------------------
    def run(self) -> None:
        while self.running:
            dt = self.clock.tick(FPS_RENDER_CAP) / 1000.0
            self._handle_events()
            if not self.panel_open:
                self._update_inputs(dt)
            self._maybe_send(dt)
            self._render()
        self._shutdown()

    def _shutdown(self) -> None:
        if self.settings.is_device:
            self._stop_motor_test(send_idle=True)
            time.sleep(0.05)  # give the WS thread a tick to flush idle motors
        self.sender.close()
        self.pwm_in.close()
        self.msp.close()
        pygame.quit()

    def _fc_disarmed(self) -> bool:
        """True when motor test is allowed (DEVICE + both local and FC disarmed)."""
        if not self.settings.is_device:
            return False
        if self.state.armed:
            return False
        msp = self.msp.snapshot()
        if msp.alive(MspDeviceLink.RX_TIMEOUT) and msp.armed:
            return False
        return True

    def _is_armed(self) -> bool:
        """Armed if local CH5 is high or FC reports armed over MSP."""
        if self.state.armed:
            return True
        msp = self.msp.snapshot()
        return bool(msp.alive(MspDeviceLink.RX_TIMEOUT) and msp.armed)

    def _motor_count(self) -> int:
        mc = self.msp.snapshot().motor_count
        return max(1, min(mc if mc > 0 else 4, 8))

    def _stop_motor_test(self, send_idle: bool = True) -> None:
        """Clear motor-test override and optionally command motors to idle (1000)."""
        was_testing = self.motor_test or any(v > 1000 for v in self.motor_override)
        self.motor_test = False
        self.motor_override = [1000] * 8
        if isinstance(self.dragging, tuple) and self.dragging[0] == "motor":
            self.dragging = None
        if send_idle and self.settings.is_device and was_testing:
            self.msp.idle_motors(self._motor_count())

    def _set_motor_override(self, idx: int, pwm: int) -> None:
        if not self._fc_disarmed():
            return
        n = self._motor_count()
        if not 0 <= idx < n:
            return
        self.motor_override[idx] = int(_clamp(pwm, 1000, 2000))
        self.motor_test = True

    def _reset_motors_if_armed(self) -> None:
        """If armed, force motor control back to idle / inactive."""
        if self._is_armed() and (
            self.motor_test or any(v > 1000 for v in self.motor_override)
        ):
            self._stop_motor_test(send_idle=True)

    # ------------------------------------------------------------------
    # Events
    # ------------------------------------------------------------------
    def _handle_events(self) -> None:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.running = False
                return

            if self.panel_open:
                if (
                    event.type == pygame.KEYDOWN
                    and not self.panel.editing
                    and event.key in (pygame.K_TAB, pygame.K_ESCAPE)
                ):
                    self.panel_open = False
                    continue
                self.panel.handle_event(event)
                continue

            if event.type == pygame.KEYDOWN:
                self._handle_flight_keydown(event)
            elif event.type == pygame.MOUSEBUTTONDOWN and event.button in (1, 3):
                self._handle_mouse_down(event.pos, event.button)
            elif event.type == pygame.MOUSEBUTTONUP and event.button == 1:
                self.dragging = None

    # Number-row keys actuate CH6..CH15 (1->CH6 ... 9->CH14, 0->CH15).
    _NUM_KEY_AUX = {
        pygame.K_1: 1, pygame.K_2: 2, pygame.K_3: 3, pygame.K_4: 4, pygame.K_5: 5,
        pygame.K_6: 6, pygame.K_7: 7, pygame.K_8: 8, pygame.K_9: 9, pygame.K_0: 10,
    }

    def _handle_flight_keydown(self, event: pygame.event.Event) -> None:
        key = event.key
        if key == pygame.K_ESCAPE:
            self.running = False
        elif key == pygame.K_TAB:
            self.panel_open = True
        elif key == pygame.K_RETURN:
            self._actuate_aux(0)  # CH5 (arm)
            self._reset_motors_if_armed()
        elif key == pygame.K_r:
            self.state.reset(self.settings)
            self._stop_motor_test(send_idle=True)
        elif key == pygame.K_m and self.settings.is_device:
            # Stop motor test / idle all motors
            self._stop_motor_test(send_idle=True)
        elif key in self._NUM_KEY_AUX:
            self._actuate_aux(self._NUM_KEY_AUX[key])

    def _actuate_aux(self, idx: int) -> None:
        """Advance an AUX channel a notch, if that channel is active."""
        if 4 + idx >= self.settings.channel_count:
            return
        self.state.actuate(idx, self.settings.aux_types)
        # Arming (CH5 high) must clear any MSP motor override immediately.
        if idx == 0:
            self._reset_motors_if_armed()

    def _cycle_aux_type(self, idx: int) -> None:
        """Cycle a channel's control type (2pos -> 3pos -> slider) and persist it."""
        cur = self.settings.aux_types[idx]
        nxt = AUX_TYPES[(AUX_TYPES.index(cur) + 1) % len(AUX_TYPES)] if cur in AUX_TYPES else "2pos"
        self.settings.aux_types[idx] = nxt
        # Re-quantize the current position for the new type.
        self.state.set_pos(idx, self.state.aux[idx], self.settings.aux_types)
        self.settings.save(self.config_path)

    def _handle_mouse_down(self, pos, button: int) -> None:
        if button == 1:
            if _dist(pos, self.left_center) <= self.gimbal_r:
                self.dragging = "left"
                self._drag_stick(pos)
                return
            if _dist(pos, self.right_center) <= self.gimbal_r:
                self.dragging = "right"
                self._drag_stick(pos)
                return
            # DEVICE + disarmed: drag a motor bar for MSP_SET_MOTOR test.
            if self._fc_disarmed():
                hit = self._motor_bar_hit(pos)
                if hit is not None:
                    self.dragging = ("motor", hit)
                    self._drag_motor(hit, pos[1])
                    return

        hit = self._channel_hit(pos)
        if hit is None:
            return
        idx, frac = hit
        if 4 + idx >= self.settings.channel_count:
            return
        if button == 3:
            self._cycle_aux_type(idx)
            return
        if self.settings.aux_types[idx] == "slider":
            self.dragging = ("aux", idx)
            self.state.set_pos(idx, frac, self.settings.aux_types)
        else:
            self.state.actuate(idx, self.settings.aux_types)

    def _drag_stick(self, pos) -> None:
        if self.dragging == "left":
            nx, ny = _normalize_in_circle(pos, self.left_center, self.gimbal_r)
            self.state.yaw = nx
            self.state.throttle = _clamp((-ny + 1.0) / 2.0, 0.0, 1.0)
        elif self.dragging == "right":
            nx, ny = _normalize_in_circle(pos, self.right_center, self.gimbal_r)
            self.state.roll = nx
            self.state.pitch = -ny

    def _drag_aux(self, idx: int) -> None:
        mx = pygame.mouse.get_pos()[0]
        for e in self._channel_layout():
            if e["i"] == 4 + idx:
                frac = _clamp((mx - e["track_x"]) / max(1, e["track_w"]), 0.0, 1.0)
                self.state.set_pos(idx, frac, self.settings.aux_types)
                return

    def _motor_bar_hit(self, pos) -> Optional[int]:
        for e in self._motor_bar_layout:
            if e["rect"].collidepoint(pos):
                return e["i"]
        return None

    def _drag_motor(self, idx: int, my: Optional[int] = None) -> None:
        if my is None:
            my = pygame.mouse.get_pos()[1]
        for e in self._motor_bar_layout:
            if e["i"] == idx:
                # Top of bar = max (2000), bottom = min (1000)
                frac = 1.0 - _clamp((my - e["top"]) / max(1, e["height"]), 0.0, 1.0)
                pwm = int(round(1000 + frac * 1000))
                self._set_motor_override(idx, pwm)
                return

    # ------------------------------------------------------------------
    # Continuous input update
    # ------------------------------------------------------------------
    def _update_inputs(self, dt: float) -> None:
        if isinstance(self.dragging, tuple) and self.dragging[0] == "motor":
            if self._fc_disarmed():
                self._drag_motor(self.dragging[1])
            else:
                self.dragging = None
                self._stop_motor_test(send_idle=True)
            return
        if isinstance(self.dragging, tuple) and self.dragging[0] == "aux":
            self._drag_aux(self.dragging[1])
            return
        if self.dragging in ("left", "right"):
            self._drag_stick(pygame.mouse.get_pos())
            # While dragging one stick, the other still self-centers.
            if self.dragging == "left":
                self._center_axis("roll", dt)
                self._center_axis("pitch", dt)
            else:
                self._center_axis("yaw", dt)
                if not self.settings.throttle_sticky:
                    self._decay_throttle(dt)
            for k in self._key_hold:
                self._key_hold[k] = 0.0
            return

        keys = pygame.key.get_pressed()

        # Right stick: arrows -> roll / pitch
        roll_in = (1 if keys[pygame.K_RIGHT] else 0) - (1 if keys[pygame.K_LEFT] else 0)
        pitch_in = (1 if keys[pygame.K_UP] else 0) - (1 if keys[pygame.K_DOWN] else 0)
        self._drive_axis("roll", roll_in, dt)
        self._drive_axis("pitch", pitch_in, dt)

        # Left stick: A/D -> yaw, W/S -> throttle (sticky)
        yaw_in = (1 if keys[pygame.K_d] else 0) - (1 if keys[pygame.K_a] else 0)
        self._drive_axis("yaw", yaw_in, dt)

        thr_in = (1 if keys[pygame.K_w] else 0) - (1 if keys[pygame.K_s] else 0)
        if thr_in != 0:
            step = self._key_rate("throttle", dt) * dt
            self.state.throttle = _clamp(self.state.throttle + thr_in * step, 0.0, 1.0)
        else:
            self._key_hold["throttle"] = 0.0
            if not self.settings.throttle_sticky:
                self._decay_throttle(dt)

    def _key_rate(self, attr: str, dt: float) -> float:
        """Current keyboard stick rate for an axis, ramping up while held."""
        hold = self._key_hold.get(attr, 0.0) + dt
        self._key_hold[attr] = hold
        rate = self.settings.key_step + self.settings.key_accel * hold
        return min(rate, self.settings.key_step_max)

    def _drive_axis(self, attr: str, direction: int, dt: float) -> None:
        cur = getattr(self.state, attr)
        if direction != 0:
            step = self._key_rate(attr, dt) * dt
            cur = _clamp(cur + direction * step, -1.0, 1.0)
            setattr(self.state, attr, cur)
        else:
            self._key_hold[attr] = 0.0
            self._center_axis(attr, dt)

    def _center_axis(self, attr: str, dt: float) -> None:
        cur = getattr(self.state, attr)
        move = self.settings.return_speed * dt
        if cur > 0:
            cur = max(0.0, cur - move)
        elif cur < 0:
            cur = min(0.0, cur + move)
        setattr(self.state, attr, cur)

    def _decay_throttle(self, dt: float) -> None:
        move = self.settings.return_speed * dt
        self.state.throttle = max(0.0, self.state.throttle - move)

    # ------------------------------------------------------------------
    # Sending
    # ------------------------------------------------------------------
    def _maybe_send(self, dt: float) -> None:
        channels = self.state.to_channels(self.settings)

        if self.settings.is_device:
            gate = self.settings.require_msp
            msp = self.msp.snapshot()
            if gate and not msp.alive(MspDeviceLink.RX_TIMEOUT):
                self.rc_active = False
                self._send_accumulator = 0.0
                if self.motor_test:
                    self._stop_motor_test(send_idle=False)
                return

            # Arming (local CH5 or FC) always resets motor override.
            armed_now = self._is_armed()
            if armed_now and not self._was_armed:
                self._stop_motor_test(send_idle=True)
            elif armed_now:
                self._reset_motors_if_armed()
            self._was_armed = armed_now

            self.rc_active = True
            interval = 1.0 / max(1, self.settings.send_hz)
            self._send_accumulator += dt
            sent = 0
            while self._send_accumulator >= interval and sent < 5:
                self.msp.send_rc(channels)
                # Never send SET_MOTOR while armed; only while actively testing.
                if self.motor_test and not armed_now:
                    n = self._motor_count()
                    self.msp.send_motors(self.motor_override[:n])
                self._send_accumulator -= interval
                sent += 1
            return

        # SITL: gate on PWM link when required.
        gate = self.settings.require_pwm and self.settings.pwm_in_enabled
        if gate and not self.pwm_in.snapshot().connected(PWMReceiver.RX_TIMEOUT):
            self.rc_active = False
            self._send_accumulator = 0.0
            return

        self.rc_active = True
        interval = 1.0 / max(1, self.settings.send_hz)
        self._send_accumulator += dt
        sent = 0
        while self._send_accumulator >= interval and sent < 5:
            self.sender.send(channels)
            self._send_accumulator -= interval
            sent += 1

    # ------------------------------------------------------------------
    # Rendering
    # ------------------------------------------------------------------
    def _render(self) -> None:
        self.screen.blit(self.bg, (0, 0))
        self._draw_header()
        self._draw_gimbal_card(self.left_card, self.left_center, "left")
        self._draw_gimbal_card(self.right_card, self.right_center, "right")
        self._draw_status_card(self.center_card)
        self._draw_channels(self.chan_card)
        self._draw_motors_card(self.motor_card)
        self._draw_legend()
        if self.panel_open:
            self._draw_panel()
        pygame.display.flip()

    # ---- shared helpers ----
    def _panel(self, rect: pygame.Rect, title: Optional[str] = None) -> None:
        pygame.draw.rect(self.screen, COL_PANEL, rect, border_radius=12)
        pygame.draw.rect(self.screen, COL_BORDER, rect, width=1, border_radius=12)
        if title:
            self._text(title.upper(), self.font_h, COL_TEXT_FAINT, (rect.x + 16, rect.y + 13))
            ly = rect.y + 34
            pygame.draw.line(self.screen, COL_BORDER, (rect.x + 14, ly), (rect.right - 14, ly), 1)

    def _text(self, s: str, font, color, pos, align: str = "left") -> pygame.Rect:
        surf = font.render(s, True, color)
        x, y = pos
        if align == "center":
            x -= surf.get_width() // 2
        elif align == "right":
            x -= surf.get_width()
        self.screen.blit(surf, (int(x), int(y)))
        return surf.get_rect(topleft=(int(x), int(y)))

    # ---- header ----
    def _draw_header(self) -> None:
        bar = pygame.Rect(0, 0, WINDOW_W, 48)
        pygame.draw.rect(self.screen, COL_PANEL, bar)
        pygame.draw.line(self.screen, COL_BORDER_HI, (0, 48), (WINDOW_W, 48), 1)
        pygame.draw.rect(self.screen, COL_ACCENT, (0, 0, 4, 48))
        mode_label = "DEVICE" if self.settings.is_device else "SITL"
        self._text(f"BETAFLIGHT {mode_label}", self.font_title, COL_TEXT, (14, 6))
        self._text("JOYSTICK EMULATOR", self.font_h, COL_ACCENT, (16, 29))

        if self.settings.is_device:
            msp = self.msp.snapshot()
            if msp.error and not msp.connected:
                status, dot_col = "WS ERROR", COL_BAD
            elif self.rc_active:
                status, dot_col = "SENDING", COL_GOOD
            elif self.settings.require_msp:
                status, dot_col = "STANDBY (no MSP)", COL_WARN
            else:
                status, dot_col = "IDLE", COL_TEXT_DIM
            try:
                host = urlparse(self.settings.msp_ws_url).netloc or self.settings.msp_ws_url
            except Exception:
                host = self.settings.msp_ws_url
            endpoint = host
            pkts = f"{msp.rc_sent:,} rc"
        else:
            if self.sender.last_error is not None:
                status, dot_col = "SEND ERROR", COL_BAD
            elif self.rc_active:
                status, dot_col = "SENDING", COL_GOOD
            elif self.settings.require_pwm and self.settings.pwm_in_enabled:
                status, dot_col = "STANDBY (no PWM)", COL_WARN
            else:
                status, dot_col = "IDLE", COL_TEXT_DIM
            endpoint = f"{self.settings.ip}:{self.settings.port}"
            pkts = f"{self.sender.packets_sent:,} pkts"

        segments = [
            (status, dot_col, True),
            (endpoint, COL_TEXT, False),
            (f"{self.settings.send_hz} Hz", COL_TEXT_DIM, False),
            (pkts, COL_TEXT_DIM, False),
        ]
        x = WINDOW_W - 14
        rev = list(reversed(segments))
        for idx, (text, color, dot) in enumerate(rev):
            r = self.font_mono_sm.render(text, True, color)
            x -= r.get_width()
            self.screen.blit(r, (x, 17))
            if dot:
                _fill_circle(self.screen, x - 11, 23, 4, color)
                x -= 20
            if idx < len(rev) - 1:
                x -= 18
                self._text("|", self.font_mono_sm, COL_BORDER_HI, (x + 7, 17))

    # ---- gimbals ----
    def _draw_gimbal_card(self, card: pygame.Rect, center, side: str) -> None:
        label = "THROTTLE / YAW" if side == "left" else "ROLL / PITCH"
        self._panel(card, label)
        if side == "left":
            nx, ny = self.state.yaw, 1.0 - self.state.throttle * 2
        else:
            nx, ny = self.state.roll, -self.state.pitch
        self._draw_gimbal(center, nx, ny, side)

        # Readout chips beneath the gimbal.
        base_y = card.bottom - 42
        q1 = card.x + card.w // 4
        q2 = card.x + 3 * card.w // 4
        if side == "left":
            self._readout(q1, base_y, "THROTTLE", f"{round(self.state.throttle * 100)}%", COL_THROTTLE)
            self._readout(q2, base_y, "YAW", _pct(self.state.yaw), COL_ACCENT)
        else:
            self._readout(q1, base_y, "ROLL", _pct(self.state.roll), COL_ACCENT)
            self._readout(q2, base_y, "PITCH", _pct(self.state.pitch), COL_ACCENT)

    def _readout(self, cx: int, top: int, label: str, value: str, color) -> None:
        self._text(label, self.font_small, COL_TEXT_DIM, (cx, top), align="center")
        self._text(value, self.font_mono, color, (cx, top + 16), align="center")

    def _draw_gimbal(self, center, nx: float, ny: float, side: str) -> None:
        cx, cy = center
        r = self.gimbal_r
        _fill_circle(self.screen, cx, cy, r, COL_GIMBAL)
        # concentric guide rings + crosshair
        for rr in (int(r * 0.34), int(r * 0.67)):
            gfxdraw.aacircle(self.screen, cx, cy, rr, COL_GIMBAL_GRID)
        pygame.draw.line(self.screen, COL_GIMBAL_GRID, (cx - r, cy), (cx + r, cy), 1)
        pygame.draw.line(self.screen, COL_GIMBAL_GRID, (cx, cy - r), (cx, cy + r), 1)
        gfxdraw.aacircle(self.screen, cx, cy, r, COL_GIMBAL_RING)
        gfxdraw.aacircle(self.screen, cx, cy, r - 1, COL_GIMBAL_RING)

        kx = cx + _clamp(nx, -1, 1) * r
        ky = cy + _clamp(ny, -1, 1) * r
        # reticle projection lines clipped to the circle
        dxr = math.sqrt(max(0.0, r * r - (ky - cy) ** 2))
        dyr = math.sqrt(max(0.0, r * r - (kx - cx) ** 2))
        pygame.draw.line(self.screen, COL_ACCENT_DIM, (cx - dxr, ky), (cx + dxr, ky), 1)
        pygame.draw.line(self.screen, COL_ACCENT_DIM, (kx, cy - dyr), (kx, cy + dyr), 1)

        grabbed = self.dragging == side
        kcol = COL_KNOB_GRAB if grabbed else COL_KNOB
        _fill_circle(self.screen, kx, ky, 11, kcol)
        _fill_circle(self.screen, kx, ky, 4, COL_GIMBAL)

    # ---- status / switches ----
    def _draw_status_card(self, card: pygame.Rect) -> None:
        self._panel(card, "Status")
        inner_x = card.x + 14
        inner_w = card.w - 28

        armed = self.state.armed
        fc_note = "CH5"
        if self.settings.is_device:
            msp = self.msp.snapshot()
            if msp.alive(MspDeviceLink.RX_TIMEOUT):
                armed = msp.armed
                fc_note = "FC"

        banner = pygame.Rect(inner_x, card.y + 44, inner_w, 50)
        if armed:
            pygame.draw.rect(self.screen, COL_GOOD, banner, border_radius=9)
            self._text("ARMED", self.font_arm, (10, 20, 15), (banner.centerx, banner.y + 12), align="center")
        else:
            pygame.draw.rect(self.screen, COL_PANEL_LIGHT, banner, border_radius=9)
            pygame.draw.rect(self.screen, COL_BAD, banner, width=2, border_radius=9)
            self._text("DISARMED", self.font_arm, COL_BAD, (banner.centerx, banner.y + 12), align="center")

        if self.settings.is_device:
            msp = self.msp.snapshot()
            hy = banner.bottom + 10
            link = "MSP LINK" if msp.alive(MspDeviceLink.RX_TIMEOUT) else "MSP WAIT"
            lcol = COL_GOOD if msp.alive(MspDeviceLink.RX_TIMEOUT) else COL_WARN
            self._text(link, self.font_h, lcol, (inner_x + 2, hy))
            self._text(f"load {msp.cpu_load}%  {msp.cycle_time}us", self.font_small,
                       COL_TEXT_DIM, (inner_x + 2, hy + 16))
            self._text(f"arm src: {fc_note}   CH5 still overrides", self.font_small,
                       COL_TEXT_FAINT, (inner_x + 2, hy + 34))
            self._text("Enter arm   Click AUX CH5-16", self.font_small,
                       COL_TEXT_DIM, (inner_x + 2, hy + 52))
        else:
            hints = [
                ("CH5-16", "AUX switches"),
                ("Left-click", "actuate / cycle"),
                ("Right-click", "change type"),
                ("Enter / 1-0", "CH5 / CH6-15"),
            ]
            hy = banner.bottom + 14
            for key, desc in hints:
                self._text(key, self.font_h, COL_ACCENT, (inner_x + 2, hy))
                self._text(desc, self.font_small, COL_TEXT_DIM, (inner_x + 88, hy))
                hy += 18
            self._text("2P 2-pos   3P 3-pos   SL slider", self.font_small,
                       COL_TEXT_FAINT, (inner_x + 2, hy + 4))

    # ---- channels ----
    def _channel_layout(self):
        """Per-channel cell/track geometry, shared by drawing and hit-testing."""
        card = self.chan_card
        cols = 4
        inner_x = card.x + 16
        inner_w = card.w - 32
        col_w = inner_w // cols
        top = card.y + 42
        row_h = 38
        out = []
        for i in range(self.settings.channel_count):
            colx = inner_x + (i % cols) * col_w
            cy = top + (i // cols) * row_h
            out.append({
                "i": i, "colx": colx, "cy": cy, "col_w": col_w, "row_h": row_h,
                "track_x": colx, "track_y": cy + 22, "track_w": col_w - 60, "track_h": 6,
            })
        return out

    def _channel_hit(self, pos):
        """Return (aux_index, track_fraction) for an AUX channel cell under pos."""
        for e in self._channel_layout():
            if e["i"] < 4:
                continue
            cell = pygame.Rect(e["colx"], e["cy"] - 2, e["col_w"] - 6, e["row_h"] - 4)
            if cell.collidepoint(pos):
                frac = _clamp((pos[0] - e["track_x"]) / max(1, e["track_w"]), 0.0, 1.0)
                return e["i"] - 4, frac
        return None

    def _channel_meta(self):
        fnmap = {
            "A": ("Roll", COL_ACCENT, True),
            "E": ("Pitch", COL_ACCENT, True),
            "T": ("Thr", COL_THROTTLE, False),
            "R": ("Yaw", COL_ACCENT, True),
        }
        meta = []
        order = self.settings.channel_order
        for i in range(NUM_PACKET_CHANNELS):
            if i < 4:
                letter = order[i] if i < len(order) else "?"
                name, col, bip = fnmap.get(letter, ("?", COL_MUTED, True))
                meta.append((name, col, bip, "axis"))
            elif i == 4:
                meta.append(("Arm", COL_GOOD, False, "aux"))
            else:
                meta.append((f"Aux{i - 4}", COL_AUX, False, "aux"))
        return meta

    _TYPE_BADGE = {"2pos": "2P", "3pos": "3P", "slider": "SL"}

    def _draw_channels(self, card: pygame.Rect) -> None:
        self._panel(card, "Channels")
        chans = self.state.to_channels(self.settings)
        meta = self._channel_meta()
        count = self.settings.channel_count
        self._text(f"{count} ACTIVE", self.font_h, COL_TEXT_FAINT, (card.right - 16, card.y + 13), align="right")

        mid = self.settings.pwm_mid
        mouse = pygame.mouse.get_pos()

        for e in self._channel_layout():
            i = e["i"]
            name, col, bipolar, kind = meta[i]
            val = chans[i]
            colx, cy, col_w = e["colx"], e["cy"], e["col_w"]
            is_aux = i >= 4

            active = val >= mid if kind == "aux" else True
            draw_col = col if active else COL_MUTED

            # Highlight the AUX cell under the cursor to signal it is clickable.
            if is_aux:
                cell = pygame.Rect(colx, cy - 2, col_w - 6, e["row_h"] - 4)
                if cell.collidepoint(mouse):
                    pygame.draw.rect(self.screen, COL_PANEL_LIGHT, cell, border_radius=5)

            label = f"CH{i + 1}"
            self._text(label, self.font_mono_sm, COL_TEXT, (colx, cy))
            if name:
                lw = self.font_mono_sm.size(label + " ")[0]
                self._text(name, self.font_small, COL_TEXT_DIM, (colx + lw + 4, cy + 1))

            if is_aux:
                badge = self._TYPE_BADGE.get(self.settings.aux_types[i - 4], "2P")
                self._text(badge, self.font_small, COL_ACCENT_DIM, (colx + col_w - 12, cy), align="right")

            track_x, track_y = e["track_x"], e["track_y"]
            track_w, track_h = e["track_w"], e["track_h"]
            pygame.draw.rect(self.screen, COL_TRACK, (track_x, track_y, track_w, track_h), border_radius=3)

            frac = _clamp((val - 1000) / 1000.0, 0.0, 1.0)
            if bipolar:
                cx_track = track_x + track_w // 2
                pos = track_x + int(frac * track_w)
                fx = min(cx_track, pos)
                fw = max(2, abs(pos - cx_track))
                pygame.draw.rect(self.screen, draw_col, (fx, track_y, fw, track_h), border_radius=3)
                pygame.draw.line(self.screen, COL_BORDER_HI,
                                 (cx_track, track_y - 3), (cx_track, track_y + track_h + 3), 1)
            else:
                pygame.draw.rect(self.screen, draw_col,
                                 (track_x, track_y, max(2, int(frac * track_w)), track_h), border_radius=3)

            self._text(str(val), self.font_mono_sm, draw_col, (colx + col_w - 12, cy + 12), align="right")

    def _draw_motors_card(self, card: pygame.Rect) -> None:
        testable = self._fc_disarmed()
        if self.settings.is_device:
            title = "Motor Test (MSP)" if testable else "Motor / MSP Output"
        else:
            title = "Motor / PWM Output"
        self._panel(card, title)
        self._motor_bar_layout = []

        if self.settings.is_device:
            msp = self.msp.snapshot()
            if not msp.enabled:
                status, scol = "DISABLED", COL_TEXT_FAINT
            elif msp.error and not msp.connected:
                status, scol = "WS ERROR", COL_BAD
            elif self.motor_test and testable:
                status, scol = "MOTOR TEST", COL_WARN
            elif msp.alive(MspDeviceLink.RX_TIMEOUT):
                status, scol = "RECEIVING", COL_GOOD
            else:
                status, scol = "WAITING", COL_WARN
            header = f"MSP   {status}"
            mc = self._motor_count()
            if self.motor_test and testable:
                values_draw = [float(v) for v in self.motor_override[:mc]]
            else:
                values_draw = list(msp.motors[:mc])
            # Servos are display-only (BF has no live MSP servo set).
            if not (self.motor_test and testable):
                for v in msp.servos:
                    if len(values_draw) >= 16:
                        break
                    if v > 0:
                        values_draw.append(v)
            if not values_draw:
                values_draw = [0.0] * mc
            total = max(1, min(len(values_draw), 16))
            motor_slots = min(mc, total)
        else:
            rx = self.pwm_in.snapshot()
            if not rx.enabled:
                status, scol = "DISABLED", COL_TEXT_FAINT
            elif rx.error:
                status, scol = "BIND ERROR", COL_BAD
            elif rx.connected(PWMReceiver.RX_TIMEOUT):
                status, scol = "RECEIVING", COL_GOOD
            else:
                status, scol = "WAITING", COL_WARN
            header = f"IN :{self.settings.pwm_in_port}   {status}"
            mc = rx.motor_count if rx.motor_count > 0 else 4
            total = mc
            for i in range(mc, 16):
                if rx.values[i] > 0:
                    total = i + 1
            total = max(1, min(total, 16))
            values_draw = rx.values
            motor_slots = mc

        hx = self._text(header, self.font_h, scol, (card.right - 16, card.y + 13), align="right").x
        _fill_circle(self.screen, hx - 10, card.y + 18, 4, scol)
        if self.settings.is_device and testable:
            self._text("drag M bars  M=idle", self.font_small, COL_TEXT_FAINT,
                       (card.x + 16, card.y + 13))

        area_top = card.y + 42
        area_bottom = card.bottom - 30
        bar_h = area_bottom - area_top
        slot = (card.w - 32) / total
        bar_w = int(min(46, slot - 14))
        base_x = card.x + 16
        mouse = pygame.mouse.get_pos()

        for i in range(total):
            cx = base_x + slot * i + slot / 2
            is_motor = i < motor_slots
            v = values_draw[i] if i < len(values_draw) else 0.0
            frac = _clamp((v - 1000) / 1000.0, 0.0, 1.0)
            bx = int(cx - bar_w / 2)
            bar_rect = pygame.Rect(bx, area_top, bar_w, bar_h)
            if is_motor and testable:
                self._motor_bar_layout.append({
                    "i": i, "rect": bar_rect, "top": area_top, "height": bar_h,
                })
                if bar_rect.collidepoint(mouse) or (
                    isinstance(self.dragging, tuple)
                    and self.dragging[0] == "motor"
                    and self.dragging[1] == i
                ):
                    pygame.draw.rect(self.screen, COL_PANEL_LIGHT, bar_rect.inflate(4, 4), border_radius=5)

            pygame.draw.rect(self.screen, COL_TRACK, bar_rect, border_radius=4)
            fh = int(bar_h * frac)
            if fh > 0:
                fill_col = COL_WARN if (is_motor and self.motor_test and testable) else (
                    COL_ACCENT if is_motor else COL_AUX
                )
                pygame.draw.rect(self.screen, fill_col, (bx, area_top + bar_h - fh, bar_w, fh), border_radius=4)
            midy = area_top + bar_h // 2
            pygame.draw.line(self.screen, COL_BORDER_HI, (bx, midy), (bx + bar_w, midy), 1)

            label = f"M{i + 1}" if is_motor else f"S{i - motor_slots + 1}"
            self._text(label, self.font_small, COL_TEXT_DIM, (cx, card.bottom - 27), align="center")
            vtxt = str(int(round(v))) if v > 0 else "----"
            vcol = COL_TEXT if v > 0 else COL_TEXT_FAINT
            self._text(vtxt, self.font_mono_sm, vcol, (cx, card.bottom - 15), align="center")

    def _draw_legend(self) -> None:
        items = [
            ("W/S", "throttle"), ("A/D", "yaw"), ("Arrows", "pitch/roll"),
            ("Drag", "sticks"), ("Enter", "arm"), ("1-0", "aux"),
            ("Click", "aux CH"), ("M", "motors idle"), ("R", "reset"),
            ("Tab", "settings"), ("Esc", "quit"),
        ]
        y = WINDOW_H - 26
        x = 14
        for key, desc in items:
            kr = self.font_mono_sm.render(key, True, COL_ACCENT)
            pad = 5
            box = pygame.Rect(x, y, kr.get_width() + pad * 2, 18)
            pygame.draw.rect(self.screen, COL_PANEL_LIGHT, box, border_radius=5)
            pygame.draw.rect(self.screen, COL_BORDER, box, width=1, border_radius=5)
            self.screen.blit(kr, (x + pad, y + 2))
            x += box.width + 5
            dr = self._text(desc, self.font_small, COL_TEXT_DIM, (x, y + 2))
            x += dr.width + 14

    # ------------------------------------------------------------------
    def _draw_panel(self) -> None:
        overlay = pygame.Surface((WINDOW_W, WINDOW_H), pygame.SRCALPHA)
        overlay.fill((0, 0, 0, 190))
        self.screen.blit(overlay, (0, 0))

        pw, ph = 560, 604
        px = (WINDOW_W - pw) // 2
        py = (WINDOW_H - ph) // 2
        pygame.draw.rect(self.screen, COL_PANEL, (px, py, pw, ph), border_radius=12)
        pygame.draw.rect(self.screen, COL_BORDER_HI, (px, py, pw, ph), 1, border_radius=12)
        pygame.draw.rect(self.screen, COL_ACCENT, (px, py, pw, 3), border_top_left_radius=12, border_top_right_radius=12)

        self._text("SETTINGS", self.font_title, COL_TEXT, (px + 18, py + 12))
        hint_lines = [
            "Up/Down select   Left/Right adjust (Shift=coarse)   Enter edit",
            "Ctrl+S save   D defaults   Tab/Esc close",
        ]
        hy = py + 40
        for line in hint_lines:
            self.screen.blit(self.font_small.render(line, True, COL_TEXT_DIM), (px + 18, hy))
            hy += 15

        y = py + 76
        row_h = 20
        panel = self.panel
        for i, f in enumerate(panel.fields):
            if f.kind == "header":
                self.screen.blit(self.font_h.render(f.label, True, COL_ACCENT), (px + 20, y + 4))
                y += row_h
                continue
            selected = i == panel.selected
            if selected:
                pygame.draw.rect(
                    self.screen, COL_PANEL_LIGHT, (px + 12, y, pw - 24, row_h), border_radius=5
                )
            label_col = COL_TEXT if selected else COL_TEXT_DIM
            self.screen.blit(self.font.render(f.label, True, label_col), (px + 28, y + 2))

            if panel.editing and selected:
                value_str = panel.edit_buffer + "_"
                val_col = COL_WARN
            else:
                value_str = self._fmt_value(f)
                val_col = COL_ACCENT if selected else COL_TEXT
            vs = self.font_mono.render(value_str, True, val_col)
            self.screen.blit(vs, (px + pw - 30 - vs.get_width(), y + 2))
            y += row_h

        if panel.status and time.time() - panel.status_time < 3.0:
            pygame.draw.line(self.screen, COL_BORDER, (px + 16, py + ph - 34), (px + pw - 16, py + ph - 34), 1)
            self.screen.blit(
                self.font.render(panel.status, True, COL_GOOD), (px + 20, py + ph - 26)
            )

    def _fmt_value(self, f: Field) -> str:
        val = getattr(self.settings, f.attr)
        if f.kind == "bool":
            return "ON" if val else "OFF"
        if f.kind == "float":
            return f"{val:.3g}"
        if f.kind == "choice":
            return str(val).upper()
        return str(val)


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------


def _dist(a, b) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def _pct(x: float) -> str:
    """Format a [-1, 1] axis as a signed percentage."""
    v = round(_clamp(x, -1, 1) * 100)
    return f"{v:+d}%"


def _sysfont(names, size: int, bold: bool = False):
    """Pick the first available system font from `names`, falling back safely."""
    return pygame.font.SysFont(",".join(names), size, bold=bold)


def _make_gradient(w: int, h: int, top, bottom) -> pygame.Surface:
    surf = pygame.Surface((w, h))
    for y in range(h):
        t = y / max(1, h - 1)
        color = (
            round(top[0] + (bottom[0] - top[0]) * t),
            round(top[1] + (bottom[1] - top[1]) * t),
            round(top[2] + (bottom[2] - top[2]) * t),
        )
        pygame.draw.line(surf, color, (0, y), (w, y))
    return surf


def _fill_circle(surface, x, y, r, color) -> None:
    """Anti-aliased filled circle."""
    x, y, r = int(x), int(y), int(r)
    if r <= 0:
        return
    gfxdraw.filled_circle(surface, x, y, r, color)
    gfxdraw.aacircle(surface, x, y, r, color)


def _normalize_in_circle(pos, center, r) -> tuple:
    dx = (pos[0] - center[0]) / r
    dy = (pos[1] - center[1]) / r
    mag = math.hypot(dx, dy)
    if mag > 1.0:
        dx /= mag
        dy /= mag
    return _clamp(dx, -1, 1), _clamp(dy, -1, 1)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------


def app_dir() -> str:
    """Directory for config.json: next to the .exe when frozen, else script dir."""
    if getattr(sys, "frozen", False):
        return os.path.dirname(os.path.abspath(sys.executable))
    return os.path.dirname(os.path.abspath(__file__))


def parse_args(argv) -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Betaflight joystick emulator (SITL / DEVICE)")
    p.add_argument("--mode", choices=list(MODES), help="sitl (UDP) or device (MSP over WebSocket)")
    p.add_argument("--ip", help="SITL target IP for RC input (overrides config)")
    p.add_argument("--port", type=int, help="SITL target UDP port (default 9004)")
    p.add_argument("--hz", type=int, help="RC send rate in Hz")
    p.add_argument("--msp-url", help="DEVICE MSP WebSocket URL (default ws://127.0.0.1:5761)")
    p.add_argument(
        "--config",
        default=os.path.join(app_dir(), CONFIG_FILENAME),
        help="path to config.json",
    )
    return p.parse_args(argv)


def main(argv=None) -> int:
    args = parse_args(argv if argv is not None else sys.argv[1:])
    settings = Settings.load(args.config)
    if args.mode:
        settings.mode = args.mode
    if args.ip:
        settings.ip = args.ip
    if args.port:
        settings.port = args.port
    if args.hz:
        settings.send_hz = args.hz
    if args.msp_url:
        settings.msp_ws_url = args.msp_url
    settings.clamp_all()

    app = App(settings, args.config)
    try:
        app.run()
    except KeyboardInterrupt:
        pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
