"""Betaflight SITL Joystick Emulator.

A virtual RC transmitter that sends RC channel packets to a Betaflight SITL
instance over UDP. Controllable with both mouse (drag the on-screen sticks) and
keyboard (WASD + arrow keys + mode keys).

Protocol (Betaflight SITL, UDP RC input on port 9004):
    struct { double timestamp; uint16_t channels[16]; }  little-endian
Channels use the 1000-2000 PWM range with AETR mapping by default.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import socket
import struct
import sys
import time
from dataclasses import asdict, dataclass, field, fields
from typing import Any, Callable, Optional

import pygame
from pygame import gfxdraw

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

CONFIG_FILENAME = "config.json"
NUM_PACKET_CHANNELS = 16  # Betaflight SITL always reads a fixed 16-channel struct.

WINDOW_W, WINDOW_H = 1060, 720
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
    ip: str = "127.0.0.1"
    port: int = 9004
    send_hz: int = 50

    # PWM range
    pwm_min: int = 1000
    pwm_mid: int = 1500
    pwm_max: int = 2000

    # Stick feel
    return_speed: float = 5.0  # normalized units/sec when recentering
    key_step: float = 2.5      # normalized units/sec when a key is held
    deadband: float = 0.03     # ignore tiny stick offsets (0..0.4)
    expo: float = 0.0          # 0 = linear, 1 = full cubic softening

    # Throttle
    throttle_sticky: bool = True   # throttle holds its position when released
    throttle_arm_safe: int = 1000  # throttle PWM applied on reset/failsafe

    # Channels
    channel_order: str = "AETR"    # order of the 4 main axes on CH1-4
    channel_count: int = 16        # active channels (packet is always 16)
    aux_low: int = 1000            # switch "off" PWM value
    aux_high: int = 2000           # switch "on" PWM value

    # ------------------------------------------------------------------
    def clamp_all(self) -> None:
        """Validate/clamp values into sane ranges."""
        self.port = int(_clamp(self.port, 1, 65535))
        self.send_hz = int(_clamp(self.send_hz, 1, 250))
        self.pwm_min = int(_clamp(self.pwm_min, 500, 2500))
        self.pwm_max = int(_clamp(self.pwm_max, 500, 2500))
        if self.pwm_max < self.pwm_min:
            self.pwm_min, self.pwm_max = self.pwm_max, self.pwm_min
        self.pwm_mid = int(_clamp(self.pwm_mid, self.pwm_min, self.pwm_max))
        self.return_speed = float(_clamp(self.return_speed, 0.2, 50.0))
        self.key_step = float(_clamp(self.key_step, 0.2, 50.0))
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
        if not os.path.isfile(path):
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

    armed: bool = False           # CH5
    aux: list = field(default_factory=lambda: [False, False, False])  # CH6, CH7, CH8

    # ------------------------------------------------------------------
    def reset(self, settings: Settings) -> None:
        """Failsafe: center sticks, cut throttle to the arm-safe value, disarm."""
        self.roll = self.pitch = self.yaw = 0.0
        self.armed = False
        span = max(1, settings.pwm_max - settings.pwm_min)
        self.throttle = _clamp((settings.throttle_arm_safe - settings.pwm_min) / span, 0.0, 1.0)

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

        # Switch channels (CH5 = ARM, CH6-8 = AUX).
        switch_states = [self.armed] + list(self.aux)
        for i, on in enumerate(switch_states):
            ch = 4 + i  # CH5 -> index 4
            if ch < NUM_PACKET_CHANNELS:
                chans[ch] = settings.aux_high if on else settings.aux_low

        # Zero out inactive channels beyond channel_count to the mid value.
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
# Settings panel
# ---------------------------------------------------------------------------


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
    ) -> None:
        self.label = label
        self.attr = attr          # None => section header (not selectable)
        self.kind = kind          # "int" | "float" | "bool" | "str" | "header"
        self.step = step
        self.big_step = big_step
        self.vmin = vmin
        self.vmax = vmax
        self.fmt = fmt

    @property
    def selectable(self) -> bool:
        return self.attr is not None and self.kind != "header"


def build_fields() -> list:
    return [
        Field("CORE", None, "header"),
        Field("Target IP", "ip", "str"),
        Field("Port", "port", "int", 1, 10, 1, 65535),
        Field("Send rate (Hz)", "send_hz", "int", 1, 10, 1, 250),
        Field("PWM RANGE", None, "header"),
        Field("PWM min", "pwm_min", "int", 5, 50, 500, 2500),
        Field("PWM mid", "pwm_mid", "int", 5, 50, 500, 2500),
        Field("PWM max", "pwm_max", "int", 5, 50, 500, 2500),
        Field("STICK FEEL", None, "header"),
        Field("Return speed", "return_speed", "float", 0.1, 1.0, 0.2, 50.0),
        Field("Key step rate", "key_step", "float", 0.1, 1.0, 0.2, 50.0),
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


class SettingsPanel:
    """Overlay UI for viewing/editing settings with live-apply."""

    def __init__(self, settings: Settings, config_path: str) -> None:
        self.settings = settings
        self.config_path = config_path
        self.fields = build_fields()
        self.selected = self._first_selectable()
        self.editing = False
        self.edit_buffer = ""
        self.status = ""
        self.status_time = 0.0
        self.on_change: Optional[Callable[[], None]] = None

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
        else:
            return
        self._apply()

    def _apply(self) -> None:
        self.settings.clamp_all()
        if self.on_change:
            self.on_change()

    def _commit_edit(self, field_obj: Field) -> None:
        raw = self.edit_buffer.strip()
        try:
            if field_obj.kind == "int":
                setattr(self.settings, field_obj.attr, int(float(raw)))
            elif field_obj.kind == "float":
                setattr(self.settings, field_obj.attr, float(raw))
            elif field_obj.kind == "str":
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

        if event.key in (pygame.K_UP, pygame.K_w):
            self._move(-1)
        elif event.key in (pygame.K_DOWN, pygame.K_s):
            self._move(1)
        elif event.key in (pygame.K_LEFT, pygame.K_a):
            step = field_obj.big_step if coarse else field_obj.step
            self._adjust(field_obj, -step)
        elif event.key in (pygame.K_RIGHT, pygame.K_d):
            step = field_obj.big_step if coarse else field_obj.step
            self._adjust(field_obj, step)
        elif event.key == pygame.K_RETURN:
            if field_obj.kind == "bool":
                self._adjust(field_obj, 0)
            else:
                self.editing = True
                self.edit_buffer = str(getattr(self.settings, field_obj.attr))
        elif event.key == pygame.K_s and (mods & pygame.KMOD_CTRL):
            self._save()

    def handle_command_key(self, key: int) -> None:
        """Keys handled while the panel is open but not editing (S/D)."""
        if self.editing:
            return
        if key == pygame.K_d:
            self._reset_defaults()

    def _save(self) -> None:
        ok = self.settings.save(self.config_path)
        self._set_status("Saved to config.json" if ok else "Save failed")

    def _reset_defaults(self) -> None:
        defaults = Settings()
        for f in fields(Settings):
            setattr(self.settings, f.name, getattr(defaults, f.name))
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

        self.panel = SettingsPanel(settings, config_path)
        self.panel.on_change = self._on_settings_changed
        self.panel_open = False

        self.running = True
        self.dragging: Optional[str] = None  # "left" | "right" | None
        self._send_accumulator = 0.0

        pygame.init()
        pygame.display.set_caption("Betaflight SITL Joystick Emulator")
        self.screen = pygame.display.set_mode((WINDOW_W, WINDOW_H))
        self.clock = pygame.time.Clock()

        # Typography: UI font for labels, mono font for numeric readouts.
        self.font_title = _sysfont(["Segoe UI Semibold", "Segoe UI", "Arial"], 22, bold=True)
        self.font_h = _sysfont(["Segoe UI", "Arial"], 12, bold=True)
        self.font = _sysfont(["Segoe UI", "Arial"], 15)
        self.font_small = _sysfont(["Segoe UI", "Arial"], 12)
        self.font_mono = _sysfont(["Consolas", "Courier New"], 15)
        self.font_mono_sm = _sysfont(["Consolas", "Courier New"], 12)
        self.font_arm = _sysfont(["Segoe UI Black", "Segoe UI", "Arial"], 26, bold=True)

        # Cached vertical-gradient background.
        self.bg = _make_gradient(WINDOW_W, WINDOW_H, COL_BG_TOP, COL_BG_BOT)

        # ---- Layout geometry ----
        margin = 22
        top = 66
        card_w = 372
        card_h = 300
        self.left_card = pygame.Rect(margin, top, card_w, card_h)
        self.right_card = pygame.Rect(WINDOW_W - margin - card_w, top, card_w, card_h)
        self.center_card = pygame.Rect(
            self.left_card.right + 16,
            top,
            self.right_card.left - self.left_card.right - 32,
            card_h,
        )
        self.chan_card = pygame.Rect(margin, top + card_h + 16, WINDOW_W - 2 * margin, 262)

        self.gimbal_r = 108
        gy = self.left_card.y + 128
        self.left_center = (self.left_card.centerx, gy)
        self.right_center = (self.right_card.centerx, gy)

    # ------------------------------------------------------------------
    def _on_settings_changed(self) -> None:
        self.sender.retarget(self.settings.ip, self.settings.port)

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
        self.sender.close()
        pygame.quit()

    # ------------------------------------------------------------------
    # Events
    # ------------------------------------------------------------------
    def _handle_events(self) -> None:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.running = False
                return

            if self.panel_open:
                if event.type == pygame.KEYDOWN and event.key in (
                    pygame.K_TAB,
                    pygame.K_ESCAPE,
                ):
                    self.panel_open = False
                    continue
                if event.type == pygame.KEYDOWN and event.key == pygame.K_d and not self.panel.editing:
                    self.panel.handle_command_key(event.key)
                    continue
                self.panel.handle_event(event)
                continue

            if event.type == pygame.KEYDOWN:
                self._handle_flight_keydown(event)
            elif event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
                self._handle_mouse_down(event.pos)
            elif event.type == pygame.MOUSEBUTTONUP and event.button == 1:
                self.dragging = None

    def _handle_flight_keydown(self, event: pygame.event.Event) -> None:
        if event.key == pygame.K_ESCAPE:
            self.running = False
        elif event.key == pygame.K_TAB:
            self.panel_open = True
        elif event.key == pygame.K_RETURN:
            self.state.armed = not self.state.armed
        elif event.key == pygame.K_r:
            self.state.reset(self.settings)
        elif event.key == pygame.K_1:
            self.state.aux[0] = not self.state.aux[0]
        elif event.key == pygame.K_2:
            self.state.aux[1] = not self.state.aux[1]
        elif event.key == pygame.K_3:
            self.state.aux[2] = not self.state.aux[2]

    def _handle_mouse_down(self, pos) -> None:
        if _dist(pos, self.left_center) <= self.gimbal_r:
            self.dragging = "left"
            self._drag_stick(pos)
        elif _dist(pos, self.right_center) <= self.gimbal_r:
            self.dragging = "right"
            self._drag_stick(pos)

    def _drag_stick(self, pos) -> None:
        if self.dragging == "left":
            nx, ny = _normalize_in_circle(pos, self.left_center, self.gimbal_r)
            self.state.yaw = nx
            self.state.throttle = _clamp((-ny + 1.0) / 2.0, 0.0, 1.0)
        elif self.dragging == "right":
            nx, ny = _normalize_in_circle(pos, self.right_center, self.gimbal_r)
            self.state.roll = nx
            self.state.pitch = -ny

    # ------------------------------------------------------------------
    # Continuous input update
    # ------------------------------------------------------------------
    def _update_inputs(self, dt: float) -> None:
        if self.dragging is not None:
            self._drag_stick(pygame.mouse.get_pos())
            # While dragging one stick, the other still self-centers.
            if self.dragging == "left":
                self._center_axis("roll", dt)
                self._center_axis("pitch", dt)
            else:
                self._center_axis("yaw", dt)
                if not self.settings.throttle_sticky:
                    self._decay_throttle(dt)
            return

        keys = pygame.key.get_pressed()
        step = self.settings.key_step * dt

        # Right stick: arrows -> roll / pitch
        roll_in = (1 if keys[pygame.K_RIGHT] else 0) - (1 if keys[pygame.K_LEFT] else 0)
        pitch_in = (1 if keys[pygame.K_UP] else 0) - (1 if keys[pygame.K_DOWN] else 0)
        self._drive_axis("roll", roll_in, step, dt)
        self._drive_axis("pitch", pitch_in, step, dt)

        # Left stick: A/D -> yaw, W/S -> throttle (sticky)
        yaw_in = (1 if keys[pygame.K_d] else 0) - (1 if keys[pygame.K_a] else 0)
        self._drive_axis("yaw", yaw_in, step, dt)

        thr_in = (1 if keys[pygame.K_w] else 0) - (1 if keys[pygame.K_s] else 0)
        if thr_in != 0:
            self.state.throttle = _clamp(self.state.throttle + thr_in * step, 0.0, 1.0)
        elif not self.settings.throttle_sticky:
            self._decay_throttle(dt)

    def _drive_axis(self, attr: str, direction: int, step: float, dt: float) -> None:
        cur = getattr(self, "state").__dict__[attr]
        if direction != 0:
            cur = _clamp(cur + direction * step, -1.0, 1.0)
            setattr(self.state, attr, cur)
        else:
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
        interval = 1.0 / max(1, self.settings.send_hz)
        self._send_accumulator += dt
        # Send at most a few packets per frame to catch up without flooding.
        sent = 0
        while self._send_accumulator >= interval and sent < 5:
            self.sender.send(self.state.to_channels(self.settings))
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
        bar = pygame.Rect(0, 0, WINDOW_W, 54)
        pygame.draw.rect(self.screen, COL_PANEL, bar)
        pygame.draw.line(self.screen, COL_BORDER_HI, (0, 54), (WINDOW_W, 54), 1)
        pygame.draw.rect(self.screen, COL_ACCENT, (0, 0, 4, 54))
        self._text("BETAFLIGHT SITL", self.font_title, COL_TEXT, (22, 8))
        self._text("JOYSTICK EMULATOR", self.font_h, COL_ACCENT, (24, 34))

        ok = self.sender.last_error is None
        dot_col = COL_GOOD if ok else COL_BAD
        status = "READY" if ok else "SEND ERROR"
        segments = [
            (status, dot_col, True),
            (f"{self.settings.ip}:{self.settings.port}", COL_TEXT, False),
            (f"{self.settings.send_hz} Hz", COL_TEXT_DIM, False),
            (f"{self.sender.packets_sent:,} pkts", COL_TEXT_DIM, False),
        ]
        x = WINDOW_W - 22
        rev = list(reversed(segments))
        for idx, (text, color, dot) in enumerate(rev):
            r = self.font_mono_sm.render(text, True, color)
            x -= r.get_width()
            self.screen.blit(r, (x, 20))
            if dot:
                _fill_circle(self.screen, x - 12, 26, 4, color)
                x -= 22
            if idx < len(rev) - 1:
                x -= 20
                self._text("|", self.font_mono_sm, COL_BORDER_HI, (x + 8, 20))

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
        base_y = card.bottom - 46
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
        _fill_circle(self.screen, kx, ky, 13, kcol)
        _fill_circle(self.screen, kx, ky, 5, COL_GIMBAL)

    # ---- status / switches ----
    def _draw_status_card(self, card: pygame.Rect) -> None:
        self._panel(card, "Status")
        inner_x = card.x + 16
        inner_w = card.w - 32

        armed = self.state.armed
        banner = pygame.Rect(inner_x, card.y + 46, inner_w, 60)
        if armed:
            pygame.draw.rect(self.screen, COL_GOOD, banner, border_radius=9)
            self._text("ARMED", self.font_arm, (10, 20, 15), (banner.centerx, banner.y + 14), align="center")
        else:
            pygame.draw.rect(self.screen, COL_PANEL_LIGHT, banner, border_radius=9)
            pygame.draw.rect(self.screen, COL_BAD, banner, width=2, border_radius=9)
            self._text("DISARMED", self.font_arm, COL_BAD, (banner.centerx, banner.y + 14), align="center")

        # AUX toggle rows
        ry = banner.bottom + 14
        for i, on in enumerate(self.state.aux):
            row = pygame.Rect(inner_x, ry, inner_w, 34)
            pygame.draw.rect(self.screen, COL_PANEL_LIGHT, row, border_radius=7)
            self._text(f"AUX {i + 1}", self.font, COL_TEXT_DIM, (row.x + 12, row.y + 8))
            pill = pygame.Rect(row.right - 66, row.y + 6, 54, 22)
            pygame.draw.rect(self.screen, COL_AUX if on else COL_TRACK, pill, border_radius=11)
            self._text("ON" if on else "OFF", self.font_small,
                       COL_TEXT if on else COL_TEXT_FAINT, (pill.centerx, pill.y + 4), align="center")
            ry += 40

    # ---- channels ----
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
                meta.append(("Arm", COL_GOOD, False, "arm"))
            elif i in (5, 6, 7):
                meta.append((f"Aux{i - 4}", COL_AUX, False, "aux"))
            else:
                meta.append(("", COL_MUTED, False, "other"))
        return meta

    def _draw_channels(self, card: pygame.Rect) -> None:
        self._panel(card, "Channels")
        chans = self.state.to_channels(self.settings)
        meta = self._channel_meta()
        count = self.settings.channel_count
        self._text(f"{count} ACTIVE", self.font_h, COL_TEXT_FAINT, (card.right - 16, card.y + 13), align="right")

        cols = 4
        inner_x = card.x + 18
        inner_w = card.w - 36
        col_w = inner_w // cols
        top = card.y + 48
        row_h = 50
        mid = self.settings.pwm_mid

        for i in range(count):
            name, col, bipolar, kind = meta[i]
            val = chans[i]
            row = i // cols
            colx = inner_x + (i % cols) * col_w
            cy = top + row * row_h

            active = True
            if kind == "arm":
                active = self.state.armed
            elif kind == "aux":
                active = val >= mid
            draw_col = col if active else COL_MUTED

            label = f"CH{i + 1}"
            self._text(label, self.font_mono_sm, COL_TEXT, (colx, cy))
            if name:
                lw = self.font_mono_sm.size(label + " ")[0]
                self._text(name, self.font_small, COL_TEXT_DIM, (colx + lw + 4, cy + 1))

            track_x = colx
            track_w = col_w - 66
            track_y = cy + 24
            track_h = 6
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

            vcol = draw_col if kind in ("arm", "aux") else COL_TEXT
            self._text(str(val), self.font_mono_sm, vcol, (colx + col_w - 12, cy + 12), align="right")

    def _draw_legend(self) -> None:
        items = [
            ("W/S", "throttle"), ("A/D", "yaw"), ("Arrows", "pitch/roll"),
            ("Drag", "sticks"), ("Enter", "arm"), ("1/2/3", "aux"),
            ("R", "reset"), ("Tab", "settings"), ("Esc", "quit"),
        ]
        y = WINDOW_H - 30
        x = 24
        for key, desc in items:
            kr = self.font_mono_sm.render(key, True, COL_ACCENT)
            pad = 6
            box = pygame.Rect(x, y, kr.get_width() + pad * 2, 20)
            pygame.draw.rect(self.screen, COL_PANEL_LIGHT, box, border_radius=5)
            pygame.draw.rect(self.screen, COL_BORDER, box, width=1, border_radius=5)
            self.screen.blit(kr, (x + pad, y + 3))
            x += box.width + 6
            dr = self._text(desc, self.font_small, COL_TEXT_DIM, (x, y + 3))
            x += dr.width + 20

    # ------------------------------------------------------------------
    def _draw_panel(self) -> None:
        overlay = pygame.Surface((WINDOW_W, WINDOW_H), pygame.SRCALPHA)
        overlay.fill((0, 0, 0, 190))
        self.screen.blit(overlay, (0, 0))

        pw, ph = 580, 588
        px = (WINDOW_W - pw) // 2
        py = (WINDOW_H - ph) // 2
        pygame.draw.rect(self.screen, COL_PANEL, (px, py, pw, ph), border_radius=12)
        pygame.draw.rect(self.screen, COL_BORDER_HI, (px, py, pw, ph), 1, border_radius=12)
        pygame.draw.rect(self.screen, COL_ACCENT, (px, py, pw, 3), border_top_left_radius=12, border_top_right_radius=12)

        self._text("SETTINGS", self.font_title, COL_TEXT, (px + 20, py + 14))
        hint_lines = [
            "Up/Down select   Left/Right adjust (Shift=coarse)   Enter edit",
            "Ctrl+S save   D defaults   Tab/Esc close",
        ]
        hy = py + 46
        for line in hint_lines:
            self.screen.blit(self.font_small.render(line, True, COL_TEXT_DIM), (px + 20, hy))
            hy += 16

        y = py + 84
        row_h = 22
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


def parse_args(argv) -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Betaflight SITL joystick emulator")
    p.add_argument("--ip", help="target IP for SITL RC input (overrides config)")
    p.add_argument("--port", type=int, help="target UDP port (default 9004)")
    p.add_argument("--hz", type=int, help="RC send rate in Hz")
    p.add_argument(
        "--config",
        default=os.path.join(os.path.dirname(os.path.abspath(__file__)), CONFIG_FILENAME),
        help="path to config.json",
    )
    return p.parse_args(argv)


def main(argv=None) -> int:
    args = parse_args(argv if argv is not None else sys.argv[1:])
    settings = Settings.load(args.config)
    if args.ip:
        settings.ip = args.ip
    if args.port:
        settings.port = args.port
    if args.hz:
        settings.send_hz = args.hz
    settings.clamp_all()

    app = App(settings, args.config)
    try:
        app.run()
    except KeyboardInterrupt:
        pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
