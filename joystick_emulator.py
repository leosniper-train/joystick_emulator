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

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

CONFIG_FILENAME = "config.json"
NUM_PACKET_CHANNELS = 16  # Betaflight SITL always reads a fixed 16-channel struct.

WINDOW_W, WINDOW_H = 940, 640
FPS_RENDER_CAP = 120  # Rendering may run faster than the RC send rate.

# Colors
COL_BG = (18, 20, 26)
COL_PANEL = (28, 31, 40)
COL_PANEL_LIGHT = (38, 42, 54)
COL_ACCENT = (64, 156, 255)
COL_TEXT = (222, 226, 235)
COL_TEXT_DIM = (140, 146, 160)
COL_GIMBAL = (44, 48, 60)
COL_GIMBAL_RING = (70, 76, 92)
COL_KNOB = (64, 156, 255)
COL_KNOB_GRAB = (120, 190, 255)
COL_GOOD = (72, 199, 116)
COL_BAD = (224, 82, 82)
COL_WARN = (240, 180, 60)
COL_BAR_BG = (46, 50, 62)
COL_BAR_FILL = (64, 156, 255)


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
        self.font = pygame.font.SysFont("consolas", 16)
        self.font_small = pygame.font.SysFont("consolas", 13)
        self.font_big = pygame.font.SysFont("consolas", 20, bold=True)

        # Gimbal geometry
        self.gimbal_r = 120
        self.left_center = (250, 250)
        self.right_center = (WINDOW_W - 250, 250)

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
        self.screen.fill(COL_BG)
        self._draw_header()
        self._draw_gimbal(self.left_center, self.state.yaw, 1.0 - self.state.throttle * 2, "THROTTLE / YAW")
        self._draw_gimbal(self.right_center, self.state.roll, -self.state.pitch, "PITCH / ROLL")
        self._draw_channels()
        self._draw_switches()
        self._draw_legend()
        if self.panel_open:
            self._draw_panel()
        pygame.display.flip()

    def _draw_header(self) -> None:
        target = f"{self.settings.ip}:{self.settings.port}"
        txt = self.font_big.render("Betaflight SITL Joystick Emulator", True, COL_TEXT)
        self.screen.blit(txt, (20, 14))
        info = f"UDP -> {target}   {self.settings.send_hz} Hz   sent {self.sender.packets_sent}"
        color = COL_GOOD if self.sender.last_error is None else COL_BAD
        self.screen.blit(self.font.render(info, True, color), (20, 44))
        if self.sender.last_error:
            self.screen.blit(
                self.font_small.render(f"send error: {self.sender.last_error}", True, COL_BAD),
                (20, 64),
            )

    def _draw_gimbal(self, center, nx: float, ny: float, label: str) -> None:
        cx, cy = center
        r = self.gimbal_r
        pygame.draw.circle(self.screen, COL_GIMBAL, center, r)
        pygame.draw.circle(self.screen, COL_GIMBAL_RING, center, r, 2)
        pygame.draw.line(self.screen, COL_GIMBAL_RING, (cx - r, cy), (cx + r, cy), 1)
        pygame.draw.line(self.screen, COL_GIMBAL_RING, (cx, cy - r), (cx, cy + r), 1)
        kx = cx + _clamp(nx, -1, 1) * r
        ky = cy + _clamp(ny, -1, 1) * r
        grabbed = (self.dragging == "left" and label.startswith("THROTTLE")) or (
            self.dragging == "right" and label.startswith("PITCH")
        )
        pygame.draw.circle(self.screen, COL_KNOB_GRAB if grabbed else COL_KNOB, (int(kx), int(ky)), 16)
        pygame.draw.circle(self.screen, COL_BG, (int(kx), int(ky)), 16, 2)
        lbl = self.font_small.render(label, True, COL_TEXT_DIM)
        self.screen.blit(lbl, (cx - lbl.get_width() // 2, cy + r + 10))

    def _channel_labels(self) -> list:
        base = {"A": "Roll", "E": "Pitch", "T": "Thr", "R": "Yaw"}
        labels = []
        for i in range(self.settings.channel_count):
            if i < 4:
                letter = self.settings.channel_order[i] if i < len(self.settings.channel_order) else "?"
                labels.append(f"CH{i+1} {base.get(letter, '?')}")
            elif i == 4:
                labels.append(f"CH{i+1} Arm")
            elif i in (5, 6, 7):
                labels.append(f"CH{i+1} Aux{i-4}")
            else:
                labels.append(f"CH{i+1}")
        return labels

    def _draw_channels(self) -> None:
        x0, y0 = 20, 420
        w = WINDOW_W - 40
        title = self.font.render("Channels", True, COL_TEXT)
        self.screen.blit(title, (x0, y0 - 26))
        chans = self.state.to_channels(self.settings)
        labels = self._channel_labels()
        cols = 4
        col_w = w // cols
        row_h = 34
        for i, label in enumerate(labels):
            col = i % cols
            row = i // cols
            bx = x0 + col * col_w
            by = y0 + row * row_h
            self.screen.blit(self.font_small.render(label, True, COL_TEXT_DIM), (bx, by))
            bar_x = bx + 78
            bar_w = col_w - 130
            bar_h = 12
            bar_y = by + 2
            pygame.draw.rect(self.screen, COL_BAR_BG, (bar_x, bar_y, bar_w, bar_h), border_radius=3)
            frac = (chans[i] - 1000) / 1000.0
            pygame.draw.rect(
                self.screen,
                COL_BAR_FILL,
                (bar_x, bar_y, int(bar_w * _clamp(frac, 0, 1)), bar_h),
                border_radius=3,
            )
            self.screen.blit(
                self.font_small.render(str(chans[i]), True, COL_TEXT),
                (bar_x + bar_w + 6, by),
            )

    def _draw_switches(self) -> None:
        x0 = 20
        y0 = 330
        chips = [("ARM", self.state.armed, COL_BAD)]
        for i, on in enumerate(self.state.aux):
            chips.append((f"AUX{i+1}", on, COL_ACCENT))
        cx = x0
        for name, on, on_color in chips:
            color = COL_GOOD if (name == "ARM" and on) else (on_color if on else COL_PANEL_LIGHT)
            label = f"{name}: {'ON' if on else 'OFF'}"
            surf = self.font_small.render(label, True, COL_TEXT if on else COL_TEXT_DIM)
            pad = 10
            rect = pygame.Rect(cx, y0, surf.get_width() + pad * 2, 26)
            pygame.draw.rect(self.screen, color, rect, border_radius=6)
            self.screen.blit(surf, (cx + pad, y0 + 5))
            cx += rect.width + 10

    def _draw_legend(self) -> None:
        lines = [
            "Left stick (W/S throttle, A/D yaw)   Right stick (Arrows pitch/roll)   Mouse: drag either stick",
            "Enter: ARM/DISARM    1/2/3: AUX1-3    R: reset/failsafe    Tab: settings    Esc: quit",
        ]
        y = WINDOW_H - 44
        for line in lines:
            self.screen.blit(self.font_small.render(line, True, COL_TEXT_DIM), (20, y))
            y += 18

    # ------------------------------------------------------------------
    def _draw_panel(self) -> None:
        overlay = pygame.Surface((WINDOW_W, WINDOW_H), pygame.SRCALPHA)
        overlay.fill((0, 0, 0, 180))
        self.screen.blit(overlay, (0, 0))

        pw, ph = 560, 560
        px = (WINDOW_W - pw) // 2
        py = (WINDOW_H - ph) // 2
        pygame.draw.rect(self.screen, COL_PANEL, (px, py, pw, ph), border_radius=10)
        pygame.draw.rect(self.screen, COL_ACCENT, (px, py, pw, ph), 2, border_radius=10)

        title = self.font_big.render("Settings", True, COL_TEXT)
        self.screen.blit(title, (px + 20, py + 14))
        hint_lines = [
            "Up/Down select   Left/Right adjust (Shift=coarse)   Enter edit",
            "Ctrl+S save   D defaults   Tab/Esc close",
        ]
        hy = py + 44
        for line in hint_lines:
            self.screen.blit(self.font_small.render(line, True, COL_TEXT_DIM), (px + 20, hy))
            hy += 16

        y = py + 82
        row_h = 21
        panel = self.panel
        for i, f in enumerate(panel.fields):
            if f.kind == "header":
                self.screen.blit(self.font_small.render(f.label, True, COL_ACCENT), (px + 20, y + 3))
                y += row_h
                continue
            selected = i == panel.selected
            if selected:
                pygame.draw.rect(
                    self.screen, COL_PANEL_LIGHT, (px + 12, y, pw - 24, row_h), border_radius=4
                )
            label_col = COL_TEXT if selected else COL_TEXT_DIM
            self.screen.blit(self.font_small.render(f.label, True, label_col), (px + 28, y + 3))

            if panel.editing and selected:
                value_str = panel.edit_buffer + "_"
                val_col = COL_WARN
            else:
                value_str = self._fmt_value(f)
                val_col = COL_TEXT
            vs = self.font_small.render(value_str, True, val_col)
            self.screen.blit(vs, (px + pw - 30 - vs.get_width(), y + 3))
            y += row_h

        if panel.status and time.time() - panel.status_time < 3.0:
            self.screen.blit(
                self.font_small.render(panel.status, True, COL_GOOD), (px + 20, py + ph - 26)
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
