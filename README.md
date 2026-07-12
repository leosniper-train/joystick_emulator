# Betaflight SITL Joystick Emulator

A virtual RC transmitter for [Betaflight SITL](https://betaflight.com/docs/development/SITL).
It sends RC channel packets to the SITL over UDP and can be flown with **mouse**
(drag the on-screen sticks) or **keyboard** (WASD + arrow keys). All parameters
(target IP, send rate, PWM range, stick feel, channel mapping, ...) are editable
in an in-app settings panel and persist to `config.json`.

![Two virtual gimbals with channel bars](docs/screenshot.png)

## How it works

Betaflight SITL listens for RC input on **UDP port 9004**. Each packet is a
little-endian C struct:

```c
typedef struct {
    double   timestamp;      // seconds
    uint16_t channels[16];   // RC values, 1000-2000
} rc_packet;
```

This emulator packs `struct.pack('<d', time.time())` followed by 16 `uint16_t`
channel values and sends it at the configured rate (default 50 Hz). Channels use
the standard **AETR** mapping:

| Channel | Function        | Source                 |
|---------|-----------------|------------------------|
| CH1     | Roll (Aileron)  | Right stick X          |
| CH2     | Pitch (Elevator)| Right stick Y          |
| CH3     | Throttle        | Left stick Y (sticky)  |
| CH4     | Yaw (Rudder)    | Left stick X           |
| CH5     | ARM             | `Enter` toggle         |
| CH6-CH8 | AUX 1-3 (modes) | `1` / `2` / `3` toggles |

## Requirements

- Python 3.9+
- [pygame](https://www.pygame.org/) (see `requirements.txt`)

## Install

```bash
pip install -r requirements.txt
```

On Windows you may need to use the Python launcher:

```bash
py -m pip install -r requirements.txt
```

## Run

```bash
python joystick_emulator.py
```

Optional command-line overrides (they take precedence over `config.json` for
that run only):

```bash
python joystick_emulator.py --ip 127.0.0.1 --port 9004 --hz 50
```

| Flag       | Description                          | Default        |
|------------|--------------------------------------|----------------|
| `--ip`     | Target IP of the SITL RC input       | `127.0.0.1`    |
| `--port`   | Target UDP port                      | `9004`         |
| `--hz`     | Send rate in Hz                      | `50`           |
| `--config` | Path to the settings file            | `./config.json`|

## Controls

### Sticks

| Input                     | Action                              |
|---------------------------|-------------------------------------|
| `W` / `S`                 | Throttle up / down (holds position) |
| `A` / `D`                 | Yaw left / right (self-centers)     |
| Arrow `Up` / `Down`       | Pitch forward / back (self-centers) |
| Arrow `Left` / `Right`    | Roll left / right (self-centers)    |
| Mouse drag on a gimbal    | Move that stick directly            |

Roll, pitch and yaw spring back to center when released. Throttle stays where
you leave it (configurable via **Sticky throttle**).

### Switches / commands

| Key     | Action                                        |
|---------|-----------------------------------------------|
| `Enter` | Toggle ARM (CH5)                              |
| `1`/`2`/`3` | Toggle AUX 1-3 (CH6-CH8)                  |
| `R`     | Reset / failsafe (center sticks, cut throttle, disarm) |
| `Tab`   | Open / close the settings panel               |
| `Esc`   | Quit                                          |

### Arming note

Betaflight will refuse to arm unless the throttle is low. Keep throttle at the
bottom, press `Enter` to arm (CH5 high), then raise throttle. Use `R` at any
time as a panic/failsafe that disarms and cuts throttle.

## Settings panel

Press `Tab` to open the panel. It is grouped into Core, PWM range, Stick feel,
Throttle and Channels.

| Key                 | Action                                    |
|---------------------|-------------------------------------------|
| `Up` / `Down`       | Select a field                            |
| `Left` / `Right`    | Adjust value (hold `Shift` for coarse step) |
| `Enter`             | Type-edit a value (numbers / IP / order); `Enter` confirm, `Esc` cancel |
| `Ctrl`+`S`          | Save to `config.json`                     |
| `D`                 | Reset all settings to defaults            |
| `Tab` / `Esc`       | Close the panel                           |

Changes apply live: editing **Target IP** / **Port** retargets the UDP socket,
**Send rate** changes the loop rate, and PWM / feel / channel changes take
effect on the next frame.

### Parameters

- **Core**: `Target IP`, `Port`, `Send rate (Hz)`
- **PWM range**: `PWM min` / `PWM mid` / `PWM max` (mid may be asymmetric)
- **Stick feel**: `Return speed` (recenter rate), `Key step rate` (keyboard axis
  speed), `Deadband`, `Expo`
- **Throttle**: `Sticky throttle` (hold vs. auto-decay), `Arm-safe throttle PWM`
  (value applied on reset)
- **Channels**: `Channel order` (permutation of `AETR`), `Active channels`,
  `AUX low PWM` / `AUX high PWM`

## Configuration file

Settings are stored in `config.json` next to the script (created on first run).
It is per-machine and git-ignored, so each environment keeps its own values.
Delete it to return to defaults.

## Using with Betaflight SITL

1. Build and start the SITL: `./obj/main/betaflight_SITL.elf` (it prints
   `start UDP server for RC input @9004`).
2. Run this emulator (defaults target `127.0.0.1:9004`). If the SITL runs on
   another host/VM (e.g. WSL), set **Target IP** accordingly.
3. In the Betaflight Configurator, map CH5/CH6 to Arm / flight-mode switches so
   the ARM and AUX toggles here do something useful.

The UDP socket is fire-and-forget (non-blocking), so the emulator runs fine
whether or not the SITL is currently listening.
