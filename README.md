# Juicer

Precise USB-controlled fluid reward dispenser for behavioral experiments. Built around an ESP32 microcontroller driving a Boxer 9QX pump. The device is commanded with newline-terminated JSON over a 2,000,000 baud serial link.

- **API**: See `api.md` for full command reference (`set` / `do` / `get`).
- **Quick test**: `test_connection.py` auto-detects the device and queries `flow_rate`.
- **Typical use**: Send JSON commands like `{"do":{"reward":0.5}}` or `{"get":["flow_rate"]}` over the serial port.
- **Python helper**: `juicer.py` provides a small, researcher-friendly wrapper with port auto-discovery and convenient methods.

## Files of interest
- `api.md` — serial API reference and CLI examples (Linux one-liners).
- `juicer.py` — importable Python helper for experiment scripts (port auto-discovery, convenience methods, raises on device failures by default).
- `test_connection.py` — cross-platform port discovery + `flow_rate` sanity check.
- `unit_test.py` — integration-style protocol/state tests over serial (some tests actuate the pump briefly).
- `update_juicer_firmware.py` — Debian-focused interactive toolchain setup + compile + firmware upload helper.
- `juice_pump3/` — firmware source (ESP32).
- `docs/assembly/README.md` — hardware assembly guide (with photos).

## Python setup (recommended)
These scripts require **Python 3.10+** and **pyserial**.

From the repo root:

```bash
python3 -m pip install -e .
```

Then run either:

```bash
juicer-test-connection
juicer-unit-test
```

## Python helper usage (`juicer.py`)

```python
from juicer import Juicer

with Juicer() as j:
    j.set(target_rps=2.5)
    j.set(reward_overlap_policy="append")
    resp = j.reward(0.5, "reward_mls", "reward_number", "juice_level")
    print(resp)
```

### Notes
- **Port selection**: `Juicer()` auto-detects the device. To force a specific port:

```python
from juicer import Juicer

j = Juicer(port="/dev/ttyACM0")  # or "COM15" on Windows
j.close()
```

- **Error handling**: the helper **raises** if the device replies with `{"status":"failure", ...}` (or other non-success status).
  - If you want to handle failures yourself, construct with `raise_on_failure=False` and check the returned dict.

- **Keeping a connection open** (recommended for long experiments): create one `Juicer()` and reuse it, then call `close()` at shutdown. e.g.,

```python
from juicer import Juicer

j = Juicer()
j.reward(0.5, "reward_mls", "reward_number", "juice_level")
j.close()
```

## Firmware compile + upload (Debian, interactive)
This repo includes an interactive helper that can install the full toolchain (user-local), patch the ESP32 core so the device enumerates as **`juicer3`**, pull the latest `main`, compile (with caching), and upload.

### Quickstart
Run from the repo root:

```bash
python3 update_juicer_firmware.py
```

Notes:
- Requires `sudo` for `apt-get` and to add you to the `dialout` group.
- After being added to `dialout`, you must **log out/in** (or reboot) once.
- Device detection prefers `/dev/serial/by-id/*juicer3*` and uses `arduino-cli board list --format json` to pick the correct ESP32-S2 vs ESP32-S3 FQBN.

## Notes
- Device enumerates as a USB serial port (e.g., `/dev/ttyACM0` on Linux, `COMx` on Windows).
- Responses are JSON per request; see `api.md` for expected shapes.








