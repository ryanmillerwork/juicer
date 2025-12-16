# Juicer

Precise USB-controlled fluid reward dispenser for behavioral experiments. Built around an ESP32 microcontroller driving a Boxer 9QX pump. The device is commanded with newline-terminated JSON over a 2,000,000 baud serial link.

- **API**: See `api.md` for full command reference (`set` / `do` / `get`).
- **Quick test**: `test_connection.py` auto-detects the device and queries `flow_rate`.
- **Typical use**: Send JSON commands like `{"do":{"reward":0.5}}` or `{"get":["flow_rate"]}` over the serial port.

## Files of interest
- `api.md` — serial API reference and CLI examples (Linux one-liners).
- `test_connection.py` — cross-platform port discovery + `flow_rate` sanity check.
- `juice_pump3/` — firmware source (ESP32).
- `firmware.md` — build/upload/calibration notes.

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

## Firmware compile + upload (Debian, interactive)
This repo includes an interactive helper that can install the full toolchain (user-local), patch the ESP32 core so the device enumerates as **`juicer3`**, pull the latest `main`, compile (with caching), and upload.

### Quickstart
Run from the repo root:

```bash
python3 scripts/juicer_flash.py
```

Notes:
- Requires `sudo` for `apt-get` and to add you to the `dialout` group.
- After being added to `dialout`, you must **log out/in** (or reboot) once.
- Device detection prefers `/dev/serial/by-id/*juicer3*` and uses `arduino-cli board list --format json` to pick the correct ESP32-S2 vs ESP32-S3 FQBN.

## Notes
- Device enumerates as a USB serial port (e.g., `/dev/ttyACM0` on Linux, `COMx` on Windows).
- Responses are JSON per request; see `api.md` for expected shapes.








