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

## Notes
- Device enumerates as a USB serial port (e.g., `/dev/ttyACM0` on Linux, `COMx` on Windows).
- Responses are JSON per request; see `api.md` for expected shapes.








