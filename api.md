# Juicer Serial API

The pump speaks newline-terminated JSON over its USB serial port (2,000,000 baud). Below is the command set implemented by the firmware in `juice_pump3/juice_pump3.ino`, plus a one-liner example to query the device on Linux.

## Quick usage example

Requires `socat` (`sudo apt-get install socat` on Debian/Ubuntu).

```bash
timeout 3s bash -lc 'dev=$(readlink -f /dev/serial/by-id/*juicer*); printf '"'"'{"get":["flow_rate"]}\n'"'"' | socat - "file:$dev,raw,echo=0,b2000000"'
```

Sample output:

```
{"flow_rate":0.5}
```

Reward 0.5 mL and get the response:

```bash
timeout 3s bash -lc 'dev=$(readlink -f /dev/serial/by-id/*juicer*); printf '"'"'{"do":{"reward":0.5},"get":["reward_mls","reward_number"]}\n'"'"' | socat - "file:$dev,raw,echo=0,b2000000"'
```

## Code examples (send/receive)

### Python (pyserial, Linux path)
```python
import serial, time
port = "/dev/ttyACM0"  # or the resolved /dev/serial/by-id/... path
s = serial.Serial(port, 2_000_000, timeout=2, write_timeout=2, dsrdtr=True, rtscts=False)
s.write(b'{"get":["flow_rate"]}\n'); s.flush()
time.sleep(0.4)
print(s.readline().decode().strip())
s.close()
```

### MATLAB (Linux path)
```matlab
port = "/dev/ttyACM0"; % or "/dev/serial/by-id/usb-...juicer..."
s = serialport(port, 2000000, "Timeout", 2);
writeline(s, '{"do":{"reward":0.5},"get":["reward_mls","reward_number"]}');
resp = readline(s)
clear s
```

### Tcl (Linux path)
```tcl
set port "/dev/ttyACM0" ;# or "/dev/serial/by-id/usb-...juicer..."
set f [open $port r+]
fconfigure $f -translation binary -buffering none -mode "2000000,n,8,1" -blocking 1
puts -nonewline $f "{\"get\":[\"flow_rate\"]}\n"
flush $f
gets $f resp
close $f
puts $resp
```

## Command reference

Send JSON objects with any combination of `set`, `do`, and `get` fields. Only one `do` operation is allowed per request. Responses include a `status` field (`success` or `failure`) and any requested values.

### Set commands

- `{"set": {"flow_rate": <float>}}` — set flow rate (must be > 0).
- `{"set": {"purge_vol": <float>}}` — set purge volume (must be > 0).
- `{"set": {"target_rps": <float>}}` — set target revolutions per second (must be > 0 and <= MAX_RPS).
- `{"set": {"direction": "<left|right>"}}` — persist pump direction for future runs.

### Do commands (only one per request)

- `{"do": "abort"}` — stop pump, update reward metrics.
- `{"do": "reset"}` — reset reward counters (number and mLs).
- `{"do": {"reward": <float>}}` — dispense a reward (must be > 0).
- `{"do": {"purge": <float>}}` — purge for given volume (must be > 0).
- `{"do": {"calibration": {"n": <int>, "on": <int>, "off": <int>}}}` — run calibration cycles; all parameters must be > 0.

### Get commands

Request fields in a `get` array; the response includes those keys:

- `{"get": ["flow_rate"]}` → `{"flow_rate": <float>}`
- `{"get": ["purge_vol"]}` → `{"purge_vol": <float>}`
- `{"get": ["target_rps"]}` → `{"target_rps": <float>}`
- `{"get": ["reward_mls"]}` → `{"reward_mls": <float>}`
- `{"get": ["reward_number"]}` → `{"reward_number": <int>}`
- `{"get": ["direction"]}` → `{"direction": "left"|"right"}`
- `{"get": ["juice_level"]}` → `{"juice_level": ">50mLs"|"<50mLs"}`
- Unknown keys (e.g., `{"get": ["foo"]}`) return `{"foo": "Unknown parameter"}`

### Combined example

```
{"set": {"target_rps": 2, "flow_rate": 0.65}, "do": {"reward": 1}, "get": ["reward_mls", "reward_number"]}
```

Expected response shape:

```
{
  "status": "success",
  "reward_mls": <float>,
  "reward_number": <int>
}
```

