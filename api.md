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

- `{"set": {"flow_rate": <float>}}` — set **calibrated flow rate** (must be > 0). This is **not** a pump speed command; it tells the microcontroller how fast the pump *actually dispenses* (given your pump, tubing, configuration, etc.) so it can convert requested volumes into run time.
- `{"set":{"adjust_flow_rate":{"expected_mls":<float>,"actual_mls":<float>}}}` — adjust and persist flow rate based on an expected vs observed dispense (both must be > 0). Uses: `flow_rate_new = flow_rate_old * (actual_mls / expected_mls)`. Response includes `flow_rate_old`, `flow_rate_new`, and `scale_factor`.
- `{"set": {"purge_vol": <float>}}` — set purge volume (must be > 0).
- `{"set": {"target_rps": <float>}}` — set the pump motor **step rate** as a target revolutions-per-second (must be > 0 and <= MAX_RPS). This is the parameter that most directly corresponds to “how fast the pump runs”.
- `{"set": {"direction": "<left|right>"}}` — persist pump direction for future runs.
- `{"set": {"reward_overlap_policy": "<replace|append|reject>"}}` — persist what to do if `do.reward` arrives while a **serial reward** is already running:
  - `replace`: stop/shorten the previous reward and start the requested reward (historical behavior)
  - `append`: add the requested reward onto the end of the current running reward
  - `reject`: reject the new reward while one is already running

#### Notes on `flow_rate`

- `flow_rate` is a **calibration constant** used to dispense accurate volumes.
- The firmware uses it to compute an approximate run duration for volume-based commands like `do.reward` and `do.purge`:
  - `seconds ≈ requested_mLs / flow_rate`
- If you change tubing length/diameter, fittings, fluid viscosity, or other parts of the plumbing, you should expect `flow_rate` to change. Prefer using `set.adjust_flow_rate` after measuring a dispense.

#### Notes on `target_rps`

- `target_rps` controls the **commanded motor speed** (it sets the step pulse frequency internally). It does **not** guarantee the pump is actually spinning at that exact speed under load.
- **Default**: `3.0` RPS if no value has been stored previously.
- **Range**: the firmware enforces `0 < target_rps <= 8` (`MAX_RPS`). The source code notes that practical max is often **~3–4 RPS** depending on the pump/load.
- Changing `target_rps` will generally change the real dispense rate, so you should **re-calibrate `flow_rate`** (prefer `set.adjust_flow_rate`) after changing `target_rps`.
- **Persistence note**: `set.target_rps` takes effect immediately and is **persisted to flash** (it will survive reboot).
- **Motor driver current limit (hardware)**: if you push to higher RPS, you may also need to increase the motor driver’s current limit using the small trim screw/potentiometer on the driver board. Do this at your own risk: higher current means **more heat** (driver + motor). Make **very small turns**—tiny adjustments can make a big difference—and set the current **only as high as needed** to reliably turn the peristaltic pump without skipping/stalling.

### Do commands (only one per request)

- `{"do": "abort"}` — stop pump, update reward metrics.
- `{"do": "reset"}` — reset reward counters (number and mLs).
- `{"do": {"reward": <float>}}` — dispense a reward (must be > 0).
  - If a serial reward is already running, behavior depends on `reward_overlap_policy`.
  - If **purge/manual/calibration** is in progress, reward is rejected with `status:"failure"` and an explanatory `error`.
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
- `{"get": ["reward_overlap_policy"]}` → `{"reward_overlap_policy": "replace"|"append"|"reject"}`
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

