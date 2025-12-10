"""
Test connectivity to the juicer over serial (Windows or Linux).

Auto-discovers the port by name/description containing "juicer", then falls
back to VID/PID 239A:8123 (via vid/pid fields or hwid), or /dev/serial/by-id/*juicer*
on Linux. Sends a flow_rate query and prints the response. Requires pyserial.

You can override detection with --port or JUICER_PORT. Use --debug to see
detected ports, VIDs, PIDs, and HWIDs when auto-detection fails.
"""

import sys
import time
import glob
import os
import argparse
import serial
from serial.tools import list_ports


def find_port(preferred: str | None, debug: bool = False) -> str | None:
    if preferred:
        return preferred

    # Primary: match "juicer" in the port description (Windows/Linux)
    for p in list_ports.comports():
        desc = (p.description or "").lower()
        if "juicer" in desc:
            return p.device

    # Secondary: match VID/PID 239A:8123 (Adafruit + this firmware).
    # First, use explicit vid/pid fields if present.
    for p in list_ports.comports():
        if (p.vid == 0x239A) and (p.pid == 0x8123):
            return p.device

    # Then, fall back to parsing hwid strings which vary by platform.
    # Windows hwid examples vary: "VID:239A PID:8123", "USB\\VID_239A&PID_8123", or "239a:8123".
    for p in list_ports.comports():
        hwid = (p.hwid or "").lower()
        if ("vid:239a" in hwid or "vid_239a" in hwid or "vid=239a" in hwid) and (
            "pid:8123" in hwid or "pid_8123" in hwid or "pid=8123" in hwid or "239a:8123" in hwid
        ):
            return p.device

    # Linux fallback: look for by-id symlink containing juicer
    for path in glob.glob("/dev/serial/by-id/*juicer*"):
        return path

    if debug:
        print("Auto-detect tried description->vid/pid->hwid->by-id and found nothing.")
        for p in list_ports.comports():
            print(f"- device={p.device}, desc={p.description!r}, vid={p.vid}, pid={p.pid}, hwid={p.hwid!r}")

    return None


def main() -> None:
    parser = argparse.ArgumentParser(description="Query juicer flow_rate over serial.")
    parser.add_argument("--port", help="Serial port (overrides auto-detect; e.g., COM15 or /dev/ttyACM0)")
    parser.add_argument("--debug", action="store_true", help="Print detected ports when auto-detect fails.")
    args = parser.parse_args()

    env_port = os.environ.get("JUICER_PORT")
    port = find_port(args.port or env_port, debug=args.debug)
    if not port:
        ports = [(p.device, p.description) for p in list_ports.comports()]
        msg = "no juicer port found"
        if ports:
            msg += "; seen ports: " + ", ".join(f"{d} ({desc})" for d, desc in ports)
        sys.exit(msg)
    print(f"Using {port}")

    # Mirror the working PowerShell behavior: assert DTR/RTS, poll for up to ~2s
    with serial.Serial(port, 2_000_000, timeout=0, write_timeout=2) as s:
        s.dtr = True
        s.rts = True
        s.reset_input_buffer()
        s.reset_output_buffer()
        s.write(b'{"get":["flow_rate"]}\n')
        s.flush()

        deadline = time.time() + 2.0
        buf = b""
        while time.time() < deadline:
            buf += s.read(s.in_waiting or 1)
            if b"\n" in buf:
                break
            time.sleep(0.05)

    print(buf.decode(errors="replace").strip() or "(no response)")


if __name__ == "__main__":
    main()

