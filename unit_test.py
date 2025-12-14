"""
Integration/unit tests for an ESP32 running `juice_pump3.ino`.

These tests talk to the device over USB serial using the newline-terminated JSON API
documented in `api.md` (2,000,000 baud).

Why "unit" tests?
- The firmware runs on hardware, so host-side tests are necessarily *integration-style*.
  This file focuses on deterministic protocol + state checks, and gates any pump actuation
  behind an explicit opt-in.

Requirements:
- Python 3.10+
- pyserial (`pip install pyserial`)
- ESP32 connected and running `juice_pump3.ino`

Typical usage:
- Auto-detect port:
    python unit_test.py -v
- Specify port:
    python unit_test.py --port /dev/ttyACM0 -v
- Allow small pump actuation tests (dispenses tiny amounts):
    JUICER_ACTUATE=1 python unit_test.py -v

Environment variables:
- JUICER_PORT: override serial port (e.g., /dev/ttyACM0 or COM15)
- JUICER_ACTUATE=1: enable tests that call successful `do.reward`/`do.purge`/`do.calibration`
- JUICER_TEST_PERSIST=1: enable persistence-across-reset tests (best-effort reset)
"""

from __future__ import annotations

import argparse
import atexit
import glob
import json
import os
import sys
import time
import unittest
from dataclasses import dataclass
from typing import Any

import serial
from serial.tools import list_ports


BAUD = 2_000_000
DEFAULT_TIMEOUT_S = 2.5
DEFAULT_WRITE_TIMEOUT_S = 2.0
DEFAULT_OPEN_SETTLE_S = 0.6  # opening the port can reset the ESP32; give USB CDC time to settle


def find_port(preferred: str | None, debug: bool = False) -> str | None:
    """Best-effort port auto-discovery (mirrors `test_connection.py`)."""
    if preferred:
        return preferred

    for p in list_ports.comports():
        desc = (p.description or "").lower()
        if "juicer" in desc:
            return p.device

    for p in list_ports.comports():
        if (p.vid == 0x239A) and (p.pid == 0x8123):
            return p.device

    for p in list_ports.comports():
        hwid = (p.hwid or "").lower()
        if ("vid:239a" in hwid or "vid_239a" in hwid or "vid=239a" in hwid) and (
            "pid:8123" in hwid or "pid_8123" in hwid or "pid=8123" in hwid or "239a:8123" in hwid
        ):
            return p.device

    for path in glob.glob("/dev/serial/by-id/*juicer*"):
        return path

    if debug:
        print("Auto-detect tried description->vid/pid->hwid->by-id and found nothing.")
        for p in list_ports.comports():
            print(f"- device={p.device}, desc={p.description!r}, vid={p.vid}, pid={p.pid}, hwid={p.hwid!r}")

    return None


class ProtocolError(RuntimeError):
    pass


@dataclass
class JuicerClient:
    port: str
    baud: int = BAUD
    read_timeout_s: float = DEFAULT_TIMEOUT_S
    write_timeout_s: float = DEFAULT_WRITE_TIMEOUT_S
    settle_s: float = DEFAULT_OPEN_SETTLE_S
    exclusive: bool = True

    def __post_init__(self) -> None:
        self._ser = self._open_serial()
        # Mirror the working behavior in `test_connection.py` (some platforms need DTR/RTS asserted).
        try:
            self._ser.dtr = True
            self._ser.rts = True
        except Exception:
            # Some backends/OSes may not support setting these; continue.
            pass
        self._ser.reset_input_buffer()
        self._ser.reset_output_buffer()
        time.sleep(self.settle_s)

    def _open_serial(self) -> serial.Serial:
        """
        Open the serial port in the same style as `test_connection.py`.

        Important: On some ESP32 USB CDC stacks, opening the port can reset the MCU or briefly stall I/O.
        Using timeout=0 (non-blocking) matches our working `test_connection.py` approach.
        """
        # Prefer exclusive access so we fail fast if another process (screen/modemmanager/etc) has the port.
        try:
            return serial.Serial(
                self.port,
                self.baud,
                timeout=0,
                write_timeout=self.write_timeout_s,
                exclusive=self.exclusive,
            )
        except TypeError:
            # Older pyserial doesn't support exclusive=
            return serial.Serial(
                self.port,
                self.baud,
                timeout=0,
                write_timeout=self.write_timeout_s,
            )

    def _reopen(self) -> None:
        try:
            self._ser.close()
        except Exception:
            pass
        time.sleep(0.1)
        self._ser = self._open_serial()
        try:
            self._ser.dtr = True
            self._ser.rts = True
        except Exception:
            pass
        try:
            self._ser.reset_input_buffer()
            self._ser.reset_output_buffer()
        except Exception:
            pass
        time.sleep(self.settle_s)

    def close(self) -> None:
        try:
            self._ser.close()
        except Exception:
            pass

    def _readline_json(self, deadline_s: float) -> dict[str, Any]:
        """
        Read until a newline, returning the first parseable JSON object found.
        Ignores non-JSON lines (e.g., boot logs) until the deadline.
        """
        buf = b""
        while time.time() < deadline_s:
            # Read at least 1 byte, otherwise empty reads will spin.
            chunk = self._ser.read(self._ser.in_waiting or 1)
            if chunk:
                buf += chunk
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                line_s = line.decode(errors="replace").strip()
                if not line_s:
                    continue
                if not line_s.startswith("{"):
                    continue
                try:
                    obj = json.loads(line_s)
                except json.JSONDecodeError:
                    continue
                if isinstance(obj, dict):
                    return obj
            time.sleep(0.01)

        raise ProtocolError("Timed out waiting for a JSON response line from device")

    def request(self, payload: dict[str, Any] | str, timeout_s: float | None = None) -> dict[str, Any]:
        """
        Send a JSON command (dict or raw string) terminated by newline; return response object.
        """
        if timeout_s is None:
            timeout_s = self.read_timeout_s

        if isinstance(payload, dict):
            # Compact JSON to stay within the firmware's small RX buffer.
            msg = json.dumps(payload, separators=(",", ":"), ensure_ascii=True)
        else:
            msg = payload

        if not msg.endswith("\n"):
            msg += "\n"

        # Best-effort sync: clear anything pending so we associate response to this request.
        try:
            self._ser.reset_input_buffer()
        except Exception:
            pass
        data = msg.encode("utf-8")

        # Write retry: the ESP32 can be mid-reset if DTR/RTS toggled elsewhere; a short retry helps.
        last_exc: Exception | None = None
        for _attempt in range(3):
            try:
                self._ser.write(data)
                self._ser.flush()
                return self._readline_json(time.time() + timeout_s)
            except (serial.SerialTimeoutException, ProtocolError) as e:
                last_exc = e
                # If we can't write, the port may be busy or the MCU may have reset.
                # Reopen + settle matches the working "unplug/replug" recovery behavior.
                try:
                    self._reopen()
                except Exception:
                    time.sleep(0.25)
                try:
                    self._ser.reset_input_buffer()
                except Exception:
                    pass
        raise ProtocolError(f"Request failed after retries: {last_exc!r}")

    def get(self, *keys: str) -> dict[str, Any]:
        return self.request({"get": list(keys)})

    def set(self, **kwargs: Any) -> dict[str, Any]:
        return self.request({"set": kwargs})

    def do(self, action: Any, get: list[str] | None = None) -> dict[str, Any]:
        payload: dict[str, Any] = {"do": action}
        if get is not None:
            payload["get"] = get
        return self.request(payload)

    def soft_reset_best_effort(self) -> None:
        """
        Best-effort reset via DTR toggle. This may or may not reset depending on board/USB stack.
        If it does reset, the device may drop the port; callers should handle reconnect.
        """
        try:
            self._ser.dtr = False
            time.sleep(0.1)
            self._ser.dtr = True
        except Exception:
            pass
        time.sleep(0.5)


class JuicerTestBase(unittest.TestCase):
    """Shared client + helpers."""

    client: JuicerClient
    _saved_settings: dict[str, Any]

    @classmethod
    def setUpClass(cls) -> None:
        global _GLOBAL_CLIENT, _GLOBAL_SAVED_SETTINGS

        if _GLOBAL_CLIENT is None:
            port = find_port(os.environ.get("JUICER_PORT"), debug=os.environ.get("JUICER_DEBUG") == "1")
            if not port:
                raise unittest.SkipTest("No juicer serial port found (set JUICER_PORT or use --port).")

            _GLOBAL_CLIENT = JuicerClient(port=port)
            print(f"[unit_test] Using {port}", file=sys.stderr)

            # Quick handshake: prove we can send/receive (allow extra time for USB CDC reset/settle).
            deadline = time.time() + 12.0
            last: Exception | None = None
            while time.time() < deadline:
                try:
                    _GLOBAL_CLIENT.get("flow_rate")
                    last = None
                    break
                except Exception as e:
                    last = e
                    time.sleep(0.4)
            if last is not None:
                raise ProtocolError(
                    "Unable to communicate with device during handshake. "
                    "If `python test_connection.py` also fails, check that no other process is using the port "
                    "(e.g. `screen`, Arduino Serial Monitor, ModemManager) and try unplug/replug.\n"
                    f"Last error: {last!r}"
                )

        cls.client = _GLOBAL_CLIENT

        if _GLOBAL_SAVED_SETTINGS is None:
            # Snapshot current persisted settings so we can restore at process exit.
            _GLOBAL_SAVED_SETTINGS = cls.client.request(
                {"get": ["flow_rate", "purge_vol", "target_rps", "direction", "reward_mls", "reward_number"]}
            )
            cls._saved_settings = _GLOBAL_SAVED_SETTINGS

            # Make reward counters deterministic for the suite.
            cls.client.do("reset")
        else:
            cls._saved_settings = _GLOBAL_SAVED_SETTINGS

    @classmethod
    def tearDownClass(cls) -> None:
        # Global cleanup handles restore/close once for the whole suite.
        return

    def assertStatusSuccess(self, resp: dict[str, Any]) -> None:
        self.assertIn("status", resp, f"Expected a status field; got: {resp}")
        self.assertEqual(resp.get("status"), "success", f"Expected status=success; got: {resp}")

    def assertStatusFailure(self, resp: dict[str, Any]) -> None:
        self.assertIn("status", resp, f"Expected a status field; got: {resp}")
        self.assertEqual(resp.get("status"), "failure", f"Expected status=failure; got: {resp}")
        self.assertIn("error", resp, f"Expected an error field; got: {resp}")


# Global singleton to avoid opening/closing the serial port per TestCase class.
# Opening/closing can toggle DTR/RTS and reset the ESP32, causing flaky timeouts.
_GLOBAL_CLIENT: JuicerClient | None = None
_GLOBAL_SAVED_SETTINGS: dict[str, Any] | None = None


def _global_cleanup() -> None:
    global _GLOBAL_CLIENT, _GLOBAL_SAVED_SETTINGS
    if _GLOBAL_CLIENT is None:
        return
    try:
        if _GLOBAL_SAVED_SETTINGS:
            restore: dict[str, Any] = {}
            for k in ("flow_rate", "purge_vol", "target_rps", "direction"):
                if k in _GLOBAL_SAVED_SETTINGS:
                    restore[k] = _GLOBAL_SAVED_SETTINGS[k]
            if restore:
                _GLOBAL_CLIENT.set(**restore)
        _GLOBAL_CLIENT.do("reset")
    except Exception:
        # Never mask original test failures
        pass
    finally:
        try:
            _GLOBAL_CLIENT.close()
        except Exception:
            pass
        _GLOBAL_CLIENT = None
        _GLOBAL_SAVED_SETTINGS = None


atexit.register(_global_cleanup)


class TestProtocolBasics(JuicerTestBase):
    def test_get_flow_rate_round_trip(self) -> None:
        resp = self.client.get("flow_rate")
        self.assertIn("flow_rate", resp)
        self.assertIsInstance(resp["flow_rate"], (int, float))
        self.assertGreater(resp["flow_rate"], 0)

    def test_unknown_get_key_returns_unknown_parameter(self) -> None:
        resp = self.client.get("definitely_not_a_real_key")
        self.assertIn("definitely_not_a_real_key", resp)
        self.assertEqual(resp["definitely_not_a_real_key"], "Unknown parameter")

    def test_invalid_json_reports_invalid_json_format(self) -> None:
        resp = self.client.request('{"get":["flow_rate"]')  # missing closing brackets/brace
        # Firmware returns this exact string in the "status" field for parse errors.
        self.assertEqual(resp.get("status"), "Invalid JSON format")

    def test_do_multiple_ops_rejected(self) -> None:
        resp = self.client.request({"do": {"reward": 1, "purge": 1}})
        self.assertStatusFailure(resp)
        self.assertIn("Only one 'do' operation", resp.get("error", ""))

    def test_do_invalid_type_rejected(self) -> None:
        resp = self.client.request({"do": 123})
        self.assertStatusFailure(resp)
        self.assertIn("Invalid 'do' command format", resp.get("error", ""))


class TestSetValidation(JuicerTestBase):
    def test_set_flow_rate_rejects_nonpositive(self) -> None:
        resp = self.client.set(flow_rate=0)
        self.assertStatusFailure(resp)
        resp = self.client.set(flow_rate=-1)
        self.assertStatusFailure(resp)

    def test_set_purge_vol_rejects_nonpositive(self) -> None:
        resp = self.client.set(purge_vol=0)
        self.assertStatusFailure(resp)
        resp = self.client.set(purge_vol=-5)
        self.assertStatusFailure(resp)

    def test_set_target_rps_bounds(self) -> None:
        resp = self.client.set(target_rps=0)
        self.assertStatusFailure(resp)
        resp = self.client.set(target_rps=-1)
        self.assertStatusFailure(resp)
        resp = self.client.set(target_rps=9)  # MAX_RPS is 8 in firmware
        self.assertStatusFailure(resp)

        resp = self.client.set(target_rps=1)
        self.assertStatusSuccess(resp)
        got = self.client.get("target_rps")
        self.assertAlmostEqual(float(got["target_rps"]), 1.0, places=3)

    def test_set_direction_accepts_left_right(self) -> None:
        resp = self.client.set(direction="right")
        self.assertStatusSuccess(resp)
        got = self.client.get("direction")
        self.assertEqual(got.get("direction"), "right")

        resp = self.client.set(direction="left")
        self.assertStatusSuccess(resp)
        got = self.client.get("direction")
        self.assertEqual(got.get("direction"), "left")

        resp = self.client.set(direction="banana")
        self.assertStatusFailure(resp)
        self.assertIn("Invalid direction", resp.get("error", ""))


class TestDoValidationNoActuation(JuicerTestBase):
    def test_do_reset_sets_counters_to_zero(self) -> None:
        # Seed counters without actuating by directly setting internal counters? Not possible;
        # so just verify reset produces 0s (the suite already resets in setUpClass).
        resp = self.client.do("reset", get=["reward_mls", "reward_number"])
        self.assertStatusSuccess(resp)
        self.assertEqual(resp.get("reward_number"), 0)
        self.assertAlmostEqual(float(resp.get("reward_mls", 999)), 0.0, places=3)

    def test_do_unknown_action_fails(self) -> None:
        resp = self.client.do("not_an_action")
        self.assertStatusFailure(resp)
        self.assertEqual(resp.get("error"), "Unknown action")

    def test_do_reward_rejects_nonpositive(self) -> None:
        resp = self.client.do({"reward": 0})
        self.assertStatusFailure(resp)
        resp = self.client.do({"reward": -1})
        self.assertStatusFailure(resp)

    def test_do_purge_rejects_nonpositive(self) -> None:
        resp = self.client.do({"purge": 0})
        self.assertStatusFailure(resp)
        resp = self.client.do({"purge": -1})
        self.assertStatusFailure(resp)

    def test_do_calibration_rejects_invalid_params(self) -> None:
        resp = self.client.do({"calibration": {"n": 0, "on": 100, "off": 100}})
        self.assertStatusFailure(resp)
        resp = self.client.do({"calibration": {"n": 1, "on": 0, "off": 100}})
        self.assertStatusFailure(resp)
        resp = self.client.do({"calibration": {"n": 1, "on": 100, "off": 0}})
        self.assertStatusFailure(resp)


@unittest.skipUnless(os.environ.get("JUICER_ACTUATE") == "1", "Set JUICER_ACTUATE=1 to enable pump actuation tests")
class TestActuationSmallVolumes(JuicerTestBase):
    def test_reward_increments_counters(self) -> None:
        self.client.do("reset")
        before = self.client.get("reward_mls", "reward_number")
        before_n = int(before["reward_number"])
        before_ml = float(before["reward_mls"])

        # Tiny reward to keep actuation short.
        resp = self.client.do({"reward": 0.05}, get=["reward_mls", "reward_number"])
        self.assertStatusSuccess(resp)

        # Give the pump time to finish (depends on flow_rate).
        time.sleep(0.3)
        after = self.client.get("reward_mls", "reward_number")
        after_n = int(after["reward_number"])
        after_ml = float(after["reward_mls"])

        self.assertEqual(after_n, before_n + 1)
        self.assertGreater(after_ml, before_ml)

    def test_purge_success_and_abort(self) -> None:
        # Start a tiny purge then abort quickly to ensure abort path is healthy.
        resp = self.client.do({"purge": 0.05})
        self.assertStatusSuccess(resp)
        time.sleep(0.1)
        resp2 = self.client.do("abort")
        self.assertStatusSuccess(resp2)

    def test_calibration_start_and_abort(self) -> None:
        # 2 short cycles; then abort immediately.
        resp = self.client.do({"calibration": {"n": 2, "on": 100, "off": 100}})
        self.assertStatusSuccess(resp)
        time.sleep(0.1)
        resp2 = self.client.do("abort")
        self.assertStatusSuccess(resp2)


@unittest.skipUnless(
    os.environ.get("JUICER_TEST_PERSIST") == "1",
    "Set JUICER_TEST_PERSIST=1 to enable persistence-across-reset tests",
)
class TestPersistenceBestEffort(JuicerTestBase):
    def test_direction_persists_across_soft_reset(self) -> None:
        # Save original direction
        orig = self.client.get("direction").get("direction")
        self.assertIn(orig, ("left", "right"))

        target = "right" if orig == "left" else "left"
        resp = self.client.set(direction=target)
        self.assertStatusSuccess(resp)

        # Best-effort reset: may not work on all platforms/boards.
        self.client.soft_reset_best_effort()
        time.sleep(0.5)

        got = self.client.get("direction").get("direction")
        self.assertEqual(got, target)

        # Restore
        self.client.set(direction=orig)


def _main() -> None:
    parser = argparse.ArgumentParser(add_help=True)
    parser.add_argument("--port", help="Serial port (e.g., /dev/ttyACM0 or COM15). Also via JUICER_PORT.")
    parser.add_argument("--debug", action="store_true", help="Print port discovery debug via JUICER_DEBUG=1.")
    parser.add_argument(
        "--actuate",
        action="store_true",
        help="Enable pump actuation tests (same as JUICER_ACTUATE=1).",
    )
    parser.add_argument(
        "--persist",
        action="store_true",
        help="Enable persistence-across-reset tests (same as JUICER_TEST_PERSIST=1).",
    )
    args, remaining = parser.parse_known_args()

    if args.port:
        os.environ["JUICER_PORT"] = args.port
    if args.debug:
        os.environ["JUICER_DEBUG"] = "1"
    if args.actuate:
        os.environ["JUICER_ACTUATE"] = "1"
    if args.persist:
        os.environ["JUICER_TEST_PERSIST"] = "1"

    # Hand control to unittest, preserving any remaining args like -v / -k patterns.
    unittest.main(argv=[sys.argv[0]] + remaining)


if __name__ == "__main__":
    _main()


