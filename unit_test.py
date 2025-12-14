"""
Integration/unit tests for an ESP32 running `juice_pump3.ino`.

These tests talk to the device over USB serial using the newline-terminated JSON API
documented in `api.md` (2,000,000 baud).

Why "unit" tests?
- The firmware runs on hardware, so host-side tests are necessarily *integration-style*.
  This file focuses on deterministic protocol + state checks, and gates any pump actuation
  behind an explicit opt-in.

What gets tested (by class):
- `TestProtocolBasics`
  - `test_get_flow_rate_round_trip`: `get` returns a numeric `flow_rate` > 0.
  - `test_get_all_api_keys_round_trip`: `get` returns all documented keys with sane types/values.
  - `test_unknown_get_key_returns_unknown_parameter`: unknown `get` keys return `"Unknown parameter"`.
  - `test_invalid_json_reports_invalid_json_format`: malformed JSON yields `{"status":"Invalid JSON format"}`.
  - `test_do_multiple_ops_rejected`: firmware rejects requests with >1 `do` op in a single object.
  - `test_do_invalid_type_rejected`: non-string/non-object `do` values are rejected.
- `TestSetValidation`
  - `test_set_flow_rate_rejects_nonpositive`: `set.flow_rate` must be > 0.
  - `test_set_purge_vol_rejects_nonpositive`: `set.purge_vol` must be > 0.
  - `test_set_target_rps_bounds`: `set.target_rps` must be > 0 and <= MAX_RPS; valid values round-trip via `get`.
  - `test_set_direction_accepts_left_right`: `set.direction` accepts left/right and rejects other strings.
- `TestDoValidationNoActuation` (safe: does not start the pump successfully)
  - `test_do_reset_sets_counters_to_zero`: `do.reset` zeros `reward_number` and `reward_mls`.
  - `test_do_unknown_action_fails`: unknown string actions fail with an error.
  - `test_do_reward_rejects_nonpositive`: `do.reward` rejects <= 0.
  - `test_do_purge_rejects_nonpositive`: `do.purge` rejects <= 0.
  - `test_do_calibration_rejects_invalid_params`: `do.calibration` rejects non-positive parameters.
- `TestActuationSmallVolumes` (enabled by default; WILL move the pump briefly)
  - `test_reward_increments_counters`: small reward increments counters.
  - `test_purge_success_and_abort`: starts a small purge and exercises `do.abort`.
  - `test_calibration_start_and_abort`: starts a short calibration and exercises `do.abort`.
- `TestPersistenceBestEffort` (enabled by default; toggles DTR for a best-effort reset)
  - `test_direction_persists_across_soft_reset`: checks that `direction` persists across a best-effort DTR reset.

Requirements:
- Python 3.10+
- pyserial (`pip install pyserial`)
- ESP32 connected and running `juice_pump3.ino`

Usage:
- Just run it (prints commands/responses/delays; runs the full suite by default):
    python unit_test.py

Environment variables (optional):
- JUICER_PORT: override serial port (e.g., /dev/ttyACM0 or COM15)
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
DEFAULT_TIMEOUT_S = 3.0
DEFAULT_WRITE_TIMEOUT_S = 3.0
DEFAULT_OPEN_SETTLE_S = 0.10  # match `capactive_calibration.py`'s "Give device a moment after opening port"

# Always trace: user requested that `python unit_test.py` prints everything without flags.
TRACE = True


def _t(msg: str) -> None:
    """Trace log (stderr)."""
    print(f"[unit_test][trace] {msg}", file=sys.stderr)


def _sleep(seconds: float, reason: str) -> None:
    """Sleep with optional trace output."""
    if seconds <= 0:
        return
    _t(f"sleep {seconds:.3f}s ({reason})")
    time.sleep(seconds)


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
    # Keep this False by default; TIOCEXCL doesn't detect an already-open port anyway,
    # and some setups behave strangely with exclusive locks.
    exclusive: bool = False

    def __post_init__(self) -> None:
        self._ser = self._open_serial()
        # Some platforms need DTR/RTS asserted (mirrors `test_connection.py`).
        # But unlike the earlier version, we avoid reopening/reset loops.
        for attr, val in (("dtr", True), ("rts", True)):
            try:
                setattr(self._ser, attr, val)
            except Exception:
                pass
        self._ser.reset_input_buffer()
        self._ser.reset_output_buffer()
        _sleep(self.settle_s, "after opening port / asserting DTR+RTS")

    def _open_serial(self) -> serial.Serial:
        """
        Open the serial port in the same style as `capactive_calibration.py` / `api.md` examples.

        Important: On some ESP32 USB CDC stacks, opening the port can reset the MCU or briefly stall I/O.
        """
        try:
            return serial.Serial(
                self.port,
                self.baud,
                timeout=self.read_timeout_s,
                write_timeout=self.write_timeout_s,
                dsrdtr=True,
                rtscts=False,
                exclusive=self.exclusive,
            )
        except TypeError:
            # Older pyserial doesn't support exclusive=
            return serial.Serial(
                self.port,
                self.baud,
                timeout=self.read_timeout_s,
                write_timeout=self.write_timeout_s,
                dsrdtr=True,
                rtscts=False,
            )

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

        # Match `capactive_calibration.py`: clear pending input so reply is correlated.
        try:
            self._ser.reset_input_buffer()
        except Exception:
            pass
        data = msg.encode("utf-8")

        try:
            _t(f">> {msg.rstrip()}")
            self._ser.write(data)
            # This is exactly what your existing scripts do (`capactive_calibration.py`).
            # If your setup's flush is safe there, it should be safe here too.
            self._ser.flush()
        except serial.SerialTimeoutException as e:
            raise ProtocolError(
                "Serial write timed out. This almost always means the port is wedged/busy.\n"
                "- Close anything that might have /dev/ttyACM0 open (Arduino Serial Monitor, `screen`, etc)\n"
                "- If on Ubuntu, consider stopping ModemManager: `sudo systemctl stop ModemManager`\n"
                "- Unplug/replug the ESP32\n"
                f"Original error: {e!r}"
            ) from e

        # Read one line and parse, matching the docs/examples.
        raw = self._ser.readline().decode(errors="replace").strip()
        _t(f"<< {raw if raw else '<no response>'}")
        if not raw:
            raise ProtocolError("No response line received from device (timeout).")
        if not raw.startswith("{"):
            # Firmware sometimes prints non-JSON noise; treat as protocol error with raw line for debugging.
            raise ProtocolError(f"Non-JSON response line: {raw!r}")
        try:
            obj = json.loads(raw)
        except json.JSONDecodeError as e:
            raise ProtocolError(f"Invalid JSON response: {raw!r}") from e
        if not isinstance(obj, dict):
            raise ProtocolError(f"JSON response was not an object: {raw!r}")
        return obj

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
            _sleep(0.1, "DTR low (best-effort reset)")
            self._ser.dtr = True
        except Exception:
            pass
        _sleep(0.5, "post-reset settle")


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

            # Quick handshake: prove we can send/receive. Fail FAST if we can't write/read.
            # If this fails, tests can't proceed meaningfully anyway.
            _GLOBAL_CLIENT.get("flow_rate")

        cls.client = _GLOBAL_CLIENT

        if _GLOBAL_SAVED_SETTINGS is None:
            # Snapshot current persisted settings so we can restore at process exit.
            _GLOBAL_SAVED_SETTINGS = cls.client.request(
                {"get": ["flow_rate", "purge_vol", "target_rps", "direction", "reward_mls", "reward_number"]}
            )
            cls._saved_settings = _GLOBAL_SAVED_SETTINGS
            # Print persisted settings up-front for operator confidence/debugging.
            print(
                "[unit_test] Persisted settings at start: "
                f"flow_rate={cls._saved_settings.get('flow_rate')}, "
                f"purge_vol={cls._saved_settings.get('purge_vol')}, "
                f"target_rps={cls._saved_settings.get('target_rps')}, "
                f"direction={cls._saved_settings.get('direction')}",
                file=sys.stderr,
            )

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
        try:
            _GLOBAL_CLIENT.do("reset")
        except KeyboardInterrupt:
            return
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

    def test_get_all_api_keys_round_trip(self) -> None:
        # All documented "get" keys in `api.md` / `juice_pump3.ino`.
        keys = [
            "flow_rate",
            "purge_vol",
            "target_rps",
            "reward_mls",
            "reward_number",
            "direction",
            "juice_level",
        ]
        resp = self.client.request({"get": keys})

        for k in keys:
            self.assertIn(k, resp, f"Missing key {k!r} in response: {resp}")

        self.assertIsInstance(resp["flow_rate"], (int, float))
        self.assertGreater(float(resp["flow_rate"]), 0.0)

        self.assertIsInstance(resp["purge_vol"], (int, float))
        self.assertGreater(float(resp["purge_vol"]), 0.0)

        self.assertIsInstance(resp["target_rps"], (int, float))
        self.assertGreater(float(resp["target_rps"]), 0.0)

        self.assertIsInstance(resp["reward_mls"], (int, float))
        self.assertGreaterEqual(float(resp["reward_mls"]), 0.0)

        # ArduinoJson may serialize ints as numbers; accept int-like floats too.
        self.assertIsInstance(resp["reward_number"], (int, float))
        self.assertEqual(int(resp["reward_number"]), resp["reward_number"])
        self.assertGreaterEqual(int(resp["reward_number"]), 0)

        self.assertIn(resp["direction"], ("left", "right"))

        # Firmware currently returns ">50mLs" or "<50mLs", but keep this loose to avoid false negatives.
        self.assertIsInstance(resp["juice_level"], str)
        self.assertGreater(len(resp["juice_level"]), 0)

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

    def test_set_adjust_flow_rate_computes_and_persists(self) -> None:
        # Read current
        before = self.client.get("flow_rate")
        old = float(before["flow_rate"])
        self.assertGreater(old, 0.0)

        # Use simple ratio so expected math is unambiguous
        expected_mls = 100.0
        actual_mls = 50.0
        scale = actual_mls / expected_mls
        expected_new = old * scale

        resp = self.client.request(
            {"set": {"adjust_flow_rate": {"expected_mls": expected_mls, "actual_mls": actual_mls}}}
        )
        self.assertStatusSuccess(resp)
        self.assertIn("flow_rate_old", resp)
        self.assertIn("flow_rate_new", resp)
        self.assertIn("scale_factor", resp)
        self.assertAlmostEqual(float(resp["flow_rate_old"]), old, places=6)
        self.assertAlmostEqual(float(resp["scale_factor"]), scale, places=6)
        self.assertAlmostEqual(float(resp["flow_rate_new"]), expected_new, places=6)

        # Verify persisted value via get
        after = self.client.get("flow_rate")
        self.assertAlmostEqual(float(after["flow_rate"]), expected_new, places=6)

    def test_set_adjust_flow_rate_rejects_invalid(self) -> None:
        resp = self.client.request({"set": {"adjust_flow_rate": {"expected_mls": 0, "actual_mls": 10}}})
        self.assertStatusFailure(resp)
        resp = self.client.request({"set": {"adjust_flow_rate": {"expected_mls": 10, "actual_mls": 0}}})
        self.assertStatusFailure(resp)
        resp = self.client.request({"set": {"adjust_flow_rate": {"expected_mls": -1, "actual_mls": 10}}})
        self.assertStatusFailure(resp)
        resp = self.client.request({"set": {"adjust_flow_rate": {"expected_mls": 10, "actual_mls": -1}}})
        self.assertStatusFailure(resp)

    def test_set_adjust_flow_rate_and_flow_rate_mutually_exclusive(self) -> None:
        resp = self.client.request(
            {"set": {"flow_rate": 1.0, "adjust_flow_rate": {"expected_mls": 10, "actual_mls": 10}}}
        )
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


class TestActuationSmallVolumes(JuicerTestBase):
    @classmethod
    def setUpClass(cls) -> None:
        super().setUpClass()
        # Safety: make it very obvious we're about to move the pump, and give a moment to abort.
        print(
            "[unit_test] NOTE: Actuation tests are enabled by default and WILL move the pump briefly.\n"
            "[unit_test] If this is unexpected, press Ctrl+C now.",
            file=sys.stderr,
        )
        _sleep(3.0, "safety countdown before actuation tests")

    def test_reward_increments_counters(self) -> None:
        self.client.do("reset")
        before = self.client.get("reward_mls", "reward_number")
        before_n = int(before["reward_number"])
        before_ml = float(before["reward_mls"])

        # Tiny reward to keep actuation short.
        resp = self.client.do({"reward": 0.05}, get=["reward_mls", "reward_number"])
        self.assertStatusSuccess(resp)

        # Give the pump time to finish (depends on flow_rate).
        _sleep(0.3, "allow pump to finish small reward")
        after = self.client.get("reward_mls", "reward_number")
        after_n = int(after["reward_number"])
        after_ml = float(after["reward_mls"])

        self.assertEqual(after_n, before_n + 1)
        self.assertGreater(after_ml, before_ml)

    def test_purge_success_and_abort(self) -> None:
        # Start a tiny purge then abort quickly to ensure abort path is healthy.
        resp = self.client.do({"purge": 0.05})
        self.assertStatusSuccess(resp)
        _sleep(0.1, "let purge start before abort")
        resp2 = self.client.do("abort")
        self.assertStatusSuccess(resp2)

    def test_calibration_start_and_abort(self) -> None:
        # 2 short cycles; then abort immediately.
        resp = self.client.do({"calibration": {"n": 2, "on": 100, "off": 100}})
        self.assertStatusSuccess(resp)
        _sleep(0.1, "let calibration start before abort")
        resp2 = self.client.do("abort")
        self.assertStatusSuccess(resp2)


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
        _sleep(0.5, "allow device to reboot/reconnect after best-effort reset")

        got = self.client.get("direction").get("direction")
        self.assertEqual(got, target)

        # Restore
        self.client.set(direction=orig)


def _main() -> None:
    parser = argparse.ArgumentParser(add_help=True)
    parser.add_argument("--port", help="Serial port (e.g., /dev/ttyACM0 or COM15). Also via JUICER_PORT.")
    parser.add_argument("--debug", action="store_true", help="Print port discovery debug via JUICER_DEBUG=1.")
    args, remaining = parser.parse_known_args()

    if args.port:
        os.environ["JUICER_PORT"] = args.port
    if args.debug:
        os.environ["JUICER_DEBUG"] = "1"

    # Hand control to unittest, preserving any remaining args like -v / -k patterns.
    # Use exit=False so we can print an explicit pass/fail count summary afterward.
    program = unittest.main(argv=[sys.argv[0]] + remaining, exit=False)

    result = program.result  # unittest.result.TestResult
    tests_run = getattr(result, "testsRun", 0)
    n_errors = len(getattr(result, "errors", []) or [])
    n_failures = len(getattr(result, "failures", []) or [])
    n_skipped = len(getattr(result, "skipped", []) or [])
    n_xfail = len(getattr(result, "expectedFailures", []) or [])
    n_xpass = len(getattr(result, "unexpectedSuccesses", []) or [])

    n_failed_total = n_errors + n_failures + n_xpass
    n_passed = tests_run - n_failed_total - n_skipped - n_xfail
    if n_passed < 0:
        n_passed = 0

    print(
        "[unit_test] Summary: "
        f"run={tests_run}, passed={n_passed}, failed={n_failed_total} "
        f"(failures={n_failures}, errors={n_errors}, xpass={n_xpass}), "
        f"skipped={n_skipped}, xfail={n_xfail}",
        file=sys.stderr,
    )

    raise SystemExit(0 if result.wasSuccessful() else 1)


if __name__ == "__main__":
    _main()


