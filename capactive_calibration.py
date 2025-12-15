#!/usr/bin/env python3
from __future__ import annotations
import argparse, json, subprocess, time
from pathlib import Path
import os
import configparser
import sys

DEFAULT_PGSERVICE = "juicer"

def connect_home_automation(
    dbname: str = "home_automation",
    host: str = "ryan-analysis",
    port: int | None = None,
    user: str | None = None,
    service: str | None = None,
):
    """Return a psycopg connection to the Postgres 17 home_automation db.

    Auth details are read from libpq sources (environment, ~/.pgpass, etc.).
    If you have a `~/.pg_service.conf` entry, pass its name via `service=...`
    (or set the `PGSERVICE` env var) so libpq can load host/port/db defaults.
    """
    try:
        import psycopg
    except Exception as e:  # pragma: no cover - convenience for missing dep
        raise RuntimeError("psycopg is required to connect to Postgres") from e

    # Explicit kwargs override values coming from the service definition.
    kwargs: dict = {}
    if service:
        kwargs["service"] = service
    if dbname:
        kwargs["dbname"] = dbname
    if host:
        kwargs["host"] = host
    if port is not None:
        kwargs["port"] = port
    if user:
        kwargs["user"] = user
    return psycopg.connect(**kwargs)

def _candidate_service_files() -> list[Path]:
    """Return libpq pg_service.conf candidate locations (user + system)."""
    # libpq supports PGSERVICEFILE to point at a specific service file.
    env_file = os.environ.get("PGSERVICEFILE")
    if env_file:
        return [Path(env_file).expanduser()]
    return [Path.home() / ".pg_service.conf", Path("/etc/pg_service.conf")]

def _service_exists(service: str) -> tuple[bool, list[Path]]:
    """Return (exists, searched_files) for a libpq service name."""
    searched: list[Path] = []
    for f in _candidate_service_files():
        searched.append(f)
        try:
            if not f.exists():
                continue
            cp = configparser.ConfigParser()
            cp.read(f)
            if service in cp.sections():
                return True, searched
        except Exception:
            # Don't block execution on parse issues; connect() will still try.
            continue
    return False, searched

def ensure_tables(conn) -> None:
    """Create juicer_runs and juicer_readings tables if they do not exist."""
    with conn.cursor() as cur:
        cur.execute(
            """
            CREATE TABLE IF NOT EXISTS juicer_runs (
                run_id          BIGSERIAL PRIMARY KEY,
                time            TIMESTAMPTZ NOT NULL DEFAULT now(),
                nom_vol         INTEGER,
                actual_vol      INTEGER,
                liquid          TEXT,
                n_readings      INTEGER,
                period          DOUBLE PRECISION,
                vol_per_period  DOUBLE PRECISION,
                direction       TEXT,
                complete        BOOLEAN,
                notes           TEXT
            );
            """
        )
        cur.execute(
            """
            CREATE TABLE IF NOT EXISTS juicer_readings (
                reading_id      BIGSERIAL PRIMARY KEY,
                time            TIMESTAMPTZ NOT NULL DEFAULT now(),
                run_id          BIGINT,
                a3              DOUBLE PRECISION,
                a4              DOUBLE PRECISION,
                a5              DOUBLE PRECISION,
                bin_sens_1      BOOLEAN,
                bin_sens_2      BOOLEAN,
                reward_mls      DOUBLE PRECISION,
                mls_remaining   DOUBLE PRECISION
            );
            """
        )
    conn.commit()

def mean_juice_levels(conn, run_id: int) -> dict[str, float | None]:
    """Return mean a3/a4/a5 readings for a run_id."""
    with conn.cursor() as cur:
        cur.execute(
            "SELECT avg(a3), avg(a4), avg(a5) FROM juicer_readings WHERE run_id = %s;",
            (run_id,),
        )
        row = cur.fetchone()
    if not row:
        return {"a3": None, "a4": None, "a5": None}
    return {"a3": row[0], "a4": row[1], "a5": row[2]}

def serial_json_command(ser, payload: dict) -> tuple[dict | None, str]:
    """Send JSON payload to the pump and return (parsed, raw)."""
    cmd = json.dumps(payload, separators=(",", ":")) + "\n"
    ser.reset_input_buffer()
    ser.write(cmd.encode())
    ser.flush()
    raw = ser.readline().decode(errors="replace").strip()
    try:
        return json.loads(raw), raw
    except Exception:
        return None, raw

def _coerce_float_or_none(x):
    """
    Coerce a device-provided value into a float (for DB insertion) or None.

    The pump/firmware may sometimes return sentinel strings like "<50" / ">50"
    (or even bare "<" / ">") for under/over-range on a 0-100-ish scale.
    Postgres rejects those for DOUBLE PRECISION, so we map them to:
      - "<..." -> 0.0
      - ">..." -> 100.0
    """
    if x is None:
        return None
    if isinstance(x, (int, float)):
        return float(x)
    if isinstance(x, str):
        s = x.strip()
        if not s:
            return None
        if s in {"<", "<="}:
            return 0.0
        if s in {">", ">="}:
            return 100.0
        if s[0] in {"<"}:
            return 0.0
        if s[0] in {">"}:
            return 100.0
        try:
            return float(s)
        except ValueError:
            return None
    # Unknown type from device -> NULL
    return None

def queue_run(
    conn,
    *,
    nom_vol: int,
    liquid: str,
    n_readings: int,
    period: float,
    vol_per_period: float,
    direction: str,
    notes: str = "",
) -> int:
    """Insert a pending juicer_run and return its run_id."""
    ensure_tables(conn)
    with conn.cursor() as cur:
        cur.execute(
            """
            INSERT INTO juicer_runs (nom_vol, liquid, n_readings, period, vol_per_period, direction, complete, notes)
            VALUES (%s, %s, %s, %s, %s, %s, false, %s)
            RETURNING run_id;
            """,
            (nom_vol, liquid, n_readings, period, vol_per_period, direction, notes),
        )
        run_id = cur.fetchone()[0]
    conn.commit()
    return run_id

def pending_runs(conn) -> list[dict]:
    """Return all incomplete runs ordered by run_id."""
    ensure_tables(conn)
    with conn.cursor() as cur:
        cur.execute(
            """
            SELECT run_id, nom_vol, liquid, n_readings, period, vol_per_period, direction, notes
            FROM juicer_runs
            WHERE complete = false
            ORDER BY run_id;
            """
        )
        rows = cur.fetchall()
    keys = ["run_id", "nom_vol", "liquid", "n_readings", "period", "vol_per_period", "direction", "notes"]
    return [dict(zip(keys, row)) for row in rows]

def mark_run_complete(conn, run_id: int) -> bool:
    """Mark a run as complete. Returns True if successful."""
    ensure_tables(conn)
    with conn.cursor() as cur:
        cur.execute(
            """
            UPDATE juicer_runs
            SET complete = true
            WHERE run_id = %s
            RETURNING run_id;
            """,
            (run_id,),
        )
        result = cur.fetchone()
    conn.commit()
    return result is not None

def perform_run(
    conn,
    port: Path,
    *,
    nom_vol: int,
    liquid: str,
    n_readings: int,
    period: float,
    vol_per_period: float,
    direction: str,
    notes: str = "",
    run_id: int | None = None,
) -> int:
    """Insert or reuse a juicer_run, command the pump, and insert readings."""
    run_id = run_id or queue_run(
        conn,
        nom_vol=nom_vol,
        liquid=liquid,
        n_readings=n_readings,
        period=period,
        vol_per_period=vol_per_period,
        direction=direction,
        notes=notes,
    )
    ensure_tables(conn)

    try:
        import serial  # type: ignore
    except Exception as e:
        raise RuntimeError(f"pyserial is required for runs: {e}") from e

    try:
        real_port = str(port.resolve())
    except OSError:
        real_port = str(port)

    last_reward_mls = 0.0
    try:
        ser = serial.Serial(real_port, baudrate=2_000_000, timeout=3, write_timeout=3)
    except Exception as e:
        raise RuntimeError(f"Failed to open serial port {real_port}: {e}") from e
    
    try:
        # Give device a moment after opening port
        time.sleep(0.1)
        # Reset counters before starting
        serial_json_command(ser, {"do": "reset"})

        next_at = time.monotonic()
        for idx in range(n_readings):
            payload = {
                "set": {"direction": direction},
                "do": {"reward": vol_per_period},
                "get": ["juice_level", "reward_mls"],
            }
            parsed, raw = serial_json_command(ser, payload)
            if not parsed:
                raise RuntimeError(f"Pump returned non-JSON: {raw}")
            if parsed.get("status") == "failure":
                raise RuntimeError(f"Pump error: {parsed}")

            juice = parsed.get("juice_level") or [None, None, None]
            a3 = _coerce_float_or_none(juice[0] if len(juice) > 0 else None)
            a4 = _coerce_float_or_none(juice[1] if len(juice) > 1 else None)
            a5 = _coerce_float_or_none(juice[2] if len(juice) > 2 else None)
            reward_mls = float(parsed.get("reward_mls", last_reward_mls))
            last_reward_mls = reward_mls
            if (direction or "").lower() == "reverse":
                mls_remaining = float(nom_vol + reward_mls)
            else:
                mls_remaining = float(max(nom_vol - reward_mls, 0))

            with conn.cursor() as cur:
                cur.execute(
                    """
                    INSERT INTO juicer_readings (run_id, a3, a4, a5, bin_sens_1, bin_sens_2, reward_mls, mls_remaining)
                    VALUES (%s, %s, %s, %s, %s, %s, %s, %s);
                    """,
                    (
                        run_id,
                        a3,
                        a4,
                        a5,
                        None,
                        None,
                        reward_mls,
                        mls_remaining,
                    ),
                )
            conn.commit()
            if (idx + 1) % 10 == 0:
                print(
                    f"Reading {idx + 1}/{n_readings} -> "
                    f"run_id={run_id}, a3={a3}, a4={a4}, a5={a5}, "
                    f"reward_mls={reward_mls}, mls_remaining={mls_remaining}"
                )
            if period > 0:
                next_at += period
                # Sleep only the remaining time so loop work doesn't accumulate drift.
                time.sleep(max(0, next_at - time.monotonic()))
        
        # Wait for the last pump operation to complete
        if period > 0:
            print(f"Waiting {period:.1f}s for final pump operation to complete...")
            time.sleep(period + 0.5)
    
    finally:
        ser.close()

    with conn.cursor() as cur:
        cur.execute(
            """
            UPDATE juicer_runs
            SET complete = true
            WHERE run_id = %s;
            """,
            (run_id,),
        )
    conn.commit()
    return run_id

def perform_run_no_db(
    port: Path,
    *,
    nom_vol: int,
    liquid: str,
    n_readings: int,
    period: float,
    vol_per_period: float,
    direction: str,
    notes: str = "",
) -> None:
    """Run the pump and print readings, without any database writes."""
    _ = liquid, notes  # currently unused in no-DB mode
    try:
        import serial  # type: ignore
    except Exception as e:
        raise RuntimeError(f"pyserial is required for runs: {e}") from e

    try:
        real_port = str(port.resolve())
    except OSError:
        real_port = str(port)

    last_reward_mls = 0.0
    try:
        ser = serial.Serial(real_port, baudrate=2_000_000, timeout=3, write_timeout=3)
    except Exception as e:
        raise RuntimeError(f"Failed to open serial port {real_port}: {e}") from e

    try:
        time.sleep(0.1)
        serial_json_command(ser, {"do": "reset"})

        next_at = time.monotonic()
        for idx in range(n_readings):
            payload = {
                "set": {"direction": direction},
                "do": {"reward": vol_per_period},
                "get": ["juice_level", "reward_mls"],
            }
            parsed, raw = serial_json_command(ser, payload)
            if not parsed:
                raise RuntimeError(f"Pump returned non-JSON: {raw}")
            if parsed.get("status") == "failure":
                raise RuntimeError(f"Pump error: {parsed}")

            juice = parsed.get("juice_level") or [None, None, None]
            a3 = _coerce_float_or_none(juice[0] if len(juice) > 0 else None)
            a4 = _coerce_float_or_none(juice[1] if len(juice) > 1 else None)
            a5 = _coerce_float_or_none(juice[2] if len(juice) > 2 else None)
            reward_mls = float(parsed.get("reward_mls", last_reward_mls))
            last_reward_mls = reward_mls
            if (direction or "").lower() == "reverse":
                mls_remaining = float(nom_vol + reward_mls)
            else:
                mls_remaining = float(max(nom_vol - reward_mls, 0))

            if (idx + 1) % 10 == 0 or idx == 0 or idx == n_readings - 1:
                print(
                    f"Reading {idx + 1}/{n_readings} -> "
                    f"a3={a3}, a4={a4}, a5={a5}, "
                    f"reward_mls={reward_mls}, mls_remaining={mls_remaining}"
                )

            if period > 0:
                next_at += period
                time.sleep(max(0, next_at - time.monotonic()))

        if period > 0:
            print(f"Waiting {period:.1f}s for final pump operation to complete...")
            time.sleep(period + 0.5)
    finally:
        ser.close()

CMD = '{"get": ["juice_level"]}\\n'

def find_usb(keyword="juicer") -> list[str]:
    try:
        out = subprocess.run(["lsusb"], capture_output=True, text=True, check=False).stdout
    except FileNotFoundError:
        return []
    return [l.strip() for l in out.splitlines() if keyword.lower() in l.lower()]

def find_serial(keyword="juicer") -> list[Path]:
    base = Path("/dev/serial/by-id")
    return [p for p in base.iterdir() if keyword.lower() in p.name.lower()] if base.exists() else []

def send_command(port: Path) -> str:
    try:
        import serial  # type: ignore
    except Exception as e:
        return f"pyserial missing/unusable: {e}"
    try:
        try:
            real_port = str(port.resolve())
        except OSError:
            real_port = str(port)
        with serial.Serial(real_port, baudrate=2_000_000, timeout=2, write_timeout=2) as ser:
            ser.write(CMD.encode())
            ser.flush()
            reply = ser.readline().decode(errors="replace").strip()
            return reply or "<no response>"
    except Exception as e:
        msg = f"serial error: {e}"
        if "Permission denied" in msg:
            msg += " (try sudo or add your user to the dialout group, then replug)"
        return msg

def _real_port(port: Path) -> str:
    try:
        return str(port.resolve())
    except OSError:
        return str(port)


def calibrate_flow_rate(port: Path, expected_mls: float, actual_mls: float) -> None:
    """Adjust the pump's flow_rate so exp/act equals the scaling factor."""
    try:
        import serial  # type: ignore
    except Exception as e:
        print(f"pyserial required for flow calibration: {e}")
        return

    if expected_mls <= 0 or actual_mls <= 0:
        print("Both --exp and --act must be positive numbers.")
        return

    try:
        real_port = _real_port(port)
        with serial.Serial(real_port, baudrate=2_000_000, timeout=3, write_timeout=3) as ser:
            parsed, raw = serial_json_command(ser, {"get": ["flow_rate"]})
            if not parsed or "flow_rate" not in parsed:
                print(f"Failed to read current flow_rate (response: {raw})")
                return
            current_flow = float(parsed["flow_rate"])

            target_flow = current_flow * (actual_mls / expected_mls)

            parsed, raw = serial_json_command(ser, {"set": {"flow_rate": target_flow}})
            if not parsed:
                print(f"Failed to set flow_rate (response: {raw})")
                return
            status = parsed.get("status", "unknown")
            if status != "success":
                print(f"Setting flow_rate failed ({status}); response: {raw}")
                return

            print(
                "Flow calibration complete:"
                f" current={current_flow:.4f} mL/s,"
                f" expected={expected_mls} mL,"
                f" actual={actual_mls} mL,"
                f" new={target_flow:.4f} mL/s"
            )
    except Exception as exc:
        msg = f"serial error during flow calibration: {exc}"
        if "Permission denied" in msg:
            msg += " (try sudo or add your user to the dialout group, then replug)"
        print(msg)

def main() -> None:
    p = argparse.ArgumentParser(description="Detect juicer device and query juice_level.")
    p.add_argument("--device", help="Serial device path override.")
    p.add_argument("--db-service", help="libpq service name from ~/.pg_service.conf (or set PGSERVICE).")
    # If you pass --db-service, leave these unset so the service can supply them.
    p.add_argument("--db-name", default=None, help="Database name (overrides service).")
    p.add_argument("--db-host", default=None, help="Postgres host (overrides service).")
    p.add_argument("--db-port", type=int, help="Postgres port (overrides service).")
    p.add_argument("--db-user", help="Postgres user override (defaults to PGUSER or system user).")
    p.add_argument("--run", action="store_true", help="Perform a dispensing run and record readings.")
    p.add_argument("--cue", action="store_true", help="Queue a run in the database without executing it.")
    p.add_argument(
        "--run-cued",
        "--run_cued",
        action="store_true",
        help="Execute all queued runs (complete=false) in run_id order.",
    )
    p.add_argument(
        "--calibrate-flow",
        action="store_true",
        help="Adjust the pump flow_rate based on an expected vs actual volume.",
    )
    p.add_argument(
        "--clear-cued",
        "--clear_cued",
        type=int,
        metavar="RUN_ID",
        help="Mark a specific run_id as complete in the database.",
    )
    p.add_argument("--exp", type=float, help="Expected volume (mL) used for flow_rate calibration.")
    p.add_argument("--act", type=float, help="Actual volume (mL) dispensed for flow_rate calibration.")
    p.add_argument("--nom-vol", type=int, help="Nominal volume target (mL).")
    p.add_argument("--liquid", default="water", help="Liquid name.")
    p.add_argument("--n-readings", type=int, default=1, help="Number of reward cycles/readings.")
    p.add_argument("--period", type=float, default=0.0, help="Delay between readings (seconds).")
    p.add_argument("--vol-per-period", type=float, default=0.0, help="Reward volume per period (mL).")
    p.add_argument("--direction", default="forward", help="Pump direction (forward|reverse).")
    p.add_argument("--notes", default="", help="Notes for the run.")
    p.add_argument("--mean-run-id", type=int, help="Print mean a3/a4/a5 for this run_id.")
    args = p.parse_args()

    # Default to a libpq service unless the user explicitly overrides host/dbname.
    if args.db_service is None and args.db_host is None and args.db_name is None and args.db_port is None:
        args.db_service = os.environ.get("PGSERVICE") or DEFAULT_PGSERVICE

    # Preserve historical defaults when not using a service definition.
    if not args.db_service:
        if args.db_name is None:
            args.db_name = "home_automation"
        if args.db_host is None:
            args.db_host = "ryan-analysis"

    exclusive_actions = sum(
        bool(flag)
        for flag in (args.run, args.cue, args.run_cued, args.calibrate_flow, args.clear_cued)
    )
    if exclusive_actions > 1:
        print("Please choose only one of --run, --cue, --run-cued, --calibrate-flow, or --clear-cued.")
        return

    conn = None
    try:
        if args.db_service:
            ok, searched = _service_exists(args.db_service)
            if not ok:
                searched_s = ", ".join(str(p) for p in searched)
                print(
                    f"Reminder: pg_service '{args.db_service}' not found in {searched_s}. "
                    "Create a matching entry (see pg_service.example.conf) or pass --db-host/--db-name."
                )
        conn = connect_home_automation(
            dbname=args.db_name,
            host=args.db_host,
            port=args.db_port,
            user=args.db_user,
            service=args.db_service,
        )
        ensure_tables(conn)
        info = conn.info
        print(
            "DB connection OK "
            f"(dbname={getattr(info, 'dbname', '?')}, "
            f"host={getattr(info, 'host', '?')}, "
            f"port={getattr(info, 'port', '?')}, "
            f"user={getattr(info, 'user', '?')})."
        )
    except Exception as e:
        print(f"DB connect failed: {e}")
        conn = None

    if conn and args.mean_run_id is not None:
        means = mean_juice_levels(conn, args.mean_run_id)
        print(f"Run {args.mean_run_id} mean juice levels: a3={means['a3']}, a4={means['a4']}, a5={means['a5']}")
        if not (args.run or args.run_cued or args.cue or args.calibrate_flow or args.clear_cued):
            conn.close()
            return

    if args.clear_cued:
        if not conn:
            print("Cannot clear run: database not connected.")
            return
        if mark_run_complete(conn, args.clear_cued):
            print(f"Run {args.clear_cued} marked as complete.")
        else:
            print(f"Run {args.clear_cued} not found or already complete.")
        conn.close()
        return

    if args.cue:
        if not conn:
            print("Cannot queue run: database not connected.")
            return
        if args.nom_vol is None or args.vol_per_period <= 0:
            print("nom-vol and vol-per-period are required (>0) when using --cue.")
            return
        run_id = queue_run(
            conn,
            nom_vol=args.nom_vol,
            liquid=args.liquid,
            n_readings=args.n_readings,
            period=args.period,
            vol_per_period=args.vol_per_period,
            direction=args.direction,
            notes=args.notes,
        )
        print(f"Queued run {run_id} (complete=false).")
        conn.close()
        return

    usb = find_usb(); serials = find_serial()
    # print("USB matches:" if usb else "No USB matches.")
    # for l in usb: print(f"- {l}")
    # print("Serial matches:" if serials else "No serial matches.")
    for s in serials:
        try: target = s.resolve()
        except OSError: target = "?"
        # print(f"- {s} -> {target}")

    device = Path(args.device) if args.device else (serials[0] if serials else None)
    if not device:
        print("No device selected; connect juicer and rerun."); return
    print(f"Using device: {device}")

    if args.calibrate_flow:
        if args.exp is None or args.act is None:
            print("Both --exp and --act are required for --calibrate-flow.")
            if conn:
                conn.close()
            return
        calibrate_flow_rate(device, args.exp, args.act)
        if conn:
            conn.close()
        return

    if args.run_cued:
        if not conn:
            print("Cannot run queued jobs: database not connected.")
            return
        pending = pending_runs(conn)
        if not pending:
            print("No queued runs with complete=false.")
            conn.close()
            return
        last_period = 0.0
        for idx, run in enumerate(pending):
            # Only validate that vol_per_period and n_readings are positive
            # nom_vol can be 0 (e.g., for reverse runs or calibration)
            missing = []
            if run.get("vol_per_period") is None or run.get("vol_per_period") <= 0:
                missing.append("vol_per_period")
            if run.get("n_readings") is None or run.get("n_readings") <= 0:
                missing.append("n_readings")
            if run.get("nom_vol") is None:
                missing.append("nom_vol")
            
            if missing:
                print(f"Skipping run {run['run_id']}: missing or invalid values for {', '.join(missing)}.")
                continue
            
            # Give device time to finish last pump operation from previous run
            if idx > 0 and last_period > 0:
                wait_time = last_period + 1.0
                print(f"Waiting {wait_time:.1f} seconds for device to finish pumping...")
                time.sleep(wait_time)
            
            print(f"Running queued run {run['run_id']} ...")
            perform_run(
                conn,
                device,
                nom_vol=run["nom_vol"],
                liquid=run["liquid"],
                n_readings=run["n_readings"],
                period=run["period"],
                vol_per_period=run["vol_per_period"],
                direction=run["direction"] or "forward",
                notes=run.get("notes") or "",
                run_id=run["run_id"],
            )
            print(f"Run {run['run_id']} complete.")
            last_period = run["period"] or 0.0
        if conn:
            conn.close()
        return

    if args.run:
        if args.nom_vol is None or args.vol_per_period <= 0:
            print("nom-vol and vol-per-period are required (>0) when using --run.")
            return
        if conn:
            run_id = perform_run(
                conn,
                device,
                nom_vol=args.nom_vol,
                liquid=args.liquid,
                n_readings=args.n_readings,
                period=args.period,
                vol_per_period=args.vol_per_period,
                direction=args.direction,
                notes=args.notes,
            )
            print(f"Run {run_id} complete.")
        else:
            if not sys.stdin.isatty():
                print("Cannot run: database not connected (non-interactive; refusing to prompt).")
                return
            ans = input("Database not connected. Continue anyway (no DB writes)? [y/N] ").strip().lower()
            if ans not in {"y", "yes"}:
                print("Aborted (database not connected).")
                return
            print("Continuing in no-DB mode (no readings will be recorded).")
            perform_run_no_db(
                device,
                nom_vol=args.nom_vol,
                liquid=args.liquid,
                n_readings=args.n_readings,
                period=args.period,
                vol_per_period=args.vol_per_period,
                direction=args.direction,
                notes=args.notes,
            )
    else:
        print(f"Sending {CMD.strip()} ...")
        print("Response:", send_command(device))

    if conn:
        conn.close()

if __name__ == "__main__":
    main()
