#!/usr/bin/env python3
"""
Juicer firmware environment + flash helper for Debian.

Goals:
- Install arduino-cli user-locally (~/.local/bin).
- Install ESP32 Arduino core (pinned) + required libraries.
- Patch ESP32-S2/S3 Reverse TFT variants so USB product string is "juicer3"
  (so Linux can discover via /dev/serial/by-id/*juicer3*).
- Pull latest firmware from repo main, then interactively detect the newly
  plugged device, compile (with caching), and upload.

This script is intentionally opinionated and interactive; it is designed for
"factory reset Debian -> working firmware upload" with minimal user friction.
"""

from __future__ import annotations

import argparse
import dataclasses
import getpass
import hashlib
import json
import os
import pathlib
import re
import shutil
import subprocess
import sys
import time
from typing import Any, Iterable


ESP32_INDEX_URL = "https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json"


DEFAULT_REPO_URL = "https://github.com/ryanmillerwork/juicer.git"
DEFAULT_REPO_DIR = os.path.expanduser("~/code/juicer")

DEFAULT_ARDUINO_CLI = os.path.expanduser("~/.local/bin/arduino-cli")
DEFAULT_ARDUINO_CFG = os.path.expanduser("~/.arduino15/arduino-cli.yaml")

DEFAULT_CORE = "esp32:esp32"
DEFAULT_CORE_VERSION = "3.1.1"

# Known boards we care about.
FQBN_S3 = "esp32:esp32:adafruit_feather_esp32s3_reversetft"
FQBN_S2 = "esp32:esp32:adafruit_feather_esp32s2_reversetft"

# Variant dirs for patching USB_PRODUCT.
VARIANT_S3 = "adafruit_feather_esp32s3_reversetft"
VARIANT_S2 = "adafruit_feather_esp32s2_reversetft"
VARIANT_S2_JUICER = "adafruit_feather_esp32s2_reversetft_juicer"


LIBS = [
    # Prefer pinned versions when possible (Arduino CLI supports "Name@version").
    ("Adafruit GFX Library", "1.12.0"),
    ("Adafruit ST7735 and ST7789 Library", "1.10.4"),
    ("Adafruit NeoPixel", "1.12.5"),
    ("ArduinoJson", "7.2.0"),
]


APT_PACKAGES = [
    "git",
    "curl",
    "ca-certificates",
    "udev",
]


@dataclasses.dataclass(frozen=True)
class PortInfo:
    address: str
    vid: str | None = None
    pid: str | None = None
    serial: str | None = None
    protocol: str | None = None
    fqbn_candidates: tuple[str, ...] = ()


def eprint(*args: object) -> None:
    print(*args, file=sys.stderr)


class CmdError(RuntimeError):
    pass


def run(
    args: list[str],
    *,
    check: bool = True,
    capture: bool = False,
    env: dict[str, str] | None = None,
    cwd: str | None = None,
) -> subprocess.CompletedProcess[str]:
    """Run a command with nice errors. Always uses text mode."""
    try:
        return subprocess.run(
            args,
            check=check,
            text=True,
            capture_output=capture,
            env=env,
            cwd=cwd,
        )
    except subprocess.CalledProcessError as ex:
        out = ex.stdout or ""
        err = ex.stderr or ""
        msg = f"Command failed: {' '.join(args)}\n\nstdout:\n{out}\n\nstderr:\n{err}".strip()
        raise CmdError(msg) from ex


def detect_repo_dir_fallback() -> str:
    """
    If we're being run from inside a git checkout that looks like the juicer repo,
    prefer that checkout as the default --repo-dir.

    This prevents surprising behavior where running the script from one clone
    still uses (or clones into) ~/code/juicer due to the historical default.
    """
    try:
        cp = subprocess.run(
            ["git", "rev-parse", "--show-toplevel"],
            check=True,
            text=True,
            capture_output=True,
        )
        top = (cp.stdout or "").strip()
        if not top:
            return DEFAULT_REPO_DIR
        # Heuristic: repo root should contain the firmware sketch directory.
        if os.path.isdir(os.path.join(top, "juice_pump3")):
            return top
    except Exception:
        pass
    return DEFAULT_REPO_DIR


def prompt(msg: str) -> None:
    input(f"\n{msg}\nPress Enter to continue...")


def confirm(msg: str, *, default_yes: bool = False) -> bool:
    suffix = "[Y/n]" if default_yes else "[y/N]"
    ans = input(f"{msg} {suffix} ").strip().lower()
    if not ans:
        return default_yes
    return ans in ("y", "yes")


def ensure_not_root() -> None:
    if os.geteuid() == 0:
        raise SystemExit("Do not run this as root; it installs user-local tools into ~/.local.")


def which(cmd: str) -> str | None:
    return shutil.which(cmd)


def sudo_available() -> bool:
    return which("sudo") is not None


def apt_install(packages: list[str]) -> None:
    if not sudo_available():
        raise SystemExit("sudo is required to install apt dependencies, but was not found.")
    # Don't capture output here: sudo may need to prompt for a password, and
    # capturing output can prevent it from using the controlling TTY.
    run(["sudo", "-v"], check=True)
    run(["sudo", "apt-get", "update"], check=True)
    run(["sudo", "apt-get", "install", "-y", *packages], check=True)


def in_group(group: str) -> bool:
    # Conservative: check current effective groups.
    try:
        import grp
        import pwd

        user = getpass.getuser()
        gids = os.getgroups()
        g = grp.getgrnam(group)
        if g.gr_gid in gids:
            return True
        # Some systems report supplementary groups only via group member list.
        return user in g.gr_mem
    except Exception:
        return False


def ensure_dialout_membership() -> None:
    if in_group("dialout"):
        return

    if not sudo_available():
        raise SystemExit("Need sudo to add you to dialout group, but sudo was not found.")

    user = getpass.getuser()
    eprint(f"Adding user {user!r} to dialout group (required for /dev/ttyACM* access).")
    run(["sudo", "usermod", "-a", "-G", "dialout", user], check=True)
    raise SystemExit(
        "Added you to dialout. Please log out and log back in (or reboot), then re-run this script."
    )


def ensure_dir(path: str) -> str:
    pathlib.Path(path).mkdir(parents=True, exist_ok=True)
    return path


def ensure_arduino_cli_config(cfg_path: str) -> None:
    cfg_file = pathlib.Path(cfg_path)
    cfg_file.parent.mkdir(parents=True, exist_ok=True)

    if not cfg_file.exists():
        cfg_file.write_text(
            "board_manager:\n"
            "  additional_urls:\n"
            f"    - {ESP32_INDEX_URL}\n",
            encoding="utf-8",
        )
        return

    txt = cfg_file.read_text(encoding="utf-8", errors="replace")
    if ESP32_INDEX_URL in txt:
        return

    # Don't try to be clever editing YAML without a parser; just append a fresh
    # section that Arduino CLI can read. If the file already has board_manager,
    # Arduino CLI will effectively see the last one for our added URL.
    txt += f"\nboard_manager:\n  additional_urls:\n    - {ESP32_INDEX_URL}\n"

    cfg_file.write_text(txt, encoding="utf-8")


def machine_arch() -> str:
    try:
        cp = subprocess.run(["uname", "-m"], check=True, text=True, capture_output=True)
        return (cp.stdout or "").strip()
    except Exception:
        return ""


def is_armv6(arch: str) -> bool:
    a = (arch or "").lower()
    return "armv6" in a


def ensure_arduino_cli(
    arduino_cli: str,
    *,
    yes: bool = False,
    allow_source_build: bool = False,
) -> str:
    """
    Ensure arduino-cli is available and return the path to use.

    On some platforms (notably Linux ARMv6), Arduino does not publish prebuilt
    arduino-cli releases, so the official install.sh fails. In that case we
    fall back to apt, and finally to building from source with Go.
    """
    if os.path.exists(arduino_cli):
        return arduino_cli

    existing = which("arduino-cli")
    if existing:
        return existing

    if which("curl") is None:
        raise SystemExit("curl is required to install arduino-cli. Please install it and re-run.")

    # First attempt: official installer (works on most architectures).
    ensure_dir(os.path.dirname(arduino_cli))
    eprint("Installing arduino-cli to ~/.local/bin ...")
    cmd = (
        "curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh"
        f' | sh -s -- -b "{os.path.dirname(arduino_cli)}"'
    )
    try:
        run(["bash", "-lc", cmd], check=True)
    except CmdError as ex:
        # The upstream install.sh has historically been fragile across
        # distros/architectures (notably Raspberry Pi OS ARM64). Prefer a
        # distro package fallback whenever possible before giving up.
        if sudo_available() and confirm("arduino-cli installer failed. Try installing arduino-cli via apt-get instead?", default_yes=yes):
            try:
                apt_install(["arduino-cli"])
            except CmdError:
                # If apt also fails, continue to architecture-specific fallbacks below.
                pass
            existing2 = which("arduino-cli")
            if existing2:
                return existing2

        arch = machine_arch()
        if not is_armv6(arch):
            raise

        # ARMv6 fallback: try distro package first.
        eprint(f"Note: official arduino-cli installer failed on architecture {arch!r}.")
        if sudo_available() and confirm("Try installing arduino-cli via apt-get instead?", default_yes=yes):
            try:
                apt_install(["arduino-cli"])
            except CmdError:
                # We'll try the Go build fallback next.
                pass
            existing3 = which("arduino-cli")
            if existing3:
                return existing3

        if not allow_source_build:
            raise SystemExit(
                "arduino-cli could not be installed automatically on this ARMv6 machine.\n"
                "That's OK—this flashing workflow is best run on a more common host (e.g. x86_64),\n"
                "or you can install arduino-cli manually and re-run.\n"
                "If you *really* want to attempt a source build here, re-run with: --build-arduino-cli-from-source"
            ) from ex

        # Last resort (explicit opt-in): build from source (requires Go).
        if which("go") is None:
            if sudo_available() and confirm("Install Go toolchain (golang-go) via apt-get to build arduino-cli?", default_yes=yes):
                apt_install(["golang-go"])
            else:
                raise SystemExit(
                    "arduino-cli install failed on ARMv6 and Go was not available.\n"
                    "Install either `arduino-cli` (if available) or `golang-go`, then re-run."
                ) from ex

        eprint("Building arduino-cli from source (Go) ...")
        env = dict(os.environ)
        env["GOBIN"] = os.path.dirname(arduino_cli)
        # In recent releases, the main package is at the module root.
        run(["go", "install", "github.com/arduino/arduino-cli@latest"], check=True, env=env)

    if os.path.exists(arduino_cli):
        return arduino_cli

    existing4 = which("arduino-cli")
    if existing4:
        return existing4

    raise SystemExit("arduino-cli installation did not produce a usable binary (neither at the requested path nor on PATH).")


def arduino_env(tmpdir: str) -> dict[str, str]:
    env = dict(os.environ)
    env["TMPDIR"] = tmpdir
    return env


def arduino_cli_cmd(arduino_cli: str, *args: str) -> list[str]:
    return [arduino_cli, *args]


def ensure_esp32_core(arduino_cli: str, core_version: str, *, tmpdir: str) -> None:
    ensure_arduino_cli_config(DEFAULT_ARDUINO_CFG)
    env = arduino_env(tmpdir)
    run(arduino_cli_cmd(arduino_cli, "core", "update-index"), env=env, check=True)
    run(arduino_cli_cmd(arduino_cli, "core", "install", f"{DEFAULT_CORE}@{core_version}"), env=env, check=True)


def try_install_lib(arduino_cli: str, name: str, version: str, *, tmpdir: str) -> None:
    env = arduino_env(tmpdir)

    # First try pinned form.
    pinned = f"{name}@{version}"
    cp = subprocess.run(
        arduino_cli_cmd(arduino_cli, "lib", "install", pinned),
        text=True,
        capture_output=True,
        env=env,
    )
    if cp.returncode == 0:
        return

    # Fall back to unpinned.
    eprint(f"Warning: could not install {pinned!r} (will try unpinned).")
    run(arduino_cli_cmd(arduino_cli, "lib", "install", name), env=env, check=True)


def ensure_libraries(arduino_cli: str, *, tmpdir: str) -> None:
    for name, version in LIBS:
        try_install_lib(arduino_cli, name, version, tmpdir=tmpdir)


def esp32_core_dir(core_version: str) -> str:
    return os.path.expanduser(f"~/.arduino15/packages/esp32/hardware/esp32/{core_version}")


def set_usb_product(pins_path: str, product: str) -> None:
    p = pathlib.Path(pins_path)
    if not p.exists():
        raise CmdError(f"Expected file not found: {pins_path}")

    txt = p.read_text(encoding="utf-8", errors="replace")
    new_line = f'#define USB_PRODUCT "{product}"'

    if re.search(r'^\s*#define\s+USB_PRODUCT\s+".*"\s*$', txt, flags=re.M):
        txt2 = re.sub(
            r'^\s*#define\s+USB_PRODUCT\s+".*"\s*$',
            new_line,
            txt,
            flags=re.M,
        )
    else:
        # Insert near top; pins_arduino.h always has some header lines.
        lines = txt.splitlines()
        insert_at = 0
        for i, line in enumerate(lines[:40]):
            if line.strip().startswith("#define"):
                insert_at = i
                break
        lines.insert(insert_at, new_line)
        txt2 = "\n".join(lines) + ("\n" if txt.endswith("\n") else "")

    if txt2 != txt:
        p.write_text(txt2, encoding="utf-8")


def patch_variants_usb_product(core_version: str, product: str) -> None:
    core = esp32_core_dir(core_version)
    variants = pathlib.Path(core) / "variants"

    # Always patch the base variants (keeps board auto-detect unambiguous).
    targets: list[pathlib.Path] = [
        variants / VARIANT_S3 / "pins_arduino.h",
        variants / VARIANT_S2 / "pins_arduino.h",
    ]

    # If the custom S2 juicer variant exists (some systems), patch it too.
    targets.append(variants / VARIANT_S2_JUICER / "pins_arduino.h")

    patched = 0
    for f in targets:
        if f.exists():
            set_usb_product(str(f), product)
            patched += 1

    if patched == 0:
        raise CmdError(
            "No expected variant pins_arduino.h files were found to patch. "
            f"Is ESP32 core {core_version} installed?"
        )


def ensure_repo(repo_url: str, repo_dir: str) -> None:
    p = pathlib.Path(repo_dir)
    if not p.exists():
        ensure_dir(str(p.parent))
        run(["git", "clone", repo_url, repo_dir], check=True)

    # Always update main as requested.
    run(["git", "-C", repo_dir, "fetch", "origin"], check=True)
    run(["git", "-C", repo_dir, "checkout", "main"], check=True)
    run(["git", "-C", repo_dir, "pull", "--ff-only", "origin", "main"], check=True)


def git_sha(repo_dir: str) -> str:
    cp = run(["git", "-C", repo_dir, "rev-parse", "HEAD"], check=True, capture=True)
    return (cp.stdout or "").strip()


def read_by_id_links() -> dict[str, str]:
    base = pathlib.Path("/dev/serial/by-id")
    if not base.exists():
        return {}
    out: dict[str, str] = {}
    for entry in base.iterdir():
        try:
            out[str(entry)] = os.path.realpath(str(entry))
        except Exception:
            continue
    return out


def list_ports_via_arduino(arduino_cli: str, *, tmpdir: str) -> list[PortInfo]:
    env = arduino_env(tmpdir)
    cp = run(arduino_cli_cmd(arduino_cli, "board", "list", "--format", "json"), env=env, check=True, capture=True)
    data = json.loads(cp.stdout or "{}")
    ports: list[PortInfo] = []
    for item in data.get("detected_ports", []) or []:
        port = item.get("port", {}) or {}
        props = port.get("properties", {}) or {}
        boards = item.get("matching_boards", []) or []
        ports.append(
            PortInfo(
                address=str(port.get("address") or port.get("label") or ""),
                vid=str(props.get("vid")) if props.get("vid") else None,
                pid=str(props.get("pid")) if props.get("pid") else None,
                serial=str(props.get("serialNumber")) if props.get("serialNumber") else None,
                protocol=str(port.get("protocol")) if port.get("protocol") else None,
                fqbn_candidates=tuple(b.get("fqbn") for b in boards if b.get("fqbn")),
            )
        )
    return [p for p in ports if p.address]


def pick_new_device(
    before_ports: list[PortInfo],
    after_ports: list[PortInfo],
    before_byid: dict[str, str],
    after_byid: dict[str, str],
) -> tuple[str, str | None]:
    before_set = {p.address for p in before_ports}
    after_set = {p.address for p in after_ports}
    new_ports = sorted(after_set - before_set)

    before_links = set(before_byid.keys())
    after_links = set(after_byid.keys())
    new_links = sorted(after_links - before_links)

    chosen_port: str | None = None
    chosen_byid: str | None = None

    # Prefer new /dev/serial/by-id entry containing juicer3
    juicer_links = [p for p in new_links if "juicer3" in p.lower()]
    if juicer_links:
        chosen_byid = juicer_links[0]
        chosen_port = after_byid.get(chosen_byid)

    if not chosen_port and new_ports:
        chosen_port = new_ports[0]

    if not chosen_port:
        raise CmdError(
            "Could not identify a newly plugged device.\n"
            f"- New ports: {new_ports}\n"
            f"- New by-id links: {new_links}\n"
            "Try unplugging other serial devices and re-running."
        )

    return chosen_port, chosen_byid


def fqbn_for_port(port: str, after_ports: list[PortInfo]) -> str:
    # If Arduino CLI already matched a board, use that.
    for p in after_ports:
        if p.address == port and p.fqbn_candidates:
            # If multiple, prefer our known ones.
            if FQBN_S3 in p.fqbn_candidates:
                return FQBN_S3
            if FQBN_S2 in p.fqbn_candidates:
                return FQBN_S2
            return p.fqbn_candidates[0]

    # Fallback by VID/PID heuristics.
    for p in after_ports:
        if p.address != port:
            continue
        vid = (p.vid or "").lower()
        pid = (p.pid or "").lower()
        if vid == "0x239a" and pid == "0x8123":
            return FQBN_S3
        if vid == "0x239a" and pid in ("0x80ed", "0x00ed", "0x80ee"):
            return FQBN_S2

    # Last resort: assume S3 (matches CI and is most common in current repo notes).
    return FQBN_S3


def cache_key(*parts: str) -> str:
    h = hashlib.sha256()
    for p in parts:
        h.update(p.encode("utf-8"))
        h.update(b"\0")
    return h.hexdigest()[:16]


def compile_sketch(
    arduino_cli: str,
    fqbn: str,
    sketch_dir: str,
    build_dir: str,
    *,
    tmpdir: str,
) -> None:
    env = arduino_env(tmpdir)
    run(
        arduino_cli_cmd(
            arduino_cli,
            "compile",
            "--fqbn",
            fqbn,
            "--build-path",
            build_dir,
            sketch_dir,
        ),
        env=env,
        check=True,
    )


def upload_build(
    arduino_cli: str,
    fqbn: str,
    port: str,
    build_dir: str,
    *,
    tmpdir: str,
) -> None:
    env = arduino_env(tmpdir)
    run(
        arduino_cli_cmd(
            arduino_cli,
            "upload",
            "--fqbn",
            fqbn,
            "--port",
            port,
            "--input-dir",
            build_dir,
        ),
        env=env,
        check=True,
    )


def main() -> None:
    ensure_not_root()

    default_repo_dir = detect_repo_dir_fallback()

    ap = argparse.ArgumentParser(description="Set up toolchain and flash Juicer firmware to ESP32-S2/S3.")
    ap.add_argument("--repo-url", default=DEFAULT_REPO_URL, help="Git repo URL to clone/pull.")
    ap.add_argument("--repo-dir", default=default_repo_dir, help="Local repo directory.")
    ap.add_argument("--arduino-cli", default=DEFAULT_ARDUINO_CLI, help="Path to arduino-cli binary.")
    ap.add_argument("--core-version", default=DEFAULT_CORE_VERSION, help="ESP32 Arduino core version to install.")
    ap.add_argument("--product", default="juicer3", help='USB product string (default: "juicer3").')
    ap.add_argument("--cache-dir", default=os.path.expanduser("~/.cache/juicer"), help="Build cache dir.")
    ap.add_argument("--force-rebuild", action="store_true", help="Always rebuild (ignore cache).")
    ap.add_argument("--skip-apt", action="store_true", help="Skip apt dependency install.")
    ap.add_argument("--skip-pull", action="store_true", help="Skip git pull (use existing repo state).")
    ap.add_argument(
        "--build-arduino-cli-from-source",
        action="store_true",
        help="(Advanced) If arduino-cli can't be installed normally (e.g. ARMv6), try building it from source with Go.",
    )
    ap.add_argument("--yes", action="store_true", help="Non-interactive: assume yes for prompts where safe.")
    args = ap.parse_args()

    tmpdir = ensure_dir(os.path.join(args.cache_dir, "tmp"))

    # 1) apt deps
    if not args.skip_apt:
        missing_cmds = [c for c in ("git", "curl") if which(c) is None]
        if missing_cmds:
            if not confirm(
                f"Missing required commands {missing_cmds}. Install via apt-get?",
                default_yes=args.yes,
            ):
                raise SystemExit("Cannot proceed without dependencies.")
            apt_install(APT_PACKAGES)

    # 2) dialout group
    ensure_dialout_membership()

    # 3) arduino-cli + core + libs
    args.arduino_cli = ensure_arduino_cli(
        args.arduino_cli,
        yes=args.yes,
        allow_source_build=args.build_arduino_cli_from_source,
    )
    ensure_esp32_core(args.arduino_cli, args.core_version, tmpdir=tmpdir)
    ensure_libraries(args.arduino_cli, tmpdir=tmpdir)

    # 4) patch USB product strings
    if args.product != "juicer3":
        eprint("Note: only 'juicer3' has been validated with existing docs; proceeding anyway.")
    patch_variants_usb_product(args.core_version, args.product)

    # 5) repo update
    if not args.skip_pull:
        ensure_repo(args.repo_url, args.repo_dir)

    sketch_dir = os.path.join(args.repo_dir, "juice_pump3")
    if not os.path.isdir(sketch_dir):
        raise SystemExit(f"Expected sketch dir not found: {sketch_dir}")

    # 6) unplug/plug detection
    prompt("UNPLUG the Juicer device from USB.")
    before_ports = list_ports_via_arduino(args.arduino_cli, tmpdir=tmpdir)
    before_byid = read_by_id_links()

    prompt("PLUG IN the Juicer device via USB.")
    time.sleep(0.75)
    after_ports = list_ports_via_arduino(args.arduino_cli, tmpdir=tmpdir)
    after_byid = read_by_id_links()

    port, byid = pick_new_device(before_ports, after_ports, before_byid, after_byid)
    fqbn = fqbn_for_port(port, after_ports)

    eprint("\nDetected device:")
    eprint(f"- Port: {port}")
    if byid:
        eprint(f"- By-id: {byid} -> {os.path.realpath(byid)}")
    eprint(f"- FQBN: {fqbn}")

    if not confirm("Proceed with compile + upload?", default_yes=args.yes):
        raise SystemExit("Cancelled.")

    # 7) build cache
    sha = git_sha(args.repo_dir)
    key = cache_key(sha, fqbn, args.core_version)
    build_dir = ensure_dir(os.path.join(args.cache_dir, "build", key))
    meta_path = os.path.join(build_dir, "juicer_build_meta.json")
    bin_path = os.path.join(build_dir, "juice_pump3.ino.bin")

    meta = {
        "repo_url": args.repo_url,
        "repo_dir": args.repo_dir,
        "git_sha": sha,
        "fqbn": fqbn,
        "esp32_core": f"{DEFAULT_CORE}@{args.core_version}",
        "usb_product": args.product,
        "sketch_dir": sketch_dir,
        "built_at": None,
    }

    cache_ok = False
    if not args.force_rebuild and os.path.exists(meta_path) and os.path.exists(bin_path):
        try:
            old = json.loads(pathlib.Path(meta_path).read_text(encoding="utf-8"))
            cache_ok = (
                old.get("git_sha") == sha
                and old.get("fqbn") == fqbn
                and old.get("esp32_core") == meta["esp32_core"]
                and old.get("usb_product") == meta["usb_product"]
            )
        except Exception:
            cache_ok = False

    if cache_ok:
        eprint(f"Using cached build: {bin_path}")
    else:
        eprint("Compiling firmware (this may take a few minutes on first run)...")
        compile_sketch(args.arduino_cli, fqbn, sketch_dir, build_dir, tmpdir=tmpdir)
        meta["built_at"] = time.strftime("%Y-%m-%dT%H:%M:%S%z")
        pathlib.Path(meta_path).write_text(json.dumps(meta, indent=2) + "\n", encoding="utf-8")

        if not os.path.exists(bin_path):
            eprint(f"Warning: expected binary not found at {bin_path} (upload may still work via build dir).")

    # 8) upload
    eprint("Uploading firmware...")
    upload_build(args.arduino_cli, fqbn, port, build_dir, tmpdir=tmpdir)

    print("\nDone.")
    print('Tip: verify detection with `ls -l /dev/serial/by-id/*juicer3*` and/or run `python3 test_connection.py`.')


if __name__ == "__main__":
    try:
        main()
    except CmdError as ex:
        eprint("\nERROR:")
        eprint(str(ex))
        sys.exit(2)

