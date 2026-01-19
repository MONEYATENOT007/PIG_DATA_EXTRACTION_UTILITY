# file: multi_board_sd_automation_gui.py
# Requires: PyQt5, pyserial, psutil, tqdm
# Persisted config: boards_config.json (serial->name,type,exclude map)

import os
import sys
import time
import json
import shutil
import subprocess
import serial
import serial.tools.list_ports
from serial import SerialException
from tqdm import tqdm
import psutil
import ctypes
import threading
from concurrent.futures import ThreadPoolExecutor, as_completed
import logging
import atexit
import traceback
from datetime import datetime
import re
from typing import Optional, Dict, Set, List, Tuple

# ---------------------------- CONFIG PATH ----------------------------
def get_config_path() -> str:
    if getattr(sys, "frozen", False):
        base = os.path.dirname(os.path.abspath(sys.executable))
    else:
        base = os.path.dirname(os.path.abspath(__file__))
    return os.path.join(base, "boards_config.json")

CONFIG_PATH = get_config_path()

def load_registry() -> Dict[str, Dict]:
    try:
        if os.path.exists(CONFIG_PATH):
            with open(CONFIG_PATH, "r", encoding="utf-8") as f:
                data = json.load(f)
                if isinstance(data, dict) and "serial_to_board" in data:
                    return data
    except Exception:
        pass
    return {"version": 3, "serial_to_board": {}}  # v2 adds "exclude", v3 adds "pipe_size"

def save_registry(reg: Dict[str, Dict]) -> None:
    try:
        os.makedirs(os.path.dirname(CONFIG_PATH), exist_ok=True)
        tmp = CONFIG_PATH + ".tmp"
        with open(tmp, "w", encoding="utf-8") as f:
            json.dump(reg, f, indent=2)
            f.flush()
            try:
                os.fsync(f.fileno())
            except Exception:
                pass
        try:
            if os.path.exists(CONFIG_PATH):
                shutil.copy2(CONFIG_PATH, CONFIG_PATH + ".bak")
        except Exception:
            pass
        os.replace(tmp, CONFIG_PATH)
    except Exception as e:
        print(f"[WARN] Failed to save registry: {e}", flush=True)

REGISTRY = load_registry()

# ---------------------------- RESOURCES ----------------------------
def resource_path(relative_path: str) -> str:
    """
    Resolve a resource path for both source and frozen (PyInstaller) modes.
    Preference order:
      1) Next to the executable (for frozen) or CWD (for source)
      2) PyInstaller _MEIPASS unpack directory (if present)
    This allows keeping firmware/config outside the EXE while still supporting
    older bundles that ship them inside _MEIPASS.
    """
    base_candidates: List[str] = []
    if getattr(sys, "frozen", False):
        # Prefer files next to the executable when frozen
        base_candidates.append(os.path.dirname(os.path.abspath(sys.executable)))
    # Also consider current working directory
    base_candidates.append(os.path.abspath("."))
    # Finally, consider the PyInstaller unpack directory if available
    if hasattr(sys, "_MEIPASS"):
        base_candidates.append(getattr(sys, "_MEIPASS"))

    for base in base_candidates:
        candidate = os.path.join(base, relative_path)
        if os.path.exists(candidate):
            return candidate

    # Fallback: first base + relative path
    if base_candidates:
        return os.path.join(base_candidates[0], relative_path)
    return relative_path

DATA_FIRMWARE = resource_path("firmware/DATA.uf2")
EGP_FIRMWARE  = resource_path("firmware/EGP.uf2")
MFL_FIRMWARE  = resource_path("firmware/MFL.uf2")
CMFL_FIRMWARE = resource_path("firmware/CMFL.uf2")

# ---------------------------- GLOBALS ----------------------------
RUN_START = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
MAX_LABEL_LEN = 11

MODE_CURRENT: Optional[str] = None
BOARD_PORTS_CURRENT: Dict[str, str] = {}        # board_name -> COMx
BOARD_TYPES_CURRENT: Dict[str, str] = {}        # board_name -> type key
BOARD_EXCLUDE_SLOTS: Dict[str, Set[int]] = {}   # board_name -> excluded slots from registry
BOARD_PIPE_SIZES: Dict[str, int] = {}           # board_name -> pipe size in inches
RUN_SELECTION: Optional[Dict[str, Set[int]]] = None

BOARD_SLOT_LIMITS = {"A-MFL": 5, "C-MFL": 8, "EGP": 5}

PROG_TOTAL_SLOTS = 0
PROG_SLOTS_DONE: Set[Tuple[str, int]] = set()

# Dead ports/boards (with 5s grace on first failure)
DEAD_PORTS: Set[str] = set()
DEAD_BOARDS: Set[str] = set()
_DEAD_LOCK = threading.RLock()

# Cancel control
CANCEL_EVENT = threading.Event()

# File selection config (data extraction)
FILE_SLICE_LOCK = threading.RLock()
FILE_SLICE_CONFIG = {"enabled": False, "offset": 0, "count": 0, "tail": 0}

def set_file_slice_config(enabled: bool, offset: int, count: int, tail: int) -> None:
    """Persist the current custom file selection settings (thread-safe)."""
    with FILE_SLICE_LOCK:
        FILE_SLICE_CONFIG["enabled"] = bool(enabled)
        FILE_SLICE_CONFIG["offset"] = max(0, int(offset or 0))
        FILE_SLICE_CONFIG["count"] = max(0, int(count or 0))
        FILE_SLICE_CONFIG["tail"] = max(0, int(tail or 0))

def get_file_slice_config() -> Dict[str, int]:
    with FILE_SLICE_LOCK:
        return dict(FILE_SLICE_CONFIG)

def _norm_port(port_name: str) -> str:
    return (port_name or "").strip().upper()

def reset_dead_state() -> None:
    with _DEAD_LOCK:
        DEAD_PORTS.clear()
        DEAD_BOARDS.clear()

def is_board_dead(board: str) -> bool:
    with _DEAD_LOCK:
        return board in DEAD_BOARDS

def mark_port_dead(port: str, reason: str = "") -> None:
    portn = _norm_port(port)
    if not portn:
        return
    with _DEAD_LOCK:
        if portn in DEAD_PORTS:
            return
        DEAD_PORTS.add(portn)
        boards = [b for b, p in BOARD_PORTS_CURRENT.items() if _norm_port(p) == portn]
        for b in boards:
            if b not in DEAD_BOARDS:
                DEAD_BOARDS.add(b)
                log(f"[{b}] Marked DEAD (port {portn}): {reason or 'serial disconnect'}")

def port_exists_now(port_name: str) -> bool:
    target = _norm_port(port_name)
    if not target:
        return False
    for p in serial.tools.list_ports.comports():
        if _norm_port(p.device) == target:
            return True
    return False

def wait_for_port_back(port_name: str, timeout_sec: float = 5.0, interval: float = 0.25) -> bool:
    end = time.time() + max(0.0, timeout_sec)
    while time.time() < end:
        if port_exists_now(port_name):
            return True
        time.sleep(interval)
    return False

# ---------------------------- STATUS TRACKER ----------------------------
class StatusTracker:
    def __init__(self):
        self._lock = threading.RLock()
        self.data: Dict[str, Dict[str, Dict[str, Dict[str, str]]]] = {}

    def clear(self):
        with self._lock:
            self.data.clear()

    def _ensure(self, board: str, slot: int, phase: str) -> None:
        s = str(slot)
        with self._lock:
            self.data.setdefault(board, {})
            self.data[board].setdefault(s, {})
            self.data[board][s].setdefault(phase, {"status": "pending", "err": ""})

    def mark(self, board: str, slot: int, phase: str, status: str, err: str = "") -> None:
        self._ensure(board, slot, phase)
        with self._lock:
            self.data[board][str(slot)][phase]["status"] = status
            self.data[board][str(slot)][phase]["err"] = err

    def snapshot(self) -> Dict[str, Dict[str, Dict[str, Dict[str, str]]]]:
        with self._lock:
            return json.loads(json.dumps(self.data))

    @staticmethod
    def slot_done(option_mode: str, phases: Dict[str, Dict[str, str]]) -> bool:
        g = lambda k: (phases.get(k, {}) or {}).get("status", "")
        if option_mode in ("DATA", "DATA_PREF", "DATA_LOG", "DATA_LOG_PREF"):
            return g("copy") == "success"
        if option_mode in ("MFL", "MFL_PREF"):
            return g("format") == "success"
        if option_mode in ("MFL_ALL_SLOTS", "CMFL_ALL_SLOTS", "EGP_ALL_SLOTS"):
            return g("mfl_upload") == "success"
        if option_mode == "AUTO_FORMAT_BURN":
            return g("format") == "success" and g("mfl_upload") == "success"
        if option_mode == "AUTO_FORMAT_BURN_EGP":
            return g("format") == "success" and g("mfl_upload") == "success"
        if option_mode == "LABEL":
            return g("label") == "success"
        if option_mode in ("ODO", "INLB"):
            return g("special") == "success"
        return False

    def summarize(self, option_mode: str) -> Dict[str, object]:
        snap = self.snapshot()
        boards_total = len(snap)
        slots_total  = sum(len(slots) for slots in snap.values())
        slots_ok = 0
        failures: List[Tuple[str, int, str]] = []
        boards_ok: Set[str] = set()
        boards_fail: Set[str] = set()

        for b, slots in snap.items():
            board_all_ok = True
            for s, phases in slots.items():
                if self.slot_done(option_mode, phases):
                    slots_ok += 1
                else:
                    board_all_ok = False
                    reasons = []
                    for ph, rec in phases.items():
                        if rec.get("status") == "failed":
                            reasons.append(f"{ph}: {rec.get('err') or 'failed'}")
                    if not reasons:
                        reasons.append("incomplete")
                    try:
                        s_int = int(s)
                    except Exception:
                        s_int = 0
                    failures.append((b, s_int, "; ".join(reasons)))
            if board_all_ok and slots:
                boards_ok.add(b)
            else:
                boards_fail.add(b)

        return {
            "boards_total": boards_total,
            "boards_ok": len(boards_ok),
            "boards_fail": len(boards_fail - boards_ok),
            "slots_total": slots_total,
            "slots_ok": slots_ok,
            "slots_fail": slots_total - slots_ok,
            "failures": failures
        }

TRACKER = StatusTracker()

# ---------------------------- LOGGING ----------------------------
LOG_DIR = os.path.join(os.path.abspath("."), "Extracted_Data")
os.makedirs(LOG_DIR, exist_ok=True)
RUN_TS = datetime.now().strftime("%Y%m%d_%H%M%S")
TIMESTAMPED_LOG = os.path.join(LOG_DIR, f"log_{RUN_TS}.txt")
LATEST_LOG      = os.path.join(LOG_DIR, "log_latest.txt")
SIMPLE_LOG      = os.path.join(LOG_DIR, f"simple_log_{RUN_TS}.txt")

_logger = logging.getLogger("mbsd")
_logger.setLevel(logging.INFO)
_handler_ts = logging.FileHandler(TIMESTAMPED_LOG, encoding="utf-8")
_handler_ts.setLevel(logging.INFO)
_handler_ts.setFormatter(logging.Formatter("%(asctime)s | %(levelname)s | %(message)s"))
_logger.addHandler(_handler_ts)
_handler_latest = logging.FileHandler(LATEST_LOG, encoding="utf-8")
_handler_latest.setLevel(logging.INFO)
_handler_latest.setFormatter(logging.Formatter("%(asctime)s | %(levelname)s | %(message)s"))
_logger.addHandler(_handler_latest)
_logger.propagate = False

def _flush_logs() -> None:
    for h in list(_logger.handlers):
        try:
            h.flush()
        except Exception:
            pass
atexit.register(_flush_logs)

def _excepthook(exc_type, exc, tb):
    try:
        _logger.error("UNCAUGHT EXCEPTION:\n%s", "".join(traceback.format_exception(exc_type, exc, tb)))
        _logger.error("STATE SNAPSHOT (on crash): %s", json.dumps(TRACKER.snapshot(), indent=2))
        _flush_logs()
        try:
            update_simple_log(BOARD_PORTS_CURRENT, MODE_CURRENT)
        except Exception:
            pass
    finally:
        sys.__excepthook__(exc_type, exc, tb)
sys.excepthook = _excepthook

def log(msg: str) -> None:
    print(msg, flush=True)
    _logger.info(msg)
    _flush_logs()
    try:
        from PyQt5.QtCore import QCoreApplication
        if QCoreApplication.instance() is not None:
            QCoreApplication.postEvent(QtLogBridge.instance(), QtLogEvent(msg))
    except Exception:
        pass

# ---------------------------- SIMPLE LOG ----------------------------
def update_simple_log(board_ports: Dict[str, str], option_mode: Optional[str]) -> None:
    if not option_mode:
        return
    snap = TRACKER.snapshot()
    lines: List[str] = []
    lines.append(f"simple_log | option={option_mode} | started={RUN_START} | now={datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    boards = list(board_ports.keys())
    try:
        boards = sorted(boards, key=lambda n: (board_index(n), n))
    except Exception:
        pass

    with _DEAD_LOCK:
        dead_boards = set(DEAD_BOARDS)

    expected_slots = sum(get_slot_limit(b) for b in boards)
    for b in boards:
        prefix = "[DEAD] " if b in dead_boards else ""
        slots = []
        lim = get_slot_limit(b)
        for s in range(1, lim + 1):
            phases = snap.get(b, {}).get(str(s), {})
            done = StatusTracker.slot_done(option_mode, phases if isinstance(phases, dict) else {})
            slots.append(f"{s}={'True' if done else 'False'}")
        lines.append(f"{prefix}{b}({get_board_type(b)}): " + ("  ".join(slots) if slots else "(no slots)"))

    summary = TRACKER.summarize(option_mode)
    dead_list = ", ".join(sorted(dead_boards, key=lambda n: (board_index(n), n))) if dead_boards else "-"
    lines.append(
        "TOTALS: "
        f"boards_detected={summary['boards_total']} "
        f"slots_expected={expected_slots} "
        f"| boards_ok={summary['boards_ok']} boards_fail={summary['boards_fail']} "
        f"| slots_seen={summary['slots_total']} slots_ok={summary['slots_ok']} slots_fail={summary['slots_fail']} "
        f"| boards_dead={len(dead_boards)} [{dead_list}]"
    )
    with open(SIMPLE_LOG, "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")

# ---------------------------- PROGRESS ----------------------------
def _is_slot_processed(mode: Optional[str], phases: Dict[str, Dict[str, str]]) -> bool:
    return StatusTracker.slot_done(mode or "", phases)

def _bump_progress_if_terminal(board: str, slot: int):
    global PROG_SLOTS_DONE
    phases = TRACKER.snapshot().get(board, {}).get(str(slot), {})
    if _is_slot_processed(MODE_CURRENT, phases):
        key = (board, int(slot))
        if key not in PROG_SLOTS_DONE:
            PROG_SLOTS_DONE.add(key)

def mark_and_update(board: str, slot: int, phase: str, status: str, err: str = "") -> None:
    TRACKER.mark(board, slot, phase, status, err)
    try:
        _bump_progress_if_terminal(board, slot)
        update_simple_log(BOARD_PORTS_CURRENT, MODE_CURRENT)
    except Exception:
        pass

def set_progress_total_from_selected(selected_boards: List[Tuple[str, str]], selection: Optional[Dict[str, Set[int]]]) -> None:
    global PROG_TOTAL_SLOTS, PROG_SLOTS_DONE
    if selection:
        total = 0
        for b, _ in selected_boards:
            lim = get_slot_limit(b)
            total += len([s for s in selection.get(b, set()) if 1 <= int(s) <= lim])
        PROG_TOTAL_SLOTS = max(1, total)
    else:
        PROG_TOTAL_SLOTS = max(1, sum(get_slot_limit(b) for b, _ in selected_boards))
    PROG_SLOTS_DONE = set()

def start_new_run_reset():
    TRACKER.clear()
    reset_dead_state()
    CANCEL_EVENT.clear()

# ---------------------------- HELPERS ----------------------------
def board_index(board_name: str) -> int:
    m = re.search(r"(\d+)$", board_name or "")
    return int(m.group(1)) if m else 0

def board_name_from_index(idx: int) -> str:
    return f"Board_{idx}"

def build_label(mode: str, bidx: int, slot: int) -> str:
    mu = (mode or "").upper()
    if mu == "EGP":
        prefix = "EGP"
    elif mu == "CMFL":
        prefix = "CMFL"
    else:
        prefix = "MFL"
    raw = f"{prefix}B{bidx}S{slot}".upper()
    cleaned = re.sub(r"[^A-Z0-9 ]", "", raw)
    label = cleaned[:MAX_LABEL_LEN]
    if label != raw:
        log(f"[LABEL] Compacting '{raw}' -> '{label}' for exFAT")
    return label

def parse_compact_label(lbl: str) -> Optional[Tuple[str, int, int]]:
    if not lbl:
        return None
    m = re.match(r"^(MFL|EGP|CMFL)B(\d+)S(\d+)$", lbl.strip().upper())
    if not m:
        return None
    try:
        return (m.group(1), int(m.group(2)), int(m.group(3)))
    except Exception:
        return None

def get_board_type(board_name: str) -> str:
    t = BOARD_TYPES_CURRENT.get(board_name)
    return t if t in BOARD_SLOT_LIMITS else "C-MFL"

def get_board_label_prefix(board_name: str) -> str:
    t = get_board_type(board_name).upper()
    if t == "EGP":
        return "EGP"
    if t == "C-MFL":
        return "CMFL"
    return "MFL"

def get_slot_limit(board_name: str) -> int:
    # ODO/INLB are slot-less
    if (board_name or "").strip().upper() in {"ODO", "INLB"}:
        return 0
    return BOARD_SLOT_LIMITS.get(get_board_type(board_name), 8)

def find_board_name_by_index(idx: int) -> str:
    target_default = f"Board_{idx}"
    if target_default in BOARD_PORTS_CURRENT:
        return target_default
    for name in BOARD_PORTS_CURRENT.keys():
        if board_index(name) == idx:
            return name
    for name in BOARD_TYPES_CURRENT.keys():
        if board_index(name) == idx:
            return name
    return target_default

def find_named_board(exact_name: str) -> Optional[Tuple[str, str]]:
    n = (exact_name or "").strip()
    if not n:
        return None
    for k, v in BOARD_PORTS_CURRENT.items():
        if k.strip() == n:
            return (k, v)
    return None

def enter_sd_menu(port: str, board: str) -> bool:
    resp = send_command(port, "2")
    if resp is None:
        log(f"[{board}] Failed to enter SD menu (cmd=2).")
        return False
    return True

# ---------------------------- DETECTION ----------------------------
def detect_boards() -> Dict[str, str]:
    serial_to_board = REGISTRY.get("serial_to_board", {})
    board_ports: Dict[str, str] = {}
    board_types: Dict[str, str] = {}
    board_excl: Dict[str, Set[int]] = {}
    board_pipe_sizes: Dict[str, int] = {}

    for port in serial.tools.list_ports.comports():
        serial_number = getattr(port, "serial_number", None)
        if not serial_number:
            continue
        if serial_number in serial_to_board:
            entry = serial_to_board[serial_number] or {}
            name = entry.get("name") or f"Board_{serial_number[-4:]}"
            btype = (entry.get("type") or "C-MFL").upper()
            pipe_raw = entry.get("pipe_size", "")
            try:
                pipe_size = int(pipe_raw)
            except Exception:
                pipe_size = 0
            exclude_raw = entry.get("exclude", [])
            try:
                if isinstance(exclude_raw, str):
                    exclude = {int(x) for x in re.split(r"[,\s]+", exclude_raw) if x.strip().isdigit()}
                elif isinstance(exclude_raw, list):
                    exclude = {int(x) for x in exclude_raw if isinstance(x, int) or (isinstance(x, str) and x.isdigit())}
                else:
                    exclude = set()
            except Exception:
                exclude = set()
            board_ports[name] = port.device
            board_types[name] = btype if btype in BOARD_SLOT_LIMITS else "C-MFL"
            board_excl[name] = exclude
            board_pipe_sizes[name] = pipe_size

    global BOARD_TYPES_CURRENT, BOARD_EXCLUDE_SLOTS, BOARD_PIPE_SIZES
    BOARD_TYPES_CURRENT = board_types
    BOARD_EXCLUDE_SLOTS = board_excl
    BOARD_PIPE_SIZES = board_pipe_sizes
    log(f"[INFO] Detected boards (registry-based): {board_ports}")
    return board_ports

def get_port_for_board(board_name: str) -> Optional[str]:
    port = BOARD_PORTS_CURRENT.get(board_name)
    if not port:
        refreshed = detect_boards()
        port = refreshed.get(board_name)
    return port

# ---------------------------- SERIAL ----------------------------
def send_command(port_name: str, cmd: str) -> Optional[str]:
    if CANCEL_EVENT.is_set():
        return None
    try:
        with serial.Serial(port_name, 115200, timeout=1) as ser:
            ser.write((cmd + "\r\n").encode())
            time.sleep(0.3)
            resp = ser.read_all().decode(errors="ignore").strip()
            log(f"[SERIAL {port_name}] > {cmd} | resp={resp[:80]}")
            return resp
    except (FileNotFoundError, SerialException) as e:
        log(f"[WARN] Serial open failed on {port_name}: {e}. Waiting up to 5s for port to return...")
        if wait_for_port_back(port_name, timeout_sec=5.0):
            log(f"[INFO] Port {port_name} reappeared; retrying command.")
            try:
                with serial.Serial(port_name, 115200, timeout=1) as ser:
                    ser.write((cmd + "\r\n").encode())
                    time.sleep(0.3)
                    resp = ser.read_all().decode(errors="ignore").strip()
                    log(f"[SERIAL {port_name}] > {cmd} | resp={resp[:80]}")
                    return resp
            except Exception as e2:
                mark_port_dead(port_name, f"retry after reappearance failed: {e2}")
                log(f"[ERROR] Serial retry failed on {port_name}: {e2}")
                return None
        else:
            mark_port_dead(port_name, str(e))
            log(f"[ERROR] Serial communication failed on {port_name}: {e} (dead after 5s)")
            return None
    except Exception as e:
        log(f"[ERROR] Serial communication failed on {port_name}: {e}")
        return None

def disconnect_mux(port_name: str, board_name: str) -> None:
    try:
        log(f"[{board_name}] Sending MUX disconnect (25)")
        send_command(port_name, "25")
        time.sleep(0.7)
    except Exception as e:
        log(f"[{board_name}] [WARN] MUX disconnect failed: {e}")

# ---------------------------- DRIVE + FS ----------------------------
def get_drive_label(drive_letter: str) -> str:
    buf = ctypes.create_unicode_buffer(1024)
    try:
        ctypes.windll.kernel32.GetVolumeInformationW(
            ctypes.c_wchar_p(drive_letter + "\\"), buf, ctypes.sizeof(buf), None, None, None, None, 0)
        return buf.value
    except Exception:
        return ""

def set_volume_label_win(drive_letter: str, new_label: str) -> bool:
    try:
        path = ctypes.c_wchar_p(drive_letter + "\\")
        label = ctypes.c_wchar_p(new_label)
        rc = ctypes.windll.kernel32.SetVolumeLabelW(path, label)
        return rc != 0
    except Exception:
        return False

def enforce_label(drive_letter: str, desired_label: str) -> bool:
    before = (get_drive_label(drive_letter.strip("\\")) or "").strip()
    if before.lower() == desired_label.lower():
        log(f"[LABEL] Already set: {drive_letter}='{before}'")
        return True
    ok = set_volume_label_win(drive_letter.strip("\\"), desired_label)
    after = (get_drive_label(drive_letter.strip("\\")) or "").strip()
    if after.lower() == desired_label.lower():
        log(f"[LABEL] Set OK: {drive_letter}='{after}' (was '{before or 'N/A'}')")
        return True
    log(f"[LABEL] Set FAILED: wanted '{desired_label}', got '{after or 'N/A'}'")
    return False

def get_fs_type_by_letter(drive_letter: str) -> str:
    dl = drive_letter.strip("\\/:").upper() + ":\\"
    for p in psutil.disk_partitions(all=False):
        if p.device.upper() == dl:
            return (p.fstype or "").strip()
    return ""

def _iter_removable_partitions():
    """
    Yield partitions, preferring those marked as removable.
    Falls back to all partitions if no removable flag is present.
    """
    parts = list(psutil.disk_partitions(all=False))
    removable = []
    for p in parts:
        try:
            opts = (getattr(p, "opts", "") or "").lower()
            if "removable" in opts:
                removable.append(p)
        except Exception:
            continue
    return removable or parts

def wait_for_new_drive(
    timeout: int = 40,
    expect_bootloader: bool = True,
    initial_state: Optional[Dict[str, str]] = None,
    debug_tag: str = "",
) -> Optional[str]:
    if CANCEL_EVENT.is_set():
        return None
    start = time.time()
    def label_of(dev: str) -> str:
        try:
            return get_drive_label(dev.strip("\\"))  # fast probe
        except Exception:
            return ""
    if initial_state is None:
        initial_state = {p.device: label_of(p.device) for p in _iter_removable_partitions()}
    if debug_tag:
        try:
            log(f"[USB DEBUG {debug_tag}] old drives = {sorted(initial_state.keys())}")
        except Exception:
            pass
    while time.time() - start < timeout:
        if CANCEL_EVENT.is_set():
            return None
        current = {p.device: label_of(p.device) for p in _iter_removable_partitions()}
        new_drives = set(current.keys()) - set(initial_state.keys())

        if debug_tag:
            try:
                log(f"[USB DEBUG {debug_tag}] current drives = {sorted(current.keys())}")
                if new_drives:
                    log(f"[USB DEBUG {debug_tag}] new drives = {sorted(new_drives)}")
            except Exception:
                pass

        for d in new_drives:
            dp = f"{d}\\"
            lbl = (current[d] or "").upper()
            if expect_bootloader:
                if "RPI-RP2" in lbl or os.path.exists(os.path.join(dp, "INFO_UF2.TXT")):
                    log(f"[USB] New bootloader drive: {dp} ({lbl})")
                    return dp
            else:
                if "RPI-RP2" not in lbl and "RP-DRIVE" not in lbl:
                    log(f"[USB] New data drive: {dp} ({lbl})")
                    return dp

        for d, lbl in current.items():
            prev = (initial_state.get(d, "") or "")
            up = (lbl or "").upper()
            prev_up = (prev or "").upper()
            if expect_bootloader:
                if ("RPI-RP2" in up and "RPI-RP2" not in prev_up) or os.path.exists(os.path.join(f"{d}\\", "INFO_UF2.TXT")):
                    log(f"[USB] Bootloader appeared on existing device: {d}\\ ({up})")
                    return f"{d}\\"
            else:
                # Special handling for LABEL_PREF: treat a label change to something
                # like "Removable Disk" as a new SD, even if the drive letter was
                # already present in the system.
                if debug_tag.startswith("LABEL_PREF") and "REMOVABLE" in up and "REMOVABLE" not in prev_up:
                    log(f"[USB] Removable data drive appeared on existing device: {d}\\ ({up}) [tag {debug_tag}]")
                    return f"{d}\\"
                if "RPI-RP2" not in up and "RP-DRIVE" not in up and "RPI-RP2" in prev_up:
                    log(f"[USB] Data drive appeared after RP2: {d}\\ ({up})")
                    return f"{d}\\"

        time.sleep(0.5)
    log("[USB] Timeout waiting for drive")
    return None

def wait_for_drive_by_label(expected_labels: List[str], timeout: int = 40) -> Optional[str]:
    if CANCEL_EVENT.is_set():
        return None
    labels = [l.strip() for l in (expected_labels or []) if l and l.strip()]
    labels_lc = {l.lower() for l in labels}
    if not labels_lc:
        return None
    end = time.time() + timeout
    while time.time() < end:
        if CANCEL_EVENT.is_set():
            return None
        for p in _iter_removable_partitions():
            root = p.device
            try:
                lbl = (get_drive_label(root.strip("\\")) or "").strip().lower()
            except Exception:
                lbl = ""
            if lbl in labels_lc:
                log(f"[USB] Matched label '{lbl}' at {root}")
                return root if root.endswith("\\") else f"{root}\\"
        time.sleep(0.3)
    log(f"[USB] Timeout waiting for labels: {sorted(labels_lc)}")
    return None

# RPI-RP2 finder & uploader (Advanced)
def list_rp2_drives() -> List[str]:
    roots: List[str] = []
    for p in psutil.disk_partitions(all=False):
        root = p.device if p.device.endswith("\\") else f"{p.device}\\"
        try:
            lbl = (get_drive_label(root.strip("\\")) or "").upper()
        except Exception:
            lbl = ""
        if "RPI-RP2" in lbl or os.path.exists(os.path.join(root, "INFO_UF2.TXT")):
            roots.append(root)
    return roots

def upload_uf2_to_all_rp2(uf2_path: str) -> Tuple[int, int, List[str]]:
    ok = 0; fail = 0; msgs: List[str] = []
    if not os.path.isfile(uf2_path):
        return 0, 0, [f"Firmware not found: {uf2_path}"]
    drives = list_rp2_drives()
    if not drives:
        return 0, 0, ["No RPI-RP2 drives detected."]
    for root in drives:
        try:
            shutil.copy2(uf2_path, os.path.join(root, os.path.basename(uf2_path)))
            msgs.append(f"Copied to {root}")
            ok += 1
        except Exception as e:
            msgs.append(f"Failed on {root}: {e}")
            fail += 1
    return ok, fail, msgs

# ---------------------------- FILE COPY ----------------------------
def copy_file_with_progress(src: str, dest: str, retries: int = 3, delay: float = 1.0) -> bool:
    os.makedirs(os.path.dirname(dest), exist_ok=True)
    for attempt in range(retries):
        if CANCEL_EVENT.is_set():
            return False
        try:
            total_size = os.path.getsize(src)
            with open(src, "rb") as fsrc, open(dest, "wb") as fdst, tqdm(
                total=total_size, unit="B", unit_scale=True,
                desc=f"Copying {os.path.basename(src)}", ascii=True) as pbar:
                while True:
                    if CANCEL_EVENT.is_set():
                        return False
                    buf = fsrc.read(1024 * 1024)
                    if not buf:
                        break
                    fdst.write(buf)
                    pbar.update(len(buf))
            log(f"[COPIED] {src} -> {dest}")
            return True
        except Exception as e:
            log(f"[ERROR] Copy failed ({attempt+1}/{retries}): {e}")
            time.sleep(delay)
    log(f"[ERROR] Failed to copy {src}")
    return False

def log_sdcard_file_info(src_drive: str, dest_folder: str, board: str, slot: int, vol_label: str) -> bool:
    try:
        if CANCEL_EVENT.is_set():
            return False
        if not os.path.exists(src_drive):
            log(f"[LOG {board}:{slot}] Drive {src_drive} not found for logging.")
            return False
        file_map = top_level_file_map(src_drive)
        if not file_map:
            log(f"[LOG {board}:{slot}] No files found on {src_drive} to log.")
            return False
        os.makedirs(dest_folder, exist_ok=True)
        log_path = os.path.join(dest_folder, "data_log.txt")
        ts = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        with open(log_path, "a", encoding="utf-8") as f:
            f.write(
                f"=== DATA LOG | board={board} slot={slot} | "
                f"label={vol_label or 'UNKNOWN'} | drive={src_drive} | time={ts} ===\n"
            )
            for name in sorted(file_map.keys()):
                size = file_map.get(name, -1)
                if size is None or size < 0:
                    f.write(f"{name} | size=unknown\n")
                else:
                    f.write(f"{name} | {size} bytes\n")
            f.write("\n")
        log(f"[LOGGED] {board} slot {slot} | {len(file_map)} files from {src_drive} -> {log_path}")
        return True
    except Exception as e:
        log(f"[ERROR] Failed to write data log for {board}:{slot}: {e}")
        return False

def top_level_file_map(root_path: str) -> Dict[str, int]:
    try:
        entries = os.listdir(root_path)
    except Exception:
        return {}
    out: Dict[str, int] = {}
    for name in entries:
        p = os.path.join(root_path, name)
        if os.path.isfile(p):
            try:
                out[name] = os.path.getsize(p)
            except Exception:
                out[name] = -1
    return out

def select_files_for_copy(files: List[str]) -> List[str]:
    """Apply custom slice rules to the provided file list."""
    cfg = get_file_slice_config()
    ordered = sorted(files)
    if not cfg.get("enabled"):
        return ordered

    offset = max(0, int(cfg.get("offset", 0) or 0))
    count = max(0, int(cfg.get("count", 0) or 0))
    tail = max(0, int(cfg.get("tail", 0) or 0))

    window = ordered[offset: offset + count] if count > 0 else []
    from_end = ordered[-tail:] if tail > 0 else []

    seen: Set[str] = set()
    selected: List[str] = []
    for name in window + from_end:
        if name in seen:
            continue
        selected.append(name)
        seen.add(name)

    log(f"[COPY] Custom selection enabled: skip={offset}, take={count}, tail={tail}, picked={len(selected)}/{len(ordered)}")
    return selected

def verify_top_level(src_drive: str, dest_folder: str, only_files: Optional[Set[str]] = None):
    src_map = top_level_file_map(src_drive)
    dest_map = top_level_file_map(dest_folder)
    if only_files is not None:
        src_map = {k: v for k, v in src_map.items() if k in only_files}
        dest_map = {k: v for k, v in dest_map.items() if k in only_files}
    missing = [f for f in src_map.keys() if f not in dest_map]
    mismatched = [f for f in src_map.keys() if f in dest_map and dest_map[f] != src_map[f]]
    return src_map, dest_map, missing, mismatched

def copy_sdcard_files(src_drive: str, dest_folder: str, board: str, slot: int, max_passes: int = 3) -> bool:
    try:
        if CANCEL_EVENT.is_set():
            mark_and_update(board, slot, "copy", "failed", "cancelled")
            return False
        mark_and_update(board, slot, "copy", "running", "")
        label = (get_drive_label(src_drive.strip("\\")) or "").upper()
        if "RPI-RP2" in label or "RP-DRIVE" in label:
            mark_and_update(board, slot, "copy", "failed", "bootloader drive")
            return False

        if not os.path.exists(src_drive):
            mark_and_update(board, slot, "copy", "failed", "drive not mounted")
            return False

        raw_files = [f for f in os.listdir(src_drive) if os.path.isfile(os.path.join(src_drive, f))]
        files = select_files_for_copy(raw_files)
        if not files:
            reason = "no files after selection" if get_file_slice_config().get("enabled") else "no files"
            mark_and_update(board, slot, "copy", "failed", reason)
            return False

        os.makedirs(dest_folder, exist_ok=True)
        time.sleep(1.0)

        for f in files:
            if CANCEL_EVENT.is_set():
                mark_and_update(board, slot, "copy", "failed", "cancelled")
                return False
            src_file = os.path.join(src_drive, f)
            dest_file = os.path.join(dest_folder, f)
            copy_file_with_progress(src_file, dest_file)

        for _attempt in range(1, max_passes + 1):
            if CANCEL_EVENT.is_set():
                mark_and_update(board, slot, "copy", "failed", "cancelled")
                return False
            _, _, missing, mismatched = verify_top_level(src_drive, dest_folder, set(files))
            if not missing and not mismatched:
                mark_and_update(board, slot, "copy", "success", "")
                return True
            for f in missing + mismatched:
                src_file = os.path.join(src_drive, f)
                dest_file = os.path.join(dest_folder, f)
                copy_file_with_progress(src_file, dest_file)
            time.sleep(0.5)

        _, _, missing, mismatched = verify_top_level(src_drive, dest_folder, set(files))
        if not missing and not mismatched:
            mark_and_update(board, slot, "copy", "success", "")
            return True

        reason = f"verify failed; missing={missing}, mismatched={mismatched}"
        mark_and_update(board, slot, "copy", "failed", reason)
        return False

    except Exception as e:
        mark_and_update(board, slot, "copy", "failed", f"exception: {e}")
        return False

# ---------------------------- FORMAT ----------------------------
def _mountvol_offline(letter: str) -> None:
    try:
        subprocess.run(["cmd", "/c", f"mountvol {letter}: /p"], capture_output=True, text=True, timeout=10)
    except Exception:
        pass

def format_usb_drive_exfat_cmd_safe(port: str, board: str, slot: int, drive_letter: str, label: str = "MFLB1S1", timeout_sec: int = 300) -> bool:
    if CANCEL_EVENT.is_set():
        return False
    dl = drive_letter.strip("\\/:").upper()
    cmd = f"format {dl}: /FS:exFAT /V:{label} /Y /Q"
    try:
        subprocess.run(["cmd", "/c", cmd], capture_output=True, text=True, input="\n", timeout=timeout_sec)
    except subprocess.TimeoutExpired:
        _format_recovery_no_retry(port, board, slot, dl)
        return False
    except Exception:
        _format_recovery_no_retry(port, board, slot, dl)
        return False

    ok = False
    for _ in range(24):
        if CANCEL_EVENT.is_set():
            break
        fs = (get_fs_type_by_letter(dl) or "").lower()
        if fs == "exfat":
            ok = True
            break
        time.sleep(0.5)

    if ok:
        return True

    _format_recovery_no_retry(port, board, slot, dl)
    return False

def _format_recovery_no_retry(port: str, board: str, slot: int, dl: str) -> None:
    _mountvol_offline(dl)
    try:
        disconnect_mux(port, board)
    except Exception:
        pass

# ---------------------------- LOCK ----------------------------
BOOT_LOCK = threading.RLock()

# ---------------------------- UF2 BOOT+UPLOAD ----------------------------
def _boot_and_upload_uf2(port: str, slot: int, uf2_path: str, board_name: str, phase_key: str) -> bool:
    if CANCEL_EVENT.is_set():
        mark_and_update(board_name, slot, phase_key, "failed", "cancelled")
        return False
    if is_board_dead(board_name) or (_norm_port(port) in DEAD_PORTS):
        mark_and_update(board_name, slot, phase_key, "failed", "port/board dead")
        return False
    for _ in range(3):
        if CANCEL_EVENT.is_set():
            mark_and_update(board_name, slot, phase_key, "failed", "cancelled")
            return False
        if is_board_dead(board_name) or (_norm_port(port) in DEAD_PORTS):
            mark_and_update(board_name, slot, phase_key, "failed", "port/board dead")
            return False
        resp = send_command(port, str(slot))
        if resp is None:
            mark_and_update(board_name, slot, phase_key, "failed", "serial open failed")
            return False
        time.sleep(2)
        rp_drive = wait_for_new_drive(timeout=5, expect_bootloader=True)
        if rp_drive:
            try:
                shutil.copy2(uf2_path, os.path.join(rp_drive, os.path.basename(uf2_path)))
                time.sleep(5)
                mark_and_update(board_name, slot, phase_key, "success", "")
                return True
            except Exception as e:
                mark_and_update(board_name, slot, phase_key, "failed", f"UF2 upload failed: {e}")
                return False
    mark_and_update(board_name, slot, phase_key, "failed", "bootloader not detected")
    return False

# ---------------------------- PER-SLOT OPS ----------------------------
def op_data_slot(board: str, port: str, expected_slot: int, *, already_in_menu: bool = False) -> bool:
    if CANCEL_EVENT.is_set():
        mark_and_update(board, expected_slot, "copy", "failed", "cancelled")
        return False
    if is_board_dead(board) or (_norm_port(port) in DEAD_PORTS):
        mark_and_update(board, expected_slot, "copy", "failed", "port/board dead")
        return False
    if not already_in_menu:
        if send_command(port, "1") is None:
            mark_and_update(board, expected_slot, "copy", "failed", "serial open failed")
            return False
        time.sleep(0.5)
    expected_bidx = board_index(board)
    with BOOT_LOCK:
        ok = _boot_and_upload_uf2(port, expected_slot, DATA_FIRMWARE, board, "boot_data")
        if not ok:
            mark_and_update(board, expected_slot, "copy", "failed", "boot data uf2 failed")
            return False

        prefix = get_board_label_prefix(board)
        expected_label = build_label(prefix, expected_bidx, expected_slot)
        sd_drive = wait_for_drive_by_label([expected_label], timeout=30)
        if not sd_drive:
            sd_drive = wait_for_new_drive(timeout=30, expect_bootloader=False)
        if not sd_drive:
            mark_and_update(board, expected_slot, "copy", "failed", "sd not detected")
            return False

        vol_label = (get_drive_label(sd_drive.strip("\\")) or "").strip().upper()
        parsed = parse_compact_label(vol_label)

        if vol_label:
            by_label_dir = os.path.join("Extracted_Data", "By_Label", vol_label)
        else:
            by_label_dir = os.path.join("Extracted_Data", "By_Label", f"UNLABELED_B{expected_bidx}S{expected_slot}")

        if parsed:
            _, lbl_bidx, lbl_slot = parsed
            t_board = find_board_name_by_index(lbl_bidx); t_slot = lbl_slot
        else:
            t_board = board; t_slot = expected_slot

    okcopy = copy_sdcard_files(sd_drive, by_label_dir, t_board, t_slot)

    if okcopy:
        mark_and_update(board, expected_slot, "copy", "success",
                        f"copied via label '{vol_label or 'UNKNOWN'}' -> {t_board}:{t_slot}")
    else:
        mark_and_update(board, expected_slot, "copy", "failed",
                        f"copy failed; label '{vol_label or 'UNKNOWN'}' mapped to {t_board}:{t_slot}")

    disconnect_mux(port, board)
    return bool(okcopy)

def op_data_log_slot(board: str, port: str, expected_slot: int, *, already_in_menu: bool = False) -> bool:
    if CANCEL_EVENT.is_set():
        mark_and_update(board, expected_slot, "copy", "failed", "cancelled")
        return False
    if is_board_dead(board) or (_norm_port(port) in DEAD_PORTS):
        mark_and_update(board, expected_slot, "copy", "failed", "port/board dead")
        return False
    if not already_in_menu:
        if send_command(port, "1") is None:
            mark_and_update(board, expected_slot, "copy", "failed", "serial open failed")
            return False
        time.sleep(0.5)
    expected_bidx = board_index(board)
    with BOOT_LOCK:
        ok = _boot_and_upload_uf2(port, expected_slot, DATA_FIRMWARE, board, "boot_data")
        if not ok:
            mark_and_update(board, expected_slot, "copy", "failed", "boot data uf2 failed")
            return False

        prefix = get_board_label_prefix(board)
        expected_label = build_label(prefix, expected_bidx, expected_slot)
        sd_drive = wait_for_drive_by_label([expected_label], timeout=30)
        if not sd_drive:
            sd_drive = wait_for_new_drive(timeout=30, expect_bootloader=False)
        if not sd_drive:
            mark_and_update(board, expected_slot, "copy", "failed", "sd not detected")
            return False

        vol_label = (get_drive_label(sd_drive.strip("\\")) or "").strip().upper()
        parsed = parse_compact_label(vol_label)

        if vol_label:
            by_label_dir = os.path.join("Extracted_Data", "By_Label", vol_label)
        else:
            by_label_dir = os.path.join("Extracted_Data", "By_Label", f"UNLABELED_B{expected_bidx}S{expected_slot}")

        if parsed:
            _, lbl_bidx, lbl_slot = parsed
            t_board = find_board_name_by_index(lbl_bidx); t_slot = lbl_slot
        else:
            t_board = board; t_slot = expected_slot

    oklog = log_sdcard_file_info(sd_drive, by_label_dir, t_board, t_slot, vol_label)

    if oklog:
        mark_and_update(board, expected_slot, "copy", "success",
                        f"logged via label '{vol_label or 'UNKNOWN'}' -> {t_board}:{t_slot}")
    else:
        mark_and_update(board, expected_slot, "copy", "failed",
                        f"log failed; label '{vol_label or 'UNKNOWN'}' mapped to {t_board}:{t_slot}")

    disconnect_mux(port, board)
    return bool(oklog)

def op_data_pref_slot(board: str, port: str, slot: int) -> bool:
    if CANCEL_EVENT.is_set():
        mark_and_update(board, slot, "copy", "failed", "cancelled")
        return False
    if is_board_dead(board) or (_norm_port(port) in DEAD_PORTS):
        mark_and_update(board, slot, "copy", "failed", "port/board dead")
        return False

    if send_command(port, str(slot)) is None:
        mark_and_update(board, slot, "copy", "failed", "serial open failed (slot connect)")
        return False

    bidx = board_index(board)
    prefix = get_board_label_prefix(board)
    expected_label = build_label(prefix, bidx, slot)
    sd_drive = wait_for_drive_by_label([expected_label], timeout=15)
    if not sd_drive:
        sd_drive = wait_for_new_drive(timeout=20, expect_bootloader=False)
    if not sd_drive:
        mark_and_update(board, slot, "copy", "failed", "sd not detected")
        send_command(port, "9"); time.sleep(2.0)
        return False

    vol_label = (get_drive_label(sd_drive.strip("\\")) or "").strip().upper()
    parsed = parse_compact_label(vol_label)

    if vol_label:
        by_label_dir = os.path.join("Extracted_Data", "By_Label", vol_label)
    else:
        by_label_dir = os.path.join("Extracted_Data", "By_Label", f"UNLABELED_B{bidx}S{slot}")

    if parsed:
        _, lbl_bidx, lbl_slot = parsed
        t_board = find_board_name_by_index(lbl_bidx); t_slot = lbl_slot
    else:
        t_board = board; t_slot = slot

    okcopy = copy_sdcard_files(sd_drive, by_label_dir, t_board, t_slot)

    if okcopy:
        mark_and_update(board, slot, "copy", "success",
                        f"copied via label '{vol_label or 'UNKNOWN'}' -> {t_board}:{t_slot}")
    else:
        mark_and_update(board, slot, "copy", "failed",
                        f"copy failed; label '{vol_label or 'UNKNOWN'}' mapped to {t_board}:{t_slot}")

    send_command(port, "9")
    time.sleep(2.0)
    return bool(okcopy)

def op_data_log_pref_slot(board: str, port: str, slot: int) -> bool:
    if CANCEL_EVENT.is_set():
        mark_and_update(board, slot, "copy", "failed", "cancelled")
        return False
    if is_board_dead(board) or (_norm_port(port) in DEAD_PORTS):
        mark_and_update(board, slot, "copy", "failed", "port/board dead")
        return False

    if send_command(port, str(slot)) is None:
        mark_and_update(board, slot, "copy", "failed", "serial open failed (slot connect)")
        return False

    bidx = board_index(board)
    prefix = get_board_label_prefix(board)
    expected_label = build_label(prefix, bidx, slot)
    sd_drive = wait_for_drive_by_label([expected_label], timeout=15)
    if not sd_drive:
        sd_drive = wait_for_new_drive(timeout=20, expect_bootloader=False)
    if not sd_drive:
        mark_and_update(board, slot, "copy", "failed", "sd not detected")
        send_command(port, "9"); time.sleep(2.0)
        return False

    vol_label = (get_drive_label(sd_drive.strip("\\")) or "").strip().upper()
    parsed = parse_compact_label(vol_label)

    if vol_label:
        by_label_dir = os.path.join("Extracted_Data", "By_Label", vol_label)
    else:
        by_label_dir = os.path.join("Extracted_Data", "By_Label", f"UNLABELED_B{bidx}S{slot}")

    if parsed:
        _, lbl_bidx, lbl_slot = parsed
        t_board = find_board_name_by_index(lbl_bidx); t_slot = lbl_slot
    else:
        t_board = board; t_slot = slot

    oklog = log_sdcard_file_info(sd_drive, by_label_dir, t_board, t_slot, vol_label)

    if oklog:
        mark_and_update(board, slot, "copy", "success",
                        f"logged via label '{vol_label or 'UNKNOWN'}' -> {t_board}:{t_slot}")
    else:
        mark_and_update(board, slot, "copy", "failed",
                        f"log failed; label '{vol_label or 'UNKNOWN'}' mapped to {t_board}:{t_slot}")

    send_command(port, "9")
    time.sleep(2.0)
    return bool(oklog)

def op_format_slot(board: str, port: str, slot: int, mode: str, *, already_in_menu: bool = False) -> bool:
    if CANCEL_EVENT.is_set():
        mark_and_update(board, slot, "format", "failed", "cancelled")
        return False
    if is_board_dead(board) or (_norm_port(port) in DEAD_PORTS):
        mark_and_update(board, slot, "format", "failed", "port/board dead")
        return False
    if not already_in_menu:
        if send_command(port, "1") is None:
            mark_and_update(board, slot, "format", "failed", "serial open failed")
            return False
        time.sleep(0.5)
    bidx = board_index(board)
    ok = _boot_and_upload_uf2(port, slot, DATA_FIRMWARE, board, "boot_data")
    if not ok:
        mark_and_update(board, slot, "format", "failed", "boot data uf2 failed")
        disconnect_mux(port, board); return False

    sd_drive = wait_for_new_drive(timeout=25, expect_bootloader=False)
    if not sd_drive:
        mark_and_update(board, slot, "format", "failed", "sd not detected")
        disconnect_mux(port, board); return False

    current_label = (get_drive_label(sd_drive.strip("\\")) or "").strip()
    vol_label = current_label if current_label else build_label(mode, bidx, slot)

    ok_fmt = format_usb_drive_exfat_cmd_safe(
        port=port, board=board, slot=slot, drive_letter=sd_drive.strip("\\"), label=vol_label
    )
    mark_and_update(board, slot, "format", "success" if ok_fmt else "failed", "" if ok_fmt else "format error")
    if not ok_fmt:
        disconnect_mux(port, board); return False

    time.sleep(1.0)
    if enforce_label(sd_drive.strip("\\"), vol_label):
        mark_and_update(board, slot, "label_verify", "success", "")
    else:
        mark_and_update(board, slot, "label_verify", "failed", "label verify failed after format")

    disconnect_mux(port, board)
    return True

def op_format_pref_slot(board: str, port: str, slot: int) -> bool:
    if CANCEL_EVENT.is_set():
        mark_and_update(board, slot, "format", "failed", "cancelled")
        return False
    if is_board_dead(board) or (_norm_port(port) in DEAD_PORTS):
        mark_and_update(board, slot, "format", "failed", "port/board dead")
        return False

    if send_command(port, str(slot)) is None:
        mark_and_update(board, slot, "format", "failed", "serial open failed (slot connect)")
        return False

    bidx = board_index(board)
    prefix = get_board_label_prefix(board)
    expected_label = build_label(prefix, bidx, slot)
    sd_drive = wait_for_drive_by_label([expected_label], timeout=15)
    if not sd_drive:
        sd_drive = wait_for_new_drive(timeout=20, expect_bootloader=False)
    if not sd_drive:
        mark_and_update(board, slot, "format", "failed", "sd not detected")
        send_command(port, "9"); time.sleep(2.0)
        return False

    current_label = (get_drive_label(sd_drive.strip("\\")) or "").strip()
    vol_label = current_label if current_label else expected_label

    ok_fmt = format_usb_drive_exfat_cmd_safe(
        port=port, board=board, slot=slot, drive_letter=sd_drive.strip("\\"), label=vol_label
    )
    mark_and_update(board, slot, "format", "success" if ok_fmt else "failed", "" if ok_fmt else "format error")
    if not ok_fmt:
        send_command(port, "9"); time.sleep(2.0)
        return False

    time.sleep(3.0)
    if enforce_label(sd_drive.strip("\\"), vol_label):
        mark_and_update(board, slot, "label_verify", "success", "")
    else:
        mark_and_update(board, slot, "label_verify", "failed", "label verify failed after format")

    send_command(port, "9")
    time.sleep(2.0)
    return True

def op_label_pref_slot(board: str, port: str, slot: int) -> bool:
    if CANCEL_EVENT.is_set():
        mark_and_update(board, slot, "label", "failed", "cancelled")
        return False
    if is_board_dead(board) or (_norm_port(port) in DEAD_PORTS):
        mark_and_update(board, slot, "label", "failed", "port/board dead")
        return False

    # Snapshot removable drives before connecting this slot so we can
    # detect the new SD drive even if it appears very quickly.
    try:
        before_devs = {p.device for p in _iter_removable_partitions()}
    except Exception:
        before_devs = set()
    # Debug helper: only for LABEL_PREF
    try:
        log(f"[{board}:{slot}] LABEL_PREF old drives = {sorted(before_devs) if before_devs else 'NONE'}")
    except Exception:
        pass

    # We are in SD-card management menu; connect this slot
    if send_command(port, str(slot)) is None:
        mark_and_update(board, slot, "label", "failed", "serial open failed (slot connect)")
        return False

    bidx = board_index(board)
    prefix = get_board_label_prefix(board)
    target_label = build_label(prefix, bidx, slot)

    initial_state = {d: "" for d in before_devs} if before_devs else None
    sd_drive = wait_for_new_drive(
        timeout=20,
        expect_bootloader=False,
        initial_state=initial_state,
        debug_tag=f"LABEL_PREF {board}:{slot}",
    )
    if not sd_drive:
        mark_and_update(board, slot, "label", "failed", "sd not detected")
        try:
            send_command(port, "9")
            time.sleep(2.0)
        except Exception:
            pass
        return False

    ok_lbl = enforce_label(sd_drive.strip("\\"), target_label)
    if ok_lbl:
        mark_and_update(board, slot, "label", "success", "")
    else:
        mark_and_update(board, slot, "label", "failed", "label verify failed")

    try:
        send_command(port, "9")
        time.sleep(2.0)
    except Exception:
        pass

    return bool(ok_lbl)

def op_mfl_burn_slot(board: str, port: str, slot: int, *, already_in_menu: bool = False) -> bool:
    if CANCEL_EVENT.is_set():
        mark_and_update(board, slot, "mfl_upload", "failed", "cancelled")
        return False
    if is_board_dead(board) or (_norm_port(port) in DEAD_PORTS):
        mark_and_update(board, slot, "mfl_upload", "failed", "port/board dead")
        return False
    if not already_in_menu:
        if send_command(port, "1") is None:
            mark_and_update(board, slot, "mfl_upload", "failed", "serial open failed")
            return False
        time.sleep(0.5)
    ok = _boot_and_upload_uf2(port, slot, MFL_FIRMWARE, board, "mfl_upload")
    disconnect_mux(port, board)
    return bool(ok)

def op_cmfl_burn_slot(board: str, port: str, slot: int, *, already_in_menu: bool = False) -> bool:
    if CANCEL_EVENT.is_set():
        mark_and_update(board, slot, "mfl_upload", "failed", "cancelled")
        return False
    if is_board_dead(board) or (_norm_port(port) in DEAD_PORTS):
        mark_and_update(board, slot, "mfl_upload", "failed", "port/board dead")
        return False
    if not already_in_menu:
        if send_command(port, "1") is None:
            mark_and_update(board, slot, "mfl_upload", "failed", "serial open failed")
            return False
        time.sleep(0.5)
    ok = _boot_and_upload_uf2(port, slot, CMFL_FIRMWARE, board, "mfl_upload")
    disconnect_mux(port, board)
    return bool(ok)

def op_egp_burn_slot(board: str, port: str, slot: int, *, already_in_menu: bool = False) -> bool:
    if CANCEL_EVENT.is_set():
        mark_and_update(board, slot, "mfl_upload", "failed", "cancelled")
        return False
    if is_board_dead(board) or (_norm_port(port) in DEAD_PORTS):
        mark_and_update(board, slot, "mfl_upload", "failed", "port/board dead")
        return False
    if not already_in_menu:
        if send_command(port, "1") is None:
            mark_and_update(board, slot, "mfl_upload", "failed", "serial open failed")
            return False
        time.sleep(0.5)
    ok = _boot_and_upload_uf2(port, slot, EGP_FIRMWARE, board, "mfl_upload")
    disconnect_mux(port, board)
    return bool(ok)

def op_label_slot_classic(board: str, port: str, slot: int, *, already_in_menu: bool = False) -> bool:
    # Original LABEL behavior: enter slot menu (1), boot DATA.uf2, and
    # then label the SD card once it enumerates as a drive.
    if CANCEL_EVENT.is_set():
        mark_and_update(board, slot, "label", "failed", "cancelled")
        return False
    if is_board_dead(board) or (_norm_port(port) in DEAD_PORTS):
        mark_and_update(board, slot, "label", "failed", "port/board dead")
        return False
    if not already_in_menu:
        if send_command(port, "1") is None:
            mark_and_update(board, slot, "label", "failed", "serial open failed")
            return False
        time.sleep(0.5)
    bidx = board_index(board)
    with BOOT_LOCK:
        if not _boot_and_upload_uf2(port, slot, DATA_FIRMWARE, board, "boot_data"):
            mark_and_update(board, slot, "label", "failed", "boot data uf2 failed")
            disconnect_mux(port, board); return False
        sd_drive = wait_for_new_drive(timeout=40, expect_bootloader=False)
        if not sd_drive:
            mark_and_update(board, slot, "label", "failed", "sd not detected")
            disconnect_mux(port, board); return False
        prefix = get_board_label_prefix(board)
        target_label = build_label(prefix, bidx, slot)
    ok_lbl = enforce_label(sd_drive.strip("\\"), target_label)
    if ok_lbl:
        mark_and_update(board, slot, "label", "success", "")
    else:
        mark_and_update(board, slot, "label", "failed", "label verify failed")
    disconnect_mux(port, board)
    return bool(ok_lbl)

def op_label_slot(board: str, port: str, slot: int, *, already_in_menu: bool = False) -> bool:
    # LABEL now uses the SD-card menu (2 → slot) flow that was
    # previously used only for LABEL_PREF.
    if CANCEL_EVENT.is_set():
        mark_and_update(board, slot, "label", "failed", "cancelled")
        return False
    if is_board_dead(board) or (_norm_port(port) in DEAD_PORTS):
        mark_and_update(board, slot, "label", "failed", "port/board dead")
        return False

    # We assume the board is already in the SD-card management menu.
    if send_command(port, str(slot)) is None:
        mark_and_update(board, slot, "label", "failed", "serial open failed (slot connect)")
        return False

    bidx = board_index(board)
    prefix = get_board_label_prefix(board)
    target_label = build_label(prefix, bidx, slot)

    sd_drive = wait_for_new_drive(timeout=20, expect_bootloader=False)
    if not sd_drive:
        mark_and_update(board, slot, "label", "failed", "sd not detected")
        try:
            send_command(port, "9")
            time.sleep(2.0)
        except Exception:
            pass
        return False

    ok_lbl = enforce_label(sd_drive.strip("\\"), target_label)
    if ok_lbl:
        mark_and_update(board, slot, "label", "success", "")
    else:
        mark_and_update(board, slot, "label", "failed", "label verify failed")

    try:
        send_command(port, "9")
        time.sleep(2.0)
    except Exception:
        pass

    return bool(ok_lbl)

def retry_single(board_name: str, slot: int, mode: str) -> bool:
    if CANCEL_EVENT.is_set():
        return False
    if is_board_dead(board_name):
        log(f"[{board_name}] Retry skipped: board marked dead.")
        return False
    port = get_port_for_board(board_name)
    if not port or (_norm_port(port) in DEAD_PORTS):
        log(f"[{board_name}] Retry skipped: port dead or not found.")
        return False
    try:
        if mode in ("DATA_PREF", "MFL_PREF", "LABEL_PREF"):
            send_command(port, "0")
            time.sleep(0.05)
            if not enter_sd_menu(port, board_name):
                return False

        if mode == "DATA":
            return op_data_slot(board_name, port, slot)
        if mode == "DATA_LOG":
            return op_data_log_slot(board_name, port, slot)
        if mode == "DATA_PREF":
            return op_data_pref_slot(board_name, port, slot)
        if mode == "DATA_LOG_PREF":
            return op_data_log_pref_slot(board_name, port, slot)
        if mode == "MFL":
            return op_format_slot(board_name, port, slot, "MFL")
        if mode == "MFL_PREF":
            return op_format_pref_slot(board_name, port, slot)
        if mode == "MFL_ALL_SLOTS":
            return op_mfl_burn_slot(board_name, port, slot)
        if mode == "CMFL_ALL_SLOTS":
            return op_cmfl_burn_slot(board_name, port, slot)
        if mode == "EGP_ALL_SLOTS":
            return op_egp_burn_slot(board_name, port, slot)
        if mode == "AUTO_FORMAT_BURN":
            ok_fmt = op_format_slot(board_name, port, slot, "MFL")
            if not ok_fmt:
                return False
            return op_mfl_burn_slot(board_name, port, slot)
        if mode == "AUTO_FORMAT_BURN_EGP":
            ok_fmt = op_format_slot(board_name, port, slot, "MFL")
            if not ok_fmt:
                return False
            return op_egp_burn_slot(board_name, port, slot)
        if mode == "LABEL":
            return op_label_slot_classic(board_name, port, slot)
        if mode == "LABEL_PREF":
            return op_label_slot(board_name, port, slot)
        return False
    finally:
        try:
            update_simple_log(BOARD_PORTS_CURRENT, MODE_CURRENT)
        except Exception:
            pass

# ---------------------------- SELECTION ----------------------------
def selected_slots_for(board: str, selection: Optional[Dict[str, Set[int]]]) -> List[int]:
    lim = get_slot_limit(board)
    if lim <= 0:
        return []
    excluded = BOARD_EXCLUDE_SLOTS.get(board, set())
    if not selection:
        return [s for s in range(1, lim + 1) if s not in excluded]
    slots = [s for s in selection.get(board, set()) if 1 <= int(s) <= lim and int(s) not in excluded]
    return sorted(int(s) for s in slots)

# ---------------------------- FLOWS ----------------------------
def _cancelled() -> bool:
    return CANCEL_EVENT.is_set()

def process_board_parallel(board: str, port: str, selection: Optional[Dict[str, Set[int]]]) -> None:
    if _cancelled():
        return
    if is_board_dead(board) or (_norm_port(port) in DEAD_PORTS):
        log(f"[{board}] Port dead. Skipping board.")
        return
    if send_command(port, "1") is None:
        log(f"[{board}] Could not enter slot menu; skipping board.")
        return
    time.sleep(0.5)
    for slot in selected_slots_for(board, selection):
        if _cancelled():
            break
        if is_board_dead(board) or (_norm_port(port) in DEAD_PORTS):
            log(f"[{board}] Port dead. Skipping remaining slots.")
            break
        try:
            op_data_slot(board, port, slot, already_in_menu=True)
            time.sleep(0.4)
        finally:
            pass

def process_board_data_log(board: str, port: str, selection: Optional[Dict[str, Set[int]]]) -> None:
    if _cancelled():
        return
    if is_board_dead(board) or (_norm_port(port) in DEAD_PORTS):
        log(f"[{board}] Port dead. Skipping board.")
        return
    if send_command(port, "1") is None:
        log(f"[{board}] Could not enter slot menu; skipping board.")
        return
    time.sleep(0.5)
    for slot in selected_slots_for(board, selection):
        if _cancelled():
            break
        if is_board_dead(board) or (_norm_port(port) in DEAD_PORTS):
            log(f"[{board}] Port dead. Skipping remaining slots.")
            break
        try:
            op_data_log_slot(board, port, slot, already_in_menu=True)
            time.sleep(0.4)
        finally:
            pass

def process_board_data_preferred(board: str, port: str, selection: Optional[Dict[str, Set[int]]]) -> None:
    if _cancelled():
        return
    if not enter_sd_menu(port, board):
        log(f"[{board}] Could not enter SD menu; skipping board.")
        return
    for slot in selected_slots_for(board, selection):
        if _cancelled():
            break
        if is_board_dead(board) or (_norm_port(port) in DEAD_PORTS):
            log(f"[{board}] Port dead. Skipping remaining slots.")
            break
        try:
            op_data_pref_slot(board, port, slot)
            time.sleep(0.4)
        finally:
            pass

def process_board_data_log_preferred(board: str, port: str, selection: Optional[Dict[str, Set[int]]]) -> None:
    if _cancelled():
        return
    if not enter_sd_menu(port, board):
        log(f"[{board}] Could not enter SD menu; skipping board.")
        return
    for slot in selected_slots_for(board, selection):
        if _cancelled():
            break
        if is_board_dead(board) or (_norm_port(port) in DEAD_PORTS):
            log(f"[{board}] Port dead. Skipping remaining slots.")
            break
        try:
            op_data_log_pref_slot(board, port, slot)
            time.sleep(0.4)
        finally:
            pass

def process_board_format(board: str, port: str, mode: str, selection: Optional[Dict[str, Set[int]]]) -> None:
    if _cancelled():
        return
    if is_board_dead(board) or (_norm_port(port) in DEAD_PORTS):
        log(f"[{board}] Port dead. Skipping board.")
        return
    if send_command(port, "1") is None:
        log(f"[{board}] Could not enter slot menu; skipping board.")
        return
    time.sleep(0.5)
    for slot in selected_slots_for(board, selection):
        if _cancelled():
            break
        if is_board_dead(board) or (_norm_port(port) in DEAD_PORTS):
            log(f"[{board}] Port dead. Skipping remaining slots.")
            break
        try:
            op_format_slot(board, port, slot, mode, already_in_menu=True)
            time.sleep(0.25)
        finally:
            pass

def process_board_format_preferred(board: str, port: str, selection: Optional[Dict[str, Set[int]]]) -> None:
    if _cancelled():
        return
    if not enter_sd_menu(port, board):
        log(f"[{board}] Could not enter SD menu; skipping board.")
        return
    for slot in selected_slots_for(board, selection):
        if _cancelled():
            break
        if is_board_dead(board) or (_norm_port(port) in DEAD_PORTS):
            log(f"[{board}] Port dead. Skipping remaining slots.")
            break
        try:
            op_format_pref_slot(board, port, slot)
            time.sleep(0.25)
        finally:
            pass

def process_board_mfl_all_slots(board: str, port: str, selection: Optional[Dict[str, Set[int]]], stagger_seconds: int = 0) -> None:
    if stagger_seconds > 0:
        time.sleep(stagger_seconds)
    if _cancelled():
        return
    if is_board_dead(board) or (_norm_port(port) in DEAD_PORTS):
        log(f"[{board}] Port dead. Skipping board.")
        return
    if send_command(port, "1") is None:
        log(f"[{board}] Could not enter slot menu; skipping board.")
        return
    time.sleep(0.5)
    for slot in selected_slots_for(board, selection):
        if _cancelled():
            break
        if is_board_dead(board) or (_norm_port(port) in DEAD_PORTS):
            log(f"[{board}] Port dead. Skipping remaining slots.")
            break
        try:
            op_mfl_burn_slot(board, port, slot, already_in_menu=True)
            time.sleep(0.25)
        finally:
            pass

def process_board_cmfl_all_slots(board: str, port: str, selection: Optional[Dict[str, Set[int]]], stagger_seconds: int = 0) -> None:
    if stagger_seconds > 0:
        time.sleep(stagger_seconds)
    if _cancelled():
        return
    if is_board_dead(board) or (_norm_port(port) in DEAD_PORTS):
        log(f"[{board}] Port dead. Skipping board.")
        return
    if send_command(port, "1") is None:
        log(f"[{board}] Could not enter slot menu; skipping board.")
        return
    time.sleep(0.5)
    for slot in selected_slots_for(board, selection):
        if _cancelled():
            break
        if is_board_dead(board) or (_norm_port(port) in DEAD_PORTS):
            log(f"[{board}] Port dead. Skipping remaining slots.")
            break
        try:
            op_cmfl_burn_slot(board, port, slot, already_in_menu=True)
            time.sleep(0.25)
        finally:
            pass

def process_board_egp_all_slots(board: str, port: str, selection: Optional[Dict[str, Set[int]]], stagger_seconds: int = 0) -> None:
    if stagger_seconds > 0:
        time.sleep(stagger_seconds)
    if _cancelled():
        return
    if is_board_dead(board) or (_norm_port(port) in DEAD_PORTS):
        log(f"[{board}] Port dead. Skipping board.")
        return
    if send_command(port, "1") is None:
        log(f"[{board}] Could not enter slot menu; skipping board.")
        return
    time.sleep(0.5)
    for slot in selected_slots_for(board, selection):
        if _cancelled():
            break
        if is_board_dead(board) or (_norm_port(port) in DEAD_PORTS):
            log(f"[{board}] Port dead. Skipping remaining slots.")
            break
        try:
            op_egp_burn_slot(board, port, slot, already_in_menu=True)
            time.sleep(0.25)
        finally:
            pass

def process_board_auto_format_and_burn(board: str, port: str, selection: Optional[Dict[str, Set[int]]], stagger_seconds: int = 0) -> None:
    if stagger_seconds > 0:
        time.sleep(stagger_seconds)
    if _cancelled():
        return
    if is_board_dead(board) or (_norm_port(port) in DEAD_PORTS):
        log(f"[{board}] Port dead. Skipping.")
        return
    process_board_format(board, port, mode="MFL", selection=selection)
    if not _cancelled() and not is_board_dead(board) and (_norm_port(port) not in DEAD_PORTS):
        process_board_mfl_all_slots(board, port, selection=selection, stagger_seconds=0)

def process_board_auto_format_and_burn_egp(board: str, port: str, selection: Optional[Dict[str, Set[int]]], stagger_seconds: int = 0) -> None:
    if stagger_seconds > 0:
        time.sleep(stagger_seconds)
    if _cancelled():
        return
    if is_board_dead(board) or (_norm_port(port) in DEAD_PORTS):
        log(f"[{board}] Port dead. Skipping.")
        return
    process_board_format(board, port, mode="MFL", selection=selection)
    if not _cancelled() and not is_board_dead(board) and (_norm_port(port) not in DEAD_PORTS):
        process_board_egp_all_slots(board, port, selection=selection, stagger_seconds=0)

def process_board_label_only(board: str, port: str, selection: Optional[Dict[str, Set[int]]]) -> None:
    if _cancelled():
        return
    if is_board_dead(board) or (_norm_port(port) in DEAD_PORTS):
        log(f"[{board}] Port dead. Skipping remaining slots.")
        return
    if send_command(port, "1") is None:
        log(f"[{board}] Could not enter slot menu; skipping board.")
        return
    time.sleep(0.5)
    for slot in selected_slots_for(board, selection):
        if _cancelled():
            break
        if is_board_dead(board) or (_norm_port(port) in DEAD_PORTS):
            log(f"[{board}] Port dead. Skipping remaining slots.")
            break
        try:
            op_label_slot_classic(board, port, slot, already_in_menu=True)
            time.sleep(0.2)
        finally:
            pass

def process_board_label_preferred(board: str, port: str, selection: Optional[Dict[str, Set[int]]]) -> None:
    if _cancelled():
        return
    if is_board_dead(board) or (_norm_port(port) in DEAD_PORTS):
        log(f"[{board}] Port dead. Skipping board.")
        return
    if not enter_sd_menu(port, board):
        log(f"[{board}] Could not enter SD menu; skipping board.")
        return
    for slot in selected_slots_for(board, selection):
        if _cancelled():
            break
        if is_board_dead(board) or (_norm_port(port) in DEAD_PORTS):
            log(f"[{board}] Port dead. Skipping remaining slots.")
            break
        try:
            op_label_pref_slot(board, port, slot)
            time.sleep(0.2)
        finally:
            pass

# -------- NEW: DATA_PREF coordinator (sequential connect, parallel copy) --------
def process_data_pref_pipelined(selected_boards: List[Tuple[str, str]], selection: Optional[Dict[str, Set[int]]], kick_gap_sec: float = 2.0) -> None:
    states: List[Dict[str, object]] = []
    for board, port in selected_boards:
        if _cancelled():
            return
        if is_board_dead(board) or (_norm_port(port) in DEAD_PORTS):
            log(f"[{board}] Port dead. Skipping board.")
            continue
        if not enter_sd_menu(port, board):
            log(f"[{board}] Could not enter SD menu; skipping board.")
            continue
        slots = selected_slots_for(board, selection)
        states.append({
            "board": board,
            "port": port,
            "pending": slots[:],
            "current": None  # (future, slot)
        })

    if not states:
        return

    max_workers = min(8, max(1, len(states)))
    last_kick = 0.0
    with ThreadPoolExecutor(max_workers=max_workers) as pool:
        while True:
            if _cancelled():
                break
            any_pending_or_running = any(s["pending"] or s["current"] for s in states)
            if not any_pending_or_running:
                break

            now = time.time()
            if now - last_kick >= kick_gap_sec:
                started_any = False
                for s in states:
                    if s["current"] is not None:
                        continue
                    if not s["pending"]:
                        continue
                    board = s["board"]; port = s["port"]
                    if is_board_dead(board) or (_norm_port(port) in DEAD_PORTS):
                        s["pending"].clear()
                        continue

                    slot = s["pending"].pop(0)
                    if send_command(port, str(slot)) is None:
                        mark_and_update(board, slot, "copy", "failed", "serial open failed (slot connect)")
                        continue

                    bidx = board_index(board)
                    prefix = get_board_label_prefix(board)
                    expected_label = build_label(prefix, bidx, slot)
                    sd_drive = wait_for_drive_by_label([expected_label], timeout=15)
                    if not sd_drive:
                        sd_drive = wait_for_new_drive(timeout=20, expect_bootloader=False)
                    if not sd_drive:
                        mark_and_update(board, slot, "copy", "failed", "sd not detected")
                        try:
                            send_command(port, "9")
                        except Exception:
                            pass
                        time.sleep(0.3)
                        continue

                    vol_label = (get_drive_label(sd_drive.strip("\\")) or "").strip().upper()
                    parsed = parse_compact_label(vol_label)

                    if vol_label:
                        by_label_dir = os.path.join("Extracted_Data", "By_Label", vol_label)
                    else:
                        by_label_dir = os.path.join("Extracted_Data", "By_Label", f"UNLABELED_B{bidx}S{slot}")

                    if parsed:
                        _, lbl_bidx, lbl_slot = parsed
                        t_board = find_board_name_by_index(lbl_bidx); t_slot = lbl_slot
                    else:
                        t_board = board; t_slot = slot

                    fut = pool.submit(copy_sdcard_files, sd_drive, by_label_dir, t_board, t_slot)
                    s["current"] = (fut, slot, board, port)
                    last_kick = time.time()
                    started_any = True
                    break  # only one kickoff per gap

                if not started_any:
                    time.sleep(0.05)

            # Harvest finished copies
            for s in states:
                cur = s["current"]
                if not cur:
                    continue
                fut, slot, board, port = cur
                if fut.done():
                    try:
                        fut.result()
                    except Exception as e:
                        mark_and_update(board, slot, "copy", "failed", f"exception: {e}")
                    try:
                        send_command(port, "9")
                    except Exception:
                        pass
                    time.sleep(0.4)
                    s["current"] = None

            time.sleep(0.05)

# ---------------------------- SPECIAL MODES (ODO / INLB) ----------------------------
def process_odo_extraction() -> None:
    target = find_named_board("ODO")
    if not target:
        log("[ODO] Board named 'ODO' not found.")
        return
    board, port = target
    ok = True
    # Try to ensure we are at the main menu first, then enter ODO menu with 'm'.
    # Extra delays added because this menu was flaky in earlier builds.
    try:
        send_command(port, "0")
    except Exception:
        pass
    time.sleep(1.0)

    resp_m = send_command(port, "m")
    if resp_m is None:
        ok = False
    time.sleep(3.0)

    if ok and send_command(port, "1") is None:
        ok = False
    time.sleep(2.0)
    if ok and send_command(port, "2") is None:
        ok = False
    mark_and_update(board, 0, "special", "success" if ok else "failed", "" if ok else "serial failed")
    log(f"[ODO] Extraction sequence {'OK' if ok else 'FAILED'} (2s gaps).")

def process_inlb_extraction() -> None:
    target = find_named_board("INLB")
    if not target:
        log("[INLB] Board named 'INLB' not found.")
        return
    board, port = target
    ok = True
    if send_command(port, "!") is None:
        ok = False
    time.sleep(2.0)
    if ok and send_command(port, "1") is None:
        ok = False
    mark_and_update(board, 0, "special", "success" if ok else "failed", "" if ok else "serial failed")
    log(f"[INLB] Extraction sequence {'OK' if ok else 'FAILED'} (2s gap).")

def process_finish_odo() -> None:
    target = find_named_board("ODO")
    if not target:
        log("[ODO] Board named 'ODO' not found.")
        return
    board, port = target
    ok = True
    if send_command(port, "1") is None:
        ok = False
    time.sleep(2.0)
    if ok and send_command(port, "0") is None:
        ok = False
    time.sleep(2.0)
    if ok and send_command(port, "0") is None:
        ok = False
    mark_and_update(board, 0, "special", "success" if ok else "failed", "" if ok else "serial failed")
    log(f"[ODO] Finish sequence {'OK' if ok else 'FAILED'} (1→0→0 with 2s gaps).")

def process_finish_inlb() -> None:
    target = find_named_board("INLB")
    if not target:
        log("[INLB] Board named 'INLB' not found.")
        return
    board, port = target
    ok = True
    if send_command(port, "8") is None:
        ok = False
    time.sleep(3.0)
    if ok and send_command(port, "9") is None:
        ok = False
    mark_and_update(board, 0, "special", "success" if ok else "failed", "" if ok else "serial failed")
    log(f"[INLB] Finish sequence {'OK' if ok else 'FAILED'} (8→wait3s→9).")

# ---------------------------- SUMMARY ----------------------------
def write_summary(option_mode: str, board_ports: Dict[str, str]) -> None:
    summary = TRACKER.summarize(option_mode)
    json_path = os.path.join(LOG_DIR, f"summary_{RUN_TS}.json")
    with open(json_path, "w", encoding="utf-8") as f:
        json.dump({
            "option": option_mode,
            "started": RUN_START,
            "finished": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
            "boards": list(board_ports.keys()),
            "stats": summary,
            "detail": TRACKER.snapshot()
        }, f, indent=2)

# ---------------------------- ENGINE ----------------------------
def _send_zero_to_selected_boards(selected_boards: List[Tuple[str, str]]) -> None:
    for b, p in selected_boards:
        if CANCEL_EVENT.is_set():
            return
        if not p:
            continue
        if is_board_dead(b) or (_norm_port(p) in DEAD_PORTS):
            continue
        try:
            send_command(p, "0")
        except Exception:
            pass
        time.sleep(0.05)

def process_all_boards_with_selection(mode: str, selection: Optional[Dict[str, Set[int]]]) -> None:
    global MODE_CURRENT, BOARD_PORTS_CURRENT, RUN_SELECTION
    start_new_run_reset()
    MODE_CURRENT = mode
    RUN_SELECTION = selection or None

    BOARD_PORTS_CURRENT = detect_boards()
    board_ports = BOARD_PORTS_CURRENT
    update_simple_log(board_ports, MODE_CURRENT)

    selected_boards = list(board_ports.items())
    if RUN_SELECTION:
        selected_boards = [(b, p) for b, p in selected_boards if selected_slots_for(b, RUN_SELECTION)]

    set_progress_total_from_selected(selected_boards, RUN_SELECTION)

    if not selected_boards and mode not in ("ODO", "INLB"):
        write_summary(mode, board_ports)
        return

    _send_zero_to_selected_boards(selected_boards)

    try:
        if mode == "DATA":
            with ThreadPoolExecutor(max_workers=min(8, max(1, len(selected_boards)))) as executor:
                futures = [executor.submit(process_board_parallel, b, p, RUN_SELECTION) for b, p in selected_boards]
                for f in as_completed(futures):
                    if CANCEL_EVENT.is_set():
                        break
                    try:
                        f.result()
                    except Exception as e:
                        log(f"[ERROR] Worker crashed: {e}")

        elif mode == "DATA_LOG":
            with ThreadPoolExecutor(max_workers=min(8, max(1, len(selected_boards)))) as executor:
                futures = [executor.submit(process_board_data_log, b, p, RUN_SELECTION) for b, p in selected_boards]
                for f in as_completed(futures):
                    if CANCEL_EVENT.is_set():
                        break
                    try:
                        f.result()
                    except Exception as e:
                        log(f"[ERROR] Worker crashed: {e}")

        elif mode == "DATA_PREF":
            process_data_pref_pipelined(selected_boards, RUN_SELECTION, kick_gap_sec=2.0)

        elif mode == "DATA_LOG_PREF":
            for board, port in selected_boards:
                if CANCEL_EVENT.is_set():
                    break
                process_board_data_log_preferred(board, port, RUN_SELECTION)
                time.sleep(0.4)

        elif mode == "MFL":
            for board, port in selected_boards:
                if CANCEL_EVENT.is_set():
                    break
                process_board_format(board, port, mode, RUN_SELECTION)
                time.sleep(0.25)

        elif mode == "MFL_PREF":
            for board, port in selected_boards:
                if CANCEL_EVENT.is_set():
                    break
                process_board_format_preferred(board, port, RUN_SELECTION)
                time.sleep(0.25)

        elif mode == "MFL_ALL_SLOTS":
            for board, port in selected_boards:
                if CANCEL_EVENT.is_set():
                    break
                process_board_mfl_all_slots(board, port, RUN_SELECTION, stagger_seconds=0)
                time.sleep(0.25)

        elif mode == "CMFL_ALL_SLOTS":
            for board, port in selected_boards:
                if CANCEL_EVENT.is_set():
                    break
                process_board_cmfl_all_slots(board, port, RUN_SELECTION, stagger_seconds=0)
                time.sleep(0.25)

        elif mode == "EGP_ALL_SLOTS":
            for board, port in selected_boards:
                if CANCEL_EVENT.is_set():
                    break
                process_board_egp_all_slots(board, port, RUN_SELECTION, stagger_seconds=0)
                time.sleep(0.25)

        elif mode == "AUTO_FORMAT_BURN":
            items = selected_boards
            with ThreadPoolExecutor(max_workers=min(8, max(1, len(items)))) as executor:
                futures = []
                for idx, (board, port) in enumerate(items):
                    futures.append(executor.submit(process_board_auto_format_and_burn, board, port, RUN_SELECTION, stagger_seconds=idx * 3))
                for f in as_completed(futures):
                    if CANCEL_EVENT.is_set():
                        break
                    try:
                        f.result()
                    except Exception as e:
                        log(f"[ERROR] AUTO worker crashed: {e}")

        elif mode == "AUTO_FORMAT_BURN_EGP":
            items = selected_boards
            with ThreadPoolExecutor(max_workers=min(8, max(1, len(items)))) as executor:
                futures = []
                for idx, (board, port) in enumerate(items):
                    futures.append(executor.submit(process_board_auto_format_and_burn_egp, board, port, RUN_SELECTION, stagger_seconds=idx * 3))
                for f in as_completed(futures):
                    if CANCEL_EVENT.is_set():
                        break
                    try:
                        f.result()
                    except Exception as e:
                        log(f"[ERROR] AUTO_EGP worker crashed: {e}")

        elif mode == "LABEL":
            for board, port in selected_boards:
                if CANCEL_EVENT.is_set():
                    break
                process_board_label_only(board, port, RUN_SELECTION)
                time.sleep(0.2)

        elif mode == "LABEL_PREF":
            for board, port in selected_boards:
                if CANCEL_EVENT.is_set():
                    break
                process_board_label_preferred(board, port, RUN_SELECTION)
                time.sleep(0.2)

        elif mode == "ODO":
            process_odo_extraction()

        elif mode == "INLB":
            process_inlb_extraction()

        else:
            log(f"[ERROR] Unknown mode: {mode}")
    finally:
        try:
            update_simple_log(board_ports, MODE_CURRENT)
        except Exception:
            pass
        write_summary(mode, board_ports)

# ---------------------------- GUI ----------------------------
from PyQt5.QtCore import (Qt, QThread, pyqtSignal, QTimer, QEvent, QObject, QPropertyAnimation)
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QLabel,
    QPushButton, QTextEdit, QComboBox, QTableWidget, QTableWidgetItem,
    QSplitter, QSizePolicy, QMessageBox, QGroupBox, QListWidget,
    QLineEdit, QRadioButton, QProgressBar, QAction, QMenuBar, QCheckBox,
    QAbstractItemView, QScrollArea, QDialog, QDialogButtonBox, QFrame, QGridLayout, QSpinBox
)

class QtLogEvent(QEvent):
    TYPE = QEvent.Type(QEvent.registerEventType())
    def __init__(self, message: str):
        super().__init__(QtLogEvent.TYPE)
        self.message = message

class QtLogBridge(QObject):
    _instance = None
    log_signal = pyqtSignal(str)
    def customEvent(self, event):
        if isinstance(event, QtLogEvent):
            self.log_signal.emit(event.message)
    @classmethod
    def instance(cls):
        if cls._instance is None:
            cls._instance = QtLogBridge()
        return cls._instance

# Serial monitor threads/dialog (Advanced)
class SerialReaderThread(QThread):
    received = pyqtSignal(str)
    error = pyqtSignal(str)
    def __init__(self, port_name: str, baud: int = 115200, parent=None):
        super().__init__(parent)
        self.port_name = port_name
        self.baud = baud
        self._stop = threading.Event()
        self._ser = None
    def run(self):
        try:
            self._ser = serial.Serial(self.port_name, self.baud, timeout=0.2)
        except Exception as e:
            self.error.emit(f"Open failed: {e}")
            return
        try:
            while not self._stop.is_set():
                try:
                    data = self._ser.read(1024)
                    if data:
                        try:
                            self.received.emit(data.decode(errors="replace"))
                        except Exception:
                            self.received.emit(repr(data))
                except Exception as e:
                    self.error.emit(f"Read error: {e}")
                    break
        finally:
            try:
                if self._ser:
                    self._ser.close()
            except Exception:
                pass
    def write_line(self, text: str):
        try:
            if self._ser and self._ser.is_open:
                self._ser.write((text + "\r\n").encode())
        except Exception as e:
            self.error.emit(f"Write error: {e}")
    def stop(self):
        self._stop.set()

class AdvancedDialog(QDialog):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Advanced Tools")
        self.resize(900, 600)
        self.reader: Optional[SerialReaderThread] = None

        v = QVBoxLayout(self)
        top = QHBoxLayout()
        self.board_combo = QComboBox()
        for b in sorted(BOARD_PORTS_CURRENT.keys(), key=lambda n: (board_index(n), n)):
            self.board_combo.addItem(f"{b}  ({BOARD_PORTS_CURRENT.get(b, '?')})", b)
        self.btn_connect = QPushButton("Connect")
        self.btn_disconnect = QPushButton("Disconnect")
        self.btn_disconnect.setEnabled(False)
        self.lbl_status = QLabel("Status: idle")
        top.addWidget(QLabel("Board:")); top.addWidget(self.board_combo, 1)
        top.addWidget(self.btn_connect); top.addWidget(self.btn_disconnect)
        top.addStretch(1); top.addWidget(self.lbl_status)
        v.addLayout(top)

        self.monitor = QTextEdit(); self.monitor.setReadOnly(True)
        v.addWidget(self.monitor, 1)
        send_row = QHBoxLayout()
        self.send_line = QLineEdit(); self.send_line.setPlaceholderText("Enter command and press Send (CRLF)")
        self.btn_send = QPushButton("Send")
        send_row.addWidget(self.send_line, 1); send_row.addWidget(self.btn_send)
        v.addLayout(send_row)

        uf_row = QHBoxLayout()
        self.btn_up_data = QPushButton("Upload DATA.uf2 to RPI-RP2")
        self.btn_up_mfl  = QPushButton("Upload MFL.uf2 to RPI-RP2")
        self.btn_up_cmfl = QPushButton("Upload CMFL.uf2 to RPI-RP2")
        self.btn_up_egp  = QPushButton("Upload EGP.uf2 to RPI-RP2")
        uf_row.addWidget(self.btn_up_data); uf_row.addWidget(self.btn_up_mfl); uf_row.addWidget(self.btn_up_cmfl); uf_row.addWidget(self.btn_up_egp); uf_row.addStretch(1)
        v.addLayout(uf_row)

        self.send_line.returnPressed.connect(self.on_send)
        self.btn_connect.clicked.connect(self.on_connect)
        self.btn_disconnect.clicked.connect(self.on_disconnect)
        self.btn_send.clicked.connect(self.on_send)
        self.btn_up_data.clicked.connect(lambda: self.on_upload(DATA_FIRMWARE))
        self.btn_up_mfl.clicked.connect(lambda: self.on_upload(MFL_FIRMWARE))
        self.btn_up_cmfl.clicked.connect(lambda: self.on_upload(CMFL_FIRMWARE))
        self.btn_up_egp.clicked.connect(lambda: self.on_upload(EGP_FIRMWARE))

    def on_connect(self):
        if self.reader and self.reader.isRunning():
            return
        board = self.board_combo.currentData()
        port = BOARD_PORTS_CURRENT.get(board)
        if not port:
            QMessageBox.warning(self, "Connect", "Selected board has no COM port detected.")
            return
        self.reader = SerialReaderThread(port, 115200)
        self.reader.received.connect(self._append_text)
        self.reader.error.connect(self._on_error)
        self.reader.start()
        self.btn_connect.setEnabled(False)
        self.btn_disconnect.setEnabled(True)
        self.lbl_status.setText(f"Status: connected to {port}")

    def on_disconnect(self):
        if self.reader:
            self.reader.stop()
            self.reader.wait(1500)
            self.reader = None
        self.btn_connect.setEnabled(True)
        self.btn_disconnect.setEnabled(False)
        self.lbl_status.setText("Status: disconnected")

    def on_send(self):
        txt = self.send_line.text().strip()
        if not txt:
            return
        if not self.reader or not self.reader.isRunning():
            QMessageBox.information(self, "Send", "Not connected.")
            return
        self.reader.write_line(txt)
        self._append_text(f"> {txt}\n")
        self.send_line.clear()

    def on_upload(self, uf2_path: str):
        ok, fail, msgs = upload_uf2_to_all_rp2(uf2_path)
        for m in msgs:
            self._append_text(m + "\n")
        QMessageBox.information(self, "UF2 Upload", f"OK: {ok}  |  Failed: {fail}")

    def _append_text(self, s: str):
        self.monitor.moveCursor(self.monitor.textCursor().End)
        self.monitor.insertPlainText(s)
        self.monitor.moveCursor(self.monitor.textCursor().End)

    def _on_error(self, msg: str):
        self._append_text(f"[ERR] {msg}\n")
        self.lbl_status.setText(f"Status: {msg}")

    def closeEvent(self, e):
        try:
            self.on_disconnect()
        except Exception:
            pass
        super().closeEvent(e)

# Worker threads
class EngineThread(QThread):
    finished_ok = pyqtSignal()
    failed = pyqtSignal(str)
    def __init__(self, mode: str, selection: Optional[Dict[str, Set[int]]]):
        super().__init__()
        self.mode = mode
        self.selection = selection
        self._err: Optional[str] = None
    def run(self):
        try:
            process_all_boards_with_selection(self.mode, self.selection)
        except Exception as e:
            self._err = str(e)
        finally:
            if self._err:
                self.failed.emit(self._err)
            else:
                self.finished_ok.emit()

class RetryThread(QThread):
    finished_ok = pyqtSignal()
    failed = pyqtSignal(str)
    def __init__(self, tasks: List[Tuple[str, int]], mode: str):
        super().__init__()
        self.tasks = tasks
        self.mode = mode
        self._err: Optional[str] = None
    def run(self):
        try:
            boards = sorted({b for b, _ in self.tasks}, key=lambda n: (board_index(n), n))
            for b in boards:
                if CANCEL_EVENT.is_set():
                    break
                p = get_port_for_board(b)
                if p and (_norm_port(p) not in DEAD_PORTS) and not is_board_dead(b):
                    try:
                        send_command(p, "0")
                    except Exception:
                        pass
                    time.sleep(0.05)

            if self.mode in ("DATA_PREF", "MFL_PREF"):
                for b in boards:
                    if CANCEL_EVENT.is_set():
                        break
                    p = get_port_for_board(b)
                    if p and (_norm_port(p) not in DEAD_PORTS) and not is_board_dead(b):
                        enter_sd_menu(p, b)
                        time.sleep(0.05)

            for (board, slot) in self.tasks:
                if CANCEL_EVENT.is_set():
                    break
                retry_single(board, slot, self.mode)
        except Exception as e:
            self._err = str(e)
        finally:
            if self._err:
                self.failed.emit(self._err)
            else:
                self.finished_ok.emit()

# ---------------------------- THEME/QSS ----------------------------
LIGHT_QSS = """
* { font-family: Segoe UI, Roboto, Arial; font-size: 10.5pt; }
QMainWindow { background-color: #fafafa; }
QGroupBox {
  color: #1e293b; border: 1px solid #e2e8f0; border-radius: 12px;
  margin-top: 12px; background-color: #ffffff;
}
QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 2px 8px; background: #fafafa; border-radius: 8px; }
QLabel { color: #1f2937; }
QTextEdit, QListWidget {
  background: #ffffff; color: #111827; border: 1px solid #e5e7eb; border-radius: 10px;
}
QTableWidget {
  background: #ffffff; color: #0f172a; gridline-color: #e5e7eb; border: 1px solid #e5e7eb; border-radius: 10px;
}
QHeaderView::section {
  background: #f1f5f9; color: #334155; border: 0px; padding: 6px; border-radius: 6px;
}
QLineEdit, QComboBox {
  background: #ffffff; color: #0f172a; border: 1px solid #e5e7eb; border-radius: 10px; padding: 6px;
}
QComboBox QAbstractItemView { background: #ffffff; selection-background-color: #dbeafe; color: #0f172a; border: 1px solid #e5e7eb; }
QPushButton {
  background: qlineargradient(x1:0,y1:0,x2:1,y2:1, stop:0 #3b82f6, stop:1 #22c55e);
  color: white; border: 0px; border-radius: 12px; padding: 8px 14px;
}
QPushButton:hover {
  background: qlineargradient(x1:0,y1:0,x2:1,y2:1, stop:0 #2563eb, stop:1 #16a34a);
}
QPushButton:disabled { background: #e5e7eb; color: #94a3b8; }
QProgressBar {
  background: #ffffff; border: 1px solid #e5e7eb; border-radius: 12px; text-align: center; color: #0f172a; padding: 2px;
}
QProgressBar::chunk {
  border-radius: 10px;
  background: qlineargradient(x1:0,y1:0,x2:1,y2:1, stop:0 #22c55e, stop:1 #3b82f6);
}
QMenuBar { background: #ffffff; color: #1f2937; border-bottom: 1px solid #e5e7eb; }
QMenuBar::item { background: transparent; padding: 6px 12px; }
QMenuBar::item:selected { background: #e5e7eb; border-radius: 8px; }
QMenu {
  background: #ffffff; color: #0f172a; border: 1px solid #e5e7eb; border-radius: 10px;
}
QMenu::item:selected { background: #eef2ff; border-radius: 6px; }
"""

# ---------------------------- PICO & TEENSY SCANNERS ----------------------------
def list_pico_text() -> str:
    lines = ["=== Connected Raspberry Pi Pico Boards ===", ""]
    found = False
    for port in serial.tools.list_ports.comports():
        try:
            if port.vid == 0x2E8A:  # Raspberry Pi Foundation
                found = True
                lines.append(f"Port:         {port.device}")
                lines.append(f"VID:PID:      {hex(port.vid)}:{hex(port.pid)}")
                lines.append(f"Serial Number:{getattr(port,'serial_number', '')}")
                lines.append(f"Description:  {port.description}")
                lines.append("-" * 50)
        except Exception:
            continue
    if not found:
        lines.append("No Raspberry Pi Pico boards detected.")
    return "\n".join(lines)

# Teensy 4.0/4.1 are PJRC VID 0x16C0. Common Serial PIDs include 0x0483, 0x0489, 0x04D9, 0x04DD.
TEENSY_VID = 0x16C0
TEENSY_SERIAL_PIDS = {0x0483, 0x0489, 0x04D9, 0x04DD}

def list_teensy_text() -> str:
    lines = ["=== Connected Teensy Boards (PJRC) ===", ""]
    found = False
    for port in serial.tools.list_ports.comports():
        try:
            vid = getattr(port, "vid", None)
            pid = getattr(port, "pid", None)
            if vid == TEENSY_VID and (pid in TEENSY_SERIAL_PIDS or pid is None):
                found = True
                lines.append(f"Port:         {port.device}")
                vp = f"{hex(vid) if vid is not None else 'N/A'}:{hex(pid) if pid is not None else 'N/A'}"
                lines.append(f"VID:PID:      {vp}")
                lines.append(f"Serial Number:{getattr(port,'serial_number', '')}")
                lines.append(f"Description:  {port.description}")
                lines.append("-" * 50)
        except Exception:
            continue
    if not found:
        lines.append("No Teensy boards detected.")
    return "\n".join(lines)

class PicoListDialog(QDialog):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Connected Pico Serials")
        self.resize(800, 500)
        v = QVBoxLayout(self)
        self.text = QTextEdit(self); self.text.setReadOnly(True)
        self.text.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOn)
        v.addWidget(self.text)
        bb = QDialogButtonBox(QDialogButtonBox.Close)
        bb.rejected.connect(self.reject); bb.accepted.connect(self.accept)
        v.addWidget(bb)
        self.refresh()
    def refresh(self):
        self.text.setPlainText(list_pico_text())

class TeensyListDialog(QDialog):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Connected Teensy Serials")
        self.resize(800, 500)
        v = QVBoxLayout(self)
        self.text = QTextEdit(self); self.text.setReadOnly(True)
        self.text.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOn)
        v.addWidget(self.text)
        bb = QDialogButtonBox(QDialogButtonBox.Close)
        bb.rejected.connect(self.reject); bb.accepted.connect(self.accept)
        v.addWidget(bb)
        self.refresh()
    def refresh(self):
        self.text.setPlainText(list_teensy_text())

# ---------------------------- ABOUT DIALOG ----------------------------
class ScrollableAbout(QDialog):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("About - PIG DATA EXTRACTION UTILITY")
        self.resize(900, 650)
        layout = QVBoxLayout(self)
        scroll = QScrollArea(); scroll.setWidgetResizable(True)
        content = QTextEdit(); content.setReadOnly(True)
        content.setHtml(self.html_text())
        scroll.setWidget(content)
        layout.addWidget(scroll)
    def html_text(self):
     return """
    <h2><b>PIG DATA EXTRACTION UTILITY</b></h2>
    <p>
    This tool automates SD formatting, firmware burning, data extraction, and 
    slot-wise operations across many Pico-based boards in parallel.
    It ensures reliable flashing, labeling, and log handling with full progress tracking.
    </p>
    <hr>
    <h3>📌 <b>HOW TO USE THE UTILITY</b></h3>
    <h4><b>1. Connect all boards</b></h4>
    Plug all HUB boards (via USB-C hub or directly) to the PC before starting.
    <h4><b>2. Scan Connected Picos</h4>
    Use: <b>Tools → List Pico Serials</b> or <b>Scan Picos</b><br>
    <h4><b>3. Scan Connected Teensy</b></h4>
    Use: <b>Tools → List Teensy Serials</b> or <b>Scan TP</b><br>
    <h4><b>4. Add Boards to Registry</b></h4>
    In the <b>Board Registry</b> panel:
    <ul>
      <li>Click <b>Scan Picos</b> / <b>Scan TP</b> to view connected devices.</li>
      <li>Click <b>Add From Detected</b> to import them.</li>
      <li>Assign each entry a <b>Board Name</b> and <b>Type</b> (A-MFL / C-MFL / EGP).</li>
      <li>Optionally set <b>Exclude Slots</b>.</li>
      <li>Click <b>Save Registry</b>.</li>
    </ul>
    The registry is saved to <code>boards_config.json</code> (stored beside the EXE).
    <hr>
    """

# ---------------------------- MAIN WINDOW ----------------------------
class MainWindow(QMainWindow):
    MODES = [
        ("DATA", "1. DATA EXTRACTION FALLBACK"),
        ("MFL", "2. FORMAT ALL SLOTS FALLBACK"),
        ("MFL_ALL_SLOTS", "3. BURN MFL.uf2"),
        ("CMFL_ALL_SLOTS", "4. BURN CMFL.uf2"),
        ("LABEL", "5. LABEL ALL SLOTS"),
        ("EGP_ALL_SLOTS", "6. BURN EGP.uf2 (all slots)"),
        ("AUTO_FORMAT_BURN_EGP", "7. AUTO: Format → Burn EGP"),
        ("DATA_PREF", "8. DATA EXTRACTION PREFERRED"),
        ("MFL_PREF", "9. FORMAT ALL SLOTS PREFERRED"),
        ("ODO", "10. ODO EXTRACTION"),
        ("INLB", "11. INLB EXTRACTION"),
        ("DATA_LOG", "12. DATA EXTRACTION LOG"),
        ("DATA_LOG_PREF", "13. DATA EXTRACTION LOG PREFERRED"),
        ("LABEL_PREF", "14. LABEL SLOTS PREFFERED"),
    ]
    BOARD_TYPES = ["A-MFL", "C-MFL", "EGP"]

    def __init__(self):
        super().__init__()
        self.setWindowTitle("PIG DATA EXTRACTION UTILITY")
        self.resize(1600, 950)
        self.setStyleSheet(LIGHT_QSS)

        self.engine_thread: Optional[EngineThread] = None
        self.retry_thread: Optional[RetryThread] = None
        self._board_order: List[str] = []
        self._timer = QTimer(self); self._timer.setInterval(500); self._timer.timeout.connect(self.refresh_views)
        self._prog_timer = QTimer(self); self._prog_timer.setInterval(400); self._prog_timer.timeout.connect(self.refresh_progress)

        self._build_menubar()

        # Top controls
        top_row = QHBoxLayout()
        self.mode_combo = QComboBox()
        for key, label in self.MODES:
            self.mode_combo.addItem(label, key)
        self.detect_btn = QPushButton("Detect Boards")
        self.start_btn = QPushButton("Start")
        self.cancel_btn = QPushButton("Cancel")
        self.finish_odo_btn = QPushButton("Finish ODO")
        self.finish_inlb_btn = QPushButton("Finish INLB")
        self.open_folder_btn = QPushButton("Open Logs Folder")
        top_row.addWidget(QLabel("Mode:"))
        top_row.addWidget(self.mode_combo)
        top_row.addWidget(self.detect_btn)
        top_row.addWidget(self.start_btn)
        top_row.addWidget(self.cancel_btn)
        top_row.addWidget(self.finish_odo_btn)
        top_row.addWidget(self.finish_inlb_btn)
        top_row.addStretch(1)
        top_row.addWidget(self.open_folder_btn)

        # Progress bar
        self.progress = QProgressBar()
        self.progress.setMinimum(0); self.progress.setMaximum(100); self.progress.setValue(0)
        self.progress_anim = QPropertyAnimation(self.progress, b"value")
        self.progress_anim.setDuration(300)

        # Data extraction options
        self.data_opts_group = QGroupBox("Data Extraction Options")
        data_layout = QGridLayout()
        self.cb_custom_files = QCheckBox("Enable custom file selection (Data/Data Pref)")
        self.cb_custom_files.setToolTip("Copy only a slice of files when extracting data.")
        self.sb_skip = QSpinBox(); self.sb_skip.setRange(0, 100000); self.sb_skip.setValue(0)
        self.sb_take = QSpinBox(); self.sb_take.setRange(0, 100000); self.sb_take.setValue(0)
        self.sb_tail = QSpinBox(); self.sb_tail.setRange(0, 100000); self.sb_tail.setValue(0)
        for sb in (self.sb_skip, self.sb_take, self.sb_tail):
            sb.setEnabled(False)
        data_layout.addWidget(self.cb_custom_files, 0, 0, 1, 3)
        data_layout.addWidget(QLabel("Skip first"), 1, 0)
        data_layout.addWidget(self.sb_skip, 1, 1)
        data_layout.addWidget(QLabel("Copy after skip"), 2, 0)
        data_layout.addWidget(self.sb_take, 2, 1)
        data_layout.addWidget(QLabel("Copy from end"), 3, 0)
        data_layout.addWidget(self.sb_tail, 3, 1)
        data_layout.setColumnStretch(2, 1)
        self.data_opts_group.setLayout(data_layout)

        # Boards list
        boards_group = QGroupBox("Detected Boards")
        boards_layout = QVBoxLayout()
        self.boards_list = QListWidget()
        self.boards_list.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOn)
        self.boards_list.setVerticalScrollMode(QListWidget.ScrollPerPixel)
        self.boards_list.setUniformItemSizes(True)
        self.boards_list.setMinimumHeight(140)
        self.boards_list.setMaximumHeight(240)
        boards_layout.addWidget(self.boards_list)
        boards_group.setLayout(boards_layout)

        # Progress table
        self.table = QTableWidget(0, 8)
        self.table.setHorizontalHeaderLabels([f"S{i}" for i in range(1, 9)])
        self.table.verticalHeader().setVisible(True)
        self.table.setEditTriggers(QTableWidget.NoEditTriggers)
        self.table.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.table.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOn)

        # Selection scope
        sel_group = QGroupBox("Scope")
        sg = QVBoxLayout()
        rb_row = QHBoxLayout()
        self.rb_all = QRadioButton("All detected boards")
        self.rb_custom = QRadioButton("Custom selection")
        self.rb_all.setChecked(True)
        rb_row.addWidget(self.rb_all); rb_row.addWidget(self.rb_custom); rb_row.addStretch(1)
        sg.addLayout(rb_row)

        # Quick selection
        quick_row = QHBoxLayout()
        self.quick_board = QComboBox()
        self.quick_slot = QComboBox()
        for i in range(1, 9):
            self.quick_slot.addItem(f"S{i}", i)
        self.btn_quick_add = QPushButton("Add")
        quick_row.addWidget(QLabel("Quick pick:"))
        self.quick_board.setMinimumWidth(200)
        quick_row.addWidget(self.quick_board)
        quick_row.addWidget(self.quick_slot)
        quick_row.addWidget(self.btn_quick_add)
        quick_row.addStretch(1)
        sg.addLayout(quick_row)

        # Selection matrix
        self.sel_table = QTableWidget(0, 8)
        self.sel_table.setHorizontalHeaderLabels([f"S{i}" for i in range(1, 9)])
        self.sel_table.verticalHeader().setVisible(True)
        self.sel_table.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Preferred)
        self.sel_table.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOn)

        sel_btns = QHBoxLayout()
        self.btn_sel_all = QPushButton("All")
        self.btn_sel_none = QPushButton("None")
        self.btn_sel_inv = QPushButton("Invert")
        sel_btns.addWidget(self.btn_sel_all); sel_btns.addWidget(self.btn_sel_none); sel_btns.addWidget(self.btn_sel_inv); sel_btns.addStretch(1)

        sg.addWidget(QLabel("Custom: select boards × slots (grey cells = not available; × = excluded by registry)"))
        sg.addWidget(self.sel_table)
        sg.addLayout(sel_btns)
        sel_group.setLayout(sg)

        # Registry
        reg_group = QGroupBox("Board Registry (Serial → Name → Type → Exclude Slots)")
        reg_layout = QVBoxLayout()
        self.reg_table = QTableWidget(0, 5)
        self.reg_table.setHorizontalHeaderLabels(["Serial", "Board Name", "Type (A-MFL/C-MFL/EGP)", "Pipe size (inches)", "Exclude Slots (e.g. 2,5)"])
        self.reg_table.setEditTriggers(QAbstractItemView.AllEditTriggers)
        self.reg_table.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOn)
        reg_btn_row = QHBoxLayout()
        self.btn_reg_scan = QPushButton("Scan Picos")
        self.btn_reg_scan_teensy = QPushButton("Scan TP")
        self.btn_reg_add = QPushButton("Add Row")
        self.btn_reg_add_detected = QPushButton("Add From Detected")
        self.btn_reg_del = QPushButton("Delete Row")
        self.btn_reg_save = QPushButton("Save Registry")
        reg_btn_row.addWidget(self.btn_reg_scan)
        reg_btn_row.addWidget(self.btn_reg_scan_teensy)
        reg_btn_row.addStretch(1)
        reg_btn_row.addWidget(self.btn_reg_add_detected)
        reg_btn_row.addWidget(self.btn_reg_add)
        reg_btn_row.addWidget(self.btn_reg_del)
        reg_btn_row.addWidget(self.btn_reg_save)
        reg_layout.addWidget(self.reg_table)
        reg_layout.addLayout(reg_btn_row)
        reg_group.setLayout(reg_layout)

        # Left column (scrollable)
        left = QVBoxLayout()
        left.addLayout(top_row)
        left.addWidget(self.progress)
        left.addWidget(self.data_opts_group)
        left.addWidget(boards_group)
        left.addWidget(QLabel("Progress (✅ ok | ❌ fail | ⏳ pending/running | – n/a | × excluded)"))
        left.addWidget(self.table)
        left.addWidget(sel_group)
        left.addWidget(reg_group)
        left_container = QWidget(); left_container.setLayout(left)
        left_scroll = QScrollArea()
        left_scroll.setWidgetResizable(True)
        left_scroll.setWidget(left_container)
        left_scroll.setVerticalScrollBarPolicy(Qt.ScrollBarAsNeeded)

        # Right column logs + retry
        self.log_text = QTextEdit(); self.log_text.setReadOnly(True)
        self.log_text.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOn)
        self.simple_text = QTextEdit(); self.simple_text.setReadOnly(True)
        self.simple_text.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOn)
        self.simple_text.setMaximumHeight(160)
        self.simple_text.setMinimumHeight(80)

        right = QVBoxLayout()
        lbl_live = QLabel("Live Log")
        lbl_simple = QLabel("Simple Log (auto)")
        right.addWidget(lbl_live)
        right.addWidget(self.log_text)
        right.addWidget(lbl_simple)
        right.addWidget(self.simple_text)

        # Retry panel
        retry_group = QGroupBox("Retry")
        retry_v = QVBoxLayout()

        row1 = QGridLayout()
        self.board_no_in = QLineEdit(); self.board_no_in.setPlaceholderText("Board No (e.g. 3)")
        self.slot_no_in = QLineEdit(); self.slot_no_in.setPlaceholderText("Slot No (1-8)")
        self.retry_one_btn = QPushButton("Retry Selected")
        self.retry_one_btn.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Fixed)

        row1.addWidget(QLabel("Board No:"), 0, 0)
        row1.addWidget(self.board_no_in, 0, 1)
        row1.addWidget(QLabel("Slot No:"), 0, 2)
        row1.addWidget(self.slot_no_in, 0, 3)
        row1.addWidget(self.retry_one_btn, 0, 4)
        row1.setColumnStretch(1, 1)
        row1.setColumnStretch(3, 1)
        retry_v.addLayout(row1)

        line = QFrame(); line.setFrameShape(QFrame.HLine); line.setFrameShadow(QFrame.Sunken)
        retry_v.addWidget(line)

        self.retry_extract_btn = QPushButton("Retry Extraction (failed in 8 → run 1, parallel)")
        self.retry_format_btn  = QPushButton("Retry Formatting (failed in 9 → run 2)")
        for b in (self.retry_extract_btn, self.retry_format_btn):
            b.setMinimumHeight(36)
            b.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
            b.setToolTip(b.text())
        retry_v.addWidget(self.retry_extract_btn)
        retry_v.addWidget(self.retry_format_btn)

        retry_group.setLayout(retry_v)
        right.addWidget(retry_group)

        # Stretch priorities
        right.setStretch(1, 5)
        right.setStretch(3, 1)
        right.setStretch(4, 3)

        rightw = QWidget(); rightw.setLayout(right)
        rightw.setMinimumWidth(380)
        rightw.setMaximumWidth(540)

        splitter = QSplitter(Qt.Horizontal)
        splitter.addWidget(left_scroll)
        splitter.addWidget(rightw)
        splitter.setStretchFactor(0, 6)
        splitter.setStretchFactor(1, 1)
        splitter.setSizes([1250, 350])

        # Bottom stats
        self.stats_label = QLabel("Stats: –")
        bottom = QHBoxLayout(); bottom.addWidget(self.stats_label)

        # Root
        container = QWidget(); root = QVBoxLayout(container)
        root.addWidget(splitter, 1)
        root.addLayout(bottom)
        self.setCentralWidget(container)

        # Wire up
        self.detect_btn.clicked.connect(self.on_detect)
        self.start_btn.clicked.connect(self.on_start)
        self.cancel_btn.clicked.connect(self.on_cancel)
        self.finish_odo_btn.clicked.connect(self.on_finish_odo)
        self.finish_inlb_btn.clicked.connect(self.on_finish_inlb)
        self.open_folder_btn.clicked.connect(self.on_open_folder)
        self.retry_one_btn.clicked.connect(self.on_retry_one)
        self.retry_extract_btn.clicked.connect(self.on_retry_extraction_failed_from_pref_parallel)
        self.retry_format_btn.clicked.connect(self.on_retry_formatting_failed_from_pref_parallel)
        self.rb_all.toggled.connect(self.on_scope_toggle)
        self.mode_combo.currentIndexChanged.connect(self.on_mode_changed)
        self.btn_sel_all.clicked.connect(lambda: self.set_all_selection(True))
        self.btn_sel_none.clicked.connect(lambda: self.set_all_selection(False))
        self.btn_sel_inv.clicked.connect(self.invert_selection)
        self.sel_table.verticalHeader().sectionDoubleClicked.connect(self.on_sel_row_header_double_clicked)
        self.btn_quick_add.clicked.connect(self.on_quick_add)
        self.cb_custom_files.toggled.connect(self.on_file_slice_toggle)
        QtLogBridge.instance().log_signal.connect(self.on_log_line)

        # registry buttons
        self.btn_reg_scan.clicked.connect(self.on_scan_picos)
        self.btn_reg_scan_teensy.clicked.connect(self.on_scan_teensy)
        self.btn_reg_add.clicked.connect(self.on_reg_add_row)
        self.btn_reg_add_detected.clicked.connect(self.on_reg_add_from_detected)
        self.btn_reg_del.clicked.connect(self.on_reg_delete_row)
        self.btn_reg_save.clicked.connect(self.on_reg_save)

        # Init
        self.populate_registry_table_from_file()
        self.on_detect()
        self._timer.start()
        self._prog_timer.start()
        self.on_scope_toggle()
        self.update_data_option_visibility()
        self._animate_progress_to(0)

    # Menubar
    def _build_menubar(self):
        menubar = QMenuBar(self)
        self.setMenuBar(menubar)

        mode_menu = menubar.addMenu("Mode")
        for key, label in self.MODES:
            act = QAction(label, self)
            act.triggered.connect(lambda _, k=key: self._set_mode_from_menu(k))
            mode_menu.addAction(act)

        file_menu = menubar.addMenu("File")
        act_detect = QAction("Detect Boards", self); act_detect.triggered.connect(self.on_detect)
        act_start = QAction("Start", self); act_start.triggered.connect(self.on_start)
        act_cancel = QAction("Cancel", self); act_cancel.triggered.connect(self.on_cancel)
        act_open = QAction("Open Logs Folder", self); act_open.triggered.connect(self.on_open_folder)
        act_exit = QAction("Exit", self); act_exit.triggered.connect(self.close)
        file_menu.addActions([act_detect, act_start, act_cancel, act_open, act_exit])

        tools_menu = menubar.addMenu("Tools")
        act_scan_pico = QAction("List Pico Serials", self); act_scan_pico.triggered.connect(self.show_pico_list_dialog)
        act_scan_teensy = QAction("List Teensy Serials", self); act_scan_teensy.triggered.connect(self.show_teensy_list_dialog)
        tools_menu.addAction(act_scan_pico)
        tools_menu.addAction(act_scan_teensy)

        view_menu = menubar.addMenu("View")
        # simple log refresh removed from menu; auto-updated

        adv_menu = menubar.addMenu("Advanced")
        act_adv = QAction("Advanced Tools…", self); act_adv.triggered.connect(self.show_advanced_tools)
        adv_menu.addAction(act_adv)

        help_menu = menubar.addMenu("Help")
        act_about = QAction("About", self)
        act_about.triggered.connect(lambda: ScrollableAbout(self).exec_())
        help_menu.addAction(act_about)

    def show_advanced_tools(self):
        dlg = AdvancedDialog(self)
        dlg.exec_()

    def show_pico_list_dialog(self):
        dlg = PicoListDialog(self); dlg.exec_()

    def show_teensy_list_dialog(self):
        dlg = TeensyListDialog(self); dlg.exec_()

    def _set_mode_from_menu(self, key: str):
        idx = next((i for i in range(self.mode_combo.count()) if self.mode_combo.itemData(i) == key), -1)
        if idx >= 0:
            self.mode_combo.setCurrentIndex(idx)

    # Animations
    def _animate_progress_to(self, value: int):
        value = max(0, min(100, value))
        self.progress_anim.stop()
        self.progress_anim.setStartValue(self.progress.value())
        self.progress_anim.setEndValue(value)
        self.progress_anim.start()

    def on_mode_changed(self):
        self.update_data_option_visibility()

    def update_data_option_visibility(self) -> None:
        is_data_mode = self.mode_combo.currentData() in ("DATA", "DATA_PREF")
        self.data_opts_group.setVisible(is_data_mode)
        self.on_file_slice_toggle(self.cb_custom_files.isChecked() and is_data_mode)

    def on_file_slice_toggle(self, checked: bool):
        enable_fields = checked and (self.mode_combo.currentData() in ("DATA", "DATA_PREF"))
        for sb in (self.sb_skip, self.sb_take, self.sb_tail):
            sb.setEnabled(enable_fields)

    def _apply_file_slice_from_ui(self, mode: str) -> None:
        if mode in ("DATA", "DATA_PREF"):
            set_file_slice_config(
                enabled=self.cb_custom_files.isChecked(),
                offset=self.sb_skip.value(),
                count=self.sb_take.value(),
                tail=self.sb_tail.value(),
            )
        else:
            set_file_slice_config(False, 0, 0, 0)

    # Registry UI
    def populate_registry_table_from_file(self):
        self.reg_table.setRowCount(0)
        serial_to_board = REGISTRY.get("serial_to_board", {})
        for ser, entry in serial_to_board.items():
            r = self.reg_table.rowCount()
            self.reg_table.insertRow(r)
            self.reg_table.setItem(r, 0, QTableWidgetItem(ser))
            self.reg_table.setItem(r, 1, QTableWidgetItem(entry.get("name", "")))
            self.reg_table.setItem(r, 2, QTableWidgetItem(entry.get("type", "C-MFL")))
            pipe_val = entry.get("pipe_size", "")
            self.reg_table.setItem(r, 3, QTableWidgetItem(str(pipe_val) if pipe_val not in (None, "") else ""))
            excl = entry.get("exclude", "")
            if isinstance(excl, list):
                excl_str = ", ".join(str(x) for x in excl)
            else:
                excl_str = str(excl or "")
            self.reg_table.setItem(r, 4, QTableWidgetItem(excl_str))
        self.reg_table.resizeColumnsToContents()

    def on_reg_add_row(self):
        r = self.reg_table.rowCount()
        self.reg_table.insertRow(r)
        self.reg_table.setItem(r, 0, QTableWidgetItem(""))
        self.reg_table.setItem(r, 1, QTableWidgetItem(f"Board_{r+1}"))
        self.reg_table.setItem(r, 2, QTableWidgetItem("C-MFL"))
        self.reg_table.setItem(r, 3, QTableWidgetItem(""))
        self.reg_table.setItem(r, 4, QTableWidgetItem(""))

    def on_reg_delete_row(self):
        row = self.reg_table.currentRow()
        if row >= 0:
            self.reg_table.removeRow(row)

    def on_reg_add_from_detected(self):
        present_serials = set()
        for i in range(self.reg_table.rowCount()):
            s = self.reg_table.item(i,0)
            if s: present_serials.add(s.text().strip())
        added = 0
        for port in serial.tools.list_ports.comports():
            serno = getattr(port, "serial_number", None)
            if not serno: continue
            if serno in present_serials: continue
            r = self.reg_table.rowCount()
            self.reg_table.insertRow(r)
            self.reg_table.setItem(r, 0, QTableWidgetItem(serno))
            self.reg_table.setItem(r, 1, QTableWidgetItem(f"Board_{serno[-4:]}"))
            # Keep default type inference simple; user can adjust.
            t = "EGP" if "EGP" in (port.description or "").upper() else "C-MFL"
            self.reg_table.setItem(r, 2, QTableWidgetItem(t))
            self.reg_table.setItem(r, 3, QTableWidgetItem(""))
            self.reg_table.setItem(r, 4, QTableWidgetItem(""))
            added += 1
        if added == 0:
            QMessageBox.information(self, "Registry", "No new connected serials found.")

    def on_reg_save(self):
        serial_to_board: Dict[str, Dict] = {}
        for i in range(self.reg_table.rowCount()):
            s_item = self.reg_table.item(i,0); n_item = self.reg_table.item(i,1); t_item = self.reg_table.item(i,2); p_item = self.reg_table.item(i,3); e_item = self.reg_table.item(i,4)
            serial = (s_item.text().strip() if s_item else "")
            name = (n_item.text().strip() if n_item else "")
            btype = (t_item.text().strip().upper() if t_item else "C-MFL")
            pipe_raw = (p_item.text().strip() if p_item else "")
            excl_raw = (e_item.text().strip() if e_item else "")
            if not serial or not name:
                QMessageBox.warning(self, "Registry", f"Row {i+1}: Serial and Name are required.")
                return
            if btype not in self.BOARD_TYPES:
                QMessageBox.warning(self, "Registry", f"Row {i+1}: Type must be one of {', '.join(self.BOARD_TYPES)}.")
                return
            if pipe_raw:
                if not pipe_raw.isdigit():
                    QMessageBox.warning(self, "Registry", f"Row {i+1}: Pipe size must be an integer (inches).")
                    return
                pipe_size = int(pipe_raw)
            else:
                pipe_size = 0
            try:
                excl = {int(x) for x in re.split(r"[,\s]+", excl_raw) if x.strip().isdigit()}
            except Exception:
                excl = set()
            serial_to_board[serial] = {"name": name, "type": btype, "pipe_size": pipe_size, "exclude": sorted(excl)}
        REGISTRY["serial_to_board"] = serial_to_board
        save_registry(REGISTRY)
        QMessageBox.information(self, "Registry", "Saved. Re-detecting boards.")
        self.on_detect()

    def on_scan_picos(self):
        self.show_pico_list_dialog()

    def on_scan_teensy(self):
        self.show_teensy_list_dialog()

    # UI actions
    def on_detect(self) -> None:
        global BOARD_PORTS_CURRENT
        BOARD_PORTS_CURRENT = detect_boards()
        self._board_order = sorted(BOARD_PORTS_CURRENT.keys(), key=lambda n: (board_index(n), n))
        self.boards_list.clear()
        for b in self._board_order:
            self.boards_list.addItem(f"{b}  [{get_board_type(b)}]  ->  {BOARD_PORTS_CURRENT[b]}")
        self.rebuild_progress_table()
        self.rebuild_selection_table(default_checked=True)
        self.reset_progress_model()
        self.quick_board.clear()
        for b in self._board_order:
            self.quick_board.addItem(b, b)

    def on_start(self) -> None:
        if self.engine_thread and self.engine_thread.isRunning():
            QMessageBox.warning(self, "Busy", "A run is already in progress.")
            return
        mode = self.mode_combo.currentData()
        if not mode:
            QMessageBox.warning(self, "Select Mode", "Choose a mode."); return
        if not BOARD_PORTS_CURRENT and mode not in ("ODO", "INLB"):
            self.on_detect()
            if not BOARD_PORTS_CURRENT:
                QMessageBox.warning(self, "No Boards", "No boards detected."); return

        selection = None
        if self.rb_custom.isChecked() and mode not in ("ODO", "INLB"):
            selection = self.collect_selection()
            if not selection:
                QMessageBox.warning(self, "Selection", "No boards/slots selected."); return

        self._apply_file_slice_from_ui(mode)

        # Fresh status/summary at each start
        start_new_run_reset()
        self.reset_progress_model()
        self.refresh_views(force_clear=True)

        self.toggle_controls(False)
        self.clear_session_ui_tables_only()
        self.engine_thread = EngineThread(mode, selection)
        self.engine_thread.finished_ok.connect(self.on_engine_finished)
        self.engine_thread.failed.connect(self.on_engine_failed)
        self.engine_thread.start()

    def on_cancel(self) -> None:
        CANCEL_EVENT.set()
        log("[CANCEL] Requested. Workers will stop soon.")

    def on_finish_odo(self) -> None:
        process_finish_odo()
        self.refresh_views()
        self.load_simple_log()

    def on_finish_inlb(self) -> None:
        process_finish_inlb()
        self.refresh_views()
        self.load_simple_log()

    def on_engine_finished(self) -> None:
        self.toggle_controls(True)
        self.refresh_views()
        self.load_simple_log()
        self.refresh_progress(finalize=True)

    def on_engine_failed(self, err: str) -> None:
        self.toggle_controls(True)
        self.refresh_views()
        self.load_simple_log()
        self.refresh_progress(finalize=True)
        QMessageBox.critical(self, "Run Failed", f"Engine error:\n{err}")

    def on_open_folder(self) -> None:
        try:
            path = LOG_DIR
            if sys.platform.startswith("win"):
                os.startfile(path)  # nosec
            elif sys.platform == "darwin":
                subprocess.call(["open", path])
            else:
                subprocess.call(["xdg-open", path])
        except Exception as e:
            QMessageBox.warning(self, "Open Folder", f"Failed to open logs folder:\n{e}")

    def on_retry_one(self) -> None:
        if self.retry_thread and self.retry_thread.isRunning():
            QMessageBox.warning(self, "Busy", "A retry is already running.")
            return
        try:
            bno = int(self.board_no_in.text().strip())
            sno = int(self.slot_no_in.text().strip())
        except Exception:
            QMessageBox.warning(self, "Input", "Enter valid integers for Board No and Slot No.")
            return
        board = find_board_name_by_index(bno)
        lim = get_slot_limit(board)
        if not (1 <= sno <= max(1, lim)):
            QMessageBox.warning(self, "Input", f"Slot No must be 1–{lim} for {board} ({get_board_type(board)}).")
            return
        if sno in BOARD_EXCLUDE_SLOTS.get(board, set()):
            QMessageBox.information(self, "Excluded", f"Slot {sno} is excluded for {board}.")
            return
        mode = MODE_CURRENT or self.mode_combo.currentData()
        self.retry_thread = RetryThread(tasks=[(board, sno)], mode=mode)
        self.retry_thread.finished_ok.connect(self.on_retry_finished)
        self.retry_thread.failed.connect(self.on_retry_failed)
        self.retry_thread.start()

    # Parallel re-run for failed from Mode 8 -> run Mode 1
    def on_retry_extraction_failed_from_pref_parallel(self) -> None:
        if self.engine_thread and self.engine_thread.isRunning():
            QMessageBox.warning(self, "Busy", "A run is already in progress.")
            return
        if MODE_CURRENT != "DATA_PREF":
            QMessageBox.information(self, "Retry Extraction", "This action only targets failures from the last Mode 8 run.")
            return
        snap = TRACKER.snapshot()
        selection: Dict[str, Set[int]] = {}
        for b, slots in snap.items():
            lim = get_slot_limit(b)
            if lim <= 0:
                continue
            for s, phases in slots.items():
                try:
                    s_int = int(s)
                except Exception:
                    continue
                if s_int < 1 or s_int > lim:
                    continue
                if s_int in BOARD_EXCLUDE_SLOTS.get(b, set()):
                    continue
                done = StatusTracker.slot_done("DATA_PREF", phases if isinstance(phases, dict) else {})
                if not done:
                    selection.setdefault(b, set()).add(s_int)
        if not selection:
            QMessageBox.information(self, "Retry Extraction", "No failed slots found from the last Mode 8 run.")
            return
        self.engine_thread = EngineThread("DATA", selection)
        self.engine_thread.finished_ok.connect(self.on_engine_finished)
        self.engine_thread.failed.connect(self.on_engine_failed)
        self.engine_thread.start()

    # Parallel re-run for failed from Mode 9 -> run Mode 2
    def on_retry_formatting_failed_from_pref_parallel(self) -> None:
        if self.engine_thread and self.engine_thread.isRunning():
            QMessageBox.warning(self, "Busy", "A run is already in progress.")
            return
        if MODE_CURRENT != "MFL_PREF":
            QMessageBox.information(self, "Retry Formatting", "This action only targets failures from the last Mode 9 run.")
            return
        snap = TRACKER.snapshot()
        selection: Dict[str, Set[int]] = {}
        for b, slots in snap.items():
            lim = get_slot_limit(b)
            if lim <= 0:
                continue
            for s, phases in slots.items():
                try:
                    s_int = int(s)
                except Exception:
                    continue
                if s_int < 1 or s_int > lim:
                    continue
                if s_int in BOARD_EXCLUDE_SLOTS.get(b, set()):
                    continue
                done = StatusTracker.slot_done("MFL_PREF", phases if isinstance(phases, dict) else {})
                if not done:
                    selection.setdefault(b, set()).add(s_int)
        if not selection:
            QMessageBox.information(self, "Retry Formatting", "No failed slots found from the last Mode 9 run.")
            return
        self.engine_thread = EngineThread("MFL", selection)
        self.engine_thread.finished_ok.connect(self.on_engine_finished)
        self.engine_thread.failed.connect(self.on_engine_failed)
        self.engine_thread.start()

    def on_retry_finished(self) -> None:
        self.refresh_views()
        self.load_simple_log()

    def on_retry_failed(self, err: str) -> None:
        self.refresh_views()
        self.load_simple_log()
        QMessageBox.critical(self, "Retry Failed", f"Retry error:\n{err}")

    # Quick add
    def on_quick_add(self):
        if not self._board_order:
            return
        self.rb_custom.setChecked(True)
        board = self.quick_board.currentData()
        lim = get_slot_limit(board)
        slot = int(self.quick_slot.currentData())
        if slot > lim:
            QMessageBox.information(self, "Limit", f"{board} ({get_board_type(board)}) has only {lim} slots.")
            return
        if slot in BOARD_EXCLUDE_SLOTS.get(board, set()):
            QMessageBox.information(self, "Excluded", f"{board} slot {slot} is excluded.")
            return
        try:
            row = self._board_order.index(board)
            col = slot - 1
            itm = self.sel_table.item(row, col)
            if itm is None:
                itm = QTableWidgetItem(); itm.setFlags(itm.flags() | Qt.ItemIsUserCheckable)
                self.sel_table.setItem(row, col, itm)
            itm.setCheckState(Qt.Checked)
        except ValueError:
            pass

    # Selection helpers/UI
    def on_scope_toggle(self) -> None:
        custom = self.rb_custom.isChecked()
        self.sel_table.setEnabled(custom)
        self.btn_sel_all.setEnabled(custom)
        self.btn_sel_none.setEnabled(custom)
        self.btn_sel_inv.setEnabled(custom)
        self.quick_board.setEnabled(custom)
        self.quick_slot.setEnabled(custom)
        self.btn_quick_add.setEnabled(custom)

    def rebuild_selection_table(self, default_checked: bool = True) -> None:
        self.sel_table.setRowCount(len(self._board_order))
        for r, b in enumerate(self._board_order):
            self.sel_table.setVerticalHeaderItem(r, QTableWidgetItem(f"{b} [{get_board_type(b)}]"))
            lim = get_slot_limit(b)
            excluded = BOARD_EXCLUDE_SLOTS.get(b, set())
            for c in range(8):
                item = QTableWidgetItem()
                if c < lim:
                    if (c+1) in excluded:
                        item.setFlags(Qt.ItemIsEnabled)
                        item.setText("×")
                    else:
                        item.setFlags(item.flags() | Qt.ItemIsUserCheckable)
                        item.setCheckState(Qt.Checked if default_checked else Qt.Unchecked)
                else:
                    item.setFlags(Qt.ItemIsEnabled)
                    item.setText("–")
                self.sel_table.setItem(r, c, item)
        self.sel_table.resizeColumnsToContents()

    def set_all_selection(self, checked: bool) -> None:
        for r, b in enumerate(self._board_order):
            lim = get_slot_limit(b)
            excluded = BOARD_EXCLUDE_SLOTS.get(b, set())
            for c in range(8):
                it = self.sel_table.item(r, c)
                if it and c < lim and (c+1) not in excluded:
                    it.setCheckState(Qt.Checked if checked else Qt.Unchecked)

    def invert_selection(self) -> None:
        for r, b in enumerate(self._board_order):
            lim = get_slot_limit(b)
            excluded = BOARD_EXCLUDE_SLOTS.get(b, set())
            for c in range(8):
                it = self.sel_table.item(r, c)
                if it and c < lim and (c+1) not in excluded:
                    it.setCheckState(Qt.Unchecked if it.checkState() == Qt.Checked else Qt.Checked)

    def on_sel_row_header_double_clicked(self, row: int) -> None:
        if row < 0 or row >= len(self._board_order):
            return
        board = self._board_order[row]
        lim = get_slot_limit(board)
        excluded = BOARD_EXCLUDE_SLOTS.get(board, set())
        items = []
        checked = 0
        total = 0
        for c in range(8):
            if c >= lim or (c+1) in excluded:
                continue
            it = self.sel_table.item(row, c)
            if not it:
                continue
            total += 1
            if it.checkState() == Qt.Checked:
                checked += 1
            items.append(it)
        if not total:
            return
        target = Qt.Unchecked if checked == total else Qt.Checked
        for it in items:
            it.setCheckState(target)

    def collect_selection(self) -> Dict[str, Set[int]]:
        selection: Dict[str, Set[int]] = {}
        for r, b in enumerate(self._board_order):
            lim = get_slot_limit(b)
            excluded = BOARD_EXCLUDE_SLOTS.get(b, set())
            slots: Set[int] = set()
            for c in range(8):
                it = self.sel_table.item(r, c)
                if it and c < lim and (c+1) not in excluded and it.checkState() == Qt.Checked:
                    slots.add(c + 1)
            if slots:
                selection[b] = slots
        return selection

    # UI helpers
    def toggle_controls(self, enabled: bool) -> None:
        self.mode_combo.setEnabled(enabled)
        self.detect_btn.setEnabled(enabled)
        self.start_btn.setEnabled(enabled)
        self.cancel_btn.setEnabled(not enabled or True)  # cancel stays enabled during run
        self.finish_odo_btn.setEnabled(enabled)
        self.finish_inlb_btn.setEnabled(enabled)
        self.rb_all.setEnabled(enabled)
        self.rb_custom.setEnabled(enabled)
        self.on_scope_toggle()
        self.btn_reg_add.setEnabled(enabled)
        self.btn_reg_add_detected.setEnabled(enabled)
        self.btn_reg_del.setEnabled(enabled)
        self.btn_reg_save.setEnabled(enabled)
        self.btn_reg_scan.setEnabled(enabled)
        self.btn_reg_scan_teensy.setEnabled(enabled)

    def clear_session_ui_tables_only(self) -> None:
        self.log_text.clear()
        self.simple_text.clear()
        self.refresh_views(force_clear=True)

    def rebuild_progress_table(self) -> None:
        self.table.setRowCount(len(self._board_order))
        for r, b in enumerate(self._board_order):
            self.table.setVerticalHeaderItem(r, QTableWidgetItem(f"{b} [{get_board_type(b)}]"))
            lim = get_slot_limit(b)
            excluded = BOARD_EXCLUDE_SLOTS.get(b, set())
            for c in range(8):
                sym = "–"
                if c < lim:
                    sym = "×" if (c+1) in excluded else "⏳"
                self.table.setItem(r, c, QTableWidgetItem(sym))
        self.table.resizeColumnsToContents()

    def refresh_views(self, force_clear: bool = False) -> None:
        snap = TRACKER.snapshot()
        mode = MODE_CURRENT or self.mode_combo.currentData()
        for r, b in enumerate(self._board_order):
            lim = get_slot_limit(b)
            excluded = BOARD_EXCLUDE_SLOTS.get(b, set())
            phases_by_slot = snap.get(b, {})
            for c in range(8):
                item = self.table.item(r, c)
                if c >= lim:
                    if item: item.setText("–")
                    continue
                if (c+1) in excluded:
                    if item: item.setText("×")
                    continue
                slot = c+1
                phases = phases_by_slot.get(str(slot), {})
                sym = "⏳"
                if phases:
                    if StatusTracker.slot_done(mode, phases):
                        sym = "✅"
                    elif any(rec.get("status") == "failed" for rec in phases.values()):
                        sym = "❌"
                    elif any(rec.get("status") == "running" for rec in phases.values()):
                        sym = "⏳"
                if item is None:
                    self.table.setItem(r, c, QTableWidgetItem(sym))
                else:
                    item.setText(sym)
        self.table.resizeColumnsToContents()

        if mode:
            s = TRACKER.summarize(mode)
            self.stats_label.setText(
                f"Boards {s['boards_ok']}/{s['boards_total']} ok | Slots {s['slots_ok']}/{s['slots_total']} ok | Fail {s['slots_fail']}"
            )

    def reset_progress_model(self):
        global PROG_TOTAL_SLOTS, PROG_SLOTS_DONE
        PROG_TOTAL_SLOTS = 0
        PROG_SLOTS_DONE = set()
        self._animate_progress_to(0)

    def refresh_progress(self, finalize: bool=False):
        if PROG_TOTAL_SLOTS <= 0:
            self._animate_progress_to(0); return
        processed = len(PROG_SLOTS_DONE)
        target = int((processed / PROG_TOTAL_SLOTS) * 100)
        if finalize:
            target = 100
        if target != self.progress.value():
            self._animate_progress_to(target)

    def load_simple_log(self) -> None:
        try:
            if os.path.exists(SIMPLE_LOG):
                with open(SIMPLE_LOG, "r", encoding="utf-8") as f:
                    self.simple_text.setPlainText(f.read())
        except Exception as e:
            self.simple_text.setPlainText(f"Failed to load simple log: {e}")

    def on_log_line(self, line: str) -> None:
        self.log_text.append(line)

# ---------------------------- CLI ----------------------------
def main_cli() -> None:
    print("CLI runs on all registry-detected boards/slots.")
    choice = input("Select option (1=DATA,2=MFL,3=MFL_ALL_SLOTS,4=CMFL_ALL_SLOTS,5=LABEL,6=EGP_ALL_SLOTS,7=AUTO_FORMAT_BURN_EGP,8=DATA_PREF,9=MFL_PREF,10=ODO,11=INLB,12=DATA_LOG,13=DATA_LOG_PREF,14=LABEL_PREF): ").strip()
    opt_map = {
        "1":"DATA","2":"MFL","3":"MFL_ALL_SLOTS","4":"CMFL_ALL_SLOTS",
        "5":"LABEL","6":"EGP_ALL_SLOTS","7":"AUTO_FORMAT_BURN_EGP","8":"DATA_PREF","9":"MFL_PREF",
        "10":"ODO","11":"INLB","12":"DATA_LOG","13":"DATA_LOG_PREF","14":"LABEL_PREF"
    }
    mode = opt_map.get(choice, "INVALID")
    if mode == "INVALID":
        print("Invalid choice."); return
    process_all_boards_with_selection(mode, selection=None)

# ---------------------------- GUI ENTRY ----------------------------
def main() -> None:
    app = QApplication(sys.argv)
    win = MainWindow(); win.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1] == "--cli":
        main_cli()
    else:
        main()
