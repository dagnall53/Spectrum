import sys
import threading
import numpy as np
import os, ctypes
import time
import math
target_dir = r"C:\Spectrum"

# ---- Portable-build helpers (PyInstaller) ----
# When frozen into an .exe, the RTL-SDR driver DLLs and pyrtlsdr_local are bundled with
# the program instead of living in C:\Spectrum. DRIVER_ROOT is the folder that contains
# the default / ms64 / sdrsharp / experimental sub-folders. Falls back to the original
# C:/Spectrum/drivers location when running as a plain .py on the dev PC.
def _find_driver_root():
    cands = []
    if getattr(sys, "frozen", False):
        cands.append(os.path.join(getattr(sys, "_MEIPASS", ""), "drivers"))
        cands.append(os.path.join(os.path.dirname(sys.executable), "drivers"))
    else:
        cands.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), "drivers"))
    cands.append("C:/Spectrum/drivers")
    for c in cands:
        if c and os.path.isdir(c):
            return c.replace("\\", "/")
    return "C:/Spectrum/drivers"

DRIVER_ROOT = _find_driver_root()

def _add_dll_dir(path):
    # os.add_dll_directory raises if the folder is missing; also put it on PATH so
    # the driver's dependent DLLs (libusb etc.) are always found.
    try:
        if os.path.isdir(path):
            os.add_dll_directory(path)
            os.environ["PATH"] = path + os.pathsep + os.environ.get("PATH", "")
    except Exception as e:
        print(f"warning: could not add DLL directory {path!r}: {e}")
# add the Current directory as it contains the crucial dll and retry
# os.add_dll_directory(target_dir)
# print(f"Called os.add_dll_directory({target_dir!r})")

#--------------------------------------------------------------
# Notes for calibration. 
# With a 5W RF transmitter at 5m  with 30db attenuator fitted between TX and antenna, 
# a signal strength of aproximately 87dBuv is expected.
# when my atenuator arrives I hope to be able to properly link the CISPR and FFT scales. 

#--------------------------------------------------------------

from PyQt5 import QtWidgets, QtCore, QtGui
import pyqtgraph as pg

init_done = False
ENABLE_DRIVER_SELECTION=False
# # ---- SWEEP PROFILE ( variables) ----
START_FREQ = 95e6       # 150 kHz
STOP_FREQ  = 102e6        # 30 MHz
#STEP_HZ    = 2000       # 300 kHz per center
SAMPLE_RATE = 2.4e6      # Hz
NFFT        = 4096
GAIN_DB        = 120     # Requested gain, in dB - NOT a raw device table index. The
                         # device only supports specific dB steps (query self.sdr.get_gains()
                         # for the list, given in TENTHS of dB) and silently rounds/clamps
                         # to the nearest one it has - e.g. on the RTL-SDR Blog V4, 49.6 dB
                         # is the maximum, so this default of 120 dB actually just requests
                         # (and receives) the device's max, 49.6 dB. See init_sdr(), which
                         # reads back and logs the actual applied gain every time it runs.

# ---- Stitching / edge-masking ----
# Each FFT block's outer edges show visible artefacts (front-end filter roll-off and/or
# window-shape effects), which show up as a regular "wave" pattern once many blocks are
# stitched edge-to-edge into one wideband trace. STITCH_EDGE_MASK_FRACTION is the fraction
# of each block's bandwidth discarded from EACH edge before it is stored/displayed; only
# the middle (1 - 2*STITCH_EDGE_MASK_FRACTION) of every block is kept. Centers are hopped
# by that same "kept" width (see build_centers()) so the kept slices tile the sweep with
# no gaps and no overlap. Adjust this constant if the FFT windowing/filtering changes.
STITCH_EDGE_MASK_FRACTION = 0.15   # 15% discarded off each edge (30% of each block total)

# Sample rates offered on the live Sample Rate control (see EmcScanner.__init__). Derived
# from the distinct sample rates actually used across self.presets below - kept as its own
# list (rather than scraped from the presets automatically) so the dropdown's order and
# contents stay predictable even if presets are added/edited later.
# The RTL-SDR only accepts sample rates in two ranges: 225,001-300,000 S/s and
# 900,001-3,200,000 S/s (nothing between 0.3 and 0.9 MS/s works). This list covers both
# ranges with the commonly used rates; add any other valid rate here if you need it.
# NB: rates above ~2.56 MS/s can drop samples on some USB ports/PCs.
# ---- previous sample-rate list (0.25 - 3.2 MS/s, 18 values), kept in case of problems ----
# SAMPLE_RATE_CHOICES = [0.25e6, 0.3e6,
#                        1.0e6, 1.024e6, 1.2e6, 1.4e6, 1.5e6, 1.6e6, 1.8e6, 1.92e6, 2.0e6, 2.048e6,
#                        2.4e6, 2.56e6, 2.8e6, 2.88e6, 3.0e6, 3.2e6]

# Only the sample rates the RTL2832U handles well are offered (the same short list SDR# and most SDR programs use).
#   * 225-300 kS/s is valid but narrow; 300-900 kS/s is NOT accepted by the chip's resampler (librtlsdr refuses it), and
#     900 kS/s itself is invalid - SDR# offers 900.001 kS/s for that reason. None of these low rates are offered here.
#   * 2.4 - 2.56 MS/s is the best balance of bandwidth and stable USB transfer; 2.048 MS/s is the usual safe default.
#   * 3.2 MS/s is supported but samples are dropped on many PCs / USB ports, and 3.0 - 3.2 MS/s showed a regular pulse
#     train on this receiver (not seen at 2.4 MS/s), so treat 3.2 MS/s results with caution.
SAMPLE_RATE_RECOMMENDED = [1.024e6, 1.8e6, 2.048e6, 2.4e6, 2.56e6, 3.2e6]
SAMPLE_RATE_CHOICES = list(SAMPLE_RATE_RECOMMENDED)
RTL_VALID_RATE_RANGES = ((225001, 300000), (900001, 3200000))   # S/s, inclusive

# ---------------------------------------------------------------------------------------
# External settings file (settings.csv)
# ---------------------------------------------------------------------------------------
# The sweep defaults and the MODE preset buttons are read from settings.csv at start-up.
# If the file is missing or unusable, the built-in _FALLBACK_PRESETS below are used so the
# program still runs. See settings.csv itself for the column layout.
import csv, re

SETTINGS_FILENAME = "settings.csv"

# Built-in fallback (the original hard-coded presets). Each entry:
# (internal_name, label, start_hz, stop_hz, sample_rate, gain_db, hf_mode_flag, dbuv_calibration)
_FALLBACK_PRESETS = [
    ("LF2", "150k-2M 3.2", 150e3, 2e6, 3.2e6, 40.2, False, -47),
    ("LF3", "2M-6 3.2", 2e6, 6e6, 3.2e6, 40.2, False, -47),
    ("LF4", "6M-12 3.2", 6e6, 12e6, 3.2e6, 40.2, False, -47),
    ("30M3", "12-30Mhz 3.2", 12e6, 30e6, 3.2e6, 40.2, False, -47),
    ("30-200M", "30-200Mhz 3", 30e6, 200e6, 3.2e6, 40.2, False, -47),
    (" 200-900", "200-900Mhz 3", 200e6, 900e6, 3.2e6, 40.2, False, -47),
    ("900M-2.4", "900Mhz - 2.4G 3", 900e6, 24000e6, 3.2e6, 40.2, False, -47),
    ("MVHF", "Marine VHF", 156e6, 162e6, 2.4e6, 40.2, False, -47),
    ("VHF", "Broadcast VHF", 88e6, 108e6, 2.4e6, 40.2, False, -47),
    ("VHF", "Broadcast VHF lw gain", 88e6, 108e6, 2.4e6, 20, False, -47),
    ("VHF HG", "High Gain  Bcst VHF", 88e6, 108e6, 2.4e6, 40.2, False, -47),
    ("FM_WB", "100.3M Wideband", 100.3e6, 100.3e6, 2.4e6, 40.2, False, -47),
    ("MVHF_WB", "Marine VHF Wideband", 156.875e6, 156.875e6, 2.4e6, 40.2, False, -47),
]


def _settings_search_paths():
    """Folders searched for settings.csv, in order: next to the .exe (portable build),
    next to this script, the current working directory, then the old C:/Spectrum folder."""
    paths = []
    if getattr(sys, "frozen", False):
        paths.append(os.path.join(os.path.dirname(sys.executable), SETTINGS_FILENAME))
    paths.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), SETTINGS_FILENAME))
    paths.append(os.path.join(os.getcwd(), SETTINGS_FILENAME))
    paths.append(os.path.join("C:/Spectrum", SETTINGS_FILENAME))
    return paths


def _parse_hz(text):
    """'150k' -> 150000.0, '2M' -> 2e6, '156.875M' -> 156875000.0, '2.4G' -> 2.4e9.
    A plain number is taken as Hz ('2400000' or '2.4e6'). A trailing 'Hz' is ignored.
    k / M / G are case-insensitive and always mean kilo / mega / giga."""
    t = str(text).strip().lower().replace(" ", "")
    if t.endswith("hz"):
        t = t[:-2]
    exp = {"k": 3, "m": 6, "g": 9}.get(t[-1:] if t else "", None)
    if exp is not None:
        t = t[:-1]
        # build the float from text ('3.2e6') so it equals the literal 3.2e6 exactly
        return float(f"{t}e{exp}")
    return float(t)


def _parse_bool(text):
    t = str(text).strip().lower()
    if t in ("1", "true", "yes", "y", "on"):
        return True
    if t in ("", "0", "false", "no", "n", "off"):
        return False
    raise ValueError(f"not a true/false value: {text!r}")


# ---- previous load_settings() (no "setting" rows), kept in case of problems ----
# def load_settings():
#     """Read settings.csv. Returns (defaults_dict, presets_list, path_used).
#     defaults_dict may hold start_freq, stop_freq, sample_rate, gain_db (any that were
#     given). presets_list uses the same tuple layout as _FALLBACK_PRESETS. On any
#     problem with the file as a whole, falls back to the built-in presets; a bad single
#     row is reported and skipped."""
#     path = next((p for p in _settings_search_paths() if os.path.isfile(p)), None)
#     if path is None:
#         print(f"settings: {SETTINGS_FILENAME} not found - using built-in presets")
#         return {}, list(_FALLBACK_PRESETS), None
#     defaults, presets = {}, []
#     try:
#         with open(path, newline="", encoding="utf-8-sig") as fh:
#             lines = [ln for ln in fh if ln.strip() and not ln.lstrip().startswith("#")]
#         reader = csv.DictReader(lines, skipinitialspace=True)
#         reader.fieldnames = [(n or "").strip().lower() for n in (reader.fieldnames or [])]
#         for rownum, row in enumerate(reader, start=2):
#             kind = (row.get("type") or "").strip().lower()
#             try:
#                 start = _parse_hz(row["start"])
#                 stop = _parse_hz(row["stop"])
#                 rate = _parse_hz(row["sample_rate"])
#                 gain = float(str(row["gain_db"]).strip())
#                 if start <= 0 or stop < start or rate <= 0:
#                     raise ValueError("need 0 < start <= stop and sample_rate > 0")
#                 if not any(lo <= rate <= hi for lo, hi in RTL_VALID_RATE_RANGES):
#                     print(f"settings: row {rownum}: sample_rate {rate:.0f} S/s is outside the "
#                           f"RTL-SDR's valid ranges (225001-300000, 900001-3200000) - the "
#                           f"driver will probably refuse it")
#                 if kind == "default":
#                     defaults = {"start_freq": start, "stop_freq": stop,
#                                 "sample_rate": rate, "gain_db": gain}
#                 elif kind == "preset":
#                     label = (row.get("label") or "").strip()
#                     if not label:
#                         raise ValueError("preset needs a label (the button text)")
#                     name = (row.get("name") or "").strip() or label
#                     hf = _parse_bool(row.get("hf_mode") or "")
#                     cal = float(str(row.get("dbuv_cal") or "0").strip() or 0)
#                     presets.append((name, label, start, stop, rate, gain, hf, cal))
#                 else:
#                     raise ValueError(f"unknown type {kind!r} (use 'default' or 'preset')")
#             except Exception as e:
#                 print(f"settings: {os.path.basename(path)} row {rownum} skipped: {e}")
#     except Exception as e:
#         print(f"settings: could not read {path}: {e} - using built-in presets")
#         return {}, list(_FALLBACK_PRESETS), None
#     if not presets:
#         print(f"settings: no valid presets in {path} - using built-in presets")
#         presets = list(_FALLBACK_PRESETS)
#     print(f"settings: loaded {len(presets)} presets from {path}")
#     return defaults, presets, path

# Program settings read from settings.csv rows of type "setting" (key in the name column, value in the
# label column - or a "value" column if you add one). Filled in by load_settings().
SETTINGS_EXTRA = {}
_KNOWN_SETTINGS = ("dc_offset_removal", "iq_balance_correction", "driver", "driver_root", "driver_dialog")


def load_settings():
    """Read settings.csv. Returns (defaults_dict, presets_list, path_used).
    defaults_dict may hold start_freq, stop_freq, sample_rate, gain_db (any that were
    given). presets_list uses the same tuple layout as _FALLBACK_PRESETS. Rows of type
    "setting" (program options such as dc_offset_removal, iq_balance_correction, driver,
    driver_root, driver_dialog) go into the module-level SETTINGS_EXTRA dict. On any
    problem with the file as a whole, falls back to the built-in presets; a bad single
    row is reported and skipped."""
    SETTINGS_EXTRA.clear()
    path = next((p for p in _settings_search_paths() if os.path.isfile(p)), None)
    if path is None:
        print(f"settings: {SETTINGS_FILENAME} not found - using built-in presets")
        return {}, list(_FALLBACK_PRESETS), None

    defaults, presets = {}, []
    try:
        with open(path, newline="", encoding="utf-8-sig") as fh:
            lines = [ln for ln in fh if ln.strip() and not ln.lstrip().startswith("#")]
        reader = csv.DictReader(lines, skipinitialspace=True)
        reader.fieldnames = [(n or "").strip().lower() for n in (reader.fieldnames or [])]
        for rownum, row in enumerate(reader, start=2):
            kind = (row.get("type") or "").strip().lower()
            try:
                if kind == "setting":
                    key = (row.get("name") or "").strip().lower()
                    val = (row.get("value") or row.get("label") or "").strip()
                    if not key:
                        raise ValueError("a setting needs a name (the setting key)")
                    if key not in _KNOWN_SETTINGS:
                        print(f"settings: row {rownum}: unknown setting {key!r} (known: {', '.join(_KNOWN_SETTINGS)})")
                    SETTINGS_EXTRA[key] = val
                    continue
                start = _parse_hz(row["start"])
                stop = _parse_hz(row["stop"])
                rate = _parse_hz(row["sample_rate"])
                gain = float(str(row["gain_db"]).strip())
                if start <= 0 or stop < start or rate <= 0:
                    raise ValueError("need 0 < start <= stop and sample_rate > 0")
                if not any(lo <= rate <= hi for lo, hi in RTL_VALID_RATE_RANGES):
                    print(f"settings: row {rownum}: sample_rate {rate:.0f} S/s is outside the "
                          f"RTL-SDR's valid ranges (225001-300000, 900001-3200000) - the "
                          f"driver will probably refuse it")
                if kind == "default":
                    defaults = {"start_freq": start, "stop_freq": stop,
                                "sample_rate": rate, "gain_db": gain}
                elif kind == "preset":
                    label = (row.get("label") or "").strip()
                    if not label:
                        raise ValueError("preset needs a label (the button text)")
                    name = (row.get("name") or "").strip() or label
                    hf = _parse_bool(row.get("hf_mode") or "")
                    cal = float(str(row.get("dbuv_cal") or "0").strip() or 0)
                    presets.append((name, label, start, stop, rate, gain, hf, cal))
                else:
                    raise ValueError(f"unknown type {kind!r} (use 'default', 'preset' or 'setting')")
            except Exception as e:
                print(f"settings: {os.path.basename(path)} row {rownum} skipped: {e}")
    except Exception as e:
        print(f"settings: could not read {path}: {e} - using built-in presets")
        return {}, list(_FALLBACK_PRESETS), None

    if not presets:
        print(f"settings: no valid presets in {path} - using built-in presets")
        presets = list(_FALLBACK_PRESETS)
    print(f"settings: loaded {len(presets)} presets from {path}")
    if SETTINGS_EXTRA:
        print("settings: program settings: " + ", ".join(f"{k}={v}" for k, v in SETTINGS_EXTRA.items()))
    return defaults, presets, path


_SETTINGS_DEFAULTS, PRESETS, SETTINGS_PATH = load_settings()
START_FREQ = _SETTINGS_DEFAULTS.get("start_freq", START_FREQ)
STOP_FREQ = _SETTINGS_DEFAULTS.get("stop_freq", STOP_FREQ)
SAMPLE_RATE = _SETTINGS_DEFAULTS.get("sample_rate", SAMPLE_RATE)
GAIN_DB = _SETTINGS_DEFAULTS.get("gain_db", GAIN_DB)


def _setting_bool(key, default):
    """A true/false program setting from settings.csv (1/0, on/off, yes/no, true/false); default if absent."""
    v = SETTINGS_EXTRA.get(key)
    if v is None or v == "":
        return default
    try:
        return _parse_bool(v)
    except ValueError:
        print(f"settings: {key} = {v!r} is not on/off - using {'on' if default else 'off'}")
        return default


def _resolve_dir(p):
    """A folder named in settings.csv: absolute, or relative to the settings file / the program."""
    p = os.path.expanduser(str(p).strip()).replace("\\", "/")
    if os.path.isabs(p):
        return p
    bases = []
    if SETTINGS_PATH:
        bases.append(os.path.dirname(SETTINGS_PATH))
    if getattr(sys, "frozen", False):
        bases.append(os.path.dirname(sys.executable))
    bases.append(os.path.dirname(os.path.abspath(__file__)))
    for b in bases:
        c = os.path.join(b, p)
        if os.path.isdir(c):
            return c.replace("\\", "/")
    return os.path.join(bases[0], p).replace("\\", "/")


# Receiver artefact corrections - both default ON; switch off in settings.csv with
#   setting,dc_offset_removal,off      /      setting,iq_balance_correction,off
DC_OFFSET_REMOVAL = _setting_bool("dc_offset_removal", True)
IQ_BALANCE_CORRECTION = _setting_bool("iq_balance_correction", True)
# RTL-SDR driver choice (experiment with alternates from settings.csv):
#   setting,driver,default | ms64 | sdrsharp | experimental | <folder under the driver root> | <full path>
#   setting,driver_root,<folder holding the driver sub-folders>      (optional)
#   setting,driver_dialog,on                                          (ask at start-up instead)
DRIVER_CHOICE = (SETTINGS_EXTRA.get("driver") or "default").strip()
if SETTINGS_EXTRA.get("driver_root"):
    DRIVER_ROOT = _resolve_dir(SETTINGS_EXTRA["driver_root"])
ENABLE_DRIVER_SELECTION = _setting_bool("driver_dialog", ENABLE_DRIVER_SELECTION)
# Any preset sample rate that is not already offered gets added to the live Sample Rate
# dropdown, so the combo can always display the active preset's rate.
SAMPLE_RATE_CHOICES = sorted(set(SAMPLE_RATE_CHOICES) | {p[4] for p in PRESETS} | {SAMPLE_RATE})

# RF gain steps (dB) the RTL-SDR tuner actually supports - the live RF Gain slider moves
# through these, so every slider position is a real hardware gain (the driver would snap
# any other value to the nearest of these anyway). Matches the "Supported gains" list the
# program prints when it opens the device (RTL-SDR Blog V4), which is given in tenths of dB.
RTL_GAIN_STEPS_DB = [g / 10.0 for g in (
    0, 9, 14, 27, 37, 77, 87, 125, 144, 157, 166, 197, 207, 229, 254,
    280, 297, 328, 338, 364, 372, 386, 402, 421, 434, 439, 445, 480, 496)]

# ---- Gain / sample-rate SURVEY defaults (see EmcScanner.survey_open_dialog) ----
SURVEY_MAX_STOP_HZ = 1.2e6     # the survey only runs on bands that end at or below this frequency...
# ...and only at sample rates whose KEPT (edge-masked) width covers the whole band in a single
# block, so there is no hopping and no stitching during a survey (see survey_rate_fits()).
SURVEY_SPIKE_THRESH_DB = 6.0   # a bin must stand this far above its local baseline to count as a spike
SURVEY_SURFACE_POINTS = 800      # default number of frequency points per surface-chart CSV (Excel copes with a few thousand)
SURVEY_FULL_NPZ_MAX_BYTES = 250e6   # full-resolution spectra are kept for the .npz only while they fit in this much memory
SURVEY_REPORT_EVERY_S = 60.0           # while a survey runs, the single report file is rewritten this often
SURVEY_OPEN_REPORT = True              # open the finished report in the default browser
SURVEY_REPORT_MAX_SPIKES = 60000       # most spikes embedded in the report (the strongest ones)
SURVEY_EXPECTED_GAIN_LIMIT_DB = 20.0     # the operator's visual estimate of where distortion starts; the report checks it
SURVEY_MAX_SCANS = 5                   # most scans of one rate/gain setting (2 if they agree, more if they do not)
SURVEY_SCANS_ESTIMATE = 2.2            # average scans per setting, for the time estimate in the survey dialog
SURVEY_REPEAT_SHIFT_DB = 1.0           # two scans differ if their median level differs by more than this
SURVEY_REPEAT_LOCAL_DB = 3.0           # ... or more than SURVEY_REPEAT_LOCAL_FRAC of the (8-bin averaged) bins differ by more than this
SURVEY_REPEAT_LOCAL_FRAC = 0.01        #     (the limit is raised automatically when averaging is low and the noise scatter is larger)
SURVEY_REPEAT_SPIKE_PROM_DB = 10.0     # ... or a spike this strong is in one scan and missing in the other
import json
# (previous report page, kept in case of problems - no longer used)
_SURVEY_REPORT_TEMPLATE_V1 = r'''<!DOCTYPE html>
<html lang="en"><head><meta charset="utf-8"><title>RF gain / sample-rate survey</title>
<style>
body{background:#16181c;color:#d8dbe0;font:14px/1.45 Segoe UI,Arial,sans-serif;margin:0 auto;max-width:1180px;padding:16px 20px 60px}
h1{font-size:22px;margin:4px 0 2px}h2{font-size:17px;margin:28px 0 6px;border-bottom:1px solid #333;padding-bottom:3px}
.sub{color:#9aa0a8}.note{color:#9aa0a8;font-size:13px;margin:2px 0 8px}
canvas{display:block;width:100%;max-width:1100px;background:#14161a;border:1px solid #2a2d33}
.ctl{margin:6px 0}select,button{background:#262a31;color:#d8dbe0;border:1px solid #444;border-radius:4px;padding:3px 8px;font:inherit}
button{cursor:pointer}button:hover{background:#323843}
.find{margin:0;padding:0;list-style:none}.find li{padding:6px 10px;margin:4px 0;border-left:4px solid #4a90d9;background:#1d2026}
.find li.warn{border-left-color:#e0a030}.find li.bad{border-left-color:#e05050}.find li.ok{border-left-color:#5aa469}
table{border-collapse:collapse;width:100%;font-size:13px}th,td{padding:4px 7px;border-bottom:1px solid #2a2d33;text-align:left}
th{background:#1d2026;position:sticky;top:0;cursor:pointer}tr.sel{background:#2b3442}tr.row:hover{background:#242932;cursor:pointer}
.tag{display:inline-block;padding:0 6px;border-radius:9px;font-size:12px;margin-right:3px}
.t-stable{background:#2f5d3a}.t-rate{background:#7a4a1c}.t-block{background:#6a2d5e}.t-lin{background:#2b4f7a}.t-fix{background:#5a5a2a}.t-sup{background:#8a2a2a}.t-cmp{background:#8a5a1a}.t-ons{background:#8a2a5a}
.hv{color:#ffd54a;margin-left:10px;font-family:Consolas,monospace;font-size:13px}
.scroll{max-height:420px;overflow:auto;border:1px solid #2a2d33}
.btns button{margin:3px 6px 3px 0}.legend span{margin-right:14px;font-size:13px}
</style></head><body>
<h1>RF scan - gain / sample-rate survey</h1>
<div class="sub" id="meta"></div>

<h2>Findings</h2>
<ul class="find" id="findings"></ul>

<h2>1. Sample-rate effects</h2>
<p class="note">Each row is one sample rate (lowest at the bottom), colour = level at the chosen gain. A real or rate-independent signal is a <b>vertical line in every row</b>. A feature that shifts, appears or vanishes from row to row depends on the sample rate.</p>
<div class="ctl">Gain: <select id="gSel"></select><span class="hv" id="hv1"></span></div>
<canvas id="heatRate" width="1100" height="320"></canvas>

<h2>2. Gain effects</h2>
<p class="note">Each row is one RF gain (lowest at the bottom) at the chosen sample rate. A signal arriving from the antenna brightens evenly with gain; a feature that appears only at higher gain, or brightens faster than its neighbours, is made inside the gain stages (compression / clipping / distortion).</p>
<div class="ctl">Sample rate: <select id="rSel"></select><span class="hv" id="hv2"></span></div>
<canvas id="heatGain" width="1100" height="320"></canvas>

<h2>3. Spectrum viewer</h2>
<p class="note">Overlay up to three sample-rate / gain combinations. Drag on the plot to zoom, double-click to reset.</p>
<div class="ctl" id="trSel"></div>
<canvas id="specPlot" width="1100" height="340"></canvas>
<div class="legend" id="specLegend"></div>

<h2>4. Features found</h2>
<p class="note">Click a row to plot its level against gain. "Gain slope" is the change in the feature's level per dB of RF gain: about 1 means it arrives with the signal; about 0 means it is generated after the gain stage (ADC / clock / digital); above about 1.6 means it rises faster than the gain (distortion or overload product).</p>
<div class="scroll"><table id="featTable"></table></div>
<canvas id="featPlot" width="1100" height="300" style="margin-top:8px"></canvas>
<div class="legend" id="featLegend"></div>

<h2>5. Noise floor and ADC headroom</h2>
<p class="note">Left: median noise floor against gain. Right: strongest component against gain; the dashed line is the ADC full-scale equivalent on this level scale.</p>
<canvas id="floorPlot" width="1100" height="300"></canvas>
<div class="legend" id="floorLegend"></div>

<h2>6. Data</h2>
<div class="btns" id="dl"></div>
<p class="note">All the numbers behind this page are embedded in this single file; the buttons export CSV (for Excel) from it.</p>

<script id="data" type="application/json">__DATA__</script>
<script>
(function(){
var D=JSON.parse(document.getElementById('data').textContent);
function $(id){return document.getElementById(id);}
var R=D.rates,G=D.gains,N=D.freq.length;
var COLORS=['#ffd54a','#4fc3f7','#ff7a7a','#9ccc65','#ba68c8','#ffb74d','#4db6ac','#f06292','#b0bec5','#dce775'];
function fmtF(hz){return hz>=1e6?(hz/1e6).toFixed(4)+' MHz':(hz/1e3).toFixed(2)+' kHz';}
function rl(r){return (r/1e6).toFixed(3)+' MS/s';}
function gl(g){return g.toFixed(1)+' dB';}
function cmap(t){t=Math.max(0,Math.min(1,t));var s=[[68,1,84],[59,82,139],[33,145,140],[94,201,98],[253,231,37]];var x=t*(s.length-1),i=Math.min(s.length-2,Math.floor(x)),f=x-i;return [0,1,2].map(function(k){return Math.round(s[i][k]+(s[i+1][k]-s[i][k])*f);});}
function el(tag,txt,cls){var e=document.createElement(tag);if(txt!==undefined)e.textContent=txt;if(cls)e.className=cls;return e;}

// ---- header + findings ----
var m=D.meta;
$('meta').textContent=m.band+' | '+R.length+' sample rate(s), '+G.length+' gain step(s), '+m.passes+' pass(es)'+(m.partial?' | PARTIAL RESULT - survey still running or stopped early':'')+' | averaging '+m.n_avg+' | '+m.created;
D.findings.forEach(function(f){var li=el('li',f.text,f.level);$('findings').appendChild(li);});

// ---- colour scale ----
var all=[];D.spec.forEach(function(row){row.forEach(function(v){if(v){for(var i=0;i<v.length;i+=3){if(v[i]!==null)all.push(v[i]);}}});});
all.sort(function(a,b){return a-b;});
var VMIN=all.length?all[Math.floor(all.length*0.02)]:-100,VMAX=all.length?all[Math.floor(all.length*0.9985)]:0;
if(VMAX<=VMIN)VMAX=VMIN+1;

function drawHeat(cv,rows,labels){
 var ctx=cv.getContext('2d'),W=cv.width,H=cv.height,L=92,T=6,B=32,Rm=10,pw=W-L-Rm,ph=H-T-B,nr=rows.length;
 ctx.fillStyle='#14161a';ctx.fillRect(0,0,W,H);
 var img=ctx.createImageData(pw,ph);
 for(var y=0;y<ph;y++){var r=nr-1-Math.min(nr-1,Math.floor(y*nr/ph));var row=rows[r];
  for(var x=0;x<pw;x++){var o=(y*pw+x)*4,v=null;if(row){v=row[Math.min(N-1,Math.floor(x*N/pw))];}
   if(v===null||v===undefined||isNaN(v)){img.data[o]=28;img.data[o+1]=28;img.data[o+2]=32;}
   else{var c=cmap((v-VMIN)/(VMAX-VMIN));img.data[o]=c[0];img.data[o+1]=c[1];img.data[o+2]=c[2];}
   img.data[o+3]=255;}}
 ctx.putImageData(img,L,T);
 ctx.fillStyle='#ccc';ctx.font='11px sans-serif';ctx.textAlign='right';
 var step=Math.max(1,Math.ceil(nr/14));
 for(var r2=0;r2<nr;r2+=step){var yy=T+(nr-1-r2+0.5)*ph/nr;ctx.fillText(labels[r2],L-6,yy+4);}
 ctx.textAlign='center';var f0=D.freq[0],f1=D.freq[N-1],nt=8;
 for(var k=0;k<=nt;k++){var fx=f0+(f1-f0)*k/nt,xx=L+pw*k/nt;ctx.fillText(fx.toFixed(3),xx,H-16);ctx.fillRect(xx,T+ph,1,4);}
 ctx.fillText('Frequency (MHz)   colour scale '+VMIN.toFixed(0)+' (dark) to '+VMAX.toFixed(0)+' dB (yellow)',L+pw/2,H-3);
 cv._g={L:L,T:T,pw:pw,ph:ph,nr:nr};
}
function hover(cv,cb){
 cv.addEventListener('mousemove',function(e){var g=cv._g;if(!g)return;var rc=cv.getBoundingClientRect();
  var x=(e.clientX-rc.left)*cv.width/rc.width-g.L,y=(e.clientY-rc.top)*cv.height/rc.height-g.T;
  if(x<0||y<0||x>=g.pw||y>=g.ph){cb(null,null);return;}
  cb(g.nr-1-Math.min(g.nr-1,Math.floor(y*g.nr/g.ph)),Math.min(N-1,Math.floor(x*N/g.pw)));});
 cv.addEventListener('mouseleave',function(){cb(null,null);});
}
// selectors
var gSel=$('gSel'),rSel=$('rSel');
G.forEach(function(g,i){var o=el('option',gl(g));o.value=i;gSel.appendChild(o);});
R.forEach(function(r,i){var o=el('option',rl(r));o.value=i;rSel.appendChild(o);});
gSel.value=Math.floor(G.length/2);rSel.value=R.length-1;
function drawHeats(){
 var gi=+gSel.value,ri=+rSel.value;
 drawHeat($('heatRate'),R.map(function(r,i){return D.spec[i][gi];}),R.map(rl));
 drawHeat($('heatGain'),G.map(function(g,j){return D.spec[ri][j];}),G.map(gl));
}
gSel.onchange=drawHeats;rSel.onchange=drawHeats;
hover($('heatRate'),function(r,c){var gi=+gSel.value;if(r===null){$('hv1').textContent='';return;}var v=D.spec[r][gi]?D.spec[r][gi][c]:null;$('hv1').textContent=D.freq[c].toFixed(4)+' MHz | '+rl(R[r])+' | '+(v===null||v===undefined?'-':v.toFixed(1)+' dB');});
hover($('heatGain'),function(r,c){var ri=+rSel.value;if(r===null){$('hv2').textContent='';return;}var v=D.spec[ri][r]?D.spec[ri][r][c]:null;$('hv2').textContent=D.freq[c].toFixed(4)+' MHz | '+gl(G[r])+' | '+(v===null||v===undefined?'-':v.toFixed(1)+' dB');});

// ---- generic line plot ----
function linePlot(cv,series,o){
 var ctx=cv.getContext('2d'),W=cv.width,H=cv.height,L=64,T=10,B=36,Rm=14,pw=W-L-Rm,ph=H-T-B;
 var x0=o.x0,x1=o.x1,y0=Infinity,y1=-Infinity;
 series.forEach(function(s){s.pts.forEach(function(p){if(p[0]>=x0&&p[0]<=x1&&p[1]!==null&&isFinite(p[1])){y0=Math.min(y0,p[1]);y1=Math.max(y1,p[1]);}});});
 if(!isFinite(y0)){y0=0;y1=1;}if(o.y0!==undefined)y0=Math.min(y0,o.y0);if(o.y1!==undefined)y1=Math.max(y1,o.y1);
 var pad=(y1-y0)*0.06||1;y0-=pad;y1+=pad;
 ctx.fillStyle='#14161a';ctx.fillRect(0,0,W,H);
 function X(v){return L+(v-x0)/(x1-x0)*pw;}function Y(v){return T+(1-(v-y0)/(y1-y0))*ph;}
 ctx.strokeStyle='#2a2d33';ctx.fillStyle='#aaa';ctx.font='11px sans-serif';
 var i,nt=8;ctx.textAlign='center';
 for(i=0;i<=nt;i++){var xv=x0+(x1-x0)*i/nt,xx=X(xv);ctx.beginPath();ctx.moveTo(xx,T);ctx.lineTo(xx,T+ph);ctx.stroke();ctx.fillText(xv.toFixed(o.xd),xx,H-18);}
 ctx.textAlign='right';
 for(i=0;i<=6;i++){var yv=y0+(y1-y0)*i/6,yy=Y(yv);ctx.beginPath();ctx.moveTo(L,yy);ctx.lineTo(L+pw,yy);ctx.stroke();ctx.fillText(yv.toFixed(1),L-6,yy+4);}
 ctx.textAlign='center';ctx.fillText(o.xl,L+pw/2,H-4);
 ctx.save();ctx.translate(12,T+ph/2);ctx.rotate(-Math.PI/2);ctx.fillText(o.yl,0,0);ctx.restore();
 ctx.save();ctx.beginPath();ctx.rect(L,T,pw,ph);ctx.clip();
 series.forEach(function(s){ctx.strokeStyle=s.color;ctx.fillStyle=s.color;ctx.lineWidth=s.w||1.4;ctx.setLineDash(s.dash||[]);ctx.beginPath();var started=false;
  s.pts.forEach(function(p){if(p[1]===null||!isFinite(p[1])){started=false;return;}var px=X(p[0]),py=Y(p[1]);if(!started){ctx.moveTo(px,py);started=true;}else ctx.lineTo(px,py);});ctx.stroke();ctx.setLineDash([]);
  if(s.dots){s.pts.forEach(function(p){if(p[1]!==null&&isFinite(p[1])){ctx.beginPath();ctx.arc(X(p[0]),Y(p[1]),3,0,6.3);ctx.fill();}});}});
 ctx.restore();
 cv._p={L:L,T:T,pw:pw,ph:ph,x0:x0,x1:x1};
}
function legend(id,items){var d=$(id);d.textContent='';items.forEach(function(it){var s=el('span',it.name);s.style.color=it.color;d.appendChild(s);});}

// ---- spectrum viewer with zoom ----
var tr=[{r:0,g:0,on:true},{r:R.length-1,g:G.length-1,on:true},{r:Math.min(1,R.length-1),g:0,on:false}];
var view={x0:D.freq[0],x1:D.freq[N-1]},drag=null;
function buildTrSel(){var box=$('trSel');box.textContent='';
 tr.forEach(function(t,i){var cb=el('input');cb.type='checkbox';cb.checked=t.on;cb.onchange=function(){t.on=cb.checked;drawSpec();};
  var rs=el('select'),gs=el('select');R.forEach(function(r,k){var o=el('option',rl(r));o.value=k;rs.appendChild(o);});G.forEach(function(g,k){var o=el('option',gl(g));o.value=k;gs.appendChild(o);});
  rs.value=t.r;gs.value=t.g;rs.onchange=function(){t.r=+rs.value;drawSpec();};gs.onchange=function(){t.g=+gs.value;drawSpec();};
  var lab=el('span',' '+String.fromCharCode(65+i)+': ');lab.style.color=COLORS[i];lab.style.marginLeft=i?'14px':'0';
  box.appendChild(cb);box.appendChild(lab);box.appendChild(rs);box.appendChild(document.createTextNode(' '));box.appendChild(gs);});}
function drawSpec(){
 var series=[],items=[];
 tr.forEach(function(t,i){if(!t.on||!D.spec[t.r][t.g])return;var v=D.spec[t.r][t.g];var pts=[];for(var k=0;k<N;k++)pts.push([D.freq[k],v[k]]);
  series.push({pts:pts,color:COLORS[i],w:1.2});items.push({name:String.fromCharCode(65+i)+': '+rl(R[t.r])+' / '+gl(G[t.g]),color:COLORS[i]});});
 linePlot($('specPlot'),series,{x0:view.x0,x1:view.x1,xd:3,xl:'Frequency (MHz)',yl:'Level (dB)'});legend('specLegend',items);
 if(drag){var cv=$('specPlot'),ctx=cv.getContext('2d');ctx.fillStyle='rgba(255,255,255,0.15)';ctx.fillRect(Math.min(drag.a,drag.b),0,Math.abs(drag.b-drag.a),cv.height);}
}
(function(){var cv=$('specPlot');
 function px(e){var rc=cv.getBoundingClientRect();return (e.clientX-rc.left)*cv.width/rc.width;}
 function fx(x){var p=cv._p;return p.x0+(x-p.L)/p.pw*(p.x1-p.x0);}
 cv.addEventListener('mousedown',function(e){drag={a:px(e),b:px(e)};});
 cv.addEventListener('mousemove',function(e){if(drag){drag.b=px(e);drawSpec();}});
 window.addEventListener('mouseup',function(){if(!drag)return;var a=drag.a,b=drag.b;drag=null;if(Math.abs(b-a)>6){var f0=fx(Math.min(a,b)),f1=fx(Math.max(a,b));view={x0:Math.max(D.freq[0],f0),x1:Math.min(D.freq[N-1],f1)};}drawSpec();});
 cv.addEventListener('dblclick',function(){view={x0:D.freq[0],x1:D.freq[N-1]};drawSpec();});})();

// ---- features table + level-vs-gain plot ----
var selFeat=-1;
function tag(txt,cls){return el('span',txt,'tag '+cls);}
function buildFeat(){
 var t=$('featTable');t.textContent='';var hr=el('tr');
 ['#','Where','Sample-rate behaviour','Seen in','Typical level','Gain slope','Gain behaviour'].forEach(function(h){hr.appendChild(el('th',h));});t.appendChild(hr);
 D.features.forEach(function(f,i){var tr_=el('tr',undefined,'row');tr_.onclick=function(){selFeat=i;buildFeat();drawFeat();};if(i===selFeat)tr_.className='row sel';
  tr_.appendChild(el('td',String(i+1)));tr_.appendChild(el('td',f.where));
  var td=el('td');td.appendChild(tag(f.rate_tag,f.rate_cls));td.appendChild(document.createTextNode(f.rate_text));tr_.appendChild(td);
  tr_.appendChild(el('td',f.seen));tr_.appendChild(el('td',f.level_text));tr_.appendChild(el('td',f.slope===null?'-':f.slope.toFixed(2)));
  var tg=el('td');f.gain_tags.forEach(function(g){tg.appendChild(tag(g[0],g[1]));});tg.appendChild(document.createTextNode(f.gain_text));tr_.appendChild(tg);t.appendChild(tr_);});
}
function drawFeat(){
 var cv=$('featPlot');
 if(selFeat<0||!D.features[selFeat]){var ctx=cv.getContext('2d');ctx.fillStyle='#14161a';ctx.fillRect(0,0,cv.width,cv.height);ctx.fillStyle='#888';ctx.font='14px sans-serif';ctx.textAlign='center';ctx.fillText('Select a feature in the table to plot its level against gain',cv.width/2,cv.height/2);$('featLegend').textContent='';return;}
 var f=D.features[selFeat],series=[],items=[],ref=null;
 R.forEach(function(r,a){var pts=G.map(function(g,b){return [g,f.levels[a][b]];});series.push({pts:pts,color:COLORS[a%COLORS.length],dots:true});items.push({name:rl(r),color:COLORS[a%COLORS.length]});
  if(!ref){for(var b=0;b<G.length;b++){if(f.levels[a][b]!==null){ref=[G[b],f.levels[a][b]];break;}}}});
 if(ref){series.push({pts:[[G[0],ref[1]+(G[0]-ref[0])],[G[G.length-1],ref[1]+(G[G.length-1]-ref[0])]],color:'#999',dash:[6,5],w:1});items.push({name:'reference: 1 dB per dB of gain',color:'#999'});}
 linePlot(cv,series,{x0:G[0],x1:G[G.length-1]>G[0]?G[G.length-1]:G[0]+1,xd:1,xl:'RF gain (dB)',yl:'Level of this feature (dB)'});legend('featLegend',items);
}

// ---- floor / headroom ----
function drawFloor(){
 var cv=$('floorPlot'),ctx=cv.getContext('2d');ctx.fillStyle='#14161a';ctx.fillRect(0,0,cv.width,cv.height);
 var half=document.createElement('canvas');half.width=cv.width/2;half.height=cv.height;
 function pane(vals,yl,fs,off){var s=[],items=[];R.forEach(function(r,a){s.push({pts:G.map(function(g,b){return [g,vals[a][b]];}),color:COLORS[a%COLORS.length],dots:true});});
  if(fs!==null)s.push({pts:[[G[0],fs],[G[G.length-1]>G[0]?G[G.length-1]:G[0]+1,fs]],color:'#e05050',dash:[6,5],w:1.2});
  linePlot(half,s,{x0:G[0],x1:G[G.length-1]>G[0]?G[G.length-1]:G[0]+1,xd:0,xl:'RF gain (dB)',yl:yl,y1:fs===null?undefined:fs});ctx.drawImage(half,off,0);}
 pane(D.floor,'Median noise floor (dB)',null,0);pane(D.peak,'Strongest component (dB)',D.fs_db,cv.width/2);
 var items=R.map(function(r,a){return {name:rl(r),color:COLORS[a%COLORS.length]};});items.push({name:'ADC full-scale equivalent',color:'#e05050'});legend('floorLegend',items);
}

// ---- downloads ----
function dl(name,text){var a=document.createElement('a');a.href=URL.createObjectURL(new Blob([text],{type:'text/csv'}));a.download=name;document.body.appendChild(a);a.click();document.body.removeChild(a);}
function csvq(v){v=String(v);return /[",\n]/.test(v)?'"'+v.replace(/"/g,'""')+'"':v;}
function addBtn(txt,fn){var b=el('button',txt);b.onclick=fn;$('dl').appendChild(b);}
addBtn('Surface chart CSV for the selected sample rate (Excel: Insert > Surface)',function(){var ri=+rSel.value,rows=[['']];
 G.forEach(function(g){rows[0].push(g.toFixed(1)+' dB');});
 for(var k=0;k<N;k++){var row=[D.freq[k]];for(var b=0;b<G.length;b++){var v=D.spec[ri][b]?D.spec[ri][b][k]:null;row.push(v===null||v===undefined?'':v);}rows.push(row);}
 dl('surface_'+(R[ri]/1e6).toFixed(3)+'MSps.csv',rows.map(function(r){return r.map(csvq).join(',');}).join('\n'));});
addBtn('Every pass (rate, gain, noise floor, peak, headroom, spikes) CSV',function(){var rows=[['sample_rate_hz','gain_db','repeat','noise_floor_db','strongest_db','headroom_to_full_scale_db','spikes','pass_seconds']];
 D.table.forEach(function(r){rows.push(r);});dl('survey_passes.csv',rows.map(function(r){return r.map(csvq).join(',');}).join('\n'));});
addBtn('Every spike found CSV',function(){var rows=[['sample_rate_hz','gain_db','freq_hz','level_db','prominence_db']];
 D.spikes.forEach(function(s){rows.push([R[s[0]],G[s[1]],s[2],s[3],s[4]]);});dl('survey_spikes.csv',rows.map(function(r){return r.map(csvq).join(',');}).join('\n'));});
addBtn('Features table CSV',function(){var rows=[['where','sample_rate_behaviour','seen_in','typical_level','gain_slope','gain_behaviour']];
 D.features.forEach(function(f){rows.push([f.where,f.rate_tag+' '+f.rate_text,f.seen,f.level_text,f.slope===null?'':f.slope,f.gain_text]);});dl('survey_features.csv',rows.map(function(r){return r.map(csvq).join(',');}).join('\n'));});

buildTrSel();buildFeat();drawHeats();drawSpec();drawFeat();drawFloor();
})();
</script></body></html>
'''
_SURVEY_REPORT_TEMPLATE_V2 = r'''<!DOCTYPE html>
<html lang="en"><head><meta charset="utf-8"><title>RF gain / sample-rate survey</title>
<style>
body{background:#16181c;color:#d8dbe0;font:14px/1.45 Segoe UI,Arial,sans-serif;margin:0 auto;max-width:1180px;padding:16px 20px 60px}
h1{font-size:22px;margin:4px 0 2px}h2{font-size:17px;margin:30px 0 6px;border-bottom:1px solid #333;padding-bottom:3px}h3{font-size:15px;margin:18px 0 4px}
.sub{color:#9aa0a8}.note{color:#9aa0a8;font-size:13px;margin:2px 0 8px}
canvas{display:block;width:100%;max-width:1100px;background:#14161a;border:1px solid #2a2d33}
.ctl{margin:6px 0}select,button{background:#262a31;color:#d8dbe0;border:1px solid #444;border-radius:4px;padding:3px 8px;font:inherit}
button{cursor:pointer}button:hover{background:#323843}
.find{margin:0;padding:0;list-style:none}.find li{padding:7px 10px;margin:4px 0;border-left:4px solid #4a90d9;background:#1d2026}
.find li.warn{border-left-color:#e0a030}.find li.bad{border-left-color:#e05050}.find li.ok{border-left-color:#5aa469}
table{border-collapse:collapse;width:100%;font-size:13px}th,td{padding:4px 7px;border-bottom:1px solid #2a2d33;text-align:left;vertical-align:top}
th{background:#1d2026}tr.sel{background:#2b3442}tr.row:hover{background:#242932;cursor:pointer}
.tag{display:inline-block;padding:0 7px;border-radius:9px;font-size:12px;margin-right:3px;color:#fff}
.v-reliable{background:#2f6d3f}.v-rate{background:#a03a3a}.v-gain{background:#b8741c}.v-internal{background:#7a3a8a}.v-unsure{background:#555}
.hv{color:#ffd54a;margin-left:10px;font-family:Consolas,monospace;font-size:13px}
.scroll{max-height:420px;overflow:auto;border:1px solid #2a2d33}
.btns button{margin:3px 6px 3px 0}.legend span{margin-right:14px;font-size:13px}
td.sc{text-align:center;cursor:pointer;min-width:52px;color:#111;font-weight:600}
.box{border:1px solid #2a2d33;padding:2px 8px 8px;margin:8px 0;background:#1a1c21}
</style></head><body>
<h1>RF scan - gain / sample-rate survey</h1>
<div class="sub" id="meta"></div>

<h2>Summary</h2>
<ul class="find" id="summary"></ul>

<h2>1. Reliably sensed signals (likely from the device under test)</h2>
<p class="note">Same frequency at (nearly) every sample rate, and the level rises in step with the RF gain (about 1 dB per dB). "Linear up to" is the highest gain at which the signal still follows the gain.</p>
<div id="tblReliable"></div>

<h2>2. Artefacts to ignore</h2>
<h3>Caused by the sample rate / tuning (aliases, images, LO leakage, fixed-offset spurs)</h3>
<div id="tblRate"></div>
<h3>Caused by the gain stages (non-linear products: harmonics, intermodulation)</h3>
<div id="tblGain"></div>
<h3>Generated inside the receiver (stable, but the level does not follow the gain)</h3>
<div id="tblInternal"></div>
<h3>Unconfirmed (not enough evidence either way)</h3>
<div id="tblUnsure"></div>

<h2>3. Maximum usable gain</h2>
<p class="note">Left: strongest DUT signal, strongest gain-caused artefact and noise floor against RF gain (median over sample rates). Real signals rise 1 dB per dB; distortion products rise 2-3 dB per dB, so they close in on the signals as the gain goes up. Red dashed: maximum usable gain found. Blue dotted: the gain you estimated visually.</p>
<div id="gainTable"></div>
<canvas id="gainPlot1" width="1100" height="320" style="margin-top:8px"></canvas>
<div class="legend" id="gainLegend1"></div>
<canvas id="gainPlot2" width="1100" height="260" style="margin-top:8px"></canvas>
<div class="legend" id="gainLegend2"></div>

<h2>4. Best spectrum setting</h2>
<p class="note">Every sample rate / gain is scored: more of the reliable signals visible and clearly above the noise is better; every artefact visible, a weak margin between the DUT signals and the strongest artefact, being near ADC full scale, or exceeding the maximum usable gain all count against it. Click a cell to show that setting in the spectrum viewer.</p>
<div id="bestTable"></div>
<div class="ctl"><button id="btnBest">Show the best setting (A) against the highest gain at the same rate (B)</button></div>
<div id="scoreTable" class="scroll"></div>

<h2>5. Sample-rate effects (map)</h2>
<p class="note">Each row is one sample rate (lowest at the bottom), colour = level at the chosen gain. A real signal is a vertical line in every row; anything that shifts, appears or vanishes between rows depends on the sample rate.</p>
<div class="ctl">Gain: <select id="gSel"></select><span class="hv" id="hv1"></span></div>
<canvas id="heatRate" width="1100" height="320"></canvas>

<h2>6. Gain effects (map)</h2>
<p class="note">Each row is one RF gain (lowest at the bottom) at the chosen sample rate. Something that appears only at higher gain, or brightens faster than its neighbours, is made inside the gain stages.</p>
<div class="ctl">Sample rate: <select id="rSel"></select><span class="hv" id="hv2"></span></div>
<canvas id="heatGain" width="1100" height="320"></canvas>

<h2>7. Spectrum viewer</h2>
<p class="note">Overlay up to three sample-rate / gain combinations. Drag on the plot to zoom, double-click to reset. Triangles mark the classified features (for the rate of trace A): <span style="color:#4cc46a">green = reliable</span>, <span style="color:#e05a5a">red = sample-rate artefact</span>, <span style="color:#f0a030">orange = gain artefact</span>, <span style="color:#c070e0">purple = receiver-internal</span>.</p>
<div class="ctl" id="trSel"></div>
<div class="ctl"><label><input type="checkbox" id="marks" checked> mark classified features</label></div>
<canvas id="specPlot" width="1100" height="360"></canvas>
<div class="legend" id="specLegend"></div>

<h2>8. All features</h2>
<div class="ctl">Show: <select id="vSel"><option value="">all</option><option value="reliable">reliable</option><option value="rate">sample-rate artefacts</option><option value="gain">gain artefacts</option><option value="internal">receiver-internal</option><option value="unsure">unconfirmed</option></select></div>
<p class="note">Click a row to plot the feature's level against gain (dashed line: the 1 dB per dB reference).</p>
<div class="scroll"><table id="featTable"></table></div>
<canvas id="featPlot" width="1100" height="300" style="margin-top:8px"></canvas>
<div class="legend" id="featLegend"></div>

<h2>9. Noise floor and ADC headroom</h2>
<canvas id="floorPlot" width="1100" height="300"></canvas>
<div class="legend" id="floorLegend"></div>

<h2>10. Data</h2>
<div class="btns" id="dl"></div>
<p class="note">All the numbers behind this page are embedded in this single file; the buttons export CSV (for Excel) from it.</p>

<script id="data" type="application/json">__DATA__</script>
<script>
(function(){
var D=JSON.parse(document.getElementById('data').textContent);
function $(id){return document.getElementById(id);}
var R=D.rates,G=D.gains,N=D.freq.length,F=D.features,GA=D.gain;
var COLORS=['#ffd54a','#4fc3f7','#ff7a7a','#9ccc65','#ba68c8','#ffb74d','#4db6ac','#f06292','#b0bec5','#dce775'];
var VCOL={reliable:'#4cc46a',rate:'#e05a5a',gain:'#f0a030',internal:'#c070e0',unsure:'#999'};
var VNAME={reliable:'reliable',rate:'sample-rate artefact',gain:'gain artefact',internal:'receiver-internal',unsure:'unconfirmed'};
function rl(r){return (r/1e6).toFixed(3)+' MS/s';}
function gl(g){return g.toFixed(1)+' dB';}
function cmap(t){t=Math.max(0,Math.min(1,t));var s=[[68,1,84],[59,82,139],[33,145,140],[94,201,98],[253,231,37]];var x=t*(s.length-1),i=Math.min(s.length-2,Math.floor(x)),f=x-i;return [0,1,2].map(function(k){return Math.round(s[i][k]+(s[i+1][k]-s[i][k])*f);});}
function el(tag,txt,cls){var e=document.createElement(tag);if(txt!==undefined)e.textContent=txt;if(cls)e.className=cls;return e;}
function mkTable(id,head,rows){var c=$(id);c.textContent='';if(!rows.length){c.appendChild(el('p','None found.','note'));return;}
 var t=el('table'),hr=el('tr');head.forEach(function(h){hr.appendChild(el('th',h));});t.appendChild(hr);
 rows.forEach(function(r){var tr=el('tr');r.forEach(function(v){tr.appendChild(el('td',String(v)));});t.appendChild(tr);});
 var w=el('div');w.className='scroll';w.appendChild(t);c.appendChild(w);}

// ---- header + summary ----
var m=D.meta;
$('meta').textContent=m.band+' | '+R.length+' sample rate(s), '+G.length+' gain step(s), '+m.passes+' pass(es)'+(m.partial?' | PARTIAL RESULT - survey still running or stopped early':'')+' | averaging '+m.n_avg+' | '+m.created;
D.summary.forEach(function(f){$('summary').appendChild(el('li',f.text,f.level));});

// ---- verdict tables ----
function by(v){return F.filter(function(f){return f.verdict===v;});}
mkTable('tblReliable',['Frequency','Seen at','Gain slope','Level (typical)','Linear up to','Evidence'],
 by('reliable').map(function(f){return [f.where,f.rate_text,f.slope===null?'-':f.slope.toFixed(2),f.level_text,f.lin_to===null?'-':gl(f.lin_to),f.cause];}));
mkTable('tblRate',['Where','What it is','Seen at','Gain slope'],by('rate').map(function(f){return [f.where,f.cause,f.rate_text,f.slope===null?'-':f.slope.toFixed(2)];}));
mkTable('tblGain',['Where','What it is','First visible at','Gain slope'],by('gain').map(function(f){return [f.where,f.cause,f.first_vis===null?'-':gl(f.first_vis),f.slope===null?'-':f.slope.toFixed(2)];}));
mkTable('tblInternal',['Where','What it is','Level (typical)'],by('internal').map(function(f){return [f.where,f.cause,f.level_text];}));
mkTable('tblUnsure',['Where','Why unconfirmed','Level (typical)'],by('unsure').map(function(f){return [f.where,f.cause,f.level_text];}));

// ---- colour scale for the maps ----
var all=[];D.spec.forEach(function(row){row.forEach(function(v){if(v){for(var i=0;i<v.length;i+=3){if(v[i]!==null)all.push(v[i]);}}});});
all.sort(function(a,b){return a-b;});
var VMIN=all.length?all[Math.floor(all.length*0.02)]:-100,VMAX=all.length?all[Math.floor(all.length*0.9985)]:0;
if(VMAX<=VMIN)VMAX=VMIN+1;
function drawHeat(cv,rows,labels){
 var ctx=cv.getContext('2d'),W=cv.width,H=cv.height,L=92,T=6,B=32,Rm=10,pw=W-L-Rm,ph=H-T-B,nr=rows.length;
 ctx.fillStyle='#14161a';ctx.fillRect(0,0,W,H);
 var img=ctx.createImageData(pw,ph);
 for(var y=0;y<ph;y++){var r=nr-1-Math.min(nr-1,Math.floor(y*nr/ph));var row=rows[r];
  for(var x=0;x<pw;x++){var o=(y*pw+x)*4,v=null;if(row){v=row[Math.min(N-1,Math.floor(x*N/pw))];}
   if(v===null||v===undefined||isNaN(v)){img.data[o]=28;img.data[o+1]=28;img.data[o+2]=32;}
   else{var c=cmap((v-VMIN)/(VMAX-VMIN));img.data[o]=c[0];img.data[o+1]=c[1];img.data[o+2]=c[2];}
   img.data[o+3]=255;}}
 ctx.putImageData(img,L,T);
 ctx.fillStyle='#ccc';ctx.font='11px sans-serif';ctx.textAlign='right';
 var step=Math.max(1,Math.ceil(nr/14));
 for(var r2=0;r2<nr;r2+=step){var yy=T+(nr-1-r2+0.5)*ph/nr;ctx.fillText(labels[r2],L-6,yy+4);}
 ctx.textAlign='center';var f0=D.freq[0],f1=D.freq[N-1],nt=8;
 for(var k=0;k<=nt;k++){var fx=f0+(f1-f0)*k/nt,xx=L+pw*k/nt;ctx.fillText(fx.toFixed(3),xx,H-16);ctx.fillRect(xx,T+ph,1,4);}
 ctx.fillText('Frequency (MHz)   colour scale '+VMIN.toFixed(0)+' (dark) to '+VMAX.toFixed(0)+' dB (yellow)',L+pw/2,H-3);
 cv._g={L:L,T:T,pw:pw,ph:ph,nr:nr};
}
function hover(cv,cb){
 cv.addEventListener('mousemove',function(e){var g=cv._g;if(!g)return;var rc=cv.getBoundingClientRect();
  var x=(e.clientX-rc.left)*cv.width/rc.width-g.L,y=(e.clientY-rc.top)*cv.height/rc.height-g.T;
  if(x<0||y<0||x>=g.pw||y>=g.ph){cb(null,null);return;}
  cb(g.nr-1-Math.min(g.nr-1,Math.floor(y*g.nr/g.ph)),Math.min(N-1,Math.floor(x*N/g.pw)));});
 cv.addEventListener('mouseleave',function(){cb(null,null);});
}
var gSel=$('gSel'),rSel=$('rSel');
G.forEach(function(g,i){var o=el('option',gl(g));o.value=i;gSel.appendChild(o);});
R.forEach(function(r,i){var o=el('option',rl(r));o.value=i;rSel.appendChild(o);});
gSel.value=Math.floor(G.length/2);rSel.value=R.length-1;
function drawHeats(){var gi=+gSel.value,ri=+rSel.value;
 drawHeat($('heatRate'),R.map(function(r,i){return D.spec[i][gi];}),R.map(rl));
 drawHeat($('heatGain'),G.map(function(g,j){return D.spec[ri][j];}),G.map(gl));}
gSel.onchange=drawHeats;rSel.onchange=drawHeats;
hover($('heatRate'),function(r,c){var gi=+gSel.value;if(r===null){$('hv1').textContent='';return;}var v=D.spec[r][gi]?D.spec[r][gi][c]:null;$('hv1').textContent=D.freq[c].toFixed(4)+' MHz | '+rl(R[r])+' | '+(v===null||v===undefined?'-':v.toFixed(1)+' dB');});
hover($('heatGain'),function(r,c){var ri=+rSel.value;if(r===null){$('hv2').textContent='';return;}var v=D.spec[ri][r]?D.spec[ri][r][c]:null;$('hv2').textContent=D.freq[c].toFixed(4)+' MHz | '+gl(G[r])+' | '+(v===null||v===undefined?'-':v.toFixed(1)+' dB');});

// ---- generic line plot (with optional vertical reference lines) ----
function linePlot(cv,series,o){
 var ctx=cv.getContext('2d'),W=cv.width,H=cv.height,L=64,T=14,B=36,Rm=14,pw=W-L-Rm,ph=H-T-B;
 var x0=o.x0,x1=o.x1,y0=Infinity,y1=-Infinity;
 series.forEach(function(s){s.pts.forEach(function(p){if(p[0]>=x0&&p[0]<=x1&&p[1]!==null&&isFinite(p[1])){y0=Math.min(y0,p[1]);y1=Math.max(y1,p[1]);}});});
 if(!isFinite(y0)){y0=0;y1=1;}if(o.y0!==undefined)y0=Math.min(y0,o.y0);if(o.y1!==undefined)y1=Math.max(y1,o.y1);
 var pad=(y1-y0)*0.06||1;y0-=pad;y1+=pad;
 ctx.fillStyle='#14161a';ctx.fillRect(0,0,W,H);
 function X(v){return L+(v-x0)/(x1-x0)*pw;}function Y(v){return T+(1-(v-y0)/(y1-y0))*ph;}
 ctx.strokeStyle='#2a2d33';ctx.fillStyle='#aaa';ctx.font='11px sans-serif';
 var i,nt=8;ctx.textAlign='center';
 for(i=0;i<=nt;i++){var xv=x0+(x1-x0)*i/nt,xx=X(xv);ctx.beginPath();ctx.moveTo(xx,T);ctx.lineTo(xx,T+ph);ctx.stroke();ctx.fillText(xv.toFixed(o.xd),xx,H-18);}
 ctx.textAlign='right';
 for(i=0;i<=6;i++){var yv=y0+(y1-y0)*i/6,yy=Y(yv);ctx.beginPath();ctx.moveTo(L,yy);ctx.lineTo(L+pw,yy);ctx.stroke();ctx.fillText(yv.toFixed(1),L-6,yy+4);}
 ctx.textAlign='center';ctx.fillText(o.xl,L+pw/2,H-4);
 ctx.save();ctx.translate(12,T+ph/2);ctx.rotate(-Math.PI/2);ctx.fillText(o.yl,0,0);ctx.restore();
 ctx.save();ctx.beginPath();ctx.rect(L,T,pw,ph);ctx.clip();
 series.forEach(function(s){ctx.strokeStyle=s.color;ctx.fillStyle=s.color;ctx.lineWidth=s.w||1.4;ctx.setLineDash(s.dash||[]);ctx.beginPath();var started=false;
  s.pts.forEach(function(p){if(p[1]===null||!isFinite(p[1])){started=false;return;}var px=X(p[0]),py=Y(p[1]);if(!started){ctx.moveTo(px,py);started=true;}else ctx.lineTo(px,py);});ctx.stroke();ctx.setLineDash([]);
  if(s.dots){s.pts.forEach(function(p){if(p[1]!==null&&isFinite(p[1])){ctx.beginPath();ctx.arc(X(p[0]),Y(p[1]),3,0,6.3);ctx.fill();}});}});
 (o.vlines||[]).forEach(function(v){if(v.x<x0||v.x>x1)return;ctx.strokeStyle=v.color;ctx.lineWidth=1.4;ctx.setLineDash(v.dash||[6,5]);ctx.beginPath();ctx.moveTo(X(v.x),T);ctx.lineTo(X(v.x),T+ph);ctx.stroke();ctx.setLineDash([]);});
 ctx.restore();
 (o.vlines||[]).forEach(function(v){if(v.x<x0||v.x>x1||!v.label)return;ctx.fillStyle=v.color;ctx.textAlign='left';ctx.fillText(v.label,X(v.x)+4,T+11+(v.row||0)*12);});
 cv._p={L:L,T:T,pw:pw,ph:ph,x0:x0,x1:x1};
}
function legend(id,items){var d=$(id);d.textContent='';items.forEach(function(it){var s=el('span',it.name);s.style.color=it.color;d.appendChild(s);});}
var GX1=G[G.length-1]>G[0]?G[G.length-1]:G[0]+1;

// ---- maximum usable gain ----
mkTable('gainTable',['Sample rate','Maximum usable gain','Limited by','Lowest useful gain (DUT signals 10 dB above noise)'],
 GA.per_rate.map(function(p){return [rl(p.rate),gl(p.gmax),p.reason,p.gmin===null?'-':gl(p.gmin)];}));
(function(){
 var vl=[{x:GA.gmax,color:'#e05050',label:'max usable '+GA.gmax.toFixed(1)+' dB',row:0},{x:GA.hypothesis,color:'#4f9fe0',dash:[2,4],label:'your estimate '+GA.hypothesis+' dB',row:1}];
 var s=[{pts:G.map(function(g,i){return [g,GA.dut_level[i]];}),color:'#4cc46a',dots:true},
        {pts:G.map(function(g,i){return [g,GA.gart_level[i]];}),color:'#f0a030',dots:true},
        {pts:G.map(function(g,i){return [g,GA.floor[i]];}),color:'#9aa0a8',dots:true}];
 linePlot($('gainPlot1'),s,{x0:G[0],x1:GX1,xd:1,xl:'RF gain (dB)',yl:'Level (dB)',vlines:vl});
 legend('gainLegend1',[{name:'strongest DUT (reliable) signal',color:'#4cc46a'},{name:'strongest gain-caused artefact',color:'#f0a030'},{name:'noise floor (median)',color:'#9aa0a8'}]);
 var cv=$('gainPlot2'),ctx=cv.getContext('2d');ctx.fillStyle='#14161a';ctx.fillRect(0,0,cv.width,cv.height);
 var half=document.createElement('canvas');half.width=cv.width/2;half.height=cv.height;
 linePlot(half,[{pts:G.map(function(g,i){return [g,GA.dut_snr[i]];}),color:'#4cc46a',dots:true}],{x0:G[0],x1:GX1,xd:0,xl:'RF gain (dB)',yl:'DUT signal above noise (dB)',vlines:[vl[0]]});ctx.drawImage(half,0,0);
 linePlot(half,[{pts:G.map(function(g,i){return [g,GA.n_gart[i]];}),color:'#f0a030',dots:true},{pts:G.map(function(g,i){return [g,GA.n_art[i]];}),color:'#e05a5a',dots:true}],{x0:G[0],x1:GX1,xd:0,xl:'RF gain (dB)',yl:'Artefacts visible (average per sample rate)',y0:0,vlines:[vl[0]]});ctx.drawImage(half,cv.width/2,0);
 legend('gainLegend2',[{name:'DUT signal SNR',color:'#4cc46a'},{name:'gain-caused artefacts visible',color:'#f0a030'},{name:'all artefacts visible',color:'#e05a5a'}]);
})();

// ---- best spectrum ----
var tr=[{r:0,g:0,on:true},{r:R.length-1,g:G.length-1,on:true},{r:Math.min(1,R.length-1),g:0,on:false}];
if(D.best.length){tr[0]={r:D.best[0].rate_i,g:D.best[0].gain_i,on:true};tr[1]={r:D.best[0].rate_i,g:G.length-1,on:true};tr[2]={r:D.best[0].rate_i,g:0,on:false};}
mkTable('bestTable',['Rank','Sample rate','Gain','DUT signals visible','Median SNR of DUT signals','Artefacts visible','Strongest artefact below DUT signal','Below ADC full scale','Score','Notes'],
 D.best.map(function(b,i){return [i+1,rl(b.rate),gl(b.gain),(D.meta.n_reliable?Math.round(b.coverage*D.meta.n_reliable)+' of '+D.meta.n_reliable:'-'),b.snr===null?'-':b.snr.toFixed(0)+' dB',b.n_art,b.margin===null?'-':b.margin.toFixed(0)+' dB',b.headroom===null?'-':b.headroom.toFixed(0)+' dB',b.score.toFixed(0),b.notes.join(', ')];}));
(function(){var c=$('scoreTable');c.textContent='';var t=el('table'),hr=el('tr');hr.appendChild(el('th','rate \\ gain'));G.forEach(function(g){hr.appendChild(el('th',gl(g)));});t.appendChild(hr);
 var vals=[];D.scores.forEach(function(r){r.forEach(function(v){if(v!==null)vals.push(v);});});var lo=Math.min.apply(null,vals),hi=Math.max.apply(null,vals);if(!(hi>lo))hi=lo+1;
 var bi=D.best.length?D.best[0]:null;
 R.forEach(function(r,a){var row=el('tr');row.appendChild(el('td',rl(r)));
  G.forEach(function(g,b){var v=D.scores[a][b],td=el('td',v===null?'-':v.toFixed(0),'sc');if(v!==null){var cc=cmap((v-lo)/(hi-lo));td.style.background='rgb('+cc[0]+','+cc[1]+','+cc[2]+')';}
   if(bi&&bi.rate_i===a&&bi.gain_i===b){td.style.outline='3px solid #fff';td.title='best setting';}
   td.onclick=function(){tr[0]={r:a,g:b,on:true};buildTrSel();drawSpec();window.scrollTo(0,$('specPlot').offsetTop-80);};row.appendChild(td);});t.appendChild(row);});
 c.appendChild(t);})();
$('btnBest').onclick=function(){if(!D.best.length)return;var b=D.best[0];tr[0]={r:b.rate_i,g:b.gain_i,on:true};tr[1]={r:b.rate_i,g:G.length-1,on:true};tr[2].on=false;buildTrSel();drawSpec();window.scrollTo(0,$('specPlot').offsetTop-80);};

// ---- spectrum viewer with zoom and feature marks ----
var view={x0:D.freq[0],x1:D.freq[N-1]},drag=null;
function buildTrSel(){var box=$('trSel');box.textContent='';
 tr.forEach(function(t,i){var cb=el('input');cb.type='checkbox';cb.checked=t.on;cb.onchange=function(){t.on=cb.checked;drawSpec();};
  var rs=el('select'),gs=el('select');R.forEach(function(r,k){var o=el('option',rl(r));o.value=k;rs.appendChild(o);});G.forEach(function(g,k){var o=el('option',gl(g));o.value=k;gs.appendChild(o);});
  rs.value=t.r;gs.value=t.g;rs.onchange=function(){t.r=+rs.value;drawSpec();};gs.onchange=function(){t.g=+gs.value;drawSpec();};
  var lab=el('span',' '+String.fromCharCode(65+i)+': ');lab.style.color=COLORS[i];lab.style.marginLeft=i?'14px':'0';
  box.appendChild(cb);box.appendChild(lab);box.appendChild(rs);box.appendChild(document.createTextNode(' '));box.appendChild(gs);});}
function featPositions(f,ri){var out=[];if(f.kind==='abs')out=[f.pos];else if(f.kind==='fixed')out=D.centres[ri].map(function(c){return c+f.pos/1e6;});else out=D.centres[ri].map(function(c){return c+f.pos*R[ri]/1e6;});return out;}
function drawMarks(){
 if(!$('marks').checked)return;var cv=$('specPlot'),ctx=cv.getContext('2d'),p=cv._p;if(!p)return;
 var ti=tr.filter(function(t){return t.on;})[0];if(!ti)return;
 function X(v){return p.L+(v-p.x0)/(p.x1-p.x0)*p.pw;}
 F.forEach(function(f){var col=VCOL[f.verdict];ctx.fillStyle=col;
  featPositions(f,ti.r).forEach(function(fx){if(fx<p.x0||fx>p.x1)return;var x=X(fx),y=p.T+2;ctx.beginPath();ctx.moveTo(x-5,y);ctx.lineTo(x+5,y);ctx.lineTo(x,y+9);ctx.closePath();ctx.fill();});});
}
function drawSpec(){
 var series=[],items=[];
 tr.forEach(function(t,i){if(!t.on||!D.spec[t.r][t.g])return;var v=D.spec[t.r][t.g];var pts=[];for(var k=0;k<N;k++)pts.push([D.freq[k],v[k]]);
  series.push({pts:pts,color:COLORS[i],w:1.2});items.push({name:String.fromCharCode(65+i)+': '+rl(R[t.r])+' / '+gl(G[t.g]),color:COLORS[i]});});
 linePlot($('specPlot'),series,{x0:view.x0,x1:view.x1,xd:3,xl:'Frequency (MHz)',yl:'Level (dB)'});legend('specLegend',items);drawMarks();
 if(drag){var cv=$('specPlot'),ctx=cv.getContext('2d');ctx.fillStyle='rgba(255,255,255,0.15)';ctx.fillRect(Math.min(drag.a,drag.b),0,Math.abs(drag.b-drag.a),cv.height);}
}
$('marks').onchange=drawSpec;
(function(){var cv=$('specPlot');
 function px(e){var rc=cv.getBoundingClientRect();return (e.clientX-rc.left)*cv.width/rc.width;}
 function fx(x){var p=cv._p;return p.x0+(x-p.L)/p.pw*(p.x1-p.x0);}
 cv.addEventListener('mousedown',function(e){drag={a:px(e),b:px(e)};});
 cv.addEventListener('mousemove',function(e){if(drag){drag.b=px(e);drawSpec();}});
 window.addEventListener('mouseup',function(){if(!drag)return;var a=drag.a,b=drag.b;drag=null;if(Math.abs(b-a)>6){var f0=fx(Math.min(a,b)),f1=fx(Math.max(a,b));view={x0:Math.max(D.freq[0],f0),x1:Math.min(D.freq[N-1],f1)};}drawSpec();});
 cv.addEventListener('dblclick',function(){view={x0:D.freq[0],x1:D.freq[N-1]};drawSpec();});})();

// ---- all features + level-vs-gain plot ----
var selFeat=-1;
function buildFeat(){
 var t=$('featTable');t.textContent='';var hr=el('tr');
 ['#','Verdict','Where','What it is','Seen at','Typical level','Gain slope'].forEach(function(h){hr.appendChild(el('th',h));});t.appendChild(hr);
 var filt=$('vSel').value;
 F.forEach(function(f,i){if(filt&&f.verdict!==filt)return;var tr_=el('tr',undefined,'row');tr_.onclick=function(){selFeat=i;buildFeat();drawFeat();};if(i===selFeat)tr_.className='row sel';
  tr_.appendChild(el('td',String(i+1)));var tv=el('td');tv.appendChild(el('span',VNAME[f.verdict],'tag v-'+f.verdict));tr_.appendChild(tv);
  tr_.appendChild(el('td',f.where));tr_.appendChild(el('td',f.cause));tr_.appendChild(el('td',f.rate_text));tr_.appendChild(el('td',f.level_text));tr_.appendChild(el('td',f.slope===null?'-':f.slope.toFixed(2)));t.appendChild(tr_);});
}
$('vSel').onchange=buildFeat;
function drawFeat(){
 var cv=$('featPlot');
 if(selFeat<0||!F[selFeat]){var ctx=cv.getContext('2d');ctx.fillStyle='#14161a';ctx.fillRect(0,0,cv.width,cv.height);ctx.fillStyle='#888';ctx.font='14px sans-serif';ctx.textAlign='center';ctx.fillText('Select a feature in the table to plot its level against gain',cv.width/2,cv.height/2);$('featLegend').textContent='';return;}
 var f=F[selFeat],series=[],items=[],ref=null;
 R.forEach(function(r,a){var pts=G.map(function(g,b){return [g,f.levels[a][b]];});series.push({pts:pts,color:COLORS[a%COLORS.length],dots:true});items.push({name:rl(r),color:COLORS[a%COLORS.length]});
  if(!ref){for(var b=0;b<G.length;b++){if(f.levels[a][b]!==null){ref=[G[b],f.levels[a][b]];break;}}}});
 if(ref){series.push({pts:[[G[0],ref[1]+(G[0]-ref[0])],[G[G.length-1],ref[1]+(G[G.length-1]-ref[0])]],color:'#999',dash:[6,5],w:1});items.push({name:'reference: 1 dB per dB of gain',color:'#999'});}
 linePlot(cv,series,{x0:G[0],x1:GX1,xd:1,xl:'RF gain (dB)',yl:'Level of this feature (dB)'});legend('featLegend',items);
}

// ---- floor / headroom ----
function drawFloor(){
 var cv=$('floorPlot'),ctx=cv.getContext('2d');ctx.fillStyle='#14161a';ctx.fillRect(0,0,cv.width,cv.height);
 var half=document.createElement('canvas');half.width=cv.width/2;half.height=cv.height;
 function pane(vals,yl,fs,off){var s=[];R.forEach(function(r,a){s.push({pts:G.map(function(g,b){return [g,vals[a][b]];}),color:COLORS[a%COLORS.length],dots:true});});
  if(fs!==null)s.push({pts:[[G[0],fs],[GX1,fs]],color:'#e05050',dash:[6,5],w:1.2});
  linePlot(half,s,{x0:G[0],x1:GX1,xd:0,xl:'RF gain (dB)',yl:yl,y1:fs===null?undefined:fs});ctx.drawImage(half,off,0);}
 pane(D.floor,'Median noise floor (dB)',null,0);pane(D.peak,'Strongest component (dB)',D.fs_db,cv.width/2);
 var items=R.map(function(r,a){return {name:rl(r),color:COLORS[a%COLORS.length]};});items.push({name:'ADC full-scale equivalent',color:'#e05050'});legend('floorLegend',items);
}

// ---- downloads ----
function dl(name,text){var a=document.createElement('a');a.href=URL.createObjectURL(new Blob([text],{type:'text/csv'}));a.download=name;document.body.appendChild(a);a.click();document.body.removeChild(a);}
function csvq(v){v=String(v);return /[",\n]/.test(v)?'"'+v.replace(/"/g,'""')+'"':v;}
function addBtn(txt,fn){var b=el('button',txt);b.onclick=fn;$('dl').appendChild(b);}
addBtn('Surface chart CSV for the selected sample rate (Excel: Insert > Surface)',function(){var ri=+rSel.value,rows=[['']];
 G.forEach(function(g){rows[0].push(g.toFixed(1)+' dB');});
 for(var k=0;k<N;k++){var row=[D.freq[k]];for(var b=0;b<G.length;b++){var v=D.spec[ri][b]?D.spec[ri][b][k]:null;row.push(v===null||v===undefined?'':v);}rows.push(row);}
 dl('surface_'+(R[ri]/1e6).toFixed(3)+'MSps.csv',rows.map(function(r){return r.map(csvq).join(',');}).join('\n'));});
addBtn('Every pass (rate, gain, noise floor, peak, headroom, spikes) CSV',function(){var rows=[['sample_rate_hz','gain_db','repeat','noise_floor_db','strongest_db','headroom_to_full_scale_db','spikes','pass_seconds']];
 D.table.forEach(function(r){rows.push(r);});dl('survey_passes.csv',rows.map(function(r){return r.map(csvq).join(',');}).join('\n'));});
addBtn('Every spike found CSV',function(){var rows=[['sample_rate_hz','gain_db','freq_hz','level_db','prominence_db']];
 D.spikes.forEach(function(s){rows.push([R[s[0]],G[s[1]],s[2],s[3],s[4]]);});dl('survey_spikes.csv',rows.map(function(r){return r.map(csvq).join(',');}).join('\n'));});
addBtn('Features and verdicts CSV',function(){var rows=[['verdict','where','what_it_is','seen_at','typical_level','gain_slope','linear_up_to_db','first_visible_db']];
 F.forEach(function(f){rows.push([VNAME[f.verdict],f.where,f.cause,f.rate_text,f.level_text,f.slope===null?'':f.slope,f.lin_to===null?'':f.lin_to,f.first_vis===null?'':f.first_vis]);});dl('survey_features.csv',rows.map(function(r){return r.map(csvq).join(',');}).join('\n'));});

buildTrSel();buildFeat();drawHeats();drawSpec();drawFeat();drawFloor();
})();
</script></body></html>
'''
SURVEY_REPORT_TEMPLATE = r'''<!DOCTYPE html>
<html lang="en"><head><meta charset="utf-8"><title>RF gain / sample-rate survey</title>
<style>
body{background:#16181c;color:#d8dbe0;font:14px/1.45 Segoe UI,Arial,sans-serif;margin:0 auto;max-width:1180px;padding:16px 20px 60px}
h1{font-size:22px;margin:4px 0 2px}h2{font-size:17px;margin:26px 0 6px;border-bottom:1px solid #333;padding-bottom:3px}
.sub{color:#9aa0a8}.note{color:#9aa0a8;font-size:13px;margin:2px 0 8px}
canvas{display:block;width:100%;max-width:1100px;background:#14161a;border:1px solid #2a2d33}
.ctl{margin:6px 0}select,button{background:#262a31;color:#d8dbe0;border:1px solid #444;border-radius:4px;padding:3px 8px;font:inherit}
button{cursor:pointer}button:hover{background:#323843}
.find{margin:0;padding:0;list-style:none}.find li{padding:7px 10px;margin:4px 0;border-left:4px solid #4a90d9;background:#1d2026}
.find li.warn{border-left-color:#e0a030}.find li.bad{border-left-color:#e05050}.find li.ok{border-left-color:#5aa469}
table{border-collapse:collapse;width:100%;font-size:13px}th,td{padding:4px 7px;border-bottom:1px solid #2a2d33;text-align:left;vertical-align:top}
th{background:#1d2026}
.tag{display:inline-block;padding:0 7px;border-radius:9px;font-size:12px;color:#fff}
.t-ok{background:#2f6d3f}.t-extra{background:#b8741c}.t-bad{background:#a03a3a}
.hv{color:#ffd54a;margin-left:10px;font-family:Consolas,monospace;font-size:13px}
.scroll{max-height:420px;overflow:auto;border:1px solid #2a2d33}
.btns button{margin:3px 6px 3px 0}.legend span{margin-right:14px;font-size:13px}
</style></head><body>
<h1>RF scan - gain / sample-rate survey</h1>
<div class="sub" id="meta"></div>

<h2>1. Sample-rate effects</h2>
<p class="note">Each row is one sample rate (lowest at the bottom), colour = level at the chosen gain. A real or rate-independent signal is a <b>vertical line in every row</b>; anything that shifts, appears or vanishes from row to row depends on the sample rate. A <b>+</b> after a row label means that setting needed more than two scans to repeat; a <b>!</b> means it never repeated (see section 3).</p>
<div class="ctl">Gain: <select id="gSel"></select><span class="hv" id="hv1"></span></div>
<canvas id="heatRate" width="1100" height="320"></canvas>

<h2>2. Gain effects</h2>
<p class="note">Each row is one RF gain (lowest at the bottom) at the chosen sample rate. Something that appears only at higher gain, or brightens faster than its neighbours, is made inside the gain stages. Row labels are marked + / ! as above.</p>
<div class="ctl">Sample rate: <select id="rSel"></select><span class="hv" id="hv2"></span></div>
<canvas id="heatGain" width="1100" height="320"></canvas>

<h2>3. Repeatability of the scans</h2>
<p class="note">Every rate / gain setting was scanned at least twice. If the two scans were noticeably different (level shifted, many bins differing, or a spike present in one scan but not the other) another scan was made, and so on until two scans in a row agreed. The maps above show the average of the two agreeing scans (the last two, flagged !, if they never agreed).</p>
<ul class="find" id="repSummary"></ul>
<div id="repTable"></div>

<h2>4. Spectrum viewer</h2>
<p class="note">Overlay up to three sample-rate / gain combinations. Drag on the plot to zoom, double-click to reset.</p>
<div class="ctl" id="trSel"></div>
<canvas id="specPlot" width="1100" height="360"></canvas>
<div class="legend" id="specLegend"></div>

<h2>5. Noise floor and ADC headroom</h2>
<p class="note">Left: median noise floor against gain. Right: strongest component against gain; the dashed line is the ADC full-scale equivalent on this level scale.</p>
<canvas id="floorPlot" width="1100" height="300"></canvas>
<div class="legend" id="floorLegend"></div>

<h2>6. Data</h2>
<div class="btns" id="dl"></div>
<p class="note">All the numbers behind this page are embedded in this single file; the buttons export CSV (for Excel) from it.</p>

<script id="data" type="application/json">__DATA__</script>
<script>
(function(){
var D=JSON.parse(document.getElementById('data').textContent);
function $(id){return document.getElementById(id);}
var R=D.rates,G=D.gains,N=D.freq.length,FL=D.flags;
var COLORS=['#ffd54a','#4fc3f7','#ff7a7a','#9ccc65','#ba68c8','#ffb74d','#4db6ac','#f06292','#b0bec5','#dce775'];
function rl(r){return (r/1e6).toFixed(3)+' MS/s';}
function gl(g){return g.toFixed(1)+' dB';}
function cmap(t){t=Math.max(0,Math.min(1,t));var s=[[68,1,84],[59,82,139],[33,145,140],[94,201,98],[253,231,37]];var x=t*(s.length-1),i=Math.min(s.length-2,Math.floor(x)),f=x-i;return [0,1,2].map(function(k){return Math.round(s[i][k]+(s[i+1][k]-s[i][k])*f);});}
function el(tag,txt,cls){var e=document.createElement(tag);if(txt!==undefined)e.textContent=txt;if(cls)e.className=cls;return e;}
function mark(a,b){var f=FL[a][b];return f===2?' !':(f===1?' +':'');}
function markText(a,b){var f=FL[a][b];return f===2?' | NEVER REPEATED':(f===1?' | needed extra scans':'');}
var m=D.meta;
$('meta').textContent=m.band+' | '+R.length+' sample rate(s), '+G.length+' gain step(s), '+m.combos+' setting(s), '+m.scans+' scan(s) in total'+(m.partial?' | PARTIAL RESULT - survey still running or stopped early':'')+' | averaging '+m.n_avg+' | '+m.created;

// ---- colour scale + maps ----
var all=[];D.spec.forEach(function(row){row.forEach(function(v){if(v){for(var i=0;i<v.length;i+=3){if(v[i]!==null)all.push(v[i]);}}});});
all.sort(function(a,b){return a-b;});
var VMIN=all.length?all[Math.floor(all.length*0.02)]:-100,VMAX=all.length?all[Math.floor(all.length*0.9985)]:0;
if(VMAX<=VMIN)VMAX=VMIN+1;
function drawHeat(cv,rows,labels){
 var ctx=cv.getContext('2d'),W=cv.width,H=cv.height,L=104,T=6,B=32,Rm=10,pw=W-L-Rm,ph=H-T-B,nr=rows.length;
 ctx.fillStyle='#14161a';ctx.fillRect(0,0,W,H);
 var img=ctx.createImageData(pw,ph);
 for(var y=0;y<ph;y++){var r=nr-1-Math.min(nr-1,Math.floor(y*nr/ph));var row=rows[r];
  for(var x=0;x<pw;x++){var o=(y*pw+x)*4,v=null;if(row){v=row[Math.min(N-1,Math.floor(x*N/pw))];}
   if(v===null||v===undefined||isNaN(v)){img.data[o]=28;img.data[o+1]=28;img.data[o+2]=32;}
   else{var c=cmap((v-VMIN)/(VMAX-VMIN));img.data[o]=c[0];img.data[o+1]=c[1];img.data[o+2]=c[2];}
   img.data[o+3]=255;}}
 ctx.putImageData(img,L,T);
 ctx.fillStyle='#ccc';ctx.font='11px sans-serif';ctx.textAlign='right';
 var step=Math.max(1,Math.ceil(nr/14));
 for(var r2=0;r2<nr;r2+=step){var yy=T+(nr-1-r2+0.5)*ph/nr;ctx.fillText(labels[r2],L-6,yy+4);}
 ctx.textAlign='center';var f0=D.freq[0],f1=D.freq[N-1],nt=8;
 for(var k=0;k<=nt;k++){var fx=f0+(f1-f0)*k/nt,xx=L+pw*k/nt;ctx.fillText(fx.toFixed(3),xx,H-16);ctx.fillRect(xx,T+ph,1,4);}
 ctx.fillText('Frequency (MHz)   colour scale '+VMIN.toFixed(0)+' (dark) to '+VMAX.toFixed(0)+' dB (yellow)',L+pw/2,H-3);
 cv._g={L:L,T:T,pw:pw,ph:ph,nr:nr};
}
function hover(cv,cb){
 cv.addEventListener('mousemove',function(e){var g=cv._g;if(!g)return;var rc=cv.getBoundingClientRect();
  var x=(e.clientX-rc.left)*cv.width/rc.width-g.L,y=(e.clientY-rc.top)*cv.height/rc.height-g.T;
  if(x<0||y<0||x>=g.pw||y>=g.ph){cb(null,null);return;}
  cb(g.nr-1-Math.min(g.nr-1,Math.floor(y*g.nr/g.ph)),Math.min(N-1,Math.floor(x*N/g.pw)));});
 cv.addEventListener('mouseleave',function(){cb(null,null);});
}
var gSel=$('gSel'),rSel=$('rSel');
G.forEach(function(g,i){var o=el('option',gl(g));o.value=i;gSel.appendChild(o);});
R.forEach(function(r,i){var o=el('option',rl(r));o.value=i;rSel.appendChild(o);});
gSel.value=Math.floor(G.length/2);rSel.value=R.length-1;
function drawHeats(){var gi=+gSel.value,ri=+rSel.value;
 drawHeat($('heatRate'),R.map(function(r,i){return D.spec[i][gi];}),R.map(function(r,i){return rl(r)+mark(i,gi);}));
 drawHeat($('heatGain'),G.map(function(g,j){return D.spec[ri][j];}),G.map(function(g,j){return gl(g)+mark(ri,j);}));}
gSel.onchange=drawHeats;rSel.onchange=drawHeats;
hover($('heatRate'),function(r,c){var gi=+gSel.value;if(r===null){$('hv1').textContent='';return;}var v=D.spec[r][gi]?D.spec[r][gi][c]:null;$('hv1').textContent=D.freq[c].toFixed(4)+' MHz | '+rl(R[r])+' | '+(v===null||v===undefined?'-':v.toFixed(1)+' dB')+markText(r,gi);});
hover($('heatGain'),function(r,c){var ri=+rSel.value;if(r===null){$('hv2').textContent='';return;}var v=D.spec[ri][r]?D.spec[ri][r][c]:null;$('hv2').textContent=D.freq[c].toFixed(4)+' MHz | '+gl(G[r])+' | '+(v===null||v===undefined?'-':v.toFixed(1)+' dB')+markText(ri,r);});

// ---- repeatability ----
D.rep.summary.forEach(function(f){$('repSummary').appendChild(el('li',f.text,f.level));});
(function(){var c=$('repTable');c.textContent='';
 if(!D.rep.rows.length){c.appendChild(el('p','Every setting repeated at the first two scans.','note'));return;}
 var t=el('table'),hr=el('tr');
 ['Sample rate','Gain','Scans made','Outcome','Scan 1 vs 2: level shift','Scan 1 vs 2: bins that differ','Scan 1 vs 2: spikes that differ','Last comparison: shift / bins / spikes'].forEach(function(h){hr.appendChild(el('th',h));});t.appendChild(hr);
 D.rep.rows.forEach(function(r){var tr=el('tr');
  [rl(r.rate),gl(r.gain),String(r.scans)].forEach(function(v){tr.appendChild(el('td',v));});
  var td=el('td');td.appendChild(el('span',r.outcome,'tag '+(r.converged?'t-extra':'t-bad')));tr.appendChild(td);
  tr.appendChild(el('td',r.first.shift===null?'-':r.first.shift.toFixed(2)+' dB'));tr.appendChild(el('td',r.first.local===null?'-':(r.first.local*100).toFixed(1)+' %'));tr.appendChild(el('td',String(r.first.spikes)));
  tr.appendChild(el('td',r.last.shift===null?'-':r.last.shift.toFixed(2)+' dB / '+(r.last.local*100).toFixed(1)+' % / '+r.last.spikes));t.appendChild(tr);});
 var w=el('div');w.className='scroll';w.appendChild(t);c.appendChild(w);})();

// ---- generic line plot ----
function linePlot(cv,series,o){
 var ctx=cv.getContext('2d'),W=cv.width,H=cv.height,L=64,T=14,B=36,Rm=14,pw=W-L-Rm,ph=H-T-B;
 var x0=o.x0,x1=o.x1,y0=Infinity,y1=-Infinity;
 series.forEach(function(s){s.pts.forEach(function(p){if(p[0]>=x0&&p[0]<=x1&&p[1]!==null&&isFinite(p[1])){y0=Math.min(y0,p[1]);y1=Math.max(y1,p[1]);}});});
 if(!isFinite(y0)){y0=0;y1=1;}if(o.y0!==undefined)y0=Math.min(y0,o.y0);if(o.y1!==undefined)y1=Math.max(y1,o.y1);
 var pad=(y1-y0)*0.06||1;y0-=pad;y1+=pad;
 ctx.fillStyle='#14161a';ctx.fillRect(0,0,W,H);
 function X(v){return L+(v-x0)/(x1-x0)*pw;}function Y(v){return T+(1-(v-y0)/(y1-y0))*ph;}
 ctx.strokeStyle='#2a2d33';ctx.fillStyle='#aaa';ctx.font='11px sans-serif';
 var i,nt=8;ctx.textAlign='center';
 for(i=0;i<=nt;i++){var xv=x0+(x1-x0)*i/nt,xx=X(xv);ctx.beginPath();ctx.moveTo(xx,T);ctx.lineTo(xx,T+ph);ctx.stroke();ctx.fillText(xv.toFixed(o.xd),xx,H-18);}
 ctx.textAlign='right';
 for(i=0;i<=6;i++){var yv=y0+(y1-y0)*i/6,yy=Y(yv);ctx.beginPath();ctx.moveTo(L,yy);ctx.lineTo(L+pw,yy);ctx.stroke();ctx.fillText(yv.toFixed(1),L-6,yy+4);}
 ctx.textAlign='center';ctx.fillText(o.xl,L+pw/2,H-4);
 ctx.save();ctx.translate(12,T+ph/2);ctx.rotate(-Math.PI/2);ctx.fillText(o.yl,0,0);ctx.restore();
 ctx.save();ctx.beginPath();ctx.rect(L,T,pw,ph);ctx.clip();
 series.forEach(function(s){ctx.strokeStyle=s.color;ctx.fillStyle=s.color;ctx.lineWidth=s.w||1.4;ctx.setLineDash(s.dash||[]);ctx.beginPath();var started=false;
  s.pts.forEach(function(p){if(p[1]===null||!isFinite(p[1])){started=false;return;}var px=X(p[0]),py=Y(p[1]);if(!started){ctx.moveTo(px,py);started=true;}else ctx.lineTo(px,py);});ctx.stroke();ctx.setLineDash([]);
  if(s.dots){s.pts.forEach(function(p){if(p[1]!==null&&isFinite(p[1])){ctx.beginPath();ctx.arc(X(p[0]),Y(p[1]),3,0,6.3);ctx.fill();}});}});
 ctx.restore();
 cv._p={L:L,T:T,pw:pw,ph:ph,x0:x0,x1:x1};
}
function legend(id,items){var d=$(id);d.textContent='';items.forEach(function(it){var s=el('span',it.name);s.style.color=it.color;d.appendChild(s);});}
var GX1=G[G.length-1]>G[0]?G[G.length-1]:G[0]+1;

// ---- spectrum viewer with zoom ----
var tr=[{r:0,g:0,on:true},{r:R.length-1,g:G.length-1,on:true},{r:Math.min(1,R.length-1),g:0,on:false}];
var view={x0:D.freq[0],x1:D.freq[N-1]},drag=null;
function buildTrSel(){var box=$('trSel');box.textContent='';
 tr.forEach(function(t,i){var cb=el('input');cb.type='checkbox';cb.checked=t.on;cb.onchange=function(){t.on=cb.checked;drawSpec();};
  var rs=el('select'),gs=el('select');R.forEach(function(r,k){var o=el('option',rl(r));o.value=k;rs.appendChild(o);});G.forEach(function(g,k){var o=el('option',gl(g));o.value=k;gs.appendChild(o);});
  rs.value=t.r;gs.value=t.g;rs.onchange=function(){t.r=+rs.value;drawSpec();};gs.onchange=function(){t.g=+gs.value;drawSpec();};
  var lab=el('span',' '+String.fromCharCode(65+i)+': ');lab.style.color=COLORS[i];lab.style.marginLeft=i?'14px':'0';
  box.appendChild(cb);box.appendChild(lab);box.appendChild(rs);box.appendChild(document.createTextNode(' '));box.appendChild(gs);});}
function drawSpec(){
 var series=[],items=[];
 tr.forEach(function(t,i){if(!t.on||!D.spec[t.r][t.g])return;var v=D.spec[t.r][t.g];var pts=[];for(var k=0;k<N;k++)pts.push([D.freq[k],v[k]]);
  series.push({pts:pts,color:COLORS[i],w:1.2});items.push({name:String.fromCharCode(65+i)+': '+rl(R[t.r])+' / '+gl(G[t.g])+mark(t.r,t.g),color:COLORS[i]});});
 linePlot($('specPlot'),series,{x0:view.x0,x1:view.x1,xd:3,xl:'Frequency (MHz)',yl:'Level (dB)'});legend('specLegend',items);
 if(drag){var cv=$('specPlot'),ctx=cv.getContext('2d');ctx.fillStyle='rgba(255,255,255,0.15)';ctx.fillRect(Math.min(drag.a,drag.b),0,Math.abs(drag.b-drag.a),cv.height);}
}
(function(){var cv=$('specPlot');
 function px(e){var rc=cv.getBoundingClientRect();return (e.clientX-rc.left)*cv.width/rc.width;}
 function fx(x){var p=cv._p;return p.x0+(x-p.L)/p.pw*(p.x1-p.x0);}
 cv.addEventListener('mousedown',function(e){drag={a:px(e),b:px(e)};});
 cv.addEventListener('mousemove',function(e){if(drag){drag.b=px(e);drawSpec();}});
 window.addEventListener('mouseup',function(){if(!drag)return;var a=drag.a,b=drag.b;drag=null;if(Math.abs(b-a)>6){var f0=fx(Math.min(a,b)),f1=fx(Math.max(a,b));view={x0:Math.max(D.freq[0],f0),x1:Math.min(D.freq[N-1],f1)};}drawSpec();});
 cv.addEventListener('dblclick',function(){view={x0:D.freq[0],x1:D.freq[N-1]};drawSpec();});})();

// ---- floor / headroom ----
function drawFloor(){
 var cv=$('floorPlot'),ctx=cv.getContext('2d');ctx.fillStyle='#14161a';ctx.fillRect(0,0,cv.width,cv.height);
 var half=document.createElement('canvas');half.width=cv.width/2;half.height=cv.height;
 function pane(vals,yl,fs,off){var s=[];R.forEach(function(r,a){s.push({pts:G.map(function(g,b){return [g,vals[a][b]];}),color:COLORS[a%COLORS.length],dots:true});});
  if(fs!==null)s.push({pts:[[G[0],fs],[GX1,fs]],color:'#e05050',dash:[6,5],w:1.2});
  linePlot(half,s,{x0:G[0],x1:GX1,xd:0,xl:'RF gain (dB)',yl:yl,y1:fs===null?undefined:fs});ctx.drawImage(half,off,0);}
 pane(D.floor,'Median noise floor (dB)',null,0);pane(D.peak,'Strongest component (dB)',D.fs_db,cv.width/2);
 var items=R.map(function(r,a){return {name:rl(r),color:COLORS[a%COLORS.length]};});items.push({name:'ADC full-scale equivalent',color:'#e05050'});legend('floorLegend',items);
}

// ---- downloads ----
function dl(name,text){var a=document.createElement('a');a.href=URL.createObjectURL(new Blob([text],{type:'text/csv'}));a.download=name;document.body.appendChild(a);a.click();document.body.removeChild(a);}
function csvq(v){v=String(v);return /[",\n]/.test(v)?'"'+v.replace(/"/g,'""')+'"':v;}
function addBtn(txt,fn){var b=el('button',txt);b.onclick=fn;$('dl').appendChild(b);}
addBtn('Surface chart CSV for the selected sample rate (Excel: Insert > Surface)',function(){var ri=+rSel.value,rows=[['']];
 G.forEach(function(g){rows[0].push(g.toFixed(1)+' dB');});
 for(var k=0;k<N;k++){var row=[D.freq[k]];for(var b=0;b<G.length;b++){var v=D.spec[ri][b]?D.spec[ri][b][k]:null;row.push(v===null||v===undefined?'':v);}rows.push(row);}
 dl('surface_'+(R[ri]/1e6).toFixed(3)+'MSps.csv',rows.map(function(r){return r.map(csvq).join(',');}).join('\n'));});
addBtn('Every scan (rate, gain, scan number, used in maps, noise floor, peak, headroom, spikes) CSV',function(){var rows=[['sample_rate_hz','gain_db','scan_number','used_in_maps','noise_floor_db','strongest_db','headroom_to_full_scale_db','spikes','scan_seconds']];
 D.table.forEach(function(r){rows.push(r);});dl('survey_scans.csv',rows.map(function(r){return r.map(csvq).join(',');}).join('\n'));});
addBtn('Repeatability CSV (every setting)',function(){var rows=[['sample_rate_hz','gain_db','scans_made','outcome','first_pair_level_shift_db','first_pair_bins_differing_percent','first_pair_spikes_differing']];
 D.rep.all.forEach(function(r){rows.push(r);});dl('survey_repeatability.csv',rows.map(function(r){return r.map(csvq).join(',');}).join('\n'));});
addBtn('Every spike found CSV',function(){var rows=[['sample_rate_hz','gain_db','freq_hz','level_db','prominence_db']];
 D.spikes.forEach(function(s){rows.push([R[s[0]],G[s[1]],s[2],s[3],s[4]]);});dl('survey_spikes.csv',rows.map(function(r){return r.map(csvq).join(',');}).join('\n'));});

buildTrSel();drawHeats();drawSpec();drawFloor();
})();
</script></body></html>
'''


class SurveySignals(QtCore.QObject):
    # survey worker thread -> GUI thread: True if the survey was stopped early
    done = QtCore.pyqtSignal(object)
    # survey worker thread -> GUI thread: a new rate/gain test has started, show its settings on the controls
    progress = QtCore.pyqtSignal()

# ---- Baseband SCOPE defaults (see the EmcScanner.scope_* methods) ----
SCOPE_MAX_SAMPLES = 2097152     # most samples captured per scope frame (2**21)
SCOPE_MIN_SAMPLES = 512         # fewest samples ever displayed per frame
SCOPE_CLIP_LEVEL = 0.99         # |I| or |Q| at/above this (1.0 = ADC full scale) counts as clipped
SCOPE_COMPLEX_DIVISOR = 128.0   # same normalisation _read_normalized_iq() applies to complex samples
                                # (if the scope's peak readout never gets above ~0.01 even on a strong
                                #  signal, your driver already returns +/-1.0 data: set this to 1.0)
SCOPE_TOP_TAPER = 0.10          # top 10 % below Nyquist rolls off smoothly (anti-alias skirt, less ringing)
SCOPE_STALE_SKIP = 8192         # samples read and thrown away at the start of EVERY capture (stale USB/FIFO data)
SCOPE_GUARD = 2048              # extra samples captured at EACH end, filtered with the rest, then trimmed off
SCOPE_LOW_CUT_FRAC = 0.01       # lower band edge: fully stopped below this fraction of Nyquist, fully passed
                                # above 3x it (smooth raised-cosine) - rejects the upconverter's 28.8 MHz
                                # LO feed-through / DC at RF ~0 without a sharp filter edge
SCOPE_WINDOWS_MS = [0.05, 0.1, 0.2, 0.5, 1, 2, 5, 10, 20, 50, 100, 200, 500, 1000]   # capture windows offered
# Scope spin boxes always use '.' as the decimal point, whatever the PC's regional settings are.
SCOPE_LOCALE = QtCore.QLocale(QtCore.QLocale.English, QtCore.QLocale.UnitedKingdom)


class ScopeSignals(QtCore.QObject):
    # worker thread -> GUI thread: (time_s, trace1, trace2_or_None, info_dict)
    frame = QtCore.pyqtSignal(object, object, object, object)


SCOPE_Y_DEFAULT = 0.1           # initial vertical range, +/- fraction of ADC full scale
SCOPE_Y_MIN = 1e-5              # smallest / largest +/- vertical range selectable
SCOPE_Y_MAX = 2.0
SCOPE_TRIG_HYST_OF_RANGE = 1.0 / 40.0   # trigger hysteresis (Schmitt band) = this fraction of the screen's full Y range
SCOPE_PULSE_K = 6.0             # a trace sample this many robust sigmas (MAD) from the median counts as a pulse
SCOPE_FS_DROP_WARN = 2.56e6     # sample rates above this can drop USB samples on some PCs/ports (see SAMPLE_RATE_CHOICES)
SCOPE_MAX_RATE = 2.56e6          # the scope only offers sample rates up to this (3.0 / 3.2 MS/s show a pulse train in the stream)
SCOPE_SAFE_RATE = 2.4e6         # rate selected automatically if the scope is started above SCOPE_MAX_RATE


def _scope_next_125(v, up):
    """Next value up/down in the 1-2-5 sequence (... 0.01, 0.02, 0.05, 0.1, 0.2, 0.5, 1 ...)."""
    v = max(float(v), 1e-12)
    e = int(math.floor(math.log10(v)))
    seq = sorted(m * 10.0 ** k for k in range(e - 2, e + 3) for m in (1, 2, 5))
    if up:
        return next((c for c in seq if c > v * 1.0001), v)
    return next((c for c in reversed(seq) if c < v * 0.9999), v)


class ScopeYSpin(QtWidgets.QDoubleSpinBox):
    # Scope-style vertical range: each up/down click goes to the next 1-2-5 value.
    def stepBy(self, steps):
        v = self.value()
        for _ in range(abs(int(steps))):
            v = _scope_next_125(v, steps > 0)
        self.setValue(v)


class StitchSignals(QtCore.QObject):
    # stitch-check worker thread -> GUI thread: the result dict (or an aborted / error result)
    done = QtCore.pyqtSignal(object)


class _DigitLabel(QtWidgets.QLabel):
    """One digit of the DigitalFreqSelector: click to select it, mouse wheel to step it."""
    def __init__(self, exponent, owner):
        super().__init__("0")
        self._k = exponent
        self._owner = owner
        self.setAlignment(QtCore.Qt.AlignCenter)
        self.setFixedSize(24, 34)
        self.setCursor(QtCore.Qt.PointingHandCursor)

    def mousePressEvent(self, e):
        self._owner.select(self._k)
        self._owner.setFocus()

    def wheelEvent(self, e):
        self._owner.select(self._k)
        self._owner.step(self._k, 1 if e.angleDelta().y() > 0 else -1)
        e.accept()


class DigitalFreqSelector(QtWidgets.QWidget):
    """
    Centre-frequency selector made of individual digits, like the dial of a bench instrument:
        0100.300 000 MHz        (4 MHz digits . 3 kHz digits   3 Hz digits)
    Every digit has an up and a down arrow, steps with the mouse wheel when the pointer is over it, and can be
    selected by clicking it; the selected digit (outlined) also answers to the Up / Down keys, Left / Right
    move the selection, and typing 0-9 overwrites it and moves on. A step adds or subtracts that digit's
    place value from the whole frequency (so it carries and borrows like any counter) and is limited to
    [min_hz, max_hz]. valueChanged(int hz) is emitted on every change made by the operator.
    """
    valueChanged = QtCore.pyqtSignal(int)
    N_DIGITS = 10                                   # up to 9999.999999 MHz in 1 Hz steps

    def __init__(self, value_hz=100000000, min_hz=100000, max_hz=1766000000, parent=None):
        super().__init__(parent)
        self._min, self._max = int(min_hz), int(max_hz)
        self._value = max(self._min, min(self._max, int(round(value_hz))))
        self._sel = 6                               # selected digit = its power of ten (6 = the 1 MHz digit)
        self._labels = {}
        self.setFocusPolicy(QtCore.Qt.StrongFocus)
        row = QtWidgets.QHBoxLayout(self)
        row.setContentsMargins(0, 0, 0, 0)
        row.setSpacing(1)
        for k in range(self.N_DIGITS - 1, -1, -1):
            if k == 5:
                row.addWidget(self._sep("."))
            elif k == 2:
                row.addSpacing(10)                  # gap between the kHz and Hz groups
            col = QtWidgets.QVBoxLayout()
            col.setSpacing(0)
            up, dn = self._arrow(QtCore.Qt.UpArrow, k, +1), self._arrow(QtCore.Qt.DownArrow, k, -1)
            lbl = _DigitLabel(k, self)
            self._labels[k] = lbl
            col.addWidget(up)
            col.addWidget(lbl)
            col.addWidget(dn)
            row.addLayout(col)
        unit = QtWidgets.QLabel("MHz")
        unit.setStyleSheet("QLabel { color: #FFD54A; font: bold 16px 'Consolas'; padding-left: 6px; }")
        row.addWidget(unit)
        self._refresh()

    def _sep(self, text):
        lab = QtWidgets.QLabel(text)
        lab.setFixedWidth(10)
        lab.setAlignment(QtCore.Qt.AlignHCenter | QtCore.Qt.AlignVCenter)
        lab.setStyleSheet("QLabel { color: #FFD54A; font: bold 22px 'Consolas'; }")
        return lab

    def _arrow(self, arrow, k, direction):
        b = QtWidgets.QToolButton()
        b.setArrowType(arrow)
        b.setFixedSize(24, 14)
        b.setAutoRepeat(True)
        b.setAutoRepeatDelay(450)
        b.setAutoRepeatInterval(110)
        b.setFocusPolicy(QtCore.Qt.NoFocus)
        b.clicked.connect(lambda _=False, k=k, d=direction: (self.select(k), self.step(k, d)))
        return b

    def value(self):
        return self._value

    def setValue(self, hz, emit_signal=False):
        hz = max(self._min, min(self._max, int(round(hz))))
        if hz != self._value:
            self._value = hz
            self._refresh()
            if emit_signal:
                self.valueChanged.emit(self._value)

    def select(self, k):
        self._sel = max(0, min(self.N_DIGITS - 1, int(k)))
        self._refresh()

    def step(self, k, direction):
        """Add / subtract the place value of digit k (carry and borrow come for free)."""
        new = self._value + direction * (10 ** int(k))
        new = max(self._min, min(self._max, new))
        if new != self._value:
            self._value = new
            self._refresh()
            self.valueChanged.emit(self._value)

    def keyPressEvent(self, e):
        key = e.key()
        if key == QtCore.Qt.Key_Up:
            self.step(self._sel, +1)
        elif key == QtCore.Qt.Key_Down:
            self.step(self._sel, -1)
        elif key == QtCore.Qt.Key_Left:
            self.select(self._sel + 1)
        elif key == QtCore.Qt.Key_Right:
            self.select(self._sel - 1)
        elif QtCore.Qt.Key_0 <= key <= QtCore.Qt.Key_9:
            k = self._sel
            cur = (self._value // 10 ** k) % 10
            new = max(self._min, min(self._max, self._value + (key - QtCore.Qt.Key_0 - cur) * 10 ** k))
            if new != self._value:
                self._value = new
                self.valueChanged.emit(self._value)
            self._refresh()
            self.select(self._sel - 1)
        else:
            super().keyPressEvent(e)

    def _refresh(self):
        for k, lab in self._labels.items():
            lab.setText(str((self._value // 10 ** k) % 10))
            dim = (self._value < 10 ** k) and k > 6          # leading zeros above the MHz units digit are dimmed
            border = "#FFD54A" if k == self._sel else "#3a3a3a"
            colour = "#6a5a22" if dim else "#FFD54A"
            lab.setStyleSheet(f"QLabel {{ background: #101010; color: {colour}; font: bold 22px 'Consolas'; "
                              f"border: 2px solid {border}; }}")


class PlotSignals(QtCore.QObject):
    update = QtCore.pyqtSignal(object, object)
    # Emitted by sweep_loop() (worker thread) whenever it rebuilds self.centers, so the
    # GUI thread can redraw the frequency markers to match. draw_block_markers() touches
    # Qt scene items directly and must only ever run on the GUI thread - this signal is
    # what makes that safe when the rebuild itself happens on the worker thread (see the
    # live Sample Rate control in sweep_loop()).
    markers_changed = QtCore.pyqtSignal()

class EmcScanner(QtWidgets.QMainWindow):


    def log(self, msg):
        # Thread‑safe append to debug window
        QtCore.QMetaObject.invokeMethod(
            self.debug,
            "appendPlainText",
            QtCore.Qt.QueuedConnection,
            QtCore.Q_ARG(str, msg)
        )

    def update_status_panel(self):
        """
        Rebuild the stable status panel text from the current live settings. Called
        whenever a setting actually changes (set_preset, averaging toggled/changed) -
        NOT once per sweep block - and safe to call from either thread since QLabel's
        setText is invoked via a queued call, matching the thread-safety pattern log() uses.
        """
        mode_label = getattr(self, "current_mode_label", "-")
        avg_text = (f"ON ({self.avg_count})" if getattr(self, "avg_enabled", False)
                    else "OFF")
        # Show the actual applied gain alongside the requested one - but ONLY when
        # actual_gain_db is known to have been measured against the CURRENTLY selected
        # gain_db. Right after a MODE switch, self.gain_db has changed but the hardware
        # hasn't been touched yet (that only happens when a sweep next starts), so
        # actual_gain_db/actual_gain_db_for_request still reflect the PREVIOUS mode -
        # comparing them against the new gain_db would report a false, stale mismatch.
        actual = getattr(self, "actual_gain_db", None)
        actual_for = getattr(self, "actual_gain_db_for_request", None)
        if actual is not None and actual_for == self.gain_db:
            if abs(actual - self.gain_db) > 0.5:
                gain_text = f"{self.gain_db:.1f} dB requested -> {actual:.1f} dB actual"
            else:
                gain_text = f"{self.gain_db:.1f} dB"
        else:
            gain_text = f"{self.gain_db:.1f} dB (not yet applied - press Start)"
        text = (
            f"Mode: {mode_label}\n"
            f"Start: {self.START_FREQ/1e6:.3f} MHz    Stop: {self.STOP_FREQ/1e6:.3f} MHz\n"
            f"Sample Rate: {self.SAMPLE_RATE/1e6:.3f} MS/s    Gain: {gain_text}\n"
            f"Averaging: {avg_text}"
        )
        QtCore.QMetaObject.invokeMethod(
            self.status_panel,
            "setText",
            QtCore.Qt.QueuedConnection,
            QtCore.Q_ARG(str, text)
        )

    # Use this to build centers across the sweep band
    # start_hz, stop_hz, sample_rate are floats
    # ---- previous build_centers() (no layout shift), kept in case of problems ----
#     def build_centers(self, start_hz, stop_hz, sample_rate):
#         """
#         Build sweep centers so that, once each block's masked edges are discarded (see
#         STITCH_EDGE_MASK_FRACTION / self.stitch_edge_mask_fraction), the remaining "kept"
#         slices tile the sweep range with no gaps and no overlap.
#         Each block still captures the FULL sample_rate of bandwidth - the FFT size,
#         window, and sample rate are unchanged. Only the displayed/stored portion of each
#         block is narrower (the middle (1 - 2*mask_frac) of it), so centers hop by that
#         narrower "display" width instead of by the full sample_rate.
#         """
#         mask_frac = float(getattr(self, "stitch_edge_mask_fraction", 0.0))
#         display_width = sample_rate * (1.0 - 2.0 * mask_frac)
#         if display_width <= 0:
#             # Degenerate configuration (mask_frac >= 0.5) - fall back to no masking
#             # rather than dividing by ~0 or producing a reversed/empty sweep.
#             display_width = sample_rate
#             mask_frac = 0.0
#         # first center is positioned so the first block's KEPT (displayed) slice starts
#         # exactly at start_hz, not the first block's raw captured edge.
#         first = start_hz + sample_rate * (0.5 - mask_frac)
#         # Number of blocks needed so the kept slices cover [start_hz, stop_hz] completely.
#         # (The old np.arange(first, stop_hz, ...) form dropped any centre at or above stop_hz,
#         # which both left the top of a band unswept - e.g. the trace ending at 1.55 MHz in a
#         # 150k-2M mode at 2.0 MS/s - and returned NO centres at all when one block covers
#         # the whole band but its centre falls above stop_hz, e.g. 150k-1.2M at 3.2 MS/s.)
#         span = stop_hz - start_hz
#         n_blocks = max(1, int(math.ceil(span / display_width - 1e-9)))
#         return first + display_width * np.arange(n_blocks, dtype=np.float64)

    def build_centers(self, start_hz, stop_hz, sample_rate):
        """
        Build sweep centers so that, once each block's masked edges are discarded (see
        STITCH_EDGE_MASK_FRACTION / self.stitch_edge_mask_fraction), the remaining "kept"
        slices tile the sweep range with no gaps and no overlap.

        Each block still captures the FULL sample_rate of bandwidth - the FFT size,
        window, and sample rate are unchanged. Only the displayed/stored portion of each
        block is narrower (the middle (1 - 2*mask_frac) of it), so centers hop by that
        narrower "display" width instead of by the full sample_rate.
        """
        mask_frac = float(getattr(self, "stitch_edge_mask_fraction", 0.0))
        display_width = sample_rate * (1.0 - 2.0 * mask_frac)
        if display_width <= 0:
            # Degenerate configuration (mask_frac >= 0.5) - fall back to no masking
            # rather than dividing by ~0 or producing a reversed/empty sweep.
            display_width = sample_rate
            mask_frac = 0.0
        # first center is positioned so the first block's KEPT (displayed) slice starts
        # exactly at start_hz, not the first block's raw captured edge.
        # STITCH CHECK: self._stitch_shift (normally 0) moves the whole block layout down by that fraction of a
        # block's kept width, so a second sweep measures every frequency from a different place in the passband.
        shift = float(getattr(self, "_stitch_shift", 0.0))
        start_hz = start_hz - shift * display_width
        first = start_hz + sample_rate * (0.5 - mask_frac)
        # Number of blocks needed so the kept slices cover [start_hz, stop_hz] completely.
        # (The old np.arange(first, stop_hz, ...) form dropped any centre at or above stop_hz,
        # which both left the top of a band unswept - e.g. the trace ending at 1.55 MHz in a
        # 150k-2M mode at 2.0 MS/s - and returned NO centres at all when one block covers
        # the whole band but its centre falls above stop_hz, e.g. 150k-1.2M at 3.2 MS/s.)
        span = stop_hz - start_hz
        n_blocks = max(1, int(math.ceil(span / display_width - 1e-9)))
        return first + display_width * np.arange(n_blocks, dtype=np.float64)

    def draw_block_markers(self):
            # remove old markers
            for item in self._block_markers:
                    try:
                            self.plot.removeItem(item)
                    except Exception:
                            pass
            self._block_markers.clear()
            # Raw captured block width (full sampled bandwidth, before masking) - green.
            half_capture = self.SAMPLE_RATE / 2.0
            # Kept/displayed width after edge masking (what actually reaches the stitched
            # trace) - cyan dashed. Computed from the same integer bin count sweep_loop()
            # actually uses, so these markers line up exactly with the real stitch boundary.
            edge_bins = self.stitch_get_edge_bins()
            kept_bins = NFFT - 2 * edge_bins
            half_kept = kept_bins * self.SAMPLE_RATE / (2.0 * NFFT)
            visible = bool(getattr(self, "markers_visible", True))
            for c in self.centers:
                    left = c - half_capture
                    right = c + half_capture
                    # vertical lines at left and right (raw captured block edges)
                    line_l = pg.InfiniteLine(pos=left, angle=90, pen=pg.mkPen('g', width=1))
                    line_r = pg.InfiniteLine(pos=right, angle=90, pen=pg.mkPen('g', width=1))
                    line_l.setVisible(visible)
                    line_r.setVisible(visible)
                    self.plot.addItem(line_l)
                    self.plot.addItem(line_r)
                    self._block_markers.append(line_l)
                    self._block_markers.append(line_r)

                    # kept/displayed slice edges (after edge masking)
                    kept_left = c - half_kept
                    kept_right = c + half_kept
                    line_kl = pg.InfiniteLine(pos=kept_left, angle=90,
                                               pen=pg.mkPen('c', width=1, style=QtCore.Qt.DashLine))
                    line_kr = pg.InfiniteLine(pos=kept_right, angle=90,
                                               pen=pg.mkPen('c', width=1, style=QtCore.Qt.DashLine))
                    line_kl.setVisible(visible)
                    line_kr.setVisible(visible)
                    self.plot.addItem(line_kl)
                    self.plot.addItem(line_kr)
                    self._block_markers.append(line_kl)
                    self._block_markers.append(line_kr)
    def __init__(self):
        super().__init__()
        # ---- CISPR state ----
        # hf_mode reflects which sensing path is active (RTL-SDR direct-sampling "HF" input
        # vs the normal tuner path). set_preset() updates this per preset.
        self.hf_mode = False
        # CISPR OFFSET: the single operator-entered value from the GUI spinbox. It is
        # deliberately NOT keyed by mode - it must persist unchanged across MODE button
        # presses, since it represents the operator's own calibration adjustment, not
        # something a preset should reset.
        self.cispr_offset_db = 0.0
        # dBuV_Calibration: a per-mode constant carried on each preset (see self.presets)
        # and copied into this attribute by set_preset() whenever a MODE button is
        # pressed. Different modes/bands/antennas can need a different fixed calibration
        # (e.g. derived from a known reference signal), independent of the operator's
        # live CISPR OFFSET.
        self.dbuv_calibration = 0.0
        # Effective offset actually applied to the right (CISPR) axis: OFFSET + CALIBRATION.
        self.cispr_effective_offset = 0.0
        # in __init__, after super().__init__() and before threads start
        self._psd_to_dbuv_const = 33.5   # default conversion constant (tuner path default)

        # ---- Stitching / edge-masking state ----
        # Copied from the module-level default so it can still be tuned per-instance if
        # ever needed, but the intent (per the user) is to adjust it in code, not via a
        # GUI control - see STITCH_EDGE_MASK_FRACTION above and stitch_get_edge_bins().
        self.stitch_edge_mask_fraction = STITCH_EDGE_MASK_FRACTION

        # ---- Time-averaging state ----
        # Mirrors the CISPR Offset pattern: a checkbox to switch it on/off, and a spinbox
        # for how many consecutive FFT captures (per frequency block) to average together.
        # Averaging is done in POWER (linear), never in dB - see avg_get_active_count()
        # and sweep_loop(). avg_count=1 or avg_enabled=False both mean "no averaging".
        self.avg_enabled = True
        self.avg_count = 20

        # ---- Artefact-correction state (DC offset removal / IQ balance correction) ----
        # Both target known direct-conversion-receiver artefacts that track center
        # frequency and sample rate rather than being real signals - see
        # _remove_dc_offset() / _correct_iq_imbalance() for what each one does and why.
        # DC Offset Removal defaults ON - it was found to be the far larger of the two
        # effects on this hardware. IQ Balance Correction still defaults off.
        # (previous: self.dc_removal_enabled = True / self.iq_balance_enabled = False)
        # Both now come from settings.csv (dc_offset_removal / iq_balance_correction) and default to ON;
        # there are no tick boxes for them on the front panel any more.
        self.dc_removal_enabled = DC_OFFSET_REMOVAL
        self.iq_balance_enabled = IQ_BALANCE_CORRECTION

        # Label of whichever preset/MODE is currently active, shown on the stable status
        # panel (see update_status_panel()). "-" until a MODE button has been pressed.
        self.current_mode_label = "-"


        # ---- previous driver selection block (hard-coded), kept in case of problems ----
#         # ---- Driver selection at startup ----
#         driver_dirs = {
#             "Default (MSVC RTL-SDR Blog V4)": DRIVER_ROOT + "/default",
#             "MS64 (libusb alt)": DRIVER_ROOT + "/ms64",
#             "SDRSharp drivers Copy": DRIVER_ROOT + "/sdrsharp",
#             "Experimental": DRIVER_ROOT + "/experimental"
#         }
#         if ENABLE_DRIVER_SELECTION:
#             items = list(driver_dirs.keys())
#             choice, ok = QtWidgets.QInputDialog.getItem(
#                 self,
#                 "Select RTL-SDR Driver",
#                 "Choose driver set:",
#                 items,
#                 0,
#                 False
#             )
#             if ok:
#                 self.driver_path = driver_dirs[choice]
#             else:
#                 self.driver_path = driver_dirs["Default (MSVC RTL-SDR Blog V4)"]
#         else:
#             # Deterministic default path when selection is disabled
#             self.driver_path = driver_dirs["Default (MSVC RTL-SDR Blog V4)"]

        # ---- Driver selection (settings.csv: driver / driver_root / driver_dialog) ----
        driver_dirs = {
            "Default (MSVC RTL-SDR Blog V4)": DRIVER_ROOT + "/default",
            "MS64 (libusb alt)": DRIVER_ROOT + "/ms64",
            "SDRSharp drivers Copy": DRIVER_ROOT + "/sdrsharp",
            "Experimental": DRIVER_ROOT + "/experimental"
        }
        self.driver_path, self._driver_note = self.resolve_driver_choice(DRIVER_CHOICE, driver_dirs)
        if ENABLE_DRIVER_SELECTION:
            items = list(driver_dirs.keys())
            if self.driver_path not in driver_dirs.values():
                items.append("Custom: " + os.path.basename(self.driver_path.rstrip("/")))
                driver_dirs[items[-1]] = self.driver_path
            current = next((i for i, k in enumerate(items) if driver_dirs[k] == self.driver_path), 0)
            choice, ok = QtWidgets.QInputDialog.getItem(
                self,
                "Select RTL-SDR Driver",
                "Choose driver set:",
                items,
                current,
                False
            )
            if ok:
                self.driver_path = driver_dirs[choice]
                self._driver_note = f"driver set '{choice}' (chosen at start-up)"

        # Ensure DLL path is active before importing the driver
        _add_dll_dir(self.driver_path)

        # ---- Window title ----
        self.setWindowTitle("EMC Sweep")

        # RTL-SDR handle
        self.sdr = None

        # ---- Current sweep profile (mutable) ----
        # These start as copies of the module-level defaults, but from here on they are
        # the live settings actually used to sweep. set_preset() updates these when a MODE
        # button is pressed; start_sweep()/start_continuous()/init_sdr()/sweep_loop() all
        # read self.START_FREQ/self.STOP_FREQ/self.SAMPLE_RATE/self.gain_db, never the
        # bare module-level constants, so a preset change actually takes effect.
        self.START_FREQ = START_FREQ
        self.STOP_FREQ = STOP_FREQ
        self.SAMPLE_RATE = SAMPLE_RATE
        # Which sample rate is actually configured on the hardware right now (set by
        # init_sdr() and by sweep_loop()'s live-rate check) - may lag self.SAMPLE_RATE
        # briefly after the Sample Rate combo box is changed while a sweep is running.
        # None until the SDR has actually been opened at least once.
        self._applied_sample_rate = None
        self.gain_db = GAIN_DB
        # Actual gain reported back by the device after the last init_sdr() call - may
        # differ from self.gain_db, since the driver silently rounds/clamps whatever we
        # request to the nearest value the hardware supports. None until a sweep has
        # actually opened the SDR at least once. See init_sdr() and update_status_panel().
        self.actual_gain_db = None
        # Which gain_db value actual_gain_db was actually measured against - i.e. what
        # self.gain_db was AT THE TIME init_sdr() applied it. Switching modes changes
        # self.gain_db immediately, but the hardware isn't reconfigured until the next
        # sweep actually starts, so actual_gain_db can be stale (left over from a
        # DIFFERENT mode) for a while after a MODE click. update_status_panel() only
        # trusts actual_gain_db when this still matches the current self.gain_db.
        self.actual_gain_db_for_request = None
        # Which gain value is actually configured on the hardware right now (set by
        # init_sdr() and apply_gain_if_changed()). The live RF Gain slider only changes
        # self.gain_db; the sweep thread notices the difference and applies it itself.
        self._applied_gain_db = None
        # Set (GUI thread) when something changes that invalidates the pass in progress -
        # currently a live sample-rate change. sweep_loop() abandons the current pass at
        # the next block boundary and starts a fresh one with the new settings.
        self._restart_pass = False
        # Completed-pass bookkeeping used by the gain/rate survey: sweep_loop() publishes a
        # snapshot of every pass that finished cleanly (all blocks stored, one sample rate,
        # one gain throughout) in self._last_pass, with an increasing sequence number.
        self._pass_seq = 0
        self._last_pass = None
        self._survey = None            # dict while a survey is running, else None
        self._survey_timer = QtCore.QTimer(self)
        self._survey_timer.setInterval(250)
        self._survey_timer.timeout.connect(self.survey_tick)

        # Sweep centers
        self.centers = self.build_centers(self.START_FREQ, self.STOP_FREQ, self.SAMPLE_RATE)
        # Diagnostic: show centers and expected block ranges (MHz)
        print("INIT Diagnostic: centers (MHz):", (self.centers / 1e6).tolist())
        block_width_hz = self.SAMPLE_RATE
        ranges = [(c - block_width_hz/2.0, c + block_width_hz/2.0) for c in self.centers]
        print("     Diagnostic: expected block ranges (MHz):",
              [f"{a/1e6:.6f}-{b/1e6:.6f}" for a, b in ranges])

        # ---- GUI ----
        cw = QtWidgets.QWidget()
        self.setCentralWidget(cw)

        # Main horizontal layout: left (debug + plot + controls), right (presets)
        main_layout = QtWidgets.QHBoxLayout(cw)

        # Left side layout
        left_layout = QtWidgets.QVBoxLayout()
        main_layout.addLayout(left_layout)

        # ---- Status panel: stable, non-scrolling display of the current scan settings ----
        # Unlike self.debug below (a scrolling log), this widget's text is always fully
        # REPLACED (setText), never appended to - so it never scrolls or "jitters". It's
        # refreshed by update_status_panel() whenever a setting actually changes (mode
        # selected, averaging toggled/changed) - not on every sweep block.
        self.status_panel = QtWidgets.QLabel()
        self.status_panel.setStyleSheet(
            "QLabel { background-color: #202020; color: #E0E0E0; padding: 4px; }"
        )
        self.status_panel.setFont(QtGui.QFont("Consolas", 9))
        left_layout.addWidget(self.status_panel)

        # ---- Debug console ----
        # Reserved for discrete lifecycle events (sweep started/stopped, mode changed) and
        # errors/warnings only - see the print() redirect further down, which keeps routine
        # per-block sweep diagnostics OUT of this window (console-only) so it stays stable
        # instead of scrolling on every FFT block.
        self.debug = QtWidgets.QPlainTextEdit()
        self.debug.setReadOnly(True)
        self.debug.setMaximumHeight(120)
        left_layout.addWidget(self.debug)

        # ---- Plot ----
        self.plot = pg.PlotWidget()
        self.plot.setLabel('bottom', 'Frequency', units='Hz')
        self.plot.setLabel('left', 'Level', units='dB')

        # main FFT curve (left axis)
        self.curve = self.plot.plot(pen='y')

        # Reference trace (e.g. captured "system noise" baseline) - display only, on the
        # same left (dB) axis as the live curve. Never subtracted from the live trace;
        # it just sits underneath it so you can visually compare the two. Light blue so
        # it reads as a background reference rather than competing with the live yellow
        # trace. Hidden until Capture Reference is pressed.
        self.reference_curve = self.plot.plot(pen=pg.mkPen(color=(173, 216, 230), width=1), name="Reference")
        self.reference_curve.hide()
        self.reference_freq = None
        self.reference_power = None

        # ---- Right-hand CISPR axis using a separate ViewBox (safe) ----
        # This creates an independent right ViewBox so the right axis can show CISPR units
        # while the main (left) axis remains the FFT dB scale.
        self.plot.showAxis('right')
        self.plot.getAxis('right').setLabel('CISPR (dBµV)')
        self.plot.getAxis('right').setPen(pg.mkPen(color='red', width=2))

        # create dedicated right ViewBox and add it to the scene
        self._vb_right = pg.ViewBox()
        self.plot.scene().addItem(self._vb_right)

        # link the right axis to the new ViewBox and keep X linked to main view
        self.plot.getAxis('right').linkToView(self._vb_right)
        self._vb_right.setXLink(self.plot.getViewBox())

        # prevent the right VB from stealing mouse events and keep it behind the main VB
        self._vb_right.setMouseEnabled(x=False, y=False)
        try:
            self._vb_right.setZValue(-100)
            self.plot.getViewBox().setZValue(0)
        except Exception:
            pass

        # CISPR curve lives in the right-hand ViewBox and is expressed in CISPR units (no offset)
        self.cispr_curve = pg.PlotCurveItem(
            pen=pg.mkPen(color='red', style=QtCore.Qt.DashLine, width=2),
            name="CISPR"
        )
        self._vb_right.addItem(self.cispr_curve)
        self.cispr_curve.hide()

        # Keep the right VB's screen geometry glued to the main viewbox (runs on resize),
        # and keep the CISPR mapping/ticks following the left axis (runs on pan/zoom).
        # NOTE: previously this connected self._on_view_range_changed to sigRangeChanged
        # TWICE (once here, once again further down in __init__), so every pan/zoom
        # redrew the CISPR line and ticks twice in a row. That double-fire, combined with
        # update_cispr_curve() also being invoked from the worker thread (see update_plot()),
        # is what made the line "jump" relative to its own axis. Both are fixed by routing
        # everything through the cispr_* methods below and connecting each signal exactly once.
        try:
            self.plot.getViewBox().sigResized.connect(self.cispr_sync_viewbox_geometry)
        except Exception:
            pass
        self.cispr_sync_viewbox_geometry()

        try:
            self.plot.getViewBox().sigRangeChanged.connect(self.cispr_on_view_range_changed)
        except Exception:
            pass
        # -----------------------------------------------------------------

        left_layout.addWidget(self.plot)


        # ---- Fixed Y-axis (no autoscale) ----
        self._left_ymin = -120.0
        self._left_ymax = 20.0
        self.plot.setYRange(self._left_ymin, self._left_ymax)
        self.plot.enableAutoRange(axis=pg.ViewBox.XAxis, enable=True)
        self.plot.enableAutoRange(axis=pg.ViewBox.YAxis, enable=False)

        # ---- Mouse cursor readout ----
        self.mouse_text = pg.TextItem("", anchor=(0, 1))
        self.plot.addItem(self.mouse_text)

        def mouse_info(evt):
            vb = self.plot.getViewBox()
            if vb is None:
                return
            pos = vb.mapSceneToView(evt)
            freq = pos.x()
            level = pos.y()
            self.mouse_text.setText(f"{freq:,.0f} Hz\n{level:.1f} dB")
            self.mouse_text.setPos(freq, level)

        self.plot.scene().sigMouseMoved.connect(mouse_info)

        self.plot_signals = PlotSignals()
        self.plot_signals.update.connect(self.set_plot_data)
        self.plot_signals.markers_changed.connect(self.draw_block_markers)

        # Add this in __init__ after creating self.plot
        self._block_markers = []   # list of (line_left, line_right) or shaded items

        # ---- Control buttons ----
        ctrl_layout = QtWidgets.QHBoxLayout()
        left_layout.addLayout(ctrl_layout)

        self.btn_start = QtWidgets.QPushButton("START SINGLE SWEEP")
        self.btn_stop = QtWidgets.QPushButton("STOP")
        self.btn_save = QtWidgets.QPushButton("SAVE CSV")
        self.chk_cispr = QtWidgets.QCheckBox("Show CISPR limit")

        self.btn_cont = QtWidgets.QPushButton("START CONTINUOUS")
        ctrl_layout.addWidget(self.btn_cont)
        self.btn_cont.clicked.connect(self.start_continuous)

        ctrl_layout.addWidget(self.btn_start)
        ctrl_layout.addWidget(self.btn_stop)
        ctrl_layout.addWidget(self.btn_save)
        ctrl_layout.addWidget(self.chk_cispr)

        self.btn_start.clicked.connect(self.start_sweep)
        self.btn_stop.clicked.connect(self.stop_sweep)
        self.btn_save.clicked.connect(self.save_csv)
        self.chk_cispr.toggled.connect(self.cispr_toggle_visibility)

        # ---- Live Sample Rate control ----
        # Independent of the MODE preset buttons: changing this does NOT stop a running
        # sweep. It updates self.SAMPLE_RATE immediately; sweep_loop() (worker thread)
        # picks the new value up and applies it to the hardware itself at the start of
        # its next full sweep pass - see sweep_loop() for why it's done there rather than
        # here. Intended for quickly cycling sample rates to check whether a suspect
        # spike/signal moves (artefact) or stays put (real) - see SAMPLE_RATE_CHOICES.
        self.sample_rate_label = QtWidgets.QLabel("Sample Rate")
        self.sample_rate_combo = QtWidgets.QComboBox()
        for rate_hz in SAMPLE_RATE_CHOICES:
            # (previous: addItem(f"{rate_hz/1e6:.3f} MS/s", rate_hz))
            rec_ = any(abs(rate_hz - r) < 1.0 for r in SAMPLE_RATE_RECOMMENDED)
            self.sample_rate_combo.addItem(f"{rate_hz/1e6:.3f} MS/s" + ("" if rec_ else " (not recommended)"), rate_hz)
        self.sample_rate_combo.setToolTip(
            "Change the SDR sample rate live, without stopping a running sweep - useful "
            "for checking whether a spike shifts with sample rate (artefact) or stays put "
            "(real signal). Takes effect immediately: the pass in progress is abandoned "
            "at the next block and a fresh pass starts at the new rate."
        )

        # ---- Second control row: Sample Rate + live RF Gain slider ----
        ctrl_layout_rf = QtWidgets.QHBoxLayout()
        left_layout.addLayout(ctrl_layout_rf)

        # ---- Live RF Gain slider ----
        # Steps through the gains the tuner really supports (RTL_GAIN_STEPS_DB). Changing
        # it only updates self.gain_db; the sweep thread applies it to the hardware at the
        # next block boundary (the RTL-SDR driver isn't safe to call from two threads), so
        # the effect is immediate without stopping the sweep. If nothing is running, it is
        # applied when the next sweep starts (init_sdr()).
        self.gain_label = QtWidgets.QLabel("RF Gain")
        self.gain_slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.gain_slider.setRange(0, len(RTL_GAIN_STEPS_DB) - 1)
        self.gain_slider.setSingleStep(1)
        self.gain_slider.setPageStep(1)
        self.gain_slider.setTickPosition(QtWidgets.QSlider.TicksBelow)
        self.gain_slider.setTickInterval(1)
        self.gain_slider.setMinimumWidth(240)
        self.gain_value_label = QtWidgets.QLabel("")
        self.gain_value_label.setMinimumWidth(70)
        self.gain_slider.setToolTip(
            "RF (tuner) gain in dB. Steps through the gains the RTL-SDR supports. Applied "
            "to the hardware at the next sweep block, without stopping the sweep. Picking "
            "a MODE button sets the slider to that mode's gain."
        )

        ctrl_layout_rf.addWidget(self.sample_rate_label)
        ctrl_layout_rf.addWidget(self.sample_rate_combo)
        ctrl_layout_rf.addSpacing(24)
        ctrl_layout_rf.addWidget(self.gain_label)
        ctrl_layout_rf.addWidget(self.gain_slider, 1)
        ctrl_layout_rf.addWidget(self.gain_value_label)
        ctrl_layout_rf.addSpacing(16)
        self.btn_survey = QtWidgets.QPushButton("GAIN/RATE SURVEY...")
        self.btn_survey.setToolTip(
            "Automatically step through sample rates and gains on the current band, "
            "recording the spikes found at each combination to CSV files for analysis. "
            "Only available for bands up to 1.2 MHz, at sample rates that cover the whole band "
            "in a single block. Press STOP to end early (partial results are still saved).")
        self.btn_survey.clicked.connect(self.survey_open_dialog)
        ctrl_layout_rf.addWidget(self.btn_survey)

        self.sample_rate_combo.currentIndexChanged.connect(self.sample_rate_on_changed)
        self.gain_slider.valueChanged.connect(self.gain_slider_on_changed)
        self.sync_gain_slider()

        # ---- CISPR Offset Control (user calibration) ----
        # This spinbox holds a single value that persists across MODE changes - it is the
        # operator's own live calibration knob, not tied to any one preset. The separate
        # dBuV_Calibration value (set per-mode from self.presets) is shown alongside it and
        # added to it; see cispr_get_effective_offset().
        self.cispr_offset_label = QtWidgets.QLabel("CISPR Offset")
        self.cispr_offset_spin = QtWidgets.QDoubleSpinBox()
        self.cispr_offset_spin.setRange(-200.0, 200.0)
        self.cispr_offset_spin.setSingleStep(1.0)
        self.cispr_offset_spin.setDecimals(1)
        self.cispr_offset_spin.setValue(self.cispr_offset_db)   # initial value from __init__
        self.cispr_offset_spin.setSuffix(" dB")
        self.cispr_offset_spin.setToolTip("Visual offset to align CISPR (accounts for antenna/attenuation). "
                                           "Persists across MODE changes.")

        # Read-only display of the current mode's dBuV_Calibration, kept in sync by
        # cispr_refresh_offset_ui() whenever set_preset() changes mode.
        self.cispr_calibration_label = QtWidgets.QLabel(f"Cal: {self.dbuv_calibration:+.1f} dB")
        self.cispr_calibration_label.setToolTip("Mode-dependent dBuV calibration, added to CISPR Offset "
                                                 "(set per MODE in self.presets).")

        # place the controls in the same control row
        ctrl_layout.addWidget(self.cispr_offset_label)
        ctrl_layout.addWidget(self.cispr_offset_spin)
        ctrl_layout.addWidget(self.cispr_calibration_label)

        # ---- Time-Averaging Control ----
        # Same pattern as the CISPR Offset control: a checkbox to switch the feature on/off,
        # plus a spinbox for how many consecutive FFT captures (per frequency block) are
        # averaged together in power before conversion to dB. Reduces trace noise at the
        # cost of a slower sweep (avg_count reads per hop instead of one).
        self.chk_avg = QtWidgets.QCheckBox("Time Averaging")
        self.avg_count_spin = QtWidgets.QSpinBox()
        self.avg_count_spin.setRange(1, 150)
        self.avg_count_spin.setValue(self.avg_count)
        self.avg_count_spin.setSuffix(" avgs")
        self.avg_count_spin.setToolTip(
            "Number of consecutive FFT captures averaged together (in power, not dB) per "
            "frequency block before moving to the next hop. Higher = smoother trace, slower "
            "sweep. Only applied while Time Averaging is checked."
        )
        ctrl_layout.addWidget(self.chk_avg)
        ctrl_layout.addWidget(self.avg_count_spin)
        self.chk_avg.toggled.connect(self.avg_on_toggle)
        self.avg_count_spin.valueChanged.connect(self.avg_on_count_changed)

        # ---- Display-toggle row (markers, reference trace) ----
        # A second control row, kept separate from the sweep/CISPR/averaging controls above
        # purely to stop the first row from growing indefinitely wide.
        ctrl_layout2 = QtWidgets.QHBoxLayout()
        left_layout.addLayout(ctrl_layout2)

        # Frequency block markers (green = raw captured block edges, cyan dashed = kept/
        # displayed edges after edge masking - see draw_block_markers()). Useful for
        # confirming the stitching, but visually intrusive once you're done checking it,
        # so they can be switched off without discarding/rebuilding them.
        self.markers_visible = True
        self.chk_markers = QtWidgets.QCheckBox("Show Freq Markers")
        self.chk_markers.setChecked(self.markers_visible)
        self.chk_markers.setToolTip("Show/hide the green (raw block) and cyan dashed "
                                     "(kept/stitched) vertical frequency markers.")
        ctrl_layout2.addWidget(self.chk_markers)
        self.chk_markers.toggled.connect(self.markers_toggle_visibility)

        # Reference trace: a one-off snapshot of whatever is currently on the live curve
        # (e.g. captured with the target system switched off, as a "system noise" baseline),
        # kept on screen in light blue for visual comparison. Never subtracted from the
        # live trace - see reference_capture().
        self.btn_capture_ref = QtWidgets.QPushButton("Capture Reference")
        self.chk_show_ref = QtWidgets.QCheckBox("Show Reference")
        self.btn_capture_ref.setToolTip("Snapshot the current live trace as a reference "
                                         "(e.g. system-noise baseline) for visual comparison only.")
        ctrl_layout2.addWidget(self.btn_capture_ref)
        ctrl_layout2.addWidget(self.chk_show_ref)
        self.btn_capture_ref.clicked.connect(self.reference_capture)
        self.chk_show_ref.toggled.connect(self.reference_toggle_visibility)

        # ---- DC Offset Removal / IQ Balance Correction: no front-panel tick boxes any more ----
        # Both default ON and are set from settings.csv (dc_offset_removal / iq_balance_correction); the
        # old tick boxes are kept below, commented out, in case of problems.

#         # ---- Signal-processing toggle row (artefact correction) ----
#         # DC Offset Removal and IQ Balance Correction each target a specific
#         # direct-conversion-receiver artefact that tracks center frequency / sample rate
#         # rather than being a real signal - see _remove_dc_offset() / _correct_iq_imbalance().
#         # DC Offset Removal defaults ON (confirmed to be the larger effect on this
#         # hardware); IQ Balance Correction still defaults off - switch it on to see
#         # whether it accounts for any remaining spike once DC removal is active.
#         ctrl_layout3 = QtWidgets.QHBoxLayout()
#         left_layout.addLayout(ctrl_layout3)
#         self.chk_dc_removal = QtWidgets.QCheckBox("DC Offset Removal")
#         self.chk_dc_removal.setChecked(self.dc_removal_enabled)
#         self.chk_dc_removal.setToolTip(
#             "Subtract each block's own mean IQ value before windowing/FFT. Targets the "
#             "repeated spike at each block's center frequency caused by LO leakage / ADC "
#             "DC bias - not a real signal if it moves when you change range or sample rate."
#         )
#         ctrl_layout3.addWidget(self.chk_dc_removal)
#         self.chk_dc_removal.toggled.connect(self.dc_removal_on_toggle)
#         self.chk_iq_balance = QtWidgets.QCheckBox("IQ Balance Correction")
#         self.chk_iq_balance.setChecked(self.iq_balance_enabled)
#         self.chk_iq_balance.setToolTip(
#             "Blind per-block correction of I/Q gain and phase mismatch. Targets a mirrored "
#             "spike reflected around each block's center frequency, caused by receiver I/Q "
#             "imbalance rather than a real signal."
#         )
#         ctrl_layout3.addWidget(self.chk_iq_balance)
#         self.chk_iq_balance.toggled.connect(self.iq_balance_on_toggle)

        # ---- Sweep state ----
        self.stop_flag = threading.Event()
        self.sweep_thread = None

        # Initialize freq/power arrays early so startup calls are safe
        self.freq_axis = np.array([], dtype=np.float64)
        self.power_axis = np.array([], dtype=np.float64)

        # connect to handler that stores the offset for the current mode and redraws the curve
        self.cispr_offset_spin.valueChanged.connect(self.cispr_on_offset_changed)
        # Force an initial update so the curve/label reflect the current mode+spinbox value
        # immediately (safe because freq_axis/power_axis exist)
        self.cispr_refresh_offset_ui()
        # Ensure initial visibility follows the checkbox (do not force show unconditionally)
        self.cispr_toggle_visibility(bool(getattr(self, "chk_cispr", None) and self.chk_cispr.isChecked()))
        # ---- Right-hand preset panel ----
        preset_panel = QtWidgets.QVBoxLayout()
        main_layout.addLayout(preset_panel)

        # Define presets as a list so we can index them and extend easily
        # Each entry: (internal_name, label, start_hz, stop_hz, sample_rate, gain_db, hf_mode_flag,
        #              dbuv_calibration)
        # dbuv_calibration is a fixed, mode-specific dB value (e.g. from comparing a known
        # reference signal's true dBuV level against what this mode reads). It is added to
        # the operator's live CISPR Offset spinbox value - see cispr_get_effective_offset().
        # All zero for now; fill these in as calibration data becomes available per mode.
        # Supported gains: [0, 9, 14, 27, 37, 77, 87, 125, 144, 157, 166, 197, 207, 229, 254,
        # 280, 297, 328, 338, 364, 372, 386, 402, 421, 434, 439, 445, 480, 496] (tenths of dB -
        # e.g. 402 -> 40.2 dB). gain_db values below are requested dB; init_sdr() reads back
        # and logs/warns if the device actually applied something different - see gain_db.
        # generally avoid HF mode as it has no gain.
        # Presets now come from settings.csv (see load_settings() near the top of this file;
        # the original hard-coded list is kept there as _FALLBACK_PRESETS).
        self.presets = list(PRESETS)

        # Create buttons in a loop so adding presets is trivial
        self.mode_buttons = []
        for idx, (_, label, *_rest) in enumerate(self.presets):
            btn = QtWidgets.QPushButton(label)
            self.mode_buttons.append(btn)
            preset_panel.addWidget(btn)
            # bind the index into the lambda to avoid late-binding trap
            # previous: btn.clicked.connect(lambda _, i=idx: self.set_preset(i))
            btn.clicked.connect(lambda _, i=idx: self.set_preset_and_run(i))   # set the preset, then start CONTINUOUS

        preset_panel.addStretch()

        # ---- CISPR curve data
        self.cispr_freqs, self.cispr_limits = self.cispr_build_reference_curve()
        print("DEBUG: cispr canonical assigned; len(freqs)=", np.size(self.cispr_freqs),
            "len(limits)=", np.size(self.cispr_limits),
            "checksum=", float(np.sum(np.round(np.asarray(self.cispr_limits, dtype=np.float64),3))))

        # ---- Redirect print() to debug window ----
        # sweep_loop() (and friends) print a LOT of routine per-block diagnostics (tuning,
        # checksums, peaks, etc.) - useful in the real console, but appending every one of
        # them to the GUI debug window is exactly what made it feel "jittery"/constantly
        # scrolling. So: everything still goes to the real console as before, but only
        # messages that look like an error or warning are also forwarded to the GUI window.
        # Routine diagnostics stay console-only; the debug window now only ever gets
        # discrete lifecycle events (logged directly via self.log(), e.g. sweep started/
        # stopped, mode changed) plus genuine errors/warnings - exactly the "stable unless
        # something goes wrong" behaviour asked for.
        import builtins
        real_print = builtins.print

        def gui_print(*args, **kwargs):
            text = " ".join(str(a) for a in args)
            real_print(*args, **kwargs)
            lowered = text.lower()
            if "error" in lowered or "warning" in lowered:
                self.log(text)

        builtins.print = gui_print

        # ---- Final startup log ----
        self.log(f"Using driver path: {self.driver_path}")
        if getattr(self, "_driver_note", ""):
            self.log(f"Driver: {self._driver_note}")
        for _p in self.presets:
            if not any(abs(_p[4] - r) < 1.0 for r in SAMPLE_RATE_RECOMMENDED):
                self.log(f"WARNING: preset '{_p[1]}' (settings.csv) uses {_p[4]/1e6:.3f} MS/s, which is not one of the recommended "
                         f"sample rates ({', '.join(f'{r/1e6:g}' for r in SAMPLE_RATE_RECOMMENDED)} MS/s)")
        self.log(f"DC offset removal: {'ON' if self.dc_removal_enabled else 'OFF'}, "
                 f"IQ balance correction: {'ON' if self.iq_balance_enabled else 'OFF'} (settings.csv)")
        self.log("Setup complete")
        self.update_status_panel()   # show initial (module-default) settings until a MODE is picked
        self.sync_sample_rate_combo()
        self.scope_build_ui(left_layout)   # IQ / ENVELOPE SCOPE: second display, hidden until its button is pressed
        self.scope_build_trigger_ui()      # sweep mode (normal/auto) + trigger housekeeping
        self.scope_build_pulse_ui()        # pulse marking / rejection (diagnosing stream glitches)
        self.scope_tame_status_label()     # scope readout must never widen the program window
        self.survey_connect_signals()      # survey runs on the worker thread and reports back by signal
        self.stitch_build_ui(left_layout)  # STITCH CHECK button: is a feature real or made by the block layout?
        self.single_build_ui(left_layout)  # SINGLE FREQUENCY mode: button above the ranges + digit selector under the gain row



    def resolve_driver_choice(self, choice, driver_dirs):
        """
        Map the settings.csv "driver" value to a driver folder. Accepts default / ms64 / sdrsharp /
        experimental, one of the menu names, a folder name under the driver root (DRIVER_ROOT, or the
        "driver_root" setting), or a full path. Returns (folder, note). An unknown choice falls back to
        the default driver set, with a WARNING in the note (shown in the debug window).
        """
        names = {"default": "Default (MSVC RTL-SDR Blog V4)", "ms64": "MS64 (libusb alt)",
                 "sdrsharp": "SDRSharp drivers Copy", "experimental": "Experimental"}
        raw = (choice or "default").strip()
        key = raw.lower()
        if key in names:
            return driver_dirs[names[key]], f"driver set '{raw}' (settings.csv)"
        for label, path in driver_dirs.items():
            if key == label.lower():
                return path, f"driver set '{label}' (settings.csv)"
        cand = raw.replace("\\", "/")
        for p in (cand, DRIVER_ROOT + "/" + cand):
            if os.path.isdir(p):
                return p.replace("\\", "/"), f"driver folder {p} (settings.csv)"
        return (driver_dirs[names["default"]],
                f"WARNING: driver '{raw}' from settings.csv not found (looked for that folder and in {DRIVER_ROOT}) "
                f"- using the default driver set")

    def init_sdr(self):
        # --- Always close any previous handle ---
        if self.sdr is not None:
            try:
                self.sdr.close()
                print("Closed previous SDR handle")
            except Exception:
                pass
            self.sdr = None
            
        # Ensure selected driver path is active
        _add_dll_dir(self.driver_path)
        
        # Import driver AFTER DLL path is set
        from pyrtlsdr_local.rtlsdr import RtlSdr
        
        # --- Open new handle ---
        print("Initializing RTL-SDR device...")
        self.sdr = RtlSdr()
            
    # Diagnostic prints must use self.sdr\py
        print("Opening device…")
        print("Tuner type:", self.sdr.get_tuner_type())
        print("Supported gains:", self.sdr.get_gains())

       # self.sdr.center_freq = 100e6
       # self.sdr.sample_rate = 2.4e6
       # self.sdr.gain = 0

        print("Reading samples…")
        samples = self.sdr.read_samples(256)
        print("Sample[0:10]:", samples[:10])

        # DO NOT close here — this is your main SDR handle
        # self.sdr.close()
        # Always reapply parameters - use the LIVE preset values (self.SAMPLE_RATE /
        # self.gain_db), which set_preset() updates when a MODE button is pressed.
        # Using the module-level SAMPLE_RATE/GAIN_DB constants here would silently ignore
        # whichever mode the user last selected.
        self.sdr.sample_rate = self.SAMPLE_RATE
        self._applied_sample_rate = self.SAMPLE_RATE   # matches sweep_loop()'s live-rate check
        self.sdr.set_agc_mode(False)
        self.sdr.gain = self.gain_db
        self._applied_gain_db = self.gain_db   # matches apply_gain_if_changed()'s check
        # IMPORTANT: the driver snaps whatever dB value we ask for to the nearest value
        # the hardware actually supports (the "Supported gains" list printed above, in
        # TENTHS of dB - e.g. 402 there means 40.2 dB) - it does NOT treat the number we
        # pass in as an index into that table, and it does not clip/error if we ask for
        # more than the hardware can do; it just silently gives us its maximum instead.
        # Reading back the actual applied gain (rather than trusting self.gain_db) is the
        # only way to know what really happened - see update_status_panel().
        self.actual_gain_db = float(self.sdr.get_gain())
        # Record which request this reading actually corresponds to, so a later MODE
        # switch (which changes self.gain_db immediately, before the hardware is touched
        # again) can't make update_status_panel() compare a NEW request against this OLD
        # actual value and report a false mismatch.
        self.actual_gain_db_for_request = self.gain_db
        print(f"Gain requested: {self.gain_db} dB -> actual: {self.actual_gain_db} dB")
        if abs(self.actual_gain_db - self.gain_db) > 0.5:
            print(f"WARNING: requested gain {self.gain_db} dB was not available; "
                  f"the device is actually using {self.actual_gain_db} dB "
                  f"(see 'Supported gains' above, in tenths of dB, for the full list)")
        self.update_status_panel()
            
        # HF mode for RTL-SDR V4
        try:
            if self.hf_mode:
                self.sdr.set_direct_sampling(2)   # HF input
                print("HF mode enabled (direct sampling input 2)")
            else:
                self.sdr.set_direct_sampling(0)   # tuner mode
            print("VHF/UHF tuner mode enabled")
        except Exception as e:
            print("Error setting sampling mode:", e)    

    def set_preset(self, preset_index):
        """
        Apply a MODE button's settings: stop any in-progress sweep, then store the new
        start/stop/sample-rate/gain/hf_mode/dbuv_calibration as the LIVE settings that
        start_sweep(), start_continuous(), init_sdr(), sweep_loop() and the CISPR mapping
        all read (self.START_FREQ, self.STOP_FREQ, self.SAMPLE_RATE, self.gain_db,
        self.hf_mode, self.dbuv_calibration). The SDR itself isn't reopened here -
        init_sdr() re-applies these values to the hardware the next time START SWEEP /
        START CONTINUOUS is pressed, so browsing between modes doesn't repeatedly
        open/close the device. This method does not start a new sweep; it only stops the
        old one and gets the new settings ready.

        Note: self.cispr_offset_db (the operator's CISPR Offset spinbox) is deliberately
        NOT touched here - it must persist unchanged across mode changes.
        """
        try:
            if preset_index < 0 or preset_index >= len(self.presets):
                print("set_preset: invalid preset index", preset_index)
                return

            # Stop any sweep in progress so it doesn't keep running (and racing self.sdr)
            # under the old settings while we switch to the new ones. Give the worker
            # thread a moment to actually exit before we hand off new parameters, the
            # same way closeEvent() does.
            if self.sweep_thread and self.sweep_thread.is_alive():
                self.log("=== MODE CHANGED: stopping current sweep ===")
                self.stop_flag.set()
                self.sweep_thread.join(timeout=1.0)

            # Unpack preset tuple: (internal_name, label, start_hz, stop_hz, sample_rate,
            # gain_db, hf_mode_flag, dbuv_calibration)
            _, _label, start_hz, stop_hz, sample_rate, gain_db, hf_flag, dbuv_cal = self.presets[preset_index]

            # Store preset values as the LIVE settings for other code to use
            self.START_FREQ = float(start_hz)
            self.STOP_FREQ = float(stop_hz)
            self.SAMPLE_RATE = float(sample_rate)
            self.sync_sample_rate_combo()

            # Preserve CISPR calibration; do not overwrite it with SDR preset gain.
            # Store SDR/preset gain separately so it cannot change CISPR mapping accidentally.
            self.gain_db = float(gain_db)
            self.sync_gain_slider()   # slider follows the mode's gain

            # HF mode flag (affects SDR path and PSD constant only)
            self.hf_mode = bool(hf_flag)

            # Mode-dependent dBuV calibration - added to the operator's CISPR Offset
            # spinbox value (which is NOT touched here; see cispr_refresh_offset_ui()).
            self.dbuv_calibration = float(dbuv_cal)

            # Recompute the sweep centers and block markers for the new range/sample rate
            # so the plot's markers and the next sweep agree on the new preset immediately,
            # without waiting for START SWEEP / START CONTINUOUS to rebuild them.
            if self.START_FREQ == self.STOP_FREQ:
                self.centers = np.array([self.START_FREQ], dtype=np.float64)
            else:
                self.centers = self.build_centers(self.START_FREQ, self.STOP_FREQ, self.SAMPLE_RATE)
            try:
                self.draw_block_markers()
            except Exception:
                pass

            self.log(f"MODE set: {_label}  {self.START_FREQ/1e6:.3f}-{self.STOP_FREQ/1e6:.3f} MHz, "
                     f"{self.SAMPLE_RATE/1e6:.3f} MS/s, gain {self.gain_db}, HF={self.hf_mode}, "
                     f"dBuV_Calibration={self.dbuv_calibration:+.1f} dB")

            # Reflect the new mode's calibration in the CISPR display and redraw the line.
            self.cispr_refresh_offset_ui()

            # Refresh the stable status panel (Start/Stop/Sample Rate/Gain/Averaging) -
            # this is the ONLY per-mode "log-like" update that matters for the operator
            # once running, so it gets the stable panel rather than another scrolling line.
            self.current_mode_label = _label
            self.update_status_panel()

        except Exception as e:
            print("set_preset: unexpected error:", e)


    # =====================================================================
    # CISPR (right-hand axis) support
    #
    # All the logic for the red CISPR reference line and its independent
    # right-hand ViewBox/axis lives in this one block, grouped in the order
    # things actually happen:
    #   1. cispr_sync_viewbox_geometry  - keep the right VB's screen rect glued
    #                                     to the main plot (runs on resize)
    #   2. cispr_on_view_range_changed  - react to the user panning/zooming the
    #                                     left (FFT) axis
    #   3. cispr_get_effective_offset   - CISPR Offset (persistent) + dBuV_Calibration (per mode)
    #   4. cispr_on_offset_changed      - spinbox -> stored per-mode offset
    #   5. cispr_refresh_offset_ui      - mode changed -> update spinbox/label
    #   6. cispr_update_curve           - the single place that (re)draws the
    #                                     red line and re-ranges the right VB
    #   7. cispr_update_axis_ticks      - relabel the right axis to match
    #   8. cispr_build_reference_curve  - the static CISPR limit table (dBuV)
    #   9. cispr_toggle_visibility      - show/hide only, never remaps anything
    #
    # Threading note: cispr_update_curve() and cispr_update_axis_ticks() touch
    # Qt scene items directly (no signal marshalling), so they must only ever
    # be called on the GUI thread. Every entry point above is a Qt slot
    # (spinbox/checkbox/button signal) or is called from set_plot_data(), which
    # itself only runs as a queued slot on self.plot_signals.update - i.e. on
    # the GUI thread, once per FFT block, after that block's data is set.
    # sweep_loop()/update_plot() run on the worker thread and must NOT call
    # into this block directly; that was the root cause of the line jumping
    # unpredictably relative to its own axis.
    # =====================================================================

    def cispr_sync_viewbox_geometry(self):
        """Keep the right-hand ViewBox's screen rectangle matching the main plot."""
        try:
            vb_main = self.plot.getViewBox()
            rect = vb_main.sceneBoundingRect()
            self._vb_right.setGeometry(rect)
            try:
                self._vb_right.linkedViewChanged(vb_main, self._vb_right.XAxis)
            except Exception:
                pass
        except Exception:
            pass

    def cispr_on_view_range_changed(self, *args):
        """
        Called when the main (left/FFT) view range changes (mouse wheel, pan, zoom).
        Re-run the CISPR mapping and right-axis ticks so the right axis keeps
        following the left view. Both calls are safe to run every time; each is
        idempotent and connected exactly once (see __init__).
        """
        self.cispr_update_curve()
        self.cispr_update_axis_ticks()

    def cispr_get_effective_offset(self):
        """
        The actual offset applied to the right (CISPR) axis: the operator's persistent
        CISPR Offset spinbox value, plus this mode's fixed dBuV_Calibration.
        """
        return float(getattr(self, "cispr_offset_db", 0.0)) + float(getattr(self, "dbuv_calibration", 0.0))

    def cispr_on_offset_changed(self, val):
        """
        Spinbox -> model. This value is deliberately global (not keyed by mode) so it
        persists across MODE button presses; only dbuv_calibration changes with mode.
        Redraws the curve AND the axis ticks - the ticks must be refreshed here too,
        since they depend on _vb_right's Y range, which cispr_update_curve() just changed.
        """
        self.cispr_offset_db = float(val)
        self.cispr_effective_offset = self.cispr_get_effective_offset()
        self.cispr_update_curve()
        self.cispr_update_axis_ticks()

    def cispr_refresh_offset_ui(self):
        """
        Call this whenever the active mode changes (set_preset()). The CISPR Offset
        spinbox is NOT touched here - it is the operator's persistent value and must
        survive mode changes unchanged. Only the mode-dependent calibration display and
        the resulting curve/ticks are refreshed.
        """
        try:
            self.cispr_calibration_label.setText(f"Cal: {self.dbuv_calibration:+.1f} dB")
        except Exception:
            pass
        self.cispr_effective_offset = self.cispr_get_effective_offset()
        self.cispr_update_curve()
        self.cispr_update_axis_ticks()

    def cispr_update_curve(self):
        """
        Draw the CISPR limit line in the right-hand ViewBox (CISPR units), then
        range the right ViewBox so it lines up with the left (FFT) axis at the
        current effective offset (= CISPR Offset + dBuV_Calibration).

        The right axis is meant to show the SAME range as the left axis, shifted
        up/down by the effective offset. So a CISPR value L is drawn at the
        screen height where the left axis reads (L + effective); equivalently,
        the right ViewBox's Y range is the left ViewBox's Y range shifted by
        -effective. Both the curve data and the axis range are computed from
        the same "effective" value in this one function.
        """
        # X axis to draw on: real stitched frequency data if we have it, else the
        # current view's X range (so the line still appears before a sweep runs).
        if getattr(self, "freq_axis", None) is None or getattr(self.freq_axis, "size", 0) == 0:
            try:
                vb = self.plot.getViewBox()
                xr = vb.viewRange()[0]
                fa = np.linspace(float(xr[0]), float(xr[1]), 512, dtype=np.float64)
            except Exception:
                return
        else:
            fa = np.asarray(self.freq_axis, dtype=np.float64)

        if not hasattr(self, "cispr_freqs") or not hasattr(self, "cispr_limits"):
            return

        try:
            freqs = np.asarray(self.cispr_freqs, dtype=np.float64)
            limits = np.asarray(self.cispr_limits, dtype=np.float64)
            L_interp = np.interp(fa, freqs, limits)
        except Exception:
            return

        # Effective offset = operator's CISPR Offset + this mode's dBuV_Calibration.
        effective = self.cispr_get_effective_offset()
        self.cispr_effective_offset = effective

        # Plot the CISPR curve in CISPR units (no offset applied to the data itself -
        # the offset is expressed entirely as a shift of the right ViewBox's range below).
        try:
            self.cispr_curve.setData(fa, L_interp)
        except Exception as e:
            print("cispr_update_curve: setData error:", e)

        # Visibility is controlled by cispr_toggle_visibility(); just mirror the checkbox here.
        try:
            if hasattr(self, "chk_cispr"):
                self.cispr_curve.setVisible(bool(self.chk_cispr.isChecked()))
        except Exception:
            pass

        # Shift the right ViewBox Y range so a CISPR value L lines up with left value (L + effective).
        try:
            vb_main = self.plot.getViewBox()
            vb_right = getattr(self, "_vb_right", None)
            if vb_right is not None and vb_main is not None:
                yr = vb_main.viewRange()[1]   # live left-axis [ymin, ymax]
                left_ymin, left_ymax = float(yr[0]), float(yr[1])
                vb_right.setYRange(left_ymin - effective, left_ymax - effective, padding=0)
        except Exception:
            pass

    def cispr_update_axis_ticks(self):
        """
        Recompute right-axis tick positions/labels from the right-hand (CISPR) ViewBox's
        OWN current Y range - not from the left/FFT axis.

        The right axis is linked to self._vb_right (linkToView), and the CISPR curve is
        plotted directly in raw CISPR units with no offset applied to the data itself
        (see cispr_update_curve). cispr_update_curve() is the ONLY place the offset is
        applied, by shifting _vb_right's Y range relative to the left axis's Y range.
        That means tick VALUES here must already be plain CISPR units (dBuV) - no
        further offset subtraction. Previously this method built tick positions from the
        LEFT axis's range and then subtracted the offset from the labels, which applied
        the offset a second time on top of the ViewBox shift; the two disagreed as soon
        as the offset was non-zero, which is what made the CISPR line appear to move
        relative to its own axis whenever the offset was changed. With this fix, changing
        the offset only ever moves the CISPR axis relative to the FFT axis - the line
        always reads correctly against the CISPR axis's own values.
        """
        try:
            axis = self.plot.getAxis('right')
            vb_right = getattr(self, "_vb_right", None)
            if vb_right is None:
                return
            # current Y range of the right (CISPR) ViewBox - already in CISPR units
            yr = vb_right.viewRange()[1]  # [ymin, ymax]
            ymin, ymax = float(yr[0]), float(yr[1])
            if ymax <= ymin:
                return

            # Snap tick positions to a 10 dB grid that covers the visible CISPR range
            step = 10.0
            start = math.floor(ymin / step) * step
            end = math.ceil(ymax / step) * step
            positions = np.arange(start, end + 0.1, step, dtype=np.float64)

            # Ticks are already in CISPR units, so the label is just the position itself.
            ticks = [(float(pos), f"{pos:.1f}") for pos in positions]

            # setTicks expects a list of levels; provide single level
            axis.setTicks([ticks])

            # show/hide axis according to checkbox
            try:
                axis.setVisible(bool(getattr(self, "chk_cispr", None) and self.chk_cispr.isChecked()))
            except Exception:
                pass
        except Exception:
            # defensive: don't crash GUI
            pass

    # def cispr_build_reference_curve(self):
        # # CISPR 16-1-1 style placeholder curve (dBµV)
        # f = np.array([9e3, 150e3, 500e3, 1e6, 30e6, 100e6, 200e6, 300e6, 600e6, 1e9
        # ], dtype=np.float64)

        # L= np.array([70, 66, 56, 56, 50, 40, 37, 33, 30, 28
        # ], dtype=np.float64)
         # ## very roughly with Tiny point antenna psd_dBuV = psd + 158.0

        # return f, L
        
    def cispr_build_reference_curve(self):
    # EN 55032 / CISPR 32 Class B radiated limits (simplified)
    # 10 m below 1 GHz, 3 m above 1 GHz – all in dBµV/m

        f = np.array([
            30e6,   # 30 MHz
            230e6,  # 230 MHz
            1000e6, # 1 GHz
            3e9,    # 3 GHz
            6e9     # 6 GHz
        ], dtype=np.float64)
        delta_3m_to_10m = -10.46  # 20*log10(3/10)
    # Use quasi-peak limits below 1 GHz, average limits above 1 GHz
        L = np.array([
            30.0,  # 30–230 MHz  (QP, 10 m)
            30.0,  # 30–230 MHz  (flat segment)
            37.0,  # 230–1000 MHz (QP, 10 m)
            50.0 + delta_3m_to_10m,            # 1–3 GHz scaled to 10 m
            54.0 + delta_3m_to_10m             # 3–6 GHz scaled to 10 m
        ], dtype=np.float64)

        return f, L


    def cispr_toggle_visibility(self, checked):
        """
        Only control visibility of the CISPR curve and right axis.
        Do NOT call cispr_update_curve() here - that remaps the right ViewBox
        and is what caused the red line to jump when the checkbox was toggled.
        """
        try:
            # show/hide the CISPR curve itself
            self.cispr_curve.setVisible(bool(checked))
        except Exception:
            pass

        try:
            # show/hide the right axis to match the checkbox
            self.plot.getAxis('right').setVisible(bool(checked))
        except Exception:
            pass

        # Refresh tick labels only (safe, does not remap the right ViewBox)
        try:
            self.cispr_update_axis_ticks()
        except Exception as e:
            print("cispr_toggle_visibility: cispr_update_axis_ticks error:", e)

    # =====================================================================
    # Stitching (edge-masking) & Time-Averaging support
    #
    # stitch_get_edge_bins / stitch_get_kept_bins define exactly how many FFT bins are
    # trimmed from each block before it enters the stitched display (see sweep_loop()
    # and build_centers()). avg_* mirror the cispr_on_offset_changed/cispr_refresh_offset_ui
    # pattern: simple GUI-thread setters for plain attributes that sweep_loop() (worker
    # thread) only ever reads - no Qt scene objects are touched here, so unlike the
    # cispr_* block above there is no GUI-thread restriction on these.
    # =====================================================================

    # ---- previous stitch_get_edge_bins() (always masked the edges), kept in case of problems ----
#     def stitch_get_edge_bins(self):
#         """Number of FFT bins discarded from EACH edge of a block before stitching."""
#         frac = float(getattr(self, "stitch_edge_mask_fraction", 0.0))
#         frac = max(0.0, min(0.49, frac))  # guard against masking away the whole block
#         return int(round(NFFT * frac))

    def stitch_get_edge_bins(self):
        """Number of FFT bins discarded from EACH edge of a block before stitching. None in single-frequency mode:
        there is only one block and nothing to stitch, so the whole captured bandwidth is shown."""
        if getattr(self, "single_mode", False):
            return 0
        frac = float(getattr(self, "stitch_edge_mask_fraction", 0.0))
        frac = max(0.0, min(0.49, frac))  # guard against masking away the whole block
        return int(round(NFFT * frac))

    def stitch_get_kept_bins(self):
        """Number of FFT bins actually stored/displayed per block, after edge masking."""
        return NFFT - 2 * self.stitch_get_edge_bins()

    def avg_on_toggle(self, checked):
        """Time Averaging checkbox -> model. Read by sweep_loop() via avg_get_active_count()."""
        self.avg_enabled = bool(checked)
        self.update_status_panel()

    def avg_on_count_changed(self, val):
        """Averages spinbox -> model. Read by sweep_loop() via avg_get_active_count()."""
        self.avg_count = int(val)
        self.update_status_panel()

    def avg_get_active_count(self):
        """Number of FFT captures to average per block: 1 (no averaging) unless enabled."""
        return max(1, int(getattr(self, "avg_count", 1))) if getattr(self, "avg_enabled", False) else 1

    def _read_normalized_iq(self, n):
        """
        Read n IQ samples from the SDR and return them as normalized complex64, using the
        same real-vs-complex / uint8-vs-float handling the sweep previously did inline.
        Factored out so sweep_loop() can call it once per averaging capture without
        duplicating this logic K times.
        """
        # The RTL-SDR USB read occasionally comes back short (device hiccup, USB
        # scheduling stall, etc.) and raises OSError("Short read..."). Retry a couple
        # of times with a brief pause before giving up - most short reads clear on
        # the very next attempt without needing to reopen the device at all.
        last_err = None
        samples = None
        for attempt in range(3):
            try:
                samples = self.sdr.read_samples(n)
                last_err = None
                break
            except OSError as e:
                last_err = e
                print(f"_read_normalized_iq: read_samples failed (attempt {attempt + 1}/3): {e}")
                time.sleep(0.05)
        if last_err is not None:
            # Still failing after retries - let the caller (sweep_loop) decide how to
            # recover (it will skip this block and/or reopen the device) instead of
            # letting the exception propagate up and kill the sweep thread.
            raise last_err

        samples = samples[:n]
        if np.iscomplexobj(samples):
            samples = samples.astype(np.complex64) / 128.0
        else:
            if samples.dtype == np.uint8:
                f = samples.astype(np.float32) - 128.0
            else:
                f = samples.astype(np.float32)
            if (f.size % 2) != 0:
                f = f[:-1]
            f = f.reshape(-1, 2)
            samples = (f[:, 0] + 1j * f[:, 1]).astype(np.complex64) / 128.0

        # ---- Optional artefact correction (both OFF by default, GUI-toggleable) ----
        # Applied here, on the RAW IQ samples, BEFORE windowing/FFT. The center-frequency
        # "DC spike" and its mirrored image are properties of the raw ADC/tuner samples
        # (LO leakage and I/Q gain-phase mismatch) - not of the window function - so
        # correcting them here is both the standard approach and more accurate than
        # trying to remove them from an already-windowed block: the window weights
        # samples unevenly, which would bias a windowed-mean DC estimate, and does
        # nothing at all for IQ imbalance, which is a per-sample I/Q relationship the
        # window doesn't touch.
        if getattr(self, "dc_removal_enabled", False):
            samples = self._remove_dc_offset(samples)
        if getattr(self, "iq_balance_enabled", False):
            samples = self._correct_iq_imbalance(samples)

        return samples

    def _remove_dc_offset(self, samples):
        """
        Subtract this block's own mean IQ value. Targets the "spike at the block's
        center frequency" artefact caused by LO leakage / ADC DC bias in direct-
        conversion receivers: since it sits at fc for every block, once many blocks are
        stitched together it shows up as a repeated spike, spaced by the per-block hop
        width - exactly the "moves with center frequency / sample rate" symptom.
        """
        return (samples - np.mean(samples)).astype(np.complex64)

    def _correct_iq_imbalance(self, samples):
        """
        Blind, per-block moment-based I/Q gain and phase imbalance correction (estimates
        the imbalance fresh from every block, rather than applying one fixed factory
        calibration). Gain/phase mismatch between a receiver's I and Q channels produces a
        mirrored, attenuated image of any strong nearby component - including the DC/LO
        leakage spike - reflected around the block's center frequency: another artefact
        that tracks center frequency and sample rate rather than being a real signal.
        Re-estimating per block adapts automatically but is noisier than a one-off
        calibration would be.
        """
        I = samples.real
        Q = samples.imag
        p_i = float(np.mean(I * I))
        p_q = float(np.mean(Q * Q))
        if p_i <= 0.0 or p_q <= 0.0:
            return samples
        gain = math.sqrt(p_q / p_i)
        sin_phi = float(np.mean(I * Q)) / math.sqrt(p_i * p_q)
        sin_phi = max(-0.999, min(0.999, sin_phi))
        cos_phi = math.sqrt(1.0 - sin_phi * sin_phi)
        Q_corrected = (Q / gain - sin_phi * I) / cos_phi
        return (I + 1j * Q_corrected).astype(np.complex64)

    def dc_removal_on_toggle(self, checked):
        """DC Offset Removal checkbox -> model. Read by _read_normalized_iq()."""
        self.dc_removal_enabled = bool(checked)

    def iq_balance_on_toggle(self, checked):
        """IQ Balance Correction checkbox -> model. Read by _read_normalized_iq()."""
        self.iq_balance_enabled = bool(checked)

    # =====================================================================
    # Display toggles: frequency markers & reference trace
    #
    # Both are pure display features - neither touches the sweep/stitching logic above,
    # and neither is safety-sensitive w.r.t. threading: markers_toggle_visibility() and
    # reference_toggle_visibility() are only ever called from Qt checkbox signals (GUI
    # thread), and reference_capture() only ever called from a Qt button signal (GUI
    # thread), reading self.curve's already-set data rather than the worker thread's
    # arrays - so there's nothing here that needs the cispr_*-style thread restriction.
    # =====================================================================

    def markers_toggle_visibility(self, checked):
        """
        Show/hide the green (raw block) and cyan dashed (kept/stitched) vertical frequency
        markers without rebuilding them. draw_block_markers() also reads self.markers_visible
        so newly (re)drawn markers - e.g. after a MODE change - come up in the same state.
        """
        self.markers_visible = bool(checked)
        for item in self._block_markers:
            try:
                item.setVisible(self.markers_visible)
            except Exception:
                pass

    def sync_sample_rate_combo(self):
        """
        Reflect self.SAMPLE_RATE in the Sample Rate combo box without re-triggering
        sample_rate_on_changed() - call this whenever something OTHER than the combo box
        itself changes self.SAMPLE_RATE (currently: set_preset()). If self.SAMPLE_RATE
        isn't one of SAMPLE_RATE_CHOICES (e.g. a preset uses a rate not in that list),
        the combo is left on whatever it currently shows rather than guessing.
        """
        idx = self.sample_rate_combo.findData(self.SAMPLE_RATE)
        if idx < 0:
            return
        self.sample_rate_combo.blockSignals(True)
        self.sample_rate_combo.setCurrentIndex(idx)
        self.sample_rate_combo.blockSignals(False)

    def sync_gain_slider(self):
        """Move the RF Gain slider to the step nearest self.gain_db WITHOUT triggering
        gain_slider_on_changed() - call this whenever something other than the slider
        changes self.gain_db (currently: set_preset() and start-up)."""
        g = float(self.gain_db)
        idx = min(range(len(RTL_GAIN_STEPS_DB)), key=lambda i: abs(RTL_GAIN_STEPS_DB[i] - g))
        self.gain_slider.blockSignals(True)
        self.gain_slider.setValue(idx)
        self.gain_slider.blockSignals(False)
        self.gain_value_label.setText(f"{RTL_GAIN_STEPS_DB[idx]:.1f} dB")

    def gain_slider_on_changed(self, index):
        """RF Gain slider -> model (GUI thread). Only records the new gain; the sweep
        thread applies it to the hardware at the next block - see apply_gain_if_changed()."""
        if index < 0 or index >= len(RTL_GAIN_STEPS_DB):
            return
        gain = float(RTL_GAIN_STEPS_DB[index])
        self.gain_value_label.setText(f"{gain:.1f} dB")
        if gain == self.gain_db:
            return
        self.gain_db = gain
        self.update_status_panel()

    def apply_gain_if_changed(self):
        """Apply self.gain_db to the hardware if it differs from what is configured.
        Called ONLY from the sweep thread (top of each block), never from the GUI thread,
        because the RTL-SDR driver isn't safe to call concurrently from two threads."""
        want = self.gain_db
        if self.sdr is None or self._applied_gain_db == want:
            return
        self._applied_gain_db = want     # set first so a failure doesn't retry every block
        try:
            self.sdr.gain = want
            self.actual_gain_db = float(self.sdr.get_gain())
            self.actual_gain_db_for_request = want
            print(f"Gain changed live: requested {want} dB -> actual {self.actual_gain_db} dB")
            self.update_status_panel()
        except Exception as e:
            print("apply_gain_if_changed: error setting gain:", e)

    def sample_rate_on_changed(self, index):
        """
        Sample Rate combo box -> model. Deliberately does NOT stop a running sweep or
        touch the hardware directly from here - see sweep_loop() for why (the RTL-SDR
        driver isn't safe to call concurrently from two threads). This just updates
        self.SAMPLE_RATE and, if nothing is currently running, rebuilds the centers and
        markers immediately for instant feedback; if a sweep IS running, sweep_loop()
        (worker thread) picks up the new rate and applies it to the hardware itself at
        the start of its next full pass, then asks the GUI thread to redraw markers via
        the markers_changed signal.
        """
        new_rate = self.sample_rate_combo.itemData(index)
        if new_rate is None:
            return
        new_rate = float(new_rate)
        if new_rate == self.SAMPLE_RATE:
            return
        self.SAMPLE_RATE = new_rate
        running = bool(self.sweep_thread and self.sweep_thread.is_alive())
        if running:
            # Tell the sweep thread to abandon the pass in progress at the next block and
            # start a fresh one at the new rate (a pass must never mix two sample rates).
            self._restart_pass = True
            self.log(f"Sample rate -> {new_rate/1e6:.3f} MS/s (restarting sweep pass)")
        else:
            self.log(f"Sample rate -> {new_rate/1e6:.3f} MS/s (applies when the sweep starts)")
        self.update_status_panel()
        if not running:
            # Nothing running - safe to rebuild centers/markers directly on the GUI thread.
            if self.START_FREQ == self.STOP_FREQ:
                self.centers = np.array([self.START_FREQ], dtype=np.float64)
            else:
                self.centers = self.build_centers(self.START_FREQ, self.STOP_FREQ, self.SAMPLE_RATE)
            self.draw_block_markers()

    def reference_capture(self):
        """
        Snapshot whatever is currently drawn on the live FFT curve and store it as the
        reference trace (e.g. a "system noise" baseline captured with the target system
        switched off). Runs on the GUI thread (button click), reading self.curve's
        already-set data - safe, since only the GUI thread (set_plot_data) ever writes it.

        This is a DISPLAY-ONLY snapshot: it is drawn in light blue for comparison and is
        never subtracted from, or combined with, the live trace.
        """
        try:
            f, p = self.curve.getData()
        except Exception as e:
            print("reference_capture: could not read live curve data:", e)
            return
        if f is None or p is None or len(f) == 0:
            self.log("Capture Reference: no live data to capture yet")
            return
        self.reference_freq = np.array(f, dtype=np.float64, copy=True)
        self.reference_power = np.array(p, dtype=np.float64, copy=True)
        self.reference_curve.setData(self.reference_freq, self.reference_power)
        self.reference_curve.setVisible(True)
        self.log(f"Reference captured: {len(self.reference_freq)} points")
        # Capturing implies you want to see it - reflect that in the checkbox without
        # re-triggering this method via its own toggled signal.
        if not self.chk_show_ref.isChecked():
            self.chk_show_ref.blockSignals(True)
            self.chk_show_ref.setChecked(True)
            self.chk_show_ref.blockSignals(False)

    def reference_toggle_visibility(self, checked):
        """Show/hide the captured reference trace. Never affects the stored snapshot."""
        try:
            self.reference_curve.setVisible(bool(checked))
        except Exception:
            pass

    def start_sweep(self):
        # Start a single fast stitched sweep (one-shot), using the LIVE preset settings
        # (self.START_FREQ/self.STOP_FREQ/self.SAMPLE_RATE/self.gain_db) - i.e. whichever
        # MODE button was last pressed, not the module-level defaults.
        if self.sweep_thread and self.sweep_thread.is_alive():
            return
        self.log("=== FAST SWEEP STARTED ===")
        # Range/Sample rate/Gain/Averaging are shown on the stable status panel (see
        # update_status_panel()) rather than logged here - logging them on every sweep
        # start would just duplicate what's already always visible above.
        self.update_status_panel()

        # Build centers from current preset (same logic as start_continuous)
        if self.START_FREQ == self.STOP_FREQ:
            self.centers = np.array([self.START_FREQ], dtype=np.float64)
        else:
            self.centers = self.build_centers(self.START_FREQ, self.STOP_FREQ, self.SAMPLE_RATE)

        # Diagnostic: show centers and expected block ranges (MHz) - console only (see
        # the print() redirect above); not forwarded to the GUI debug window.
        print("start_sweep Diagnostic: centers (MHz):", (self.centers / 1e6).tolist())
        block_width_hz = self.SAMPLE_RATE
        ranges = [(c - block_width_hz/2.0, c + block_width_hz/2.0) for c in self.centers]
        print("     Diagnostic: expected block ranges (MHz):",
            [f"{a/1e6:.6f}-{b/1e6:.6f}" for a, b in ranges])
        self.log(f"Total hops: {len(self.centers)}")

        # Allocate and clear stitched arrays for this sweep
        total_bins = len(self.centers) * self.stitch_get_kept_bins()
        if total_bins == 0:
            print("start_sweep: no centers defined, aborting sweep")
            return

        self.freq_axis = np.full(total_bins, np.nan, dtype=np.float64)
        self.power_axis = np.full(total_bins, -200.0, dtype=np.float64)
        print(f"start_sweep: allocated freq_axis/power_axis length {total_bins}")

        # Draw markers for visual debugging (safe call)
        try:
            self.draw_block_markers()
        except Exception:
            pass

        # Start sweep thread
        self.stop_flag.clear()
        # init_sdr should be idempotent; it will skip reopen if already open, and always
        # re-applies self.SAMPLE_RATE/self.gain_db/self.hf_mode to the hardware.
        self.init_sdr()
        self.sweep_thread = threading.Thread(target=self.single_sweep_loop, daemon=True)
        self.sweep_thread.start()

    def start_continuous(self):
        # Start continuous repeating stitched sweep, using the same LIVE preset settings
        # as start_sweep() above.
        if self.sweep_thread and self.sweep_thread.is_alive():
            return

        self.log("=== CONTINUOUS MODE STARTED ===")
        # Range/Sample rate/Gain/Averaging are shown on the stable status panel rather
        # than logged here - see update_status_panel().
        self.update_status_panel()
        self.stop_flag.clear()
        # init_sdr should be idempotent; it will skip reopen if already open, and always
        # re-applies self.SAMPLE_RATE/self.gain_db/self.hf_mode to the hardware.
        self.init_sdr()

        # Build centers from current preset (same logic as start_sweep)
        if self.START_FREQ == self.STOP_FREQ:
            self.centers = np.array([self.START_FREQ], dtype=np.float64)
        else:
            self.centers = self.build_centers(self.START_FREQ, self.STOP_FREQ, self.SAMPLE_RATE)

        # Diagnostic: show centers and expected block ranges (MHz)
        print("start_continuous Diagnostic: centers (MHz):", (self.centers / 1e6).tolist())
        block_width_hz = self.SAMPLE_RATE
        ranges = [(c - block_width_hz/2.0, c + block_width_hz/2.0) for c in self.centers]
        print("         Diagnostic: expected block ranges (MHz):",
            [f"{a/1e6:.6f}-{b/1e6:.6f}" for a, b in ranges])

        # Allocate and clear stitched arrays for continuous mode
        total_bins = len(self.centers) * self.stitch_get_kept_bins()
        if total_bins == 0:
            print("start_continuous: no centers defined, aborting")
            return

        self.freq_axis = np.full(total_bins, np.nan, dtype=np.float64)
        self.power_axis = np.full(total_bins, -200.0, dtype=np.float64)
        print(f"start_continuous: allocated freq_axis/power_axis length {total_bins}")

        # Draw markers for visual debugging (safe call)
        try:
            self.draw_block_markers()
        except Exception:
            pass

        # If single-center, tune SDR to that center; if multi-center, tune to first center
        if len(self.centers) >= 1:
            try:
                self.sdr.center_freq = float(self.centers[0])
            except Exception:
                pass

        # Start repeating sweep in a thread
        self.sweep_thread = threading.Thread(target=self.continuous_loop, daemon=True)
        self.sweep_thread.start()

    def sweep_loop(self):
        # Clear the restart request BEFORE reading the live settings below: a change made
        # after this point sets it again and is caught at the next block boundary.
        self._restart_pass = False
        # Rebuild sweep centers from the CURRENT live settings every time this runs - not
        # just once when the sweep first started. This is what lets a live Sample Rate
        # change (from the combo box) take effect on the very next pass without needing
        # to stop continuous mode: continuous_loop() just calls sweep_loop() repeatedly,
        # and each call re-derives self.centers from self.START_FREQ/STOP_FREQ/SAMPLE_RATE
        # as they stand right now. A change mid-pass only takes effect on the NEXT pass -
        # this pass keeps running with the centers it already had, so a single sweep never
        # ends up half old-rate, half new-rate.
        if self.START_FREQ == self.STOP_FREQ:
            new_centers = np.array([self.START_FREQ], dtype=np.float64)
        else:
            new_centers = self.build_centers(self.START_FREQ, self.STOP_FREQ, self.SAMPLE_RATE)
        centers_changed = (self.centers is None or len(self.centers) != len(new_centers)
                            or not np.allclose(self.centers, new_centers))
        self.centers = new_centers

        if self.centers is None or len(self.centers) == 0:
            print("sweep_loop: no centers defined, exiting")
            return

        if centers_changed:
            # draw_block_markers() touches Qt scene items directly and must only ever run
            # on the GUI thread - ask for it via a queued signal rather than calling it here.
            try:
                self.plot_signals.markers_changed.emit()
            except Exception:
                pass

        # Apply the current sample rate to the hardware if it's changed since we last
        # configured it. Done here (worker thread) and ONLY here during a running sweep -
        # never from the GUI thread directly, since the RTL-SDR driver isn't safe to call
        # concurrently from two threads. This is what actually makes the Sample Rate combo
        # box take effect without stopping continuous mode.
        if self.sdr is not None and getattr(self, "_applied_sample_rate", None) != self.SAMPLE_RATE:
            try:
                print(f"sweep_loop: applying live sample-rate change -> {self.SAMPLE_RATE/1e6:.3f} MS/s")
                self.sdr.sample_rate = self.SAMPLE_RATE
                self._applied_sample_rate = self.SAMPLE_RATE
            except Exception as e:
                print("sweep_loop: error applying live sample-rate change:", e)

        # Number of bins actually stored per block AFTER edge masking (see
        # stitch_get_kept_bins()) - this, not NFFT, is the real per-block stride into
        # freq_axis/power_axis, since the masked-off edge bins are never stored.
        edge_bins = self.stitch_get_edge_bins()
        kept_bins = NFFT - 2 * edge_bins

        expected_bins = len(self.centers) * kept_bins
        if (self.freq_axis is None or self.power_axis is None or self.freq_axis.size != expected_bins
                or centers_changed):
            print(f"sweep_loop: stitched arrays missing or wrong size; reallocating to {expected_bins}")
            self.freq_axis = np.full(expected_bins, np.nan, dtype=np.float64)
            self.power_axis = np.full(expected_bins, -200.0, dtype=np.float64)

        # Diagnostic: show SDR object identity so we can detect mid-sweep reinitialisation
        try:
            print("sweep_loop: sdr object id:", id(self.sdr))
        except Exception:
            print("sweep_loop: sdr object not present")

        # Pass bookkeeping for the survey: which gains were in force while the blocks were
        # captured, how many blocks were stored, and the rate this pass really ran at.
        pass_t0 = time.time()
        pass_rate = getattr(self, "_applied_sample_rate", self.SAMPLE_RATE)
        pass_gmin = pass_gmax = None
        blocks_stored = 0

        window = np.hanning(NFFT)
        # optional counter to throttle GUI updates if needed
        update_counter = 0

        # How many consecutive FFT captures to average (in power) per block this sweep.
        # Read once per sweep rather than once per block: averaging count is a GUI
        # setting the operator can change mid-sweep, but re-reading it block-to-block
        # would let a single sweep silently mix differently-averaged blocks together.
        n_avg = self.avg_get_active_count()
        if n_avg > 1:
            print(f"sweep_loop: time averaging ON - {n_avg} captures/block")

        for i, fc in enumerate(self.centers):
            if self.stop_flag.is_set():
                break

            # Live sample-rate change: abandon this pass; the caller starts a fresh one.
            if self._restart_pass:
                print("sweep_loop: sample rate changed - restarting pass")
                break

            # Live RF gain change: applied here, at a block boundary, from this thread.
            self.apply_gain_if_changed()
            _g = self._applied_gain_db
            if _g is not None:
                pass_gmin = _g if pass_gmin is None else min(pass_gmin, _g)
                pass_gmax = _g if pass_gmax is None else max(pass_gmax, _g)

            # Diagnostic: about to tune this block
            print(f"sweep_loop: i={i}, planned_center={fc/1e6:.6f} MHz")
            try:
                print(f"sweep_loop: block {i} using sdr id {id(self.sdr)}")
            except Exception:
                pass

            # Tune and settle
            try:
                self.sdr.center_freq = float(fc)
            except Exception as e:
                print("sweep_loop: error setting center_freq:", e)

            # Diagnostic: confirm device reports center (if API supports get)
            try:
                reported = getattr(self.sdr, "center_freq", None)
                if reported is not None:
                    print(f"sweep_loop: device center_freq={reported/1e6:.6f} MHz")
            except Exception:
                pass

            # allow tuner/AGC to settle and flush driver buffers (once per hop, not once
            # per averaged capture - the tuner isn't retuned between averaging captures)
            time.sleep(0.12)
            try:
                _ = self.sdr.read_samples(512)
                _ = self.sdr.read_samples(512)
            except Exception as e:
                print("sweep_loop: flush read_samples error:", e)

            # ---- Capture + average (in POWER, not dB) ----
            # Averaging in the log domain would bias the result low; averaging the linear
            # power |spec|^2 across n_avg independent captures and THEN converting to dB
            # is the standard periodogram-averaging approach (as used e.g. in Welch's
            # method) and is what actually reduces trace noise. With n_avg=1 (averaging
            # off) this reduces to exactly the original single-capture formula, since
            # 10*log10(|spec|^2) == 20*log10(|spec|).
            power_acc = None
            captures_done = 0
            read_failed = False
            for _k in range(n_avg):
                if self.stop_flag.is_set():
                    break
                try:
                    samples = self._read_normalized_iq(NFFT)
                except OSError as e:
                    # Hard USB failure that survived the retries in
                    # _read_normalized_iq(). Don't let this kill the sweep thread:
                    # drop this block, try to reopen the device, and carry on with
                    # the next block/hop instead of crashing the whole scan.
                    print(f"sweep_loop: read error on block {i}, capture {_k}: {e}")
                    read_failed = True
                    break
                spec = np.fft.fftshift(np.fft.fft(samples * window))
                spec = spec / NFFT
                power = np.abs(spec) ** 2
                power_acc = power if power_acc is None else power_acc + power
                captures_done += 1

            if read_failed:
                try:
                    print("sweep_loop: attempting to reopen SDR device after read failure...")
                    self.init_sdr()
                    print("sweep_loop: SDR device reopened successfully")
                except Exception as e:
                    print(f"sweep_loop: failed to reopen SDR device: {e}")
                    # Give the hardware/driver a moment before the next attempt so we
                    # don't spin tight in a reopen-fail loop.
                    time.sleep(0.5)
                continue

            if captures_done == 0:
                # Stopped before a single capture completed - nothing to store this block.
                continue

            mean_power = power_acc / float(captures_done)
            psd_full = 10.0 * np.log10(mean_power + 1e-12) + self._psd_to_dbuv_const

            # Frequency axis for this FFT block (use same fc). Use the LIVE sample rate
            # (self.SAMPLE_RATE), which set_preset() updates - not the module-level default.
            freqs_full = np.fft.fftshift(np.fft.fftfreq(NFFT, d=1.0 / self.SAMPLE_RATE)) + float(fc)

            # ---- Mask the outer edges before this block enters the stitched display ----
            # freqs_full/psd_full cover the FULL captured bandwidth (including the noisy/
            # rolled-off edges); freqs/psd below are the trimmed "kept" slice that actually
            # gets stored and stitched. edge_bins/kept_bins were computed once above.
            if edge_bins > 0:
                freqs = freqs_full[edge_bins:NFFT - edge_bins]
                psd = psd_full[edge_bins:NFFT - edge_bins]
            else:
                freqs = freqs_full
                psd = psd_full

            # Diagnostic: block peak info and sample checksum
            peak_idx = np.nanargmax(psd)
            peak_freq = freqs[peak_idx]
            peak_level = psd[peak_idx]
            psd_checksum = np.sum(np.round(psd, 3))
            #print(f"DIAG block {i}: peak {peak_freq/1e6:.6f} MHz @ {peak_level:.1f} dB, checksum {psd_checksum:.3f}")
            #print("DIAG block sample freqs[0:6] (MHz):", (freqs[:6]/1e6).tolist())
            #print("DIAG block sample psd[0:6]:", psd[:6].tolist())

            # Diagnostic: show block frequency span and planned storage indices
            fmin = freqs.min()
            fmax = freqs.max()
            start = i * kept_bins
            stop = start + kept_bins
            #print(f"sweep_loop: block {i} freq span {fmin/1e6:.6f}-{fmax/1e6:.6f} MHz planned store [{start}:{stop}]")

            # Safety checks before writing into stitched arrays
            if self.freq_axis is None or self.power_axis is None:
                print("sweep_loop: ERROR - stitched arrays not allocated")
                break

            if stop > self.freq_axis.size:
                print(f"sweep_loop: ERROR - block {i} stop index {stop} exceeds array length {self.freq_axis.size}")
                break

            # Detect accidental overwrite (indicates logic bug)
            existing_mask = ~np.isnan(self.freq_axis[start:stop])
            #if existing_mask.any():
            #    print(f"sweep_loop: WARNING - block {i} would overwrite existing data at indices {start}:{stop}")

            # Diagnostic: compute checksums and peaks for comparison
            peak_idx = np.nanargmax(psd)
            peak_freq = freqs[peak_idx]
            peak_level = psd[peak_idx]
            psd_checksum = float(np.sum(np.round(psd, 6)))
            print(f"sweep_loop DIAG: block {i} peak {peak_freq/1e6:.6f} MHz @ {peak_level:.2f} dB checksum {psd_checksum:.6f}")

            # Compare with previous block if present and non-empty
            if i > 0:
                prev = self.power_axis[start-kept_bins:start]
                if not np.all(prev == -200.0):
                    prev_checksum = float(np.sum(np.round(prev, 6)))
                    prev_peak_idx = int(np.nanargmax(prev))
                    prev_peak_freq = float(self.freq_axis[start-kept_bins:start][prev_peak_idx])
                    print(f"sweep_loop DIAG: prev block {i-1} peak {prev_peak_freq/1e6:.6f} MHz checksum {prev_checksum:.6f}")
                    if abs(psd_checksum - prev_checksum) < 1e-6:
                        print(f"sweep_loop: WARNING - block {i} PSD checksum equals previous block -> skipping write")
                        continue

            # Store block in stitched spectrum (single atomic write) - the MASKED/kept
            # slice only. The full, unmasked freqs_full/psd_full never get stored/stitched.
            self.freq_axis[start:stop] = freqs
            self.power_axis[start:stop] = psd
            blocks_stored += 1

            # show current block overlay (debug only) - shows the FULL captured block,
            # including the masked edges, so you can see exactly what's being trimmed off.
            try:
                if hasattr(self, "debug_block_curve"):
                    self.debug_block_curve.setData(freqs_full, psd_full)
            except Exception:
                pass

            # Immediately update the plot so GUI reflects the stored block
            self.update_plot()

            # Throttle if necessary (kept here for easy tuning)
            update_counter += 1
            if update_counter >= 1:
                update_counter = 0

        # Final update after sweep completes
        self.update_plot()

        # Publish this pass for the survey - but only if it is a CLEAN pass: every block
        # stored, not stopped or restarted, one gain and one sample rate throughout.
        if (blocks_stored == len(self.centers) and not self._restart_pass
                and not self.stop_flag.is_set() and pass_gmin is not None
                and pass_gmin == pass_gmax and pass_rate == self.SAMPLE_RATE):
            self._pass_seq += 1
            self._last_pass = {
                "seq": self._pass_seq, "rate": pass_rate, "gain": pass_gmin,
                "n_avg": n_avg, "dur": time.time() - pass_t0,
                "f": self.freq_axis.copy(), "p": self.power_axis.copy(),
            }

    # ------------------------------------------------------------------------------------
    # Gain / sample-rate SURVEY
    # ------------------------------------------------------------------------------------
    # Steps through combinations of sample rate and RF gain on the CURRENT band (whichever
    # MODE was last chosen), waits for one clean complete pass at each, and records:
    #   <base>_spikes.csv   every spike found: repeat,sample_rate_hz,gain_db,freq_hz,level_db,
    #                       prominence_db,baseline_db
    #   <base>_summary.csv  one row per combination (noise floor, spike count, strongest spike)
    #   <base>_spectra.npz  the full stitched spectrum of every pass (numpy compressed)
    # It runs on a GUI timer and drives the same combo box / gain model as manual use, so the
    # normal live rate/gain machinery does the hardware work. STOP ends it early.

    @staticmethod
    def survey_analyse(f, p, thresh_db):
        """Return (f_clean, p_clean, floor_db, spikes) for one stitched spectrum.
        spikes = list of (freq_hz, level_db, prominence_db, baseline_db), where baseline is the
        local median (about 100 bins wide) and a spike is a local maximum standing at least
        thresh_db above it."""
        f = np.asarray(f, dtype=np.float64)
        p = np.asarray(p, dtype=np.float64)
        ok = np.isfinite(f) & np.isfinite(p) & (p > -190.0)
        f, p = f[ok], p[ok]
        order = np.argsort(f)
        f, p = f[order], p[order]
        n = len(p)
        if n < 20:
            return f, p, float("nan"), []
        floor = float(np.median(p))
        base = np.full(n, floor)
        try:
            from numpy.lib.stride_tricks import sliding_window_view
            w = 101
            pad = w // 2
            pp = np.pad(p, pad, mode="edge")
            for a in range(0, n, 20000):                  # chunked to keep memory modest
                b = min(a + 20000, n)
                base[a:b] = np.median(sliding_window_view(pp[a:b + w - 1], w), axis=1)
        except Exception:
            pass                                          # fall back to the global median
        prom = p - base
        pk = np.zeros(n, dtype=bool)
        pk[2:-2] = ((prom[2:-2] > thresh_db)
                    & (p[2:-2] >= p[1:-3]) & (p[2:-2] >= p[3:-1])
                    & (p[2:-2] >= p[:-4]) & (p[2:-2] >= p[4:]))
        spikes = [(float(f[i]), float(p[i]), float(prom[i]), float(base[i])) for i in np.nonzero(pk)[0]]
        return f, p, floor, spikes

    def survey_rate_fits(self, rate):
        """True if the current band fits inside ONE block at this sample rate: the middle
        (1 - 2*edge_mask) of the block - the part that is actually kept - must be at least as
        wide as the band. Then a pass is a single capture with no hopping or stitching."""
        kept_width = rate * (1.0 - 2.0 * float(self.stitch_edge_mask_fraction))
        return kept_width >= (self.STOP_FREQ - self.START_FREQ) - 1e-6

    # ---- previous survey_band_ok(), kept in case of problems ----
#     def survey_band_ok(self):
#         """Check the current band is allowed for a survey; returns an error text or ''."""
#         if self.START_FREQ == self.STOP_FREQ:
#             return "pick a MODE with a frequency range first (not a single frequency)"
#         if self.STOP_FREQ > SURVEY_MAX_STOP_HZ:
#             return (f"the survey only runs on bands that end at or below "
#                     f"{SURVEY_MAX_STOP_HZ/1e6:g} MHz (this band ends at {self.STOP_FREQ/1e6:g} MHz)")
#         if not any(self.survey_rate_fits(r) for r in SAMPLE_RATE_CHOICES):
#             return "no sample rate covers this band in a single block - choose a narrower band"
#         return ""

    def survey_band_ok(self):
        """Check the current band can be surveyed; returns an error text or ''. Any range works now:
        a band wider than one block is swept in several blocks and stitched, as in normal use."""
        if self.START_FREQ == self.STOP_FREQ:
            return "pick a MODE with a frequency range first (not a single frequency)"
        return ""

    def survey_set_lock(self, locked):
        """Disable the controls that would fight the survey while it runs (STOP stays live)."""
        for wdg in ([self.sample_rate_combo, self.gain_slider, self.btn_survey,
                     self.btn_cont, self.btn_start] + list(self.mode_buttons)):
            wdg.setEnabled(not locked)

    # ---- previous survey_open_dialog() / survey_start(), kept in case of problems ----
#     def survey_open_dialog(self):
#         if self._survey is not None:
#             return
#         problem = self.survey_band_ok()
#         if problem:
#             self.log(f"Survey not started: {problem}")
#             QtWidgets.QMessageBox.information(self, "Gain / sample-rate survey", f"Survey not available: {problem}.")
#             return
#         dlg = QtWidgets.QDialog(self)
#         dlg.setWindowTitle("Gain / sample-rate survey")
#         lay = QtWidgets.QVBoxLayout(dlg)
#         lay.addWidget(QtWidgets.QLabel(
#             f"Band: {self.START_FREQ/1e6:.3f} - {self.STOP_FREQ/1e6:.3f} MHz "
#             f"(the current MODE). Averaging setting is used as-is.\n"
#             f"Only sample rates that cover the whole band in one block can be selected."))
#         form = QtWidgets.QFormLayout()
#         gain_texts = [f"{g:.1f} dB" for g in RTL_GAIN_STEPS_DB]
#         cb_lo, cb_hi = QtWidgets.QComboBox(), QtWidgets.QComboBox()
#         cb_lo.addItems(gain_texts)
#         cb_hi.addItems(gain_texts)
#         cb_lo.setCurrentIndex(RTL_GAIN_STEPS_DB.index(12.5))
#         cb_hi.setCurrentIndex(RTL_GAIN_STEPS_DB.index(33.8))
#         sp_stride = QtWidgets.QSpinBox()
#         sp_stride.setRange(1, 10)
#         sp_stride.setValue(1)
#         sp_rep = QtWidgets.QSpinBox()
#         sp_rep.setRange(1, 5)
#         sp_rep.setValue(1)
#         form.addRow("Gain from", cb_lo)
#         form.addRow("Gain to", cb_hi)
#         form.addRow("Use every Nth gain step", sp_stride)
#         form.addRow("Repeats per combination", sp_rep)
#         lay.addLayout(form)
#         lay.addWidget(QtWidgets.QLabel("Sample rates to test:"))
#         lst = QtWidgets.QListWidget()
#         for r in SAMPLE_RATE_CHOICES:
#             fits = self.survey_rate_fits(r)
#             it = QtWidgets.QListWidgetItem(f"{r/1e6:.3f} MS/s" + ("" if fits else "   (band too wide for one block)"))
#             it.setData(QtCore.Qt.UserRole, float(r))
#             if fits:
#                 it.setFlags(it.flags() | QtCore.Qt.ItemIsUserCheckable)
#                 it.setCheckState(QtCore.Qt.Checked)
#             else:
#                 it.setFlags(it.flags() & ~(QtCore.Qt.ItemIsEnabled | QtCore.Qt.ItemIsUserCheckable | QtCore.Qt.ItemIsSelectable))
#             lst.addItem(it)
#         lst.setMinimumHeight(180)
#         lay.addWidget(lst)
#         info = QtWidgets.QLabel("")
#         lay.addWidget(info)
#         def chosen():
#             lo, hi = sorted((cb_lo.currentIndex(), cb_hi.currentIndex()))
#             gains = [float(g) for g in RTL_GAIN_STEPS_DB[lo:hi + 1:sp_stride.value()]]
#             rates = [lst.item(i).data(QtCore.Qt.UserRole) for i in range(lst.count())
#                      if (lst.item(i).flags() & QtCore.Qt.ItemIsUserCheckable)
#                      and lst.item(i).checkState() == QtCore.Qt.Checked]
#             return rates, gains, sp_rep.value()
#         def refresh_info(*_):
#             rates, gains, reps = chosen()
#             info.setText(f"{len(rates)} rates x {len(gains)} gains x {reps} repeat(s) = "
#                          f"{len(rates) * len(gains) * reps} passes")
#         for wdg in (cb_lo, cb_hi):
#             wdg.currentIndexChanged.connect(refresh_info)
#         sp_stride.valueChanged.connect(refresh_info)
#         sp_rep.valueChanged.connect(refresh_info)
#         lst.itemChanged.connect(refresh_info)
#         refresh_info()
#         bb = QtWidgets.QDialogButtonBox(QtWidgets.QDialogButtonBox.Ok | QtWidgets.QDialogButtonBox.Cancel)
#         bb.accepted.connect(dlg.accept)
#         bb.rejected.connect(dlg.reject)
#         lay.addWidget(bb)
#         if dlg.exec_() != QtWidgets.QDialog.Accepted:
#             return
#         rates, gains, reps = chosen()
#         if not rates or not gains:
#             self.log("Survey: nothing selected")
#             return
#         path, _ = QtWidgets.QFileDialog.getSaveFileName(
#             self, "Survey results - base file name", "rf_survey.csv", "CSV Files (*.csv)")
#         if not path:
#             return
#         self.survey_start(rates, gains, reps, path)
#     def survey_start(self, rates, gains, repeats, path):
#         if self._survey is not None:
#             return
#         problem = self.survey_band_ok()
#         if problem:
#             self.log(f"Survey not started: {problem}")
#             return
#         rates = [r for r in rates if self.survey_rate_fits(r)]
#         if not rates:
#             self.log("Survey not started: none of the chosen sample rates covers the band in one block")
#             return
#         base = os.path.splitext(path)[0]
#         try:
#             spikes_fh = open(base + "_spikes.csv", "w", newline="")
#             summary_fh = open(base + "_summary.csv", "w", newline="")
#         except Exception as e:
#             self.log(f"Survey: cannot create result files: {e}")
#             return
#         spikes_w, summary_w = csv.writer(spikes_fh), csv.writer(summary_fh)
#         spikes_w.writerow(["repeat", "sample_rate_hz", "gain_db", "freq_hz", "level_db",
#                            "prominence_db", "baseline_db"])
#         summary_w.writerow(["repeat", "sample_rate_hz", "gain_db", "n_avg", "bins",
#                             "floor_db_median", f"spikes_over_{SURVEY_SPIKE_THRESH_DB:g}dB",
#                             "max_prominence_db", "max_spike_freq_hz", "pass_seconds"])
#         alive = bool(self.sweep_thread and self.sweep_thread.is_alive())
#         self._survey = {
#             "combos": [(r, g, k) for r in rates for g in gains for k in range(repeats)],
#             "idx": -1, "base": base, "spikes_fh": spikes_fh, "summary_fh": summary_fh,
#             "spikes_w": spikes_w, "summary_w": summary_w, "spectra": [],
#             "t_start": time.time(), "orig_rate": self.SAMPLE_RATE, "orig_gain": self.gain_db,
#             "was_running": alive, "pass_dur": 10.0, "seq_floor": 0, "t_set": time.time(),
#             "timeout": 90.0, "skipped": 0,
#         }
#         self.log(f"=== SURVEY STARTED: {len(self._survey['combos'])} passes -> {base}_*.csv/.npz ===")
#         self.survey_set_lock(True)
#         self.survey_next_combo()           # sets rate/gain for the first combination
#         if not alive:
#             self.start_continuous()
#         self._survey_timer.start()

    # ---- previous survey_connect_signals(), kept in case of problems ----
#     def survey_connect_signals(self):
#         self.survey_signals = SurveySignals()
#         self.survey_signals.done.connect(self.survey_finish)
#         self.btn_survey.setToolTip(
#             "Automatically step through sample rates and RF gains on the current band (any width - wide "
#             "bands are swept in several blocks) and save the results: spikes and summary CSVs, plus one "
#             "surface-chart CSV per sample rate for Excel. Press STOP to end early (partial results are saved).")

    # ------------------------------------------------------------------------------------
    # STITCH CHECK: is a feature in the stitched spectrum real, or made by the block layout?
    # ------------------------------------------------------------------------------------
    # The whole band is swept twice: once with the normal block layout and once with every block shifted by
    # half a block width (so each frequency is now measured from a different tuner frequency and a different
    # place in the receiver's passband). A real signal measures the same both times. Anything that depends on
    # WHERE in a block a frequency falls (LO / DC leakage, I-Q images, IF filter edges, clock spurs sitting
    # at a block edge, the tuner changing path at 28.8 MHz ...) changes or moves, so the two traces disagree
    # there. The second trace is drawn over the normal one (cyan) and the disagreeing ranges are shaded.

    def stitch_build_ui(self, left_layout):
        self._stitch = None
        self._stitch_shift = 0.0              # fraction of a block width that build_centers() shifts the layout by
        self._stitch_regions = []
        self.stitch_curve = self.plot.plot(pen=pg.mkPen((0, 220, 255), width=1))
        self.stitch_curve.hide()
        self.stitch_signals = StitchSignals()
        self.stitch_signals.done.connect(self.stitch_finish)
        row = QtWidgets.QHBoxLayout()
        left_layout.addLayout(row)
        self.btn_stitch = QtWidgets.QPushButton("STITCH CHECK")
        self.btn_stitch.setToolTip(
            "Sweep the current band twice - normal block layout, then every block shifted by half a block - and "
            "compare. Real signals look the same both times; anything that depends on where a frequency falls in "
            "a block (LO / DC leakage, images, filter edges, clock spurs at a block edge) differs. The second "
            "sweep is drawn in cyan and the ranges where the two disagree are shaded magenta.")
        self.btn_stitch.clicked.connect(self.stitch_start)
        self.chk_stitch_show = QtWidgets.QCheckBox("Show stitch-check overlay")
        self.chk_stitch_show.setChecked(True)
        self.chk_stitch_show.toggled.connect(self.stitch_on_overlay_toggled)
        self.stitch_label = QtWidgets.QLabel("")
        self.stitch_label.setSizePolicy(QtWidgets.QSizePolicy.Ignored, QtWidgets.QSizePolicy.Fixed)
        self.stitch_label.setMinimumWidth(0)
        row.addWidget(self.btn_stitch)
        row.addWidget(self.chk_stitch_show)
        row.addWidget(self.stitch_label, 1)

    def stitch_on_overlay_toggled(self, checked):
        has = self.stitch_curve.xData is not None and len(self.stitch_curve.xData) > 0
        self.stitch_curve.setVisible(bool(checked) and has)
        for r in self._stitch_regions:
            r.setVisible(bool(checked))

    def stitch_clear_overlay(self):
        for r in self._stitch_regions:
            try:
                self.plot.removeItem(r)
            except Exception:
                pass
        self._stitch_regions = []
        self.stitch_curve.setData([], [])
        self.stitch_curve.hide()

    def stitch_start(self):
        if self._stitch is not None:
            return
        if self._survey is not None or getattr(self, "scope_active", False):
            self.log("Stitch check: not available while a survey or the scope is running")
            return
        if self.START_FREQ == self.STOP_FREQ:
            self.log("Stitch check: pick a MODE with a frequency range first (not a single frequency)")
            return
        was_running = bool(self.sweep_thread and self.sweep_thread.is_alive())
        if was_running:
            self.stop_flag.set()
            self.sweep_thread.join(timeout=3.0)
            if self.sweep_thread.is_alive():
                self.log("Stitch check: ERROR - the running sweep did not stop; try again")
                return
        self.stop_flag.clear()
        try:
            self.init_sdr()
        except Exception as e:
            self.log(f"Stitch check: error opening the SDR: {e}")
            return
        self._stitch = {"was_running": was_running, "n_avg": self.avg_get_active_count(),
                        "rate": self.SAMPLE_RATE, "f0": float(self.START_FREQ), "f1": float(self.STOP_FREQ)}
        self.stitch_clear_overlay()
        self.survey_set_lock(True)
        self.btn_stitch.setEnabled(False)
        self.btn_scope.setEnabled(False)
        self.stitch_label.setText("running: sweep 1 of 2 (normal block layout), then sweep 2 (shifted by half a block) ...")
        self.log("=== STITCH CHECK STARTED: normal layout, then layout shifted by half a block ===")
        self.sweep_thread = threading.Thread(target=self.stitch_loop, daemon=True)
        self.sweep_thread.start()

    def stitch_loop(self):
        """Worker thread: two clean sweeps (shift 0 and 0.5), compare them, restore the normal display."""
        s = self._stitch
        result = {"aborted": False, "ranges": [], "error": ""}
        try:
            self._stitch_shift = 0.0
            c0 = self.build_centers(self.START_FREQ, self.STOP_FREQ, self.SAMPLE_RATE)
            self.settle_radio(float(c0[0]))
            runs = []
            for shift in (0.0, 0.5):
                self._stitch_shift = shift
                got = None
                for _attempt in range(3):
                    if self.stop_flag.is_set():
                        result["aborted"] = True
                        break
                    seq0 = self._pass_seq
                    self._restart_pass = False
                    self.sweep_loop()
                    lp = self._last_pass
                    if lp is not None and lp["seq"] > seq0 and lp["rate"] == self.SAMPLE_RATE and lp["gain"] == self.gain_db:
                        got = lp
                        break
                if result["aborted"]:
                    break
                if got is None:
                    raise RuntimeError("no clean sweep could be completed")
                runs.append(got)
            if not result["aborted"]:
                result.update(self.stitch_compare(runs[0], runs[1], s))
                # leave the normal-layout spectrum on the display
                self._stitch_shift = 0.0
                self.centers = self.build_centers(self.START_FREQ, self.STOP_FREQ, self.SAMPLE_RATE)
                self.freq_axis = np.asarray(runs[0]["f"], dtype=np.float64).copy()
                self.power_axis = np.asarray(runs[0]["p"], dtype=np.float64).copy()
                self.update_plot()
                self.plot_signals.markers_changed.emit()
        except Exception as e:
            import traceback
            print(f"stitch_loop: error {e!r}")
            traceback.print_exc()
            result["error"] = str(e)
        finally:
            self._stitch_shift = 0.0
            self.stitch_signals.done.emit(result)

    def stitch_compare(self, lp_a, lp_b, s):
        """Reduce both sweeps to the same frequency bins (peak in each bin) and find where they disagree."""
        f0, f1, rate = s["f0"], s["f1"], s["rate"]
        native = rate / NFFT
        npts = int(min(3000, max(200, (f1 - f0) / (6.0 * native))))
        va = self.survey_surface_vector(lp_a["f"], lp_a["p"], npts, True, f0, f1)
        vb = self.survey_surface_vector(lp_b["f"], lp_b["p"], npts, True, f0, f1)
        diff = vb - va
        thr = max(6.0, 20.0 / math.sqrt(max(1, s["n_avg"])))        # more noise scatter with little averaging
        bad = np.isfinite(diff) & (np.abs(diff) > thr)
        edges = f0 + (np.arange(npts + 1)) * (f1 - f0) / npts
        shift_was = self._stitch_shift
        self._stitch_shift = 0.0
        ca = self.build_centers(f0, f1, rate)                          # block centres of the normal layout
        self._stitch_shift = 0.5
        cb = self.build_centers(f0, f1, rate)                          # ... and of the shifted layout
        self._stitch_shift = shift_was
        ranges = []
        i = 0
        while i < npts:
            if not bad[i]:
                i += 1
                continue
            j = i
            gap = 0
            k = i
            while k < npts and gap <= 3:
                if bad[k]:
                    j = k
                    gap = 0
                else:
                    gap += 1
                k += 1
            seg = diff[i:j + 1]
            lo, hi = float(edges[i]), float(edges[j + 1])
            mid = 0.5 * (lo + hi)
            peak_d = float(seg[np.argmax(np.abs(seg))])
            cen = cb if peak_d > 0 else ca            # second layout higher: the artefact is in the shifted layout, else the normal one
            near = float(cen[np.argmin(np.abs(cen - mid))])
            ranges.append({"lo": lo, "hi": hi, "max": peak_d, "block_centre": near, "offset": mid - near,
                           "in_second": peak_d > 0})
            i = j + 1
        ranges.sort(key=lambda r: -abs(r["max"]))
        fb = np.asarray(lp_b["f"], dtype=np.float64)
        pb = np.asarray(lp_b["p"], dtype=np.float64)
        ok = np.isfinite(fb) & np.isfinite(pb) & (pb > -190.0)
        o = np.argsort(fb[ok])
        return {"ranges": ranges[:40], "n_ranges": len(ranges), "thr": thr, "npts": npts,
                "fB": fb[ok][o], "pB": pb[ok][o], "agree": float(np.mean(~bad[np.isfinite(diff)])) if np.isfinite(diff).any() else 0.0}

    def stitch_finish(self, result):
        """GUI thread: draw the overlay, report, and give the controls back."""
        s = self._stitch
        if s is None:
            return
        self.survey_set_lock(False)
        self.btn_stitch.setEnabled(True)
        self.btn_scope.setEnabled(True)
        self._stitch = None
        if result.get("error"):
            self.stitch_label.setText("stitch check failed: " + result["error"])
            self.log("Stitch check failed: " + result["error"])
        elif result.get("aborted"):
            self.stitch_label.setText("stitch check stopped")
            self.log("=== STITCH CHECK STOPPED ===")
        else:
            self.stitch_curve.setData(result["fB"], result["pB"])
            for r in result["ranges"]:
                reg = pg.LinearRegionItem(values=(r["lo"], r["hi"]), movable=False, brush=pg.mkBrush(255, 0, 255, 45))
                try:
                    for ln in reg.lines:
                        ln.setPen(pg.mkPen((255, 0, 255, 120), width=1))
                except Exception:
                    pass
                reg.setZValue(-5)
                self.plot.addItem(reg)
                self._stitch_regions.append(reg)
            self.stitch_on_overlay_toggled(self.chk_stitch_show.isChecked())
            n = result["n_ranges"]
            if n == 0:
                msg = (f"STITCH CHECK: the two block layouts agree everywhere (no bin differs by more than {result['thr']:.0f} dB) - "
                       f"the stitched spectrum is consistent; its features do not depend on the block layout.")
                self.stitch_label.setText("no disagreement between the two block layouts - the stitched spectrum is consistent")
            else:
                msg = (f"STITCH CHECK: the two block layouts disagree (by more than {result['thr']:.0f} dB) in {n} range(s) - "
                       f"features there depend on where the frequency falls in a block (receiver artefact), not on the signal.")
                self.stitch_label.setText(f"{n} range(s) differ between the two block layouts (shaded magenta) - see the debug window")
            self.log("=== " + msg + " ===")
            for r in result["ranges"][:12]:
                self.log(f"  {r['lo'] / 1e6:.3f} - {r['hi'] / 1e6:.3f} MHz: differs by up to {abs(r['max']):.0f} dB - the extra energy is in the "
                         f"{'shifted' if r['in_second'] else 'normal'} layout, in the block centred {r['block_centre'] / 1e6:.3f} MHz, "
                         f"{r['offset'] / 1e3:+.0f} kHz from its centre")
            if n > 12:
                self.log(f"  ... and {n - 12} more range(s)")
        if s["was_running"] and not result.get("aborted"):
            self.continuous_when_idle()

    # ------------------------------------------------------------------------------------
    # SINGLE FREQUENCY mode: tune the SDR to one centre frequency and show ONE spectrum of everything it
    # receives there (the whole captured bandwidth - there is nothing to stitch, so no edge masking).
    # ------------------------------------------------------------------------------------

    def single_build_ui(self, left_layout):
        self.single_mode = False
        self.single_centre_hz = 100000000
        self._pending_centre = None
        self._single_lock = threading.Lock()
        self._last_preset_index = None
        # the mode button goes ABOVE the range (preset) buttons, same size and shape as them
        first = self.mode_buttons[0]
        self.btn_single = QtWidgets.QPushButton("Single Frequency")
        self.btn_single.setCheckable(True)
        self.btn_single.setSizePolicy(first.sizePolicy())
        self.btn_single.setMinimumSize(first.minimumSize())
        self.btn_single.setToolTip("Test mode: tune to one centre frequency and show a single spectrum of the whole captured "
                                    "bandwidth. The centre frequency is set digit by digit below the gain controls. Press a "
                                    "range button to go back to the range sweeps.")
        self.btn_single.toggled.connect(self.single_on_toggled)
        main_layout = self.centralWidget().layout()
        placed = False
        for i in range(main_layout.count()):
            lay = main_layout.itemAt(i).layout()
            if lay is not None and lay.indexOf(first) >= 0:
                lay.insertWidget(lay.indexOf(first), self.btn_single)
                placed = True
                break
        if not placed:
            main_layout.addWidget(self.btn_single)
        self.mode_buttons.append(self.btn_single)       # joins the controls that are locked during a survey / the scope
        # the digit selector sits in its own row BELOW the gain row, visible only in this mode
        self.single_panel = QtWidgets.QWidget()
        prow = QtWidgets.QHBoxLayout(self.single_panel)
        prow.setContentsMargins(0, 2, 0, 2)
        lab = QtWidgets.QLabel("Centre frequency")
        self.freq_selector = DigitalFreqSelector(self.single_centre_hz)
        self.freq_selector.setToolTip("Click a digit (or use its arrows / the mouse wheel) to change it; with a digit selected "
                                       "the Up / Down keys step it, Left / Right move along, and typing 0-9 overwrites it.")
        self.freq_selector.valueChanged.connect(self.single_on_freq)
        hint = QtWidgets.QLabel("(click a digit, then arrows, wheel, Up/Down or type 0-9)")
        hint.setStyleSheet("QLabel { color: #9aa0a8; }")
        prow.addWidget(lab)
        prow.addWidget(self.freq_selector)
        prow.addWidget(hint)
        prow.addStretch()
        pos = left_layout.count()
        for i in range(left_layout.count()):
            lay = left_layout.itemAt(i).layout()
            if lay is not None and lay.indexOf(self.gain_slider) >= 0:
                pos = i + 1                              # directly under the row that holds the gain slider
                break
        left_layout.insertWidget(pos, self.single_panel)
        self.single_panel.setVisible(False)

    def single_on_toggled(self, checked):
        if checked:
            self.single_enter()
        else:
            self.single_leave()

    def single_enter(self):
        """Switch to the single-frequency test mode and start capturing at once."""
        if self._survey is not None or getattr(self, "scope_active", False) or self._stitch is not None:
            self.log("Single frequency mode is not available while a survey, the scope or a stitch check is running")
            self.btn_single.blockSignals(True)
            self.btn_single.setChecked(False)
            self.btn_single.blockSignals(False)
            return
        if self.sweep_thread and self.sweep_thread.is_alive():
            self.stop_flag.set()
            self.sweep_thread.join(timeout=1.0)
        centre = int(round(0.5 * (self.START_FREQ + self.STOP_FREQ))) if not self.single_mode else self.single_centre_hz
        centre = max(self.freq_selector._min, min(self.freq_selector._max, centre))
        self.single_mode = True
        self.single_centre_hz = centre
        self.freq_selector.setValue(centre)
        with self._single_lock:
            self._pending_centre = None
        self.START_FREQ = self.STOP_FREQ = float(centre)
        self.hf_mode = False
        self.centers = np.array([float(centre)], dtype=np.float64)
        self.current_mode_label = f"Single frequency {centre / 1e6:.6f} MHz"
        self.single_panel.setVisible(True)
        self.log(f"=== SINGLE FREQUENCY MODE: centre {centre / 1e6:.6f} MHz ===")
        self.update_status_panel()
        self.continuous_when_idle()

    def single_leave(self):
        """The mode button was switched off: go back to the last range that was selected (if any)."""
        self.single_exit()
        if self._last_preset_index is not None:
            self.set_preset_and_run(self._last_preset_index)

    def single_exit(self):
        """Leave single-frequency mode (a range button was pressed, or the mode button was switched off)."""
        if not self.single_mode:
            return
        self.single_mode = False
        self.single_panel.setVisible(False)
        self.btn_single.blockSignals(True)
        self.btn_single.setChecked(False)
        self.btn_single.blockSignals(False)
        with self._single_lock:
            self._pending_centre = None

    def single_on_freq(self, hz):
        """A digit changed. While a sweep thread owns the SDR it applies the new centre itself at the start of its
        next scan (single_apply_pending) - both START and STOP are changed there together; otherwise apply now."""
        if not self.single_mode:
            return                                       # the selector is only in use (and visible) in single-frequency mode
        hz = int(hz)
        self.single_centre_hz = hz
        self.current_mode_label = f"Single frequency {hz / 1e6:.6f} MHz"
        if self.sweep_thread and self.sweep_thread.is_alive():
            with self._single_lock:
                self._pending_centre = hz
            self._restart_pass = True                    # abandon the scan in progress; the next one starts at the new centre
        else:
            self.START_FREQ = self.STOP_FREQ = float(hz)
            self.centers = np.array([float(hz)], dtype=np.float64)
            try:
                self.draw_block_markers()
            except Exception:
                pass
            self.update_status_panel()

    def single_apply_pending(self):
        """Worker thread, before each scan: apply a centre-frequency change made on the front panel."""
        if not getattr(self, "single_mode", False):
            return
        with self._single_lock:
            hz, self._pending_centre = self._pending_centre, None
        if hz is not None:
            self.START_FREQ = self.STOP_FREQ = float(hz)
            self.update_status_panel()

    def survey_connect_signals(self):
        self.survey_signals = SurveySignals()
        self.survey_signals.done.connect(self.survey_finish)
        self.survey_signals.progress.connect(self.survey_sync_controls)
        self.btn_survey.setToolTip(
            "Automatically step through sample rates and RF gains on the current band (any width - wide "
            "bands are swept in several blocks) and write ONE HTML report: findings on sample-rate effects and "
            "gain-stage artefacts, charts and exportable data. The Sample Rate box and RF Gain slider follow the "
            "test being run. Press STOP to end early (a partial report is saved).")

    def survey_sync_controls(self):
        """GUI thread: show the settings of the survey test now running on the Sample Rate box, the RF Gain
        slider and the status panel. The survey worker thread only changes the attributes (it owns the SDR);
        the sync_* helpers move the widgets without triggering their change handlers."""
        self.sync_sample_rate_combo()
        self.sync_gain_slider()
        self.update_status_panel()

    def survey_estimate_pass_seconds(self, rate):
        """Rough time of ONE complete pass at this rate with the current Averaging setting."""
        centers = self.build_centers(self.START_FREQ, self.STOP_FREQ, rate)
        n_avg = self.avg_get_active_count()
        per_block = 0.12 + 0.03 + n_avg * (NFFT / rate + 0.004) + 0.03    # settle + flush + captures + plot
        return len(centers) * per_block

    @staticmethod
    def survey_fmt_seconds(sec):
        sec = int(round(sec))
        if sec >= 3600:
            return f"{sec // 3600} h {(sec % 3600) // 60:02d} min"
        return f"{sec // 60} min {sec % 60:02d} s"

    # ---- previous survey_open_dialog() / survey_start() (CSV outputs), kept in case of problems ----
#     def survey_open_dialog(self):
#         if self._survey is not None:
#             return
#         problem = self.survey_band_ok()
#         if problem:
#             self.log(f"Survey not started: {problem}")
#             QtWidgets.QMessageBox.information(self, "Gain / sample-rate survey", f"Survey not available: {problem}.")
#             return
#         dlg = QtWidgets.QDialog(self)
#         dlg.setWindowTitle("Gain / sample-rate survey")
#         lay = QtWidgets.QVBoxLayout(dlg)
#         lay.addWidget(QtWidgets.QLabel(
#             f"Band: {self.START_FREQ/1e6:.3f} - {self.STOP_FREQ/1e6:.3f} MHz (the current MODE), any width.\n"
#             f"Wide bands are swept in several blocks and stitched: one complete pass per rate/gain combination.\n"
#             f"The Averaging setting is used as-is."))
#         form = QtWidgets.QFormLayout()
#         gain_texts = [f"{g:.1f} dB" for g in RTL_GAIN_STEPS_DB]
#         cb_lo, cb_hi = QtWidgets.QComboBox(), QtWidgets.QComboBox()
#         cb_lo.addItems(gain_texts)
#         cb_hi.addItems(gain_texts)
#         cb_lo.setCurrentIndex(RTL_GAIN_STEPS_DB.index(12.5))
#         cb_hi.setCurrentIndex(RTL_GAIN_STEPS_DB.index(33.8))
#         sp_stride = QtWidgets.QSpinBox()
#         sp_stride.setRange(1, 10)
#         sp_stride.setValue(1)
#         sp_rep = QtWidgets.QSpinBox()
#         sp_rep.setRange(1, 5)
#         sp_rep.setValue(1)
#         sp_pts = QtWidgets.QSpinBox()
#         sp_pts.setLocale(SCOPE_LOCALE)
#         sp_pts.setRange(50, 5000)
#         sp_pts.setValue(SURVEY_SURFACE_POINTS)
#         sp_pts.setToolTip("Frequency points in each surface-chart CSV (the full-resolution data stays in the spikes "
#                            "CSV and the .npz). Excel's surface chart is happiest with a few hundred to a few thousand.")
#         cb_red = QtWidgets.QComboBox()
#         cb_red.addItem("Peak (keeps spikes)")
#         cb_red.addItem("Average power")
#         form.addRow("Gain from", cb_lo)
#         form.addRow("Gain to", cb_hi)
#         form.addRow("Use every Nth gain step", sp_stride)
#         form.addRow("Repeats per combination", sp_rep)
#         form.addRow("Surface chart frequency points", sp_pts)
#         form.addRow("Combine bins by", cb_red)
#         lay.addLayout(form)
#         lay.addWidget(QtWidgets.QLabel("Sample rates to test:"))
#         lst = QtWidgets.QListWidget()
#         for r in SAMPLE_RATE_CHOICES:
#             it = QtWidgets.QListWidgetItem(f"{r/1e6:.3f} MS/s")
#             it.setData(QtCore.Qt.UserRole, float(r))
#             it.setFlags(it.flags() | QtCore.Qt.ItemIsUserCheckable)
#             it.setCheckState(QtCore.Qt.Checked)
#             lst.addItem(it)
#         lst.setMinimumHeight(180)
#         lay.addWidget(lst)
#         info = QtWidgets.QLabel("")
#         lay.addWidget(info)
#         def chosen():
#             lo, hi = sorted((cb_lo.currentIndex(), cb_hi.currentIndex()))
#             gains = [float(g) for g in RTL_GAIN_STEPS_DB[lo:hi + 1:sp_stride.value()]]
#             rates = [lst.item(i).data(QtCore.Qt.UserRole) for i in range(lst.count())
#                      if lst.item(i).checkState() == QtCore.Qt.Checked]
#             return rates, gains, sp_rep.value()
#         def total_seconds():
#             rates, gains, reps = chosen()
#             return sum(self.survey_estimate_pass_seconds(r) for r in rates) * len(gains) * reps
#         def refresh_info(*_):
#             rates, gains, reps = chosen()
#             info.setText(f"{len(rates)} rates x {len(gains)} gains x {reps} repeat(s) = "
#                          f"{len(rates) * len(gains) * reps} passes, roughly {self.survey_fmt_seconds(total_seconds())}")
#         for wdg in (cb_lo, cb_hi):
#             wdg.currentIndexChanged.connect(refresh_info)
#         sp_stride.valueChanged.connect(refresh_info)
#         sp_rep.valueChanged.connect(refresh_info)
#         lst.itemChanged.connect(refresh_info)
#         refresh_info()
#         bb = QtWidgets.QDialogButtonBox(QtWidgets.QDialogButtonBox.Ok | QtWidgets.QDialogButtonBox.Cancel)
#         bb.accepted.connect(dlg.accept)
#         bb.rejected.connect(dlg.reject)
#         lay.addWidget(bb)
#         if dlg.exec_() != QtWidgets.QDialog.Accepted:
#             return
#         rates, gains, reps = chosen()
#         if not rates or not gains:
#             self.log("Survey: nothing selected")
#             return
#         secs = total_seconds()
#         if secs > 1800:
#             ans = QtWidgets.QMessageBox.question(
#                 self, "Gain / sample-rate survey",
#                 f"This survey will take roughly {self.survey_fmt_seconds(secs)}. Continue?\n"
#                 f"(Press STOP at any time - partial results are saved.)")
#             if ans != QtWidgets.QMessageBox.Yes:
#                 return
#         path, _ = QtWidgets.QFileDialog.getSaveFileName(
#             self, "Survey results - base file name", "rf_survey.csv", "CSV Files (*.csv)")
#         if not path:
#             return
#         self.survey_start(rates, gains, reps, path, sp_pts.value(), cb_red.currentIndex() == 0)
#     def survey_start(self, rates, gains, repeats, path, npts=SURVEY_SURFACE_POINTS, peak=True):
#         if self._survey is not None:
#             return
#         problem = self.survey_band_ok()
#         if problem:
#             self.log(f"Survey not started: {problem}")
#             return
#         if not rates or not gains:
#             self.log("Survey not started: nothing selected")
#             return
#         base = os.path.splitext(path)[0]
#         try:
#             spikes_fh = open(base + "_spikes.csv", "w", newline="")
#             summary_fh = open(base + "_summary.csv", "w", newline="")
#         except Exception as e:
#             self.log(f"Survey: cannot create result files: {e}")
#             return
#         # The survey owns the SDR on its own worker thread: stop whatever is running first.
#         was_running = bool(self.sweep_thread and self.sweep_thread.is_alive())
#         if was_running:
#             self.stop_flag.set()
#             self.sweep_thread.join(timeout=3.0)
#             if self.sweep_thread.is_alive():
#                 self.log("Survey: ERROR - the running sweep did not stop; try again")
#                 spikes_fh.close()
#                 summary_fh.close()
#                 return
#         self.stop_flag.clear()
#         try:
#             self.init_sdr()
#         except Exception as e:
#             self.log(f"Survey: error opening the SDR: {e}")
#             spikes_fh.close()
#             summary_fh.close()
#             return
#         spikes_w, summary_w = csv.writer(spikes_fh), csv.writer(summary_fh)
#         spikes_w.writerow(["repeat", "sample_rate_hz", "gain_db", "freq_hz", "level_db",
#                            "prominence_db", "baseline_db"])
#         summary_w.writerow(["repeat", "sample_rate_hz", "gain_db", "n_avg", "bins",
#                             "floor_db_median", f"spikes_over_{SURVEY_SPIKE_THRESH_DB:g}dB",
#                             "max_prominence_db", "max_spike_freq_hz", "pass_seconds"])
#         self._survey = {
#             "combos": [(r, g, k) for r in rates for g in gains for k in range(repeats)],
#             "idx": -1, "base": base, "spikes_fh": spikes_fh, "summary_fh": summary_fh,
#             "spikes_w": spikes_w, "summary_w": summary_w,
#             "surface": {}, "spectra": [], "freq_by_rate": {}, "full_bytes": 0.0, "full_warned": False,
#             "npts": int(npts), "peak": bool(peak), "f0": float(self.START_FREQ), "f1": float(self.STOP_FREQ),
#             "t_start": time.time(), "orig_rate": self.SAMPLE_RATE, "orig_gain": self.gain_db,
#             "was_running": was_running, "last_rate": None, "skipped": 0,
#         }
#         self.log(f"=== SURVEY STARTED: {len(self._survey['combos'])} passes -> {base}_*.csv/.npz ===")
#         self.survey_set_lock(True)
#         self.sweep_thread = threading.Thread(target=self.survey_loop, daemon=True)
#         self.sweep_thread.start()

    # ---- previous survey_open_dialog() (12.5 dB default, manual repeats), kept in case of problems ----
#     def survey_open_dialog(self):
#         if self._survey is not None:
#             return
#         problem = self.survey_band_ok()
#         if problem:
#             self.log(f"Survey not started: {problem}")
#             QtWidgets.QMessageBox.information(self, "Gain / sample-rate survey", f"Survey not available: {problem}.")
#             return
#         dlg = QtWidgets.QDialog(self)
#         dlg.setWindowTitle("Gain / sample-rate survey")
#         lay = QtWidgets.QVBoxLayout(dlg)
#         lay.addWidget(QtWidgets.QLabel(
#             f"Band: {self.START_FREQ/1e6:.3f} - {self.STOP_FREQ/1e6:.3f} MHz (the current MODE), any width.\n"
#             f"Wide bands are swept in several blocks and stitched: one complete pass per rate/gain combination.\n"
#             f"The Averaging setting is used as-is."))
#         form = QtWidgets.QFormLayout()
#         gain_texts = [f"{g:.1f} dB" for g in RTL_GAIN_STEPS_DB]
#         cb_lo, cb_hi = QtWidgets.QComboBox(), QtWidgets.QComboBox()
#         cb_lo.addItems(gain_texts)
#         cb_hi.addItems(gain_texts)
#         cb_lo.setCurrentIndex(RTL_GAIN_STEPS_DB.index(12.5))
#         cb_hi.setCurrentIndex(RTL_GAIN_STEPS_DB.index(33.8))
#         sp_stride = QtWidgets.QSpinBox()
#         sp_stride.setRange(1, 10)
#         sp_stride.setValue(1)
#         sp_rep = QtWidgets.QSpinBox()
#         sp_rep.setRange(1, 5)
#         sp_rep.setValue(1)
#         sp_pts = QtWidgets.QSpinBox()
#         sp_pts.setLocale(SCOPE_LOCALE)
#         sp_pts.setRange(50, 5000)
#         sp_pts.setValue(SURVEY_SURFACE_POINTS)
#         sp_pts.setToolTip("Frequency points per spectrum in the report's charts and surface-chart export (the list of "
#                            "spikes in the report keeps full resolution). A few hundred to a few thousand is plenty.")
#         cb_red = QtWidgets.QComboBox()
#         cb_red.addItem("Peak (keeps spikes)")
#         cb_red.addItem("Average power")
#         form.addRow("Gain from", cb_lo)
#         form.addRow("Gain to", cb_hi)
#         form.addRow("Use every Nth gain step", sp_stride)
#         form.addRow("Repeats per combination", sp_rep)
#         form.addRow("Chart frequency points", sp_pts)
#         form.addRow("Combine bins by", cb_red)
#         lay.addLayout(form)
#         lay.addWidget(QtWidgets.QLabel("Sample rates to test:"))
#         lst = QtWidgets.QListWidget()
#         for r in SAMPLE_RATE_CHOICES:
#             it = QtWidgets.QListWidgetItem(f"{r/1e6:.3f} MS/s")
#             it.setData(QtCore.Qt.UserRole, float(r))
#             it.setFlags(it.flags() | QtCore.Qt.ItemIsUserCheckable)
#             it.setCheckState(QtCore.Qt.Checked)
#             lst.addItem(it)
#         lst.setMinimumHeight(180)
#         lay.addWidget(lst)
#         info = QtWidgets.QLabel("")
#         lay.addWidget(info)
#         def chosen():
#             lo, hi = sorted((cb_lo.currentIndex(), cb_hi.currentIndex()))
#             gains = [float(g) for g in RTL_GAIN_STEPS_DB[lo:hi + 1:sp_stride.value()]]
#             rates = [lst.item(i).data(QtCore.Qt.UserRole) for i in range(lst.count())
#                      if lst.item(i).checkState() == QtCore.Qt.Checked]
#             return rates, gains, sp_rep.value()
#         def total_seconds():
#             rates, gains, reps = chosen()
#             return sum(self.survey_estimate_pass_seconds(r) for r in rates) * len(gains) * reps
#         def refresh_info(*_):
#             rates, gains, reps = chosen()
#             info.setText(f"{len(rates)} rates x {len(gains)} gains x {reps} repeat(s) = "
#                          f"{len(rates) * len(gains) * reps} passes, roughly {self.survey_fmt_seconds(total_seconds())}")
#         for wdg in (cb_lo, cb_hi):
#             wdg.currentIndexChanged.connect(refresh_info)
#         sp_stride.valueChanged.connect(refresh_info)
#         sp_rep.valueChanged.connect(refresh_info)
#         lst.itemChanged.connect(refresh_info)
#         refresh_info()
#         bb = QtWidgets.QDialogButtonBox(QtWidgets.QDialogButtonBox.Ok | QtWidgets.QDialogButtonBox.Cancel)
#         bb.accepted.connect(dlg.accept)
#         bb.rejected.connect(dlg.reject)
#         lay.addWidget(bb)
#         if dlg.exec_() != QtWidgets.QDialog.Accepted:
#             return
#         rates, gains, reps = chosen()
#         if not rates or not gains:
#             self.log("Survey: nothing selected")
#             return
#         secs = total_seconds()
#         if secs > 1800:
#             ans = QtWidgets.QMessageBox.question(
#                 self, "Gain / sample-rate survey",
#                 f"This survey will take roughly {self.survey_fmt_seconds(secs)}. Continue?\n"
#                 f"(Press STOP at any time - partial results are saved.)")
#             if ans != QtWidgets.QMessageBox.Yes:
#                 return
#         path, _ = QtWidgets.QFileDialog.getSaveFileName(
#             self, "Save survey report (one HTML file)", "rf_survey_report.html", "HTML report (*.html)")
#         if not path:
#             return
#         self.survey_start(rates, gains, reps, path, sp_pts.value(), cb_red.currentIndex() == 0)

    # ---- previous survey_open_dialog() (offered every rate in SAMPLE_RATE_CHOICES), kept in case of problems ----
#     def survey_open_dialog(self):
#         if self._survey is not None:
#             return
#         problem = self.survey_band_ok()
#         if problem:
#             self.log(f"Survey not started: {problem}")
#             QtWidgets.QMessageBox.information(self, "Gain / sample-rate survey", f"Survey not available: {problem}.")
#             return
#         dlg = QtWidgets.QDialog(self)
#         dlg.setWindowTitle("Gain / sample-rate survey")
#         lay = QtWidgets.QVBoxLayout(dlg)
#         lay.addWidget(QtWidgets.QLabel(
#             f"Band: {self.START_FREQ/1e6:.3f} - {self.STOP_FREQ/1e6:.3f} MHz (the current MODE), any width.\n"
#             f"Wide bands are swept in several blocks and stitched. Every rate/gain setting is scanned at least twice;\n"
#             f"if the two scans differ noticeably it is scanned again until two in a row agree.\n"
#             f"The Averaging setting is used as-is."))
#         form = QtWidgets.QFormLayout()
#         gain_texts = [f"{g:.1f} dB" for g in RTL_GAIN_STEPS_DB]
#         cb_lo, cb_hi = QtWidgets.QComboBox(), QtWidgets.QComboBox()
#         cb_lo.addItems(gain_texts)
#         cb_hi.addItems(gain_texts)
#         cb_lo.setCurrentIndex(RTL_GAIN_STEPS_DB.index(0.0))      # default lowest gain: 0 dB
#         cb_hi.setCurrentIndex(RTL_GAIN_STEPS_DB.index(33.8))
#         sp_stride = QtWidgets.QSpinBox()
#         sp_stride.setRange(1, 10)
#         sp_stride.setValue(1)
#         sp_pts = QtWidgets.QSpinBox()
#         sp_pts.setLocale(SCOPE_LOCALE)
#         sp_pts.setRange(50, 5000)
#         sp_pts.setValue(SURVEY_SURFACE_POINTS)
#         sp_pts.setToolTip("Frequency points per spectrum in the report's charts and surface-chart export (the list of "
#                            "spikes in the report keeps full resolution). A few hundred to a few thousand is plenty.")
#         cb_red = QtWidgets.QComboBox()
#         cb_red.addItem("Peak (keeps spikes)")
#         cb_red.addItem("Average power")
#         form.addRow("Gain from", cb_lo)
#         form.addRow("Gain to", cb_hi)
#         form.addRow("Use every Nth gain step", sp_stride)
#         form.addRow("Chart frequency points", sp_pts)
#         form.addRow("Combine bins by", cb_red)
#         lay.addLayout(form)
#         lay.addWidget(QtWidgets.QLabel("Sample rates to test:"))
#         lst = QtWidgets.QListWidget()
#         for r in SAMPLE_RATE_CHOICES:
#             it = QtWidgets.QListWidgetItem(f"{r/1e6:.3f} MS/s")
#             it.setData(QtCore.Qt.UserRole, float(r))
#             it.setFlags(it.flags() | QtCore.Qt.ItemIsUserCheckable)
#             it.setCheckState(QtCore.Qt.Checked)
#             lst.addItem(it)
#         lst.setMinimumHeight(180)
#         lay.addWidget(lst)
#         info = QtWidgets.QLabel("")
#         lay.addWidget(info)
#         def chosen():
#             lo, hi = sorted((cb_lo.currentIndex(), cb_hi.currentIndex()))
#             gains = [float(g) for g in RTL_GAIN_STEPS_DB[lo:hi + 1:sp_stride.value()]]
#             rates = [lst.item(i).data(QtCore.Qt.UserRole) for i in range(lst.count())
#                      if lst.item(i).checkState() == QtCore.Qt.Checked]
#             return rates, gains, 1
#         def total_seconds():
#             rates, gains, reps = chosen()
#             return sum(self.survey_estimate_pass_seconds(r) for r in rates) * len(gains) * SURVEY_SCANS_ESTIMATE
#         def refresh_info(*_):
#             rates, gains, reps = chosen()
#             info.setText(f"{len(rates)} rates x {len(gains)} gains = {len(rates) * len(gains)} settings, each scanned at least twice "
#                          f"- roughly {self.survey_fmt_seconds(total_seconds())}")
#         for wdg in (cb_lo, cb_hi):
#             wdg.currentIndexChanged.connect(refresh_info)
#         sp_stride.valueChanged.connect(refresh_info)
#         lst.itemChanged.connect(refresh_info)
#         refresh_info()
#         bb = QtWidgets.QDialogButtonBox(QtWidgets.QDialogButtonBox.Ok | QtWidgets.QDialogButtonBox.Cancel)
#         bb.accepted.connect(dlg.accept)
#         bb.rejected.connect(dlg.reject)
#         lay.addWidget(bb)
#         if dlg.exec_() != QtWidgets.QDialog.Accepted:
#             return
#         rates, gains, reps = chosen()
#         if not rates or not gains:
#             self.log("Survey: nothing selected")
#             return
#         secs = total_seconds()
#         if secs > 1800:
#             ans = QtWidgets.QMessageBox.question(
#                 self, "Gain / sample-rate survey",
#                 f"This survey will take roughly {self.survey_fmt_seconds(secs)}. Continue?\n"
#                 f"(Press STOP at any time - partial results are saved.)")
#             if ans != QtWidgets.QMessageBox.Yes:
#                 return
#         path, _ = QtWidgets.QFileDialog.getSaveFileName(
#             self, "Save survey report (one HTML file)", "rf_survey_report.html", "HTML report (*.html)")
#         if not path:
#             return
#         self.survey_start(rates, gains, reps, path, sp_pts.value(), cb_red.currentIndex() == 0)
#     # ---- previous survey_start(), kept in case of problems ----
# #     def survey_start(self, rates, gains, repeats, path, npts=SURVEY_SURFACE_POINTS, peak=True):
# #         if self._survey is not None:
# #             return
# #         problem = self.survey_band_ok()
# #         if problem:
# #             self.log(f"Survey not started: {problem}")
# #             return
# #         if not rates or not gains:
# #             self.log("Survey not started: nothing selected")
# #             return
# #         if not path.lower().endswith(".html"):
# #             path = os.path.splitext(path)[0] + ".html"
# #         try:
# #             with open(path, "w", encoding="utf-8") as fh:
# #                 fh.write("<html><body style='font-family:sans-serif'>Survey in progress - this file is replaced by "
# #                          "the report while the survey runs.</body></html>")
# #         except Exception as e:
# #             self.log(f"Survey: cannot create the report file: {e}")
# #             return
# #         # The survey owns the SDR on its own worker thread: stop whatever is running first.
# #         was_running = bool(self.sweep_thread and self.sweep_thread.is_alive())
# #         if was_running:
# #             self.stop_flag.set()
# #             self.sweep_thread.join(timeout=3.0)
# #             if self.sweep_thread.is_alive():
# #                 self.log("Survey: ERROR - the running sweep did not stop; try again")
# #                 return
# #         self.stop_flag.clear()
# #         try:
# #             self.init_sdr()
# #         except Exception as e:
# #             self.log(f"Survey: error opening the SDR: {e}")
# #             return
# #         self._survey = {
# #             "combos": [(r, g, k) for r in rates for g in gains for k in range(repeats)],
# #             "idx": -1, "report_path": path, "passes": [], "last_report": time.time(),
# #             "freq_by_rate": {}, "full_bytes": 0.0, "full_warned": False,
# #             "npts": int(npts), "peak": bool(peak), "f0": float(self.START_FREQ), "f1": float(self.STOP_FREQ),
# #             "t_start": time.time(), "orig_rate": self.SAMPLE_RATE, "orig_gain": self.gain_db,
# #             "was_running": was_running, "last_rate": None, "skipped": 0,
# #         }
# #         self.log(f"=== SURVEY STARTED: {len(self._survey['combos'])} passes -> {path} ===")
# #         self.survey_set_lock(True)
# #         self.sweep_thread = threading.Thread(target=self.survey_loop, daemon=True)
# #         self.sweep_thread.start()
# #     # ---- previous survey_loop(), kept in case of problems ----
# # #     def survey_loop(self):
# # #         """
# # #         Survey worker thread. For every rate/gain combination: set the live settings (plain
# # #         attributes - this thread owns the SDR), run ONE complete pass with the normal sweep_loop()
# # #         and record it. Because the settings change only between passes, every pass is clean - no
# # #         pass is wasted on a half-old/half-new setting, which matters on wide bands where a pass
# # #         takes tens of seconds. Reports back to the GUI thread with survey_signals.done.
# # #         """
# # #         s = self._survey
# # #         aborted = False
# # #         try:
# # #             rate0, gain0, _rep0 = s["combos"][0]
# # #             self.SAMPLE_RATE = rate0
# # #             self.gain_db = gain0
# # #             c0 = self.build_centers(self.START_FREQ, self.STOP_FREQ, rate0)
# # #             self.sdr.sample_rate = rate0
# # #             self._applied_sample_rate = rate0
# # #             self.settle_radio(float(c0[0]))
# # #             for idx, (rate, gain, rep) in enumerate(s["combos"]):
# # #                 if self.stop_flag.is_set():
# # #                     aborted = True
# # #                     break
# # #                 s["idx"] = idx
# # #                 self.SAMPLE_RATE = rate
# # #                 self.gain_db = gain
# # #                 n_blocks = len(self.build_centers(self.START_FREQ, self.STOP_FREQ, rate))
# # #                 # A very short pass right after a rate change can still be a start-up transient: throw one
# # #                 # away. On wide bands every block already has its own settling time, so don't waste a pass.
# # #                 discard = 1 if (rate != s["last_rate"] and n_blocks <= 3) else 0
# # #                 s["last_rate"] = rate
# # #                 recorded = False
# # #                 for _attempt in range(discard + 3):
# # #                     if self.stop_flag.is_set():
# # #                         aborted = True
# # #                         break
# # #                     seq0 = self._pass_seq
# # #                     self._restart_pass = False
# # #                     self.sweep_loop()                       # one complete pass at these settings
# # #                     lp = self._last_pass
# # #                     if lp is None or lp["seq"] <= seq0 or lp["rate"] != rate or lp["gain"] != gain:
# # #                         continue                            # not a clean pass: try again
# # #                     if discard > 0:
# # #                         discard -= 1
# # #                         continue
# # #                     self.survey_record(lp)
# # #                     recorded = True
# # #                     break
# # #                 if aborted:
# # #                     break
# # #                 if not recorded:
# # #                     s["skipped"] += 1
# # #                     self.log(f"Survey: no clean pass at {rate/1e6:.3f} MS/s / {gain:.1f} dB - skipped")
# # #         except Exception as e:
# # #             import traceback
# # #             print(f"survey_loop: error {e!r}")
# # #             traceback.print_exc()
# # #             aborted = True
# # #         finally:
# # #             self.SAMPLE_RATE = s["orig_rate"]               # put the operator's settings back
# # #             self.gain_db = s["orig_gain"]
# # #             self.survey_signals.done.emit(bool(aborted))

    def survey_open_dialog(self):
        if self._survey is not None:
            return
        problem = self.survey_band_ok()
        if problem:
            self.log(f"Survey not started: {problem}")
            QtWidgets.QMessageBox.information(self, "Gain / sample-rate survey", f"Survey not available: {problem}.")
            return
        dlg = QtWidgets.QDialog(self)
        dlg.setWindowTitle("Gain / sample-rate survey")
        lay = QtWidgets.QVBoxLayout(dlg)
        lay.addWidget(QtWidgets.QLabel(
            f"Band: {self.START_FREQ/1e6:.3f} - {self.STOP_FREQ/1e6:.3f} MHz (the current MODE), any width.\n"
            f"Wide bands are swept in several blocks and stitched. Every rate/gain setting is scanned at least twice;\n"
            f"if the two scans differ noticeably it is scanned again until two in a row agree.\n"
            f"The Averaging setting is used as-is."))

        form = QtWidgets.QFormLayout()
        gain_texts = [f"{g:.1f} dB" for g in RTL_GAIN_STEPS_DB]
        cb_lo, cb_hi = QtWidgets.QComboBox(), QtWidgets.QComboBox()
        cb_lo.addItems(gain_texts)
        cb_hi.addItems(gain_texts)
        cb_lo.setCurrentIndex(RTL_GAIN_STEPS_DB.index(0.0))      # default lowest gain: 0 dB
        cb_hi.setCurrentIndex(RTL_GAIN_STEPS_DB.index(33.8))
        sp_stride = QtWidgets.QSpinBox()
        sp_stride.setRange(1, 10)
        sp_stride.setValue(1)
        sp_pts = QtWidgets.QSpinBox()
        sp_pts.setLocale(SCOPE_LOCALE)
        sp_pts.setRange(50, 5000)
        sp_pts.setValue(SURVEY_SURFACE_POINTS)
        sp_pts.setToolTip("Frequency points per spectrum in the report's charts and surface-chart export (the list of "
                           "spikes in the report keeps full resolution). A few hundred to a few thousand is plenty.")
        cb_red = QtWidgets.QComboBox()
        cb_red.addItem("Peak (keeps spikes)")
        cb_red.addItem("Average power")
        form.addRow("Gain from", cb_lo)
        form.addRow("Gain to", cb_hi)
        form.addRow("Use every Nth gain step", sp_stride)
        form.addRow("Chart frequency points", sp_pts)
        form.addRow("Combine bins by", cb_red)
        lay.addLayout(form)

        lay.addWidget(QtWidgets.QLabel("Sample rates to test:"))
        lst = QtWidgets.QListWidget()
        for r in SAMPLE_RATE_RECOMMENDED:                 # (previous: SAMPLE_RATE_CHOICES)
            it = QtWidgets.QListWidgetItem(f"{r/1e6:.3f} MS/s")
            it.setData(QtCore.Qt.UserRole, float(r))
            it.setFlags(it.flags() | QtCore.Qt.ItemIsUserCheckable)
            it.setCheckState(QtCore.Qt.Checked)
            lst.addItem(it)
        lst.setMinimumHeight(180)
        lay.addWidget(lst)

        info = QtWidgets.QLabel("")
        lay.addWidget(info)

        def chosen():
            lo, hi = sorted((cb_lo.currentIndex(), cb_hi.currentIndex()))
            gains = [float(g) for g in RTL_GAIN_STEPS_DB[lo:hi + 1:sp_stride.value()]]
            rates = [lst.item(i).data(QtCore.Qt.UserRole) for i in range(lst.count())
                     if lst.item(i).checkState() == QtCore.Qt.Checked]
            return rates, gains, 1

        def total_seconds():
            rates, gains, reps = chosen()
            return sum(self.survey_estimate_pass_seconds(r) for r in rates) * len(gains) * SURVEY_SCANS_ESTIMATE

        def refresh_info(*_):
            rates, gains, reps = chosen()
            info.setText(f"{len(rates)} rates x {len(gains)} gains = {len(rates) * len(gains)} settings, each scanned at least twice "
                         f"- roughly {self.survey_fmt_seconds(total_seconds())}")

        for wdg in (cb_lo, cb_hi):
            wdg.currentIndexChanged.connect(refresh_info)
        sp_stride.valueChanged.connect(refresh_info)
        lst.itemChanged.connect(refresh_info)
        refresh_info()

        bb = QtWidgets.QDialogButtonBox(QtWidgets.QDialogButtonBox.Ok | QtWidgets.QDialogButtonBox.Cancel)
        bb.accepted.connect(dlg.accept)
        bb.rejected.connect(dlg.reject)
        lay.addWidget(bb)

        if dlg.exec_() != QtWidgets.QDialog.Accepted:
            return
        rates, gains, reps = chosen()
        if not rates or not gains:
            self.log("Survey: nothing selected")
            return
        secs = total_seconds()
        if secs > 1800:
            ans = QtWidgets.QMessageBox.question(
                self, "Gain / sample-rate survey",
                f"This survey will take roughly {self.survey_fmt_seconds(secs)}. Continue?\n"
                f"(Press STOP at any time - partial results are saved.)")
            if ans != QtWidgets.QMessageBox.Yes:
                return
        path, _ = QtWidgets.QFileDialog.getSaveFileName(
            self, "Save survey report (one HTML file)", "rf_survey_report.html", "HTML report (*.html)")
        if not path:
            return
        self.survey_start(rates, gains, reps, path, sp_pts.value(), cb_red.currentIndex() == 0)

    # ---- previous survey_start(), kept in case of problems ----
#     def survey_start(self, rates, gains, repeats, path, npts=SURVEY_SURFACE_POINTS, peak=True):
#         if self._survey is not None:
#             return
#         problem = self.survey_band_ok()
#         if problem:
#             self.log(f"Survey not started: {problem}")
#             return
#         if not rates or not gains:
#             self.log("Survey not started: nothing selected")
#             return
#         if not path.lower().endswith(".html"):
#             path = os.path.splitext(path)[0] + ".html"
#         try:
#             with open(path, "w", encoding="utf-8") as fh:
#                 fh.write("<html><body style='font-family:sans-serif'>Survey in progress - this file is replaced by "
#                          "the report while the survey runs.</body></html>")
#         except Exception as e:
#             self.log(f"Survey: cannot create the report file: {e}")
#             return
#         # The survey owns the SDR on its own worker thread: stop whatever is running first.
#         was_running = bool(self.sweep_thread and self.sweep_thread.is_alive())
#         if was_running:
#             self.stop_flag.set()
#             self.sweep_thread.join(timeout=3.0)
#             if self.sweep_thread.is_alive():
#                 self.log("Survey: ERROR - the running sweep did not stop; try again")
#                 return
#         self.stop_flag.clear()
#         try:
#             self.init_sdr()
#         except Exception as e:
#             self.log(f"Survey: error opening the SDR: {e}")
#             return
#         self._survey = {
#             "combos": [(r, g, k) for r in rates for g in gains for k in range(repeats)],
#             "idx": -1, "report_path": path, "passes": [], "last_report": time.time(),
#             "freq_by_rate": {}, "full_bytes": 0.0, "full_warned": False,
#             "npts": int(npts), "peak": bool(peak), "f0": float(self.START_FREQ), "f1": float(self.STOP_FREQ),
#             "t_start": time.time(), "orig_rate": self.SAMPLE_RATE, "orig_gain": self.gain_db,
#             "was_running": was_running, "last_rate": None, "skipped": 0,
#         }
#         self.log(f"=== SURVEY STARTED: {len(self._survey['combos'])} passes -> {path} ===")
#         self.survey_set_lock(True)
#         self.sweep_thread = threading.Thread(target=self.survey_loop, daemon=True)
#         self.sweep_thread.start()
#     # ---- previous survey_loop(), kept in case of problems ----
# #     def survey_loop(self):
# #         """
# #         Survey worker thread. For every rate/gain combination: set the live settings (plain
# #         attributes - this thread owns the SDR), run ONE complete pass with the normal sweep_loop()
# #         and record it. Because the settings change only between passes, every pass is clean - no
# #         pass is wasted on a half-old/half-new setting, which matters on wide bands where a pass
# #         takes tens of seconds. Reports back to the GUI thread with survey_signals.done.
# #         """
# #         s = self._survey
# #         aborted = False
# #         try:
# #             rate0, gain0, _rep0 = s["combos"][0]
# #             self.SAMPLE_RATE = rate0
# #             self.gain_db = gain0
# #             c0 = self.build_centers(self.START_FREQ, self.STOP_FREQ, rate0)
# #             self.sdr.sample_rate = rate0
# #             self._applied_sample_rate = rate0
# #             self.settle_radio(float(c0[0]))
# #             for idx, (rate, gain, rep) in enumerate(s["combos"]):
# #                 if self.stop_flag.is_set():
# #                     aborted = True
# #                     break
# #                 s["idx"] = idx
# #                 self.SAMPLE_RATE = rate
# #                 self.gain_db = gain
# #                 n_blocks = len(self.build_centers(self.START_FREQ, self.STOP_FREQ, rate))
# #                 # A very short pass right after a rate change can still be a start-up transient: throw one
# #                 # away. On wide bands every block already has its own settling time, so don't waste a pass.
# #                 discard = 1 if (rate != s["last_rate"] and n_blocks <= 3) else 0
# #                 s["last_rate"] = rate
# #                 recorded = False
# #                 for _attempt in range(discard + 3):
# #                     if self.stop_flag.is_set():
# #                         aborted = True
# #                         break
# #                     seq0 = self._pass_seq
# #                     self._restart_pass = False
# #                     self.sweep_loop()                       # one complete pass at these settings
# #                     lp = self._last_pass
# #                     if lp is None or lp["seq"] <= seq0 or lp["rate"] != rate or lp["gain"] != gain:
# #                         continue                            # not a clean pass: try again
# #                     if discard > 0:
# #                         discard -= 1
# #                         continue
# #                     self.survey_record(lp)
# #                     recorded = True
# #                     break
# #                 if aborted:
# #                     break
# #                 if not recorded:
# #                     s["skipped"] += 1
# #                     self.log(f"Survey: no clean pass at {rate/1e6:.3f} MS/s / {gain:.1f} dB - skipped")
# #         except Exception as e:
# #             import traceback
# #             print(f"survey_loop: error {e!r}")
# #             traceback.print_exc()
# #             aborted = True
# #         finally:
# #             self.SAMPLE_RATE = s["orig_rate"]               # put the operator's settings back
# #             self.gain_db = s["orig_gain"]
# #             self.survey_signals.done.emit(bool(aborted))

    def survey_start(self, rates, gains, repeats, path, npts=SURVEY_SURFACE_POINTS, peak=True):
        if self._survey is not None:
            return
        problem = self.survey_band_ok()
        if problem:
            self.log(f"Survey not started: {problem}")
            return
        if not rates or not gains:
            self.log("Survey not started: nothing selected")
            return
        if not path.lower().endswith(".html"):
            path = os.path.splitext(path)[0] + ".html"
        try:
            with open(path, "w", encoding="utf-8") as fh:
                fh.write("<html><body style='font-family:sans-serif'>Survey in progress - this file is replaced by "
                         "the report while the survey runs.</body></html>")
        except Exception as e:
            self.log(f"Survey: cannot create the report file: {e}")
            return
        # The survey owns the SDR on its own worker thread: stop whatever is running first.
        was_running = bool(self.sweep_thread and self.sweep_thread.is_alive())
        if was_running:
            self.stop_flag.set()
            self.sweep_thread.join(timeout=3.0)
            if self.sweep_thread.is_alive():
                self.log("Survey: ERROR - the running sweep did not stop; try again")
                return
        self.stop_flag.clear()
        try:
            self.init_sdr()
        except Exception as e:
            self.log(f"Survey: error opening the SDR: {e}")
            return
        self._survey = {
            "combos": [(r, g, 0) for r in rates for g in gains],
            "idx": -1, "report_path": path, "passes": [], "combo_info": [], "last_report": time.time(),
            "freq_by_rate": {}, "full_bytes": 0.0, "full_warned": False,
            "npts": int(npts), "peak": bool(peak), "f0": float(self.START_FREQ), "f1": float(self.STOP_FREQ),
            "t_start": time.time(), "orig_rate": self.SAMPLE_RATE, "orig_gain": self.gain_db,
            "was_running": was_running, "last_rate": None, "skipped": 0,
        }
        self.log(f"=== SURVEY STARTED: {len(self._survey['combos'])} passes -> {path} ===")
        self.survey_set_lock(True)
        self.sweep_thread = threading.Thread(target=self.survey_loop, daemon=True)
        self.sweep_thread.start()

    # ---- previous survey_loop(), kept in case of problems ----
#     def survey_loop(self):
#         """
#         Survey worker thread. For every rate/gain combination: set the live settings (plain
#         attributes - this thread owns the SDR), run ONE complete pass with the normal sweep_loop()
#         and record it. Because the settings change only between passes, every pass is clean - no
#         pass is wasted on a half-old/half-new setting, which matters on wide bands where a pass
#         takes tens of seconds. Reports back to the GUI thread with survey_signals.done.
#         """
#         s = self._survey
#         aborted = False
#         try:
#             rate0, gain0, _rep0 = s["combos"][0]
#             self.SAMPLE_RATE = rate0
#             self.gain_db = gain0
#             c0 = self.build_centers(self.START_FREQ, self.STOP_FREQ, rate0)
#             self.sdr.sample_rate = rate0
#             self._applied_sample_rate = rate0
#             self.settle_radio(float(c0[0]))
#             for idx, (rate, gain, rep) in enumerate(s["combos"]):
#                 if self.stop_flag.is_set():
#                     aborted = True
#                     break
#                 s["idx"] = idx
#                 self.SAMPLE_RATE = rate
#                 self.gain_db = gain
#                 n_blocks = len(self.build_centers(self.START_FREQ, self.STOP_FREQ, rate))
#                 # A very short pass right after a rate change can still be a start-up transient: throw one
#                 # away. On wide bands every block already has its own settling time, so don't waste a pass.
#                 discard = 1 if (rate != s["last_rate"] and n_blocks <= 3) else 0
#                 s["last_rate"] = rate
#                 recorded = False
#                 for _attempt in range(discard + 3):
#                     if self.stop_flag.is_set():
#                         aborted = True
#                         break
#                     seq0 = self._pass_seq
#                     self._restart_pass = False
#                     self.sweep_loop()                       # one complete pass at these settings
#                     lp = self._last_pass
#                     if lp is None or lp["seq"] <= seq0 or lp["rate"] != rate or lp["gain"] != gain:
#                         continue                            # not a clean pass: try again
#                     if discard > 0:
#                         discard -= 1
#                         continue
#                     self.survey_record(lp)
#                     recorded = True
#                     break
#                 if aborted:
#                     break
#                 if not recorded:
#                     s["skipped"] += 1
#                     self.log(f"Survey: no clean pass at {rate/1e6:.3f} MS/s / {gain:.1f} dB - skipped")
#         except Exception as e:
#             import traceback
#             print(f"survey_loop: error {e!r}")
#             traceback.print_exc()
#             aborted = True
#         finally:
#             self.SAMPLE_RATE = s["orig_rate"]               # put the operator's settings back
#             self.gain_db = s["orig_gain"]
#             self.survey_signals.done.emit(bool(aborted))

    # ---- previous survey_loop() (one scan per setting), kept in case of problems ----
#     def survey_loop(self):
#         """
#         Survey worker thread. For every rate/gain combination: set the live settings (plain
#         attributes - this thread owns the SDR), run ONE complete pass with the normal sweep_loop()
#         and record it. Because the settings change only between passes, every pass is clean - no
#         pass is wasted on a half-old/half-new setting, which matters on wide bands where a pass
#         takes tens of seconds. Reports back to the GUI thread with survey_signals.done.
#         """
#         s = self._survey
#         aborted = False
#         try:
#             rate0, gain0, _rep0 = s["combos"][0]
#             self.SAMPLE_RATE = rate0
#             self.gain_db = gain0
#             self.survey_signals.progress.emit()             # controls show the first test
#             c0 = self.build_centers(self.START_FREQ, self.STOP_FREQ, rate0)
#             self.sdr.sample_rate = rate0
#             self._applied_sample_rate = rate0
#             self.settle_radio(float(c0[0]))
#             for idx, (rate, gain, rep) in enumerate(s["combos"]):
#                 if self.stop_flag.is_set():
#                     aborted = True
#                     break
#                 s["idx"] = idx
#                 self.SAMPLE_RATE = rate
#                 self.gain_db = gain
#                 self.survey_signals.progress.emit()         # Sample Rate box + RF Gain slider follow the test
#                 n_blocks = len(self.build_centers(self.START_FREQ, self.STOP_FREQ, rate))
#                 # A very short pass right after a rate change can still be a start-up transient: throw one
#                 # away. On wide bands every block already has its own settling time, so don't waste a pass.
#                 discard = 1 if (rate != s["last_rate"] and n_blocks <= 3) else 0
#                 s["last_rate"] = rate
#                 recorded = False
#                 for _attempt in range(discard + 3):
#                     if self.stop_flag.is_set():
#                         aborted = True
#                         break
#                     seq0 = self._pass_seq
#                     self._restart_pass = False
#                     self.sweep_loop()                       # one complete pass at these settings
#                     lp = self._last_pass
#                     if lp is None or lp["seq"] <= seq0 or lp["rate"] != rate or lp["gain"] != gain:
#                         continue                            # not a clean pass: try again
#                     if discard > 0:
#                         discard -= 1
#                         continue
#                     self.survey_record(lp)
#                     recorded = True
#                     break
#                 if aborted:
#                     break
#                 if not recorded:
#                     s["skipped"] += 1
#                     self.log(f"Survey: no clean pass at {rate/1e6:.3f} MS/s / {gain:.1f} dB - skipped")
#         except Exception as e:
#             import traceback
#             print(f"survey_loop: error {e!r}")
#             traceback.print_exc()
#             aborted = True
#         finally:
#             self.SAMPLE_RATE = s["orig_rate"]               # put the operator's settings back
#             self.gain_db = s["orig_gain"]
#             self.survey_signals.done.emit(bool(aborted))

    def survey_loop(self):
        """
        Survey worker thread. For every rate/gain setting: set the live settings (plain attributes -
        this thread owns the SDR), then scan the whole band with the normal sweep_loop() AT LEAST
        TWICE. If two scans in a row are noticeably different (see survey_compare_scans) another scan is
        made, and so on until two consecutive scans agree (at most SURVEY_MAX_SCANS), so the result for
        every setting is repeatable. Settings that needed more than two scans, or never repeated, are
        recorded for the report. Reports back to the GUI thread with survey_signals.done.
        """
        s = self._survey
        aborted = False
        try:
            rate0, gain0, _rep0 = s["combos"][0]
            self.SAMPLE_RATE = rate0
            self.gain_db = gain0
            self.survey_signals.progress.emit()             # controls show the first test
            c0 = self.build_centers(self.START_FREQ, self.STOP_FREQ, rate0)
            self.sdr.sample_rate = rate0
            self._applied_sample_rate = rate0
            self.settle_radio(float(c0[0]))
            for idx, (rate, gain, _rep) in enumerate(s["combos"]):
                if self.stop_flag.is_set():
                    aborted = True
                    break
                s["idx"] = idx
                self.SAMPLE_RATE = rate
                self.gain_db = gain
                self.survey_signals.progress.emit()         # Sample Rate box + RF Gain slider follow the test
                info = {"rate": rate, "gain": gain, "scans": 0, "converged": False, "comparisons": [], "unclean": 0}
                recs = []
                prev = None
                attempts = 0
                while len(recs) < SURVEY_MAX_SCANS and attempts < SURVEY_MAX_SCANS + 3:
                    if self.stop_flag.is_set():
                        aborted = True
                        break
                    attempts += 1
                    seq0 = self._pass_seq
                    self._restart_pass = False
                    self.sweep_loop()                       # one complete scan at these settings
                    lp = self._last_pass
                    if lp is None or lp["seq"] <= seq0 or lp["rate"] != rate or lp["gain"] != gain:
                        info["unclean"] += 1                # not a clean scan (stopped / restarted / gain changed): try again
                        continue
                    rec = self.survey_record(lp, len(recs) + 1, rate, gain)
                    recs.append(rec)
                    if prev is not None:
                        cmpd = self.survey_compare_scans(prev[0], prev[1], rec, lp)
                        info["comparisons"].append(cmpd)
                        if cmpd["same"]:
                            info["converged"] = True
                            prev[0]["accepted"] = True
                            rec["accepted"] = True
                            break
                    prev = (rec, lp)
                info["scans"] = len(recs)
                if aborted:
                    break
                if recs and not info["converged"]:
                    for rec in recs[-2:]:                   # never repeated: use the last two, flagged in the report
                        rec["accepted"] = True
                if not recs:
                    self.log(f"Survey: no clean scan at {rate/1e6:.3f} MS/s / {gain:.1f} dB - skipped")
                    s["skipped"] += 1
                else:
                    s["combo_info"].append(info)
                    if info["scans"] > 2 or not info["converged"]:
                        self.log(f"Survey: {rate/1e6:.3f} MS/s / {gain:.1f} dB " +
                                 (f"needed {info['scans']} scans to repeat" if info["converged"] else
                                  f"did NOT repeat after {info['scans']} scans"))
        except Exception as e:
            import traceback
            print(f"survey_loop: error {e!r}")
            traceback.print_exc()
            aborted = True
        finally:
            self.SAMPLE_RATE = s["orig_rate"]               # put the operator's settings back
            self.gain_db = s["orig_gain"]
            self.survey_signals.done.emit(bool(aborted))

    @staticmethod
    def survey_surface_vector(f, p, npts, peak, f0, f1):
        """Reduce one stitched spectrum to npts equal frequency bins between f0 and f1 (max or
        power-average of the samples in each bin); empty bins are filled by interpolation."""
        f = np.asarray(f, dtype=np.float64)
        p = np.asarray(p, dtype=np.float64)
        ok = np.isfinite(f) & np.isfinite(p) & (p > -190.0) & (f >= f0) & (f <= f1)
        f, p = f[ok], p[ok]
        if f.size == 0:
            return np.full(npts, np.nan)
        order = np.argsort(f)
        f, p = f[order], p[order]
        idx = np.minimum(((f - f0) / (f1 - f0) * npts).astype(np.int64), npts - 1)
        starts = np.concatenate(([0], np.flatnonzero(np.diff(idx)) + 1))
        if peak:
            red = np.maximum.reduceat(p, starts)
        else:
            lin = 10.0 ** (p / 10.0)
            cnt = np.diff(np.concatenate((starts, [p.size])))
            red = 10.0 * np.log10(np.add.reduceat(lin, starts) / cnt)
        out = np.full(npts, np.nan)
        out[idx[starts]] = red
        nan = np.isnan(out)
        if nan.any() and (~nan).any():
            out[nan] = np.interp(np.flatnonzero(nan), np.flatnonzero(~nan), out[~nan])
        return out

    def survey_next_combo(self):
        s = self._survey
        s["idx"] += 1
        if s["idx"] >= len(s["combos"]):
            self.survey_finish(False)
            return
        rate, gain, _rep = s["combos"][s["idx"]]
        self.survey_apply(rate, gain)
        # The first pass after the survey starts or the sample rate changes is a warm-up and
        # is thrown away (the first pass of the first survey looked like a start-up transient).
        s["discard"] = 1 if rate != s.get("last_rate") else 0
        s["last_rate"] = rate
        s["seq_floor"] = self._pass_seq
        s["t_set"] = time.time()
        s["timeout"] = max(90.0, 8.0 * s["pass_dur"])

    def survey_apply(self, rate, gain):
        """Drive the normal live controls (the sweep thread applies them to the hardware)."""
        if self.SAMPLE_RATE != rate:
            i = self.sample_rate_combo.findData(rate)
            if i >= 0:
                self.sample_rate_combo.setCurrentIndex(i)     # runs sample_rate_on_changed()
            else:
                self.SAMPLE_RATE = rate
        if self.gain_db != gain:
            self.gain_db = gain
            self.sync_gain_slider()
            self.update_status_panel()

    def survey_tick(self):
        s = self._survey
        if s is None:
            self._survey_timer.stop()
            return
        alive = bool(self.sweep_thread and self.sweep_thread.is_alive())
        if self.stop_flag.is_set() or (not alive and time.time() - s["t_start"] > 3.0):
            self.survey_finish(True)
            return
        rate, gain, _rep = s["combos"][s["idx"]]
        lp = self._last_pass
        if (lp is not None and lp["seq"] > s["seq_floor"]
                and lp["rate"] == rate and lp["gain"] == gain):
            if s.get("discard", 0) > 0:
                s["discard"] -= 1                 # warm-up pass: ignore it
                s["seq_floor"] = lp["seq"]
                s["t_set"] = time.time()
                return
            self.survey_record(lp)
            self.survey_next_combo()
        elif time.time() - s["t_set"] > s["timeout"]:
            s["skipped"] += 1
            self.log(f"Survey: no clean pass at {rate/1e6:.3f} MS/s / {gain:.1f} dB "
                     f"within {s['timeout']:.0f} s - skipped")
            self.survey_next_combo()

    # ---- previous survey_record() / survey_finish(), kept in case of problems ----
#     def survey_record(self, lp):
#         s = self._survey
#         rate, gain, rep = s["combos"][s["idx"]]
#         f, p, floor, spikes = self.survey_analyse(lp["f"], lp["p"], SURVEY_SPIKE_THRESH_DB)
#         s["pass_dur"] = lp["dur"]
#         for fr, lv, pr, bs in spikes:
#             s["spikes_w"].writerow([rep, f"{rate:.0f}", f"{gain:.1f}", f"{fr:.1f}",
#                                     f"{lv:.2f}", f"{pr:.2f}", f"{bs:.2f}"])
#         top = max(spikes, key=lambda x: x[2]) if spikes else None
#         s["summary_w"].writerow([rep, f"{rate:.0f}", f"{gain:.1f}", lp["n_avg"], len(p),
#                                  f"{floor:.2f}", len(spikes),
#                                  f"{top[2]:.2f}" if top else "", f"{top[0]:.1f}" if top else "",
#                                  f"{lp['dur']:.1f}"])
#         s["spikes_fh"].flush()
#         s["summary_fh"].flush()
#         s["spectra"].append((rate, gain, rep, f, p.astype(np.float32)))
#         n_done, n_all = s["idx"] + 1, len(s["combos"])
#         elapsed = time.time() - s["t_start"]
#         eta = elapsed / n_done * (n_all - n_done)
#         self.log(f"Survey {n_done}/{n_all}: {rate/1e6:.3f} MS/s, {gain:.1f} dB -> "
#                  f"{len(spikes)} spikes (ETA {int(eta // 60)}m{int(eta % 60):02d}s)")
#     def survey_finish(self, aborted):
#         s = self._survey
#         if s is None:
#             return
#         self._survey_timer.stop()
#         for k in ("spikes_fh", "summary_fh"):
#             try:
#                 s[k].close()
#             except Exception:
#                 pass
#         specs = s["spectra"]
#         if specs:
#             try:
#                 lengths = np.array([len(x[3]) for x in specs])
#                 np.savez_compressed(
#                     s["base"] + "_spectra.npz",
#                     rate_hz=np.array([x[0] for x in specs]),
#                     gain_db=np.array([x[1] for x in specs]),
#                     repeat=np.array([x[2] for x in specs]),
#                     start=np.concatenate(([0], np.cumsum(lengths)[:-1])),
#                     length=lengths,
#                     freq_hz=np.concatenate([x[3] for x in specs]),
#                     level_db=np.concatenate([x[4] for x in specs]))
#             except Exception as e:
#                 self.log(f"Survey: could not write spectra file: {e}")
#         # put the operator's settings back
#         self.survey_apply(s["orig_rate"], s["orig_gain"])
#         self.survey_set_lock(False)
#         if not s["was_running"] and not self.stop_flag.is_set():
#             self.stop_sweep()
#         self._survey = None
#         self.log(f"=== SURVEY {'STOPPED EARLY' if aborted else 'FINISHED'}: {len(specs)} passes saved"
#                  f"{', ' + str(s['skipped']) + ' skipped' if s['skipped'] else ''} -> "
#                  f"{s['base']}_spikes.csv / _summary.csv / _spectra.npz ===")

    # ---- previous survey_record() / survey_write_surfaces() / survey_finish() (CSV outputs), kept in case of problems ----
#     def survey_record(self, lp):
#         """Analyse and store one finished pass (runs on the survey worker thread)."""
#         s = self._survey
#         rate, gain, rep = s["combos"][s["idx"]]
#         f, p, floor, spikes = self.survey_analyse(lp["f"], lp["p"], SURVEY_SPIKE_THRESH_DB)
#         for fr, lv, pr, bs in spikes:
#             s["spikes_w"].writerow([rep, f"{rate:.0f}", f"{gain:.1f}", f"{fr:.1f}",
#                                     f"{lv:.2f}", f"{pr:.2f}", f"{bs:.2f}"])
#         top = max(spikes, key=lambda x: x[2]) if spikes else None
#         s["summary_w"].writerow([rep, f"{rate:.0f}", f"{gain:.1f}", lp["n_avg"], len(p),
#                                  f"{floor:.2f}", len(spikes),
#                                  f"{top[2]:.2f}" if top else "", f"{top[0]:.1f}" if top else "",
#                                  f"{lp['dur']:.1f}"])
#         s["spikes_fh"].flush()
#         s["summary_fh"].flush()
#         # surface-chart data: this pass reduced to npts frequency bins
#         npts = max(10, min(s["npts"], len(p)))
#         vec = self.survey_surface_vector(f, p, npts, s["peak"], s["f0"], s["f1"])
#         s["surface"].setdefault(rate, {}).setdefault(gain, []).append(vec)
#         # full-resolution levels for the .npz (one frequency axis per rate), while they fit in memory
#         known = s["freq_by_rate"].get(rate)
#         if known is None:
#             s["freq_by_rate"][rate] = known = np.asarray(f, dtype=np.float64)
#         if len(known) == len(p) and s["full_bytes"] + 4.0 * len(p) <= SURVEY_FULL_NPZ_MAX_BYTES:
#             s["spectra"].append((rate, gain, rep, np.asarray(p, dtype=np.float32)))
#             s["full_bytes"] += 4.0 * len(p)
#         elif not s["full_warned"]:
#             s["full_warned"] = True
#             self.log("Survey: full-resolution spectra no longer fit in memory - the .npz keeps only what is "
#                      "already stored; spikes, summary and surface CSVs are unaffected")
#         n_done, n_all = s["idx"] + 1, len(s["combos"])
#         elapsed = time.time() - s["t_start"]
#         eta = elapsed / n_done * (n_all - n_done)
#         self.log(f"Survey {n_done}/{n_all}: {rate/1e6:.3f} MS/s, {gain:.1f} dB -> "
#                  f"{len(spikes)} spikes (ETA {self.survey_fmt_seconds(eta)})")
#     def survey_write_surfaces(self, s):
#         """One CSV per sample rate laid out for Excel's 3-D Surface chart: frequency down the rows (category
#         axis, left blank in the corner cell so Excel treats it as labels), one column per gain with the
#         LOWEST gain first (= front of the chart), cells = level in dB (repeats averaged in power)."""
#         written = []
#         f0, f1, npts_cfg = s["f0"], s["f1"], s["npts"]
#         for rate, by_gain in sorted(s["surface"].items()):
#             gains = sorted(by_gain)
#             cols = []
#             for g in gains:
#                 stack = np.vstack(by_gain[g])
#                 cols.append(10.0 * np.log10(np.mean(10.0 ** (stack / 10.0), axis=0)))
#             n = len(cols[0])
#             centres = f0 + (np.arange(n) + 0.5) * (f1 - f0) / n
#             step_mhz = (f1 - f0) / n / 1e6
#             dec = max(3, int(math.ceil(-math.log10(step_mhz))) + 2) if step_mhz > 0 else 6
#             path = f"{s['base']}_surface_{rate / 1e6:.3f}MSps.csv"
#             with open(path, "w", newline="") as fh:
#                 w = csv.writer(fh)
#                 w.writerow([""] + [f"{g:.1f} dB" for g in gains])
#                 for k in range(n):
#                     w.writerow([f"{centres[k] / 1e6:.{dec}f}"] +
#                                [("" if not np.isfinite(c[k]) else f"{c[k]:.2f}") for c in cols])
#             written.append(path)
#         return written
#     def survey_finish(self, aborted):
#         """GUI thread, after the survey worker has finished: close files, write the surface CSVs and
#         the .npz, give the controls back and (if it was running before) resume CONTINUOUS."""
#         s = self._survey
#         if s is None:
#             return
#         for k in ("spikes_fh", "summary_fh"):
#             try:
#                 s[k].close()
#             except Exception:
#                 pass
#         surfaces = []
#         try:
#             surfaces = self.survey_write_surfaces(s)
#         except Exception as e:
#             self.log(f"Survey: could not write the surface-chart CSVs: {e}")
#         specs = s["spectra"]
#         if specs:
#             try:
#                 lengths = np.array([len(x[3]) for x in specs])
#                 extra = {f"freq_hz_{int(r)}": fr for r, fr in s["freq_by_rate"].items()}
#                 np.savez_compressed(
#                     s["base"] + "_spectra.npz",
#                     rate_hz=np.array([x[0] for x in specs]),
#                     gain_db=np.array([x[1] for x in specs]),
#                     repeat=np.array([x[2] for x in specs]),
#                     length=lengths,
#                     level_db=np.concatenate([x[3] for x in specs]),
#                     **extra)
#             except Exception as e:
#                 self.log(f"Survey: could not write spectra file: {e}")
#         # put the operator's settings back on the controls
#         self.sync_sample_rate_combo()
#         self.sync_gain_slider()
#         self.update_status_panel()
#         self.survey_set_lock(False)
#         n_done = sum(len(v) for by_g in s["surface"].values() for v in by_g.values())
#         self._survey = None
#         self.log(f"=== SURVEY {'STOPPED EARLY' if aborted else 'FINISHED'}: {n_done} passes saved"
#                  f"{', ' + str(s['skipped']) + ' skipped' if s['skipped'] else ''} -> "
#                  f"{s['base']}_spikes.csv / _summary.csv / _spectra.npz ===")
#         if surfaces:
#             self.log(f"Surface charts for Excel ({len(surfaces)} file(s), one per sample rate): open one, select all, "
#                      f"Insert > Charts > Surface > 3-D Surface. Frequency runs along the bottom, level is vertical, "
#                      f"lowest gain is at the front: {', '.join(os.path.basename(p) for p in surfaces)}")
#         if s["was_running"] and not aborted:
#             self.continuous_when_idle()

    # ------------------------------------------------------------------------------------
    # Survey REPORT: one self-contained HTML file (data, charts and analysis all inside)
    # ------------------------------------------------------------------------------------

    @staticmethod
    def survey_mode_clusters(vals, pids, rates_idx, valid, tol, min_pids, min_rates, max_clusters, accept=None):
        """
        Find tight groups of spikes whose value (frequency, or offset from a block centre) agrees to
        within tol. Mode seeking: the busiest bins first, so a dense spectrum cannot chain everything
        into one cluster. A group counts only if it spans at least min_pids different passes and
        min_rates different sample rates (and, if given, accept(members) says yes). Returns a list of
        index arrays into vals.
        """
        idxs = np.flatnonzero(valid)
        if idxs.size == 0:
            return []
        order = idxs[np.argsort(vals[idxs], kind="stable")]
        sv = vals[order]
        keys = np.round(sv / tol).astype(np.int64)
        uk, cnt = np.unique(keys, return_counts=True)
        taken = np.zeros(order.size, dtype=bool)
        out = []
        for j in np.argsort(-cnt, kind="stable"):
            if cnt[j] < min_pids:
                break
            k = uk[j]
            lo = np.searchsorted(sv, (k - 1.5) * tol, "left")
            hi = np.searchsorted(sv, (k + 1.5) * tol, "right")
            sel = np.arange(lo, hi)
            sel = sel[~taken[sel]]
            if sel.size < min_pids:
                continue
            centre = float(np.median(sv[sel]))
            lo2 = np.searchsorted(sv, centre - tol, "left")
            hi2 = np.searchsorted(sv, centre + tol, "right")
            sel = np.arange(lo2, hi2)
            sel = sel[~taken[sel]]
            if sel.size < min_pids:
                continue
            members = order[sel]
            if len(set(pids[members].tolist())) < min_pids or len(set(rates_idx[members].tolist())) < min_rates:
                continue
            if accept is not None and not accept(members):
                continue
            out.append(members)
            taken[sel] = True
            if len(out) >= max_clusters:
                break
        return out

    # ---- previous survey_analyse_all() (no verdicts), kept in case of problems ----
#     def survey_analyse_all(self, s):
#         """
#         Turn the recorded passes into everything the report shows. Returns a dict.
#         SAMPLE-RATE EFFECTS: spikes are grouped (a) by their offset from the nearest block centre -
#         a feature made by the receiver itself moves with the tuning/sample rate, so it keeps the same
#         offset (in Hz, or as a fraction of the sample rate) from each block centre while its absolute
#         frequency changes - and then (b) by absolute frequency: seen at (almost) every rate = stable,
#         seen at only some rates = rate-dependent.
#         GAIN EFFECTS: for every feature the level is followed against RF gain (read from the stored
#         spectra at the feature's frequency, so below-threshold points count too) and the slope in dB
#         per dB is found: ~1 arrives with the signal, ~0 generated after the gain stage, >1.6 rises
#         faster than the gain (distortion / overload product); plus onset gain and high-gain compression.
#         """
#         passes = s["passes"]
#         rates = sorted({q["rate"] for q in passes})
#         gains = sorted({q["gain"] for q in passes})
#         nR, nG = len(rates), len(gains)
#         ri = {r: i for i, r in enumerate(rates)}
#         gi = {g: i for i, g in enumerate(gains)}
#         f0, f1, npts = s["f0"], s["f1"], s["npts"]
#         n_pass = len(passes)
#         bin_hz = max(rates) / NFFT
#         fs_db = 10.0 * math.log10(0.25) + float(getattr(self, "_psd_to_dbuv_const", 33.5))
#         # ---- per rate/gain tables ----
#         grp = {}
#         for q in passes:
#             grp.setdefault((ri[q["rate"]], gi[q["gain"]]), []).append(q)
#         spec = [[None] * nG for _ in range(nR)]
#         floor = [[None] * nG for _ in range(nR)]
#         peak = [[None] * nG for _ in range(nR)]
#         for (a, b), qs in grp.items():
#             st = np.vstack([q["vec"] for q in qs])
#             with np.errstate(all="ignore"):
#                 spec[a][b] = 10.0 * np.log10(np.mean(10.0 ** (st / 10.0), axis=0))
#             fl = [q["floor"] for q in qs if np.isfinite(q["floor"])]
#             pk = [q["peak"] for q in qs if np.isfinite(q["peak"])]
#             floor[a][b] = float(np.mean(fl)) if fl else None
#             peak[a][b] = float(np.max(pk)) if pk else None
#         # ---- spikes ----
#         rows = [(ri[q["rate"]], gi[q["gain"]], f, lv, pr, q["pid"]) for q in passes for (f, lv, pr, _bs) in q["spikes"]]
#         A = np.array(rows, dtype=np.float64) if rows else np.zeros((0, 6))
#         ia, ib = A[:, 0].astype(int), A[:, 1].astype(int)
#         pid_arr = A[:, 5].astype(int)
#         cent = {a: self.build_centers(f0, f1, rates[a]) for a in range(nR)}
#         feats = []
#         def make_feature(kind, members, value):
#             """value: absolute Hz (kind 'abs'), offset Hz ('fixed') or fraction of Fs ('scaled')."""
#             det = [[None] * nG for _ in range(nR)]
#             for i in members:
#                 a, b, lv = int(A[i, 0]), int(A[i, 1]), float(A[i, 3])
#                 if det[a][b] is None or lv > det[a][b]:
#                     det[a][b] = lv
#             lev = [row[:] for row in det]
#             refined = {}
#             for q in passes:                              # level at the feature's frequency from the stored spectra
#                 if q.get("p") is None:
#                     continue
#                 a, b = ri[q["rate"]], gi[q["gain"]]
#                 fa = s["freq_by_rate"][q["rate"]]
#                 tol = 1.5 * q["rate"] / NFFT
#                 if kind == "abs":
#                     targets = [value]
#                 elif kind == "fixed":
#                     targets = cent[a] + value
#                 else:
#                     targets = cent[a] + value * q["rate"]
#                 best = -np.inf
#                 for t in np.atleast_1d(targets):
#                     lo, hi = np.searchsorted(fa, t - tol, "left"), np.searchsorted(fa, t + tol, "right")
#                     if hi > lo:
#                         best = max(best, float(np.max(q["p"][lo:hi])))
#                 if np.isfinite(best):
#                     refined[(a, b)] = max(refined.get((a, b), -np.inf), best)
#             for (a, b), v in refined.items():
#                 lev[a][b] = v
#             # gain behaviour from the points that stand clear of the noise floor
#             vis = [[(lev[a][b] is not None and floor[a][b] is not None and
#                      lev[a][b] - floor[a][b] >= SURVEY_SPIKE_THRESH_DB) for b in range(nG)] for a in range(nR)]
#             slopes, lows, highs, onsets = [], [], [], []
#             for a in range(nR):
#                 pts = [(gains[b], lev[a][b]) for b in range(nG) if vis[a][b]]
#                 if len(pts) >= 3 and pts[-1][0] - pts[0][0] >= 6.0:
#                     x = np.array([p[0] for p in pts]); y = np.array([p[1] for p in pts])
#                     slopes.append(float(np.polyfit(x, y, 1)[0]))
#                     if len(pts) >= 6:
#                         h = len(pts) // 2
#                         lows.append(float(np.polyfit(x[:h + 1], y[:h + 1], 1)[0]))
#                         highs.append(float(np.polyfit(x[h:], y[h:], 1)[0]))
#                 if pts:
#                     onsets.append(pts[0][0])
#             slope = float(np.median(slopes)) if slopes else None
#             tags, gtext = [], ""
#             if slope is None:
#                 gtext = "too few points above the noise floor to judge"
#             elif slope >= 1.6:
#                 tags.append(("rises faster than gain", "t-sup"))
#                 gtext = f"{slope:.1f} dB per dB - typical of a distortion / overload product made inside the gain stages"
#             elif slope >= 1.4:
#                 tags.append(("steeper than gain", "t-lin"))
#                 gtext = f"{slope:.1f} dB per dB - slightly steeper than the gain; possibly an early distortion product"
#             elif slope >= 0.6:
#                 tags.append(("tracks gain", "t-lin"))
#                 gtext = f"{slope:.1f} dB per dB - arrives with the signal (antenna side) or follows the receiver noise"
#             elif slope >= 0.25:
#                 tags.append(("weakly gain-dependent", "t-fix"))
#                 gtext = f"{slope:.1f} dB per dB - mostly independent of gain"
#             else:
#                 tags.append(("independent of gain", "t-fix"))
#                 gtext = f"{slope:.1f} dB per dB - level does not follow the gain: made after the gain stage (ADC / clock / digital)"
#             if lows and highs and np.median(lows) > 0.6 and np.median(highs) < 0.4 * np.median(lows):
#                 k = gains[len(gains) // 2]
#                 tags.append(("flattens at high gain", "t-cmp"))
#                 gtext += f"; stops rising above about {k:.0f} dB (compression / clipping?)"
#             onset = float(np.median(onsets)) if onsets else None
#             if onset is not None and onset > gains[0] + 0.25 * (gains[-1] - gains[0]):
#                 tags.append((f"appears above {onset:.1f} dB", "t-ons"))
#                 gtext += f"; first stands out of the noise at {onset:.1f} dB gain"
#             rates_present = [any(det[a][b] is not None for b in range(nG)) for a in range(nR)]
#             n_rates = sum(rates_present)
#             n_pids = len({int(pid_arr[i]) for i in members})
#             lv_all = [float(A[i, 3]) for i in members]
#             prom_all = [float(A[i, 4]) for i in members]
#             return {"kind": kind, "value": value, "rates_present": rates_present, "n_rates": n_rates,
#                     "n_pids": n_pids, "median_level": float(np.median(lv_all)), "max_prom": max(prom_all),
#                     "slope": slope, "tags": tags, "gtext": gtext, "levels": lev, "onset": onset,
#                     "super": slope is not None and slope >= 1.6, "fixedgain": slope is not None and slope < 0.25,
#                     "compress": any(t[0].startswith("flattens") for t in tags),
#                     "onset_tag": any(t[0].startswith("appears") for t in tags)}
#         used = np.zeros(len(A), dtype=bool)
#         if nR >= 2 and len(A):
#             # offset of every spike from its nearest block centre
#             d = np.zeros(len(A))
#             kb = np.zeros(len(A), dtype=int)               # index of the nearest block
#             for a in range(nR):
#                 m = ia == a
#                 c = cent[a]
#                 ff = A[m, 2]
#                 if len(c) == 1:
#                     near = np.full(ff.shape, c[0])
#                 else:
#                     k = np.searchsorted(c, ff)
#                     k0, k1 = np.clip(k - 1, 0, len(c) - 1), np.clip(k, 0, len(c) - 1)
#                     near = np.where(np.abs(ff - c[k0]) <= np.abs(ff - c[k1]), c[k0], c[k1])
#                     kb[m] = np.where(np.abs(ff - c[k0]) <= np.abs(ff - c[k1]), k0, k1)
#                 d[m] = ff - near
#             frac = d / np.array([rates[a] for a in ia])
#             min_p = max(3, int(0.3 * n_pass))
#             def accept_block(members):
#                 # A tuning / sample-rate artefact repeats in (nearly) EVERY block of a pass. A real signal that
#                 # merely happens to sit at the same offset from a centre at two rates shows up in one block only.
#                 cov = []
#                 for a in range(nR):
#                     nb = len(cent[a])
#                     if nb < 3:
#                         continue
#                     per = {}
#                     for i in members[ia[members] == a]:
#                         per.setdefault(int(pid_arr[i]), set()).add(int(kb[i]))
#                     if per:
#                         cov.append(float(np.mean([len(v) / nb for v in per.values()])))
#                 return (not cov) or float(np.median(cov)) >= 0.5
#             for members in self.survey_mode_clusters(d, pid_arr, ia, ~used, 2.5 * bin_hz, min_p, 2, 12, accept_block):
#                 used[members] = True
#                 feats.append(make_feature("fixed", members, float(np.median(d[members]))))
#             for members in self.survey_mode_clusters(frac, pid_arr, ia, ~used, 2.5 / NFFT, min_p, 2, 12, accept_block):
#                 used[members] = True
#                 feats.append(make_feature("scaled", members, float(np.median(frac[members]))))
#         if len(A):
#             for members in self.survey_mode_clusters(A[:, 2], pid_arr, ia, ~used, 3.0 * bin_hz, 2, 1, 150):
#                 used[members] = True
#                 feats.append(make_feature("abs", members, float(np.median(A[members, 2]))))
#         n_single = int((~used).sum())
#         # ---- present the features ----
#         def simple_fraction(fr):
#             for n in (2, 3, 4, 5, 6, 8, 10, 12, 16):
#                 if abs(abs(fr) - 1.0 / n) < 3.0 / NFFT:
#                     return f" (about Fs/{n})"
#             return ""
#         out_feats = []
#         for ft in feats:
#             kind, v = ft["kind"], ft["value"]
#             if kind == "abs":
#                 where = self.survey_fmt_hz(v)
#                 share = ft["n_rates"] / nR
#                 if nR < 2:
#                     tag, cls, txt = "single rate", "t-stable", ""
#                 elif share >= 0.75:
#                     tag, cls, txt = "stable", "t-stable", f"{ft['n_rates']}/{nR} rates"
#                 elif share > 0.5:
#                     tag, cls, txt = "partly rate-dependent", "t-rate", f"{ft['n_rates']}/{nR} rates"
#                 else:
#                     tag, cls, txt = "rate-dependent", "t-rate", f"{ft['n_rates']}/{nR} rates"
#             elif kind == "fixed":
#                 where = ("at the centre of every block" if abs(v) < 1.5 * bin_hz else
#                          f"{v / 1e3:+.2f} kHz from every block centre")
#                 tag, cls, txt = "moves with tuning", "t-block", f"{ft['n_rates']}/{nR} rates"
#             else:
#                 where = f"{v:+.4f} x sample rate from every block centre{simple_fraction(v)}"
#                 tag, cls, txt = "scales with sample rate", "t-block", f"{ft['n_rates']}/{nR} rates"
#             ft.update({"where": where, "rate_tag": tag, "rate_cls": cls, "rate_text": txt})
#             lv = [x for row in ft["levels"] for x in row if x is not None]
#             ft["level_text"] = (f"{ft['median_level']:.1f} dB (up to {ft['max_prom']:.0f} dB above its surroundings)")
#             out_feats.append(ft)
#         order = sorted(out_feats, key=lambda x: (0 if x["kind"] != "abs" else 1, -x["n_pids"], -x["max_prom"]))
#         # ---- findings ----
#         fnd = []
#         def add(level, text):
#             fnd.append({"level": level, "text": text})
#         add("", f"{n_pass} pass(es): {nR} sample rate(s) x {nG} gain step(s), band {self.survey_fmt_hz(f0)} - {self.survey_fmt_hz(f1)}. "
#                 f"{len(A)} spike(s) found; {len(order)} recurring feature(s) listed below"
#                 + (f", {n_single} isolated spike(s) seen only once (not listed - normally noise)." if n_single else "."))
#         if nR < 2:
#             add("warn", "Only one sample rate was surveyed, so sample-rate effects cannot be separated from real signals. Survey at least two rates.")
#         blk = [x for x in order if x["kind"] != "abs"]
#         absf = [x for x in order if x["kind"] == "abs"]
#         if nR >= 2:
#             if blk:
#                 for x in blk[:5]:
#                     add("bad", f"SAMPLE-RATE / TUNING ARTEFACT - {x['where']}: seen at {x['n_rates']} of {nR} rates, in {x['n_pids']} of {n_pass} passes, "
#                                f"typically {x['median_level']:.1f} dB. Its absolute frequency changes with the block centres, so it is made by the "
#                                f"receiver, not received from outside. Gain: {x['gtext']}.")
#             else:
#                 add("ok", "No feature keeps a fixed offset from the block centres across sample rates (no tuning / sample-rate artefact found).")
#             rd = [x for x in absf if x["rate_tag"] in ("rate-dependent", "partly rate-dependent")]
#             st = [x for x in absf if x["rate_tag"] == "stable"]
#             if rd:
#                 top = sorted(rd, key=lambda x: -x["max_prom"])[:5]
#                 add("warn", f"{len(rd)} feature(s) appear at only some sample rates - candidates for sample-rate effects (aliasing / images / spurs). Strongest: " +
#                     "; ".join(f"{x['where']} at {x['n_rates']}/{nR} rates ({x['max_prom']:.0f} dB above surroundings)" for x in top) + ".")
#             if st:
#                 top = sorted(st, key=lambda x: -x["median_level"])[:5]
#                 add("ok", f"{len(st)} feature(s) are stable across sample rates (real signals or rate-independent spurs). Strongest: " +
#                     "; ".join(f"{x['where']} ({x['median_level']:.1f} dB)" for x in top) + ".")
#         sup = sorted([x for x in order if x["super"]], key=lambda x: -x["slope"])
#         if sup:
#             add("bad", "GAIN-STAGE DISTORTION SUSPECTED - these rise faster than the gain: " +
#                 "; ".join(f"{x['where']} ({x['slope']:.1f} dB per dB)" for x in sup[:6]) +
#                 ". Signals from the antenna rise 1 dB per dB; 2 dB per dB and 3 dB per dB are the signature of 2nd and 3rd order products. Try lower gain or an attenuator.")
#         ons = [x for x in order if x["onset_tag"]]
#         if ons:
#             add("warn", "Appear only above a certain gain: " + "; ".join(f"{x['where']} (from about {x['onset']:.1f} dB)" for x in ons[:6]) + ".")
#         cmp_ = [x for x in order if x["compress"]]
#         if cmp_:
#             add("warn", "Stop rising at high gain (possible compression / clipping): " + "; ".join(x["where"] for x in cmp_[:6]) + ".")
#         fx = [x for x in order if x["fixedgain"]]
#         if fx:
#             add("warn", f"{len(fx)} feature(s) do not follow the gain at all (generated after the gain stage - ADC, clocks, digital): " +
#                 "; ".join(f"{x['where']}" for x in fx[:6]) + ("; ..." if len(fx) > 6 else "") + ".")
#         hmin, where_min = np.inf, None
#         for a in range(nR):
#             for b in range(nG):
#                 if peak[a][b] is not None and fs_db - peak[a][b] < hmin:
#                     hmin, where_min = fs_db - peak[a][b], (a, b)
#         if where_min is not None:
#             a, b = where_min
#             if hmin < 6.0:
#                 add("bad", f"CLIPPING RISK: the strongest component comes within {hmin:.1f} dB of the ADC full-scale equivalent at {gains[b]:.1f} dB gain / "
#                            f"{rates[a] / 1e6:.3f} MS/s. Above that gain the front end is likely overloaded.")
#             else:
#                 add("ok", f"The strongest single component never comes closer than {hmin:.1f} dB to the ADC full-scale equivalent. "
#                           f"(Many strong signals together can still overload the ADC without any one tone being near full scale.)")
#         fs_slopes = []
#         for a in range(nR):
#             xs = [(gains[b], floor[a][b]) for b in range(nG) if floor[a][b] is not None]
#             if len(xs) >= 3 and xs[-1][0] - xs[0][0] >= 6.0:
#                 fs_slopes.append(float(np.polyfit([p[0] for p in xs], [p[1] for p in xs], 1)[0]))
#         if fs_slopes:
#             add("", f"The noise floor rises {np.median(fs_slopes):.2f} dB per dB of gain (about 1 while receiver noise dominates; lower means ADC / digital noise dominates).")
#         return {"rates": rates, "gains": gains, "spec": spec, "floor": floor, "peak": peak, "fs_db": fs_db,
#                 "features": order, "findings": fnd, "A": A, "n_pass": n_pass}

    def survey_analyse_all(self, s):
        """
        Turn the recorded passes into everything the report shows. Returns a dict.

        GOAL: tell signals that really come from the device under test apart from artefacts.
        Every recurring spike (a "feature") is followed through every sample rate and every gain and
        given a VERDICT:
          reliable  - at the same frequency at (nearly) every sample rate AND its level tracks the RF
                      gain (about 1 dB per dB): the receiver is sensing something real.
          rate      - caused by the sample rate / tuning: it keeps a fixed offset from the block
                      centres (LO / DC leakage, I-Q image, Fs-related spurs), or it appears only at some
                      sample rates (aliases, images); where possible it is matched to the reliable
                      signal it is an image / alias / harmonic of.
          gain      - caused by the gain stages: it rises faster than the gain (2nd / 3rd order
                      distortion products, matched to harmonics / intermodulation of reliable signals).
          internal  - stable across rates but its level does not follow the gain: generated inside the
                      receiver after the gain stage (ADC, clocks, digital).
          unsure    - not enough evidence either way.
        Then the gain behaviour is summarised per gain (DUT signal level, gain-caused artefacts, noise
        floor, compression, headroom) to find the maximum usable gain, and every rate/gain pass is
        scored to suggest the "best spectrum" setting.
        """
        passes = s["passes"]
        rates = sorted({q["rate"] for q in passes})
        gains = sorted({q["gain"] for q in passes})
        nR, nG = len(rates), len(gains)
        ri = {r: i for i, r in enumerate(rates)}
        gi = {g: i for i, g in enumerate(gains)}
        f0, f1, npts = s["f0"], s["f1"], s["npts"]
        n_pass = len(passes)
        bin_hz = max(rates) / NFFT
        fs_db = 10.0 * math.log10(0.25) + float(getattr(self, "_psd_to_dbuv_const", 33.5))
        fmt = self.survey_fmt_hz

        # ---- per rate/gain tables ----
        grp = {}
        for q in passes:
            grp.setdefault((ri[q["rate"]], gi[q["gain"]]), []).append(q)
        spec = [[None] * nG for _ in range(nR)]
        floor = [[None] * nG for _ in range(nR)]
        peak = [[None] * nG for _ in range(nR)]
        for (a, b), qs in grp.items():
            st = np.vstack([q["vec"] for q in qs])
            with np.errstate(all="ignore"):
                spec[a][b] = 10.0 * np.log10(np.mean(10.0 ** (st / 10.0), axis=0))
            fl = [q["floor"] for q in qs if np.isfinite(q["floor"])]
            pk = [q["peak"] for q in qs if np.isfinite(q["peak"])]
            floor[a][b] = float(np.mean(fl)) if fl else None
            peak[a][b] = float(np.max(pk)) if pk else None

        # ---- spikes ----
        rows = [(ri[q["rate"]], gi[q["gain"]], f, lv, pr, q["pid"]) for q in passes for (f, lv, pr, _bs) in q["spikes"]]
        A = np.array(rows, dtype=np.float64) if rows else np.zeros((0, 6))
        ia, ib = A[:, 0].astype(int), A[:, 1].astype(int)
        pid_arr = A[:, 5].astype(int)
        cent = {a: self.build_centers(f0, f1, rates[a]) for a in range(nR)}
        feats = []

        def make_feature(kind, members, value):
            """value: absolute Hz (kind 'abs'), offset Hz ('fixed') or fraction of Fs ('scaled')."""
            det = [[None] * nG for _ in range(nR)]
            for i in members:
                a, b, lv = int(A[i, 0]), int(A[i, 1]), float(A[i, 3])
                if det[a][b] is None or lv > det[a][b]:
                    det[a][b] = lv
            lev = [row[:] for row in det]
            refined = {}
            for q in passes:                              # level at the feature's frequency from the stored spectra
                if q.get("p") is None:
                    continue
                a, b = ri[q["rate"]], gi[q["gain"]]
                fa = s["freq_by_rate"][q["rate"]]
                tol = 1.5 * q["rate"] / NFFT
                if kind == "abs":
                    targets = [value]
                elif kind == "fixed":
                    targets = cent[a] + value
                else:
                    targets = cent[a] + value * q["rate"]
                best = -np.inf
                for t in np.atleast_1d(targets):
                    lo, hi = np.searchsorted(fa, t - tol, "left"), np.searchsorted(fa, t + tol, "right")
                    if hi > lo:
                        best = max(best, float(np.max(q["p"][lo:hi])))
                if np.isfinite(best):
                    refined[(a, b)] = max(refined.get((a, b), -np.inf), best)
            for (a, b), v in refined.items():
                lev[a][b] = v
            vis = [[(lev[a][b] is not None and floor[a][b] is not None and
                     lev[a][b] - floor[a][b] >= SURVEY_SPIKE_THRESH_DB) for b in range(nG)] for a in range(nR)]
            rates_vis = [any(vis[a]) for a in range(nR)]
            slopes, onsets, knees, lows = [], [], [], []
            for a in range(nR):
                pts = [(b, gains[b], lev[a][b]) for b in range(nG) if vis[a][b]]
                if len(pts) >= 3 and pts[-1][1] - pts[0][1] >= 6.0:
                    x = np.array([p[1] for p in pts]); y = np.array([p[2] for p in pts])
                    slopes.append(float(np.polyfit(x, y, 1)[0]))
                    lows.append(float(np.polyfit(x[:3], y[:3], 1)[0]))
                    if len(pts) >= 4:                    # compression: falls below the 1 dB/dB line fitted to the lowest points
                        c = float(np.mean(y[:3] - x[:3]))
                        for j in range(3, len(pts)):
                            short = (x[j] + c) - y[j]
                            nxt = ((x[j + 1] + c) - y[j + 1]) if j + 1 < len(pts) else short
                            if short > 2.0 and nxt > 1.0:
                                knees.append(float(x[j]))
                                break
                if pts:
                    onsets.append(pts[0][1])
            slope = float(np.median(slopes)) if slopes else None
            knee = float(np.median(knees)) if (knees and slope is not None and 0.4 <= slope < 1.5 and
                                               lows and 0.7 <= float(np.median(lows)) <= 1.4) else None
            onset = float(np.median(onsets)) if onsets else None
            n_pids = len({int(pid_arr[i]) for i in members})
            lv_all = [float(A[i, 3]) for i in members]
            prom_all = [float(A[i, 4]) for i in members]
            return {"kind": kind, "value": value, "vis": vis, "rates_vis": rates_vis, "n_vis_rates": sum(rates_vis),
                    "n_pids": n_pids, "median_level": float(np.median(lv_all)), "max_prom": max(prom_all),
                    "slope": slope, "knee": knee, "onset": onset, "levels": lev,
                    "first_vis": onset}

        used = np.zeros(len(A), dtype=bool)
        if nR >= 2 and len(A):
            d = np.zeros(len(A))
            kb = np.zeros(len(A), dtype=int)               # index of the nearest block
            for a in range(nR):
                m = ia == a
                c = cent[a]
                ff = A[m, 2]
                if len(c) == 1:
                    near = np.full(ff.shape, c[0])
                else:
                    k = np.searchsorted(c, ff)
                    k0, k1 = np.clip(k - 1, 0, len(c) - 1), np.clip(k, 0, len(c) - 1)
                    near = np.where(np.abs(ff - c[k0]) <= np.abs(ff - c[k1]), c[k0], c[k1])
                    kb[m] = np.where(np.abs(ff - c[k0]) <= np.abs(ff - c[k1]), k0, k1)
                d[m] = ff - near
            frac = d / np.array([rates[a] for a in ia])
            min_p = max(3, int(0.3 * n_pass))

            def accept_block(members):
                # A tuning / sample-rate artefact repeats in (nearly) EVERY block of a pass. A real signal that
                # merely happens to sit at the same offset from a centre at two rates shows up in one block only.
                # Every sample rate that contributes spikes must show that, and at least two rates must.
                cov = []
                for a in range(nR):
                    nb = len(cent[a])
                    if nb < 3:
                        continue
                    per = {}
                    for i in members[ia[members] == a]:
                        per.setdefault(int(pid_arr[i]), set()).add(int(kb[i]))
                    if per:
                        cov.append(float(np.mean([len(v) / nb for v in per.values()])))
                return (not cov) or (min(cov) >= 0.5 and (len(cov) < 2 or sum(1 for c_ in cov if c_ >= 0.5) >= 2))
            for members in self.survey_mode_clusters(d, pid_arr, ia, ~used, 2.5 * bin_hz, min_p, 2, 12, accept_block):
                used[members] = True
                feats.append(make_feature("fixed", members, float(np.median(d[members]))))
            for members in self.survey_mode_clusters(frac, pid_arr, ia, ~used, 2.5 / NFFT, min_p, 2, 12, accept_block):
                used[members] = True
                feats.append(make_feature("scaled", members, float(np.median(frac[members]))))
        if len(A):
            for members in self.survey_mode_clusters(A[:, 2], pid_arr, ia, ~used, 3.0 * bin_hz, 2, 1, 150):
                used[members] = True
                feats.append(make_feature("abs", members, float(np.median(A[members, 2]))))
        n_single = int((~used).sum())

        # ---- VERDICTS ----
        for ft in feats:
            sl = ft["slope"]
            share = ft["n_vis_rates"] / nR
            if ft["kind"] != "abs":
                ft["verdict"] = "rate"
                ft["rate_state"] = "moves"
            else:
                state = "untested" if nR < 2 else ("stable" if share >= 0.75 else ("partial" if share > 0.5 else "dependent"))
                ft["rate_state"] = state
                if sl is not None and sl >= 1.6:
                    ft["verdict"] = "gain"
                elif state == "dependent":
                    ft["verdict"] = "rate"
                elif sl is not None and sl < 0.25 and state in ("stable", "partial"):
                    ft["verdict"] = "internal"
                elif state == "stable" and sl is not None and 0.6 <= sl < 1.4:
                    ft["verdict"] = "reliable"
                else:
                    ft["verdict"] = "unsure"

        def hz_where(ft):
            kind, v = ft["kind"], ft["value"]
            if kind == "abs":
                return fmt(v)
            if kind == "fixed":
                return ("at the centre of every block" if abs(v) < 1.5 * bin_hz else f"{v / 1e3:+.2f} kHz from every block centre")
            for n in (2, 3, 4, 5, 6, 8, 10, 12, 16):
                if abs(abs(v) - 1.0 / n) < 3.0 / NFFT:
                    return f"{v:+.4f} x sample rate from every block centre (about Fs/{n})"
            return f"{v:+.4f} x sample rate from every block centre"

        reliable = sorted([x for x in feats if x["verdict"] == "reliable"], key=lambda x: -x["median_level"])
        top_rel = reliable[:12]

        def explain(ft):
            """Relate an artefact to the reliable signals: harmonic, intermodulation, image, alias."""
            if ft["kind"] == "fixed":
                return ("leakage of the receiver's own oscillator / DC at the block centre" if abs(ft["value"]) < 1.5 * bin_hz
                        else "fixed offset from every block centre - a receiver spur that moves with the tuning")
            if ft["kind"] == "scaled":
                return "spur at a fixed fraction of the sample rate from every block centre - aliasing / clock related"
            f = ft["value"]
            tol = 3.0 * bin_hz
            hits = []
            for r in top_rel:
                fr = r["value"]
                for n in (2, 3, 4):
                    if abs(f - n * fr) <= tol:
                        hits.append(f"harmonic (x{n}) of {fmt(fr)}")
                for a in range(nR):
                    if not ft["rates_vis"][a]:
                        continue
                    for c in cent[a]:
                        if abs(c - fr) < rates[a] / 2.0 and abs(f - (2.0 * c - fr)) <= tol:
                            hits.append(f"I-Q mirror image of {fmt(fr)} (about a block centre at {rates[a] / 1e6:.3f} MS/s)")
                            break
                    for m_ in (1, 2, 3):
                        if abs(abs(f - fr) - m_ * rates[a]) <= tol:
                            hits.append(f"alias of {fmt(fr)} ({m_} x {rates[a] / 1e6:.3f} MS/s away)")
                            break
            for i1 in range(len(top_rel)):
                for i2 in range(i1 + 1, len(top_rel)):
                    f1_, f2_ = top_rel[i1]["value"], top_rel[i2]["value"]
                    for lab, fx in (("sum", f1_ + f2_), ("difference", abs(f1_ - f2_)),
                                    ("3rd-order intermodulation", 2 * f1_ - f2_), ("3rd-order intermodulation", 2 * f2_ - f1_)):
                        if abs(f - fx) <= tol:
                            hits.append(f"{lab} product of {fmt(f1_)} and {fmt(f2_)}")
            seen_h = []
            for h in hits:
                if h not in seen_h:
                    seen_h.append(h)
            return "; ".join(seen_h[:2])

        for ft in feats:
            ft["where"] = hz_where(ft)
            share_txt = f"{ft['n_vis_rates']}/{nR} rates"
            ft["rate_text"] = share_txt
            ex = explain(ft) if ft["verdict"] in ("rate", "gain") else ""
            ft["short"] = ex.split(";")[0] if ex else ""
            sl = ft["slope"]
            slt = "n/a" if sl is None else f"{sl:.2f} dB/dB"
            v = ft["verdict"]
            if v == "reliable":
                ft["cause"] = f"seen at {share_txt} at the same frequency; level tracks gain ({slt})"
            elif v == "rate":
                if ft["kind"] != "abs":
                    ft["cause"] = ex
                else:
                    ft["cause"] = (ex + "; " if ex else "") + f"appears at only {share_txt} - depends on the sample rate"
            elif v == "gain":
                ft["cause"] = ((ex + "; ") if ex else "") + f"rises {slt} (signals rise 1.0) - non-linear product made in the gain stages"
            elif v == "internal":
                ft["cause"] = f"stable across rates but level does not follow the gain ({slt}) - generated inside the receiver after the gain stage"
            else:
                if nR < 2:
                    ft["cause"] = "only one sample rate surveyed - cannot tell real signals from rate artefacts"
                elif ft["rate_state"] == "partial":
                    ft["cause"] = f"seen at {share_txt} - not clearly stable"
                elif sl is None:
                    ft["cause"] = "too few points above the noise floor to judge the gain behaviour"
                else:
                    ft["cause"] = f"gain slope {slt} is between the linear and distortion ranges"
            # gain range over which a reliable signal stays linear
            ft["lin_to"] = None
            if v == "reliable":
                top = max((gains[b] for a in range(nR) for b in range(nG) if ft["vis"][a][b]), default=gains[-1])
                if ft["knee"] is not None:
                    below = [g for g in gains if g < ft["knee"]]
                    ft["lin_to"] = below[-1] if below else gains[0]
                    ft["cause"] += f"; compresses from about {ft['knee']:.1f} dB gain"
                else:
                    ft["lin_to"] = top

        byv = {k: [x for x in feats if x["verdict"] == k] for k in ("reliable", "rate", "gain", "internal", "unsure")}

        # ---- per rate x gain behaviour of the groups ----
        def grp_lv(group, a, b):
            return [x["levels"][a][b] for x in group if x["vis"][a][b]]
        dut_level = [[None] * nG for _ in range(nR)]
        dut_snr = [[None] * nG for _ in range(nR)]
        gart_level = [[None] * nG for _ in range(nR)]
        n_gart = [[0] * nG for _ in range(nR)]
        n_art = [[0] * nG for _ in range(nR)]
        art_level = [[None] * nG for _ in range(nR)]
        for a in range(nR):
            for b in range(nG):
                rl_ = grp_lv(byv["reliable"], a, b)
                if rl_:
                    dut_level[a][b] = max(rl_)
                    if floor[a][b] is not None:
                        dut_snr[a][b] = float(np.median([v - floor[a][b] for v in rl_]))
                gl_ = grp_lv(byv["gain"], a, b)
                n_gart[a][b] = len(gl_)
                gart_level[a][b] = max(gl_) if gl_ else None
                al_ = grp_lv(byv["gain"] + byv["rate"] + byv["internal"], a, b)
                n_art[a][b] = len(al_)
                art_level[a][b] = max(al_) if al_ else None

        # ---- maximum usable gain (per rate, then overall) ----
        per_rate = []
        for a in range(nR):
            cands = []                                   # (gain index limit, reason)
            first_d = next((b for b in range(nG) if n_gart[a][b] > 0 and (b == nG - 1 or n_gart[a][b + 1] > 0)), None)
            if first_d is not None:
                cands.append((max(first_d - 1, 0), f"distortion products appear at {gains[first_d]:.1f} dB"
                              + (" (already at the lowest surveyed gain)" if first_d == 0 else "")))
            knees = [x["knee"] for x in byv["reliable"] if x["knee"] is not None]
            if knees and len(knees) >= max(1, len(byv["reliable"]) // 2):
                kg = float(np.median(knees))
                kb_ = max([b for b in range(nG) if gains[b] < kg], default=0)
                cands.append((kb_, f"DUT signals compress above {kg:.1f} dB"))
            fl = [(b, floor[a][b]) for b in range(nG) if floor[a][b] is not None]
            for j in range(len(fl) - 2):
                (b0, v0), (b1, v1), (b2, v2) = fl[j], fl[j + 1], fl[j + 2]
                s1 = (v1 - v0) / max(gains[b1] - gains[b0], 1e-6)
                s2 = (v2 - v1) / max(gains[b2] - gains[b1], 1e-6)
                if s1 > 1.5 and s2 > 1.5:
                    cands.append((b0, f"noise floor rising faster than the gain from {gains[b0]:.1f} dB"))
                    break
            hb = next((b for b in range(nG) if peak[a][b] is not None and fs_db - peak[a][b] < 6.0), None)
            if hb is not None:
                cands.append((max(hb - 1, 0), f"strongest component within 6 dB of ADC full scale at {gains[hb]:.1f} dB"))
            if cands:
                bl, why = min(cands, key=lambda c: c[0])
                gmax_b, reason = bl, why
            else:
                gmax_b, reason = nG - 1, f"no limit found up to the highest surveyed gain ({gains[-1]:.1f} dB)"
            gmin_b = next((b for b in range(nG) if dut_snr[a][b] is not None and dut_snr[a][b] >= 10.0), None)
            per_rate.append({"rate": rates[a], "gmax_b": gmax_b, "reason": reason, "gmin_b": gmin_b,
                             "first_dist_b": first_d})
        gmax_all_b = int(np.floor(np.median([p["gmax_b"] for p in per_rate]))) if per_rate else 0
        gmin_list = [p["gmin_b"] for p in per_rate if p["gmin_b"] is not None]
        gmin_all_b = int(np.ceil(np.median(gmin_list))) if gmin_list else None
        dist_on = [gains[p["first_dist_b"]] for p in per_rate if p["first_dist_b"] is not None]

        # ---- score every rate/gain for the "best spectrum" ----
        n_rel = len(byv["reliable"])
        scores = [[None] * nG for _ in range(nR)]
        best_rows = []
        for a in range(nR):
            for b in range(nG):
                if floor[a][b] is None:
                    continue
                vis_rel = [x for x in byv["reliable"] if x["vis"][a][b]]
                cov = (len(vis_rel) / n_rel) if n_rel else 0.0
                snr = float(np.median([min(x["levels"][a][b] - floor[a][b], 30.0) for x in vis_rel])) if vis_rel else 0.0
                n_a = n_art[a][b]
                strongest_dut = dut_level[a][b]
                margin = (strongest_dut - art_level[a][b]) if (strongest_dut is not None and art_level[a][b] is not None) else None
                head = fs_db - peak[a][b] if peak[a][b] is not None else None
                # coverage of the reliable signals (0-100) + their SNR (0-30) + margin over the strongest artefact
                # (0-30; full marks when no artefact is visible) - 2 per artefact visible
                sc = 100.0 * cov + min(snr, 30.0) + (min(margin, 30.0) if margin is not None else (30.0 if vis_rel else 0.0)) - 2.0 * n_a
                notes = []
                if head is not None and head < 6.0:
                    sc -= 100.0
                    notes.append("near ADC full scale")
                if b > per_rate[a]["gmax_b"]:
                    sc -= 50.0
                    notes.append("above the maximum usable gain")
                if not n_rel:
                    sc = -2.0 * n_a
                scores[a][b] = sc
                best_rows.append({"a": a, "b": b, "score": sc, "coverage": cov, "snr": snr if vis_rel else None, "n_art": n_a,
                                  "margin": margin, "headroom": head, "notes": notes})
        best_rows.sort(key=lambda r: -r["score"])

        return {"rates": rates, "gains": gains, "spec": spec, "floor": floor, "peak": peak, "fs_db": fs_db,
                "feats": feats, "byv": byv, "n_single": n_single, "A": A, "n_pass": n_pass, "cent": cent,
                "dut_level": dut_level, "dut_snr": dut_snr, "gart_level": gart_level, "n_gart": n_gart,
                "n_art": n_art, "per_rate": per_rate, "gmax_all_b": gmax_all_b, "gmin_all_b": gmin_all_b,
                "dist_on": dist_on, "scores": scores, "best": best_rows[:3], "n_rel": n_rel, "nR": nR, "nG": nG,
                "bin_hz": bin_hz}

    @staticmethod
    def survey_fmt_hz(hz):
        hz = abs(float(hz))
        return f"{hz / 1e6:.4f} MHz" if hz >= 1e6 else f"{hz / 1e3:.2f} kHz"

    # ---- previous survey_build_report() (no verdicts), kept in case of problems ----
#     def survey_build_report(self, s, partial=False):
#         """Build the complete single-file HTML report for the passes recorded so far."""
#         r = self.survey_analyse_all(s)
#         rates, gains = r["rates"], r["gains"]
#         nR, nG = len(rates), len(gains)
#         f0, f1, npts = s["f0"], s["f1"], s["npts"]
#         def v1(x):
#             return None if x is None or not np.isfinite(x) else round(float(x), 1)
#         def arr1(v):
#             return None if v is None else [v1(x) for x in v]
#         freq = [round((f0 + (k + 0.5) * (f1 - f0) / npts) / 1e6, 6) for k in range(npts)]
#         feats = []
#         for ft in r["features"]:
#             feats.append({
#                 "where": ft["where"], "rate_tag": ft["rate_tag"], "rate_cls": ft["rate_cls"], "rate_text": ft["rate_text"],
#                 "seen": f"{ft['n_pids']}/{r['n_pass']} passes", "level_text": ft["level_text"],
#                 "slope": None if ft["slope"] is None else round(ft["slope"], 2),
#                 "gain_tags": [list(t) for t in ft["tags"]], "gain_text": ft["gtext"],
#                 "levels": [[v1(x) for x in row] for row in ft["levels"]]})
#         A = r["A"]
#         spikes = []
#         if len(A):
#             top = np.argsort(-A[:, 4])[:SURVEY_REPORT_MAX_SPIKES]
#             spikes = [[int(A[i, 0]), int(A[i, 1]), int(round(A[i, 2])), v1(A[i, 3]), v1(A[i, 4])] for i in top]
#         ri = {x: i for i, x in enumerate(rates)}
#         fs_db = r["fs_db"]
#         table = [[int(q["rate"]), q["gain"], q["rep"], v1(q["floor"]), v1(q["peak"]),
#                   v1(fs_db - q["peak"]) if np.isfinite(q["peak"]) else None, len(q["spikes"]), round(q["dur"], 1)]
#                  for q in s["passes"]]
#         D = {"meta": {"band": f"{self.survey_fmt_hz(f0)} - {self.survey_fmt_hz(f1)}", "passes": r["n_pass"],
#                       "n_avg": s["passes"][0]["n_avg"] if s["passes"] else 1,
#                       "created": time.strftime("%Y-%m-%d %H:%M"), "partial": bool(partial)},
#              "freq": freq, "rates": [int(x) for x in rates], "gains": [float(x) for x in gains],
#              "spec": [[arr1(x) for x in row] for row in r["spec"]],
#              "floor": [[v1(x) for x in row] for row in r["floor"]], "peak": [[v1(x) for x in row] for row in r["peak"]],
#              "fs_db": round(fs_db, 1), "findings": r["findings"], "features": feats, "spikes": spikes, "table": table}
#         payload = json.dumps(D, separators=(",", ":"), allow_nan=False).replace("</", "<\\/")
#         return SURVEY_REPORT_TEMPLATE.replace("__DATA__", payload)

    # ---- previous survey_build_report() (verdict analysis page), kept in case of problems ----
#     def survey_build_report(self, s, partial=False):
#         """Build the complete single-file HTML report for the passes recorded so far."""
#         r = self.survey_analyse_all(s)
#         rates, gains = r["rates"], r["gains"]
#         nR, nG, n_rel = r["nR"], r["nG"], r["n_rel"]
#         f0, f1, npts = s["f0"], s["f1"], s["npts"]
#         byv, fs_db = r["byv"], r["fs_db"]
#         fmt = self.survey_fmt_hz
#         def v1(x):
#             return None if x is None or not np.isfinite(x) else round(float(x), 1)
#         def arr1(v):
#             return None if v is None else [v1(x) for x in v]
#         def med(vals):
#             vals = [x for x in vals if x is not None]
#             return float(np.median(vals)) if vals else None
#         freq = [round((f0 + (k + 0.5) * (f1 - f0) / npts) / 1e6, 6) for k in range(npts)]
#         # ---- features for the report ----
#         order = sorted(r["feats"], key=lambda x: ({"reliable": 0, "rate": 1, "gain": 2, "internal": 3, "unsure": 4}[x["verdict"]],
#                                                     -x["n_pids"], -x["max_prom"]))
#         feats = []
#         for ft in order:
#             pos = None
#             if ft["kind"] == "abs":
#                 pos = round(ft["value"] / 1e6, 6)
#             elif ft["kind"] == "fixed":
#                 pos = round(ft["value"], 1)
#             else:
#                 pos = round(ft["value"], 6)
#             feats.append({"where": ft["where"], "verdict": ft["verdict"], "kind": ft["kind"], "pos": pos, "cause": ft["cause"],
#                           "rate_text": ft["rate_text"], "seen": f"{ft['n_pids']}/{r['n_pass']} passes",
#                           "level_text": f"{ft['median_level']:.1f} dB (up to {ft['max_prom']:.0f} dB above its surroundings)",
#                           "level": round(ft["median_level"], 1), "slope": None if ft["slope"] is None else round(ft["slope"], 2),
#                           "lin_to": ft["lin_to"], "first_vis": ft["first_vis"], "knee": ft["knee"],
#                           "levels": [[v1(x) for x in row] for row in ft["levels"]]})
#         A = r["A"]
#         spikes = []
#         if len(A):
#             top = np.argsort(-A[:, 4])[:SURVEY_REPORT_MAX_SPIKES]
#             spikes = [[int(A[i, 0]), int(A[i, 1]), int(round(A[i, 2])), v1(A[i, 3]), v1(A[i, 4])] for i in top]
#         table = [[int(q["rate"]), q["gain"], q["rep"], v1(q["floor"]), v1(q["peak"]),
#                   v1(fs_db - q["peak"]) if np.isfinite(q["peak"]) else None, len(q["spikes"]), round(q["dur"], 1)]
#                  for q in s["passes"]]
#         # ---- gain analysis for the charts ----
#         def col(m, b, fn):
#             return fn([m[a][b] for a in range(nR)])
#         gain_info = {
#             "gains": [float(g) for g in gains],
#             "floor": [v1(col(r["floor"], b, med)) for b in range(nG)],
#             "dut_level": [v1(col(r["dut_level"], b, med)) for b in range(nG)],
#             "dut_snr": [v1(col(r["dut_snr"], b, med)) for b in range(nG)],
#             "gart_level": [v1(col(r["gart_level"], b, lambda v: max([x for x in v if x is not None], default=None))) for b in range(nG)],
#             "n_gart": [round(float(np.mean([r["n_gart"][a][b] for a in range(nR)])), 2) for b in range(nG)],
#             "n_art": [round(float(np.mean([r["n_art"][a][b] for a in range(nR)])), 2) for b in range(nG)],
#             "gmax": gains[r["gmax_all_b"]], "gmin": (gains[r["gmin_all_b"]] if r["gmin_all_b"] is not None else None),
#             "hypothesis": SURVEY_EXPECTED_GAIN_LIMIT_DB,
#             "per_rate": [{"rate": int(p["rate"]), "gmax": gains[p["gmax_b"]], "reason": p["reason"],
#                           "gmin": (gains[p["gmin_b"]] if p["gmin_b"] is not None else None)} for p in r["per_rate"]]}
#         best = [{"rate_i": x["a"], "gain_i": x["b"], "rate": int(rates[x["a"]]), "gain": gains[x["b"]], "score": round(x["score"], 1),
#                  "coverage": round(x["coverage"], 2), "snr": v1(x["snr"]), "n_art": x["n_art"], "margin": v1(x["margin"]),
#                  "headroom": v1(x["headroom"]), "notes": x["notes"]} for x in r["best"]]
#         scores = [[v1(x) for x in row] for row in r["scores"]]
#         # ---- summary text ----
#         summary = []
#         def add(level, text):
#             summary.append({"level": level, "text": text})
#         nrate, ngain, nint, nuns = len(byv["rate"]), len(byv["gain"]), len(byv["internal"]), len(byv["unsure"])
#         add("", f"{r['n_pass']} pass(es): {nR} sample rate(s) x {nG} gain step(s), band {fmt(f0)} - {fmt(f1)}. "
#                 f"{len(A)} spike(s) found, grouped into {len(feats)} recurring feature(s): {n_rel} reliably sensed, {nrate} caused by the sample rate / tuning, "
#                 f"{ngain} caused by the gain stages, {nint} generated inside the receiver, {nuns} unconfirmed"
#                 + (f"; {r['n_single']} isolated spike(s) seen only once were ignored (normally noise)." if r["n_single"] else "."))
#         navg = s["passes"][0]["n_avg"] if s["passes"] else 1
#         if navg < 5:
#             add("warn", f"Averaging was only {navg} capture(s) per block. With so little averaging, random noise peaks can look like weak features and distort the verdicts - use 10 or more (the Time Averaging setting) for the survey.")
#         if nR < 2:
#             add("warn", "Only one sample rate was surveyed, so a real signal cannot be told from a sample-rate artefact. Survey at least two (preferably three or more) rates; nothing can be confirmed as reliably sensed.")
#         if n_rel:
#             top = byv["reliable"][:6]
#             add("ok", f"RELIABLY SENSED - {n_rel} signal(s) appear at the same frequency at (nearly) every sample rate and rise in step with the RF gain, "
#                       f"so they are the best candidates for coming from the device under test. Strongest: " +
#                 "; ".join(f"{x['where']} ({x['median_level']:.1f} dB)" for x in top) + ("; ..." if n_rel > 6 else "") + ".")
#         elif nR >= 2:
#             add("warn", "No feature was both stable across the sample rates and tracking the gain, so nothing could be confirmed as reliably sensed. "
#                         "Check the 'unconfirmed' list, or whether the device under test is producing a signal in this band.")
#         if nrate:
#             add("bad", f"IGNORE - {nrate} feature(s) are caused by the sample rate / tuning (LO leakage, images, aliases, fixed-offset spurs): " +
#                 "; ".join(f"{x['where']}" + (f" ({x['short']})" if x.get("short") else "") for x in byv["rate"][:5]) + ("; ..." if nrate > 5 else "") + " (details below).")
#         if ngain:
#             add("bad", f"IGNORE - {ngain} feature(s) are caused by the gain stages (non-linear products that rise faster than the gain): " +
#                 "; ".join(f"{x['where']} ({x['slope']:.1f} dB/dB" + (f", {x['short']}" if x.get("short") else "") + ")" for x in byv["gain"][:5]) + ("; ..." if ngain > 5 else "") + " (details below).")
#         if nint:
#             add("warn", f"IGNORE - {nint} feature(s) are stable but do not follow the gain, so they are generated inside the receiver after the gain stage: " +
#                 "; ".join(f"{x['where']}" for x in byv["internal"][:5]) + ("; ..." if nint > 5 else "") + ".")
#         # maximum usable gain
#         gmax, gmin = gain_info["gmax"], gain_info["gmin"]
#         reasons = [p["reason"] for p in gain_info["per_rate"]]
#         rep = min(r["per_rate"], key=lambda p: abs(p["gmax_b"] - r["gmax_all_b"]))         # a rate that sets the overall limit
#         lim_rate = rep
#         lower = [p for p in r["per_rate"] if p["gmax_b"] < r["gmax_all_b"]]
#         add("ok" if gmax >= gains[-1] else "warn",
#             f"MAXIMUM USABLE GAIN - about {gmax:.1f} dB" + (" (the highest surveyed gain; no limit was found)" if gmax >= gains[-1] else
#             f". Limited by: {rep['reason']}") + "."
#             + (" Lower limits at other sample rates: " + "; ".join(f"{p['rate'] / 1e6:.3f} MS/s {gains[p['gmax_b']]:.1f} dB ({p['reason']})" for p in lower) +
#                " - a lower limit at one rate only is often a coincidence with a rate-dependent artefact, see the table below." if lower else "")
#             + (f" Below about {gmin:.1f} dB the DUT signals are less than 10 dB above the noise floor, so the useful window is {gmin:.1f} - {gmax:.1f} dB." if gmin is not None and n_rel else ""))
#         dist_on = r["dist_on"]
#         exp = SURVEY_EXPECTED_GAIN_LIMIT_DB
#         gsl = [x["slope"] for x in byv["gain"] if x["slope"] is not None]
#         if dist_on:
#             on = float(np.median(dist_on))
#             sl = float(np.median(gsl)) if gsl else None
#             sltxt = f" and rise about {sl:.1f} dB per dB of gain against 1 dB per dB for real signals" if sl is not None else ""
#             if abs(on - exp) <= 3.0:
#                 add("bad", f"YOUR ESTIMATE (gains above about {exp:g} dB add disproportionate distortion) IS CONFIRMED: gain-caused artefacts first appear at about {on:.1f} dB{sltxt}.")
#             elif on < exp - 3.0:
#                 add("bad", f"YOUR ESTIMATE (about {exp:g} dB) IS PARTLY CONFIRMED but optimistic: gain-caused artefacts already appear from about {on:.1f} dB{sltxt}.")
#             else:
#                 add("warn", f"YOUR ESTIMATE (about {exp:g} dB) IS NOT CONFIRMED as stated: gain-caused artefacts only appear above about {on:.1f} dB{sltxt}.")
#         else:
#             lim_txt = ""
#             if gmax < gains[-1]:
#                 lim_txt = f" Another limit does appear: {lim_rate['reason']} (at {lim_rate['rate'] / 1e6:.3f} MS/s)."
#             add("warn", f"YOUR ESTIMATE (gains above about {exp:g} dB add disproportionate distortion) IS NOT CONFIRMED by this survey: no gain-caused artefacts "
#                         f"were identified between {gains[0]:.1f} and {gains[-1]:.1f} dB.{lim_txt}")
#         if best:
#             b0 = best[0]
#             parts = [f"{b0['rate'] / 1e6:.3f} MS/s at {b0['gain']:.1f} dB"]
#             det = []
#             if n_rel:
#                 det.append(f"shows {int(round(b0['coverage'] * n_rel))} of {n_rel} reliable signal(s)")
#                 if b0["snr"] is not None:
#                     det.append(f"median {b0['snr']:.0f} dB above the noise")
#             det.append(f"{b0['n_art']} known artefact(s) visible")
#             if b0["margin"] is not None:
#                 det.append(f"the strongest artefact is {b0['margin']:.0f} dB below the strongest DUT signal")
#             if b0["headroom"] is not None:
#                 det.append(f"{b0['headroom']:.0f} dB below ADC full scale")
#             add("ok", "BEST SPECTRUM - " + parts[0] + ": " + "; ".join(det) + ".")
#         # other observations
#         hmin, where_min = np.inf, None
#         for a in range(nR):
#             for b in range(nG):
#                 if r["peak"][a][b] is not None and fs_db - r["peak"][a][b] < hmin:
#                     hmin, where_min = fs_db - r["peak"][a][b], (a, b)
#         if where_min is not None:
#             a, b = where_min
#             if hmin < 6.0:
#                 add("bad", f"CLIPPING RISK: the strongest component comes within {hmin:.1f} dB of the ADC full-scale equivalent at {gains[b]:.1f} dB gain / {rates[a] / 1e6:.3f} MS/s.")
#             else:
#                 add("", f"The strongest single component never comes closer than {hmin:.1f} dB to the ADC full-scale equivalent (many strong signals together can still overload the ADC).")
#         fs_slopes = []
#         for a in range(nR):
#             xs = [(gains[b], r["floor"][a][b]) for b in range(nG) if r["floor"][a][b] is not None]
#             if len(xs) >= 3 and xs[-1][0] - xs[0][0] >= 6.0:
#                 fs_slopes.append(float(np.polyfit([p[0] for p in xs], [p[1] for p in xs], 1)[0]))
#         if fs_slopes:
#             add("", f"The noise floor rises {np.median(fs_slopes):.2f} dB per dB of gain (about 1 while receiver noise dominates; lower means ADC / digital noise dominates; clearly higher means the gain stages are adding noise).")
#         D = {"meta": {"band": f"{fmt(f0)} - {fmt(f1)}", "passes": r["n_pass"], "n_reliable": n_rel,
#                       "n_avg": s["passes"][0]["n_avg"] if s["passes"] else 1,
#                       "created": time.strftime("%Y-%m-%d %H:%M"), "partial": bool(partial)},
#              "freq": freq, "rates": [int(x) for x in rates], "gains": [float(x) for x in gains],
#              "centres": [[round(float(c) / 1e6, 6) for c in r["cent"][a]] for a in range(nR)],
#              "spec": [[arr1(x) for x in row] for row in r["spec"]],
#              "floor": [[v1(x) for x in row] for row in r["floor"]], "peak": [[v1(x) for x in row] for row in r["peak"]],
#              "fs_db": round(fs_db, 1), "summary": summary, "features": feats, "gain": gain_info, "best": best, "scores": scores,
#              "spikes": spikes, "table": table}
#         payload = json.dumps(D, separators=(",", ":"), allow_nan=False).replace("</", "<\\/")
#         return SURVEY_REPORT_TEMPLATE.replace("__DATA__", payload)

    def survey_build_report(self, s, partial=False):
        """Build the single-file HTML report: the two maps first, then the repeatability results."""
        passes = s["passes"]
        rates = sorted({q["rate"] for q in passes})
        gains = sorted({q["gain"] for q in passes})
        nR, nG = len(rates), len(gains)
        ri = {r: i for i, r in enumerate(rates)}
        gi = {g: i for i, g in enumerate(gains)}
        f0, f1, npts = s["f0"], s["f1"], s["npts"]
        fmt = self.survey_fmt_hz
        fs_db = 10.0 * math.log10(0.25) + float(getattr(self, "_psd_to_dbuv_const", 33.5))

        def v1(x):
            return None if x is None or not np.isfinite(x) else round(float(x), 1)

        def arr1(v):
            return None if v is None else [v1(x) for x in v]

        # ---- what the maps show: the average of the accepted (agreeing) scans of every setting ----
        grp = {}
        for q in passes:
            grp.setdefault((ri[q["rate"]], gi[q["gain"]]), []).append(q)
        spec = [[None] * nG for _ in range(nR)]
        floor = [[None] * nG for _ in range(nR)]
        peak = [[None] * nG for _ in range(nR)]
        used = {}
        for (a, b), qs in grp.items():
            acc = [q for q in qs if q["accepted"]] or qs[-2:]          # in-progress setting: its latest scans
            used[(a, b)] = {id(q) for q in acc}
            st = np.vstack([q["vec"] for q in acc])
            with np.errstate(all="ignore"):
                spec[a][b] = 10.0 * np.log10(np.mean(10.0 ** (st / 10.0), axis=0))
            fl = [q["floor"] for q in acc if np.isfinite(q["floor"])]
            pk = [q["peak"] for q in acc if np.isfinite(q["peak"])]
            floor[a][b] = float(np.mean(fl)) if fl else None
            peak[a][b] = float(np.max(pk)) if pk else None
        freq = [round((f0 + (k + 0.5) * (f1 - f0) / npts) / 1e6, 6) for k in range(npts)]

        # ---- repeatability ----
        flags = [[0] * nG for _ in range(nR)]                 # 0 repeated at once, 1 needed extra scans, 2 never repeated
        rows, allrows = [], []
        n_combo = len(s["combo_info"])
        by_rate = {}
        for info in s["combo_info"]:
            a, b = ri[info["rate"]], gi[info["gain"]]
            extra = info["scans"] > 2
            bad = not info["converged"]
            flags[a][b] = 2 if bad else (1 if extra else 0)
            c1 = info["comparisons"][0] if info["comparisons"] else {"shift": None, "local": None, "spikes": 0}
            cl = info["comparisons"][-1] if info["comparisons"] else c1
            outcome = (f"never repeated ({info['scans']} scans)" if bad else
                       f"repeated after {info['scans']} scans" if extra else "repeated at once")
            allrows.append([int(info["rate"]), info["gain"], info["scans"], outcome,
                            None if c1["shift"] is None else round(c1["shift"], 2),
                            None if c1["local"] is None else round(c1["local"] * 100.0, 1), c1["spikes"]])
            by_rate.setdefault(a, [0, 0, 0])
            by_rate[a][0] += 1
            by_rate[a][1] += 1 if (extra or bad) else 0
            by_rate[a][2] += 1 if bad else 0
            if extra or bad:
                rows.append({"rate": int(info["rate"]), "gain": info["gain"], "scans": info["scans"], "converged": bool(info["converged"]),
                             "outcome": outcome,
                             "first": {"shift": None if c1["shift"] is None else round(c1["shift"], 2),
                                       "local": None if c1["local"] is None else round(c1["local"], 4), "spikes": c1["spikes"]},
                             "last": {"shift": None if cl["shift"] is None else round(cl["shift"], 2),
                                      "local": 0.0 if cl["local"] is None else round(cl["local"], 4), "spikes": cl["spikes"]}})
        rows.sort(key=lambda r: (r["rate"], r["gain"]))
        summary = []
        n_extra = sum(1 for r in rows if r["converged"])
        n_bad = sum(1 for r in rows if not r["converged"])
        if n_combo == 0:
            summary.append({"level": "", "text": "No setting has completed its repeat scans yet."})
        elif not rows:
            summary.append({"level": "ok", "text": f"All {n_combo} rate / gain settings repeated at the first two scans - the SDR gave consistent results everywhere."})
        else:
            summary.append({"level": "warn" if not n_bad else "bad",
                            "text": f"{len(rows)} of {n_combo} rate / gain settings needed more than two scans to repeat: {n_extra} repeated after extra scans"
                                    f"{', ' + str(n_bad) + ' never repeated (the maps show their last two scans, marked !)' if n_bad else ''}. "
                                    "They are listed below and marked + / ! in the maps - treat them with caution."})
            worst = sorted(by_rate.items(), key=lambda kv: -kv[1][1])
            summary.append({"level": "", "text": "By sample rate (settings needing extra scans / settings scanned): " +
                            "; ".join(f"{rates[a] / 1e6:.3f} MS/s: {v[1]}/{v[0]}" for a, v in sorted(by_rate.items())) + "."})
            if worst and worst[0][1][1] and len(by_rate) > 1 and worst[0][1][1] >= 2 * max(1, sorted(v[1] for v in by_rate.values())[len(by_rate) // 2]):
                summary.append({"level": "warn", "text": f"{rates[worst[0][0]] / 1e6:.3f} MS/s stands out: most of the settings that needed extra scans are at this rate - "
                                                         f"the SDR (or the PC / USB link at that speed) is not delivering repeatable data there."})
        navg = passes[0]["n_avg"] if passes else 1
        if navg < 5:
            summary.append({"level": "warn", "text": f"Averaging was only {navg} capture(s) per block, so random noise differences make extra scans more likely - use 10 or more for the survey."})

        A = []
        for q in passes:
            if q["accepted"] or (used.get((ri[q["rate"]], gi[q["gain"]])) and id(q) in used[(ri[q["rate"]], gi[q["gain"]])]):
                for (fr, lv, pr, _bs) in q["spikes"]:
                    A.append((ri[q["rate"]], gi[q["gain"]], int(round(fr)), v1(lv), v1(pr), pr))
        A.sort(key=lambda x: -x[5])
        spikes = [list(x[:5]) for x in A[:SURVEY_REPORT_MAX_SPIKES]]
        table = [[int(q["rate"]), q["gain"], q["scan"], 1 if q["accepted"] else 0, v1(q["floor"]), v1(q["peak"]),
                  v1(fs_db - q["peak"]) if np.isfinite(q["peak"]) else None, len(q["spikes"]), round(q["dur"], 1)] for q in passes]
        D = {"meta": {"band": f"{fmt(f0)} - {fmt(f1)}", "scans": len(passes), "combos": n_combo if n_combo else len(grp),
                      "n_avg": navg, "created": time.strftime("%Y-%m-%d %H:%M"), "partial": bool(partial)},
             "freq": freq, "rates": [int(x) for x in rates], "gains": [float(x) for x in gains],
             "spec": [[arr1(x) for x in row] for row in spec],
             "floor": [[v1(x) for x in row] for row in floor], "peak": [[v1(x) for x in row] for row in peak],
             "fs_db": round(fs_db, 1), "flags": flags, "rep": {"summary": summary, "rows": rows, "all": allrows},
             "spikes": spikes, "table": table}
        payload = json.dumps(D, separators=(",", ":"), allow_nan=False).replace("</", "<\\/")
        return SURVEY_REPORT_TEMPLATE.replace("__DATA__", payload)

    def survey_write_report(self, s, partial=False):
        """Write (overwrite) the single report file. Called every SURVEY_REPORT_EVERY_S seconds while
        the survey runs, so something useful exists even if the run is interrupted, and at the end."""
        if not s["passes"]:
            return None
        html = self.survey_build_report(s, partial)
        tmp = s["report_path"] + ".tmp"
        with open(tmp, "w", encoding="utf-8") as fh:
            fh.write(html)
        os.replace(tmp, s["report_path"])
        return s["report_path"]

    # ---- previous survey_record(), kept in case of problems ----
#     def survey_record(self, lp):
#         """Analyse and store one finished pass (runs on the survey worker thread)."""
#         s = self._survey
#         rate, gain, rep = s["combos"][s["idx"]]
#         f, p, floor, spikes = self.survey_analyse(lp["f"], lp["p"], SURVEY_SPIKE_THRESH_DB)
#         vec = self.survey_surface_vector(f, p, s["npts"], s["peak"], s["f0"], s["f1"])
#         rec = {"pid": len(s["passes"]), "rate": rate, "gain": gain, "rep": rep, "floor": floor,
#                "peak": float(np.max(p)) if len(p) else float("nan"), "n_avg": lp["n_avg"], "dur": lp["dur"],
#                "spikes": spikes, "vec": vec, "p": None}
#         # full-resolution levels are kept (one frequency axis per rate) while they fit in memory, so the
#         # report can follow each feature's level through every pass, including passes where it was too
#         # weak to count as a spike
#         known = s["freq_by_rate"].setdefault(rate, np.asarray(f, dtype=np.float64))
#         if len(known) == len(p) and s["full_bytes"] + 4.0 * len(p) <= SURVEY_FULL_NPZ_MAX_BYTES:
#             rec["p"] = np.asarray(p, dtype=np.float32)
#             s["full_bytes"] += 4.0 * len(p)
#         elif not s["full_warned"]:
#             s["full_warned"] = True
#             self.log("Survey: full-resolution spectra no longer fit in memory - the report will follow feature "
#                      "levels through the detected spikes only for the remaining passes")
#         s["passes"].append(rec)
#         n_done, n_all = s["idx"] + 1, len(s["combos"])
#         elapsed = time.time() - s["t_start"]
#         eta = elapsed / n_done * (n_all - n_done)
#         self.log(f"Survey {n_done}/{n_all}: {rate/1e6:.3f} MS/s, {gain:.1f} dB -> "
#                  f"{len(spikes)} spikes (ETA {self.survey_fmt_seconds(eta)})")
#         if time.time() - s["last_report"] > SURVEY_REPORT_EVERY_S:
#             s["last_report"] = time.time()
#             try:
#                 self.survey_write_report(s, partial=True)
#             except Exception as e:
#                 self.log(f"Survey: could not update the report file: {e}")

    def survey_compare_scans(self, rec_a, lp_a, rec_b, lp_b):
        """
        Are two consecutive scans of the same rate/gain the same? Three tests, with limits that account for
        how much the averaging smooths the noise:
          level shift   - median dB difference over all bins (the SDR still settling, gain not applied yet)
          bins differ   - share of (8-bin power-averaged) bins that differ by more than the noise allows
          spikes differ - strong spikes (>= SURVEY_REPEAT_SPIKE_PROM_DB) present in one scan with no spike at
                          that frequency in the other (features appearing / disappearing)
        Returns a dict with the three measurements and "same".
        """
        out = {"same": False, "shift": None, "local": None, "spikes": 0}
        pa, pb = np.asarray(lp_a["p"], dtype=np.float64), np.asarray(lp_b["p"], dtype=np.float64)
        if pa.shape != pb.shape:
            return out
        ok = np.isfinite(pa) & np.isfinite(pb) & (pa > -190.0) & (pb > -190.0)
        if int(ok.sum()) < 64:
            return out
        a, b = pa[ok], pb[ok]
        shift = float(np.median(b - a))
        k = 8
        n = (a.size // k) * k
        la = (10.0 ** (a[:n] / 10.0)).reshape(-1, k).mean(axis=1)
        lb = (10.0 ** (b[:n] / 10.0)).reshape(-1, k).mean(axis=1)
        dl = 10.0 * np.log10(lb / la)
        n_avg = max(1, int(lp_a.get("n_avg", 1)))
        sigma = math.sqrt(2.0) * 5.57 / math.sqrt(n_avg * k)          # dB scatter of the difference from noise alone
        limit = max(SURVEY_REPEAT_LOCAL_DB, 4.0 * sigma)
        local = float(np.mean(np.abs(dl - np.median(dl)) > limit))
        tol = 3.0 * lp_a["rate"] / NFFT
        fa = np.array([sp[0] for sp in rec_a["spikes"] if sp[2] >= SURVEY_REPEAT_SPIKE_PROM_DB])
        fb = np.array([sp[0] for sp in rec_b["spikes"] if sp[2] >= SURVEY_REPEAT_SPIKE_PROM_DB])
        all_a = np.array([sp[0] for sp in rec_a["spikes"]])
        all_b = np.array([sp[0] for sp in rec_b["spikes"]])
        missing = 0
        for f_ in fa:
            if all_b.size == 0 or np.min(np.abs(all_b - f_)) > tol:
                missing += 1
        for f_ in fb:
            if all_a.size == 0 or np.min(np.abs(all_a - f_)) > tol:
                missing += 1
        out.update({"shift": shift, "local": local, "spikes": int(missing)})
        out["same"] = bool(abs(shift) <= max(SURVEY_REPEAT_SHIFT_DB, 0.0) and local <= SURVEY_REPEAT_LOCAL_FRAC and missing == 0)
        return out

    # ---- previous survey_record() (no rate read-back), kept in case of problems ----
#     def survey_record(self, lp, scan_no, rate, gain):
#         """Analyse and store one finished scan (runs on the survey worker thread); returns its record."""
#         s = self._survey
#         f, p, floor, spikes = self.survey_analyse(lp["f"], lp["p"], SURVEY_SPIKE_THRESH_DB)
#         vec = self.survey_surface_vector(f, p, s["npts"], s["peak"], s["f0"], s["f1"])
#         rec = {"pid": len(s["passes"]), "rate": rate, "gain": gain, "scan": scan_no, "accepted": False, "floor": floor,
#                "peak": float(np.max(p)) if len(p) else float("nan"), "n_avg": lp["n_avg"], "dur": lp["dur"],
#                "spikes": spikes, "vec": vec}
#         s["passes"].append(rec)
#         n_done, n_all = s["idx"], len(s["combos"])
#         elapsed = time.time() - s["t_start"]
#         eta = elapsed / max(n_done + 0.5, 0.5) * (n_all - n_done - 0.5) if n_done + 0.5 > 0 else 0.0
#         self.log(f"Survey {n_done + 1}/{n_all}: {rate/1e6:.3f} MS/s, {gain:.1f} dB, scan {scan_no} "
#                  f"({len(spikes)} spikes, ETA {self.survey_fmt_seconds(max(eta, 0.0))})")
#         if time.time() - s["last_report"] > SURVEY_REPORT_EVERY_S:
#             s["last_report"] = time.time()
#             try:
#                 self.survey_write_report(s, partial=True)
#             except Exception as e:
#                 self.log(f"Survey: could not update the report file: {e}")
#         return rec

    def survey_record(self, lp, scan_no, rate, gain):
        """Analyse and store one finished scan (runs on the survey worker thread); returns its record."""
        s = self._survey
        self.check_applied_rate(float(rate))               # logs once per rate: requested v hardware rate
        f, p, floor, spikes = self.survey_analyse(lp["f"], lp["p"], SURVEY_SPIKE_THRESH_DB)
        vec = self.survey_surface_vector(f, p, s["npts"], s["peak"], s["f0"], s["f1"])
        rec = {"pid": len(s["passes"]), "rate": rate, "gain": gain, "scan": scan_no, "accepted": False, "floor": floor,
               "peak": float(np.max(p)) if len(p) else float("nan"), "n_avg": lp["n_avg"], "dur": lp["dur"],
               "spikes": spikes, "vec": vec}
        s["passes"].append(rec)
        n_done, n_all = s["idx"], len(s["combos"])
        elapsed = time.time() - s["t_start"]
        eta = elapsed / max(n_done + 0.5, 0.5) * (n_all - n_done - 0.5) if n_done + 0.5 > 0 else 0.0
        self.log(f"Survey {n_done + 1}/{n_all}: {rate/1e6:.3f} MS/s, {gain:.1f} dB, scan {scan_no} "
                 f"({len(spikes)} spikes, ETA {self.survey_fmt_seconds(max(eta, 0.0))})")
        if time.time() - s["last_report"] > SURVEY_REPORT_EVERY_S:
            s["last_report"] = time.time()
            try:
                self.survey_write_report(s, partial=True)
            except Exception as e:
                self.log(f"Survey: could not update the report file: {e}")
        return rec

    def survey_finish(self, aborted):
        """GUI thread, after the survey worker has finished: write the report, give the controls back,
        open the report and (if it was running before) resume CONTINUOUS."""
        s = self._survey
        if s is None:
            return
        path = None
        try:
            path = self.survey_write_report(s, partial=bool(aborted))
        except Exception as e:
            import traceback
            traceback.print_exc()
            self.log(f"Survey: could not write the report: {e}")
        # put the operator's settings back on the controls
        self.sync_sample_rate_combo()
        self.sync_gain_slider()
        self.update_status_panel()
        self.survey_set_lock(False)
        n_done = len(s["passes"])
        n_set = len(s["combo_info"])
        self._survey = None
        self.log(f"=== SURVEY {'STOPPED EARLY' if aborted else 'FINISHED'}: {n_set} settings, {n_done} scans"
                 f"{', ' + str(s['skipped']) + ' skipped' if s['skipped'] else ''}"
                 f"{' -> report: ' + path if path else ' (nothing to report)'} ===")
        if path and SURVEY_OPEN_REPORT:
            try:
                QtGui.QDesktopServices.openUrl(QtCore.QUrl.fromLocalFile(os.path.abspath(path)))
            except Exception as e:
                self.log(f"Survey: could not open the report automatically: {e}")
        if s["was_running"] and not aborted:
            self.continuous_when_idle()

    # =====================================================================
    # BASEBAND SCOPE  (time-domain alternative to the spectrum display)
    #
    # Model: the scope shows the probe signal from DC up to Nyquist (Sample Rate / 2), as
    # a normal sampled scope would. There is NO centre-frequency or bandwidth setting - the
    # Sample Rate combo is the only thing that sets how fast a signal the scope can show.
    #
    # How it is done on an RTL-SDR Blog V4 (no direct sampling): the tuner is parked at
    # Fs/4, so the complex IQ stream covers RF -Fs/4 .. +3Fs/4. The wanted band 0 .. Fs/2
    # sits in the middle half of it. scope_process_capture() keeps only that band (this also
    # rejects the upconverter's mirror image and anything above Nyquist, i.e. it is the
    # anti-alias filter), then shifts it back to true RF by multiplying with e^(j*pi*n/2) -
    # the Fs/4 shift is exactly the sequence 1, j, -1, -j - and takes the real part. The
    # result is an ordinary real waveform sampled at Fs.
    #
    # LIMITS OF THE V4 (cannot be fixed in software):
    #   * The tuner LO phase is random, so the waveform's overall phase is unknown. A pure
    #     sine is unaffected (a phase change is just a time shift, and the trigger removes
    #     it). Waveforms made of several harmonics (square, pulse, triangle) can look
    #     distorted/slowly morphing; the Phase control lets you rotate it by hand.
    #   * The V4's HF input is specified from about 500 kHz up and is not DC coupled; its
    #     gain and phase are not flat. It is a good way to see WHAT is there, not a
    #     calibrated amplitude/shape measurement.
    #
    # Threading follows the sweep's rule: ONLY the worker thread (scope_loop) touches
    # self.sdr while the scope runs. The GUI thread just records settings in plain attributes
    # (scope_*) that scope_loop reads; frames come back via ScopeSignals.frame and are drawn
    # in scope_on_frame() on the GUI thread. The scope thread is stored in self.sweep_thread
    # so closeEvent(), STOP and sample_rate_on_changed() treat it as "a sweep is running".
    # =====================================================================

    # ---- previous scope_build_ui(), kept in case of problems ----
#     def scope_build_ui(self, left_layout):
#         # ---- state ----
#         self.scope_active = False
#         self.scope_hold = False
#         self.scope_channel = 0            # 0 signal, 1 envelope, 2 signal + envelope
#         self.scope_window_s = 0.002
#         self.scope_trig_mode = 0          # 0 free run, 1 rising, 2 falling
#         self.scope_trig_level = 0.0
#         self.scope_bw_hz = 0.0            # optional extra low-pass (0 = off, i.e. full DC..Nyquist)
#         self.scope_phase_rad = 0.0        # manual overall phase rotation
#         self.scope_remove_lo_spike = True
#         self.scope_auto_y = True
#         self._scope_pending = False       # a frame is queued for the GUI; don't flood it
#         self._scope_last = None
#         self._scope_mask_cache = None
#         self.scope_signals = ScopeSignals()
#         self.scope_signals.frame.connect(self.scope_on_frame)
#         # ---- scope plot (sits where the spectrum plot is; shown only in scope mode) ----
#         self.scope_plot = pg.PlotWidget()
#         self.scope_plot.setLabel('bottom', 'Time', units='s')
#         self.scope_plot.setLabel('left', 'Amplitude (1.0 = ADC full scale)')
#         self.scope_plot.showGrid(x=True, y=True, alpha=0.3)
#         self.scope_curve1 = self.scope_plot.plot(pen=pg.mkPen('y', width=1))
#         self.scope_curve2 = self.scope_plot.plot(pen=pg.mkPen('c', width=1))
#         for _c in (self.scope_curve1, self.scope_curve2):
#             try:
#                 _c.setClipToView(True)
#                 _c.setDownsampling(auto=True, method='peak')
#             except Exception:
#                 pass
#         self.scope_curve2.hide()
#         # draggable trigger-level line (orange) and trigger-point marker (grey, t = 0)
#         self.scope_trig_line = pg.InfiniteLine(
#             pos=0.0, angle=0, movable=True,
#             pen=pg.mkPen((255, 128, 0), width=1, style=QtCore.Qt.DashLine))
#         self.scope_plot.addItem(self.scope_trig_line)
#         self.scope_zero_line = pg.InfiniteLine(
#             pos=0.0, angle=90, movable=False,
#             pen=pg.mkPen((140, 140, 140), width=1, style=QtCore.Qt.DotLine))
#         self.scope_plot.addItem(self.scope_zero_line)
#         self.scope_trig_line.sigPositionChanged.connect(self.scope_on_trig_line)
#         self.scope_plot.setVisible(False)
#         left_layout.insertWidget(left_layout.indexOf(self.plot), self.scope_plot)
#         # ---- always-visible row: the mode button + a one-line readout ----
#         row_top = QtWidgets.QHBoxLayout()
#         left_layout.addLayout(row_top)
#         self.btn_scope = QtWidgets.QPushButton("BASEBAND SCOPE (DC to Fs/2)")
#         self.btn_scope.setCheckable(True)
#         self.btn_scope.setToolTip(
#             "Switch the display from the spectrum to a time-domain scope of the probe signal "
#             "from DC up to half the Sample Rate. The Sample Rate control sets the maximum input "
#             "frequency. Built from tuned IQ on the V4 (no direct sampling), so overall phase is "
#             "unknown - see the Phase control. Press again to return to the spectrum.")
#         self.btn_scope.toggled.connect(self.scope_toggle)
#         self.scope_status = QtWidgets.QLabel("")
#         self.scope_status.setFont(QtGui.QFont("Consolas", 9))
#         row_top.addWidget(self.btn_scope)
#         row_top.addWidget(self.scope_status, 1)
#         # ---- scope controls (hidden until scope mode) ----
#         self.scope_ctrl_widget = QtWidgets.QWidget()
#         vbox = QtWidgets.QVBoxLayout(self.scope_ctrl_widget)
#         vbox.setContentsMargins(0, 0, 0, 0)
#         r1 = QtWidgets.QHBoxLayout()
#         r2 = QtWidgets.QHBoxLayout()
#         vbox.addLayout(r1)
#         vbox.addLayout(r2)
#         self.scope_channel_combo = QtWidgets.QComboBox()
#         for txt in ("Signal", "Envelope", "Signal + envelope"):
#             self.scope_channel_combo.addItem(txt)
#         self.scope_channel_combo.setToolTip("Signal = the reconstructed waveform. Envelope = its magnitude "
#                                              "(faithful regardless of the unknown phase).")
#         self.scope_channel_combo.currentIndexChanged.connect(self.scope_on_channel_changed)
#         self.scope_window_combo = QtWidgets.QComboBox()
#         for ms in SCOPE_WINDOWS_MS:
#             label = f"{ms * 1000:g} us" if ms < 1 else f"{ms:g} ms"
#             self.scope_window_combo.addItem(label, ms / 1000.0)
#         self.scope_window_combo.setCurrentIndex(SCOPE_WINDOWS_MS.index(2))
#         self.scope_window_combo.setToolTip(
#             "Total time shown across the screen. Limited by SCOPE_MAX_SAMPLES at high sample "
#             "rates (the readout shows the window actually used). Lower sample rates reach "
#             "longer windows.")
#         self.scope_window_combo.currentIndexChanged.connect(self.scope_on_window_changed)
#         self.scope_bw_spin = QtWidgets.QDoubleSpinBox()
#         self.scope_bw_spin.setLocale(SCOPE_LOCALE)
#         self.scope_bw_spin.setRange(0.0, 1600.0)
#         self.scope_bw_spin.setDecimals(1)
#         self.scope_bw_spin.setSingleStep(10.0)
#         self.scope_bw_spin.setSuffix(" kHz")
#         self.scope_bw_spin.setSpecialValueText("Full")
#         self.scope_bw_spin.setKeyboardTracking(False)
#         self.scope_bw_spin.setToolTip(
#             "Optional bandwidth limit (like a scope's BW-limit button): a smooth low-pass, -3 dB "
#             "at this frequency, to reduce noise. Full = DC to Nyquist (Sample Rate / 2). "
#             "Values at or above Nyquist have no effect.")
#         self.scope_bw_spin.valueChanged.connect(self.scope_on_bw_changed)
#         self.scope_phase_spin = QtWidgets.QDoubleSpinBox()
#         self.scope_phase_spin.setLocale(SCOPE_LOCALE)
#         self.scope_phase_spin.setRange(0.0, 359.0)
#         self.scope_phase_spin.setDecimals(0)
#         self.scope_phase_spin.setSingleStep(15.0)
#         self.scope_phase_spin.setWrapping(True)
#         self.scope_phase_spin.setSuffix(" deg")
#         self.scope_phase_spin.setToolTip(
#             "Overall phase of the reconstructed waveform. The V4's tuner LO phase is random, so this "
#             "is unknown; it does not matter for a sine wave, but rotates the SHAPE of waveforms "
#             "with harmonics (square waves, pulses). Adjust until a known waveform looks right.")
#         self.scope_phase_spin.valueChanged.connect(self.scope_on_phase_changed)
#         self.chk_scope_lo = QtWidgets.QCheckBox("Remove LO spike")
#         self.chk_scope_lo.setChecked(True)
#         self.chk_scope_lo.setToolTip(
#             "The tuner's own DC spike shows up as a steady sine at exactly Fs/4. Tick to remove it. "
#             "(It also removes a genuine input signal at exactly Fs/4.)")
#         self.chk_scope_lo.toggled.connect(self.scope_on_lo_toggled)
#         r1.addWidget(QtWidgets.QLabel("Show"))
#         r1.addWidget(self.scope_channel_combo)
#         r1.addWidget(QtWidgets.QLabel("Window"))
#         r1.addWidget(self.scope_window_combo)
#         r1.addWidget(QtWidgets.QLabel("BW limit"))
#         r1.addWidget(self.scope_bw_spin)
#         r1.addWidget(QtWidgets.QLabel("Phase"))
#         r1.addWidget(self.scope_phase_spin)
#         r1.addWidget(self.chk_scope_lo)
#         r1.addStretch()
#         self.scope_trig_combo = QtWidgets.QComboBox()
#         for txt in ("Trigger: free run", "Trigger: rising edge", "Trigger: falling edge"):
#             self.scope_trig_combo.addItem(txt)
#         self.scope_trig_combo.setToolTip(
#             "Edge trigger on the first trace. If no edge is found the frame is shown "
#             "untriggered (the readout says so) so the display never freezes.")
#         self.scope_trig_combo.currentIndexChanged.connect(self.scope_on_trig_mode_changed)
#         self.scope_trig_spin = QtWidgets.QDoubleSpinBox()
#         self.scope_trig_spin.setLocale(SCOPE_LOCALE)
#         self.scope_trig_spin.setRange(-1.0, 1.0)
#         self.scope_trig_spin.setDecimals(4)
#         self.scope_trig_spin.setSingleStep(0.01)
#         self.scope_trig_spin.setPrefix("Level ")
#         self.scope_trig_spin.setToolTip("Trigger level as a fraction of ADC full scale. Also draggable as the "
#                                          "orange line on the plot.")
#         self.scope_trig_spin.valueChanged.connect(self.scope_on_trig_level_changed)
#         self.chk_scope_auto_y = QtWidgets.QCheckBox("Auto Y")
#         self.chk_scope_auto_y.setChecked(True)
#         self.chk_scope_auto_y.toggled.connect(self.scope_on_auto_y_toggled)
#         self.scope_y_spin = QtWidgets.QDoubleSpinBox()
#         self.scope_y_spin.setLocale(SCOPE_LOCALE)
#         self.scope_y_spin.setRange(0.0001, 10.0)
#         self.scope_y_spin.setDecimals(4)
#         self.scope_y_spin.setSingleStep(0.05)
#         self.scope_y_spin.setValue(0.5)
#         self.scope_y_spin.setPrefix("Y +/- ")
#         self.scope_y_spin.setEnabled(False)
#         self.scope_y_spin.setToolTip("Fixed vertical range (used when Auto Y is off).")
#         self.scope_y_spin.valueChanged.connect(self.scope_on_y_changed)
#         self.chk_scope_hold = QtWidgets.QCheckBox("Hold")
#         self.chk_scope_hold.setToolTip("Freeze the display on the current frame (the SDR stays open).")
#         self.chk_scope_hold.toggled.connect(self.scope_on_hold_toggled)
#         self.btn_scope_save = QtWidgets.QPushButton("SAVE SCOPE CSV")
#         self.btn_scope_save.clicked.connect(self.scope_save_csv)
#         r2.addWidget(self.scope_trig_combo)
#         r2.addWidget(self.scope_trig_spin)
#         r2.addWidget(self.chk_scope_auto_y)
#         r2.addWidget(self.scope_y_spin)
#         r2.addWidget(self.chk_scope_hold)
#         r2.addWidget(self.btn_scope_save)
#         r2.addStretch()
#         self.scope_ctrl_widget.setVisible(False)
#         left_layout.addWidget(self.scope_ctrl_widget)

    def scope_build_ui(self, left_layout):
        # ---- state ----
        self.scope_active = False
        self.scope_hold = False
        self.scope_channel = 0            # 0 signal, 1 envelope, 2 signal + envelope
        self.scope_window_s = 0.002
        self.scope_trig_mode = 0          # 0 free run, 1 rising, 2 falling
        self.scope_trig_level = 0.0
        self.scope_bw_hz = 0.0            # optional extra low-pass (0 = off, i.e. full DC..Nyquist)
        self.scope_phase_rad = 0.0        # manual overall phase rotation
        self.scope_remove_lo_spike = True
        self.scope_auto_y = False         # vertical range is MANUAL by default (Auto Y is optional)
        self._scope_pending = False       # a frame is queued for the GUI; don't flood it
        self._scope_last = None
        self._scope_mask_cache = None
        self.scope_signals = ScopeSignals()
        self.scope_signals.frame.connect(self.scope_on_frame)

        # ---- scope plot (sits where the spectrum plot is; shown only in scope mode) ----
        self.scope_plot = pg.PlotWidget()
        self.scope_plot.setLabel('bottom', 'Time', units='s')
        self.scope_plot.setLabel('left', 'Amplitude (1.0 = ADC full scale)')
        self.scope_plot.showGrid(x=True, y=True, alpha=0.3)
        self.scope_curve1 = self.scope_plot.plot(pen=pg.mkPen('y', width=1))
        self.scope_curve2 = self.scope_plot.plot(pen=pg.mkPen('c', width=1))
        for _c in (self.scope_curve1, self.scope_curve2):
            try:
                _c.setClipToView(True)
                _c.setDownsampling(auto=True, method='peak')
            except Exception:
                pass
        self.scope_curve2.hide()
        # draggable trigger-level line (orange) and trigger-point marker (grey, t = 0)
        self.scope_trig_line = pg.InfiniteLine(
            pos=0.0, angle=0, movable=True,
            pen=pg.mkPen((255, 128, 0), width=1, style=QtCore.Qt.DashLine))
        self.scope_plot.addItem(self.scope_trig_line)
        self.scope_zero_line = pg.InfiniteLine(
            pos=0.0, angle=90, movable=False,
            pen=pg.mkPen((140, 140, 140), width=1, style=QtCore.Qt.DotLine))
        self.scope_plot.addItem(self.scope_zero_line)
        self.scope_trig_line.sigPositionChanged.connect(self.scope_on_trig_line)
        self.scope_plot.setVisible(False)
        left_layout.insertWidget(left_layout.indexOf(self.plot), self.scope_plot)

        # ---- always-visible row: the mode button + a one-line readout ----
        row_top = QtWidgets.QHBoxLayout()
        left_layout.addLayout(row_top)
        self.btn_scope = QtWidgets.QPushButton("BASEBAND SCOPE (DC to Fs/2)")
        self.btn_scope.setCheckable(True)
        self.btn_scope.setToolTip(
            "Switch the display from the spectrum to a time-domain scope of the probe signal "
            "from DC up to half the Sample Rate. The Sample Rate control sets the maximum input "
            "frequency. Built from tuned IQ on the V4 (no direct sampling), so overall phase is "
            "unknown - see the Phase control. Press again to return to the spectrum.")
        self.btn_scope.toggled.connect(self.scope_toggle)
        self.scope_status = QtWidgets.QLabel("")
        self.scope_status.setFont(QtGui.QFont("Consolas", 9))
        row_top.addWidget(self.btn_scope)
        row_top.addWidget(self.scope_status, 1)

        # ---- scope controls (hidden until scope mode) ----
        self.scope_ctrl_widget = QtWidgets.QWidget()
        vbox = QtWidgets.QVBoxLayout(self.scope_ctrl_widget)
        vbox.setContentsMargins(0, 0, 0, 0)
        r1 = QtWidgets.QHBoxLayout()
        r2 = QtWidgets.QHBoxLayout()
        vbox.addLayout(r1)
        vbox.addLayout(r2)

        self.scope_channel_combo = QtWidgets.QComboBox()
        for txt in ("Signal", "Envelope", "Signal + envelope"):
            self.scope_channel_combo.addItem(txt)
        self.scope_channel_combo.setToolTip("Signal = the reconstructed waveform. Envelope = its magnitude "
                                             "(faithful regardless of the unknown phase).")
        self.scope_channel_combo.currentIndexChanged.connect(self.scope_on_channel_changed)

        self.scope_window_combo = QtWidgets.QComboBox()
        for ms in SCOPE_WINDOWS_MS:
            label = f"{ms * 1000:g} us" if ms < 1 else f"{ms:g} ms"
            self.scope_window_combo.addItem(label, ms / 1000.0)
        self.scope_window_combo.setCurrentIndex(SCOPE_WINDOWS_MS.index(2))
        self.scope_window_combo.setToolTip(
            "Total time shown across the screen. Limited by SCOPE_MAX_SAMPLES at high sample "
            "rates (the readout shows the window actually used). Lower sample rates reach "
            "longer windows.")
        self.scope_window_combo.currentIndexChanged.connect(self.scope_on_window_changed)

        self.scope_bw_spin = QtWidgets.QDoubleSpinBox()
        self.scope_bw_spin.setLocale(SCOPE_LOCALE)
        self.scope_bw_spin.setRange(0.0, 1600.0)
        self.scope_bw_spin.setDecimals(1)
        self.scope_bw_spin.setSingleStep(10.0)
        self.scope_bw_spin.setSuffix(" kHz")
        self.scope_bw_spin.setSpecialValueText("Full")
        self.scope_bw_spin.setKeyboardTracking(False)
        self.scope_bw_spin.setToolTip(
            "Optional bandwidth limit (like a scope's BW-limit button): a smooth low-pass, -3 dB "
            "at this frequency, to reduce noise. Full = DC to Nyquist (Sample Rate / 2). "
            "Values at or above Nyquist have no effect.")
        self.scope_bw_spin.valueChanged.connect(self.scope_on_bw_changed)

        self.scope_phase_spin = QtWidgets.QDoubleSpinBox()
        self.scope_phase_spin.setLocale(SCOPE_LOCALE)
        self.scope_phase_spin.setRange(0.0, 359.0)
        self.scope_phase_spin.setDecimals(0)
        self.scope_phase_spin.setSingleStep(15.0)
        self.scope_phase_spin.setWrapping(True)
        self.scope_phase_spin.setSuffix(" deg")
        self.scope_phase_spin.setToolTip(
            "Overall phase of the reconstructed waveform. The V4's tuner LO phase is random, so this "
            "is unknown; it does not matter for a sine wave, but rotates the SHAPE of waveforms "
            "with harmonics (square waves, pulses). Adjust until a known waveform looks right.")
        self.scope_phase_spin.valueChanged.connect(self.scope_on_phase_changed)

        self.chk_scope_lo = QtWidgets.QCheckBox("Remove LO spike")
        self.chk_scope_lo.setChecked(True)
        self.chk_scope_lo.setToolTip(
            "The tuner's own DC spike shows up as a steady sine at exactly Fs/4. Tick to remove it. "
            "(It also removes a genuine input signal at exactly Fs/4.)")
        self.chk_scope_lo.toggled.connect(self.scope_on_lo_toggled)

        r1.addWidget(QtWidgets.QLabel("Show"))
        r1.addWidget(self.scope_channel_combo)
        r1.addWidget(QtWidgets.QLabel("Window"))
        r1.addWidget(self.scope_window_combo)
        r1.addWidget(QtWidgets.QLabel("BW limit"))
        r1.addWidget(self.scope_bw_spin)
        r1.addWidget(QtWidgets.QLabel("Phase"))
        r1.addWidget(self.scope_phase_spin)
        r1.addWidget(self.chk_scope_lo)
        r1.addStretch()

        self.scope_trig_combo = QtWidgets.QComboBox()
        for txt in ("Trigger: free run", "Trigger: rising edge", "Trigger: falling edge"):
            self.scope_trig_combo.addItem(txt)
        self.scope_trig_combo.setToolTip(
            "Edge trigger on the first trace. If no edge is found the frame is shown "
            "untriggered (the readout says so) so the display never freezes.")
        self.scope_trig_combo.currentIndexChanged.connect(self.scope_on_trig_mode_changed)

        self.scope_trig_spin = QtWidgets.QDoubleSpinBox()
        self.scope_trig_spin.setLocale(SCOPE_LOCALE)
        self.scope_trig_spin.setRange(-1.0, 1.0)
        self.scope_trig_spin.setDecimals(6)
        self.scope_trig_spin.setSingleStep(0.01)
        self.scope_trig_spin.setPrefix("Level ")
        self.scope_trig_spin.setToolTip("Trigger level as a fraction of ADC full scale. Each up/down click moves it "
                                         "by 1/20 of the screen's full vertical range. Also draggable as the "
                                         "orange line on the plot.")
        self.scope_trig_spin.valueChanged.connect(self.scope_on_trig_level_changed)

        self.chk_scope_auto_y = QtWidgets.QCheckBox("Auto Y")
        self.chk_scope_auto_y.setChecked(False)
        self.chk_scope_auto_y.setToolTip("Let the display range itself to the data. Off (default) = you set the "
                                          "vertical range with the Y +/- box.")
        self.chk_scope_auto_y.toggled.connect(self.scope_on_auto_y_toggled)

        self.scope_y_spin = ScopeYSpin()   # steps 1-2-5 like a scope's V/div control
        self.scope_y_spin.setLocale(SCOPE_LOCALE)
        self.scope_y_spin.setRange(SCOPE_Y_MIN, SCOPE_Y_MAX)
        self.scope_y_spin.setDecimals(5)
        self.scope_y_spin.setSingleStep(0.05)
        self.scope_y_spin.setValue(SCOPE_Y_DEFAULT)
        self.scope_y_spin.setPrefix("Y +/- ")
        self.scope_y_spin.setEnabled(True)
        self.scope_y_spin.setSuffix(" FS")
        self.scope_y_spin.setToolTip("Vertical range of the screen, +/- this many ADC full scales (envelope alone: 0 to this). "
                                      "Up/down steps 1-2-5. Used when Auto Y is off.")
        self.scope_y_spin.valueChanged.connect(self.scope_on_y_changed)

        self.chk_scope_hold = QtWidgets.QCheckBox("Hold")
        self.chk_scope_hold.setToolTip("Freeze the display on the current frame (the SDR stays open).")
        self.chk_scope_hold.toggled.connect(self.scope_on_hold_toggled)

        self.btn_scope_save = QtWidgets.QPushButton("SAVE SCOPE CSV")
        self.btn_scope_save.clicked.connect(self.scope_save_csv)

        r2.addWidget(self.scope_trig_combo)
        r2.addWidget(self.scope_trig_spin)
        r2.addWidget(self.chk_scope_auto_y)
        r2.addWidget(self.scope_y_spin)
        r2.addWidget(self.chk_scope_hold)
        r2.addWidget(self.btn_scope_save)
        r2.addStretch()

        self.scope_ctrl_widget.setVisible(False)
        left_layout.addWidget(self.scope_ctrl_widget)

        # ---- vertical axis is controlled ONLY by the Y +/- box: no mouse zoom/pan in Y ----
        self.scope_plot.setMouseEnabled(x=True, y=False)
        # keep the trigger step tied to whatever the screen's full Y range is (also follows Auto Y)
        self.scope_plot.getViewBox().sigYRangeChanged.connect(self.scope_update_trig_step)
        self.scope_apply_y_range()

    def scope_build_trigger_ui(self):
        """Adds the Sweep (normal/auto) selector next to the trigger controls and wires up the
        fixed-time-axis housekeeping. Called once from __init__ after scope_build_ui()."""
        self.scope_sweep_normal = True      # NORMAL: wait for a trigger (default); AUTO: free-run if none
        self.scope_trig_hyst = 0.0
        self._scope_x_follow = True         # hold the time axis at the trigger-relative span
        self.scope_sweep_combo = QtWidgets.QComboBox()
        self.scope_sweep_combo.addItem("Sweep: normal")
        self.scope_sweep_combo.addItem("Sweep: auto")
        self.scope_sweep_combo.setToolTip(
            "Only matters when an edge trigger is selected. Normal: if no edge crosses the trigger "
            "level the display keeps the last triggered frame and the readout says WAITING. Auto: "
            "free-runs (untriggered) when no edge is found, so the screen never freezes.")
        self.scope_sweep_combo.currentIndexChanged.connect(self.scope_on_sweep_changed)
        try:
            # row 2 of the control panel holds the trigger combo at index 0: put this right after it
            self.scope_ctrl_widget.layout().itemAt(1).layout().insertWidget(1, self.scope_sweep_combo)
        except Exception:
            self.scope_ctrl_widget.layout().addWidget(self.scope_sweep_combo)
        self.scope_plot.getViewBox().sigRangeChangedManually.connect(self.scope_on_manual_range)
        self.scope_window_combo.currentIndexChanged.connect(self.scope_follow_x_again)
        self.btn_scope.toggled.connect(self.scope_follow_x_again)
        self.scope_update_trig_step()

    def scope_on_sweep_changed(self, idx):
        self.scope_sweep_normal = (int(idx) == 0)

    def scope_on_manual_range(self, *args):
        self._scope_x_follow = False        # the user zoomed/panned the time axis: stop re-framing it

    def scope_follow_x_again(self, *args):
        self._scope_x_follow = True

    def scope_build_pulse_ui(self):
        """Adds the pulse marking / rejection row to the scope control panel (called once from __init__)."""
        self.scope_mark_pulses = True
        self.scope_reject_pulses = False
        self._scope_rejected = 0
        self.scope_pulse_marks = pg.ScatterPlotItem(size=8, symbol='o', pen=pg.mkPen((255, 40, 40), width=1.5),
                                                    brush=pg.mkBrush(None))
        self.scope_plot.addItem(self.scope_pulse_marks)
        self.chk_scope_mark = QtWidgets.QCheckBox("Mark pulses")
        self.chk_scope_mark.setChecked(True)
        self.chk_scope_mark.setToolTip(
            "Circle isolated spikes that stand far outside the trace's own noise (more than %g robust "
            "sigmas) in red and count them in the readout. Marking only - the trace is not altered." % SCOPE_PULSE_K)
        self.chk_scope_mark.toggled.connect(self.scope_on_mark_toggled)
        self.chk_scope_reject = QtWidgets.QCheckBox("Reject pulsed frames")
        self.chk_scope_reject.setToolTip(
            "Do not draw a frame that contains a marked pulse; keep the last clean frame instead. "
            "This hides real transients as well, so switch it off when hunting for them. If frames are "
            "rejected constantly the stream itself is glitching (see the sample-rate warning).")
        self.chk_scope_reject.toggled.connect(self.scope_on_reject_toggled)
        r3 = QtWidgets.QHBoxLayout()
        r3.addWidget(self.chk_scope_mark)
        r3.addWidget(self.chk_scope_reject)
        r3.addStretch()
        self.scope_ctrl_widget.layout().addLayout(r3)

    def scope_on_mark_toggled(self, checked):
        self.scope_mark_pulses = bool(checked)
        if not checked:
            self.scope_pulse_marks.setData(x=[], y=[])

    def scope_on_reject_toggled(self, checked):
        self.scope_reject_pulses = bool(checked)
        self._scope_rejected = 0

    # ---- GUI-thread setters (plain attributes read by scope_loop) ----

    def scope_on_channel_changed(self, idx):
        self.scope_channel = int(idx)
        self.scope_curve2.setVisible(self.scope_channel == 2 and self._scope_last is not None)
        self.scope_apply_y_range()

    def scope_on_window_changed(self, idx):
        sec = self.scope_window_combo.itemData(idx)
        if sec is not None:
            self.scope_window_s = float(sec)
        self.scope_plot.enableAutoRange(axis=pg.ViewBox.XAxis, enable=True)

    def scope_on_bw_changed(self, val):
        self.scope_bw_hz = float(val) * 1e3

    def scope_on_phase_changed(self, val):
        self.scope_phase_rad = math.radians(float(val))

    def scope_on_lo_toggled(self, checked):
        self.scope_remove_lo_spike = bool(checked)

    def scope_on_trig_mode_changed(self, idx):
        self.scope_trig_mode = int(idx)

    def scope_on_trig_level_changed(self, val):
        self.scope_trig_level = float(val)
        self.scope_trig_line.blockSignals(True)
        self.scope_trig_line.setValue(self.scope_trig_level)
        self.scope_trig_line.blockSignals(False)

    def scope_on_trig_line(self, line):
        val = max(-1.0, min(1.0, float(line.value())))
        self.scope_trig_level = val
        self.scope_trig_spin.blockSignals(True)
        self.scope_trig_spin.setValue(val)
        self.scope_trig_spin.blockSignals(False)

    def scope_on_auto_y_toggled(self, checked):
        self.scope_auto_y = bool(checked)
        self.scope_y_spin.setEnabled(not self.scope_auto_y)
        self.scope_apply_y_range()

    def scope_on_y_changed(self, val):
        self.scope_apply_y_range()

    def scope_on_hold_toggled(self, checked):
        self.scope_hold = bool(checked)

    # ---- previous scope_apply_y_range(), kept in case of problems ----
#     def scope_apply_y_range(self):
#         """Auto Y on -> let pyqtgraph range it; off -> fixed +/- range (0..+ for the envelope alone)."""
#         try:
#             if self.scope_auto_y:
#                 self.scope_plot.enableAutoRange(axis=pg.ViewBox.YAxis, enable=True)
#             else:
#                 v = float(self.scope_y_spin.value())
#                 lo = 0.0 if self.scope_channel == 1 else -v
#                 self.scope_plot.setYRange(lo, v, padding=0)
#         except Exception:
#             pass

    def scope_apply_y_range(self):
        """
        Manual (default): the screen shows +/- Y (0..Y for the envelope alone), with 10 divisions
        - so one trigger step (1/20 of the range) is half a division. Auto Y: pyqtgraph ranges it.
        """
        try:
            ax = self.scope_plot.getAxis('left')
            if self.scope_auto_y:
                self.scope_plot.enableAutoRange(axis=pg.ViewBox.YAxis, enable=True)
                ax.setTickSpacing()                         # back to automatic ticks
            else:
                v = float(self.scope_y_spin.value())
                lo = 0.0 if self.scope_channel == 1 else -v
                self.scope_plot.setYRange(lo, v, padding=0)
                full = v - lo
                ax.setTickSpacing(major=full / 10.0, minor=full / 20.0)
            self.scope_update_trig_step()
        except Exception:
            pass

    # ---- previous scope_update_trig_step(), kept in case of problems ----
#     def scope_update_trig_step(self, *args):
#         """Trigger level spin box: each up/down click moves 1/20 of the screen's full Y range."""
#         try:
#             if self.scope_auto_y:
#                 yr = self.scope_plot.getViewBox().viewRange()[1]
#                 full = float(yr[1]) - float(yr[0])
#             else:
#                 v = float(self.scope_y_spin.value())
#                 full = v - (0.0 if self.scope_channel == 1 else -v)
#             if full > 0.0:
#                 self.scope_trig_spin.setSingleStep(full / 20.0)
#         except Exception:
#             pass

    def scope_update_trig_step(self, *args):
        """Trigger level spin box: each up/down click moves 1/20 of the screen's full Y range.
        Also sets the trigger hysteresis (a fraction of the same range) used by the edge trigger."""
        try:
            if self.scope_auto_y:
                yr = self.scope_plot.getViewBox().viewRange()[1]
                full = float(yr[1]) - float(yr[0])
            else:
                v = float(self.scope_y_spin.value())
                full = v - (0.0 if self.scope_channel == 1 else -v)
            if full > 0.0:
                self.scope_trig_spin.setSingleStep(full / 20.0)
                self.scope_trig_hyst = full * SCOPE_TRIG_HYST_OF_RANGE
        except Exception:
            pass

    # ---- mode entry / exit (GUI thread) ----

    # ---- previous scope_toggle(), kept in case of problems ----
#     def scope_toggle(self, checked):
#         if checked:
#             if not self.scope_enter():
#                 self.btn_scope.blockSignals(True)
#                 self.btn_scope.setChecked(False)
#                 self.btn_scope.blockSignals(False)
#         else:
#             self.scope_exit()

    # ---- previous scope_toggle(), kept in case of problems ----
#     def scope_toggle(self, checked):
#         if checked:
#             self.scope_limit_rates(True)            # BEFORE the SDR is opened, so it never starts at 3.0/3.2 MS/s
#             if not self.scope_enter():
#                 self.btn_scope.blockSignals(True)
#                 self.btn_scope.setChecked(False)
#                 self.btn_scope.blockSignals(False)
#                 self.scope_limit_rates(False)
#         else:
#             self.scope_exit()
#             self.scope_limit_rates(False)

    def scope_toggle(self, checked):
        if checked:
            self.scope_limit_rates(True)            # BEFORE the SDR is opened, so it never starts at 3.0/3.2 MS/s
            if not self.scope_enter():
                self.btn_scope.blockSignals(True)
                self.btn_scope.setChecked(False)
                self.btn_scope.blockSignals(False)
                self.scope_limit_rates(False)
        else:
            self.scope_exit()
            self.scope_limit_rates(False)
            self.continuous_when_idle()             # back on the spectrum: CONTINUOUS starts by itself

    def scope_limit_rates(self, limit):
        """
        In scope mode the Sample Rate combo only offers rates up to SCOPE_MAX_RATE: above that the
        receiver's stream carries a regular pulse train (not the probe, not the processing), so the
        scope would show it as signal. Entering the scope above the limit switches to
        SCOPE_SAFE_RATE; leaving puts the previous rate back.
        """
        combo = self.sample_rate_combo
        model = combo.model()
        for i in range(combo.count()):
            item = model.item(i) if hasattr(model, "item") else None
            rate = combo.itemData(i)
            if item is None or rate is None:
                continue
            item.setEnabled((not limit) or float(rate) <= SCOPE_MAX_RATE + 1.0)
        if limit:
            self._scope_prev_rate = None
            if float(self.SAMPLE_RATE) > SCOPE_MAX_RATE + 1.0:
                i = combo.findData(SCOPE_SAFE_RATE)
                if i >= 0:
                    self._scope_prev_rate = float(self.SAMPLE_RATE)
                    self.log(f"Scope: sample rate limited to {SCOPE_MAX_RATE/1e6:.2f} MS/s or less - "
                             f"switching from {self.SAMPLE_RATE/1e6:.3f} to {SCOPE_SAFE_RATE/1e6:.3f} MS/s")
                    combo.setCurrentIndex(i)        # runs sample_rate_on_changed()
        else:
            prev = getattr(self, "_scope_prev_rate", None)
            self._scope_prev_rate = None
            if prev is not None:
                i = combo.findData(prev)
                if i >= 0:
                    combo.setCurrentIndex(i)

    def scope_tame_status_label(self):
        """
        The scope readout line is long (and longer still with pulse counts / warnings). A QLabel
        refuses to be narrower than its text, so it used to push the whole program window wider -
        off the screen. Now it ignores its text width, wraps onto at most three lines and has a
        fixed height, so no settings change can resize the window.
        """
        lab = self.scope_status
        lab.setWordWrap(True)
        lab.setSizePolicy(QtWidgets.QSizePolicy.Ignored, QtWidgets.QSizePolicy.Fixed)
        lab.setMinimumWidth(0)
        lab.setFixedHeight(3 * lab.fontMetrics().lineSpacing() + 6)
        lab.setAlignment(QtCore.Qt.AlignLeft | QtCore.Qt.AlignVCenter)

    def scope_set_lock(self, locked):
        """Spectrum-only controls are disabled while the scope owns the SDR."""
        for wdg in ([self.btn_start, self.btn_cont, self.btn_survey] + list(self.mode_buttons)):
            wdg.setEnabled(not locked)

    # ---- previous scope_enter(), kept in case of problems ----
#     def scope_enter(self):
#         if self._survey is not None:
#             self.log("Scope: not available while a survey is running")
#             return False
#         # Stop any sweep first so two threads never share the SDR.
#         if self.sweep_thread and self.sweep_thread.is_alive():
#             self.log("=== SCOPE: stopping current sweep ===")
#             self.stop_flag.set()
#             self.sweep_thread.join(timeout=3.0)
#             if self.sweep_thread.is_alive():
#                 self.log("Scope: ERROR - the sweep thread did not stop; try again")
#                 return False
#         self.stop_flag.clear()
#         try:
#             self.init_sdr()
#         except Exception as e:
#             self.log(f"Scope: error opening the SDR: {e}")
#             return False
#         self._scope_pending = False
#         self.scope_active = True
#         self.scope_set_lock(True)
#         self.plot.hide()
#         self.scope_plot.show()
#         self.scope_ctrl_widget.setVisible(True)
#         self.btn_scope.setText("BACK TO SPECTRUM")
#         self.scope_apply_y_range()
#         self.log("=== BASEBAND SCOPE STARTED ===")
#         self.sweep_thread = threading.Thread(target=self.scope_loop, daemon=True)
#         self.sweep_thread.start()
#         return True

    def scope_enter(self):
        if self._survey is not None:
            self.log("Scope: not available while a survey is running")
            return False
        # Stop any sweep first so two threads never share the SDR.
        if self.sweep_thread and self.sweep_thread.is_alive():
            self.log("=== SCOPE: stopping current sweep ===")
            self.stop_flag.set()
            self.sweep_thread.join(timeout=3.0)
            if self.sweep_thread.is_alive():
                self.log("Scope: ERROR - the sweep thread did not stop; try again")
                return False
        self.stop_flag.clear()
        try:
            self.init_sdr()
        except Exception as e:
            self.log(f"Scope: error opening the SDR: {e}")
            return False
        self._scope_pending = False
        self.scope_active = True
        self.scope_set_lock(True)
        self.plot.hide()
        self.scope_plot.show()
        self.scope_ctrl_widget.setVisible(True)
        self.btn_scope.setText("BACK TO SPECTRUM")
        self.scope_apply_y_range()
        self.log("=== BASEBAND SCOPE STARTED ===")
        self.sweep_thread = threading.Thread(target=self.scope_main, daemon=True)
        self.sweep_thread.start()
        return True

    def scope_main(self):
        """Scope worker thread entry: settle the radio (tune to Fs/4, re-apply gain) then run the scope."""
        try:
            self.settle_radio(float(self._applied_sample_rate) / 4.0)
        except Exception as e:
            print(f"scope_main: settle_radio failed: {e!r}")
        self.scope_loop()

    def scope_exit(self):
        self.stop_flag.set()
        if self.sweep_thread and self.sweep_thread.is_alive():
            self.sweep_thread.join(timeout=3.0)
        self.scope_active = False
        self.scope_ctrl_widget.setVisible(False)
        self.scope_plot.hide()
        self.plot.show()
        self.btn_scope.setText("BASEBAND SCOPE (DC to Fs/2)")
        self.scope_status.setText("")
        self.scope_set_lock(False)
        try:
            self.cispr_sync_viewbox_geometry()
        except Exception:
            pass
        self.log("=== BASEBAND SCOPE STOPPED ===")

    # ---- worker thread ----

    def _scope_read_iq(self, n):
        """
        Read n IQ samples as normalised complex64 - the same real/complex and uint8/float
        handling and short-read retry as _read_normalized_iq(), but WITHOUT the DC-removal /
        IQ-balance corrections (the scope does its own LO-spike removal, see
        scope_process_capture()).
        """
        last_err = None
        samples = None
        for attempt in range(3):
            try:
                samples = self.sdr.read_samples(n)
                last_err = None
                break
            except OSError as e:
                last_err = e
                print(f"_scope_read_iq: read_samples failed (attempt {attempt + 1}/3): {e}")
                time.sleep(0.05)
        if last_err is not None:
            raise last_err
        samples = samples[:n]
        if np.iscomplexobj(samples):
            return samples.astype(np.complex64) / SCOPE_COMPLEX_DIVISOR
        if samples.dtype == np.uint8:
            f = samples.astype(np.float32) - 128.0
        else:
            f = samples.astype(np.float32)
        if (f.size % 2) != 0:
            f = f[:-1]
        f = f.reshape(-1, 2)
        return (f[:, 0] + 1j * f[:, 1]).astype(np.complex64) / 128.0

    @staticmethod
    def _scope_fmt_hz(f):
        f = abs(float(f))
        if f >= 1e6:
            return f"{f / 1e6:.4f} MHz"
        return f"{f / 1e3:.2f} kHz"

    # ---- previous scope_process_capture(), kept in case of problems ----
#     def scope_process_capture(self, x, rate, n_disp, fc, gain):
#         """
#         Turn one raw IQ capture (tuned at fc = rate/4) into (time_s, trace1, trace2_or_None,
#         info) ready to plot. Steps: metrics on the RAW capture (peak / clipping / DC) ->
#         optional removal of the tuner's DC spike -> keep only RF 0..Nyquist (rejects the mirror
#         image and anything above Nyquist; optional BW-limit low-pass) -> shift back to true RF
#         and take the real part -> edge trigger -> slice n_disp samples for display.
#         """
#         n_total = int(x.size)
#         n_disp = min(int(n_disp), n_total)
#         # ---- metrics on the raw capture ----
#         i_raw = x.real
#         q_raw = x.imag
#         peak = float(max(np.max(np.abs(i_raw)), np.max(np.abs(q_raw))))
#         clipped = bool(np.any((np.abs(i_raw) >= SCOPE_CLIP_LEVEL) | (np.abs(q_raw) >= SCOPE_CLIP_LEVEL)))
#         # ---- the tuner's own DC spike sits at the exact centre = RF Fs/4 ----
#         if self.scope_remove_lo_spike:
#             x = x - np.mean(x)
#         # ---- band select: keep RF 0 .. Nyquist (mask cached; it only depends on these four) ----
#         bw = float(self.scope_bw_hz)
#         nyq = rate / 2.0
#         key = (n_total, float(rate), float(fc), bw)
#         cache = self._scope_mask_cache
#         if cache is None or cache[0] != key:
#             nu = np.fft.fftfreq(n_total, d=1.0 / rate) + fc          # RF frequency of every FFT bin
#             h = ((nu >= 0.0) & (nu <= nyq)).astype(np.float64)
#             t_edge = (1.0 - SCOPE_TOP_TAPER) * nyq                   # smooth skirt just below Nyquist
#             sk = (nu > t_edge) & (nu <= nyq)
#             h[sk] = 0.5 * (1.0 + np.cos(np.pi * (nu[sk] - t_edge) / (nyq - t_edge)))
#             if 0.0 < bw < nyq:
#                 h *= np.exp(-0.5 * math.log(2.0) * (nu / bw) ** 2)   # -3 dB at nu = bw
#             self._scope_mask_cache = cache = (key, h, nu)
#         h, nu = cache[1], cache[2]
#         band = np.fft.fft(x) * h
#         z = np.fft.ifft(band)                                        # one-sided (analytic) signal
#         # ---- shift back to true RF (Fs/4 shift = 1, j, -1, -j) and take the real part ----
#         lut = np.array([1.0, 1j, -1.0, -1j], dtype=np.complex128) * np.exp(1j * float(self.scope_phase_rad))
#         sig = (z * lut[np.arange(n_total) & 3]).real.astype(np.float32)
#         env = np.abs(z).astype(np.float32)
#         ch = int(self.scope_channel)
#         tr2 = None
#         if ch == 0:
#             tr1 = sig
#         elif ch == 1:
#             tr1 = env
#         else:
#             tr1, tr2 = sig, env
#         # ---- edge trigger on trace 1 (10 % pre-trigger) ----
#         mode = int(self.scope_trig_mode)
#         pre = int(0.1 * n_disp)
#         start = 0
#         t0_idx = 0
#         trig_text = "free run"
#         if mode in (1, 2):
#             lvl = float(self.scope_trig_level)
#             a = tr1[:-1]
#             b = tr1[1:]
#             if mode == 1:
#                 hits = np.nonzero((a < lvl) & (b >= lvl))[0] + 1
#             else:
#                 hits = np.nonzero((a > lvl) & (b <= lvl))[0] + 1
#             hits = hits[(hits >= pre) & (hits <= n_total - n_disp + pre)]
#             if hits.size:
#                 start = int(hits[0]) - pre
#                 t0_idx = pre
#                 trig_text = "triggered"
#             else:
#                 trig_text = "NO TRIGGER (showing free run)"
#         sl = slice(start, start + n_disp)
#         tr1 = tr1[sl]
#         if tr2 is not None:
#             tr2 = tr2[sl]
#         t = (np.arange(n_disp, dtype=np.float64) - t0_idx) / rate
#         vpp = float(np.max(sig[sl]) - np.min(sig[sl])) if n_disp else 0.0
#         kpk = int(np.argmax(np.abs(band)))
#         text = (f"Input DC-{self._scope_fmt_hz(nyq)} (Fs {rate / 1e6:.3f} MS/s) | "
#                 f"{n_disp / rate * 1e3:.3f} ms | gain {gain:.1f} dB | "
#                 f"peak {peak:.2f} FS{'  ** CLIPPING **' if clipped else ''} | "
#                 f"pk-pk {vpp:.3g} FS | strongest {self._scope_fmt_hz(nu[kpk])} | {trig_text}")
#         info = {"text": text, "clip": clipped, "trig": trig_text, "channel": ch}
#         return t, tr1, tr2, info

    # ---- previous scope_process_capture(), kept in case of problems ----
#     def scope_process_capture(self, x, rate, n_disp, fc, gain, guard=0):
#         """
#         Turn one raw IQ capture (tuned at fc = rate/4) into (time_s, trace1, trace2_or_None,
#         info) ready to plot. x holds n_disp-plus-headroom samples PLUS `guard` extra samples at
#         each end. Steps: metrics on the RAW capture -> optional removal of the tuner's DC spike
#         -> keep only RF 0..Nyquist with SMOOTH edges at both ends -> shift back to true RF and
#         take the real part -> TRIM the guard samples off both ends -> edge trigger -> slice.
#         Why the guard and the smooth lower edge (they cure the start/end/middle spikes):
#         filtering with an FFT treats the capture as endlessly repeating, so anything that is not
#         perfectly periodic in the frame - above all a strong, steady component sitting near a
#         sharp filter edge, such as the upconverter's 28.8 MHz LO feed-through at RF ~0 - shows up
#         as log-shaped spikes at the frame ends (and at any step/phase jump inside it) with long
#         tails across the whole trace. The lower edge is now a smooth high-pass that stops that
#         feed-through, and the frame ends - where the wrap-around lands - are captured but trimmed
#         away before display.
#         """
#         n_cap = int(x.size)
#         guard = int(max(0, min(guard, (n_cap - 256) // 2)))
#         n_total = n_cap - 2 * guard                     # samples that survive the trim
#         n_disp = min(int(n_disp), n_total)
#         # ---- metrics on the raw capture ----
#         i_raw = x.real
#         q_raw = x.imag
#         peak = float(max(np.max(np.abs(i_raw)), np.max(np.abs(q_raw))))
#         clipped = bool(np.any((np.abs(i_raw) >= SCOPE_CLIP_LEVEL) | (np.abs(q_raw) >= SCOPE_CLIP_LEVEL)))
#         # ---- the tuner's own DC spike sits at the exact centre = RF Fs/4 ----
#         if self.scope_remove_lo_spike:
#             x = x - np.mean(x)
#         # ---- band select: RF 0..Nyquist, smooth at BOTH edges (mask cached) ----
#         bw = float(self.scope_bw_hz)
#         nyq = rate / 2.0
#         key = (n_cap, float(rate), float(fc), bw)
#         cache = self._scope_mask_cache
#         if cache is None or cache[0] != key:
#             nu = np.fft.fftfreq(n_cap, d=1.0 / rate) + fc            # RF frequency of every FFT bin
#             h = ((nu >= 0.0) & (nu <= nyq)).astype(np.float64)
#             f1 = SCOPE_LOW_CUT_FRAC * nyq                            # lower edge: stop below f1,
#             f2 = 3.0 * f1                                            # pass above f2, raised cosine between
#             lo = (nu > f1) & (nu < f2)
#             h[(nu <= f1)] = 0.0
#             h[lo] = 0.5 * (1.0 - np.cos(np.pi * (nu[lo] - f1) / (f2 - f1)))
#             t_edge = (1.0 - SCOPE_TOP_TAPER) * nyq                   # upper edge: smooth skirt below Nyquist
#             sk = (nu > t_edge) & (nu <= nyq)
#             h[sk] = 0.5 * (1.0 + np.cos(np.pi * (nu[sk] - t_edge) / (nyq - t_edge)))
#             if 0.0 < bw < nyq:
#                 h *= np.exp(-0.5 * math.log(2.0) * (nu / bw) ** 2)   # -3 dB at nu = bw
#             self._scope_mask_cache = cache = (key, h, nu)
#         h, nu = cache[1], cache[2]
#         band = np.fft.fft(x) * h
#         z = np.fft.ifft(band)                                        # one-sided (analytic) signal
#         # ---- shift back to true RF (Fs/4 shift = 1, j, -1, -j) and take the real part ----
#         lut = np.array([1.0, 1j, -1.0, -1j], dtype=np.complex128) * np.exp(1j * float(self.scope_phase_rad))
#         sig = (z * lut[np.arange(n_cap) & 3]).real.astype(np.float32)
#         env = np.abs(z).astype(np.float32)
#         if guard:                                                    # drop the wrap-around zones
#             sig = sig[guard:n_cap - guard]
#             env = env[guard:n_cap - guard]
#         ch = int(self.scope_channel)
#         tr2 = None
#         if ch == 0:
#             tr1 = sig
#         elif ch == 1:
#             tr1 = env
#         else:
#             tr1, tr2 = sig, env
#         # ---- edge trigger on trace 1 (10 % pre-trigger) ----
#         mode = int(self.scope_trig_mode)
#         pre = int(0.1 * n_disp)
#         start = 0
#         t0_idx = 0
#         trig_text = "free run"
#         if mode in (1, 2):
#             lvl = float(self.scope_trig_level)
#             a = tr1[:-1]
#             b = tr1[1:]
#             if mode == 1:
#                 hits = np.nonzero((a < lvl) & (b >= lvl))[0] + 1
#             else:
#                 hits = np.nonzero((a > lvl) & (b <= lvl))[0] + 1
#             hits = hits[(hits >= pre) & (hits <= n_total - n_disp + pre)]
#             if hits.size:
#                 start = int(hits[0]) - pre
#                 t0_idx = pre
#                 trig_text = "triggered"
#             else:
#                 trig_text = "NO TRIGGER (showing free run)"
#         sl = slice(start, start + n_disp)
#         tr1 = tr1[sl]
#         if tr2 is not None:
#             tr2 = tr2[sl]
#         t = (np.arange(n_disp, dtype=np.float64) - t0_idx) / rate
#         vpp = float(np.max(sig[sl]) - np.min(sig[sl])) if n_disp else 0.0
#         kpk = int(np.argmax(np.abs(band)))
#         text = (f"Input DC-{self._scope_fmt_hz(nyq)} (Fs {rate / 1e6:.3f} MS/s) | "
#                 f"{n_disp / rate * 1e3:.3f} ms | gain {gain:.1f} dB | "
#                 f"peak {peak:.2f} FS{'  ** CLIPPING **' if clipped else ''} | "
#                 f"pk-pk {vpp:.3g} FS | strongest {self._scope_fmt_hz(nu[kpk])} | {trig_text}")
#         info = {"text": text, "clip": clipped, "trig": trig_text, "channel": ch}
#         return t, tr1, tr2, info

    def scope_process_capture(self, x, rate, n_disp, fc, gain, guard=0):
        """
        Turn one raw IQ capture (tuned at fc = rate/4) into (time_s, trace1, trace2_or_None,
        info) ready to plot. x holds n_disp-plus-headroom samples PLUS `guard` extra samples at
        each end. Steps: metrics on the RAW capture -> optional removal of the tuner's DC spike
        -> keep only RF 0..Nyquist with SMOOTH edges at both ends -> shift back to true RF and
        take the real part -> TRIM the guard samples off both ends -> edge trigger -> slice.

        Why the guard and the smooth lower edge (they cure the start/end/middle spikes):
        filtering with an FFT treats the capture as endlessly repeating, so anything that is not
        perfectly periodic in the frame - above all a strong, steady component sitting near a
        sharp filter edge, such as the upconverter's 28.8 MHz LO feed-through at RF ~0 - shows up
        as log-shaped spikes at the frame ends (and at any step/phase jump inside it) with long
        tails across the whole trace. The lower edge is now a smooth high-pass that stops that
        feed-through, and the frame ends - where the wrap-around lands - are captured but trimmed
        away before display.
        """
        n_cap = int(x.size)
        guard = int(max(0, min(guard, (n_cap - 256) // 2)))
        n_total = n_cap - 2 * guard                     # samples that survive the trim
        n_disp = min(int(n_disp), n_total)

        # ---- metrics on the raw capture ----
        i_raw = x.real
        q_raw = x.imag
        peak = float(max(np.max(np.abs(i_raw)), np.max(np.abs(q_raw))))
        clipped = bool(np.any((np.abs(i_raw) >= SCOPE_CLIP_LEVEL) | (np.abs(q_raw) >= SCOPE_CLIP_LEVEL)))

        # ---- the tuner's own DC spike sits at the exact centre = RF Fs/4 ----
        if self.scope_remove_lo_spike:
            x = x - np.mean(x)

        # ---- band select: RF 0..Nyquist, smooth at BOTH edges (mask cached) ----
        bw = float(self.scope_bw_hz)
        nyq = rate / 2.0
        key = (n_cap, float(rate), float(fc), bw)
        cache = self._scope_mask_cache
        if cache is None or cache[0] != key:
            nu = np.fft.fftfreq(n_cap, d=1.0 / rate) + fc            # RF frequency of every FFT bin
            h = ((nu >= 0.0) & (nu <= nyq)).astype(np.float64)
            f1 = SCOPE_LOW_CUT_FRAC * nyq                            # lower edge: stop below f1,
            f2 = 3.0 * f1                                            # pass above f2, raised cosine between
            lo = (nu > f1) & (nu < f2)
            h[(nu <= f1)] = 0.0
            h[lo] = 0.5 * (1.0 - np.cos(np.pi * (nu[lo] - f1) / (f2 - f1)))
            t_edge = (1.0 - SCOPE_TOP_TAPER) * nyq                   # upper edge: smooth skirt below Nyquist
            sk = (nu > t_edge) & (nu <= nyq)
            h[sk] = 0.5 * (1.0 + np.cos(np.pi * (nu[sk] - t_edge) / (nyq - t_edge)))
            if 0.0 < bw < nyq:
                h *= np.exp(-0.5 * math.log(2.0) * (nu / bw) ** 2)   # -3 dB at nu = bw
            self._scope_mask_cache = cache = (key, h, nu)
        h, nu = cache[1], cache[2]
        band = np.fft.fft(x) * h
        z = np.fft.ifft(band)                                        # one-sided (analytic) signal

        # ---- shift back to true RF (Fs/4 shift = 1, j, -1, -j) and take the real part ----
        lut = np.array([1.0, 1j, -1.0, -1j], dtype=np.complex128) * np.exp(1j * float(self.scope_phase_rad))
        sig = (z * lut[np.arange(n_cap) & 3]).real.astype(np.float32)
        env = np.abs(z).astype(np.float32)
        if guard:                                                    # drop the wrap-around zones
            sig = sig[guard:n_cap - guard]
            env = env[guard:n_cap - guard]

        ch = int(self.scope_channel)
        tr2 = None
        if ch == 0:
            tr1 = sig
        elif ch == 1:
            tr1 = env
        else:
            tr1, tr2 = sig, env

        # ---- edge trigger on trace 1 (10 % pre-trigger) ----
        # Works on the RECONSTRUCTED trace you are looking at (band-limited, guard trimmed) - not
        # on the raw IQ - over the whole captured stretch (display + headroom), and picks the
        # first genuine edge. "Genuine" = a Schmitt trigger: the trace must first have been
        # beyond the level by the hysteresis (a fraction of the screen's Y range) before the
        # crossing counts, so noise wobbling around the level cannot fire it. The crossing time
        # is interpolated between samples, so every triggered sweep lines up on the exact
        # instant of the crossing (t = 0), not just to the nearest sample.
        mode = int(self.scope_trig_mode)
        pre = int(0.1 * n_disp)
        start = 0
        t0_pos = 0.0                        # (fractional) sample index that is plotted at t = 0
        trig_text = "free run"
        waiting = False
        if mode in (1, 2):
            lvl = float(self.scope_trig_level)
            hyst = max(0.0, float(getattr(self, "scope_trig_hyst", 0.0)))
            a = tr1[:-1]
            b = tr1[1:]
            if mode == 1:
                cand = np.nonzero((a < lvl) & (b >= lvl))[0] + 1
                armed = tr1 < (lvl - hyst)
            else:
                cand = np.nonzero((a > lvl) & (b <= lvl))[0] + 1
                armed = tr1 > (lvl + hyst)
            last_armed = np.maximum.accumulate(np.where(armed, np.arange(tr1.size), -1))
            cand = cand[(cand >= pre) & (cand <= n_total - n_disp + pre)]
            cand = cand[last_armed[cand - 1] >= 0]
            if cand.size:
                hit = int(cand[0])
                y0 = float(tr1[hit - 1])
                y1 = float(tr1[hit])
                frac = (lvl - y0) / (y1 - y0) if y1 != y0 else 0.0
                start = hit - pre
                t0_pos = (hit - 1) + frac
                trig_text = "triggered"
            elif getattr(self, "scope_sweep_normal", False):
                waiting = True                # NORMAL sweep: no edge -> keep the last triggered frame
                trig_text = f"WAITING FOR TRIGGER (level {lvl:.4g}) - showing last triggered frame"
            else:
                trig_text = "NO TRIGGER (auto: free run)"   # AUTO sweep: free-run so the screen never freezes

        sl = slice(start, start + n_disp)
        tr1 = tr1[sl]
        if tr2 is not None:
            tr2 = tr2[sl]
        t = (np.arange(n_disp, dtype=np.float64) + start - t0_pos) / rate
        pre_shown = pre if trig_text == "triggered" else 0
        xrange_s = (-pre_shown / rate, (n_disp - 1 - pre_shown) / rate)

        vpp = float(np.max(sig[sl]) - np.min(sig[sl])) if n_disp else 0.0
        kpk = int(np.argmax(np.abs(band)))
        text = (f"Input DC-{self._scope_fmt_hz(nyq)} (Fs {rate / 1e6:.3f} MS/s) | "
                f"{n_disp / rate * 1e3:.3f} ms | gain {gain:.1f} dB | "
                f"peak {peak:.2f} FS{'  ** CLIPPING **' if clipped else ''} | "
                f"pk-pk {vpp:.3g} FS | strongest {self._scope_fmt_hz(nu[kpk])} | {trig_text}")
        info = {"text": text, "clip": clipped, "trig": trig_text, "channel": ch,
                "waiting": waiting, "xrange": xrange_s}
        if waiting:
            return None, None, None, info      # status only - the display keeps its last triggered frame
        return t, tr1, tr2, info

    # ---- previous scope_loop(), kept in case of problems ----
#     def scope_loop(self):
#         applied_center = None
#         need_settle = False
#         while not self.stop_flag.is_set():
#             if self.scope_hold:
#                 time.sleep(0.05)
#                 continue
#             try:
#                 if self.sdr is None:
#                     break
#                 # Live sample rate - applied here (worker thread) only, like sweep_loop().
#                 self._restart_pass = False      # set by sample_rate_on_changed(); meaningless here
#                 if getattr(self, "_applied_sample_rate", None) != self.SAMPLE_RATE:
#                     print(f"scope_loop: applying sample-rate change -> {self.SAMPLE_RATE/1e6:.3f} MS/s")
#                     self.sdr.sample_rate = self.SAMPLE_RATE
#                     self._applied_sample_rate = self.SAMPLE_RATE
#                     need_settle = True
#                 self.apply_gain_if_changed()    # live RF Gain slider
#                 rate = float(self._applied_sample_rate)
#                 fc = rate / 4.0                 # tuner parked at Fs/4: RF 0..Fs/2 fills the middle half
#                 if applied_center != fc:
#                     self.sdr.center_freq = fc
#                     applied_center = fc
#                     need_settle = True
#                 if need_settle:
#                     time.sleep(0.12)
#                     _ = self.sdr.read_samples(512)
#                     _ = self.sdr.read_samples(512)
#                     need_settle = False
#                 n_disp = int(round(self.scope_window_s * rate))
#                 n_disp = max(SCOPE_MIN_SAMPLES, ((n_disp + 255) // 256) * 256)
#                 n_total = n_disp if int(self.scope_trig_mode) == 0 else 2 * n_disp   # headroom to find an edge
#                 n_total = min(n_total, SCOPE_MAX_SAMPLES)
#                 n_total = max(256, (n_total // 256) * 256)
#                 n_disp = min(n_disp, n_total)
#                 x = self._scope_read_iq(n_total)
#                 gain = self.actual_gain_db if self.actual_gain_db is not None else self.gain_db
#                 res = self.scope_process_capture(x, rate, n_disp, fc, float(gain))
#                 if res is not None and not self._scope_pending:
#                     self._scope_pending = True
#                     self.scope_signals.frame.emit(*res)
#             except OSError as e:
#                 print(f"scope_loop: read error: {e}")
#                 try:
#                     self.init_sdr()             # reopen, then re-apply the tuning next time round
#                     applied_center = None
#                     need_settle = True
#                 except Exception as e2:
#                     print(f"scope_loop: failed to reopen SDR device: {e2}")
#                     time.sleep(0.5)
#             except Exception as e:
#                 import traceback
#                 print(f"scope_loop: error {e!r}")
#                 traceback.print_exc()
#                 time.sleep(0.5)
#             time.sleep(0.02)

    def scope_loop(self):
        applied_center = None
        need_settle = False
        while not self.stop_flag.is_set():
            if self.scope_hold:
                time.sleep(0.05)
                continue
            try:
                if self.sdr is None:
                    break
                # Live sample rate - applied here (worker thread) only, like sweep_loop().
                self._restart_pass = False      # set by sample_rate_on_changed(); meaningless here
                if getattr(self, "_applied_sample_rate", None) != self.SAMPLE_RATE:
                    print(f"scope_loop: applying sample-rate change -> {self.SAMPLE_RATE/1e6:.3f} MS/s")
                    self.sdr.sample_rate = self.SAMPLE_RATE
                    self._applied_sample_rate = self.SAMPLE_RATE
                    need_settle = True
                self.apply_gain_if_changed()    # live RF Gain slider
                rate = float(self._applied_sample_rate)
                fc = rate / 4.0                 # tuner parked at Fs/4: RF 0..Fs/2 fills the middle half
                if applied_center != fc:
                    self.sdr.center_freq = fc
                    applied_center = fc
                    need_settle = True
                if need_settle:
                    time.sleep(0.12)
                    _ = self.sdr.read_samples(512)
                    _ = self.sdr.read_samples(512)
                    need_settle = False

                # n_disp = samples shown; n_win = samples kept after trimming (display + trigger
                # headroom); n_cap = what is actually captured = n_win + a guard at each end.
                n_disp = int(round(self.scope_window_s * rate))
                n_disp = max(SCOPE_MIN_SAMPLES, ((n_disp + 255) // 256) * 256)
                n_win = n_disp if int(self.scope_trig_mode) == 0 else 2 * n_disp   # headroom to find an edge
                n_cap = min(n_win + 2 * SCOPE_GUARD, SCOPE_MAX_SAMPLES)
                n_cap = max(256 + 2 * SCOPE_GUARD, (n_cap // 256) * 256)
                n_win = n_cap - 2 * SCOPE_GUARD
                n_disp = min(n_disp, n_win)

                # The first part of a read that follows an idle gap can be OLD data left in the
                # device/driver buffers, with a jump where fresh data begins. Read extra and
                # throw that start away so every frame is one continuous stretch of samples.
                raw = self._scope_read_iq(SCOPE_STALE_SKIP + n_cap)
                x = raw[SCOPE_STALE_SKIP:]
                gain = self.actual_gain_db if self.actual_gain_db is not None else self.gain_db
                res = self.scope_process_capture(x, rate, n_disp, fc, float(gain), SCOPE_GUARD)
                if res is not None and not self._scope_pending:
                    self._scope_pending = True
                    self.scope_signals.frame.emit(*res)
            except OSError as e:
                print(f"scope_loop: read error: {e}")
                try:
                    self.init_sdr()             # reopen, then re-apply the tuning next time round
                    applied_center = None
                    need_settle = True
                except Exception as e2:
                    print(f"scope_loop: failed to reopen SDR device: {e2}")
                    time.sleep(0.5)
            except Exception as e:
                import traceback
                print(f"scope_loop: error {e!r}")
                traceback.print_exc()
                time.sleep(0.5)
            time.sleep(0.02)

    # ---- GUI thread: draw a frame ----

    # ---- previous scope_on_frame(), kept in case of problems ----
#     # ---- previous scope_on_frame(), kept in case of problems ----
#     def scope_on_frame(self, t, tr1, tr2, info):
# #         try:
# #             self.scope_curve1.setData(t, tr1)
# #             if tr2 is None:
# #                 self.scope_curve2.hide()
# #             else:
# #                 self.scope_curve2.setData(t, tr2)
# #                 self.scope_curve2.show()
# #             self.scope_status.setText(info.get("text", ""))
# #             self.scope_status.setStyleSheet(
# #                 "QLabel { color: #FF4040; font-weight: bold; }" if info.get("clip") else "")
# #             self._scope_last = (t, tr1, tr2, info)
# #         finally:
# #             self._scope_pending = False
#     def scope_on_frame(self, t, tr1, tr2, info):
#         """
#         Draw a frame (GUI thread). A "waiting" frame (NORMAL sweep, no trigger found) carries no
#         trace: only the status line is updated and the display keeps its last triggered frame.
#         The time axis is held at a fixed span relative to the trigger (t = 0) so the sub-sample
#         alignment of each triggered sweep is actually visible, instead of the view re-fitting to
#         the data every frame; zooming with the mouse stops that until the Window is changed.
#         """
#         try:
#             waiting = bool(info.get("waiting"))
#             if t is not None:
#                 self.scope_curve1.setData(t, tr1)
#                 if tr2 is None:
#                     self.scope_curve2.hide()
#                 else:
#                     self.scope_curve2.setData(t, tr2)
#                     self.scope_curve2.show()
#                 xr = info.get("xrange")
#                 if xr is not None and getattr(self, "_scope_x_follow", True):
#                     try:
#                         self.scope_plot.getViewBox().setXRange(xr[0], xr[1], padding=0)
#                     except Exception:
#                         pass
#                 self._scope_last = (t, tr1, tr2, info)
#             self.scope_status.setText(info.get("text", ""))
#             if info.get("clip"):
#                 self.scope_status.setStyleSheet("QLabel { color: #FF4040; font-weight: bold; }")
#             elif waiting:
#                 self.scope_status.setStyleSheet("QLabel { color: #FFA040; }")
#             else:
#                 self.scope_status.setStyleSheet("")
#         finally:
#             self._scope_pending = False

    def scope_on_frame(self, t, tr1, tr2, info):
        """
        Draw a frame (GUI thread). A "waiting" frame (NORMAL sweep, no trigger found) carries no
        trace: only the status line is updated and the display keeps its last triggered frame.
        The time axis is held at a fixed span relative to the trigger (t = 0) so the sub-sample
        alignment of each triggered sweep is visible; zooming the time axis with the mouse stops
        that until the Window is changed.

        Pulse handling (diagnostic): isolated spikes far outside the trace's own noise are found
        with a robust threshold (SCOPE_PULSE_K x MAD-sigma). "Mark pulses" circles them in red and
        counts them in the readout; "Reject pulsed frames" keeps the last clean frame instead of
        drawing a frame that contains any. Nothing is ever removed from the data that IS drawn.
        """
        try:
            waiting = bool(info.get("waiting"))
            extra = ""
            rejected = False
            if t is not None:
                idx, sigma = [], 0.0
                if self.scope_mark_pulses or self.scope_reject_pulses:
                    idx, sigma = self.scope_find_pulses(tr1)
                n_p = len(idx)
                if self.scope_reject_pulses and n_p > 0:
                    rejected = True
                    self._scope_rejected += 1
                    extra = (f" | REJECTED: {n_p} pulse(s) over {SCOPE_PULSE_K:g} sigma "
                             f"({self._scope_rejected} frame(s) in a row) - showing last clean frame")
                else:
                    self._scope_rejected = 0
                    self.scope_curve1.setData(t, tr1)
                    if tr2 is None:
                        self.scope_curve2.hide()
                    else:
                        self.scope_curve2.setData(t, tr2)
                        self.scope_curve2.show()
                    if self.scope_mark_pulses and n_p:
                        ia = np.asarray(idx, dtype=int)
                        self.scope_pulse_marks.setData(x=np.asarray(t)[ia], y=np.asarray(tr1)[ia])
                        extra = f" | pulses {n_p} (>{SCOPE_PULSE_K:g} sigma, sigma {sigma:.2g} FS)"
                    else:
                        self.scope_pulse_marks.setData(x=[], y=[])
                        if self.scope_mark_pulses:
                            extra = " | pulses 0"
                    xr = info.get("xrange")
                    if xr is not None and getattr(self, "_scope_x_follow", True):
                        try:
                            self.scope_plot.getViewBox().setXRange(xr[0], xr[1], padding=0)
                        except Exception:
                            pass
                    self._scope_last = (t, tr1, tr2, info)
            rate = getattr(self, "_applied_sample_rate", None)
            if rate is not None and float(rate) > SCOPE_FS_DROP_WARN + 1.0:
                extra += f" | Fs above {SCOPE_FS_DROP_WARN / 1e6:.2f} MS/s: USB may drop samples"
            self.scope_status.setText(info.get("text", "") + extra)
            if info.get("clip"):
                self.scope_status.setStyleSheet("QLabel { color: #FF4040; font-weight: bold; }")
            elif waiting or rejected:
                self.scope_status.setStyleSheet("QLabel { color: #FFA040; }")
            else:
                self.scope_status.setStyleSheet("")
        finally:
            self._scope_pending = False

    @staticmethod
    def scope_find_pulses(y):
        """
        Isolated pulses in a trace: samples further than SCOPE_PULSE_K robust sigmas (1.4826 x
        MAD about the median) from the median, grouped into events (samples closer than 16
        apart are one event). Returns (indices of each event's largest sample, sigma). A
        trace dominated by a strong signal has a large sigma, so only pulses much bigger than
        the signal are reported.
        """
        y = np.asarray(y, dtype=np.float64)
        if y.size < 64:
            return [], 0.0
        med = float(np.median(y))
        dev = np.abs(y - med)
        sigma = 1.4826 * float(np.median(dev))
        if sigma <= 0.0:
            return [], 0.0
        hit = np.nonzero(dev > SCOPE_PULSE_K * sigma)[0]
        if hit.size == 0:
            return [], sigma
        groups = np.split(hit, np.nonzero(np.diff(hit) > 16)[0] + 1)
        peaks = [int(g[np.argmax(dev[g])]) for g in groups[:500]]
        return peaks, sigma

    def scope_save_csv(self):
        if self._scope_last is None:
            self.log("Save scope: no frame captured yet")
            return
        path, _ = QtWidgets.QFileDialog.getSaveFileName(
            self, "Save Scope CSV", "scope.csv", "CSV Files (*.csv)")
        if not path:
            return
        t, tr1, tr2, info = self._scope_last
        names = {0: ("signal_fs",), 1: ("envelope_fs",), 2: ("signal_fs", "envelope_fs")}
        cols = [np.asarray(t, dtype=np.float64), np.asarray(tr1, dtype=np.float64)]
        header = ["time_s"] + list(names.get(info.get("channel", self.scope_channel), ("trace1",)))
        if tr2 is not None:
            cols.append(np.asarray(tr2, dtype=np.float64))
        np.savetxt(path, np.column_stack(cols), delimiter=",", header=",".join(header), comments="")
        self.log(f"Saved scope frame ({len(t)} points) to {path}")

    def stop_sweep(self):
        self.log(" STOP SWEEP ")
        self.stop_flag.set()

    # ---- previous single_sweep_loop() / continuous_loop(), kept in case of problems ----
#     def single_sweep_loop(self):
#         # One-shot sweep, but re-run it if a live sample-rate change abandoned the pass.
#         while not self.stop_flag.is_set():
#             self.sweep_loop()
#             if not self._restart_pass:
#                 break
#     def continuous_loop(self):
#         while not self.stop_flag.is_set():
#             try:
#                 self.sweep_loop()   # run one sweep
#             except Exception as e:
#                 # Last-resort safety net: an unhandled exception anywhere in
#                 # sweep_loop() used to kill this whole thread (silently, from the
#                 # GUI's point of view, apart from a console traceback). Log it and
#                 # keep the continuous scan alive instead of letting it die.
#                 import traceback
#                 print(f"continuous_loop: sweep_loop raised {e!r}, continuing")
#                 traceback.print_exc()
#                 time.sleep(0.5)

    def single_sweep_loop(self):
        # One-shot sweep, but re-run it if a live sample-rate change abandoned the pass.
        try:
            self.settle_radio(float(self.centers[0]) if len(self.centers) else float(self.START_FREQ))
        except Exception as e:
            print(f"single_sweep_loop: settle_radio failed: {e!r}")
        while not self.stop_flag.is_set():
            self.sweep_loop()
            if not self._restart_pass:
                break

    # ---- previous continuous_loop() (no single-frequency hook), kept in case of problems ----
#     def continuous_loop(self):
#         try:
#             self.settle_radio(float(self.centers[0]) if len(self.centers) else float(self.START_FREQ))
#         except Exception as e:
#             print(f"continuous_loop: settle_radio failed: {e!r}")
#         while not self.stop_flag.is_set():
#             try:
#                 self.sweep_loop()   # run one sweep
#             except Exception as e:
#                 # Last-resort safety net: an unhandled exception anywhere in
#                 # sweep_loop() used to kill this whole thread (silently, from the
#                 # GUI's point of view, apart from a console traceback). Log it and
#                 # keep the continuous scan alive instead of letting it die.
#                 import traceback
#                 print(f"continuous_loop: sweep_loop raised {e!r}, continuing")
#                 traceback.print_exc()
#                 time.sleep(0.5)

    def continuous_loop(self):
        try:
            self.settle_radio(float(self.centers[0]) if len(self.centers) else float(self.START_FREQ))
        except Exception as e:
            print(f"continuous_loop: settle_radio failed: {e!r}")
        while not self.stop_flag.is_set():
            self.single_apply_pending()     # single-frequency mode: a new centre chosen on the front panel starts here
            try:
                self.sweep_loop()   # run one sweep
            except Exception as e:
                # Last-resort safety net: an unhandled exception anywhere in
                # sweep_loop() used to kill this whole thread (silently, from the
                # GUI's point of view, apart from a console traceback). Log it and
                # keep the continuous scan alive instead of letting it die.
                import traceback
                print(f"continuous_loop: sweep_loop raised {e!r}, continuing")
                traceback.print_exc()
                time.sleep(0.5)

    def check_applied_rate(self, requested):
        """
        Read the sample rate back from the driver and say so if it is not what was asked for. The RTL2832U
        can only make rates whose divider ratio it can represent, so the hardware rate differs slightly from
        the request (by well under 1 ppm for every recommended rate); a bigger difference would mean the
        request was not honoured. Runs on the worker thread (the driver is not safe from two threads) and
        logs once per rate (again if the hardware later reports a different value for that rate).
        """
        done = getattr(self, "_rate_checked", None)
        if done is None:
            done = self._rate_checked = {}
        if self.sdr is None:
            return None
        actual = None
        try:
            actual = float(self.sdr.sample_rate)
        except Exception:
            try:
                actual = float(self.sdr.get_sample_rate())
            except Exception:
                pass
        if actual is None or actual <= 0.0:
            return None
        self.actual_sample_rate = actual
        err_ppm = (actual - requested) / requested * 1e6
        key = round(requested)
        if key not in done or abs(done[key] - actual) > 1.0:        # first time for this rate, or the hardware now reports something else
            done[key] = actual
            txt = (f"Sample rate: requested {requested / 1e6:.6f} MS/s, hardware reports {actual / 1e6:.6f} MS/s "
                   f"({err_ppm:+.2f} ppm)")
            if abs(err_ppm) > 20.0:
                txt = "WARNING: " + txt + " - the hardware is NOT running at the requested rate"
            self.log(txt)
        return actual

    def settle_radio(self, first_centre_hz):
        """
        Runs ONCE, on the worker thread, when a spectrum sweep or the scope starts (never from the
        GUI thread - the driver is not safe from two threads).

        init_sdr() sets sample rate, AGC and gain while the tuner has not yet been tuned or streamed.
        get_gain() only reads back the value we asked for, so the program cannot see whether the
        hardware really took it, and in practice the gain only became effective once the slider
        had been moved after the sweep was running. This does that move for you: tune to the first
        centre, start the stream, step the gain to the neighbouring setting and back to the wanted
        one (AGC re-asserted off), then discard a couple of reads.
        """
        if self.sdr is None:
            return
        want = float(self.gain_db)
        self.check_applied_rate(float(self.SAMPLE_RATE))
        self.sdr.center_freq = float(first_centre_hz)
        time.sleep(0.12)
        _ = self.sdr.read_samples(512)
        try:
            self.sdr.set_agc_mode(False)
        except Exception as e:
            print("settle_radio: could not re-assert manual gain mode:", e)
        steps = RTL_GAIN_STEPS_DB
        idx = min(range(len(steps)), key=lambda i: abs(steps[i] - want))
        alt = steps[idx + 1] if idx + 1 < len(steps) else steps[idx - 1]
        self.sdr.gain = alt
        time.sleep(0.05)
        _ = self.sdr.read_samples(512)
        self.sdr.gain = want
        self._applied_gain_db = want
        self.actual_gain_db = float(self.sdr.get_gain())
        self.actual_gain_db_for_request = want
        time.sleep(0.05)
        _ = self.sdr.read_samples(512)
        _ = self.sdr.read_samples(512)
        print(f"settle_radio: gain re-applied after tuning ({want} dB requested -> {self.actual_gain_db} dB)")
        self.update_status_panel()

    def continuous_when_idle(self, tries=100):
        """
        Start CONTINUOUS as soon as no sweep / scope thread is running. Called from the GUI thread
        after a preset or a scope -> spectrum switch has asked the old thread to stop: polls every
        100 ms (so a sweep that is slow to notice STOP - e.g. long averaging - cannot make the
        start silently do nothing).
        """
        if self._survey is not None:
            return
        if self.sweep_thread and self.sweep_thread.is_alive():
            if tries <= 0:
                self.log("Could not start continuous mode: the previous sweep did not stop")
                return
            self.stop_flag.set()
            QtCore.QTimer.singleShot(100, lambda: self.continuous_when_idle(tries - 1))
            return
        self.start_continuous()

    # ---- previous set_preset_and_run(), kept in case of problems ----
#     def set_preset_and_run(self, preset_index):
#         """MODE button: apply the preset, then immediately (re)start CONTINUOUS mode."""
#         self.set_preset(preset_index)
#         self.continuous_when_idle()

    def set_preset_and_run(self, preset_index):
        """MODE button: leave single-frequency mode, apply the preset, then immediately (re)start CONTINUOUS mode."""
        self.single_exit()
        self._last_preset_index = preset_index
        self.set_preset(preset_index)
        self.continuous_when_idle()

    def update_plot(self):

        # Basic guards
        if getattr(self, "freq_axis", None) is None or getattr(self, "power_axis", None) is None:
            return

        # Sort and copy arrays
        try:
            idx = np.argsort(self.freq_axis)
            f = self.freq_axis[idx].copy()
            p = self.power_axis[idx].copy()
        except Exception:
            return

        # Remove NaNs and ensure we have data to plot
        valid = ~np.isnan(f)
        if not np.any(valid):
            return
        f = f[valid]
        p = p[valid]

        # --- IMPORTANT: do NOT overwrite stitched arrays used by the sweep ---
        # Keep a plotting copy instead so the sweep's stitched storage remains intact.
        self._plot_freq_axis = f
        self._plot_power_axis = p
        # ---------------------------------------------------------------------

        # Emit the plot update using the plotting copy. update_plot() runs on the
        # worker (sweep) thread, so it must not touch Qt scene items directly -
        # that includes the CISPR line. The queued signal below hands the data to
        # set_plot_data() on the GUI thread, which draws the FFT curve and then
        # calls cispr_update_curve() itself, so the CISPR line is always redrawn
        # right after (never before, and never concurrently with) each FFT update.
        try:
            self.plot_signals.update.emit(self._plot_freq_axis, self._plot_power_axis)
        except Exception as e:
            print("update_plot: emit error:", e)


    @QtCore.pyqtSlot(object, object)
    def set_plot_data(self, f, p):
        """
        Runs on the GUI thread (queued slot from plot_signals.update). Draws the
        latest stitched FFT data, then draws the CISPR line immediately after -
        this is the ONLY place the CISPR line is redrawn as part of a normal
        sweep, so it is always in step with the FFT curve it sits alongside.
        (Previously this method also built its own interpolated CISPR curve with
        a hard-coded "L_interp - 80.0" calibration, in parallel with
        cispr_update_curve()'s offset/ViewBox-range approach. The two disagreed,
        so whichever one last executed - the timing depended on the worker
        thread's call to update_cispr_curve() racing this queued slot - won,
        which is why the line and its axis appeared to move relative to each
        other unpredictably. There is now a single source of truth.)
        """
        self.curve.setData(f, p)
        self.cispr_update_curve()

    def save_csv(self):
        if self.freq_axis is None or self.power_axis is None:
            self.log("Save FFT: no live data to save yet")
            return

        path, _ = QtWidgets.QFileDialog.getSaveFileName(
            self, "Save Spectrum CSV", "spectrum.csv", "CSV Files (*.csv)"
        )
        if not path:
            return

        idx = np.argsort(self.freq_axis)
        f = self.freq_axis[idx]
        p = self.power_axis[idx]

        have_ref = (getattr(self, "reference_freq", None) is not None
                    and getattr(self, "reference_power", None) is not None
                    and len(self.reference_freq) > 0)

        if not have_ref:
            # No reference captured - unchanged behaviour from before.
            data = np.column_stack((f, p))
            np.savetxt(path, data, delimiter=",", header="freq_hz,level_db", comments="")
            self.log(f"Saved live trace ({len(f)} points) to {path}")
            return

        # A captured reference trace may have been taken under different sweep settings
        # (different MODE, sample rate, etc.) than the current live trace, so it can have
        # a different length and a different frequency grid. Rather than assume the two
        # line up row-for-row, write them as two independent (freq, level) column pairs;
        # whichever trace is shorter just leaves its trailing cells blank.
        rf = np.asarray(self.reference_freq, dtype=np.float64)
        rp = np.asarray(self.reference_power, dtype=np.float64)
        n = max(len(f), len(rf))
        with open(path, "w", newline="") as fh:
            fh.write("freq_hz,level_db,ref_freq_hz,ref_level_db\n")
            for i in range(n):
                a = f"{f[i]:.6f}" if i < len(f) else ""
                b = f"{p[i]:.3f}" if i < len(p) else ""
                c = f"{rf[i]:.6f}" if i < len(rf) else ""
                d = f"{rp[i]:.3f}" if i < len(rp) else ""
                fh.write(f"{a},{b},{c},{d}\n")
        self.log(f"Saved live ({len(f)} pts) + reference ({len(rf)} pts) trace to {path}")

    def closeEvent(self, event):
        self.stop_flag.set()
        if self.sweep_thread and self.sweep_thread.is_alive():
            self.sweep_thread.join(timeout=1.0)
        if self.sdr is not None:
            try:
                self.sdr.close()
            except Exception:
                pass
        event.accept()


# ---- previous main(), kept in case of problems ----
# def main():
#     app = QtWidgets.QApplication(sys.argv)
#     win = EmcScanner()
#     win.resize(1200, 600)
#     win.show()
#     sys.exit(app.exec_())

def main():
    app = QtWidgets.QApplication(sys.argv)
    # Always use '.' as the decimal point in every spin box, whatever the PC's regional
    # settings are (a French/European locale otherwise shows e.g. 98,500). Set BEFORE any
    # widget is created.
    QtCore.QLocale.setDefault(QtCore.QLocale(QtCore.QLocale.English, QtCore.QLocale.UnitedKingdom))
    win = EmcScanner()
    win.resize(1200, 600)
    win.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()