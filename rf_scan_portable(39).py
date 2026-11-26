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

#--------------------------------------------------------------
# Notes for calibration. 
# With a 5W RF transmitter at 5m  with 30db attenuator fitted between TX and antenna, 
# a signal strength of aproximately 87dBuv is expected.
# when my atenuator arrives I hope to be able to properly link the CISPR and FFT scales. 
#
# ---- THE ONE dBuV CALIBRATION CONSTANT ----
# Displayed level (dBuV at the INPUT of the external attenuator) =
#       SDR dB  -  RF gain (dB)  +  attenuator (dB)  +  CAL_SDR_DB_TO_DBUV
# To recalibrate: feed a known level, note the displayed peak, then
#       CAL_SDR_DB_TO_DBUV = CAL_SDR_DB_TO_DBUV + (true dBuV - displayed dBuV)
CAL_SDR_DB_TO_DBUV = 107

#--------------------------------------------------------------

from PyQt5 import QtWidgets, QtCore, QtGui
import pyqtgraph as pg

init_done = False
ENABLE_DRIVER_SELECTION=False
# # ---- SWEEP PROFILE ( variables) ----
START_FREQ = 95e6       # 150 kHz
STOP_FREQ  = 102e6        # 30 MHz
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
# (internal_name, label, start_hz, stop_hz, sample_rate, gain_db, hf_mode_flag)
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


# Program settings read from settings.csv rows of type "setting" (key in the name column, value in the
# label column - or a "value" column if you add one). Filled in by load_settings().
SETTINGS_EXTRA = {}
_KNOWN_SETTINGS = ("dc_offset_removal", "iq_balance_correction", "driver", "driver_root", "driver_dialog",
                   "saturation_line", "saturation_margin_db", "waterfall", "waterfall_rows", "y_span_above_floor_db", "display_dbfs")   # (plus cal_<sample rate Hz> rows)


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
                    if key not in _KNOWN_SETTINGS and not key.startswith("cal_"):   # cal_<rate> rows are the per-rate dB -> dBuV factors
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

# ---- Per-sample-rate dB -> dBuV calibration (written by the CAL button, read here) ----
# settings.csv rows   setting,cal_<sample rate in Hz>,<factor>    e.g.   setting,cal_2400000,108.37
# A sample rate with no row of its own uses the default CAL_SDR_DB_TO_DBUV (top of the file).
CAL_BY_RATE = {}
for _k, _v in SETTINGS_EXTRA.items():
    if _k.startswith("cal_"):
        try:
            CAL_BY_RATE[float(_k[4:])] = float(_v)
        except ValueError:
            print(f"settings: warning - calibration setting {_k} = {_v!r} is not valid and is ignored")
if CAL_BY_RATE:
    print("settings: per-sample-rate dB->dBuV factors: " + ", ".join(f"{r/1e6:g} MS/s = {v:.2f}" for r, v in sorted(CAL_BY_RATE.items())))

# ADC saturation line on the display, off unless asked for (settings.csv:  setting,saturation_line,on|off   setting,saturation_margin_db,10)
SATURATION_LINE = _setting_bool("saturation_line", False)
try:
    SATURATION_MARGIN_DB = float(SETTINGS_EXTRA.get("saturation_margin_db") or 10.0)
except ValueError:
    print(f"settings: warning - saturation_margin_db = {SETTINGS_EXTRA.get('saturation_margin_db')!r} is not a number - using 10")
    SATURATION_MARGIN_DB = 10.0


def cal_for_rate(rate):
    """The dB -> dBuV factor for this sample rate (Hz): its own value from settings.csv / the CAL button, else the default."""
    for r, v in CAL_BY_RATE.items():
        if abs(r - float(rate)) < 1.0:
            return v
    return CAL_SDR_DB_TO_DBUV


def cal_has_rate(rate):
    return any(abs(r - float(rate)) < 1.0 for r in CAL_BY_RATE)


def cal_reference_db():
    """Factor used for the saturation line: that of the most sensitive calibrated rate (the smallest), else the default."""
    return min(CAL_BY_RATE.values()) if CAL_BY_RATE else CAL_SDR_DB_TO_DBUV


# =====================================================================================================================
# HOW THE dB -> dBuV CALIBRATION FACTORS ARE USED - everything in one place
# =====================================================================================================================
# "SDR dB" is the level the program stores for every FFT bin: 10*log10(|FFT/N|^2) + 33.5 of the samples as read, with NO gain
# compensation. From it the DISPLAYED level is built in these steps, and every place that does so goes through these functions:
#
#   1. factor            cal_for_rate(sample_rate): the factor of this sample rate - its own cal_<Hz> row in settings.csv (written by
#                        the CAL button) or, if it has none, the default CAL_SDR_DB_TO_DBUV at the top of the file.
#   2. THE FORMULA       cal_model_offset_db(factor, RF gain, attenuator):
#                            dBuV at the INPUT of the external attenuator  =  SDR dB  -  RF gain  +  attenuator  +  factor
#                        (RF gain: the gain applied to the pass; attenuator: the front-panel Attenuator box = the external attenuator.)
#   3. correction        + the Antenna factor (Radiated) / LISN correction (Conducted) box (cispr_offset_db). NOT part of the CAL.
#   4. dBFS option       bypasses steps 1-3: displayed = SDR dB relative to ADC full scale, no calibration of any kind.
#
# Where they are used:
#   display (trace, waterfall rows, initial graph view, CSV):  EmcScanner.display_offset_for()  (steps 1-4);  cal_total_offset_db() calls it
#   ADC saturation lines:        EmcScanner.sat_full_scale_display()   (steps 2-3 with the most sensitive rate's factor, cal_reference_db())
#   CAL button / CAL SURVEY:     cal_measure() converts a measured tone with step 2;  cal_quick_evaluate() and the survey's "implied"
#                                column solve step 2 backwards with cal_solve_factor() to find the factor that reads the tone correctly
#   the CAL's own test level:    cal_source_dbuv(): generator amplitude -> series (dropper) resistor -> attenuator input impedance
#   reading / writing:           CAL_BY_RATE above (read at start-up);  settings_write_cal() below (written only if the user agrees)
# The CAL never includes step 3 (so the factors never depend on the antenna factor) and is not affected by the dBFS option.


def cal_model_offset_db(factor, rf_gain_db, attenuator_db):
    """Step 2 of the chain above: dB to ADD to an SDR dB value to get dBuV at the INPUT of the external attenuator."""
    return float(factor) - float(rf_gain_db) + float(attenuator_db)


def cal_solve_factor(true_dbuv, tone_sdr_db, rf_gain_db, attenuator_db):
    """Step 2 solved backwards: the factor for which cal_model_offset_db() makes a tone of SDR level tone_sdr_db read true_dbuv
    (true_dbuv = its real level at the attenuator input)."""
    return float(true_dbuv) - float(tone_sdr_db) + float(rf_gain_db) - float(attenuator_db)


def cal_source_dbuv(vpp_generator, r_series_ohm=0.0, z_in_ohm=50.0):
    """Level of the CAL test signal at the INPUT of the external attenuator. vpp_generator is the generator amplitude (V pk-pk) as
    measured at the generator output; it drives r_series_ohm (the series 'dropper' resistor) into z_in_ohm (the 50 ohm input of the
    attenuator / SDR), so  V_in = V_generator * z_in / (r_series + z_in).
    Returns (dBuV, V_in pk-pk, V_in rms). With r_series = 0 the generator drives the attenuator directly."""
    z = max(float(z_in_ohm), 1e-6)
    r = max(float(r_series_ohm), 0.0)
    vin_pp = float(vpp_generator) * z / (r + z)
    vrms = vin_pp / (2.0 * math.sqrt(2.0))
    return 20.0 * math.log10(max(vrms, 1e-15) / 1e-6), vin_pp, vrms


def settings_write_cal(cal_by_rate, gain_db=None, tone_hz=None):
    """Write the per-rate factors into settings.csv as 'setting,cal_<Hz>,<value>' rows. Every other line (presets, other
    settings, comments) is kept exactly; old cal_ rows are replaced; the old file is first copied to settings.csv.bak.
    The column the value goes in follows the file's own header ('value' if there is one, else 'label'). The gain the CAL was made at is recorded in the comment line. Returns the path."""
    import shutil
    if SETTINGS_PATH:
        path = SETTINGS_PATH
    else:
        here = os.path.dirname(sys.executable) if getattr(sys, "frozen", False) else os.path.dirname(os.path.abspath(__file__))
        path = os.path.join(here, SETTINGS_FILENAME)
    text = ""
    if os.path.isfile(path):
        with open(path, "r", encoding="utf-8-sig", newline="") as fh:
            text = fh.read()
        shutil.copy2(path, path + ".bak")
    nl = "\r\n" if ("\r\n" in text or "\n" not in text) else "\n"
    fields = None
    keep = []
    for ln in text.splitlines():
        s = ln.strip()
        if s and not s.startswith("#"):
            row = next(csv.reader([ln], skipinitialspace=True), [])
            if fields is None:                                   # the header line
                fields = [x.strip().lower() for x in row]
            elif len(row) >= 2 and row[0].strip().lower() == "setting" and row[1].strip().lower().startswith("cal_"):
                continue                                         # an old per-rate calibration row: replaced below
        keep.append(ln)
    if fields is None:
        fields = ["type", "name", "label", "start", "stop", "sample_rate", "gain_db", "hf_mode", "dbuv_cal"]
        keep.insert(0, ",".join(fields))
    i_type = fields.index("type") if "type" in fields else 0
    i_name = fields.index("name") if "name" in fields else 1
    i_val = fields.index("value") if "value" in fields else (fields.index("label") if "label" in fields else 2)

    def make_row(name, val):
        row = [""] * max(len(fields), i_val + 1, i_name + 1, i_type + 1)
        row[i_type], row[i_name], row[i_val] = "setting", name, val
        return ",".join(row)

    keep.append("# dB -> dBuV factor per sample rate (written by the CAL button " + time.strftime("%Y-%m-%d %H:%M") +
                ("" if gain_db is None else f", made at RF gain {gain_db:g} dB") +
                ("" if tone_hz is None else f" with a {tone_hz/1e6:g} MHz tone") + ")")
    for rate, val in sorted(cal_by_rate.items()):
        keep.append(make_row(f"cal_{int(round(rate))}", f"{val:.2f}"))
    with open(path, "w", encoding="utf-8", newline="") as fh:
        fh.write(nl.join(keep) + nl)
    return path


# Waterfall display (settings.csv:  setting,waterfall,on|off   setting,waterfall_rows,150)
WATERFALL_ON = _setting_bool("waterfall", False)
try:
    WATERFALL_ROWS = max(20, min(2000, int(float(SETTINGS_EXTRA.get("waterfall_rows") or 150))))
except ValueError:
    print(f"settings: warning - waterfall_rows = {SETTINGS_EXTRA.get('waterfall_rows')!r} is not a number - using 150")
    WATERFALL_ROWS = 150


# Initial Y range of the graph when a mode is selected: from just under the lowest level measured up to this many dB above it
# (settings.csv:  setting,y_span_above_floor_db,60)
Y_MARGIN_BELOW_FLOOR_DB = 3.0
try:
    Y_SPAN_ABOVE_FLOOR_DB = max(3.0, float(SETTINGS_EXTRA.get("y_span_above_floor_db") or 60.0))
except ValueError:
    print(f"settings: warning - y_span_above_floor_db = {SETTINGS_EXTRA.get('y_span_above_floor_db')!r} is not a number - using 60")
    Y_SPAN_ABOVE_FLOOR_DB = 60.0

# Display units at start-up (settings.csv:  setting,display_dbfs,on|off). dBFS = level relative to ADC full scale at the SDR output,
# with NO calibration: the dB -> dBuV factors, RF gain and attenuator are all ignored. Default off (dBuV at the attenuator input).
DISPLAY_DBFS = _setting_bool("display_dbfs", False)
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


# =====================================================================================================================
# DECODER (single-frequency mode): audio demodulators (WFM / NFM / AM) and an OOK / FSK burst decoder.
# Pure numpy - no scipy, no extra installs. Audio goes to the PC speaker through PyQt5's own QtMultimedia.
# The DSP below has no Qt in it; the front-panel / thread / audio-output side is in EmcScanner (see "DECODER" section).
# =====================================================================================================================
try:
    from PyQt5 import QtMultimedia
except Exception as _dec_err:          # a stripped-down PyQt5 install: audio modes are then disabled, burst decoding still works
    QtMultimedia = None
    print(f"decoder: PyQt5.QtMultimedia not available ({_dec_err}) - audio modes disabled")

DECODER_BLOCK = 16 * 4096              # IQ samples per read (about 27 ms at 2.4 MS/s); one spectrum / waterfall row per 4096
DECODER_UI_INTERVAL_S = 0.12           # spectrum / waterfall / status refresh interval while decoding
DECODER_OFFSET_DIVISOR = 8.0           # offset tuning: the SDR is tuned Fs/8 above the wanted frequency and mixed back in software,
                                       # so the tuner's own DC spike never lands on the signal being decoded

# ---------------------------------------------------------------- filters
def _fir_lowpass(cutoff_hz, rate, ntaps=101):
    """Windowed-sinc low-pass FIR (Hamming), unity DC gain."""
    n = np.arange(ntaps) - (ntaps - 1) / 2.0
    fc = cutoff_hz / rate
    h = 2 * fc * np.sinc(2 * fc * n) * np.hamming(ntaps)
    return (h / h.sum()).astype(np.float32)


class Decimator:
    """Streaming FIR low-pass + integer decimation for complex or real data (keeps filter state between blocks)."""
    def __init__(self, rate_in, factor, cutoff_hz, ntaps=None):
        self.factor = max(1, int(factor))
        if ntaps is None:
            ntaps = int(max(31, min(401, 4.0 * rate_in / max(cutoff_hz, 1.0)))) | 1
        self.h = _fir_lowpass(cutoff_hz, rate_in, ntaps)
        self.tail = None
        self.phase = 0

    def process(self, x):
        if self.tail is None:
            self.tail = np.zeros(self.h.size - 1, dtype=x.dtype)
        buf = np.concatenate([self.tail, x])
        self.tail = buf[-(self.h.size - 1):].copy()
        y = np.convolve(buf, self.h, mode="valid")          # len == len(x)
        out = y[self.phase::self.factor]
        self.phase = (self.phase - y.size) % self.factor
        return out


# ---------------------------------------------------------------- audio demodulator
class AudioDemod:
    """IQ -> mono audio at AUDIO_RATE.  mode: 'WFM', 'NFM', 'AM'.
    Stage 1: IQ rate -> ~240 kHz IF (coarse channel filter).  NFM / AM add stage 2: IF -> ~48 kHz with a sharp channel filter.
    Then demodulate, audio low-pass, resample to exactly 48 kHz (linear interpolation) and de-emphasise (FM only)."""
    AUDIO_RATE = 48000
    # mode: (stage-1 cutoff Hz, stage-2 cutoff Hz or None, de-emphasis tau s or None, audio low-pass Hz, deviation Hz)
    MODES = {"WFM": (110e3, None, 50e-6, 15e3, 75e3),      # tau 75e-6 for the Americas
             "NFM": (30e3, 7e3, None, 3.4e3, 5e3),
             "AM":  (30e3, 6e3, None, 5e3, 0.0)}
    IF_TARGET = 240e3

    def __init__(self, rate_in, mode, deemph_tau=None):
        self.mode = mode
        c1, c2, tau, alp, self.dev = self.MODES[mode]
        if deemph_tau is not None and tau is not None:
            tau = deemph_tau
        self.rate_in = float(rate_in)
        f1 = max(1, int(self.rate_in // self.IF_TARGET))
        rate1 = self.rate_in / f1
        self.dec1 = Decimator(self.rate_in, f1, min(c1, rate1 * 0.45))
        if c2 is None:
            self.dec2, self.demod_rate = None, rate1
        else:
            f2 = max(1, int(rate1 // self.AUDIO_RATE))
            self.dec2 = Decimator(rate1, f2, c2, ntaps=241)
            self.demod_rate = rate1 / f2
        self.alp = Decimator(self.demod_rate, 1, min(alp, self.demod_rate * 0.45), ntaps=63)
        self.ratio = self.demod_rate / self.AUDIO_RATE
        self.pos = 0.0
        self.hist = np.zeros(1, dtype=np.float32)
        self.prev = np.complex64(1.0)
        self.dc = 0.0
        self.deemph_a = (1.0 - math.exp(-1.0 / (tau * self.AUDIO_RATE))) if tau else None
        self.de_state = 0.0

    def _resample(self, y):
        x = np.concatenate([self.hist, y.astype(np.float32)])
        n = x.size - 1
        if n < 1:
            return np.zeros(0, np.float32)
        idx = np.arange(self.pos, n, self.ratio)
        i0 = idx.astype(np.int64)
        fr = (idx - i0).astype(np.float32)
        out = x[i0] * (1 - fr) + x[np.minimum(i0 + 1, n)] * fr
        self.pos = (idx[-1] + self.ratio - n) if idx.size else self.pos - n
        self.hist = x[-1:].copy()
        return out

    def _deemph(self, x):
        """Single-pole de-emphasis as a streaming 64-tap FIR (the truncated impulse response; error < 1e-11). Replaces a
        48000-iterations-per-second Python loop."""
        a = self.deemph_a
        h = getattr(self, "_de_h", None)
        if h is None:
            h = self._de_h = a * (1.0 - a) ** np.arange(64)
            self._de_tail = np.zeros(63)
        buf = np.concatenate([self._de_tail, x.astype(np.float64)])
        self._de_tail = buf[-63:].copy()
        return np.convolve(buf, h, mode="valid").astype(np.float32)

    def process(self, iq):
        """iq: complex64 at rate_in.  Returns (audio float32 at 48 kHz, about -1..1;  channel level, linear, 1.0 = ADC full scale)."""
        z = self.dec1.process(iq)
        if self.dec2 is not None:
            z = self.dec2.process(z)
        if z.size < 2:
            return np.zeros(0, np.float32), 0.0
        level = float(np.abs(z).mean())
        if self.mode == "AM":
            env = np.abs(z)
            m = float(env.mean())
            self.dc = m if self.dc == 0.0 else 0.995 * self.dc + 0.005 * m     # slow carrier tracker = AGC
            a = env / max(self.dc, 1e-6) - 1.0                                  # carrier-normalised, DC removed
        else:
            zz = np.concatenate([[self.prev], z])
            d = np.angle(zz[1:] * np.conj(zz[:-1]))                              # instantaneous frequency, radians/sample
            self.prev = z[-1]
            a = d * (self.demod_rate / (2 * math.pi * self.dev))                 # +-1.0 at nominal deviation
        a = self.alp.process(a.astype(np.float32))
        out = self._resample(a)
        if self.deemph_a is not None and out.size:
            out = self._deemph(out)
        return out.astype(np.float32), level
        # ---- version 1 (per-sample Python loop), kept in case of problems ----
        #OLD#         if self.deemph_a is not None and out.size:
        #OLD#             al = self.deemph_a
        #OLD#             y = np.empty_like(out)
        #OLD#             s = self.de_state
        #OLD#             for i in range(out.size):
        #OLD#                 s += al * (out[i] - s)
        #OLD#                 y[i] = s
        #OLD#             self.de_state = s
        #OLD#             out = y
        #OLD#         return out.astype(np.float32), level


# ---------------------------------------------------------------- burst decoder
def _runs(bits):
    """Run-length encode a 0/1 array -> (values, lengths)."""
    if bits.size == 0:
        return np.zeros(0, np.int8), np.zeros(0, np.int64)
    ch = np.flatnonzero(np.diff(bits)) + 1
    starts = np.concatenate([[0], ch])
    ends = np.concatenate([ch, [bits.size]])
    return bits[starts], ends - starts


def _to_hex(bits):
    b = list(bits)
    while len(b) % 8:
        b.append(0)
    return "".join(f"{int(''.join(str(int(x)) for x in b[i:i+8]), 2):02X}" for i in range(0, len(b), 8))


def _cluster_widths(w, tol=0.30):
    """Group pulse widths into clusters of similar value -> sorted list of cluster centres."""
    cl = []
    for x in sorted(w):
        if cl and abs(x - cl[-1][-1]) <= tol * cl[-1][-1]:
            cl[-1].append(x)
        else:
            cl.append([x])
    return [float(np.mean(c)) for c in cl if len(c) >= 1]


def analyse_bits(sl, rate):
    """sl: 0/1 array of the demodulated/sliced burst at sample rate `rate`.
    Returns dict: baud, coding, bits (list), hex, notes.  Tries (a) NRZ at constant bit period,
    (b) Manchester, (c) pulse-width / pulse-position (PWM/PPM) coding as used by most 433 MHz remotes."""
    vals, lens = _runs(sl.astype(np.int8))
    if vals.size < 4:
        return None
    # drop a leading/trailing partial run of zeros (idle) - keep from the first 1 to the last 1
    first = int(np.argmax(vals == 1)) if (vals == 1).any() else 0
    last = vals.size - 1 - int(np.argmax(vals[::-1] == 1))
    vals, lens = vals[first:last + 1], lens[first:last + 1]
    if vals.size < 3:
        return None
    L = lens.astype(np.float64)
    centres = _cluster_widths(L, 0.30)
    res = {"runs": int(vals.size), "centres_us": [c / rate * 1e6 for c in centres]}
    # candidate unit = smallest cluster centre (shortest run = one bit period in NRZ/Manchester half-period)
    unit = centres[0]
    # ---- (a) Manchester / NRZ : all run lengths ~ integer multiples of unit
    mult = L / unit
    frac_err = np.abs(mult - np.round(mult))
    integer_like = float(np.mean(frac_err < 0.25)) > 0.9 and mult.max() < 40
    # ---- (b) PWM/PPM : exactly 2 (or 3) distinct widths among the HIGH pulses (or gaps)
    hi_w = L[vals == 1]
    lo_w = L[vals == 0]
    hi_c = _cluster_widths(hi_w, 0.30) if hi_w.size else []
    lo_c = _cluster_widths(lo_w, 0.30) if lo_w.size else []
    bits = []
    coding, baud, notes = "unknown", None, ""
    # (hi, lo) pairs: every HIGH run followed by its LOW run (the final pulse has no trailing gap - it was trimmed)
    pairs = [(L[i], L[i + 1]) for i in range(vals.size - 1) if vals[i] == 1 and vals[i + 1] == 0]
    psum = np.array([a + b for a, b in pairs]) if pairs else np.zeros(0)
    const_period = psum.size >= 6 and float(psum.std() / psum.mean()) < 0.12     # every bit takes the same time
    # PWM: HIGH width carries the bit (2 clusters, ratio ~2-4) and either the gap is constant or the bit period is constant.
    # (Manchester also has 2 pulse widths, but neither the gap nor the period is constant - so it falls through to the next test.)
    if (len(hi_c) == 2 and 1.6 < hi_c[1] / hi_c[0] < 4.5 and hi_w.size >= 8
            and (len(lo_c) <= 1 or const_period)):
        thr = math.sqrt(hi_c[0] * hi_c[1])
        bits = [1 if L[i] > thr else 0 for i in range(vals.size) if vals[i] == 1]
        bit_period = float(psum.mean()) if psum.size else unit
        baud = rate / bit_period
        coding = "PWM (long pulse=1, short=0)" + (" fixed bit period" if const_period else " fixed gap")
        notes = f"pulse widths {hi_c[0]/rate*1e6:.0f} / {hi_c[1]/rate*1e6:.0f} us"
    elif (len(lo_c) == 2 and 1.6 < lo_c[1] / lo_c[0] < 4.5 and len(hi_c) <= 1 and lo_w.size >= 8):
        thr = math.sqrt(lo_c[0] * lo_c[1])
        bits = [1 if L[i] > thr else 0 for i in range(vals.size - 1) if vals[i] == 0]
        coding = "PPM / gap-width (long gap=1, short=0)"
        baud = rate / (np.mean(hi_c) + np.mean(lo_c)) if hi_c else None
        notes = f"gap widths {lo_c[0]/rate*1e6:.0f} / {lo_c[1]/rate*1e6:.0f} us"
    elif integer_like:
        # unit estimate refinement: least-squares over all runs
        unit = float(np.sum(L * np.round(mult)) / np.sum(np.round(mult) ** 2))
        n = np.maximum(1, np.round(L / unit)).astype(int)
        raw = np.repeat(vals, n)
        has_long = bool((n >= 2).any())
        # Manchester check: no run longer than 2 half-bits
        if n.max() <= 2 and raw.size >= 8:
            # Manchester half-bit rate = 1/unit; bit rate = half of that.  Two phase choices; pick the one with fewer violations.
            def man(off):
                pairs = raw[off:off + (raw.size - off) // 2 * 2].reshape(-1, 2)
                bad = int(np.sum(pairs[:, 0] == pairs[:, 1]))
                return bad, (pairs[:, 0] == 0).astype(int) if pairs.size else np.zeros(0, int), pairs.shape[0]
            b0, bits0, n0 = man(0)
            b1, bits1, n1 = man(1)
            ok0 = b0 <= 0.1 * max(n0, 1)
            ok1 = b1 <= 0.1 * max(n1, 1)
            if ok0 or ok1:
                use1 = (ok1 and not ok0) or (ok0 and ok1 and b1 < b0)
                bits = list(bits1 if use1 else bits0)
                coding = "Manchester (01=1, 10=0 - invert all bits if your protocol is the other way round)"
                baud = rate / (2 * unit)
                notes = f"half-bit {unit/rate*1e6:.0f} us; a first bit that starts LOW cannot be told from idle, so it may be missing"
                if ok0 and ok1:
                    alt = bits0 if use1 else bits1
                    res["hex_alt"] = _to_hex(list(alt))
                    notes += " (other half-bit alignment also valid - see alt)"
        if not bits:
            bits = list(raw[::1][::1]) if unit < 1.5 else list(np.repeat(vals, n))
            coding = "NRZ / plain OOK at a fixed bit period"
            baud = rate / unit
            notes = f"bit period {unit/rate*1e6:.0f} us"
    else:
        notes = "irregular widths - could be analogue/noise or a coding not recognised; raw run widths listed"
    res.update({"baud": baud, "coding": coding, "bits": bits, "hex": _to_hex(bits) if bits else "", "notes": notes,
                "nbits": len(bits)})
    return res


# ---- version 2 of the burst decoder (frame splitting, hysteresis slicer, voting across frames, locked timing). The version 1 class is kept,
# ---- commented out, at the end of this block in case of problems.
def _clusters_counts(w, tol=0.30):
    """[(centre, count), ...] groups of similar widths, sorted by centre."""
    cl = []
    for x in sorted(w):
        if cl and abs(x - cl[-1][-1]) <= tol * cl[-1][-1]:
            cl[-1].append(x)
        else:
            cl.append([x])
    return [(float(np.mean(c)), len(c)) for c in cl]


def _deglitch(vals, lens, min_len):
    """Merge runs shorter than min_len into their neighbours (edge chatter / noise dips on a weak signal)."""
    v = [int(x) for x in vals]
    l = [int(x) for x in lens]
    i = 0
    while i < len(v):
        if l[i] < min_len and len(v) > 1:
            if i == 0:
                del v[0]
                del l[0]
                continue
            if i == len(v) - 1:
                del v[i]
                del l[i]
                break
            l[i - 1] += l[i] + l[i + 1]
            del v[i:i + 2]
            del l[i:i + 2]
            i = max(i - 1, 0)
            continue
        i += 1
    return np.array(v, np.int8), np.array(l, np.int64)


def _ook_slice(x, rate):
    """Complex burst -> 0/1 array. 40 us envelope smoothing, then a Schmitt trigger whose switching level FOLLOWS the local signal
    strength (running maximum over +-3 ms, never lower than 25 % of the transmission's span), so pulses late in a frame whose
    amplitude has sagged are still seen, while the long gaps between frames cannot trigger on noise."""
    from numpy.lib.stride_tricks import sliding_window_view
    mag = np.abs(x).astype(np.float32)
    n = max(3, int(round(rate * 40e-6)))
    sm = np.convolve(mag, np.ones(n, np.float32) / n, mode="same")
    hi = float(np.percentile(sm, 92))
    lo = float(np.percentile(sm, 8))
    if hi < 1.6 * lo:
        lo = 0.0
    span = max(hi - lo, 1e-12)
    d = 8
    pad_n = (-sm.size) % d
    m = np.concatenate([sm, np.full(pad_n, sm[-1], np.float32)]).reshape(-1, d).max(axis=1)
    w = max(3, int(0.003 * rate / d))
    lm = sliding_window_view(np.pad(m, w, mode="edge"), 2 * w + 1).max(axis=1)
    local = np.repeat(lm, d)[:sm.size]
    peak = np.minimum(np.maximum(local, lo + 0.25 * span), lo + 1.5 * span)
    mid = lo + 0.5 * (peak - lo)
    hy = 0.18 * (peak - lo)
    s = np.full(sm.size, -1, np.int8)
    s[sm > mid + hy] = 1
    s[sm < mid - hy] = 0
    idx = np.maximum.accumulate(np.where(s >= 0, np.arange(sm.size), 0))
    st = s[idx]
    st[st < 0] = 0
    return st.astype(np.int8)


def _bits_to_hex_q(code):
    """'1111 10?1...' -> hex, with '?' for any nibble that contains an unsure bit."""
    out = ""
    for i in range(0, len(code), 4):
        nib = code[i:i + 4].ljust(4, "0")
        out += "?" if "?" in nib else f"{int(nib, 2):X}"
    return out


def _pwm_frames(pulses, lows, rate, profile, strip_sync, skip_first=False):
    """Split a transmission into frames at the long sync gaps, decode every frame with ONE set of pulse-width thresholds
    (learned from the whole transmission, or the locked profile from the previous one) and vote across the frames
    position by position, so frames that fade out part-way still contribute their reliable first bits."""
    from collections import Counter
    p75 = float(np.percentile(lows, 75))
    gap_thr = max(2.2 * p75, 0.0012 * rate)            # a gap this long is not a bit
    big = [lo for _, lo in pulses if lo is not None and lo > gap_thr]
    sync_thr = gap_thr
    if len(big) >= 4:                                  # separate real sync gaps (the typical big gap) from shorter drop-outs
        G = float(np.median(big))
        if G > 2.5 * p75:
            sync_thr = max(gap_thr, 0.7 * G)
    segs, cur, gaps, broken = [], [], [], False
    for hi, lo in pulses:
        if not broken:
            cur.append((hi, lo))
        if lo is not None and lo > gap_thr:
            if lo > sync_thr:                          # frame separator
                segs.append((cur, not broken))
                gaps.append(lo)
                cur, broken = [], False
            else:                                      # drop-out inside a frame: what follows is unreliable until the next sync
                broken = True
    if cur:
        segs.append((cur, False))
    if skip_first and len(segs) >= 2:                  # this chunk continues a long press: its first piece starts mid-frame
        segs = segs[1:]
    frames = []
    for pl, gap_after in segs:
        data = pl[:-1] if (strip_sync and gap_after) else pl      # the pulse just before a sync gap is the sync pulse
        if len(data) >= 4:
            frames.append(data)
    if not frames:
        return None
    his = np.array([h for f in frames for h, _ in f])
    if his.size < 8:
        return None
    cc = sorted(sorted(_clusters_counts(his, 0.30), key=lambda t: -t[1])[:2])
    ok = (len(cc) == 2 and 1.6 < cc[1][0] / cc[0][0] < 4.5 and (cc[0][1] + cc[1][1]) >= 0.8 * his.size)
    locked = False
    if profile is not None:
        c0, c1 = profile["c0_us"] * rate * 1e-6, profile["c1_us"] * rate * 1e-6
        fit = float(np.mean((np.abs(his - c0) < 0.3 * c0) | (np.abs(his - c1) < 0.3 * c1)))
        if fit >= 0.85:
            cc, ok, locked = [(c0, 1), (c1, 1)], True, True
    if not ok:
        return None
    # PWM (as opposed to Manchester, which also has two pulse widths): constant gap, or constant bit period
    inner = [(h, l) for f in frames for h, l in f if l is not None and l <= gap_thr]
    if len(inner) < 6:
        return None
    psum = np.array([h + l for h, l in inner])
    const_period = float(psum.std() / psum.mean()) < 0.12
    low_cl = [c for c, k in _clusters_counts(np.array([l for _, l in inner]), 0.30) if k >= 0.10 * len(inner)]
    if not (const_period or len(low_cl) <= 1):
        return None
    thr = math.sqrt(cc[0][0] * cc[1][0])
    cand = []
    for f in frames:
        if all(0.6 * cc[0][0] < h < 1.5 * cc[1][0] for h, _ in f):
            s = "".join("1" if h > thr else "0" for h, _ in f)
            if len(s) >= 6:
                cand.append(s)
    if not cand:
        return None
    def ref_len(c):
        n_f = len(c)
        nd = max(2, math.ceil(0.2 * n_f)) if n_f > 1 else 1
        return nd, sorted((len(x) for x in c), reverse=True)[min(nd, n_f) - 1]   # longest length that at least nd frames reach
    need, Lref = ref_len(cand)
    if len(cand) > 2 and len(cand[0]) < Lref:          # a press that starts mid-frame (fade-in) begins with a TAIL fragment: drop it
        cand = cand[1:]
        need, Lref = ref_len(cand)
    F = len(cand)
    def majority(vs):
        return "".join("1" if sum(x[i] == "1" for x in vs) * 2 > len(vs) else "0" for i in range(len(vs[0])))
    full = [s[:Lref] for s in cand if len(s) >= Lref]
    partial = [s[:-1] for s in cand if len(s) < Lref and len(s) > 1]   # cut short: its last bit is suspect
    if len(full) >= 2:                                 # a short frame may only vote if it agrees with the full-length frames
        prov = majority(full)                          # (a genuine truncated frame does; a misaligned tail piece does not)
        partial = [v for v in partial if sum(a == b for a, b in zip(v, prov)) >= 0.85 * len(v)]
    voters = full + partial
    code, conf = [], []
    for i in range(Lref):
        col = [v[i] for v in voters if len(v) > i]
        if len(col) < min(need, len(voters)):
            break
        ones = col.count("1")
        code.append("1" if ones * 2 > len(col) else "0")
        conf.append(max(ones, len(col) - ones) / len(col))
    while code and conf[-1] < 0.6:                     # drop an unstable tail
        code.pop()
        conf.pop()
    if len(code) < 4:
        return None
    marked = "".join(c if q >= 0.8 else "?" for c, q in zip(code, conf))
    if marked.count("?") * 2 > len(marked):            # most bits unsure (e.g. 3 frames at the start of a press): not worth a line
        return "drop"
    plain = "".join(code)
    k = sum(1 for v in voters if v[:len(plain)] == plain[:len(v)])
    full = sum(1 for s in cand if len(s) >= len(plain))
    period = float(np.mean(psum))                      # average bit period (constant for fixed-period codes)
    lens_ct = Counter(len(s) for s in cand).most_common(4)
    r = {"baud": rate / period, "nbits": len(plain), "bits": [int(c) for c in plain],
         "hex": _bits_to_hex_q(marked), "bin": marked, "frames": F, "frames_all": len(frames), "agree": k, "full": full,
         "locked": locked,
         "coding": "PWM (long pulse=1, short=0), " + ("fixed bit period" if const_period else "fixed gap"),
         "notes": f"pulses {cc[0][0] / rate * 1e6:.0f}/{cc[1][0] / rate * 1e6:.0f} us; bit period {period / rate * 1e6:.0f} us"
                  + (f"; sync gap {float(np.median(gaps)) / rate * 1e3:.1f} ms" if gaps else "")
                  + "; frame lengths " + " ".join(f"{n}x{c}" for n, c in lens_ct)}
    prof = profile if locked else {"c0_us": cc[0][0] / rate * 1e6, "c1_us": cc[1][0] / rate * 1e6, "period_us": period / rate * 1e6}
    return r, prof


def analyse_ook_transmission(sl, rate, profile=None, strip_sync=True, pulses_out=None, skip_first=False):
    """One OOK transmission (0/1 array) -> ([result dicts], profile). A press of a PWM remote gives ONE result: the code that most
    frames agree on. Anything that is not PWM falls back to analyse_bits() on each burst separated by >= 4 ms."""
    vals, lens = _runs(sl)
    if vals.size < 6:
        return [], profile
    vals, lens = _deglitch(vals, lens, max(2, int(40e-6 * rate)))
    if vals.size < 6:
        return [], profile
    cc = _clusters_counts(lens.astype(float), 0.30)
    cand = [c for c, k in cc if k >= 0.08 * lens.size and c > 100e-6 * rate]
    if cand:
        vals, lens = _deglitch(vals, lens, int(0.45 * min(cand)))
    ones = np.flatnonzero(vals == 1)
    if ones.size < 4:
        return [], profile
    vals, lens = vals[ones[0]:ones[-1] + 1], lens[ones[0]:ones[-1] + 1]
    L = lens.astype(np.float64)
    pulses = [(L[i], L[i + 1] if i + 1 < L.size else None) for i in range(0, L.size, 2)]
    if pulses_out is not None:
        pulses_out[:] = pulses
    lows = np.array([lo for _, lo in pulses if lo is not None])
    if lows.size >= 6:
        res = _pwm_frames(pulses, lows, rate, profile, strip_sync, skip_first)
        if isinstance(res, str):                       # "drop": recognised as PWM but too unreliable to report
            return [], profile
        if res is not None:
            return [res[0]], res[1]
    out = []                                           # fallback: NRZ / Manchester / irregular, one analysis per burst
    sl2 = np.repeat(vals, lens)
    cuts = [0]
    pos = 0
    for v, n in zip(vals, lens):
        if v == 0 and n >= 0.004 * rate:
            cuts.append(pos + n // 2)
        pos += n
    cuts.append(sl2.size)
    for a, b in zip(cuts[:-1], cuts[1:]):
        if b - a >= 0.0008 * rate:
            r = analyse_bits(sl2[a:b], rate)
            if r is not None:
                r["t_off_s"] = a / rate
                r["dur_ms"] = (b - a) / rate * 1e3
                out.append(r)
    return out, profile


class BurstDecoder:
    """Streaming burst detector + demodulator.
    mode 'OOK': envelope -> Schmitt slicer; one TRANSMISSION (a whole button press, up to 0.6 s per report) is analysed as a
                whole: split into frames, one set of timing thresholds, vote across frames. Timing is remembered (self.profile)
                so the next press on the same remote is decoded with the same settings.
    mode 'FSK': FM discriminator -> adaptive threshold, one burst at a time (as before).
    Feed it IQ (complex64, rate_in); it yields result dicts."""
    def __init__(self, rate_in, mode="OOK", target_rate=None, bw_hz=None):
        self.mode = mode
        if target_rate is None:
            target_rate = 600e3 if mode == "OOK" else 250e3     # OOK: wide channel - cheap transmitters drift and chirp
        if bw_hz is None:
            bw_hz = 200e3 if mode == "OOK" else 100e3
        self.bw_hz = float(bw_hz)
        f = max(1, int(rate_in // target_rate))
        self.rate = rate_in / f
        cutoff = min(bw_hz, self.rate * 0.45)
        self.dec = Decimator(rate_in, f, cutoff, ntaps=int(min(401, max(63, 8.0 * rate_in / cutoff))) | 1)
        self.last_pulses_us = []
        self._flush_g = -10 ** 12       # global sample index of the last forced (max_len) flush
        self._cur_cont = False          # the buffer being collected continues a long press that was flushed a moment ago
        self.noise = None
        self.sens = 6.0
        self.strip_sync = True
        self.profile = None
        self.last_nbits = None          # length of the last well-supported (2+ frames) code, to reject lone fragments
        self.active = False
        self.buf = []
        self.pre_n = max(8, int(0.0005 * self.rate))
        self.pre = np.zeros(0, np.complex64)
        self.t_samples = 0
        self.t_burst0 = 0
        self.last_above = -10 ** 12
        self.gap_max = int((0.025 if mode == "OOK" else 0.004) * self.rate)   # quiet this long ends a transmission / burst
        self.min_len = int(0.0008 * self.rate)
        self.max_len = int((0.6 if mode == "OOK" else 0.5) * self.rate)

    def process(self, iq):
        out = []
        z = self.dec.process(iq)
        n = z.size
        if n == 0:
            return out
        sm = np.convolve(np.abs(z), np.ones(8, np.float32) / 8.0, mode="same")
        if self.noise is None:
            self.noise = float(np.median(sm))
        thr = max(self.noise * self.sens, 1e-5)
        above = sm > thr
        g0 = self.t_samples
        idx = np.arange(n)
        la = np.maximum.accumulate(np.where(above, idx, -1))
        last_g = np.where(la >= 0, g0 + la, self.last_above)       # global index of the last above-threshold sample so far
        held = (g0 + idx - last_g) < self.gap_max                  # True until gap_max quiet samples have passed
        prev_last = self.last_above
        self.last_above = int(last_g[-1])
        if not self.active:
            q = sm[~held]
            if q.size:
                self.noise = 0.98 * self.noise + 0.02 * float(np.median(q))
        ch = np.flatnonzero(np.diff(held.astype(np.int8))) + 1
        for a, b in zip(np.concatenate([[0], ch]), np.concatenate([ch, [n]])):
            a, b = int(a), int(b)
            if held[a]:
                if not self.active:
                    self.active = True
                    self._cur_cont = (g0 + a - self._flush_g) < int(0.003 * self.rate)
                    if a < self.pre_n and self.pre.size:
                        pre = self.pre[-(self.pre_n - a):]
                    else:
                        pre = np.zeros(0, np.complex64)
                    lead = z[max(0, a - self.pre_n):a]
                    self.buf = [np.concatenate([pre, lead, z[a:b]])]
                    self.t_burst0 = g0 + a - (pre.size + lead.size)
                else:
                    self.buf.append(z[a:b])
                if b < n:                                           # the run ended inside this block: transmission over
                    self._end(int(last_g[b - 1]), out)
            elif self.active:                                       # ended exactly at a block boundary
                self._end(int(last_g[a - 1]) if a > 0 else prev_last, out)
        if self.active and sum(c.size for c in self.buf) > self.max_len:
            self._end(self.last_above, out)                         # long press: report now, carry on with the next chunk
            self._flush_g = g0 + n
        self.pre = np.concatenate([self.pre, z])[-self.pre_n:]
        self.t_samples += n
        return out

    def _end(self, last_above_g, out):
        x = np.concatenate(self.buf) if self.buf else np.zeros(0, np.complex64)
        self.buf = []
        self.active = False
        keep = max(0, min(int(last_above_g + 1 + 16 - self.t_burst0), x.size))
        self._finish(x[:keep], out, self._cur_cont)

    def _finish(self, x, out, cont=False):
        if x.size < self.min_len:
            return
        t0 = self.t_burst0 / self.rate
        dur = x.size / self.rate * 1e3
        if self.mode == "OOK":
            pl = []
            res, self.profile = analyse_ook_transmission(_ook_slice(x, self.rate), self.rate, self.profile, self.strip_sync, pl, cont)
            self.last_pulses_us = [(h / self.rate * 1e6, None if l is None else l / self.rate * 1e6) for h, l in pl]
            for r in res:
                if r.get("frames") is not None:                      # PWM consensus result
                    if r["frames"] >= 3 and r["nbits"] >= 8:
                        self.last_nbits = r["nbits"]
                    elif self.last_nbits is not None and r["nbits"] != self.last_nbits:
                        continue                                      # a thinly supported result of a different length = a chopped fragment
                    elif "?" in r["bin"] or r["nbits"] < 8:
                        continue                                      # thinly supported AND unsure / tiny: a fragment, not a code
                r.update({"t_s": t0 + r.get("t_off_s", 0.0), "mode": "OOK"})
                r.setdefault("dur_ms", dur)
                out.append(r)
            return
        d = np.angle(x[1:] * np.conj(x[:-1]))
        mag = np.abs(x[1:])
        d = np.convolve(d, np.ones(5) / 5.0, mode="same")
        act = mag > 0.5 * np.percentile(mag, 90)
        if act.sum() < 8:
            return
        dd = d[act]
        thr = 0.5 * (np.percentile(dd, 95) + np.percentile(dd, 5))
        sl = (d > thr).astype(np.int8)
        sl[~act] = 0
        dev = float(0.5 * (np.percentile(dd, 95) - np.percentile(dd, 5)) * self.rate / (2 * math.pi))
        r = analyse_bits(sl, self.rate)
        if r is None:
            return
        r.update({"t_s": t0, "dur_ms": dur, "mode": "FSK", "dev_khz": dev / 1e3})
        out.append(r)

    def dump_text(self, max_frames=8):
        """The raw pulse table (high/low widths in us) of the last OOK transmission, one line per frame (split at gaps > 3 ms)."""
        if not self.last_pulses_us:
            return "no OOK transmission analysed yet"
        lines, cur = [], []
        for h, l in self.last_pulses_us:
            cur.append(f"{h:.0f}/" + ("end" if l is None else f"{l:.0f}"))
            if l is not None and l > 3000:
                lines.append(cur)
                cur = []
        if cur:
            lines.append(cur)
        out = [f"pulse table (high/low widths in us), {len(lines)} frames split at gaps > 3 ms; first {min(len(lines), max_frames)}:"]
        for i, ln in enumerate(lines[:max_frames]):
            out.append(f"  F{i + 1:<2d} ({len(ln):2d} pulses) " + " ".join(ln))
        return "\n".join(out)


#OLD# class BurstDecoder:
#OLD#     """Streaming burst detector + demodulator.
#OLD#     mode 'OOK': envelope -> adaptive threshold -> slicer.   mode 'FSK': FM discriminator -> adaptive threshold.
#OLD#     Feed it IQ (complex64, rate_in); it decimates, finds bursts (energy above noise floor), and when one ends calls
#OLD#     analyse_bits() and yields result dicts."""
#OLD#     def __init__(self, rate_in, mode="OOK", target_rate=250e3, bw_hz=100e3):
#OLD#         self.mode = mode
#OLD#         f = max(1, int(rate_in // target_rate))
#OLD#         self.rate = rate_in / f
#OLD#         self.dec = Decimator(rate_in, f, min(bw_hz, self.rate * 0.45))
#OLD#         self.noise = None           # running noise-floor estimate of smoothed magnitude
#OLD#         self.active = False
#OLD#         self.buf = []               # decimated complex samples of the burst in progress
#OLD#         self.pre = np.zeros(0, np.complex64)   # a little pre-roll
#OLD#         self.quiet = 0
#OLD#         self.t_samples = 0          # decimated samples consumed (for timestamps)
#OLD#         self.t_burst0 = 0
#OLD#         self.gap_max = int(0.004 * self.rate)    # a gap longer than 4 ms ends the burst
#OLD#         self.min_len = int(0.0008 * self.rate)   # ignore anything shorter than 0.8 ms
#OLD#         self.max_len = int(0.5 * self.rate)
#OLD#         self.sens = 6.0             # burst threshold = this many times the noise floor (amplitude)
#OLD#
#OLD#     def process(self, iq):
#OLD#         out = []
#OLD#         z = self.dec.process(iq)
#OLD#         if z.size == 0:
#OLD#             return out
#OLD#         mag = np.abs(z)
#OLD#         sm = np.convolve(mag, np.ones(8) / 8.0, mode="same")
#OLD#         if self.noise is None:
#OLD#             self.noise = float(np.median(sm))
#OLD#         # thr = max(self.noise * self.sens, 0.01)   # OLD: the fixed 0.01 (-40 dBFS) floor hid every burst weaker than that
#OLD#         thr = max(self.noise * self.sens, 1e-5)      # tiny floor only, so it can never be zero
#OLD#         above = sm > thr
#OLD#         # noise floor tracks only samples not in a burst
#OLD#         quiet = sm[~above]
#OLD#         if quiet.size:
#OLD#             self.noise = 0.98 * self.noise + 0.02 * float(np.median(quiet)) if not self.active else self.noise
#OLD#         i = 0
#OLD#         n = z.size
#OLD#         while i < n:
#OLD#             if not self.active:
#OLD#                 idx = np.flatnonzero(above[i:])
#OLD#                 if idx.size == 0:
#OLD#                     self.pre = z[-int(0.0005 * self.rate):].copy()
#OLD#                     break
#OLD#                 s = i + int(idx[0])
#OLD#                 self.active = True
#OLD#                 self.quiet = 0
#OLD#                 self.buf = [self.pre, z[s:]] if s == 0 else [z[max(0, s - 40):]]
#OLD#                 self.t_burst0 = self.t_samples + s
#OLD#                 # fallthrough: scan the rest of this block for the end
#OLD#                 seg_above = above[s:]
#OLD#                 rest_start = s
#OLD#             else:
#OLD#                 self.buf.append(z[i:])
#OLD#                 seg_above = above[i:]
#OLD#                 rest_start = i
#OLD#             # look for the end: a run of >= gap_max below-threshold samples
#OLD#             below = ~seg_above
#OLD#             if below.size >= self.gap_max:
#OLD#                 # sliding sum via cumsum
#OLD#                 cs = np.concatenate([[0], np.cumsum(below)])
#OLD#                 w = cs[self.gap_max:] - cs[:-self.gap_max]
#OLD#                 full = np.flatnonzero(w >= self.gap_max)
#OLD#             else:
#OLD#                 full = np.zeros(0, int)
#OLD#             tot = sum(b.size for b in self.buf)
#OLD#             if full.size:
#OLD#                 end_in_seg = int(full[0])                       # burst ends gap_max after last activity ~ start of the quiet run
#OLD#                 # trim buffer: remove the part after the last active sample
#OLD#                 keep = (tot - seg_above.size) + end_in_seg
#OLD#                 x = np.concatenate(self.buf)[:keep]
#OLD#                 self._finish(x, out)
#OLD#                 self.active = False
#OLD#                 self.buf = []
#OLD#                 i = rest_start + end_in_seg + self.gap_max
#OLD#                 continue
#OLD#             if tot > self.max_len:
#OLD#                 x = np.concatenate(self.buf)
#OLD#                 self._finish(x, out)
#OLD#                 self.active = False
#OLD#                 self.buf = []
#OLD#             break
#OLD#         self.t_samples += z.size
#OLD#         return out
#OLD#
#OLD#     def _finish(self, x, out):
#OLD#         if x.size < self.min_len:
#OLD#             return
#OLD#         t0 = self.t_burst0 / self.rate
#OLD#         if self.mode == "OOK":
#OLD#             sig = np.abs(x).astype(np.float32)
#OLD#             sig = np.convolve(sig, np.ones(3) / 3.0, mode="same")
#OLD#             hi, lo = np.percentile(sig, 95), np.percentile(sig, 10)
#OLD#             thr = 0.5 * (hi + lo) if hi > 2 * lo else 0.5 * hi
#OLD#             sl = (sig > thr).astype(np.int8)
#OLD#         else:
#OLD#             d = np.angle(x[1:] * np.conj(x[:-1]))
#OLD#             mag = np.abs(x[1:])
#OLD#             d = np.convolve(d, np.ones(5) / 5.0, mode="same")
#OLD#             act = mag > 0.5 * np.percentile(mag, 90)
#OLD#             if act.sum() < 8:
#OLD#                 return
#OLD#             dd = d[act]
#OLD#             thr = 0.5 * (np.percentile(dd, 95) + np.percentile(dd, 5))
#OLD#             sl = (d > thr).astype(np.int8)
#OLD#             sl[~act] = 0
#OLD#             res_dev = float(0.5 * (np.percentile(dd, 95) - np.percentile(dd, 5)) * self.rate / (2 * math.pi))
#OLD#         r = analyse_bits(sl, self.rate)
#OLD#         if r is None:
#OLD#             return
#OLD#         r.update({"t_s": t0, "dur_ms": x.size / self.rate * 1e3, "mode": self.mode})
#OLD#         if self.mode == "FSK":
#OLD#             r["dev_khz"] = res_dev / 1e3
#OLD#         out.append(r)


class DecoderSignals(QtCore.QObject):
    audio = QtCore.pyqtSignal(object)      # bytes: signed 16-bit mono, 48 kHz
    burst = QtCore.pyqtSignal(object)      # one decoded burst (dict from analyse_bits + timing)
    status = QtCore.pyqtSignal(str)
    marker = QtCore.pyqtSignal(float)      # the listening frequency, for the marker line on the spectrum
    stopped = QtCore.pyqtSignal(int)       # decoder thread ended (argument = session number)


DECODER_AUDIO_CUSHION_S = 0.30         # audio is pre-buffered this long before playing starts (and again after an under-run)


class DecoderAudioSink(QtCore.QObject):
    """Owns the QAudioOutput and lives in its OWN QThread (see EmcScanner.decoder_audio_open). The decoder thread sends 48 kHz mono int16
    chunks straight to push() through a queued signal, so a slow GUI redraw can no longer starve the sound card."""
    def __init__(self):
        super().__init__()
        self.out = None
        self.dev = None
        self.channels = 1
        self.primed = False
        self.prebuf = bytearray()
        self.prime_bytes = 0
        self.dropped = 0
        self.underruns = 0
        self.ok = False
        self.name = ""
        self.error = ""

    @QtCore.pyqtSlot()
    def open(self):
        self.ok = False
        try:
            info = QtMultimedia.QAudioDeviceInfo.defaultOutputDevice()
            fmt = None
            for ch in (1, 2):
                f = QtMultimedia.QAudioFormat()
                f.setSampleRate(AudioDemod.AUDIO_RATE)
                f.setChannelCount(ch)
                f.setSampleSize(16)
                f.setCodec("audio/pcm")
                f.setByteOrder(QtMultimedia.QAudioFormat.LittleEndian)
                f.setSampleType(QtMultimedia.QAudioFormat.SignedInt)
                if info.isFormatSupported(f):
                    fmt = f
                    self.channels = ch
                    break
            if fmt is None:
                self.error = "the default audio output does not accept 48 kHz 16-bit audio"
                return
            self.out = QtMultimedia.QAudioOutput(info, fmt)
            self.out.setBufferSize(int(AudioDemod.AUDIO_RATE * 2 * self.channels * 1.0))
            self.dev = self.out.start()
            self.prime_bytes = int(AudioDemod.AUDIO_RATE * 2 * self.channels * DECODER_AUDIO_CUSHION_S)
            self.name = info.deviceName()
            self.ok = True
        except Exception as e:
            self.error = repr(e)

    @QtCore.pyqtSlot(object)
    def push(self, data):
        out, dev = self.out, self.dev
        if out is None or dev is None:
            return
        if self.channels == 2:
            data = np.repeat(np.frombuffer(data, dtype=np.int16), 2).tobytes()
        if self.primed and out.state() == QtMultimedia.QAudio.IdleState:
            self.primed = False                        # under-run: build the cushion up again
            self.prebuf = bytearray()
            self.underruns += 1
        if not self.primed:
            self.prebuf += data
            if len(self.prebuf) >= self.prime_bytes:
                n = min(out.bytesFree(), len(self.prebuf))
                dev.write(bytes(self.prebuf[:n]))
                self.prebuf = bytearray()
                self.primed = True
            return
        if out.bytesFree() >= len(data):
            dev.write(data)
        else:
            self.dropped += 1

    @QtCore.pyqtSlot()
    def close(self):
        try:
            if self.out is not None:
                self.out.stop()
        except Exception:
            pass
        self.out = None
        self.dev = None
        self.primed = False


import collections
DECODER_USE_ASYNC = True        # gap-free capture through librtlsdr's ASYNCHRONOUS reader (see SdrStreamer). False = old synchronous reads
DECODER_LIVE_RETUNE = True      # centre-frequency / gain changes are made while the stream runs (as rtl_fm does).
                                # False = stop the stream, change, restart it (slower, most cautious)


def _iq_from_samples(samples):
    """Whatever read_samples() / packed_bytes_to_iq() delivered -> normalised complex64, identically to _scope_read_iq()."""
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


class SdrStreamer:
    """Continuous, gap-free IQ capture.

    Why: calling sdr.read_samples() repeatedly with processing in between is a SYNCHRONOUS USB read - while Python is busy (filters, FFT,
    redraws) nothing is queued to the dongle, so every sample it streams in that time is lost. That is a small hole at the end of every
    block: audible as warble, and it chops the end off every key-fob frame. Worse, the audio then gets fewer samples than real time
    and slowly runs dry.

    How: librtlsdr's asynchronous reader (read_bytes_async) runs in its own thread with ~15 USB buffers queued in the driver (about
    0.4 s of cushion), so the dongle is always being drained. Each buffer is converted and put in a queue; the decoder takes blocks
    from the queue at its own pace. Only if the decoder falls more than MAX_BLOCKS behind are blocks dropped (counted in .overruns)."""
    BUF_BYTES = 131072          # per USB buffer = 65536 samples (must be a multiple of 512)
    MAX_BLOCKS = 48             # about 1.3 s at 2.4 MS/s

    def __init__(self, sdr):
        self.sdr = sdr
        self.cv = threading.Condition()
        self.q = collections.deque()
        self.pending = np.zeros(0, np.complex64)
        self.thread = None
        self.running = False
        self.error = None
        self.overruns = 0
        self.blocks = 0

    def _convert(self, values):
        conv = getattr(self.sdr, "packed_bytes_to_iq", None)     # the same converter read_samples() uses -> identical scaling
        samples = conv(values) if conv is not None else np.ctypeslib.as_array(values).copy()
        return _iq_from_samples(samples)

    def _cb(self, values, ctx):
        """Runs in librtlsdr's reader thread: convert, queue, return - never block."""
        try:
            iq = self._convert(values)
        except Exception as e:
            self.error = e
            return
        with self.cv:
            self.q.append(iq)
            self.blocks += 1
            while len(self.q) > self.MAX_BLOCKS:
                self.q.popleft()
                self.overruns += 1
            self.cv.notify()

    def _run(self):
        try:
            self.sdr.read_bytes_async(self._cb, self.BUF_BYTES)    # blocks here until cancel_read_async()
        except Exception as e:
            self.error = e
        finally:
            self.running = False
            with self.cv:
                self.cv.notify_all()

    def start(self):
        self.stop()
        with self.cv:
            self.q.clear()
        self.pending = np.zeros(0, np.complex64)
        self.error = None
        self.overruns = 0
        self.blocks = 0
        self.running = True
        self.thread = threading.Thread(target=self._run, name="rtl-async-reader", daemon=True)
        self.thread.start()
        t_end = time.time() + 3.0                              # wait for the first buffer so a start-up failure is reported here
        with self.cv:
            while not self.q and self.error is None and self.running and time.time() < t_end:
                self.cv.wait(0.05)
        if self.error is not None:
            raise OSError(f"SDR async stream could not start: {self.error!r}")

    def stop(self):
        th = self.thread
        if th is None:
            return
        try:
            self.sdr.cancel_read_async()
        except Exception as e:
            print(f"SdrStreamer.stop: cancel_read_async: {e!r}")
        th.join(timeout=3.0)
        if th.is_alive():
            print("SdrStreamer.stop: reader thread did not finish in 3 s")
        self.thread = None
        self.running = False

    def flush(self):
        """Throw away everything captured so far (after a retune: the data is from the old frequency / still settling)."""
        with self.cv:
            self.q.clear()
        self.pending = np.zeros(0, np.complex64)

    @property
    def depth(self):
        return len(self.q)

    def read(self, n, timeout=2.0):
        """Exactly n samples (complex64), continuous with the previous call."""
        parts = [self.pending] if self.pending.size else []
        have = self.pending.size
        t_end = time.time() + timeout
        while have < n:
            with self.cv:
                while not self.q:
                    if self.error is not None:
                        raise OSError(f"SDR stream error: {self.error!r}")
                    if not self.running:
                        raise OSError("SDR stream stopped")
                    left = t_end - time.time()
                    if left <= 0:
                        raise OSError(f"SDR stream stalled (no data for {timeout:.1f} s)")
                    self.cv.wait(min(left, 0.1))
                blk = self.q.popleft()
            parts.append(blk)
            have += blk.size
        x = np.concatenate(parts) if len(parts) > 1 else parts[0]
        self.pending = x[n:].copy() if x.size > n else np.zeros(0, np.complex64)
        return x[:n]


import base64
import struct

WHITEN = [255, 254, 252, 248, 240, 225, 194, 133, 11, 23, 47, 94, 188, 120, 241, 227, 198, 141, 26, 52, 104, 208, 160, 64, 128, 1, 2, 4, 8, 17, 35, 71, 142, 28, 56, 113, 226, 196, 137, 18, 37, 75, 151, 46, 92, 184, 112, 224, 192, 129, 3, 6, 12, 25, 50, 100, 201, 146, 36, 73, 147, 38, 77, 155, 55, 110, 220, 185, 114, 228, 200, 144, 32, 65, 130, 5, 10, 21, 43, 86, 173, 91, 182, 109, 218, 181, 107, 214, 172, 89, 178, 101, 203, 150, 44, 88, 176, 97, 195, 135, 15, 31, 62, 125, 251, 246, 237, 219, 183, 111, 222, 189, 122, 245, 235, 215, 174, 93, 186, 116, 232, 209, 162, 68, 136, 16, 33, 67, 134, 13, 27, 54, 108, 216, 177, 99, 199, 143, 30, 60, 121, 243, 231, 206, 156, 57, 115, 230, 204, 152, 49, 98, 197, 139, 22, 45, 90, 180, 105, 210, 164, 72, 145, 34, 69, 138, 20, 41, 82, 165, 74, 149, 42, 84, 169, 83, 167, 78, 157, 59, 119, 238, 221, 187, 118, 236, 217, 179, 103, 207, 158, 61, 123, 247, 239, 223, 191, 126, 253, 250, 244, 233, 211, 166, 76, 153, 51, 102, 205, 154, 53, 106, 212, 168, 81, 163, 70, 140, 24, 48, 96, 193, 131, 7, 14, 29, 58, 117, 234, 213, 170, 85, 171, 87, 175, 95, 190, 124, 249, 242, 229, 202, 148, 40, 80, 161, 66, 132, 9, 19, 39, 79, 159, 63, 127]
HDR_MATRIX = [[1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0], [1, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 1], [0, 1, 0, 0, 1, 0, 0, 1, 1, 0, 1, 0], [0, 0, 1, 0, 0, 1, 0, 1, 0, 1, 1, 1], [0, 0, 0, 1, 0, 0, 1, 0, 1, 1, 1, 1]]

def _lbit(w, k):            # MATLAB bitget(w,k), k 1-based
    return (int(w) >> (k - 1)) & 1

def calc_crc_bits(data_bytes):
    """CRC-16 poly 0x1021, init 0, MSB first, over the given bytes."""
    crc = 0
    for b in data_bytes:
        crc ^= (b << 8)
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc

def lora_crc(data):
    """LoRaPHY.calc_crc -> [b1, b2] (the two bytes that follow the payload on air)."""
    n = len(data)
    if n == 0: return [0, 0]
    if n == 1: return [data[-1], 0]
    if n == 2: return [data[-1], data[-2]]
    c = calc_crc_bits(data[:-2])
    return [(c & 0xFF) ^ data[-1], ((c >> 8) & 0xFF) ^ data[-2]]

def header_checksum(n1, n2, n3):
    bits = []
    for n in (n1, n2, n3):
        bits += [(n >> 3) & 1, (n >> 2) & 1, (n >> 1) & 1, n & 1]
    return [sum(r * b for r, b in zip(row, bits)) & 1 for row in HDR_MATRIX]

def diag_deinterleave(symbols, ppm):
    rdd = len(symbols)
    # rows of bits, MSB first
    b = [[(s >> (ppm - 1 - c)) & 1 for c in range(ppm)] for s in symbols]
    cw = [0] * ppm
    for j in range(rdd):
        t = [b[j][(c + j) % ppm] for c in range(ppm)]       # circular left shift by j
        for r in range(ppm):
            cw[r] |= t[r] << j
    return cw[::-1]                                          # flipud

def hamming_decode(cws, rdd):
    out = []
    for w in cws:
        if rdd in (5, 6):
            out.append(w & 15)
        else:
            p2 = _lbit(w,7)^_lbit(w,4)^_lbit(w,2)^_lbit(w,1)
            p3 = _lbit(w,5)^_lbit(w,3)^_lbit(w,2)^_lbit(w,1)
            p5 = _lbit(w,6)^_lbit(w,4)^_lbit(w,3)^_lbit(w,2)
            pf = {3: 4, 5: 8, 6: 1, 7: 2}.get(p2 * 4 + p3 * 2 + p5, 0)
            out.append((w ^ pf) & 15)
    return out

def gray_coding(din, sf, ldr):
    d = list(din)
    out = []
    for i, v in enumerate(d):
        if i < 8 or ldr:
            s = int(v) // 4
        else:
            s = (int(v) - 1) % (1 << sf)
        out.append(s ^ (s >> 1))
    return out

def calc_sym_num(plen, sf, cr, crc, has_header, ldr):
    return 8 + max((4 + cr) * math.ceil((2 * plen - sf + 7 + 4 * crc - 5 * (1 - has_header)) / (sf - 2 * ldr)), 0)

def lora_whiten(data):
    return [b ^ WHITEN[i] for i, b in enumerate(data)]

def lora_decode(symbols, sf, ldr, has_header=1, cr=None, crc=None, plen=None):
    """symbols: demodulated values (after CFO/drift compensation). Returns dict."""
    g = gray_coding(symbols, sf, ldr)
    cws = diag_deinterleave(g[:8], sf - 2)
    nibs = hamming_decode(cws, 8)
    info = {}
    if has_header:
        plen = nibs[0] * 16 + nibs[1]; crc = nibs[2] & 1; cr = nibs[2] >> 1
        chk = [nibs[3] & 1] + [(nibs[4] >> s) & 1 for s in (3, 2, 1, 0)]
        calc = header_checksum(nibs[0], nibs[1], nibs[2])
        info["header_ok"] = (chk == calc)
        nibs = nibs[5:]
    rdd = cr + 4
    ii = 8
    while ii + rdd <= len(g):
        cw = diag_deinterleave(g[ii:ii + rdd], sf - 2 * ldr)
        nibs += hamming_decode(cw, rdd)
        ii += rdd
    nb = len(nibs) // 2                       # (the MATLAB reference caps this at 255 bytes, cutting off the CRC of a 255-byte payload)
    by = [nibs[2 * k] | (nibs[2 * k + 1] << 4) for k in range(nb)]
    if crc:
        data = lora_whiten(by[:plen])
        rx_crc = by[plen:plen + 2]
        info["crc_ok"] = (lora_crc(data) == rx_crc)
        info["rx_crc"] = rx_crc
    else:
        data = lora_whiten(by[:plen])
    info.update(payload=data, plen=plen, cr=cr, crc=crc)
    return info


class FracResampler:
    """Streaming windowed-sinc resampler fs_in -> fs_out (any ratio), with anti-alias cut-off just below fs_out/2."""
    def __init__(self, fs_in, fs_out, zeros=6, phases=512, cutoff_frac=0.46):
        self.ratio = fs_in / float(fs_out)
        fc = cutoff_frac * min(fs_out, fs_in) / fs_in                    # cycles per input sample
        self.hw = max(4, int(math.ceil(zeros / (2 * fc))))
        self.P = phases
        offs = np.arange(-self.hw + 1, self.hw + 1)
        tab = np.empty((phases, offs.size), np.float32)
        win = np.kaiser(2 * self.hw + 1, 8.0)
        for p in range(phases):
            x = offs - p / phases
            h = 2 * fc * np.sinc(2 * fc * x)
            w = np.interp(x, np.arange(-self.hw, self.hw + 1), win)
            k = h * w
            tab[p] = k / k.sum()
        self.tab, self.offs = tab, offs
        self.tail = np.zeros(0, np.complex64)
        self.t = 0.0                                                       # next output time, in input samples relative to tail[0]

    def process(self, x):
        xa = np.concatenate([self.tail, x.astype(np.complex64)])
        if xa.size < 2 * self.hw + 2:
            self.tail = xa
            return np.zeros(0, np.complex64)
        n_out = int((xa.size - self.hw - 1 - self.t) // self.ratio) + 1
        if n_out <= 0:
            self.tail = xa
            return np.zeros(0, np.complex64)
        tk = self.t + np.arange(n_out) * self.ratio
        i0 = np.floor(tk).astype(np.int64)
        ph = np.rint((tk - i0) * self.P).astype(np.int64)
        i0 += ph // self.P
        ph %= self.P
        # the first outputs need samples before xa[0] when t < hw: they were not computable earlier; pad with zeros at start
        idx = i0[:, None] + self.offs[None, :]
        idx = np.clip(idx, 0, xa.size - 1)
        y = (xa[idx] * self.tab[ph]).sum(axis=1).astype(np.complex64)
        t_next = self.t + n_out * self.ratio
        keep_from = max(0, int(math.floor(t_next)) - self.hw)
        self.tail = xa[keep_from:].copy()
        self.t = t_next - keep_from
        return y


class NeedMore(Exception):
    def __init__(self, upto):
        self.upto = upto


class LoraRx:
    OSR = 2

    def __init__(self, sf, bw, fs_in, rf_freq=868e6, sync_word=None, zp=4, pdet=5, ldr=None, min_snr_peak=2.0, ref_resample=True,
                 cutoff_frac=0.44):
        self.sf, self.bw, self.fs_in = int(sf), float(bw), float(fs_in)
        self.N = 1 << self.sf
        self.W = self.OSR * self.N
        self.zp = int(zp)
        self.rf_freq = float(rf_freq)
        self.sync_word = sync_word
        self.pdet = int(pdet)
        self.min_peak = min_snr_peak
        self.ldr = (1 if self.N / self.bw > 16e-3 else 0) if ldr is None else int(ldr)
        n = np.arange(self.W)
        f = ((n / self.OSR) / self.N - 0.5)                                   # cycles per (1/BW) -> fraction of BW
        ph = 2 * np.pi * np.cumsum(f * self.bw / (self.OSR * self.bw))          # fs = OSR*bw
        ph = np.concatenate([[0], ph[:-1]])
        up = np.exp(1j * ph).astype(np.complex64)
        self.up = up
        self.dn = np.conj(up)
        self.cutoff_frac = float(cutoff_frac)          # front-end cut-off as a fraction of fs_out (= 0.8 BW for 0.40 at OSR 2)
        self.fe = FracResampler(self.fs_in, self.OSR * self.bw, cutoff_frac=cutoff_frac) if ref_resample else None
        self.buf = np.zeros(0, np.complex64)
        self.base = 0                                                           # absolute index of buf[0]
        self.scan = 0                                                           # absolute index of the next search window
        self.run = []                                                           # current run of equal preamble bins [(bin, abs_start)]
        self.ctx = None                                                         # frame in progress
        self.frames_seen = 0
        self.detections = 0
        self.noise_floor = 0.0

    # ---------------------------------------------------------------- helpers
    @staticmethod
    def _fold(F1, F2):
        """Combine the two partial tones of a wrapped chirp (bins k and k+N of the full-rate FFT) COHERENTLY. Their relative phase is
        exp(-j*pi*tau) for a window offset of tau samples (+-1 when aligned to the 1/BW grid, anything otherwise), so try 1, j, -1, -j
        and keep the strongest: within 0.7 dB of knowing the phase, ~4 dB better than adding the two magnitudes."""
        a = np.abs(F1 + F2)
        b = np.abs(F1 - F2)
        c = np.abs(F1 + 1j * F2)
        d = np.abs(F1 - 1j * F2)
        return np.maximum(np.maximum(a, b), np.maximum(c, d)) * 0.5
    def _seg(self, a, n=None):
        n = self.W if n is None else n
        i = a - self.base
        if i < 0:
            raise ValueError("data no longer in buffer")
        if i + n > self.buf.size:
            raise NeedMore(a + n)
        return self.buf[i:i + n]

    def _dechirp(self, a, up=True, zp=None):
        """peak (magnitude, bin in 1/zp units 0-based, folded) of the dechirped window starting at absolute index a."""
        zp = self.zp if zp is None else zp
        seg = self._seg(a)
        ref = self.dn if up else self.up
        F = np.fft.fft(seg * ref, n=self.W * zp)
        mag = self._fold(F[:self.N * zp], F[self.N * zp:])
        k = int(np.argmax(mag))
        return float(mag[k]), k, mag

    # ---------------------------------------------------------------- streaming interface
    def process(self, iq):
        x = self.fe.process(iq) if self.fe is not None else iq
        return self.feed_bb(x)

    def feed_bb(self, x):
        self.buf = np.concatenate([self.buf, x.astype(np.complex64)]) if x.size else self.buf
        frames = []
        for _ in range(10000):
            if self.ctx is not None:
                try:
                    r = self._advance_frame()
                except NeedMore:
                    break
                if r is not None:
                    frames.append(r)
                continue
            if not self._search():
                break
        self._trim()
        return frames

    def _trim(self):
        if self.ctx is None:
            keep_from = max(self.base, self.scan - (self.pdet + 4) * self.W)
        else:
            keep_from = max(self.base, self.ctx["det_end"] - (self.pdet + 8) * self.W)
        cut = keep_from - self.base
        if cut > 0:
            self.buf = self.buf[cut:]
            self.base += cut

    def _search(self):
        """Vectorised scan of whole windows for >= pdet consecutive equal up-chirp peaks. Returns True if it made progress."""
        W, N = self.W, self.N
        i0 = self.scan - self.base
        nwin = (self.buf.size - i0) // W
        if nwin <= 0:
            return False
        seg = self.buf[i0:i0 + nwin * W].reshape(nwin, W)
        F = np.fft.fft(seg * self.dn[None, :], axis=1)
        mag = self._fold(F[:, :N], F[:, N:])
        bins = mag.argmax(axis=1)
        peak = mag.max(axis=1)
        med = np.median(mag, axis=1)
        good = peak > self.min_peak * med
        if self.run == [] and nwin:
            self.noise_floor = float(np.median(med))
        for j in range(nwin):
            a = self.scan + j * W
            if not good[j]:
                self.run = []
                continue
            b = int(bins[j])
            if self.run:
                d = (b - self.run[-1][0]) % N
                d = min(d, N - d)
                if d <= 1:
                    self.run.append((b, a))
                else:
                    self.run = [(b, a)]
            else:
                self.run = [(b, a)]
            if len(self.run) >= self.pdet:
                det_end = a + W
                self.detections += 1
                self.ctx = {"stage": "sync", "det_end": det_end, "bin0": b, "run0": self.run[0][1]}
                self.run = []
                self.scan = det_end
                return True
        self.scan += nwin * W
        return True

    # ---------------------------------------------------------------- frame state machine
    def _fail(self, skip_windows):
        self.scan = self.ctx["det_end"] + skip_windows * self.W
        self.ctx = None
        self.run = []
        return None

    def _advance_frame(self):
        c = self.ctx
        W, N, zp = self.W, self.N, self.zp
        if c["stage"] == "sync":
            try:
                self._sync(c)
            except ValueError:
                return self._fail(2)
            c["stage"] = "header"
        if c["stage"] == "header":
            x = c["x"]
            syms = []
            for ii in range(8):
                pk, k, _ = self._dechirp(x + ii * W)
                syms.append(((k - c["pre_bin"]) / zp) % N)
            valid, plen, cr, crc, info = self._parse_header(syms, c)
            if not valid:
                return self._fail(9)
            c.update(plen=plen, cr=cr, crc=crc, hdr_syms=syms)
            c["sym_num"] = calc_sym_num(plen, self.sf, cr, crc, 1, self.ldr)
            c["stage"] = "payload"
        if c["stage"] == "payload":
            x = c["x"]
            self._seg(x + (c["sym_num"] - 1) * W)                 # raises NeedMore until the whole frame is in the buffer
            syms = list(c["hdr_syms"])
            peaks = []
            for ii in range(8, c["sym_num"]):
                pk, k, _ = self._dechirp(x + ii * W)
                syms.append(((k - c["pre_bin"]) / zp) % N)
            comp = self._dynamic_compensation(syms, c)
            sym_i = [int(round(s)) % N for s in comp]
            r = lora_decode(sym_i, self.sf, self.ldr, has_header=1)
            frame = dict(sf=self.sf, bw=self.bw, cr=r["cr"], has_crc=bool(r["crc"]), payload=bytes(r["payload"]),
                         length=r["plen"], crc_ok=(r.get("crc_ok") if r["crc"] else None), header_ok=r.get("header_ok", True),
                         cfo_hz=c["cfo"], snr_db=c["snr_db"], sync=c.get("netid"), n_symbols=c["sym_num"],
                         t_start=(c["run0"]) / (self.OSR * self.bw), t_end=(x + c["sym_num"] * W) / (self.OSR * self.bw),
                         ldro=self.ldr, rx_crc=r.get("rx_crc"))
            self.scan = x + c["sym_num"] * W
            self.ctx = None
            self.run = []
            self.frames_seen += 1
            return frame
        return None

    def _sync(self, c):
        """Port of LoRaPHY.sync(): find the SFD, align up/down, derive CFO and the first data symbol."""
        W, N, zp = self.W, self.N, self.zp
        nb = N * zp
        # coarse alignment from the preamble bin ("shift the up peak to position 0")
        b = c["bin0"]
        x = c["det_end"] - int(round(b * self.OSR))
        # find the first window dominated by a down-chirp
        found = False
        for _ in range(self.pdet + 40):
            up_pk = self._dechirp(x, True)[0]
            dn_pk = self._dechirp(x, False)[0]
            x += W
            if dn_pk > up_pk:
                found = True
                break
        if not found:
            raise ValueError("no SFD")
        pkd = self._dechirp(x, False)
        kd = pkd[1]
        to = round((kd - nb) / zp) if kd > nb / 2 else round(kd / zp)
        x += int(to)
        pku = self._dechirp(x - 4 * W)
        pre_bin = pku[1]
        c["pre_bin"] = pre_bin
        cfo_bins = (pre_bin - nb) / zp if pre_bin > nb / 2 else pre_bin / zp
        c["cfo"] = cfo_bins * self.bw / N
        pu = self._dechirp(x - W)[0]
        pd = self._dechirp(x - W, False)[0]
        x_sync = x + int(round(2.25 * W)) if pu > pd else x + int(round(1.25 * W))
        c["x"] = x_sync
        # sync word (netid) chirps, relative to the preamble bin
        n1 = self._dechirp(int(round(x_sync - 4.25 * W)))[1]
        n2 = self._dechirp(int(round(x_sync - 3.25 * W)))[1]
        v1 = ((n1 + nb - pre_bin) / zp) % N
        v2 = ((n2 + nb - pre_bin) / zp) % N
        c["netid"] = (int(round(v1 / 8)) % 16) << 4 | (int(round(v2 / 8)) % 16)
        # SNR estimate from the aligned last preamble window
        pk, k, mag = self._dechirp(x - 4 * W)
        m2 = mag.astype(np.float64) ** 2
        k0 = int(k)
        mask = np.ones(m2.size, bool)
        for d in range(-2 * zp, 2 * zp + 1):
            mask[(k0 + d) % m2.size] = False
        noise = float(np.mean(m2[mask])) if mask.any() else 1e-12
        c["snr_db"] = 10 * math.log10(max(pk ** 2 / max(noise, 1e-20), 1e-3) / N * (2 * self.cutoff_frac * self.OSR))   # noise bandwidth correction

    def _dynamic_compensation(self, data, c):
        """Port of LoRaPHY.dynamic_compensation: sampling-frequency-offset drift + LDRO bin tracking."""
        N = self.N
        out = []
        for i, v in enumerate(data):
            drift = (1 + (i + 1)) * N * c["cfo"] / self.rf_freq if self.rf_freq else 0.0
            out.append((v - drift) % N)
        if self.ldr:
            bin_offset, v_last = 0.0, 1.0
            res = []
            for v in out:
                d = (v - v_last) % 4
                bin_offset += (-d if d < 2 else (-d + 4))
                v_last = v
                res.append((v + bin_offset) % N)
            out = res
        return out

    def _parse_header(self, syms, c):
        sf = self.sf
        comp = self._dynamic_compensation(syms, c)
        comp = [int(round(s)) % self.N for s in comp]
        g = gray_coding(comp, sf, self.ldr)
        cws = diag_deinterleave(g[:8], sf - 2)
        nibs = hamming_decode(cws, 8)
        plen = nibs[0] * 16 + nibs[1]
        crc = nibs[2] & 1
        cr = nibs[2] >> 1
        chk = [nibs[3] & 1] + [(nibs[4] >> s) & 1 for s in (3, 2, 1, 0)]
        ok = chk == header_checksum(nibs[0], nibs[1], nibs[2]) and 1 <= cr <= 4 and plen > 0
        return ok, plen, cr, crc, {}


# Generated from the official meshtastic protobuf definitions (meshtastic python package).
MT_MSG = {'Data': {1: ('portnum', 'enum:PortNum', False), 2: ('payload', 'bytes', False), 3: ('want_response', 'bool', False), 4: ('dest', 'fixed32', False), 5: ('source', 'fixed32', False), 6: ('request_id', 'fixed32', False), 7: ('reply_id', 'fixed32', False), 8: ('emoji', 'fixed32', False), 9: ('bitfield', 'uint', False)}, 'Position': {1: ('latitude_i', 'sfixed32', False), 2: ('longitude_i', 'sfixed32', False), 3: ('altitude', 'int', False), 4: ('time', 'fixed32', False), 5: ('location_source', 'enum:LocSource', False), 6: ('altitude_source', 'enum:AltSource', False), 7: ('timestamp', 'fixed32', False), 8: ('timestamp_millis_adjust', 'int', False), 9: ('altitude_hae', 'sint', False), 10: ('altitude_geoidal_separation', 'sint', False), 11: ('PDOP', 'uint', False), 12: ('HDOP', 'uint', False), 13: ('VDOP', 'uint', False), 14: ('gps_accuracy', 'uint', False), 15: ('ground_speed', 'uint', False), 16: ('ground_track', 'uint', False), 17: ('fix_quality', 'uint', False), 18: ('fix_type', 'uint', False), 19: ('sats_in_view', 'uint', False), 20: ('sensor_id', 'uint', False), 21: ('next_update', 'uint', False), 22: ('seq_number', 'uint', False), 23: ('precision_bits', 'uint', False)}, 'User': {1: ('id', 'string', False), 2: ('long_name', 'string', False), 3: ('short_name', 'string', False), 4: ('macaddr', 'bytes', False), 5: ('hw_model', 'enum:HardwareModel', False), 6: ('is_licensed', 'bool', False), 7: ('role', 'enum:Role', False), 8: ('public_key', 'bytes', False), 9: ('is_unmessagable', 'bool', False)}, 'DeviceMetrics': {1: ('battery_level', 'uint', False), 2: ('voltage', 'float', False), 3: ('channel_utilization', 'float', False), 4: ('air_util_tx', 'float', False), 5: ('uptime_seconds', 'uint', False)}, 'EnvironmentMetrics': {1: ('temperature', 'float', False), 2: ('relative_humidity', 'float', False), 3: ('barometric_pressure', 'float', False), 4: ('gas_resistance', 'float', False), 5: ('voltage', 'float', False), 6: ('current', 'float', False), 7: ('iaq', 'uint', False), 8: ('distance', 'float', False), 9: ('lux', 'float', False), 10: ('white_lux', 'float', False), 11: ('ir_lux', 'float', False), 12: ('uv_lux', 'float', False), 13: ('wind_direction', 'uint', False), 14: ('wind_speed', 'float', False), 15: ('weight', 'float', False), 16: ('wind_gust', 'float', False), 17: ('wind_lull', 'float', False), 18: ('radiation', 'float', False), 19: ('rainfall_1h', 'float', False), 20: ('rainfall_24h', 'float', False), 21: ('soil_moisture', 'uint', False), 22: ('soil_temperature', 'float', False), 23: ('one_wire_temperature', 'float', True)}, 'AirQualityMetrics': {1: ('pm10_standard', 'uint', False), 2: ('pm25_standard', 'uint', False), 3: ('pm100_standard', 'uint', False), 4: ('pm10_environmental', 'uint', False), 5: ('pm25_environmental', 'uint', False), 6: ('pm100_environmental', 'uint', False), 7: ('particles_03um', 'uint', False), 8: ('particles_05um', 'uint', False), 9: ('particles_10um', 'uint', False), 10: ('particles_25um', 'uint', False), 11: ('particles_50um', 'uint', False), 12: ('particles_100um', 'uint', False), 13: ('co2', 'uint', False), 14: ('co2_temperature', 'float', False), 15: ('co2_humidity', 'float', False), 16: ('form_formaldehyde', 'float', False), 17: ('form_humidity', 'float', False), 18: ('form_temperature', 'float', False), 19: ('pm40_standard', 'uint', False), 20: ('particles_40um', 'uint', False), 21: ('pm_temperature', 'float', False), 22: ('pm_humidity', 'float', False), 23: ('pm_voc_idx', 'float', False), 24: ('pm_nox_idx', 'float', False), 25: ('particles_tps', 'float', False)}, 'PowerMetrics': {1: ('ch1_voltage', 'float', False), 2: ('ch1_current', 'float', False), 3: ('ch2_voltage', 'float', False), 4: ('ch2_current', 'float', False), 5: ('ch3_voltage', 'float', False), 6: ('ch3_current', 'float', False), 7: ('ch4_voltage', 'float', False), 8: ('ch4_current', 'float', False), 9: ('ch5_voltage', 'float', False), 10: ('ch5_current', 'float', False), 11: ('ch6_voltage', 'float', False), 12: ('ch6_current', 'float', False), 13: ('ch7_voltage', 'float', False), 14: ('ch7_current', 'float', False), 15: ('ch8_voltage', 'float', False), 16: ('ch8_current', 'float', False)}, 'LocalStats': {1: ('uptime_seconds', 'uint', False), 2: ('channel_utilization', 'float', False), 3: ('air_util_tx', 'float', False), 4: ('num_packets_tx', 'uint', False), 5: ('num_packets_rx', 'uint', False), 6: ('num_packets_rx_bad', 'uint', False), 7: ('num_online_nodes', 'uint', False), 8: ('num_total_nodes', 'uint', False), 9: ('num_rx_dupe', 'uint', False), 10: ('num_tx_relay', 'uint', False), 11: ('num_tx_relay_canceled', 'uint', False), 12: ('heap_total_bytes', 'uint', False), 13: ('heap_free_bytes', 'uint', False), 14: ('num_tx_dropped', 'uint', False), 15: ('noise_floor', 'int', False)}, 'HealthMetrics': {1: ('heart_bpm', 'uint', False), 2: ('spO2', 'uint', False), 3: ('temperature', 'float', False)}, 'HostMetrics': {1: ('uptime_seconds', 'uint', False), 2: ('freemem_bytes', 'uint', False), 3: ('diskfree1_bytes', 'uint', False), 4: ('diskfree2_bytes', 'uint', False), 5: ('diskfree3_bytes', 'uint', False), 6: ('load1', 'uint', False), 7: ('load5', 'uint', False), 8: ('load15', 'uint', False), 9: ('user_string', 'string', False)}, 'TrafficManagementStats': {1: ('packets_inspected', 'uint', False), 2: ('position_dedup_drops', 'uint', False), 3: ('nodeinfo_cache_hits', 'uint', False), 4: ('rate_limit_drops', 'uint', False), 5: ('unknown_packet_drops', 'uint', False), 6: ('hop_exhausted_packets', 'uint', False), 7: ('router_hops_preserved', 'uint', False)}, 'Telemetry': {1: ('time', 'fixed32', False), 2: ('device_metrics', 'msg:DeviceMetrics', False), 3: ('environment_metrics', 'msg:EnvironmentMetrics', False), 4: ('air_quality_metrics', 'msg:AirQualityMetrics', False), 5: ('power_metrics', 'msg:PowerMetrics', False), 6: ('local_stats', 'msg:LocalStats', False), 7: ('health_metrics', 'msg:HealthMetrics', False), 8: ('host_metrics', 'msg:HostMetrics', False), 9: ('traffic_management_stats', 'msg:TrafficManagementStats', False)}, 'RouteDiscovery': {1: ('route', 'fixed32', True), 2: ('snr_towards', 'int', True), 3: ('route_back', 'fixed32', True), 4: ('snr_back', 'int', True)}, 'Routing': {1: ('route_request', 'msg:RouteDiscovery', False), 2: ('route_reply', 'msg:RouteDiscovery', False), 3: ('error_reason', 'enum:Error', False)}, 'Waypoint': {1: ('id', 'uint', False), 2: ('latitude_i', 'sfixed32', False), 3: ('longitude_i', 'sfixed32', False), 4: ('expire', 'uint', False), 5: ('locked_to', 'uint', False), 6: ('name', 'string', False), 7: ('description', 'string', False), 8: ('icon', 'fixed32', False)}, 'Neighbor': {1: ('node_id', 'uint', False), 2: ('snr', 'float', False), 3: ('last_rx_time', 'fixed32', False), 4: ('node_broadcast_interval_secs', 'uint', False)}, 'NeighborInfo': {1: ('node_id', 'uint', False), 2: ('last_sent_by_id', 'uint', False), 3: ('node_broadcast_interval_secs', 'uint', False), 4: ('neighbors', 'msg:Neighbor', True)}}

MT_ENUM = {'PortNum': {0: 'UNKNOWN_APP', 1: 'TEXT_MESSAGE_APP', 2: 'REMOTE_HARDWARE_APP', 3: 'POSITION_APP', 4: 'NODEINFO_APP', 5: 'ROUTING_APP', 6: 'ADMIN_APP', 7: 'TEXT_MESSAGE_COMPRESSED_APP', 8: 'WAYPOINT_APP', 9: 'AUDIO_APP', 10: 'DETECTION_SENSOR_APP', 11: 'ALERT_APP', 12: 'KEY_VERIFICATION_APP', 13: 'REMOTE_SHELL_APP', 32: 'REPLY_APP', 33: 'IP_TUNNEL_APP', 34: 'PAXCOUNTER_APP', 35: 'STORE_FORWARD_PLUSPLUS_APP', 36: 'NODE_STATUS_APP', 64: 'SERIAL_APP', 65: 'STORE_FORWARD_APP', 66: 'RANGE_TEST_APP', 67: 'TELEMETRY_APP', 68: 'ZPS_APP', 69: 'SIMULATOR_APP', 70: 'TRACEROUTE_APP', 71: 'NEIGHBORINFO_APP', 72: 'ATAK_PLUGIN', 73: 'MAP_REPORT_APP', 74: 'POWERSTRESS_APP', 75: 'LORAWAN_BRIDGE', 76: 'RETICULUM_TUNNEL_APP', 77: 'CAYENNE_APP', 78: 'ATAK_PLUGIN_V2', 112: 'GROUPALARM_APP', 256: 'PRIVATE_APP', 257: 'ATAK_FORWARDER', 511: 'MAX'}, 'LocSource': {0: 'LOC_UNSET', 1: 'LOC_MANUAL', 2: 'LOC_INTERNAL', 3: 'LOC_EXTERNAL'}, 'AltSource': {0: 'ALT_UNSET', 1: 'ALT_MANUAL', 2: 'ALT_INTERNAL', 3: 'ALT_EXTERNAL', 4: 'ALT_BAROMETRIC'}, 'HardwareModel': {0: 'UNSET', 1: 'TLORA_V2', 2: 'TLORA_V1', 3: 'TLORA_V2_1_1P6', 4: 'TBEAM', 5: 'HELTEC_V2_0', 6: 'TBEAM_V0P7', 7: 'T_ECHO', 8: 'TLORA_V1_1P3', 9: 'RAK4631', 10: 'HELTEC_V2_1', 11: 'HELTEC_V1', 12: 'LILYGO_TBEAM_S3_CORE', 13: 'RAK11200', 14: 'NANO_G1', 15: 'TLORA_V2_1_1P8', 16: 'TLORA_T3_S3', 17: 'NANO_G1_EXPLORER', 18: 'NANO_G2_ULTRA', 19: 'LORA_TYPE', 20: 'WIPHONE', 21: 'WIO_WM1110', 22: 'RAK2560', 23: 'HELTEC_HRU_3601', 24: 'HELTEC_WIRELESS_BRIDGE', 25: 'STATION_G1', 26: 'RAK11310', 27: 'SENSELORA_RP2040', 28: 'SENSELORA_S3', 29: 'CANARYONE', 30: 'RP2040_LORA', 31: 'STATION_G2', 32: 'LORA_RELAY_V1', 33: 'T_ECHO_PLUS', 34: 'PPR', 35: 'GENIEBLOCKS', 36: 'NRF52_UNKNOWN', 37: 'PORTDUINO', 38: 'ANDROID_SIM', 39: 'DIY_V1', 40: 'NRF52840_PCA10059', 41: 'DR_DEV', 42: 'M5STACK', 43: 'HELTEC_V3', 44: 'HELTEC_WSL_V3', 45: 'BETAFPV_2400_TX', 46: 'BETAFPV_900_NANO_TX', 47: 'RPI_PICO', 48: 'HELTEC_WIRELESS_TRACKER', 49: 'HELTEC_WIRELESS_PAPER', 50: 'T_DECK', 51: 'T_WATCH_S3', 52: 'PICOMPUTER_S3', 53: 'HELTEC_HT62', 54: 'EBYTE_ESP32_S3', 55: 'ESP32_S3_PICO', 56: 'CHATTER_2', 57: 'HELTEC_WIRELESS_PAPER_V1_0', 58: 'HELTEC_WIRELESS_TRACKER_V1_0', 59: 'UNPHONE', 60: 'TD_LORAC', 61: 'CDEBYTE_EORA_S3', 62: 'TWC_MESH_V4', 63: 'NRF52_PROMICRO_DIY', 64: 'RADIOMASTER_900_BANDIT_NANO', 65: 'HELTEC_CAPSULE_SENSOR_V3', 66: 'HELTEC_VISION_MASTER_T190', 67: 'HELTEC_VISION_MASTER_E213', 68: 'HELTEC_VISION_MASTER_E290', 69: 'HELTEC_MESH_NODE_T114', 70: 'SENSECAP_INDICATOR', 71: 'TRACKER_T1000_E', 72: 'RAK3172', 73: 'WIO_E5', 74: 'RADIOMASTER_900_BANDIT', 75: 'ME25LS01_4Y10TD', 76: 'RP2040_FEATHER_RFM95', 77: 'M5STACK_COREBASIC', 78: 'M5STACK_CORE2', 79: 'RPI_PICO2', 80: 'M5STACK_CORES3', 81: 'SEEED_XIAO_S3', 82: 'MS24SF1', 83: 'TLORA_C6', 84: 'WISMESH_TAP', 85: 'ROUTASTIC', 86: 'MESH_TAB', 87: 'MESHLINK', 88: 'XIAO_NRF52_KIT', 89: 'THINKNODE_M1', 90: 'THINKNODE_M2', 91: 'T_ETH_ELITE', 92: 'HELTEC_SENSOR_HUB', 93: 'MUZI_BASE', 94: 'HELTEC_MESH_POCKET', 95: 'SEEED_SOLAR_NODE', 96: 'NOMADSTAR_METEOR_PRO', 97: 'CROWPANEL', 98: 'LINK_32', 99: 'SEEED_WIO_TRACKER_L1', 100: 'SEEED_WIO_TRACKER_L1_EINK', 101: 'MUZI_R1_NEO', 102: 'T_DECK_PRO', 103: 'T_LORA_PAGER', 104: 'M5STACK_RESERVED', 105: 'WISMESH_TAG', 106: 'RAK3312', 107: 'THINKNODE_M5', 108: 'HELTEC_MESH_SOLAR', 109: 'T_ECHO_LITE', 110: 'HELTEC_V4', 111: 'M5STACK_C6L', 112: 'M5STACK_CARDPUTER_ADV', 113: 'HELTEC_WIRELESS_TRACKER_V2', 114: 'T_WATCH_ULTRA', 115: 'THINKNODE_M3', 116: 'WISMESH_TAP_V2', 117: 'RAK3401', 118: 'RAK6421', 119: 'THINKNODE_M4', 120: 'THINKNODE_M6', 121: 'MESHSTICK_1262', 122: 'TBEAM_1_WATT', 123: 'T5_S3_EPAPER_PRO', 124: 'TBEAM_BPF', 125: 'MINI_EPAPER_S3', 126: 'TDISPLAY_S3_PRO', 127: 'HELTEC_MESH_NODE_T096', 128: 'TRACKER_T1000_E_PRO', 129: 'THINKNODE_M7', 130: 'THINKNODE_M8', 131: 'THINKNODE_M9', 132: 'HELTEC_V4_R8', 133: 'HELTEC_MESH_NODE_T1', 134: 'STATION_G3', 135: 'T_IMPULSE_PLUS', 136: 'T_ECHO_CARD', 137: 'SEEED_WIO_TRACKER_L2', 138: 'CROWPANEL_P4', 139: 'HELTEC_MESH_TOWER_V2', 140: 'MESHNOLOGY_W10', 255: 'PRIVATE_HW'}, 'Role': {0: 'CLIENT', 1: 'CLIENT_MUTE', 2: 'ROUTER', 3: 'ROUTER_CLIENT', 4: 'REPEATER', 5: 'TRACKER', 6: 'SENSOR', 7: 'TAK', 8: 'CLIENT_HIDDEN', 9: 'LOST_AND_FOUND', 10: 'TAK_TRACKER', 11: 'ROUTER_LATE', 12: 'CLIENT_BASE'}, 'Error': {0: 'NONE', 1: 'NO_ROUTE', 2: 'GOT_NAK', 3: 'TIMEOUT', 4: 'NO_INTERFACE', 5: 'MAX_RETRANSMIT', 6: 'NO_CHANNEL', 7: 'TOO_LARGE', 8: 'NO_RESPONSE', 9: 'DUTY_CYCLE_LIMIT', 32: 'BAD_REQUEST', 33: 'NOT_AUTHORIZED', 34: 'PKI_FAILED', 35: 'PKI_UNKNOWN_PUBKEY', 36: 'ADMIN_BAD_SESSION_KEY', 37: 'ADMIN_PUBLIC_KEY_UNAUTHORIZED', 38: 'RATE_LIMIT_EXCEEDED', 39: 'PKI_SEND_FAIL_PUBLIC_KEY'}}


# =====================================================================================================================
# AES-128 / AES-256 (encrypt direction only - all the modes used here need nothing else), CTR, CMAC
# =====================================================================================================================
def _build_sbox():
    sbox = [0] * 256
    p = q = 1
    while True:
        p = (p ^ ((p << 1) & 0xFF) ^ (0x1B if p & 0x80 else 0)) & 0xFF            # p *= 3
        q ^= (q << 1) & 0xFF
        q ^= (q << 2) & 0xFF
        q ^= (q << 4) & 0xFF
        if q & 0x80:
            q ^= 0x09
        q &= 0xFF
        rot = lambda v, n: ((v << n) | (v >> (8 - n))) & 0xFF
        sbox[p] = q ^ rot(q, 1) ^ rot(q, 2) ^ rot(q, 3) ^ rot(q, 4) ^ 0x63
        if p == 1:
            break
    sbox[0] = 0x63
    return sbox

_SBOX = _build_sbox()


def _xt(a):
    return ((a << 1) ^ 0x1B) & 0xFF if a & 0x80 else a << 1


class AES:
    def __init__(self, key):
        key = bytes(key)
        if len(key) not in (16, 32):
            raise ValueError("AES key must be 16 or 32 bytes")
        nk = len(key) // 4
        self.nr = nk + 6
        w = [list(key[4 * i:4 * i + 4]) for i in range(nk)]
        rcon = 1
        for i in range(nk, 4 * (self.nr + 1)):
            t = list(w[i - 1])
            if i % nk == 0:
                t = t[1:] + t[:1]
                t = [_SBOX[b] for b in t]
                t[0] ^= rcon
                rcon = _xt(rcon)
            elif nk > 6 and i % nk == 4:
                t = [_SBOX[b] for b in t]
            w.append([a ^ b for a, b in zip(w[i - nk], t)])
        self.rk = [sum((w[4 * r + c] for c in range(4)), []) for r in range(self.nr + 1)]

    def encrypt(self, block):
        s = [b ^ k for b, k in zip(block, self.rk[0])]
        for r in range(1, self.nr + 1):
            s = [_SBOX[b] for b in s]
            s = [s[(i % 4) + 4 * (((i // 4) + (i % 4)) % 4)] for i in range(16)]       # ShiftRows
            if r != self.nr:
                out = []
                for c in range(4):
                    a0, a1, a2, a3 = s[4 * c:4 * c + 4]
                    t = a0 ^ a1 ^ a2 ^ a3
                    out += [a0 ^ t ^ _xt(a0 ^ a1), a1 ^ t ^ _xt(a1 ^ a2), a2 ^ t ^ _xt(a2 ^ a3), a3 ^ t ^ _xt(a3 ^ a0)]
                s = out
            s = [b ^ k for b, k in zip(s, self.rk[r])]
        return bytes(s)


def aes_ctr(key, counter_block, data):
    """AES-CTR; the LAST 4 bytes of the counter block are a big-endian counter (what Meshtastic's firmware uses)."""
    aes = AES(key)
    ctr = bytearray(counter_block)
    out = bytearray()
    for i in range(0, len(data), 16):
        ks = aes.encrypt(bytes(ctr))
        out += bytes(a ^ b for a, b in zip(data[i:i + 16], ks))
        n = (int.from_bytes(ctr[12:], "big") + 1) & 0xFFFFFFFF
        ctr[12:] = n.to_bytes(4, "big")
    return bytes(out)


def aes_cmac(key, msg):
    aes = AES(key)
    def dbl(b):
        v = int.from_bytes(b, "big") << 1
        v = (v ^ 0x87) if v >> 128 else v
        return (v & ((1 << 128) - 1)).to_bytes(16, "big")
    L = aes.encrypt(bytes(16))
    k1 = dbl(L)
    k2 = dbl(k1)
    n = max(1, (len(msg) + 15) // 16)
    complete = len(msg) > 0 and len(msg) % 16 == 0
    last = msg[16 * (n - 1):]
    if complete:
        last = bytes(a ^ b for a, b in zip(last, k1))
    else:
        last = last + b"\x80" + bytes(15 - len(last))
        last = bytes(a ^ b for a, b in zip(last, k2))
    x = bytes(16)
    for i in range(n - 1):
        x = aes.encrypt(bytes(a ^ b for a, b in zip(x, msg[16 * i:16 * i + 16])))
    return aes.encrypt(bytes(a ^ b for a, b in zip(x, last)))


# =====================================================================================================================
# protobuf wire decoder driven by the generated Meshtastic tables
# =====================================================================================================================
def pb_parse(data):
    """-> list of (field_no, wire_type, value). Raises ValueError if malformed."""
    i, n, out = 0, len(data), []
    def varint():
        nonlocal i
        shift = v = 0
        while True:
            if i >= n or shift > 63:
                raise ValueError("bad varint")
            b = data[i]
            i += 1
            v |= (b & 0x7F) << shift
            if not b & 0x80:
                return v
            shift += 7
    while i < n:
        tag = varint()
        fno, wt = tag >> 3, tag & 7
        if fno == 0:
            raise ValueError("field 0")
        if wt == 0:
            out.append((fno, 0, varint()))
        elif wt == 1:
            if i + 8 > n: raise ValueError("short fixed64")
            out.append((fno, 1, data[i:i + 8])); i += 8
        elif wt == 2:
            ln = varint()
            if i + ln > n: raise ValueError("short length-delimited")
            out.append((fno, 2, data[i:i + ln])); i += ln
        elif wt == 5:
            if i + 4 > n: raise ValueError("short fixed32")
            out.append((fno, 5, data[i:i + 4])); i += 4
        else:
            raise ValueError("unsupported wire type %d" % wt)
    return out


def _signed(v, bits):
    v &= (1 << 64) - 1
    if v >= (1 << 63):
        v -= 1 << 64
    return v


def pb_value(ftype, wt, val):
    """Convert a raw wire value to a Python value according to the declared field type."""
    try:
        if wt == 0:
            if ftype == "bool": return bool(val)
            if ftype == "sint": return (val >> 1) ^ -(val & 1)
            if ftype == "int": return _signed(val, 64)
            return val
        if wt == 5:
            if ftype == "float": return struct.unpack("<f", val)[0]
            if ftype == "sfixed32": return struct.unpack("<i", val)[0]
            return struct.unpack("<I", val)[0]
        if wt == 1:
            if ftype == "double": return struct.unpack("<d", val)[0]
            if ftype == "sfixed64": return struct.unpack("<q", val)[0]
            return struct.unpack("<Q", val)[0]
    except Exception:
        pass
    return val


def pb_decode_msg(name, data, depth=0):
    """-> list of (field_name, python_value) with nested messages as ('name', [..]); unknown fields as ('field N', raw)."""
    fields = MT_MSG.get(name, {})
    out = []
    for fno, wt, raw in pb_parse(data):
        if fno in fields:
            fname, ftype, rep = fields[fno]
            if ftype.startswith("msg:") and wt == 2:
                try:
                    out.append((fname, pb_decode_msg(ftype[4:], raw, depth + 1)))
                except ValueError:
                    out.append((fname, raw))
            elif ftype == "string" and wt == 2:
                out.append((fname, raw.decode("utf-8", "replace")))
            elif ftype == "bytes" and wt == 2:
                out.append((fname, bytes(raw)))
            elif ftype.startswith("enum:") and wt == 0:
                out.append((fname, MT_ENUM.get(ftype[5:], {}).get(raw, f"{raw}")))
            elif wt == 2:                                    # packed repeated scalars: leave raw
                out.append((fname, bytes(raw)))
            else:
                out.append((fname, pb_value(ftype, wt, raw)))
        else:
            out.append((f"field {fno}", raw if wt in (2,) else pb_value("uint", wt, raw)))
    return out


def _fmt_val(v):
    if isinstance(v, (bytes, bytearray)):
        return v.hex() if len(v) <= 24 else v[:24].hex() + f"...({len(v)} B)"
    if isinstance(v, float):
        return f"{v:.6g}"
    return str(v)


def pb_lines(items, indent=0):
    pad = "  " * indent
    scal = [(k, v) for k, v in items if not isinstance(v, list)]
    nest = [(k, v) for k, v in items if isinstance(v, list)]
    lines = []
    if scal:
        lines.append(pad + "  ".join(f"{k}={_fmt_val(v)}" for k, v in scal))
    for k, v in nest:
        lines.append(f"{pad}{k}:")
        lines += pb_lines(v, indent + 1)
    return lines


# =====================================================================================================================
# Meshtastic
# =====================================================================================================================
MT_DEFAULT_PSK = bytes([0xd4, 0xf1, 0xbb, 0x3a, 0x20, 0x29, 0x07, 0x59, 0xf0, 0xbc, 0xff, 0xab, 0xcf, 0x4e, 0x69, 0x01])
MT_PRESET_NAMES = ["LongFast", "LongSlow", "LongModerate", "LongTurbo", "MediumFast", "MediumSlow", "ShortFast", "ShortSlow", "ShortTurbo",
                   "VeryLongSlow"]


def mt_expand_key(spec):
    """Channel key as the user types it: base64 or hex. 1 byte = default-key index (AQ== = 1 = the public default key)."""
    s = spec.strip()
    if not s:
        return None
    try:
        raw = bytes.fromhex(s.replace(" ", "").replace(":", "")) if all(c in "0123456789abcdefABCDEF :" for c in s) and len(s.replace(" ", "")) % 2 == 0 \
            and len(s.replace(" ", "").replace(":", "")) in (2, 32, 64) else base64.b64decode(s + "=" * (-len(s) % 4))
    except Exception:
        return None
    if len(raw) == 1:
        if raw[0] == 0:
            return None                                          # index 0 = no encryption
        k = bytearray(MT_DEFAULT_PSK)
        k[15] = (k[15] + raw[0] - 1) & 0xFF
        return bytes(k)
    if len(raw) < 16:
        return raw + bytes(16 - len(raw))
    if len(raw) in (16, 32):
        return raw
    if len(raw) < 32:
        return raw + bytes(32 - len(raw))
    return raw[:32]


def mt_channel_hash(name, key):
    h = 0
    for b in name.encode():
        h ^= b
    for b in key:
        h ^= b
    return h


def mt_hex_node(n):
    return "^all" if n == 0xFFFFFFFF else f"!{n:08x}"


def parse_meshtastic(payload, key_specs=("AQ==",), channel_names=()):
    """Return dict(ok, header(dict), lines(list[str]), summary(str))."""
    p = bytes(payload)
    if len(p) < 16:
        return {"ok": False, "summary": "too short for a Meshtastic header", "lines": []}
    to, frm, pid = struct.unpack("<III", p[:12])
    flags, ch, nh, rn = p[12], p[13], p[14], p[15]
    hdr = {"to": to, "from": frm, "id": pid, "hop_limit": flags & 7, "want_ack": bool(flags & 8), "via_mqtt": bool(flags & 16),
           "hop_start": flags >> 5, "channel": ch, "next_hop": nh, "relay_node": rn}
    body = p[16:]
    lines = [f"Meshtastic  to={mt_hex_node(to)}  from={mt_hex_node(frm)}  id=0x{pid:08x}  hops={hdr['hop_limit']}/{hdr['hop_start']}"
             f"  want_ack={int(hdr['want_ack'])}  via_mqtt={int(hdr['via_mqtt'])}  channel_hash=0x{ch:02x}  next_hop=0x{nh:02x}  relay=0x{rn:02x}"]
    keys = []
    for spec in key_specs:
        k = mt_expand_key(spec)
        if k and k not in [x[1] for x in keys]:
            keys.append((spec, k))
    for idx in range(1, 11):                                       # the public default key and its nine documented variants
        k = bytearray(MT_DEFAULT_PSK); k[15] = (k[15] + idx - 1) & 0xFF
        if bytes(k) not in [x[1] for x in keys]:
            keys.append((f"default#{idx}", bytes(k)))
    names = list(channel_names) + MT_PRESET_NAMES
    cand = []
    for spec, k in keys:                                           # keys whose channel hash matches the packet come first
        match = [n for n in names if mt_channel_hash(n, k) == ch]
        cand.append((0 if match else 1, spec, k, match))
    cand.sort(key=lambda c: c[0])
    if ch == 0 and len(body) > 12:
        lines.append("  channel hash 0 + 12 extra bytes: PKI direct message (Curve25519 + AES-CCM) - cannot be decrypted without the node keys")
        lines.append("  encrypted: " + body.hex())
        return {"ok": True, "header": hdr, "lines": lines, "summary": f"Meshtastic PKI DM {mt_hex_node(frm)}->{mt_hex_node(to)}"}
    nonce = struct.pack("<Q", pid) + struct.pack("<I", frm) + bytes(4)
    for _, spec, k, match in cand:
        plain = aes_ctr(k, nonce, body)
        try:
            items = pb_parse(plain)
        except ValueError:
            continue
        d = {f: (w, v) for f, w, v in items}
        port = d.get(1, (0, None))[1]
        if 1 not in d or d[1][0] != 0 or port is None or port not in MT_ENUM["PortNum"] and port > 511:
            continue
        data = pb_decode_msg("Data", plain)
        pname = MT_ENUM["PortNum"].get(port, str(port))
        hdr["port"] = pname
        kdesc = (f"key {spec}" + (f" (hash matches channel '{match[0]}')" if match else ""))
        lines.append(f"  decrypted with {kdesc}:  port={pname}")
        pay = next((v for n_, v in data if n_ == "payload"), b"")
        others = [(n_, v) for n_, v in data if n_ not in ("portnum", "payload")]
        if others:
            lines.append("  " + "  ".join(f"{n_}={_fmt_val(v)}" for n_, v in others))
        summary = f"Meshtastic {pname}"
        sub = {"POSITION_APP": "Position", "NODEINFO_APP": "User", "TELEMETRY_APP": "Telemetry", "ROUTING_APP": "Routing",
               "WAYPOINT_APP": "Waypoint", "NEIGHBORINFO_APP": "NeighborInfo", "TRACEROUTE_APP": "RouteDiscovery"}.get(pname)
        if pname in ("TEXT_MESSAGE_APP", "ALERT_APP"):
            txt = pay.decode("utf-8", "replace")
            lines.append(f"  text: {txt!r}")
            summary += f" {txt!r}"
        elif pname == "TEXT_MESSAGE_COMPRESSED_APP":
            lines.append("  compressed text (Unishox2): " + pay.hex())
        elif sub:
            try:
                dec = pb_decode_msg(sub, pay)
                if sub == "Position":
                    dec = _pos_friendly(dec)
                lines += pb_lines(dec, 2)
                if sub == "User":
                    nm = next((v for n_, v in dec if n_ == "long_name"), None)
                    if nm: summary += f" {nm!r}"
            except ValueError:
                lines.append("  payload (could not parse as " + sub + "): " + pay.hex())
        else:
            lines.append("  payload: " + (pay.hex() if len(pay) <= 64 else pay[:64].hex() + f"...({len(pay)} B)"))
        return {"ok": True, "header": hdr, "lines": lines, "summary": summary}
    lines.append(f"  payload {len(body)} B: could not decrypt with the available keys (hash 0x{ch:02x}) - add the channel key under Keys...")
    lines.append("  encrypted: " + (body.hex() if len(body) <= 80 else body[:80].hex() + "..."))
    return {"ok": True, "header": hdr, "lines": lines, "summary": f"Meshtastic {mt_hex_node(frm)} (encrypted)"}


def _pos_friendly(items):
    out = []
    for n_, v in items:
        if n_ == "latitude_i" and isinstance(v, int):
            out.append(("latitude", round(v * 1e-7, 7)))
        elif n_ == "longitude_i" and isinstance(v, int):
            out.append(("longitude", round(v * 1e-7, 7)))
        else:
            out.append((n_, v))
    return out


# =====================================================================================================================
# LoRaWAN
# =====================================================================================================================
LW_MTYPE = ["JoinRequest", "JoinAccept", "UnconfirmedDataUp", "UnconfirmedDataDown", "ConfirmedDataUp", "ConfirmedDataDown",
            "RejoinRequest", "Proprietary"]
LW_MAC_UP = {0x01: ("ResetInd", 1), 0x02: ("LinkCheckReq", 0), 0x03: ("LinkADRAns", 1), 0x04: ("DutyCycleAns", 0), 0x05: ("RXParamSetupAns", 1),
             0x06: ("DevStatusAns", 2), 0x07: ("NewChannelAns", 1), 0x08: ("RXTimingSetupAns", 0), 0x09: ("TxParamSetupAns", 0),
             0x0A: ("DlChannelAns", 1), 0x0B: ("RekeyInd", 1), 0x0C: ("ADRParamSetupAns", 0), 0x0D: ("DeviceTimeReq", 0),
             0x0E: ("RejoinParamSetupAns", 1)}
LW_MAC_DOWN = {0x01: ("ResetConf", 1), 0x02: ("LinkCheckAns", 2), 0x03: ("LinkADRReq", 4), 0x04: ("DutyCycleReq", 1), 0x05: ("RXParamSetupReq", 4),
               0x06: ("DevStatusReq", 0), 0x07: ("NewChannelReq", 5), 0x08: ("RXTimingSetupReq", 1), 0x09: ("TxParamSetupReq", 1),
               0x0A: ("DlChannelReq", 4), 0x0B: ("RekeyConf", 1), 0x0C: ("ADRParamSetupReq", 1), 0x0D: ("DeviceTimeAns", 5),
               0x0E: ("ForceRejoinReq", 2), 0x0F: ("RejoinParamSetupReq", 1)}


def _eui(b):
    return bytes(reversed(b)).hex().upper()


def lw_looks_valid(p):
    if len(p) < 12:
        return False
    mhdr = p[0]
    mt, major = mhdr >> 5, mhdr & 3
    if major != 0:
        return False
    if mt == 0:
        return len(p) == 23
    if mt == 1:
        return len(p) in (17, 33)
    if mt in (2, 3, 4, 5):
        fopts = p[5] & 0x0F
        return 1 + 7 + fopts <= len(p) - 4 + 0
    return mt in (6, 7)


def _lw_mac(cmds, up):
    table = LW_MAC_UP if up else LW_MAC_DOWN
    out, i = [], 0
    while i < len(cmds):
        cid = cmds[i]
        name, ln = table.get(cid, (f"CID 0x{cid:02X}", None))
        if ln is None or i + 1 + ln > len(cmds):
            out.append(f"{name}(+{cmds[i + 1:].hex()})")
            break
        out.append(f"{name}" + (f"[{cmds[i + 1:i + 1 + ln].hex()}]" if ln else ""))
        i += 1 + ln
    return out


def parse_lorawan(payload, nwk_skey=None, app_skey=None, app_key=None):
    p = bytes(payload)
    if len(p) < 5:
        return {"ok": False, "summary": "too short", "lines": []}
    mhdr = p[0]
    mt, major = mhdr >> 5, mhdr & 3
    lines = [f"LoRaWAN  {LW_MTYPE[mt]}  major={major}  (MHDR 0x{mhdr:02x})"]
    summary = f"LoRaWAN {LW_MTYPE[mt]}"
    mic = p[-4:]
    if mt == 0 and len(p) == 23:
        app_eui, dev_eui, nonce = _eui(p[1:9]), _eui(p[9:17]), struct.unpack("<H", p[17:19])[0]
        lines.append(f"  JoinEUI/AppEUI={app_eui}  DevEUI={dev_eui}  DevNonce=0x{nonce:04x}  MIC={mic.hex()}")
        if app_key:
            ok = aes_cmac(app_key, p[:-4])[:4] == mic
            lines.append(f"  MIC {'VALID with the given AppKey' if ok else 'does not match the given AppKey'}")
        summary += f" DevEUI {dev_eui}"
    elif mt == 1:
        lines.append(f"  encrypted JoinAccept ({len(p) - 1} B): {p[1:].hex()}")
        if app_key:
            aes = AES(app_key)
            dec = b"".join(aes.encrypt(p[1 + i:17 + i]) for i in range(0, len(p) - 1, 16))
            ok = aes_cmac(app_key, bytes([mhdr]) + dec[:-4])[:4] == dec[-4:]
            lines.append(f"  decrypted: AppNonce={dec[0:3][::-1].hex()}  NetID={dec[3:6][::-1].hex()}  DevAddr={dec[6:10][::-1].hex().upper()}"
                         f"  DLSettings=0x{dec[10]:02x}  RxDelay={dec[11] & 15}s" + (f"  CFList={dec[12:28].hex()}" if len(dec) == 32 else "")
                         + f"  MIC {'VALID' if ok else 'INVALID'}")
    elif mt in (2, 3, 4, 5) and len(p) >= 12:
        up = mt in (2, 4)
        dev_addr = struct.unpack("<I", p[1:5])[0]
        fctrl = p[5]
        fcnt = struct.unpack("<H", p[6:8])[0]
        fopts_len = fctrl & 0x0F
        fopts = p[8:8 + fopts_len]
        rest = p[8 + fopts_len:-4]
        fport = rest[0] if rest else None
        frm = rest[1:] if rest else b""
        if up:
            fl = f"ADR={fctrl >> 7 & 1} ADRACKReq={fctrl >> 6 & 1} ACK={fctrl >> 5 & 1} ClassB={fctrl >> 4 & 1}"
        else:
            fl = f"ADR={fctrl >> 7 & 1} ACK={fctrl >> 5 & 1} FPending={fctrl >> 4 & 1}"
        lines.append(f"  DevAddr={dev_addr:08X}  FCtrl=0x{fctrl:02x} ({fl} FOptsLen={fopts_len})  FCnt={fcnt}"
                     f"  FPort={fport if fport is not None else '-'}  FRMPayload {len(frm)} B  MIC={mic.hex()}")
        if fopts:
            lines.append("  FOpts MAC commands: " + ", ".join(_lw_mac(fopts, up)))
        if nwk_skey:
            dirb = 0 if up else 1
            b0 = b"\x49" + bytes(4) + bytes([dirb]) + struct.pack("<I", dev_addr) + struct.pack("<I", fcnt) + b"\x00" + bytes([len(p) - 4])
            calc = aes_cmac(nwk_skey, b0 + p[:-4])[:4]
            lines.append(f"  MIC {'VALID' if calc == mic else 'does not match'} with the given NwkSKey (FCnt assumed < 65536)")
        key = (nwk_skey if fport == 0 else app_skey)
        if frm and key:
            dirb = 0 if up else 1
            out = bytearray()
            for i in range(0, len(frm), 16):
                a = b"\x01" + bytes(4) + bytes([dirb]) + struct.pack("<I", dev_addr) + struct.pack("<I", fcnt) + b"\x00" + bytes([i // 16 + 1])
                s = AES(key).encrypt(a)
                out += bytes(x ^ y for x, y in zip(frm[i:i + 16], s))
            dec = bytes(out)
            if fport == 0:
                lines.append("  MAC commands (FPort 0): " + ", ".join(_lw_mac(dec, up)))
            else:
                try:
                    txt = dec.decode("ascii")
                    printable = all(32 <= ord(c) < 127 for c in txt)
                except UnicodeDecodeError:
                    printable = False
                lines.append(f"  decrypted payload: {dec.hex()}" + (f"  ({dec.decode('ascii')!r})" if printable else ""))
        elif frm:
            lines.append(f"  payload (encrypted): {frm.hex()}" + ("  - give the "
                         + ("NwkSKey" if fport == 0 else "AppSKey") + " under Keys... to decrypt" if True else ""))
        summary += f" DevAddr {dev_addr:08X} FCnt {fcnt}" + (f" port {fport}" if fport is not None else "")
    elif mt == 6:
        lines.append("  RejoinRequest: " + p[1:].hex())
    else:
        lines.append("  proprietary / unrecognised: " + p[1:].hex())
    return {"ok": True, "lines": lines, "summary": summary}

MT_REGIONS = {'US': (902.0, 928.0, 0.0), 'EU_433': (433.0, 434.0, 0.0), 'EU_868': (869.4, 869.65, 0.0), 'CN': (470.0, 510.0, 0.0), 'JP': (920.5, 923.5, 0.0), 'ANZ': (915.0, 928.0, 0.0), 'ANZ_433': (433.05, 434.79, 0.0), 'RU': (868.7, 869.2, 0.0), 'KR': (920.0, 923.0, 0.0), 'TW': (920.0, 925.0, 0.0), 'IN': (865.0, 867.0, 0.0), 'NZ_865': (864.0, 868.0, 0.0), 'TH': (920.0, 925.0, 0.0), 'UA_433': (433.0, 434.7, 0.0), 'UA_868': (868.0, 868.6, 0.0), 'MY_433': (433.0, 435.0, 0.0), 'MY_919': (919.0, 924.0, 0.0), 'SG_923': (917.0, 925.0, 0.0), 'PH_433': (433.0, 434.7, 0.0), 'PH_868': (868.0, 869.4, 0.0), 'PH_915': (915.0, 918.0, 0.0), 'KZ_433': (433.075, 434.775, 0.0), 'KZ_863': (863.0, 868.0, 0.0), 'NP_865': (865.0, 868.0, 0.0), 'BR_902': (902.0, 907.5, 0.0), 'LORA_24': (2400.0, 2483.5, 0.0), 'UNSET': (902.0, 928.0, 0.0)}


# =====================================================================================================================
# LoRa presets, Meshtastic default-frequency calculation (the firmware's own rule) and the frame describer
# =====================================================================================================================
# modem presets from the Meshtastic firmware (MeshRadio.h): display name -> (SF, bandwidth kHz, coding rate denominator)
MT_MODEM_PRESETS = {"LongFast": (11, 250, 5), "LongSlow": (12, 125, 8), "LongModerate": (11, 125, 8), "LongTurbo": (11, 500, 8),
                    "MediumFast": (9, 250, 5), "MediumSlow": (10, 250, 5), "ShortFast": (7, 250, 5), "ShortSlow": (8, 250, 5),
                    "ShortTurbo": (7, 500, 5)}


def mt_djb2(s):
    h = 5381
    for c in s.encode():
        h = (h * 33 + c) & 0xFFFFFFFF
    return h


def mt_default_freq(region, preset_name, bw_khz=None):
    """Default channel frequency in MHz for a Meshtastic modem preset in a region (firmware: numChannels = floor(span / bw),
    slot = djb2(preset name) % numChannels, f = start + bw/2 + slot * bw). None if the region is unknown."""
    if region not in MT_REGIONS or preset_name not in MT_MODEM_PRESETS:
        return None
    start, end, spacing = MT_REGIONS[region]
    bw = float(bw_khz if bw_khz else MT_MODEM_PRESETS[preset_name][1])
    n = int((end - start) // (spacing + bw / 1000.0))
    if n < 1:
        return None
    slot = mt_djb2(preset_name) % n
    return start + bw / 2000.0 + slot * bw / 1000.0


# (label, settings) - settings: sf, bw (Hz), sync (None = any), parse, freq (MHz) or mt=<preset name> (frequency from the Region box), invert
LORA_PRESETS = [("Custom - use the SF / BW / sync boxes", None)]
for _n, (_sf, _bw, _cr) in MT_MODEM_PRESETS.items():
    LORA_PRESETS.append((f"Meshtastic {_n}  SF{_sf} / {_bw} kHz / CR 4/{_cr}",
                         dict(sf=_sf, bw=_bw * 1e3, sync=0x2B, parse="mesh", mt=_n)))
LORA_PRESETS += [
    ("LoRaWAN EU868  SF7 / 125 kHz   868.1 MHz", dict(sf=7, bw=125e3, sync=0x34, parse="lorawan", freq=868.1)),
    ("LoRaWAN EU868  SF9 / 125 kHz   868.3 MHz", dict(sf=9, bw=125e3, sync=0x34, parse="lorawan", freq=868.3)),
    ("LoRaWAN EU868  SF12 / 125 kHz  868.5 MHz", dict(sf=12, bw=125e3, sync=0x34, parse="lorawan", freq=868.5)),
    ("LoRaWAN EU868  RX2 downlink SF12 / 125 kHz  869.525 MHz (inverted IQ)",
     dict(sf=12, bw=125e3, sync=0x34, parse="lorawan", freq=869.525, invert=True)),
    ("LoRaWAN EU433  SF7 / 125 kHz   433.175 MHz", dict(sf=7, bw=125e3, sync=0x34, parse="lorawan", freq=433.175)),
    ("LoRaWAN US915  SF7 / 125 kHz   902.3 MHz", dict(sf=7, bw=125e3, sync=0x34, parse="lorawan", freq=902.3)),
    ("LoRaWAN US915  SF8 / 500 kHz   903.0 MHz", dict(sf=8, bw=500e3, sync=0x34, parse="lorawan", freq=903.0)),
    ("LoRaWAN AS923  SF7 / 125 kHz   923.2 MHz", dict(sf=7, bw=125e3, sync=0x34, parse="lorawan", freq=923.2)),
    ("LoRaWAN IN865  SF7 / 125 kHz   865.0625 MHz", dict(sf=7, bw=125e3, sync=0x34, parse="lorawan", freq=865.0625)),
    ("Generic LoRa, private sync 0x12  SF7 / 125 kHz", dict(sf=7, bw=125e3, sync=0x12, parse="raw")),
]
LORA_SYNC_CHOICES = [("Any sync word", None), ("0x2B  Meshtastic", 0x2B), ("0x34  LoRaWAN public", 0x34), ("0x12  private / default", 0x12)]
LORA_PARSE_CHOICES = [("Auto", "auto"), ("Meshtastic", "mesh"), ("LoRaWAN", "lorawan"), ("Raw hex only", "raw")]


def lora_hex_lines(b, per=32):
    return ["  hex: " + b[i:i + per].hex(" ") if i == 0 else "       " + b[i:i + per].hex(" ") for i in range(0, max(len(b), 1), per)]


def lora_describe(f, parse_mode="auto", cfg=None):
    """Decoded PHY frame -> list of text lines. First line = physical layer; then hex; then the parsed protocol layer."""
    cfg = cfg or {}
    T = (1 << f["sf"]) / f["bw"]
    crc = "CRC OK" if f["crc_ok"] else ("CRC FAIL" if f["crc_ok"] is False else "no CRC")
    lines = [f"LoRa SF{f['sf']}/BW{f['bw'] / 1e3:g} CR4/{4 + f['cr']}  len {f['length']}  {crc}  SNR {f['snr_db']:+.1f} dB  "
             f"CFO {f['cfo_hz'] / 1e3:+.2f} kHz  sync " + (f"0x{f['sync']:02X}" if f.get("sync") is not None else "?")
             + f"  header+payload {f['n_symbols']} sym = {f['n_symbols'] * T * 1e3:.0f} ms" + ("  LDRO" if f.get("ldro") else "")]
    p = f["payload"]
    lines += lora_hex_lines(p)
    if f["crc_ok"] is False:
        lines.append("  CRC failed: payload may be corrupt - not parsed")
        return lines
    mode = parse_mode
    sync = f.get("sync")
    if mode == "auto":
        if sync == 0x2B:
            mode = "mesh"
        elif sync == 0x34 or (lw_looks_valid(p) and sync != 0x2B):
            mode = "lorawan" if lw_looks_valid(p) else "mesh" if len(p) >= 17 else "raw"
        else:
            mode = "mesh" if len(p) >= 17 and not lw_looks_valid(p) else "raw"
    try:
        if mode == "mesh":
            r = parse_meshtastic(p, key_specs=tuple(cfg.get("mt_keys") or ("AQ==",)), channel_names=tuple(cfg.get("mt_names") or ()))
            lines += r["lines"]
        elif mode == "lorawan":
            r = parse_lorawan(p, cfg.get("nwk"), cfg.get("app"), cfg.get("appkey"))
            lines += r["lines"] if r["lines"] else ["  not a valid LoRaWAN PHYPayload"]
    except Exception as e:                                         # a parser must never take the decoder down
        lines.append(f"  parser error: {e!r}")
    return lines


class SurveySignals(QtCore.QObject):
    # survey worker thread -> GUI thread: True if the survey was stopped early
    done = QtCore.pyqtSignal(object)
    # survey worker thread -> GUI thread: a new rate/gain test has started, show its settings on the controls
    progress = QtCore.pyqtSignal()

# ---- CAL SURVEY defaults (see EmcScanner.cal_open_dialog) ----
# A calibration survey feeds a KNOWN sine into the SDR and measures how the displayed level of that tone
# changes with sample rate and RF gain, so CAL_SDR_DB_TO_DBUV can be checked against the whole range.
CAL_TONE_HZ_DEFAULT = 1.5e6        # frequency of the test tone
CAL_TONE_VPP_DEFAULT = 0.11        # its amplitude (V pk-pk) at the generator output / attenuator input
CAL_WINDOW_HZ_DEFAULT = 25e3       # the tone is searched for within +/- this of its nominal frequency
CAL_OFFSET_KHZ_DEFAULT = "200"     # tone position above the block centre (kHz); a comma list maps the passband
CAL_SCANS_DEFAULT = 2              # scans per setting (the median is reported)
CAL_LOBE_BINS = 3                  # bins either side of the peak summed to get the scalloping-free tone level
CAL_MAX_OFFSET_FRAC = 0.40         # offsets beyond this fraction of the sample rate from the centre are skipped
CAL_AGREE_DB = 1.0                 # sample rates "agree" at a gain if their levels are within this
CAL_DEPART_DB = 1.0                # a rate "departs" at the first gain where it is this far from its low-gain level
CAL_SPREAD_WARN_DB = 0.5           # repeat scans of one setting differing by more than this are flagged
CAL_QUICK_TONE_HZ = 1.5e6          # the CAL button's test tone
CAL_QUICK_VPP_DEFAULT = 2.85        # CAL button: generator amplitude offered first (V pk-pk, as measured at the generator output)
CAL_QUICK_RSERIES_DEFAULT = 10000.0  # CAL button: series (dropper) resistor offered first (ohm)
CAL_QUICK_SCANS = 2                # scans per tone position in the CAL button's measurement
CAL_POS_FRACTIONS = (-0.30, -0.15, 0.15, 0.30)   # tone positions in the block, as a fraction of the sample rate from the centre
CAL_POS_SPREAD_WARN_DB = 3.0       # CAL: tell the user if the level varies more than this with tone position
CAL_MIN_HEADROOM_DB = 10.0         # CAL: reject a rate whose tone is closer than this to ADC full scale
CAL_MIN_SNR_DB = 25.0              # CAL: reject a rate whose tone is less than this above the noise
CAL_WINDOW_ENBW_DB = 10.0 * math.log10(0.375 / 0.25)   # Hann: sum of power over the tone's lobe / on-bin peak power


class CalSurveySignals(QtCore.QObject):
    # calibration survey worker thread -> GUI thread
    done = QtCore.pyqtSignal(object)      # True if stopped early
    progress = QtCore.pyqtSignal()        # a new rate/gain setting has started: move the Sample Rate box / gain slider


# ---- WATERFALL defaults (see EmcScanner.wf_build_ui) ----
WF_MAX_BINS = 4096                 # a row wider than this is reduced by peak-preserving max pooling (keeps the image light)
WF_FAST_ROWS = 128                 # rows in one contiguous "fast" capture (single-frequency mode): 128 x 4096 samples
WF_DEFAULT_ROWS = 150              # history depth (rows) unless settings.csv says otherwise


class WaterfallSignals(QtCore.QObject):
    # worker thread -> GUI thread: new waterfall rows
    rows = QtCore.pyqtSignal(object, object, str, float)   # freqs (Hz), rows (n x bins, NEWEST FIRST, dBuV), kind, seconds per row



class VisibilityFilter(QtCore.QObject):
    """Event filter: calls back when the watched widget is explicitly shown or hidden (ShowToParent / HideToParent, which -
    unlike Show / Hide - are NOT sent when only a parent is shown or hidden, so there is no feedback loop)."""
    def __init__(self, callback, parent=None):
        super().__init__(parent)
        self._cb = callback

    def eventFilter(self, obj, ev):
        if ev.type() in (QtCore.QEvent.ShowToParent, QtCore.QEvent.HideToParent):
            try:
                self._cb()
            except Exception:
                pass
        return False


class ViewSignals(QtCore.QObject):
    # worker thread -> GUI thread: set the graph's X / Y range (a NaN Y range means "leave Y alone")
    apply = QtCore.pyqtSignal(float, float, float, float)


# ---- PEAK detection defaults (see EmcScanner.peak_build_ui) ----
PEAK_MIN_ABOVE_MEDIAN_DB = 6.0     # a marked peak (other than the strongest) must stand this far above the median level of the trace
PEAK_MAX_MARKERS = 10


class PeakSignals(QtCore.QObject):
    # worker thread -> GUI thread: one finished pass for peak hold / peak markers
    update = QtCore.pyqtSignal(object, object, str)      # freqs (Hz), levels (displayed units), band key


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
        # the front-panel "Cal:" label follows the sample rate in use (queued call: safe from either thread)
        lbl = getattr(self, "cispr_calibration_label", None)
        if lbl is not None:
            QtCore.QMetaObject.invokeMethod(lbl, "setText", QtCore.Qt.QueuedConnection,
                                            QtCore.Q_ARG(str, self.cal_label_text()))

    # Use this to build centers across the sweep band
    # start_hz, stop_hz, sample_rate are floats

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
        # CISPR limit type shown by the red line / right axis: "radiated" (dBuV/m @ 10 m) or
        # "conducted" (dBuV, mains port). Selected with the combo box next to 'Show CISPR limit'.
        self.cispr_mode = "radiated"
        # The correction (antenna factor / LISN loss) is remembered separately for each type.
        self._cispr_corr_by_mode = {"radiated": 0.0, "conducted": 0.0}
   
        # Effective offset actually applied to the right (CISPR) axis: OFFSET + CALIBRATION.
        self.cispr_effective_offset = 0.0
        self.input_atten_db = 0.0   # external attenuator in front of the SDR (front-panel spinbox)
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
        # Both come from settings.csv (dc_offset_removal / iq_balance_correction) and default to ON;
        # there are no tick boxes for them on the front panel.
        self.dc_removal_enabled = DC_OFFSET_REMOVAL
        self.iq_balance_enabled = IQ_BALANCE_CORRECTION

        # Label of whichever preset/MODE is currently active, shown on the stable status
        # panel (see update_status_panel()). "-" until a MODE button has been pressed.
        self.current_mode_label = "-"


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
        self.plot.setLabel('left', 'Level at input (dBµV)')

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
        self.plot.getAxis('right').setLabel(self.cispr_axis_label())
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
        # Everything is routed through the cispr_* methods below, and each signal is connected
        # exactly once, so a pan/zoom redraws the CISPR line and ticks only once.
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
        self._left_ymin = -20.0
        self._left_ymax = 120.0
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
        self.cispr_mode_combo = QtWidgets.QComboBox()
        self.cispr_mode_combo.addItem("Radiated (dBµV/m @10 m)", "radiated")
        self.cispr_mode_combo.addItem("Conducted (dBµV)", "conducted")
        self.cispr_mode_combo.setCurrentIndex(self.cispr_mode_combo.findData(self.cispr_mode))
        self.cispr_mode_combo.setToolTip("Which CISPR limit the red line and right-hand axis show.")

        self.btn_cont = QtWidgets.QPushButton("START CONTINUOUS")
        ctrl_layout.addWidget(self.btn_cont)
        self.btn_cont.clicked.connect(self.start_continuous)

        ctrl_layout.addWidget(self.btn_start)
        ctrl_layout.addWidget(self.btn_stop)
        ctrl_layout.addWidget(self.btn_save)
        ctrl_layout.addWidget(self.chk_cispr)
        ctrl_layout.addWidget(self.cispr_mode_combo)

        self.btn_start.clicked.connect(self.start_sweep)
        self.btn_stop.clicked.connect(self.stop_sweep)
        self.btn_save.clicked.connect(self.save_csv)
        self.chk_cispr.toggled.connect(self.cispr_toggle_visibility)
        self.cispr_mode_combo.currentIndexChanged.connect(self.cispr_on_mode_changed)

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
        # label and tooltip are set per limit type by cispr_refresh_corr_ui()

        # Read-only display of the current mode's dBuV_Calibration, kept in sync by
        # cispr_refresh_offset_ui() whenever set_preset() changes mode.

        self.cispr_calibration_label = QtWidgets.QLabel(f"Cal: {CAL_SDR_DB_TO_DBUV:+.1f} dB")
        self.cispr_calibration_label.setToolTip("CAL_SDR_DB_TO_DBUV - set in the code (top of the file).")

        # ---- Attenuator (external, between antenna and SDR) ----
        self.atten_label = QtWidgets.QLabel("Attenuator")
        self.atten_spin = QtWidgets.QDoubleSpinBox()
        self.atten_spin.setRange(0.0, 100.0)
        self.atten_spin.setSingleStep(1.0)
        self.atten_spin.setDecimals(1)
        self.atten_spin.setValue(self.input_atten_db)
        self.atten_spin.setSuffix(" dB")
        self.atten_spin.setToolTip("Attenuator fitted in front of the SDR. The left axis shows dBuV at the "
                                   "attenuator's input (antenna side).")
        self.atten_spin.valueChanged.connect(self.atten_on_changed)

        # place the controls in the same control row
        ctrl_layout.addWidget(self.cispr_offset_label)
        ctrl_layout.addWidget(self.cispr_offset_spin)
        ctrl_layout.addWidget(self.cispr_calibration_label)
        ctrl_layout.addWidget(self.atten_label)
        ctrl_layout.addWidget(self.atten_spin)

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

        # ---- DC Offset Removal / IQ Balance Correction: no front-panel tick boxes ----
        # Both default ON and are set from settings.csv (dc_offset_removal / iq_balance_correction).

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
        self.cispr_refresh_corr_ui()
        self.cispr_refresh_offset_ui()
        # Ensure initial visibility follows the checkbox (do not force show unconditionally)
        self.cispr_toggle_visibility(bool(getattr(self, "chk_cispr", None) and self.chk_cispr.isChecked()))
        # ---- Right-hand preset panel ----
        preset_panel = QtWidgets.QVBoxLayout()
        main_layout.addLayout(preset_panel)

        # Define presets as a list so we can index them and extend easily
        # Each entry: (internal_name, label, start_hz, stop_hz, sample_rate, gain_db, hf_mode_flag,

        # Supported gains: [0, 9, 14, 27, 37, 77, 87, 125, 144, 157, 166, 197, 207, 229, 254,
        # 280, 297, 328, 338, 364, 372, 386, 402, 421, 434, 439, 445, 480, 496] (tenths of dB -
        # e.g. 402 -> 40.2 dB). gain_db values below are requested dB; init_sdr() reads back
        # and logs/warns if the device actually applied something different - see gain_db.
        # generally avoid HF mode as it has no gain.
        # Presets come from settings.csv (see load_settings() near the top of this file;
        # the built-in list is kept there as _FALLBACK_PRESETS).
        self.presets = list(PRESETS)

        # Create buttons in a loop so adding presets is trivial
        self.mode_buttons = []
        for idx, (_, label, *_rest) in enumerate(self.presets):
            btn = QtWidgets.QPushButton(label)
            self.mode_buttons.append(btn)
            preset_panel.addWidget(btn)
            # bind the index into the lambda to avoid late-binding trap
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
        # scrolling. So: everything goes to the real console, but only
        # messages that look like an error or warning are also forwarded to the GUI window.
        # Routine diagnostics stay console-only; the debug window only gets
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
        self.layout_tidy(left_layout)      # compact front panel: 2-line message box, waterfall splitter, regrouped buttons



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

        print("Reading samples…")
        samples = self.sdr.read_samples(256)
        print("Sample[0:10]:", samples[:10])

        # DO NOT close here — this is your main SDR handle
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
        start/stop/sample-rate/gain/hf_mode as the LIVE settings that
        start_sweep(), start_continuous(), init_sdr(), sweep_loop() and the CISPR mapping
        all read (self.START_FREQ, self.STOP_FREQ, self.SAMPLE_RATE, self.gain_db,
        self.hf_mode). The SDR itself isn't reopened here -
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
            # gain_db, hf_mode_flag)
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
                     f"{self.SAMPLE_RATE/1e6:.3f} MS/s, gain {self.gain_db}, HF={self.hf_mode}")

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

 
    def cal_total_offset_db(self):
        """dB to ADD to a stored 'SDR dB' value to get the displayed level: in dBuV mode dBuV at the INPUT of the external attenuator
        (the dB -> dBuV factor of the sample rate in use, minus the RF gain, plus the attenuator); in dBFS mode simply dB relative to
        ADC full scale with no calibration at all."""
        applied = getattr(self, "_applied_gain_db", None)
        gain = float(applied if applied is not None else self.gain_db)
        rate = getattr(self, "_applied_sample_rate", None) or self.SAMPLE_RATE
        return self.display_offset_for(rate, gain)

    def atten_on_changed(self, val):
        """Attenuator spinbox -> model; redraw the stored trace at the new calibration."""
        self.input_atten_db = float(val)
        self.update_plot()

    def cispr_get_effective_offset(self):
        """
        Offset of the right (CISPR) axis relative to the left axis. The antenna factor (Radiated) / LISN correction (Conducted) is now
        ADDED DIRECTLY to the displayed values (see display_offset_for), so the trace is already in the units the limit is written in
        and the two axes coincide: the offset is 0.
        """
        return 0.0

    def cispr_axis_label(self):
        if getattr(self, "cispr_mode", "radiated") == "conducted":
            return "CISPR conducted (dBµV)"
        return "CISPR radiated (dBµV/m @10 m)"

    def cispr_refresh_corr_ui(self):
        """Set the correction spinbox's label/tooltip for the current limit type."""
        try:
            if self.cispr_mode == "conducted":
                self.cispr_offset_label.setText("LISN corr.")
                self.cispr_offset_spin.setToolTip(
                    "Correction ADDED to the displayed level to get the level at the LISN measuring port "
                    "(e.g. the loss of a coupling network). The displayed values already include it, so the trace is compared directly with the limit line.")
            else:
                self.cispr_offset_label.setText("Antenna factor")
                self.cispr_offset_spin.setToolTip(
                    "Antenna factor (plus cable loss), dB(1/m), ADDED to the displayed level to give field "
                    "strength in dBuV/m. The displayed values already include it, so the trace is compared directly with the limit line.")
        except Exception:
            pass

    def cispr_on_mode_changed(self, index):
        """Radiated / Conducted combo -> model (GUI thread): swap the limit table, label and correction."""
        mode = self.cispr_mode_combo.itemData(index)
        if mode not in ("radiated", "conducted") or mode == self.cispr_mode:
            return
        self._cispr_corr_by_mode[self.cispr_mode] = float(self.cispr_offset_db)   # remember the old type's value
        self.cispr_mode = mode
        self.cispr_offset_db = float(self._cispr_corr_by_mode[mode])
        self.cispr_offset_spin.blockSignals(True)
        self.cispr_offset_spin.setValue(self.cispr_offset_db)
        self.cispr_offset_spin.blockSignals(False)
        self.cispr_freqs, self.cispr_limits = self.cispr_build_reference_curve()
        self.plot.getAxis('right').setLabel(self.cispr_axis_label())
        self.cispr_refresh_corr_ui()
        self.cispr_refresh_offset_ui()

    def cispr_on_offset_changed(self, val):
        """
        Spinbox -> model. This value is deliberately global (not keyed by mode) so it
        persists across MODE button presses;
        Redraws the curve AND the axis ticks - the ticks must be refreshed here too,
        since they depend on _vb_right's Y range, which cispr_update_curve() just changed.
        """
        self.cispr_offset_db = float(val)
        self._cispr_corr_by_mode[self.cispr_mode] = self.cispr_offset_db
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
            self.cispr_calibration_label.setText(self.cal_label_text())
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
            L_interp = np.interp(fa, freqs, limits, left=np.nan, right=np.nan)   # no line outside the table's range
        except Exception:
            return

        # Effective offset = operator's CISPR Offset + this mode's dBuV_Calibration.
        effective = self.cispr_get_effective_offset()
        self.cispr_effective_offset = effective

        # Plot the CISPR curve in CISPR units (no offset applied to the data itself -
        # the offset is expressed entirely as a shift of the right ViewBox's range below).
        try:
            if np.all(np.isnan(L_interp)):
                self.cispr_curve.setData([], [])          # scan range is outside the limit table
            else:
                try:
                    self.cispr_curve.setData(fa, L_interp, connect='finite')
                except Exception:
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
        further offset subtraction. Changing the offset only ever moves the CISPR axis
        relative to the FFT axis - the line always reads correctly against the CISPR axis's
        own values.
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

    def cispr_build_reference_curve(self, mode=None):
        """
        Return (freqs_hz, limits) for the selected CISPR limit type (self.cispr_mode if mode is None).
          "radiated"  : EN 55032 / CISPR 32 Class B, dBuV/m at 10 m, 30 MHz - 6 GHz (simplified)
          "conducted" : EN 55032 / CISPR 32 Class B, AC mains port, quasi-peak, dBuV, 150 kHz - 30 MHz
        """
        if mode is None:
            mode = getattr(self, "cispr_mode", "radiated")

        if mode == "conducted":
            # 66 -> 56 dBuV falls linearly with log(f) from 150 kHz to 500 kHz, then 56 to 5 MHz, 60 to 30 MHz.
            f_lo = np.geomspace(150e3, 500e3, 20)
            L_lo = 66.0 - 10.0 * np.log10(f_lo / 150e3) / np.log10(500e3 / 150e3)
            f = np.concatenate([f_lo, [5e6, 5.0001e6, 30e6]])
            L = np.concatenate([L_lo, [56.0, 60.0, 60.0]])
            return f.astype(np.float64), L.astype(np.float64)

        # radiated (10 m below 1 GHz, 3 m above 1 GHz scaled to 10 m) - all in dBuV/m
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
            30.0,  # 30-230 MHz  (QP, 10 m)
            30.0,  # 30-230 MHz  (flat segment)
            37.0,  # 230-1000 MHz (QP, 10 m)
            50.0 + delta_3m_to_10m,            # 1-3 GHz scaled to 10 m
            54.0 + delta_3m_to_10m             # 3-6 GHz scaled to 10 m
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
        same real-vs-complex / uint8-vs-float handling as the sweep itself.
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

    # ---- sweep_loop() is a wrapper: it runs the sweep body (_sweep_loop_orig()), then hands a finished
    # ---- clean pass to the waterfall (when it is switched on).
    def sweep_loop(self):
        seq0 = self._pass_seq
        key0 = self.view_band_key() if hasattr(self, "view_signals") else None     # band as it was when this pass started
        self._sweep_loop_orig()
        if self._pass_seq != seq0:
            if key0 is not None:
                try:
                    self.view_note_pass(self._last_pass, key0)                      # first pass in a new band: set the graph's view
                except Exception as e:
                    print(f"view: could not set the initial range: {e!r}")
            if getattr(self, "peak_hold_on", False) or getattr(self, "peak_mark_on", False):
                try:
                    self.peak_note_pass(self._last_pass)                            # peak hold / markers
                except Exception as e:
                    print(f"peak: could not use the finished pass: {e!r}")
            if getattr(self, "wf_enabled", False):
                try:
                    self.wf_note_pass(self._last_pass)
                except Exception as e:
                    print(f"waterfall: could not add a row: {e!r}")

    def _sweep_loop_orig(self):
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
            # Peak detector (EmcScanner.peak_*): keep the largest power per bin over the captures instead of the mean. Surveys, the CAL
            # and the stitch check always use the average detector.
            peak_det = bool(getattr(self, 'detector_peak', False)) and self._survey is None and self._stitch is None
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
                if peak_det:
                    power_acc = power if power_acc is None else np.maximum(power_acc, power)
                else:
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

            mean_power = power_acc if peak_det else power_acc / float(captures_done)   # peak detector: the maximum itself, not a mean
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

            # Diagnostic: show block frequency span and planned storage indices
            fmin = freqs.min()
            fmax = freqs.max()
            start = i * kept_bins
            stop = start + kept_bins

            # Safety checks before writing into stitched arrays
            if self.freq_axis is None or self.power_axis is None:
                print("sweep_loop: ERROR - stitched arrays not allocated")
                break

            if stop > self.freq_axis.size:
                print(f"sweep_loop: ERROR - block {i} stop index {stop} exceeds array length {self.freq_axis.size}")
                break

            # Detect accidental overwrite (indicates logic bug)
            existing_mask = ~np.isnan(self.freq_axis[start:stop])

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

    # ---- single_build_ui(): the version before the decoder was added is kept, commented out, just below the new one ----
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
        self.decoder_build_ui(left_layout)               # DECODER panel (audio / burst decode), under the centre-frequency row

    #OLD# def single_build_ui(self, left_layout):
    #OLD#     self.single_mode = False
    #OLD#     self.single_centre_hz = 100000000
    #OLD#     self._pending_centre = None
    #OLD#     self._single_lock = threading.Lock()
    #OLD#     self._last_preset_index = None
    #OLD#     # the mode button goes ABOVE the range (preset) buttons, same size and shape as them
    #OLD#     first = self.mode_buttons[0]
    #OLD#     self.btn_single = QtWidgets.QPushButton("Single Frequency")
    #OLD#     self.btn_single.setCheckable(True)
    #OLD#     self.btn_single.setSizePolicy(first.sizePolicy())
    #OLD#     self.btn_single.setMinimumSize(first.minimumSize())
    #OLD#     self.btn_single.setToolTip("Test mode: tune to one centre frequency and show a single spectrum of the whole captured "
    #OLD#                                 "bandwidth. The centre frequency is set digit by digit below the gain controls. Press a "
    #OLD#                                 "range button to go back to the range sweeps.")
    #OLD#     self.btn_single.toggled.connect(self.single_on_toggled)
    #OLD#     main_layout = self.centralWidget().layout()
    #OLD#     placed = False
    #OLD#     for i in range(main_layout.count()):
    #OLD#         lay = main_layout.itemAt(i).layout()
    #OLD#         if lay is not None and lay.indexOf(first) >= 0:
    #OLD#             lay.insertWidget(lay.indexOf(first), self.btn_single)
    #OLD#             placed = True
    #OLD#             break
    #OLD#     if not placed:
    #OLD#         main_layout.addWidget(self.btn_single)
    #OLD#     self.mode_buttons.append(self.btn_single)       # joins the controls that are locked during a survey / the scope
    #OLD#     # the digit selector sits in its own row BELOW the gain row, visible only in this mode
    #OLD#     self.single_panel = QtWidgets.QWidget()
    #OLD#     prow = QtWidgets.QHBoxLayout(self.single_panel)
    #OLD#     prow.setContentsMargins(0, 2, 0, 2)
    #OLD#     lab = QtWidgets.QLabel("Centre frequency")
    #OLD#     self.freq_selector = DigitalFreqSelector(self.single_centre_hz)
    #OLD#     self.freq_selector.setToolTip("Click a digit (or use its arrows / the mouse wheel) to change it; with a digit selected "
    #OLD#                                    "the Up / Down keys step it, Left / Right move along, and typing 0-9 overwrites it.")
    #OLD#     self.freq_selector.valueChanged.connect(self.single_on_freq)
    #OLD#     hint = QtWidgets.QLabel("(click a digit, then arrows, wheel, Up/Down or type 0-9)")
    #OLD#     hint.setStyleSheet("QLabel { color: #9aa0a8; }")
    #OLD#     prow.addWidget(lab)
    #OLD#     prow.addWidget(self.freq_selector)
    #OLD#     prow.addWidget(hint)
    #OLD#     prow.addStretch()
    #OLD#     pos = left_layout.count()
    #OLD#     for i in range(left_layout.count()):
    #OLD#         lay = left_layout.itemAt(i).layout()
    #OLD#         if lay is not None and lay.indexOf(self.gain_slider) >= 0:
    #OLD#             pos = i + 1                              # directly under the row that holds the gain slider
    #OLD#             break
    #OLD#     left_layout.insertWidget(pos, self.single_panel)
    #OLD#     self.single_panel.setVisible(False)

    def single_on_toggled(self, checked):
        if checked:
            self.single_enter()
        else:
            self.single_leave()

    # ---- single_enter(): the version before the decoder was added is kept, commented out, just below the new one ----
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
        self.decoder_panel.setVisible(True)
        self.log(f"=== SINGLE FREQUENCY MODE: centre {centre / 1e6:.6f} MHz ===")
        self.update_status_panel()
        self.continuous_when_idle()

    #OLD# def single_enter(self):
    #OLD#     """Switch to the single-frequency test mode and start capturing at once."""
    #OLD#     if self._survey is not None or getattr(self, "scope_active", False) or self._stitch is not None:
    #OLD#         self.log("Single frequency mode is not available while a survey, the scope or a stitch check is running")
    #OLD#         self.btn_single.blockSignals(True)
    #OLD#         self.btn_single.setChecked(False)
    #OLD#         self.btn_single.blockSignals(False)
    #OLD#         return
    #OLD#     if self.sweep_thread and self.sweep_thread.is_alive():
    #OLD#         self.stop_flag.set()
    #OLD#         self.sweep_thread.join(timeout=1.0)
    #OLD#     centre = int(round(0.5 * (self.START_FREQ + self.STOP_FREQ))) if not self.single_mode else self.single_centre_hz
    #OLD#     centre = max(self.freq_selector._min, min(self.freq_selector._max, centre))
    #OLD#     self.single_mode = True
    #OLD#     self.single_centre_hz = centre
    #OLD#     self.freq_selector.setValue(centre)
    #OLD#     with self._single_lock:
    #OLD#         self._pending_centre = None
    #OLD#     self.START_FREQ = self.STOP_FREQ = float(centre)
    #OLD#     self.hf_mode = False
    #OLD#     self.centers = np.array([float(centre)], dtype=np.float64)
    #OLD#     self.current_mode_label = f"Single frequency {centre / 1e6:.6f} MHz"
    #OLD#     self.single_panel.setVisible(True)
    #OLD#     self.log(f"=== SINGLE FREQUENCY MODE: centre {centre / 1e6:.6f} MHz ===")
    #OLD#     self.update_status_panel()
    #OLD#     self.continuous_when_idle()

    def single_leave(self):
        """The mode button was switched off: go back to the last range that was selected (if any)."""
        self.single_exit()
        if self._last_preset_index is not None:
            self.set_preset_and_run(self._last_preset_index)

    # ---- single_exit(): the version before the decoder was added is kept, commented out, just below the new one ----
    def single_exit(self):
        """Leave single-frequency mode (a range button was pressed, or the mode button was switched off)."""
        if not self.single_mode:
            return
        self.single_mode = False
        self.single_panel.setVisible(False)
        self.decoder_cleanup()                           # decoder off, audio closed (the sweep thread is stopped by the caller)
        self.decoder_panel.setVisible(False)
        self.btn_single.blockSignals(True)
        self.btn_single.setChecked(False)
        self.btn_single.blockSignals(False)
        with self._single_lock:
            self._pending_centre = None

    #OLD# def single_exit(self):
    #OLD#     """Leave single-frequency mode (a range button was pressed, or the mode button was switched off)."""
    #OLD#     if not self.single_mode:
    #OLD#         return
    #OLD#     self.single_mode = False
    #OLD#     self.single_panel.setVisible(False)
    #OLD#     self.btn_single.blockSignals(True)
    #OLD#     self.btn_single.setChecked(False)
    #OLD#     self.btn_single.blockSignals(False)
    #OLD#     with self._single_lock:
    #OLD#         self._pending_centre = None

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

    # ------------------------------------------------------------------------------------
    # DECODER (single-frequency mode only)
    # ------------------------------------------------------------------------------------
    # Pick a decoder in the combo box under the centre-frequency digits:
    #   WFM / NFM / AM audio   - demodulate the signal at the single frequency and play it on the PC speaker
    #                            (volume + squelch controls).
    #   OOK bursts / FSK bursts - find short data bursts (433 MHz remotes, sensors...), work out the bit rate (baud) and the
    #                            coding (PWM, Manchester, plain NRZ) and list the data in hex, one entry per burst.
    # While a decoder runs it OWNS the SDR (like the baseband scope): the decoder thread is self.sweep_thread, so every
    # existing "stop the sweep first" path (mode buttons, scope, closeEvent) stops it too. It keeps the spectrum and the
    # waterfall live from the same IQ. The RF Gain slider, Sample Rate and the centre-frequency digits all act immediately.
    #
    # Offset tuning (tick box, default on): the SDR is tuned Fs/8 ABOVE the wanted frequency and the signal is mixed back to
    # zero in software, so the RTL-SDR's own DC spike (always at its tuned centre) is not sitting on the signal. The spectrum
    # is therefore centred Fs/8 above the frequency you set; the orange dashed line marks the frequency being decoded.

    DEC_MODES = [("Off", None),
                 ("WFM audio (broadcast FM)", "WFM"),
                 ("NFM audio (marine VHF / PMR)", "NFM"),
                 ("AM audio", "AM"),
                 ("OOK bursts (433 MHz remotes)", "OOK"),
                 ("FSK bursts", "FSK"),
                 ("LoRa (Meshtastic / LoRaWAN)", "LORA")]

    # ---- decoder_build_ui(): version 2. The version 1 code is kept, commented out, just below ----
    def decoder_build_ui(self, left_layout):
        self.dec_active = False
        self.dec_mode = None
        self.dec_volume = 0.5
        self.dec_squelch_db = -80.0
        self.dec_burst_sens = 6.0
        self.dec_offset_tune = True
        self.dec_lite = True                       # audio modes: slower spectrum / waterfall refresh so the GUI never starves the SDR reads
        self.dec_strip_sync = True                 # PWM remotes: the pulse just before the sync gap is the sync pulse, not a data bit
        self.dec_reset_profile = False
        self.dec_burst_bw = 200.0                  # OOK channel half-width in kHz: cheap transmitters drift and chirp, so it is wide
        self.dec_dump_request = False
        self.dec_lora_sf = 11                      # LoRa decoder settings (defaults = Meshtastic LongFast)
        self.dec_lora_bw = 250e3
        self.dec_lora_sync = None                  # None = accept any sync word
        self.dec_lora_parse = "auto"
        self.dec_lora_invert = False               # LoRaWAN downlinks use inverted IQ
        self.dec_lora_show_bad = True
        self.dec_lora_region = "EU_868"
        self.dec_keys = {"mt": "AQ==", "mt_names": "", "nwk": "", "app": "", "appkey": ""}
        self._lora_cfg = self.decoder_lora_cfg()
        self._dec_session = 0
        self._dec_out = None                       # (name kept from version 1) = the audio sink object that lives in its own thread
        self._dec_thread = None
        self._dec_entries = []                     # burst list entries; repeats of the same code are merged into one entry
        self._dec_hinted = False
        self.dec_signals = DecoderSignals()
        # audio is NOT connected here: decoder_audio_open() connects it straight to the audio thread (the GUI thread is bypassed)
        self.dec_signals.burst.connect(self.decoder_on_burst)
        self.dec_signals.marker.connect(self.decoder_on_marker)
        self.dec_signals.stopped.connect(self.decoder_on_thread_stopped)

        self.decoder_panel = QtWidgets.QWidget()
        vbox = QtWidgets.QVBoxLayout(self.decoder_panel)
        vbox.setContentsMargins(0, 2, 0, 2)
        vbox.setSpacing(2)

        # ---- row 1: decoder choice + audio controls ----
        r1 = QtWidgets.QHBoxLayout()
        vbox.addLayout(r1)
        r1.addWidget(QtWidgets.QLabel("Decoder"))
        self.dec_combo = QtWidgets.QComboBox()
        for label, _m in self.DEC_MODES:
            self.dec_combo.addItem(label)
        self.dec_combo.setToolTip("Decode the signal at the single frequency.\n"
                                  "WFM / NFM / AM: play the demodulated audio on the PC speaker.\n"
                                  "OOK / FSK bursts: find data bursts (e.g. 433 MHz remote controls) and list baud rate, coding and data in hex.\n"
                                  "Off: back to the normal single-frequency spectrum.")
        self.dec_combo.currentIndexChanged.connect(self.decoder_on_mode)
        r1.addWidget(self.dec_combo)

        self.dec_audio_widgets = []
        lab = QtWidgets.QLabel("Volume")
        self.dec_vol_slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.dec_vol_slider.setRange(0, 100)
        self.dec_vol_slider.setValue(int(round(self.dec_volume * 100)))
        self.dec_vol_slider.setFixedWidth(90)
        self.dec_vol_slider.valueChanged.connect(lambda v: setattr(self, "dec_volume", v / 100.0))
        lab2 = QtWidgets.QLabel("Squelch")
        self.dec_sq_spin = QtWidgets.QSpinBox()
        self.dec_sq_spin.setRange(-80, 0)
        self.dec_sq_spin.setValue(-80)
        self.dec_sq_spin.setSuffix(" dBFS")
        self.dec_sq_spin.setSpecialValueText("Off")
        self.dec_sq_spin.setToolTip("Audio is muted while the signal level in the channel (shown in the status line) is below this.\n"
                                    "Off = never muted (noise is loud on an empty channel!). Set it a few dB above the level you see with no signal.")
        self.dec_sq_spin.valueChanged.connect(lambda v: setattr(self, "dec_squelch_db", float(v)))
        self.dec_chk_lite = QtWidgets.QCheckBox("Light display")
        self.dec_chk_lite.setChecked(True)
        self.dec_chk_lite.setToolTip("While listening, redraw the spectrum about 3 times a second and the waterfall about every 0.6 s (instead of 8 times\n"
                                     "a second). The graph redraws are the heaviest thing the program does; when they take too long the SDR is not read\n"
                                     "in time and the audio breaks up. Untick for a smoother display if your PC copes.")
        self.dec_chk_lite.toggled.connect(lambda c: setattr(self, "dec_lite", bool(c)))
        for w in (lab, self.dec_vol_slider, lab2, self.dec_sq_spin, self.dec_chk_lite):
            r1.addWidget(w)
            self.dec_audio_widgets.append(w)

        self.dec_burst_widgets = []
        lab3 = QtWidgets.QLabel("Burst threshold")
        self.dec_sens_spin = QtWidgets.QDoubleSpinBox()
        self.dec_sens_spin.setRange(2.0, 40.0)
        self.dec_sens_spin.setDecimals(1)
        self.dec_sens_spin.setSingleStep(0.5)
        self.dec_sens_spin.setValue(self.dec_burst_sens)
        self.dec_sens_spin.setSuffix(" x noise")
        self.dec_sens_spin.setToolTip("A burst starts when the signal amplitude exceeds the measured noise floor by this factor\n"
                                      "(6 = about 16 dB). Lower it for weak remotes; raise it if noise triggers false bursts.")
        self.dec_sens_spin.valueChanged.connect(lambda v: setattr(self, "dec_burst_sens", float(v)))
        lab_bw = QtWidgets.QLabel("Channel \u00b1")
        self.dec_bw_spin = QtWidgets.QSpinBox()
        self.dec_bw_spin.setRange(25, 450)
        self.dec_bw_spin.setSingleStep(25)
        self.dec_bw_spin.setValue(int(self.dec_burst_bw))
        self.dec_bw_spin.setSuffix(" kHz")
        self.dec_bw_spin.setToolTip("OOK decoder channel width either side of the marked frequency. Cheap remotes drift in frequency and their\n"
                                    "amplitude sags during a frame, so a narrow channel loses the end of each frame. Wider = more tolerant of\n"
                                    "drift but a little more noise (and other signals nearby can interfere). 200 kHz is a good start.")
        self.dec_bw_spin.valueChanged.connect(lambda v: setattr(self, "dec_burst_bw", float(v)))
        btn_dump = QtWidgets.QPushButton("Dump")
        btn_dump.setToolTip("Print the raw pulse table (high/low widths in us, one line per frame) of the last OOK transmission into the list.\n"
                            "Useful to see exactly what the remote is sending when a code will not decode cleanly.")
        btn_dump.clicked.connect(lambda: setattr(self, "dec_dump_request", True))
        self.dec_chk_sync = QtWidgets.QCheckBox("Strip sync pulse")
        self.dec_chk_sync.setChecked(True)
        self.dec_chk_sync.setToolTip("PWM remotes (EV1527 / PT2262 style) send a short sync pulse followed by a long gap before each frame.\n"
                                     "That pulse is not part of the code, so it is left out. If your code comes out one bit LONGER than the\n"
                                     "remote's real length, leave this ticked; if it is one bit SHORT, untick it.")
        self.dec_chk_sync.toggled.connect(lambda c: setattr(self, "dec_strip_sync", bool(c)))
        btn_clear = QtWidgets.QPushButton("Clear")
        btn_clear.setToolTip("Clear the list AND forget the learned timing (use when you change to a different remote).")
        btn_clear.clicked.connect(self.decoder_clear_list)
        btn_copy = QtWidgets.QPushButton("Copy")
        btn_copy.clicked.connect(lambda: QtWidgets.QApplication.clipboard().setText(self.dec_log.toPlainText()))
        for w in (lab3, self.dec_sens_spin, lab_bw, self.dec_bw_spin, self.dec_chk_sync, btn_dump):
            r1.addWidget(w)
            self.dec_burst_widgets.append(w)
        self.dec_list_widgets = []
        for w in (btn_clear, btn_copy):
            r1.addWidget(w)
            self.dec_list_widgets.append(w)

        self.dec_chk_offset = QtWidgets.QCheckBox("Offset tune")
        self.dec_chk_offset.setChecked(True)
        self.dec_chk_offset.setToolTip("Tune the SDR Fs/8 above the wanted frequency and mix back in software, so the tuner's DC spike\n"
                                       "is not on top of the signal. The spectrum is then centred Fs/8 above the frequency you set\n"
                                       "(the orange dashed line marks the frequency being decoded). Untick to tune exactly on it.")
        self.dec_chk_offset.toggled.connect(lambda c: setattr(self, "dec_offset_tune", bool(c)))
        r1.addWidget(self.dec_chk_offset)
        r1.addStretch(1)

        # ---- row 1b: LoRa controls (only visible for the LoRa decoder) ----
        r2 = QtWidgets.QHBoxLayout()
        vbox.addLayout(r2)
        self.dec_lora_widgets = []
        def _lw(w):
            r2.addWidget(w)
            self.dec_lora_widgets.append(w)
            return w
        _lw(QtWidgets.QLabel("Preset"))
        self.dec_lora_preset = _lw(QtWidgets.QComboBox())
        for label, _p in LORA_PRESETS:
            self.dec_lora_preset.addItem(label)
        self.dec_lora_preset.setToolTip("Picks SF, bandwidth, sync word, parser and the centre frequency (Meshtastic: from the Region box using the\n"
                                        "firmware's own slot rule; LoRaWAN: the default channel).")
        self.dec_lora_preset.currentIndexChanged.connect(self.decoder_lora_preset)
        _lw(QtWidgets.QLabel("Region"))
        self.dec_lora_region_combo = _lw(QtWidgets.QComboBox())
        for rname in MT_REGIONS:
            if rname not in ("UNSET", "LORA_24"):
                self.dec_lora_region_combo.addItem(rname)
        self.dec_lora_region_combo.setCurrentText(self.dec_lora_region)
        self.dec_lora_region_combo.setToolTip("Meshtastic region: sets the default channel frequency for a Meshtastic preset (EU_868 = UK / Europe).")
        self.dec_lora_region_combo.currentTextChanged.connect(self.decoder_lora_region_changed)
        _lw(QtWidgets.QLabel("SF"))
        self.dec_lora_sf_spin = _lw(QtWidgets.QSpinBox())
        self.dec_lora_sf_spin.setRange(7, 12)
        self.dec_lora_sf_spin.setValue(self.dec_lora_sf)
        self.dec_lora_sf_spin.valueChanged.connect(lambda v: self.decoder_lora_manual("sf", v))
        _lw(QtWidgets.QLabel("BW"))
        self.dec_lora_bw_combo = _lw(QtWidgets.QComboBox())
        for kh in (62.5, 125.0, 250.0, 500.0):
            self.dec_lora_bw_combo.addItem(f"{kh:g} kHz", kh * 1e3)
        self.dec_lora_bw_combo.setCurrentIndex(2)
        self.dec_lora_bw_combo.currentIndexChanged.connect(lambda i: self.decoder_lora_manual("bw", self.dec_lora_bw_combo.itemData(i)))
        self.dec_lora_sync_combo = _lw(QtWidgets.QComboBox())
        for label, val in LORA_SYNC_CHOICES:
            self.dec_lora_sync_combo.addItem(label, val)
        self.dec_lora_sync_combo.setToolTip("Only show frames with this sync word (Any = show everything; the sync word is listed per frame).")
        self.dec_lora_sync_combo.currentIndexChanged.connect(lambda i: self.decoder_lora_manual("sync", self.dec_lora_sync_combo.itemData(i)))
        _lw(QtWidgets.QLabel("Parse"))
        self.dec_lora_parse_combo = _lw(QtWidgets.QComboBox())
        for label, val in LORA_PARSE_CHOICES:
            self.dec_lora_parse_combo.addItem(label, val)
        self.dec_lora_parse_combo.currentIndexChanged.connect(lambda i: setattr(self, "dec_lora_parse", self.dec_lora_parse_combo.itemData(i)))
        self.dec_lora_inv_chk = _lw(QtWidgets.QCheckBox("Invert IQ"))
        self.dec_lora_inv_chk.setToolTip("LoRaWAN gateway DOWNLINKS are sent with inverted IQ; tick this to receive them (uplinks need it off).")
        self.dec_lora_inv_chk.toggled.connect(lambda c: setattr(self, "dec_lora_invert", bool(c)))
        self.dec_lora_bad_chk = _lw(QtWidgets.QCheckBox("Show CRC errors"))
        self.dec_lora_bad_chk.setChecked(True)
        self.dec_lora_bad_chk.setToolTip("Also list frames whose header decoded but whose payload CRC failed (marked CRC FAIL, not parsed).")
        self.dec_lora_bad_chk.toggled.connect(lambda c: setattr(self, "dec_lora_show_bad", bool(c)))
        btn_keys = _lw(QtWidgets.QPushButton("Keys..."))
        btn_keys.setToolTip("Meshtastic channel keys and LoRaWAN session keys (optional): needed to read encrypted payloads / check MICs.")
        btn_keys.clicked.connect(self.decoder_lora_keys)
        r2.addStretch(1)

        # ---- row 2: status line, and (burst modes) the list of decoded bursts ----
        self.dec_status = QtWidgets.QLabel("")
        self.dec_status.setFont(QtGui.QFont("Consolas", 9))
        self.dec_status.setSizePolicy(QtWidgets.QSizePolicy.Ignored, QtWidgets.QSizePolicy.Fixed)
        self.dec_status.setMinimumWidth(0)
        vbox.addWidget(self.dec_status)
        self.dec_signals.status.connect(self.dec_status.setText)
        self.dec_log = QtWidgets.QPlainTextEdit()
        self.dec_log.setReadOnly(True)
        self.dec_log.setFont(QtGui.QFont("Consolas", 9))
        self.dec_log.setLineWrapMode(QtWidgets.QPlainTextEdit.NoWrap)
        self.dec_log.setFixedHeight(8 * self.dec_log.fontMetrics().lineSpacing() + 10)
        vbox.addWidget(self.dec_log)

        # marker for the frequency being decoded on the spectrum
        self._dec_line = pg.InfiniteLine(pos=0, angle=90, movable=False,
                                         pen=pg.mkPen("#e08a00", width=1, style=QtCore.Qt.DashLine))
        self.plot.addItem(self._dec_line, ignoreBounds=True)
        self._dec_line.setVisible(False)

        pos = left_layout.indexOf(self.single_panel)
        left_layout.insertWidget(pos + 1 if pos >= 0 else left_layout.count(), self.decoder_panel)
        self.decoder_panel.setVisible(False)
        self.decoder_show_controls()

    #OLD# def decoder_build_ui(self, left_layout):
    #OLD#     self.dec_active = False
    #OLD#     self.dec_mode = None
    #OLD#     self.dec_volume = 0.5
    #OLD#     self.dec_squelch_db = -80.0
    #OLD#     self.dec_burst_sens = 6.0
    #OLD#     self.dec_offset_tune = True
    #OLD#     self._dec_session = 0
    #OLD#     self._dec_out = None
    #OLD#     self._dec_dev = None
    #OLD#     self._dec_channels = 1
    #OLD#     self._dec_prebuf = bytearray()
    #OLD#     self._dec_primed = False
    #OLD#     self._dec_prime_bytes = 0
    #OLD#     self._dec_dropped = 0
    #OLD#     self._dec_last_hex = None
    #OLD#     self._dec_last_t = 0.0
    #OLD#     self.dec_signals = DecoderSignals()
    #OLD#     self.dec_signals.audio.connect(self.decoder_on_audio)
    #OLD#     self.dec_signals.burst.connect(self.decoder_on_burst)
    #OLD#     self.dec_signals.marker.connect(self.decoder_on_marker)
    #OLD#     self.dec_signals.stopped.connect(self.decoder_on_thread_stopped)
    #OLD#
    #OLD#     self.decoder_panel = QtWidgets.QWidget()
    #OLD#     vbox = QtWidgets.QVBoxLayout(self.decoder_panel)
    #OLD#     vbox.setContentsMargins(0, 2, 0, 2)
    #OLD#     vbox.setSpacing(2)
    #OLD#
    #OLD#     # ---- row 1: decoder choice + audio controls ----
    #OLD#     r1 = QtWidgets.QHBoxLayout()
    #OLD#     vbox.addLayout(r1)
    #OLD#     r1.addWidget(QtWidgets.QLabel("Decoder"))
    #OLD#     self.dec_combo = QtWidgets.QComboBox()
    #OLD#     for label, _m in self.DEC_MODES:
    #OLD#         self.dec_combo.addItem(label)
    #OLD#     self.dec_combo.setToolTip("Decode the signal at the single frequency.\n"
    #OLD#                               "WFM / NFM / AM: play the demodulated audio on the PC speaker.\n"
    #OLD#                               "OOK / FSK bursts: find data bursts (e.g. 433 MHz remote controls) and list baud rate, coding and data in hex.\n"
    #OLD#                               "Off: back to the normal single-frequency spectrum.")
    #OLD#     self.dec_combo.currentIndexChanged.connect(self.decoder_on_mode)
    #OLD#     r1.addWidget(self.dec_combo)
    #OLD#
    #OLD#     self.dec_audio_widgets = []
    #OLD#     lab = QtWidgets.QLabel("Volume")
    #OLD#     self.dec_vol_slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
    #OLD#     self.dec_vol_slider.setRange(0, 100)
    #OLD#     self.dec_vol_slider.setValue(int(round(self.dec_volume * 100)))
    #OLD#     self.dec_vol_slider.setFixedWidth(90)
    #OLD#     self.dec_vol_slider.valueChanged.connect(lambda v: setattr(self, "dec_volume", v / 100.0))
    #OLD#     lab2 = QtWidgets.QLabel("Squelch")
    #OLD#     self.dec_sq_spin = QtWidgets.QSpinBox()
    #OLD#     self.dec_sq_spin.setRange(-80, 0)
    #OLD#     self.dec_sq_spin.setValue(-80)
    #OLD#     self.dec_sq_spin.setSuffix(" dBFS")
    #OLD#     self.dec_sq_spin.setSpecialValueText("Off")
    #OLD#     self.dec_sq_spin.setToolTip("Audio is muted while the signal level in the channel (shown in the status line) is below this.\n"
    #OLD#                                 "Off = never muted (noise is loud on an empty channel!). Set it a few dB above the level you see with no signal.")
    #OLD#     self.dec_sq_spin.valueChanged.connect(lambda v: setattr(self, "dec_squelch_db", float(v)))
    #OLD#     for w in (lab, self.dec_vol_slider, lab2, self.dec_sq_spin):
    #OLD#         r1.addWidget(w)
    #OLD#         self.dec_audio_widgets.append(w)
    #OLD#
    #OLD#     self.dec_burst_widgets = []
    #OLD#     lab3 = QtWidgets.QLabel("Burst threshold")
    #OLD#     self.dec_sens_spin = QtWidgets.QDoubleSpinBox()
    #OLD#     self.dec_sens_spin.setRange(2.0, 40.0)
    #OLD#     self.dec_sens_spin.setDecimals(1)
    #OLD#     self.dec_sens_spin.setSingleStep(0.5)
    #OLD#     self.dec_sens_spin.setValue(self.dec_burst_sens)
    #OLD#     self.dec_sens_spin.setSuffix(" x noise")
    #OLD#     self.dec_sens_spin.setToolTip("A burst starts when the signal amplitude exceeds the measured noise floor by this factor\n"
    #OLD#                                   "(6 = about 16 dB). Lower it for weak remotes; raise it if noise triggers false bursts.")
    #OLD#     self.dec_sens_spin.valueChanged.connect(lambda v: setattr(self, "dec_burst_sens", float(v)))
    #OLD#     btn_clear = QtWidgets.QPushButton("Clear")
    #OLD#     btn_clear.clicked.connect(lambda: self.dec_log.clear())
    #OLD#     btn_copy = QtWidgets.QPushButton("Copy")
    #OLD#     btn_copy.clicked.connect(lambda: QtWidgets.QApplication.clipboard().setText(self.dec_log.toPlainText()))
    #OLD#     for w in (lab3, self.dec_sens_spin, btn_clear, btn_copy):
    #OLD#         r1.addWidget(w)
    #OLD#         self.dec_burst_widgets.append(w)
    #OLD#
    #OLD#     self.dec_chk_offset = QtWidgets.QCheckBox("Offset tune")
    #OLD#     self.dec_chk_offset.setChecked(True)
    #OLD#     self.dec_chk_offset.setToolTip("Tune the SDR Fs/8 above the wanted frequency and mix back in software, so the tuner's DC spike\n"
    #OLD#                                    "is not on top of the signal. The spectrum is then centred Fs/8 above the frequency you set\n"
    #OLD#                                    "(the orange dashed line marks the frequency being decoded). Untick to tune exactly on it.")
    #OLD#     self.dec_chk_offset.toggled.connect(lambda c: setattr(self, "dec_offset_tune", bool(c)))
    #OLD#     r1.addWidget(self.dec_chk_offset)
    #OLD#     r1.addStretch(1)
    #OLD#
    #OLD#     # ---- row 2: status line, and (burst modes) the list of decoded bursts ----
    #OLD#     self.dec_status = QtWidgets.QLabel("")
    #OLD#     self.dec_status.setFont(QtGui.QFont("Consolas", 9))
    #OLD#     self.dec_status.setSizePolicy(QtWidgets.QSizePolicy.Ignored, QtWidgets.QSizePolicy.Fixed)
    #OLD#     self.dec_status.setMinimumWidth(0)
    #OLD#     vbox.addWidget(self.dec_status)
    #OLD#     self.dec_signals.status.connect(self.dec_status.setText)
    #OLD#     self.dec_log = QtWidgets.QPlainTextEdit()
    #OLD#     self.dec_log.setReadOnly(True)
    #OLD#     self.dec_log.setFont(QtGui.QFont("Consolas", 9))
    #OLD#     self.dec_log.setLineWrapMode(QtWidgets.QPlainTextEdit.NoWrap)
    #OLD#     self.dec_log.setMaximumBlockCount(1000)
    #OLD#     self.dec_log.setFixedHeight(8 * self.dec_log.fontMetrics().lineSpacing() + 10)
    #OLD#     vbox.addWidget(self.dec_log)
    #OLD#
    #OLD#     # marker for the frequency being decoded on the spectrum
    #OLD#     self._dec_line = pg.InfiniteLine(pos=0, angle=90, movable=False,
    #OLD#                                      pen=pg.mkPen("#e08a00", width=1, style=QtCore.Qt.DashLine))
    #OLD#     self.plot.addItem(self._dec_line, ignoreBounds=True)
    #OLD#     self._dec_line.setVisible(False)
    #OLD#
    #OLD#     pos = left_layout.indexOf(self.single_panel)
    #OLD#     left_layout.insertWidget(pos + 1 if pos >= 0 else left_layout.count(), self.decoder_panel)
    #OLD#     self.decoder_panel.setVisible(False)
    #OLD#     self.decoder_show_controls()

    # ---- decoder_show_controls(): version 2. The version 1 code is kept, commented out, just below ----
    def decoder_show_controls(self):
        """Show only the controls that belong to the selected decoder."""
        audio = self.dec_mode in ("WFM", "NFM", "AM")
        burst = self.dec_mode in ("OOK", "FSK")
        lora = self.dec_mode == "LORA"
        for w in self.dec_audio_widgets:
            w.setVisible(audio)
        for w in self.dec_burst_widgets:
            w.setVisible(burst)
        for w in self.dec_lora_widgets:
            w.setVisible(lora)
        for w in self.dec_list_widgets:
            w.setVisible(burst or lora)
        self.dec_chk_offset.setVisible(audio or burst or lora)
        self.dec_log.setVisible(burst or lora)
        self.dec_status.setVisible(audio or burst or lora)

    # ---- LoRa decoder front-panel handling ----

    #OLD# def decoder_show_controls(self):
    #OLD#     """Show only the controls that belong to the selected decoder."""
    #OLD#     audio = self.dec_mode in ("WFM", "NFM", "AM")
    #OLD#     burst = self.dec_mode in ("OOK", "FSK")
    #OLD#     for w in self.dec_audio_widgets:
    #OLD#         w.setVisible(audio)
    #OLD#     for w in self.dec_burst_widgets:
    #OLD#         w.setVisible(burst)
    #OLD#     self.dec_chk_offset.setVisible(audio or burst)
    #OLD#     self.dec_log.setVisible(burst)
    #OLD#     self.dec_status.setVisible(audio or burst)
    #OLD#
    #OLD# # ---- front-panel events (GUI thread) ----

    def decoder_on_mode(self, index):
        mode = self.DEC_MODES[index][1]
        if mode is None:
            self.decoder_stop(resume=True)
            return
        if not getattr(self, "single_mode", False):
            self.log("Decoder: select Single Frequency mode first")
            self.decoder_reset_combo()
            return
        if mode in ("WFM", "NFM", "AM") and QtMultimedia is None:
            self.log("Decoder: audio is not available (PyQt5.QtMultimedia could not be imported) - burst decoding still works")
            self.decoder_reset_combo()
            return
        self.dec_mode = mode
        self.decoder_show_controls()
        if self.dec_active:
            # only the type of decoder changed: the running thread rebuilds its demodulator; just open / close the audio device
            if mode in ("WFM", "NFM", "AM"):
                if self._dec_out is None:
                    self.decoder_audio_open()
            else:
                self.decoder_audio_close()
            self.log(f"Decoder: {mode}")
            return
        self.decoder_start()

    def decoder_reset_combo(self):
        self.dec_combo.blockSignals(True)
        self.dec_combo.setCurrentIndex(0)
        self.dec_combo.blockSignals(False)

    def decoder_set_lock(self, locked):
        """While the decoder owns the SDR, the buttons that would start a sweep or the scope are disabled."""
        for name in ("btn_start", "btn_cont", "btn_survey", "btn_scope"):
            w = getattr(self, name, None)
            if w is not None:
                w.setEnabled(not locked)

    def decoder_start(self):
        if self._survey is not None or getattr(self, "scope_active", False) or self._stitch is not None:
            self.log("Decoder: not available while a survey, the scope or a stitch check is running")
            self.decoder_reset_combo()
            self.dec_mode = None
            self.decoder_show_controls()
            return
        # stop the single-frequency sweep first so two threads never share the SDR
        if self.sweep_thread and self.sweep_thread.is_alive():
            self.stop_flag.set()
            self.sweep_thread.join(timeout=3.0)
            if self.sweep_thread.is_alive():
                self.log("Decoder: ERROR - the sweep thread did not stop; try again")
                self.decoder_reset_combo()
                self.dec_mode = None
                self.decoder_show_controls()
                return
        self.stop_flag.clear()
        try:
            self.init_sdr()
        except Exception as e:
            self.log(f"Decoder: error opening the SDR: {e}")
            self.decoder_reset_combo()
            self.dec_mode = None
            self.decoder_show_controls()
            return
        with self._single_lock:
            self._pending_centre = None
        self.START_FREQ = self.STOP_FREQ = float(self.single_centre_hz)
        if self.dec_mode in ("WFM", "NFM", "AM"):
            self.decoder_audio_open()
        self._dec_session += 1
        self.dec_active = True
        self.decoder_set_lock(True)
        self._dec_line.setVisible(True)
        self.decoder_show_controls()
        self.log(f"=== DECODER {self.dec_mode} at {self.START_FREQ / 1e6:.6f} MHz ===")
        self.sweep_thread = threading.Thread(target=self.decoder_main, args=(self._dec_session,), daemon=True)
        self.sweep_thread.start()

    def decoder_stop(self, resume=True):
        """Decoder switched off from the combo box: stop its thread, then resume the normal single-frequency spectrum."""
        was = self.dec_active
        self.decoder_cleanup()
        self.stop_flag.set()
        if self.sweep_thread and self.sweep_thread.is_alive():
            self.sweep_thread.join(timeout=3.0)
        if was:
            self.log("=== DECODER STOPPED ===")
        if resume and was and getattr(self, "single_mode", False):
            self.continuous_when_idle()

    def decoder_cleanup(self):
        """Idempotent: everything on the GUI side that has to be put back when the decoder is no longer running."""
        self.dec_active = False
        self.dec_mode = None
        self.decoder_audio_close()
        self.decoder_set_lock(False)
        self.decoder_reset_combo()
        self.decoder_show_controls()
        try:
            self._dec_line.setVisible(False)
        except Exception:
            pass
        self.dec_status.setText("")

    def decoder_on_thread_stopped(self, session):
        """The decoder thread ended. If something else stopped it (a mode button, a read failure...) tidy the front panel up."""
        if session == self._dec_session and self.dec_active:
            self.decoder_cleanup()
            self.log("=== DECODER STOPPED ===")

    def decoder_on_marker(self, hz):
        try:
            self._dec_line.setValue(float(hz))
        except Exception:
            pass

    # ---- audio output (GUI thread) ----

    # ---- decoder_audio_open(): version 2. The version 1 code is kept, commented out, just below ----
    def decoder_audio_open(self):
        """Open the sound device in its OWN thread. Everything that touches the sound card (queueing, pre-buffering, writing) happens
        there, so a GUI redraw that takes 100 ms can no longer starve it. The decoder thread feeds it through a queued signal."""
        if QtMultimedia is None:
            return False
        self.decoder_audio_close()
        th = QtCore.QThread(self)
        sink = DecoderAudioSink()
        sink.moveToThread(th)
        th.start()
        QtCore.QMetaObject.invokeMethod(sink, "open", QtCore.Qt.BlockingQueuedConnection)
        if not sink.ok:
            self.log(f"Decoder: no sound - {sink.error}")
            th.quit()
            th.wait(2000)
            return False
        self._dec_thread = th
        self._dec_out = sink
        self.dec_signals.audio.connect(sink.push)
        self.log(f"Decoder audio: {sink.name} (own audio thread, {int(DECODER_AUDIO_CUSHION_S * 1000)} ms cushion)")
        return True

    #OLD# def decoder_audio_open(self):
    #OLD#     if QtMultimedia is None:
    #OLD#         return False
    #OLD#     self.decoder_audio_close()
    #OLD#     info = QtMultimedia.QAudioDeviceInfo.defaultOutputDevice()
    #OLD#     fmt = None
    #OLD#     for ch in (1, 2):
    #OLD#         f = QtMultimedia.QAudioFormat()
    #OLD#         f.setSampleRate(AudioDemod.AUDIO_RATE)
    #OLD#         f.setChannelCount(ch)
    #OLD#         f.setSampleSize(16)
    #OLD#         f.setCodec("audio/pcm")
    #OLD#         f.setByteOrder(QtMultimedia.QAudioFormat.LittleEndian)
    #OLD#         f.setSampleType(QtMultimedia.QAudioFormat.SignedInt)
    #OLD#         if info.isFormatSupported(f):
    #OLD#             fmt = f
    #OLD#             self._dec_channels = ch
    #OLD#             break
    #OLD#     if fmt is None:
    #OLD#         self.log("Decoder: the default audio output does not accept 48 kHz 16-bit audio - no sound")
    #OLD#         return False
    #OLD#     out = QtMultimedia.QAudioOutput(info, fmt, self)
    #OLD#     out.setBufferSize(int(AudioDemod.AUDIO_RATE * 2 * self._dec_channels * 0.5))
    #OLD#     self._dec_dev = out.start()
    #OLD#     self._dec_out = out
    #OLD#     self._dec_prime_bytes = int(AudioDemod.AUDIO_RATE * 2 * self._dec_channels * 0.15)    # 150 ms pre-buffer
    #OLD#     self._dec_prebuf = bytearray()
    #OLD#     self._dec_primed = False
    #OLD#     self._dec_dropped = 0
    #OLD#     self.log(f"Decoder audio: {info.deviceName()}")
    #OLD#     return True

    # ---- decoder_audio_close(): version 2. The version 1 code is kept, commented out, just below ----
    def decoder_audio_close(self):
        sink = getattr(self, "_dec_out", None)
        th = getattr(self, "_dec_thread", None)
        self._dec_out = None
        self._dec_thread = None
        if sink is None:
            return
        try:
            self.dec_signals.audio.disconnect(sink.push)
        except Exception:
            pass
        try:
            if th is not None and th.isRunning():
                QtCore.QMetaObject.invokeMethod(sink, "close", QtCore.Qt.BlockingQueuedConnection)
                th.quit()
                th.wait(2000)
        except Exception:
            pass

    #OLD# def decoder_audio_close(self):
    #OLD#     out = getattr(self, "_dec_out", None)
    #OLD#     self._dec_out = None
    #OLD#     self._dec_dev = None
    #OLD#     self._dec_primed = False
    #OLD#     self._dec_prebuf = bytearray()
    #OLD#     if out is not None:
    #OLD#         try:
    #OLD#             out.stop()
    #OLD#             out.deleteLater()
    #OLD#         except Exception:
    #OLD#             pass

    # ---- decoder_on_audio(): version 2. The version 1 code is kept, commented out, just below ----
    def decoder_on_audio(self, data):
        pass                                       # audio no longer passes through the GUI thread - see decoder_audio_open()

    # ---- burst list (GUI thread) ----

    #OLD# def decoder_on_audio(self, data):
    #OLD#     """GUI thread: queue 48 kHz mono int16 into the sound device. 150 ms is collected before playing starts (and again after an
    #OLD#     under-run); if the device is full the chunk is dropped, which only happens if the sound card clock runs slower than the SDR's."""
    #OLD#     out, dev = self._dec_out, self._dec_dev
    #OLD#     if out is None or dev is None:
    #OLD#         return
    #OLD#     if self._dec_channels == 2:
    #OLD#         data = np.repeat(np.frombuffer(data, dtype=np.int16), 2).tobytes()
    #OLD#     if self._dec_primed and out.state() == QtMultimedia.QAudio.IdleState:
    #OLD#         self._dec_primed = False                              # under-run: build the cushion up again
    #OLD#         self._dec_prebuf = bytearray()
    #OLD#     if not self._dec_primed:
    #OLD#         self._dec_prebuf += data
    #OLD#         if len(self._dec_prebuf) >= self._dec_prime_bytes:
    #OLD#             n = min(out.bytesFree(), len(self._dec_prebuf))
    #OLD#             dev.write(bytes(self._dec_prebuf[:n]))
    #OLD#             self._dec_prebuf = bytearray()
    #OLD#             self._dec_primed = True
    #OLD#         return
    #OLD#     if out.bytesFree() >= len(data):
    #OLD#         dev.write(data)
    #OLD#     else:
    #OLD#         self._dec_dropped += 1
    #OLD#
    #OLD# # ---- burst list (GUI thread) ----

    # ---- decoder_on_burst(): version 2. The version 1 code is kept, commented out, just below ----
    def decoder_on_burst(self, r):
        import datetime
        now = time.time()
        if r.get("dump_text"):
            self._dec_entries.append({"text": r["dump_text"], "r": {"mode": r.get("mode", "")}, "ts": "", "n": 1,
                                      "t_first": now, "t_last": now, "agree": 0, "frames": 0})
            self.decoder_render_log()
            return
        hx = r.get("hex") or ""
        last = self._dec_entries[-1] if self._dec_entries else None
        if (last is not None and hx and not r.get("lines") and last["r"].get("hex") == hx and last["r"]["mode"] == r["mode"]
                and now - last["t_last"] < 3.0):
            # the same code again (a held button repeats it): one list entry, with a count
            last["n"] += 1
            last["t_last"] = now
            if r.get("frames") is not None:
                last["agree"] += r.get("agree", 0)
                last["frames"] += r.get("frames", 0)
        else:
            ts = datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]
            self._dec_entries.append({"r": r, "ts": ts, "n": 1, "t_first": now, "t_last": now,
                                      "agree": r.get("agree", 0), "frames": r.get("frames", 0)})
            del self._dec_entries[:-300]
        self.decoder_render_log()

    #OLD# def decoder_on_burst(self, r):
    #OLD#     import datetime
    #OLD#     now = time.time()
    #OLD#     hx = r.get("hex") or ""
    #OLD#     repeat = bool(hx) and hx == self._dec_last_hex and (now - self._dec_last_t) < 1.5
    #OLD#     self._dec_last_hex, self._dec_last_t = hx, now
    #OLD#     ts = datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]
    #OLD#     if r.get("baud") and r.get("nbits"):
    #OLD#         line = f"{ts}  {r['mode']}  {r['dur_ms']:6.1f} ms  {r['baud']:7.0f} baud  {r['nbits']:3d} bits  {hx}"
    #OLD#         if r.get("dev_khz") is not None:
    #OLD#             line += f"   dev +/-{r['dev_khz']:.1f} kHz"
    #OLD#         if repeat:
    #OLD#             line += "   (repeat)"
    #OLD#         extra = f"          {r['coding']}"
    #OLD#         if r.get("notes"):
    #OLD#             extra += f";  {r['notes']}"
    #OLD#         if r.get("hex_alt"):
    #OLD#             extra += f";  alt: {r['hex_alt']}"
    #OLD#         self.dec_log.appendPlainText(line)
    #OLD#         self.dec_log.appendPlainText(extra)
    #OLD#     else:
    #OLD#         w = ", ".join(f"{c:.0f}" for c in r.get("centres_us", [])[:6])
    #OLD#         self.dec_log.appendPlainText(f"{ts}  {r['mode']}  {r['dur_ms']:6.1f} ms  no regular coding found "
    #OLD#                                      f"({r.get('runs', 0)} runs, widths about {w} us)")
    #OLD#
    #OLD# # ---- the decoder thread ----

    # ---- decoder_main(): version 2. The version 1 code is kept, commented out, just below ----
    def decoder_clear_list(self):
        self._dec_entries = []
        self.dec_log.clear()
        self.dec_reset_profile = True              # the decoder thread forgets the learned timing at its next block

    @staticmethod
    def _dec_entry_lines(e):
        if e.get("text"):
            return e["text"].split("\n")
        r = e["r"]
        if r.get("lines"):                                  # a LoRa frame: physical layer line, hex, parsed protocol layer
            return [f"{e['ts']}  " + r["lines"][0]] + ["          " + ln for ln in r["lines"][1:]]
        hx = r.get("hex") or ""
        if r.get("baud") and r.get("nbits"):
            line = f"{e['ts']}  {r['mode']}  {r['baud']:6.0f} baud  {r['nbits']:3d} bits  {hx}"
            if r.get("frames") is not None:
                line += f"   {e['agree']}/{e['frames']} frames agree"
            else:
                line += f"   {r['dur_ms']:.1f} ms"
            if r.get("dev_khz") is not None:
                line += f"   dev +/-{r['dev_khz']:.1f} kHz"
            if e["n"] > 1:
                line += f"   (x{e['n']} over {e['t_last'] - e['t_first']:.1f} s)"
            parts = []
            if r.get("bin"):
                b = r["bin"]
                parts.append("bin " + " ".join(b[i:i + 4] for i in range(0, len(b), 4)))
            if "?" in (r.get("bin") or ""):
                parts.append("? = bit the frames disagree on")
            parts.append(r["coding"])
            if r.get("notes"):
                parts.append(r["notes"])
            if r.get("hex_alt"):
                parts.append(f"alt: {r['hex_alt']}")
            if r.get("locked"):
                parts.append("timing locked from earlier press")
            return [line, "          " + ";  ".join(parts)]
        w = ", ".join(f"{c:.0f}" for c in r.get("centres_us", [])[:6])
        return [f"{e['ts']}  {r['mode']}  {r.get('dur_ms', 0):6.1f} ms  no regular coding found "
                f"({r.get('runs', 0)} runs, widths about {w} us)"]

    def decoder_render_log(self):
        lines = []
        for e in self._dec_entries:
            lines.extend(self._dec_entry_lines(e))
        sb = self.dec_log.verticalScrollBar()
        at_end = sb.value() >= sb.maximum() - 2
        self.dec_log.setPlainText("\n".join(lines))
        if at_end:
            sb.setValue(sb.maximum())

    # ---- the decoder thread ----

    def decoder_lora_cfg(self):
        """Parse the Keys... dialog texts into what lora_describe() needs."""
        k = self.dec_keys
        def hx(t):
            t = "".join(c for c in t if c in "0123456789abcdefABCDEF")
            return bytes.fromhex(t) if len(t) == 32 else None
        return {"mt_keys": [x.strip() for x in k["mt"].split(",") if x.strip()] or ["AQ=="],
                "mt_names": [x.strip() for x in k["mt_names"].split(",") if x.strip()],
                "nwk": hx(k["nwk"]), "app": hx(k["app"]), "appkey": hx(k["appkey"])}

    def decoder_lora_set(self, sf=None, bw=None, sync="keep", parse=None, invert=None):
        """Set the LoRa widgets (without re-triggering the 'manual change' handlers) and the attributes the decoder thread reads."""
        if sf is not None:
            self.dec_lora_sf_spin.blockSignals(True)
            self.dec_lora_sf_spin.setValue(int(sf))
            self.dec_lora_sf_spin.blockSignals(False)
            self.dec_lora_sf = int(sf)
        if bw is not None:
            i = self.dec_lora_bw_combo.findData(float(bw))
            if i >= 0:
                self.dec_lora_bw_combo.blockSignals(True)
                self.dec_lora_bw_combo.setCurrentIndex(i)
                self.dec_lora_bw_combo.blockSignals(False)
                self.dec_lora_bw = float(bw)
        if sync != "keep":
            i = self.dec_lora_sync_combo.findData(sync)
            self.dec_lora_sync_combo.blockSignals(True)
            self.dec_lora_sync_combo.setCurrentIndex(max(i, 0))
            self.dec_lora_sync_combo.blockSignals(False)
            self.dec_lora_sync = sync
        if parse is not None:
            i = self.dec_lora_parse_combo.findData(parse)
            self.dec_lora_parse_combo.blockSignals(True)
            self.dec_lora_parse_combo.setCurrentIndex(max(i, 0))
            self.dec_lora_parse_combo.blockSignals(False)
            self.dec_lora_parse = parse
        if invert is not None:
            self.dec_lora_inv_chk.blockSignals(True)
            self.dec_lora_inv_chk.setChecked(bool(invert))
            self.dec_lora_inv_chk.blockSignals(False)
            self.dec_lora_invert = bool(invert)

    def decoder_lora_manual(self, what, value):
        """A box was changed by hand: apply it and mark the preset as 'Custom'."""
        if what == "sf":
            self.dec_lora_sf = int(value)
        elif what == "bw":
            self.dec_lora_bw = float(value)
        elif what == "sync":
            self.dec_lora_sync = value
        self.dec_lora_preset.blockSignals(True)
        self.dec_lora_preset.setCurrentIndex(0)
        self.dec_lora_preset.blockSignals(False)

    def decoder_lora_preset(self, idx):
        label, p = LORA_PRESETS[idx]
        if not p:
            return
        self.decoder_lora_set(sf=p["sf"], bw=p["bw"], sync=p.get("sync"), parse=p.get("parse", "auto"), invert=p.get("invert", False))
        freq = p.get("freq")
        if p.get("mt"):
            freq = mt_default_freq(self.dec_lora_region, p["mt"], p["bw"] / 1e3)
        if freq:
            self.freq_selector.setValue(int(round(freq * 1e6)), emit_signal=True)
            self.log(f"LoRa preset: {label.strip()} -> {freq:.4f} MHz")

    def decoder_lora_region_changed(self, name):
        self.dec_lora_region = name
        idx = self.dec_lora_preset.currentIndex()
        if idx > 0 and LORA_PRESETS[idx][1] and LORA_PRESETS[idx][1].get("mt"):
            self.decoder_lora_preset(idx)                      # re-apply: the default slot depends on the region

    def decoder_lora_keys(self):
        dlg = QtWidgets.QDialog(self)
        dlg.setWindowTitle("LoRa decoder keys")
        form = QtWidgets.QFormLayout(dlg)
        spec = [("mt", "Meshtastic channel key(s)", "base64 or hex, comma separated. AQ== (the public default key) is always tried as well."),
                ("mt_names", "Meshtastic channel name(s)", "optional, comma separated: lets the channel hash byte identify the channel"),
                ("nwk", "LoRaWAN NwkSKey", "32 hex digits: checks the MIC and decrypts MAC commands (FPort 0)"),
                ("app", "LoRaWAN AppSKey", "32 hex digits: decrypts the application payload (FPort 1..223)"),
                ("appkey", "LoRaWAN AppKey", "32 hex digits (OTAA): checks a join-request MIC and decrypts a join-accept")]
        edits = {}
        for key, label, tip in spec:
            e = QtWidgets.QLineEdit(self.dec_keys.get(key, ""))
            e.setToolTip(tip)
            e.setMinimumWidth(420)
            form.addRow(label, e)
            edits[key] = e
        form.addRow(QtWidgets.QLabel("Keys stay in this program's memory only (they are not saved)."))
        bb = QtWidgets.QDialogButtonBox(QtWidgets.QDialogButtonBox.Ok | QtWidgets.QDialogButtonBox.Cancel)
        bb.accepted.connect(dlg.accept)
        bb.rejected.connect(dlg.reject)
        form.addRow(bb)
        if dlg.exec_() == QtWidgets.QDialog.Accepted:
            for key in edits:
                self.dec_keys[key] = edits[key].text().strip()
            self._lora_cfg = self.decoder_lora_cfg()

    def decoder_main(self, session):
        """Worker thread entry: settle the radio, then run the decoder loop. Always stops the async stream and reports back."""
        try:
            rate = float(self.SAMPLE_RATE)
            off = rate / DECODER_OFFSET_DIVISOR if self.dec_offset_tune else 0.0
            try:
                self.settle_radio(float(self.START_FREQ) + off)
            except Exception as e:
                print(f"decoder_main: settle_radio failed: {e!r}")
            self.decoder_loop(session)
        except Exception as e:
            import traceback
            print(f"decoder_main: unhandled error {e!r}")
            traceback.print_exc()
        finally:
            st = getattr(self, "_dec_stream", None)
            self._dec_stream = None
            if st is not None:
                st.stop()                                  # the sweep / scope threads that may follow use synchronous reads
            self.dec_signals.stopped.emit(session)

    #OLD# def decoder_main(self, session):
    #OLD#     """Worker thread entry: settle the radio, then run the decoder loop. Always reports back when it ends."""
    #OLD#     try:
    #OLD#         rate = float(self.SAMPLE_RATE)
    #OLD#         off = rate / DECODER_OFFSET_DIVISOR if self.dec_offset_tune else 0.0
    #OLD#         try:
    #OLD#             self.settle_radio(float(self.START_FREQ) + off)
    #OLD#         except Exception as e:
    #OLD#             print(f"decoder_main: settle_radio failed: {e!r}")
    #OLD#         self.decoder_loop(session)
    #OLD#     except Exception as e:
    #OLD#         import traceback
    #OLD#         print(f"decoder_main: unhandled error {e!r}")
    #OLD#         traceback.print_exc()
    #OLD#     finally:
    #OLD#         self.dec_signals.stopped.emit(session)
    #OLD#
    #OLD# # ---- decoder_loop(): version 2. The version 1 code is kept, commented out, just below ----

    def decoder_loop(self, session):
        sig = self.dec_signals
        window = np.hanning(NFFT).astype(np.float32)
        applied_tune = None
        need_settle = False
        cur_mode = None
        demod = None
        burst = None
        lora = None
        lora_params = None
        n_lora = 0
        n_lora_ok = 0
        phase = 0.0
        view_key = None
        marker_sent = None
        acc = None                  # spectrum accumulated since the last display update
        acc_n = 0
        wf_blocks = []              # waterfall frames (dB) since the last waterfall update
        t_ui = 0.0
        t_wf = 0.0
        lvl_db = -120.0
        sq_open = True
        n_bursts = 0
        errors = 0
        stream = None
        use_async = bool(DECODER_USE_ASYNC) and self.sdr is not None and hasattr(self.sdr, "read_bytes_async")
        if use_async:
            self.log("Decoder capture: asynchronous, gap-free (SdrStreamer)")
        else:
            self.log("Decoder capture: SYNCHRONOUS reads - samples arriving between reads are LOST (warble, truncated key frames). "
                     "pyrtlsdr_local has no read_bytes_async, or DECODER_USE_ASYNC is False")
        while (not self.stop_flag.is_set()) and self.dec_active and session == self._dec_session:
            try:
                if self.sdr is None:
                    break
                self._restart_pass = False                 # set by the sample-rate combo; meaningless here
                self.single_apply_pending()                # a new centre chosen on the front panel
                if getattr(self, "_applied_sample_rate", None) != self.SAMPLE_RATE:
                    print(f"decoder_loop: applying sample-rate change -> {self.SAMPLE_RATE / 1e6:.3f} MS/s")
                    if stream is not None:                 # a new sample rate always restarts the stream
                        stream.stop()
                        stream = None
                    self.sdr.sample_rate = self.SAMPLE_RATE
                    self._applied_sample_rate = self.SAMPLE_RATE
                    need_settle = True
                    cur_mode = None
                if stream is not None and not DECODER_LIVE_RETUNE and self._applied_gain_db != self.gain_db:
                    stream.stop()
                    stream = None
                self.apply_gain_if_changed()               # live RF Gain slider
                rate = float(self._applied_sample_rate)
                centre = float(self.START_FREQ)
                off = rate / DECODER_OFFSET_DIVISOR if self.dec_offset_tune else 0.0
                tune = centre + off
                if applied_tune != tune:
                    if stream is not None and not DECODER_LIVE_RETUNE:
                        stream.stop()
                        stream = None
                    self.sdr.center_freq = tune
                    applied_tune = tune
                    need_settle = True
                    cur_mode = None
                    acc, acc_n, wf_blocks = None, 0, []
                if use_async and stream is None:
                    stream = SdrStreamer(self.sdr)
                    stream.start()
                    self._dec_stream = stream              # decoder_main() stops it when the thread ends
                    need_settle = True
                if need_settle:
                    time.sleep(0.12)
                    if stream is not None:
                        stream.flush()                     # everything captured while the tuner settled is discarded
                    else:
                        _ = self.sdr.read_samples(512)
                        _ = self.sdr.read_samples(512)
                    need_settle = False
                if marker_sent != centre:
                    marker_sent = centre
                    sig.marker.emit(centre)
                if view_key != (tune, rate):                # keep the X axis on what the SDR is actually covering
                    view_key = (tune, rate)
                    try:
                        self.view_signals.apply.emit(tune - rate / 2.0, tune + rate / 2.0, float("nan"), float("nan"))
                    except Exception:
                        pass
                mode = self.dec_mode
                if mode is None:
                    break
                if mode != cur_mode:
                    demod = AudioDemod(rate, mode) if mode in ("WFM", "NFM", "AM") else None
                    burst = (BurstDecoder(rate, mode, bw_hz=self.dec_burst_bw * 1e3) if mode == "OOK"
                             else BurstDecoder(rate, mode) if mode == "FSK" else None)
                    cur_mode = mode
                    lora = None
                    acc, acc_n = None, 0
                    if demod is not None and rate > 2.1e6 and not self._dec_hinted:
                        self._dec_hinted = True
                        self.log("Decoder hint: for audio, 1.024 or 2.048 MS/s needs less CPU and USB bandwidth than "
                                 "2.4 MS/s and above, and is less likely to glitch")
                if burst is not None and burst.mode == "OOK" and abs(burst.bw_hz - self.dec_burst_bw * 1e3) > 1.0:
                    old_b = burst                          # channel width changed on the front panel: rebuild, keep the learned timing
                    burst = BurstDecoder(rate, "OOK", bw_hz=self.dec_burst_bw * 1e3)
                    burst.profile, burst.last_nbits = old_b.profile, old_b.last_nbits
                lite = bool(self.dec_lite) and demod is not None      # audio: spend as little time as possible on the display
                stride = 4 if lite else 1

                # raw IQ (no DC removal / IQ balance: they would remove an AM carrier), continuous from the async stream
                raw = stream.read(DECODER_BLOCK) if stream is not None else self._scope_read_iq(DECODER_BLOCK)
                errors = 0
                n = raw.size
                if off != 0.0:                             # mix the offset-tuned signal back down to 0 Hz
                    w = 2.0 * math.pi * off / rate
                    iq = (raw * np.exp(1j * (phase + w * np.arange(n))).astype(np.complex64)).astype(np.complex64)
                    phase = (phase + w * n) % (2.0 * math.pi)
                else:
                    iq = raw
                    phase = 0.0

                # ---- decode ----
                if demod is not None:
                    audio, level = demod.process(iq)
                    lvl_db = 20.0 * math.log10(level + 1e-9)
                    sq_open = not (self.dec_squelch_db > -79.5 and lvl_db < self.dec_squelch_db)
                    if not sq_open:
                        audio = np.zeros_like(audio)
                    gain = (self.dec_volume ** 2) * 2.0
                    pcm = (np.clip(audio * gain, -1.0, 1.0) * 32767.0).astype(np.int16)
                    if pcm.size:
                        sig.audio.emit(pcm.tobytes())
                elif mode == "LORA":
                    sync_sel = self.dec_lora_sync
                    params = (int(self.dec_lora_sf), float(self.dec_lora_bw), rate, centre)
                    if lora is None or params != lora_params:
                        lora = LoraRx(params[0], params[1], rate, rf_freq=centre)       # (re)build when SF / BW / rate / frequency change
                        lora_params = params
                    x_in = np.conj(iq) if self.dec_lora_invert else iq
                    for fr in lora.process(x_in):
                        n_lora += 1
                        n_lora_ok += 1 if fr["crc_ok"] is not False else 0
                        if sync_sel is not None and fr.get("sync") != sync_sel:
                            continue
                        if fr["crc_ok"] is False and not self.dec_lora_show_bad:
                            continue
                        sig.burst.emit({"mode": "LORA", "hex": "", "lines": lora_describe(fr, self.dec_lora_parse, self._lora_cfg)})
                    lvl_db = 20.0 * math.log10(float(np.mean(np.abs(iq))) + 1e-12)
                else:
                    if self.dec_reset_profile:
                        burst.profile = None
                        burst.last_nbits = None
                        self.dec_reset_profile = False
                    if self.dec_dump_request:
                        self.dec_dump_request = False
                        sig.burst.emit({"dump_text": burst.dump_text(), "mode": burst.mode})
                    burst.sens = float(self.dec_burst_sens)
                    burst.strip_sync = bool(self.dec_strip_sync)
                    for r in burst.process(iq):
                        n_bursts += 1
                        sig.burst.emit(r)
                    lvl_db = 20.0 * math.log10((burst.noise or 1e-6) + 1e-9)

                # ---- spectrum (from the raw capture, so it shows what the SDR really covers) ----
                m = (n // NFFT) * NFFT
                if m >= NFFT:
                    seg = raw[:m].reshape(-1, NFFT)[::stride] * window[None, :]     # light display: every 4th frame only
                    spec = np.fft.fftshift(np.fft.fft(seg, axis=1), axes=1) / NFFT
                    pw = spec.real ** 2 + spec.imag ** 2
                    if burst is not None or lora is not None:      # bursts are short: keep the peak, not the average
                        bp = pw.max(axis=0)
                        acc = bp if acc is None else np.maximum(acc, bp)
                    else:
                        bp = pw.mean(axis=0)
                        acc = bp if acc is None else acc + bp
                    acc_n += 1
                    if self.wf_enabled:
                        wf_blocks.append(10.0 * np.log10(pw + 1e-12) + self._psd_to_dbuv_const + self.cal_total_offset_db())

                now = time.time()
                if now - t_ui >= (0.30 if lite else DECODER_UI_INTERVAL_S) and acc is not None:
                    t_ui = now
                    mean_pw = acc if (burst is not None or lora is not None) else acc / float(max(acc_n, 1))
                    psd = 10.0 * np.log10(mean_pw + 1e-12) + self._psd_to_dbuv_const
                    freqs = np.fft.fftshift(np.fft.fftfreq(NFFT, d=1.0 / rate)) + tune
                    self.freq_axis = freqs.astype(np.float64)
                    self.power_axis = psd.astype(np.float64)
                    self.update_plot()
                    acc, acc_n = None, 0
                    if not self.wf_enabled:
                        wf_blocks = []
                    elif wf_blocks and ((not lite) or now - t_wf >= 0.6):
                        t_wf = now
                        newest_first = np.concatenate([b[::-1] for b in wf_blocks[::-1]], axis=0)
                        f_r, rows = self.wf_reduce(freqs, newest_first)
                        self.wf_signals.rows.emit(f_r, np.asarray(rows, dtype=np.float32), f"fast:{tune:.0f}",
                                                  float(NFFT * stride) / rate)
                        wf_blocks = []
                    cap = (f"stream OK, overruns {stream.overruns}" if stream is not None else "SYNC reads (samples lost!)")
                    if mode == "LORA":
                        sy = "any" if self.dec_lora_sync is None else f"0x{self.dec_lora_sync:02X}"
                        txt = (f"LoRa SF{int(self.dec_lora_sf)}/BW{self.dec_lora_bw / 1e3:g}  sync {sy}  {centre / 1e6:.6f} MHz   "
                               f"level {lvl_db:6.1f} dBFS   preambles seen {lora.detections if lora else 0}   frames {n_lora} (CRC ok {n_lora_ok})   {cap}")
                    elif demod is not None:
                        txt = (f"{cur_mode}  {centre / 1e6:.6f} MHz   channel level {lvl_db:6.1f} dBFS   "
                               f"squelch {'open' if sq_open else 'CLOSED'}   volume {int(self.dec_volume * 100)}%   {cap}")
                    else:
                        txt = (f"{cur_mode} burst decoder  {centre / 1e6:.6f} MHz   noise floor {lvl_db:6.1f} dBFS   "
                               f"threshold x{self.dec_burst_sens:.1f}   reports: {n_bursts}   {cap}")
                    sig.status.emit(txt)
            except OSError as e:
                errors += 1
                print(f"decoder_loop: read error: {e}")
                if errors > 30 or self.stop_flag.is_set():
                    break
                try:
                    if stream is not None:
                        stream.stop()
                        stream = None
                    self.init_sdr()                        # reopen the device and carry on
                    applied_tune = None
                    need_settle = True
                    cur_mode = None
                except Exception as e2:
                    print(f"decoder_loop: could not reopen the SDR: {e2}")
                    time.sleep(0.5)
            except Exception as e:
                import traceback
                errors += 1
                print(f"decoder_loop: error {e!r}")
                traceback.print_exc()
                if errors > 30:
                    break
                time.sleep(0.1)

    #OLD# def decoder_loop(self, session):
    #OLD#     sig = self.dec_signals
    #OLD#     window = np.hanning(NFFT).astype(np.float32)
    #OLD#     applied_tune = None
    #OLD#     need_settle = False
    #OLD#     cur_mode = None
    #OLD#     demod = None
    #OLD#     burst = None
    #OLD#     phase = 0.0
    #OLD#     view_key = None
    #OLD#     marker_sent = None
    #OLD#     acc = None                  # spectrum accumulated since the last display update
    #OLD#     acc_n = 0
    #OLD#     wf_blocks = []              # waterfall frames (dB) since the last display update
    #OLD#     t_ui = 0.0
    #OLD#     lvl_db = -120.0
    #OLD#     sq_open = True
    #OLD#     n_bursts = 0
    #OLD#     errors = 0
    #OLD#     while (not self.stop_flag.is_set()) and self.dec_active and session == self._dec_session:
    #OLD#         try:
    #OLD#             if self.sdr is None:
    #OLD#                 break
    #OLD#             self._restart_pass = False                 # set by the sample-rate combo; meaningless here
    #OLD#             self.single_apply_pending()                # a new centre chosen on the front panel
    #OLD#             if getattr(self, "_applied_sample_rate", None) != self.SAMPLE_RATE:
    #OLD#                 print(f"decoder_loop: applying sample-rate change -> {self.SAMPLE_RATE / 1e6:.3f} MS/s")
    #OLD#                 self.sdr.sample_rate = self.SAMPLE_RATE
    #OLD#                 self._applied_sample_rate = self.SAMPLE_RATE
    #OLD#                 need_settle = True
    #OLD#                 cur_mode = None
    #OLD#             self.apply_gain_if_changed()               # live RF Gain slider
    #OLD#             rate = float(self._applied_sample_rate)
    #OLD#             centre = float(self.START_FREQ)
    #OLD#             off = rate / DECODER_OFFSET_DIVISOR if self.dec_offset_tune else 0.0
    #OLD#             tune = centre + off
    #OLD#             if applied_tune != tune:
    #OLD#                 self.sdr.center_freq = tune
    #OLD#                 applied_tune = tune
    #OLD#                 need_settle = True
    #OLD#                 cur_mode = None
    #OLD#                 acc, acc_n, wf_blocks = None, 0, []
    #OLD#             if need_settle:
    #OLD#                 time.sleep(0.12)
    #OLD#                 _ = self.sdr.read_samples(512)
    #OLD#                 _ = self.sdr.read_samples(512)
    #OLD#                 need_settle = False
    #OLD#             if marker_sent != centre:
    #OLD#                 marker_sent = centre
    #OLD#                 sig.marker.emit(centre)
    #OLD#             if view_key != (tune, rate):                # keep the X axis on what the SDR is actually covering
    #OLD#                 view_key = (tune, rate)
    #OLD#                 try:
    #OLD#                     self.view_signals.apply.emit(tune - rate / 2.0, tune + rate / 2.0, float("nan"), float("nan"))
    #OLD#                 except Exception:
    #OLD#                     pass
    #OLD#             mode = self.dec_mode
    #OLD#             if mode is None:
    #OLD#                 break
    #OLD#             if mode != cur_mode:
    #OLD#                 demod = AudioDemod(rate, mode) if mode in ("WFM", "NFM", "AM") else None
    #OLD#                 burst = BurstDecoder(rate, mode) if mode in ("OOK", "FSK") else None
    #OLD#                 cur_mode = mode
    #OLD#                 acc, acc_n = None, 0
    #OLD#
    #OLD#             raw = self._scope_read_iq(DECODER_BLOCK)   # raw IQ: no DC removal / IQ balance (they would remove an AM carrier)
    #OLD#             errors = 0
    #OLD#             n = raw.size
    #OLD#             if off != 0.0:                             # mix the offset-tuned signal back down to 0 Hz
    #OLD#                 w = 2.0 * math.pi * off / rate
    #OLD#                 iq = (raw * np.exp(1j * (phase + w * np.arange(n))).astype(np.complex64)).astype(np.complex64)
    #OLD#                 phase = (phase + w * n) % (2.0 * math.pi)
    #OLD#             else:
    #OLD#                 iq = raw
    #OLD#                 phase = 0.0
    #OLD#
    #OLD#             # ---- decode ----
    #OLD#             if demod is not None:
    #OLD#                 audio, level = demod.process(iq)
    #OLD#                 lvl_db = 20.0 * math.log10(level + 1e-9)
    #OLD#                 sq_open = not (self.dec_squelch_db > -79.5 and lvl_db < self.dec_squelch_db)
    #OLD#                 if not sq_open:
    #OLD#                     audio = np.zeros_like(audio)
    #OLD#                 gain = (self.dec_volume ** 2) * 2.0
    #OLD#                 pcm = (np.clip(audio * gain, -1.0, 1.0) * 32767.0).astype(np.int16)
    #OLD#                 if pcm.size:
    #OLD#                     sig.audio.emit(pcm.tobytes())
    #OLD#             else:
    #OLD#                 burst.sens = float(self.dec_burst_sens)
    #OLD#                 for r in burst.process(iq):
    #OLD#                     n_bursts += 1
    #OLD#                     sig.burst.emit(r)
    #OLD#                 lvl_db = 20.0 * math.log10((burst.noise or 1e-6) + 1e-9)
    #OLD#
    #OLD#             # ---- spectrum (from the raw capture, so it shows what the SDR really covers) ----
    #OLD#             m = (n // NFFT) * NFFT
    #OLD#             if m >= NFFT:
    #OLD#                 seg = raw[:m].reshape(-1, NFFT) * window[None, :]
    #OLD#                 spec = np.fft.fftshift(np.fft.fft(seg, axis=1), axes=1) / NFFT
    #OLD#                 pw = spec.real ** 2 + spec.imag ** 2
    #OLD#                 if burst is not None:                  # bursts are short: keep the peak, not the average
    #OLD#                     bp = pw.max(axis=0)
    #OLD#                     acc = bp if acc is None else np.maximum(acc, bp)
    #OLD#                 else:
    #OLD#                     bp = pw.mean(axis=0)
    #OLD#                     acc = bp if acc is None else acc + bp
    #OLD#                 acc_n += 1
    #OLD#                 if self.wf_enabled:
    #OLD#                     wf_blocks.append(10.0 * np.log10(pw + 1e-12) + self._psd_to_dbuv_const + self.cal_total_offset_db())
    #OLD#
    #OLD#             now = time.time()
    #OLD#             if now - t_ui >= DECODER_UI_INTERVAL_S and acc is not None:
    #OLD#                 t_ui = now
    #OLD#                 mean_pw = acc if burst is not None else acc / float(max(acc_n, 1))
    #OLD#                 psd = 10.0 * np.log10(mean_pw + 1e-12) + self._psd_to_dbuv_const
    #OLD#                 freqs = np.fft.fftshift(np.fft.fftfreq(NFFT, d=1.0 / rate)) + tune
    #OLD#                 self.freq_axis = freqs.astype(np.float64)
    #OLD#                 self.power_axis = psd.astype(np.float64)
    #OLD#                 self.update_plot()
    #OLD#                 acc, acc_n = None, 0
    #OLD#                 if wf_blocks and self.wf_enabled:
    #OLD#                     newest_first = np.concatenate([b[::-1] for b in wf_blocks[::-1]], axis=0)
    #OLD#                     f_r, rows = self.wf_reduce(freqs, newest_first)
    #OLD#                     self.wf_signals.rows.emit(f_r, np.asarray(rows, dtype=np.float32), f"fast:{tune:.0f}", float(NFFT) / rate)
    #OLD#                 wf_blocks = []
    #OLD#                 if demod is not None:
    #OLD#                     txt = (f"{cur_mode}  {centre / 1e6:.6f} MHz   channel level {lvl_db:6.1f} dBFS   "
    #OLD#                            f"squelch {'open' if sq_open else 'CLOSED'}   volume {int(self.dec_volume * 100)}%")
    #OLD#                 else:
    #OLD#                     txt = (f"{cur_mode} burst decoder  {centre / 1e6:.6f} MHz   noise floor {lvl_db:6.1f} dBFS   "
    #OLD#                            f"threshold x{self.dec_burst_sens:.1f}   bursts found: {n_bursts}")
    #OLD#                 sig.status.emit(txt)
    #OLD#         except OSError as e:
    #OLD#             errors += 1
    #OLD#             print(f"decoder_loop: read error: {e}")
    #OLD#             if errors > 30 or self.stop_flag.is_set():
    #OLD#                 break
    #OLD#             try:
    #OLD#                 self.init_sdr()                        # reopen the device and carry on
    #OLD#                 applied_tune = None
    #OLD#                 need_settle = True
    #OLD#                 cur_mode = None
    #OLD#             except Exception as e2:
    #OLD#                 print(f"decoder_loop: could not reopen the SDR: {e2}")
    #OLD#                 time.sleep(0.5)
    #OLD#         except Exception as e:
    #OLD#             import traceback
    #OLD#             errors += 1
    #OLD#             print(f"decoder_loop: error {e!r}")
    #OLD#             traceback.print_exc()
    #OLD#             if errors > 30:
    #OLD#                 break
    #OLD#             time.sleep(0.1)

    # ------------------------------------------------------------------------------------
    # CAL SURVEY: how does the displayed level of a KNOWN tone vary with sample rate and RF gain?
    # ------------------------------------------------------------------------------------
    # Feed a sine of known level (default 1.5 MHz, 0.11 Vpp, via an attenuator) and press CAL SURVEY.
    # For every sample rate x RF gain (x tone position in the block) the program measures the tone and converts it
    # with the SAME formula the main display uses:
    #       dBuV = SDR dB - RF gain + attenuator + factor of the sample rate   (see "HOW THE dB -> dBuV CALIBRATION FACTORS ARE USED" near the top)
    # then compares it with the level the tone really has (from Vpp). If one CAL_SDR_DB_TO_DBUV were right
    # everywhere, every rate/gain would read the same. The report shows where that holds, where it does not,
    # and which CAL_SDR_DB_TO_DBUV would be right for the region that does agree.
    #
    # Two things in the measurement are deliberately controlled:
    #   * Position in the block. By default each rate is tuned so the tone sits at the SAME offset above the block
    #     centre (200 kHz) - otherwise the tone would land at a different place in the tuner's IF passband at every
    #     rate, and a passband ripple / roll-off would be mistaken for a sample-rate dependence. Enter several
    #     offsets (e.g. -300,-150,150,300) to map the passband at every rate. Or tick the MODE-band option to
    #     measure it exactly as the main display does (stitched blocks, whatever offset it falls at).
    #   * Scalloping. A tone that falls between two FFT bins reads up to 1.4 dB low in the peak bin, by an amount
    #     that changes with sample rate (the bin width does). The tone level is therefore taken from the power summed
    #     over the tone's whole lobe, expressed as the on-bin peak the display would show. The raw peak-bin level
    #     is recorded as well.

    def cal_build_ui(self):
        """Create the CAL and CAL SURVEY buttons next to GAIN/RATE SURVEY and the signals (called from survey_connect_signals)."""
        self._cal = None
        self.cal_signals = CalSurveySignals()
        self.cal_signals.done.connect(self.cal_finish)
        self.cal_signals.progress.connect(self.survey_sync_controls)
        self.btn_cal_quick = QtWidgets.QPushButton("CAL...")
        self.btn_cal_quick.setToolTip(
            "Calibrate the dB -> dBuV factor of EVERY sample rate: feed a sine of known frequency (default 1.5 MHz) and amplitude; each rate is measured at "
            "the RF gain set on the front panel and the best factor is worked out. Set the RF gain to about the value you will use for measurements first. "
            "You are asked before anything is applied or written to settings.csv.")
        self.btn_cal_quick.clicked.connect(self.cal_quick_open_dialog)
        self.btn_cal = QtWidgets.QPushButton("CAL SURVEY...")
        self.btn_cal.setToolTip(
            "Diagnostic: feed a known sine (default 1.5 MHz) and measure how its displayed dBuV changes with sample rate and RF gain. "
            "Writes an HTML report and CSV files. Press STOP to end early (partial results are saved).")
        self.btn_cal.clicked.connect(self.cal_open_dialog)
        sub = self.find_layout_with(self.btn_survey)
        if sub is not None:
            at = sub.indexOf(self.btn_survey) + 1
            sub.insertWidget(at, self.btn_cal_quick)
            sub.insertWidget(at + 1, self.btn_cal)
        else:
            self.centralWidget().layout().addWidget(self.btn_cal_quick)
            self.centralWidget().layout().addWidget(self.btn_cal)
        self.mode_buttons.append(self.btn_cal_quick)   # join the controls that are locked while a survey / the scope runs
        self.mode_buttons.append(self.btn_cal)

    def find_layout_with(self, widget):
        """The (nested) layout that directly holds this widget, searched two levels down from the central layout."""
        main_layout = self.centralWidget().layout()
        for i in range(main_layout.count()):
            lay = main_layout.itemAt(i).layout()
            if lay is None:
                continue
            if lay.indexOf(widget) >= 0:
                return lay
            for j in range(lay.count()):
                sub = lay.itemAt(j).layout()
                if sub is not None and sub.indexOf(widget) >= 0:
                    return sub
        return None

    def cal_busy(self):
        return (self._survey is not None or self._cal is not None or self._stitch is not None
                or getattr(self, "scope_active", False))

    @staticmethod
    def cal_expected_dbuv(vpp):
        """dBuV of a sine of this peak-to-peak voltage: 20*log10(Vrms / 1 uV)."""
        return 20.0 * math.log10(max(float(vpp), 1e-12) / (2.0 * math.sqrt(2.0)) / 1e-6)

    def cal_source_rows(self, form, on_change, defaults):
        """Add the test-signal rows to a form: generator amplitude (V pk-pk, as MEASURED at the generator output), the series (dropper)
        resistor and the input impedance it works into (the 50 ohm attenuator). The level at the attenuator input is worked out by
        cal_source_dbuv() and shown. Values used last time are offered again. Returns get(), a function giving a dict:
        vpp_gen, r_series, z_in, vpp_in (V pk-pk at the attenuator input), dbuv (its level), text (a description for the report)."""
        last = getattr(self, "_cal_source_last", None)
        vpp0, rs0, zin0 = (last["vpp_gen"], last["r_series"], last["z_in"]) if last else defaults
        sp_vpp = QtWidgets.QDoubleSpinBox()
        sp_vpp.setLocale(SCOPE_LOCALE)
        sp_vpp.setRange(0.0001, 50.0)
        sp_vpp.setDecimals(4)
        sp_vpp.setSuffix(" V pk-pk")
        sp_vpp.setValue(vpp0)
        sp_vpp.setToolTip("Peak-to-peak amplitude of the signal generator, as measured at its OUTPUT with the scope - with a load comparable to\n"
                          "what it really drives (series resistor + input impedance). A 10 kohm scope load does that for a 10 kohm dropper.\n"
                          "A generator with a high output impedance reads lower when loaded, so measure it loaded, as it will be used.")
        sp_rs = QtWidgets.QDoubleSpinBox()
        sp_rs.setLocale(SCOPE_LOCALE)
        sp_rs.setRange(0.0, 1.0e7)
        sp_rs.setDecimals(1)
        sp_rs.setSingleStep(100.0)
        sp_rs.setSuffix(" ohm")
        sp_rs.setValue(rs0)
        sp_rs.setToolTip("Series (dropper) resistor between the generator and the attenuator input. 0 = the generator drives the attenuator directly.")
        sp_zin = QtWidgets.QDoubleSpinBox()
        sp_zin.setLocale(SCOPE_LOCALE)
        sp_zin.setRange(1.0, 10000.0)
        sp_zin.setDecimals(1)
        sp_zin.setSuffix(" ohm")
        sp_zin.setValue(zin0)
        sp_zin.setToolTip("Input impedance of the external attenuator / SDR that the series resistor works into (50 ohm).")
        lbl = QtWidgets.QLabel("")

        def get():
            dbuv, vin_pp, vrms = cal_source_dbuv(sp_vpp.value(), sp_rs.value(), sp_zin.value())
            text = (f"generator {sp_vpp.value():g} Vpp" + (f" through {sp_rs.value():g} ohm into {sp_zin.value():g} ohm = {vin_pp * 1e3:.4g} mVpp "
                    f"at the attenuator input" if sp_rs.value() > 0 else " direct to the attenuator input"))
            return {"vpp_gen": sp_vpp.value(), "r_series": sp_rs.value(), "z_in": sp_zin.value(), "vpp_in": vin_pp, "dbuv": dbuv, "text": text}

        def update(*_):
            g = get()
            lbl.setText(f"<b>{g['dbuv']:.2f} dBuV</b>   ({g['vpp_in'] * 1e3:.4g} mV pk-pk = {g['vpp_in'] / (2 * math.sqrt(2)) * 1e3:.4g} mV rms)")
            on_change()

        form.addRow("Generator amplitude (V pk-pk, measured at the generator output)", sp_vpp)
        form.addRow("Series (dropper) resistor", sp_rs)
        form.addRow("Input impedance it feeds (the attenuator)", sp_zin)
        form.addRow("= Level at the attenuator input", lbl)
        for sp in (sp_vpp, sp_rs, sp_zin):
            sp.valueChanged.connect(update)
        g0 = get()
        lbl.setText(f"<b>{g0['dbuv']:.2f} dBuV</b>   ({g0['vpp_in'] * 1e3:.4g} mV pk-pk = {g0['vpp_in'] / (2 * math.sqrt(2)) * 1e3:.4g} mV rms)")
        return get

    @staticmethod
    def cal_parse_offsets(text):
        """'200' or '-300, -150; 150, 300' -> list of kHz floats. Raises ValueError on rubbish."""
        vals = []
        for t in str(text).replace(";", ",").split(","):
            t = t.strip()
            if t:
                vals.append(float(t))
        if not vals:
            raise ValueError("no offsets given")
        return vals

    def cal_open_dialog(self):
        if self._survey is not None or self._cal is not None or self._stitch is not None or getattr(self, "scope_active", False):
            self.log("CAL survey: not available while another survey, the stitch check or the scope is running")
            return
        if self.hf_mode:
            QtWidgets.QMessageBox.information(self, "Cal survey", "Not available in an HF (direct-sampling) mode - pick a tuner MODE first.")
            return
        use_band_ok = (self.START_FREQ != self.STOP_FREQ) and not getattr(self, "single_mode", False)
        dlg = QtWidgets.QDialog(self)
        dlg.setWindowTitle("Calibration survey - known tone")
        lay = QtWidgets.QVBoxLayout(dlg)
        lay.addWidget(QtWidgets.QLabel(
            "Feed a sine of known level. Every sample rate x RF gain is measured (scanned several times, median kept)\n"
            "and the tone's displayed dBuV is compared with its true level. Averaging setting is used as-is."))
        form = QtWidgets.QFormLayout()
        sp_tone = QtWidgets.QDoubleSpinBox()
        sp_tone.setLocale(SCOPE_LOCALE)
        sp_tone.setRange(0.01, 30.0)
        sp_tone.setDecimals(4)
        sp_tone.setSuffix(" MHz")
        sp_tone.setValue(CAL_TONE_HZ_DEFAULT / 1e6)
        sp_att = QtWidgets.QDoubleSpinBox()
        sp_att.setLocale(SCOPE_LOCALE)
        sp_att.setRange(0.0, 100.0)
        sp_att.setDecimals(1)
        sp_att.setSuffix(" dB")
        sp_att.setValue(float(self.input_atten_db))
        sp_att.setToolTip("Attenuator between the generator and the SDR. Defaults to the front-panel Attenuator setting.")
        sp_win = QtWidgets.QDoubleSpinBox()
        sp_win.setLocale(SCOPE_LOCALE)
        sp_win.setRange(1.0, 500.0)
        sp_win.setDecimals(1)
        sp_win.setSuffix(" kHz")
        sp_win.setValue(CAL_WINDOW_HZ_DEFAULT / 1e3)
        sp_win.setToolTip("The tone is looked for within +/- this of its nominal frequency.")
        chk_band = QtWidgets.QCheckBox("Measure on the current MODE band (stitched blocks, as the main display)")
        chk_band.setEnabled(use_band_ok)
        chk_band.setToolTip("Unticked (default): one block per measurement with the tone at a fixed offset from the block "
                            "centre, so the tone is always in the same part of the tuner passband at every sample rate.")
        ed_off = QtWidgets.QLineEdit(CAL_OFFSET_KHZ_DEFAULT)
        ed_off.setToolTip("Tone position above the block centre, kHz (negative = below). Several values separated by commas "
                          "measure the passband shape at every sample rate. Offsets outside +/-40% of a rate are skipped.")
        chk_band.toggled.connect(lambda on: ed_off.setEnabled(not on))
        gain_texts = [f"{g:.1f} dB" for g in RTL_GAIN_STEPS_DB]
        cb_lo, cb_hi = QtWidgets.QComboBox(), QtWidgets.QComboBox()
        cb_lo.addItems(gain_texts)
        cb_hi.addItems(gain_texts)
        cb_lo.setCurrentIndex(0)
        cb_hi.setCurrentIndex(len(gain_texts) - 1)
        sp_stride = QtWidgets.QSpinBox()
        sp_stride.setRange(1, 10)
        sp_stride.setValue(1)
        sp_scans = QtWidgets.QSpinBox()
        sp_scans.setRange(1, 5)
        sp_scans.setValue(CAL_SCANS_DEFAULT)
        form.addRow("Tone frequency", sp_tone)
        src_get = self.cal_source_rows(form, lambda: refresh_info(), (CAL_TONE_VPP_DEFAULT, 0.0, 50.0))
        form.addRow("Attenuator fitted", sp_att)
        form.addRow("Search window", sp_win)
        form.addRow("", chk_band)
        form.addRow("Tone offset from block centre (kHz)", ed_off)
        form.addRow("Gain from", cb_lo)
        form.addRow("Gain to", cb_hi)
        form.addRow("Use every Nth gain step", sp_stride)
        form.addRow("Scans per setting", sp_scans)
        lay.addLayout(form)
        lay.addWidget(QtWidgets.QLabel("Sample rates to test:"))
        lst = QtWidgets.QListWidget()
        for r in SAMPLE_RATE_CHOICES:
            it = QtWidgets.QListWidgetItem(f"{r/1e6:.3f} MS/s")
            it.setData(QtCore.Qt.UserRole, float(r))
            it.setFlags(it.flags() | QtCore.Qt.ItemIsUserCheckable)
            it.setCheckState(QtCore.Qt.Checked if any(abs(r - q) < 1.0 for q in SAMPLE_RATE_RECOMMENDED) else QtCore.Qt.Unchecked)
            lst.addItem(it)
        lst.setMinimumHeight(150)
        lay.addWidget(lst)
        info = QtWidgets.QLabel("")
        lay.addWidget(info)

        def offsets_or_none():
            try:
                return self.cal_parse_offsets(ed_off.text())
            except ValueError:
                return None

        def chosen():
            lo, hi = sorted((cb_lo.currentIndex(), cb_hi.currentIndex()))
            gains = [float(g) for g in RTL_GAIN_STEPS_DB[lo:hi + 1:sp_stride.value()]]
            rates = [lst.item(i).data(QtCore.Qt.UserRole) for i in range(lst.count()) if lst.item(i).checkState() == QtCore.Qt.Checked]
            return rates, gains

        def valid_offsets(rate, offs):
            return [o for o in offs if abs(o * 1e3) <= CAL_MAX_OFFSET_FRAC * rate and sp_tone.value() * 1e6 - o * 1e3 > 50e3]

        def n_points():
            rates, gains = chosen()
            if chk_band.isChecked():
                return rates, gains, [(r, None) for r in rates]
            offs = offsets_or_none() or []
            return rates, gains, [(r, o) for r in rates for o in valid_offsets(r, offs)]

        def total_seconds():
            rates, gains, ro = n_points()
            n_avg = self.avg_get_active_count()
            tot = 0.0
            for r, _o in ro:
                nblk = len(self.build_centers(self.START_FREQ, self.STOP_FREQ, r)) if chk_band.isChecked() else 1
                per = nblk * (0.12 + 0.15 + n_avg * (NFFT / r + 0.004))
                tot += per * (len(gains) * sp_scans.value() + 1)
            return tot

        def refresh_info(*_):
            rates, gains, ro = n_points()
            bad = (not chk_band.isChecked()) and offsets_or_none() is None
            info.setText("Offsets must be numbers (kHz), comma separated." if bad else
                         f"{len(rates)} rates x {len(gains)} gains, {len(ro)} rate/offset combination(s), {sp_scans.value()} scan(s) each "
                         f"- roughly {self.survey_fmt_seconds(total_seconds())}")

        for wdg in (sp_tone, sp_att, sp_win):
            wdg.valueChanged.connect(refresh_info)
        for wdg in (cb_lo, cb_hi):
            wdg.currentIndexChanged.connect(refresh_info)
        sp_stride.valueChanged.connect(refresh_info)
        sp_scans.valueChanged.connect(refresh_info)
        ed_off.textChanged.connect(refresh_info)
        chk_band.toggled.connect(refresh_info)
        lst.itemChanged.connect(refresh_info)
        refresh_info()
        bb = QtWidgets.QDialogButtonBox(QtWidgets.QDialogButtonBox.Ok | QtWidgets.QDialogButtonBox.Cancel)
        bb.accepted.connect(dlg.accept)
        bb.rejected.connect(dlg.reject)
        lay.addWidget(bb)
        if dlg.exec_() != QtWidgets.QDialog.Accepted:
            return
        rates, gains = chosen()
        if not rates or not gains:
            self.log("CAL survey: nothing selected")
            return
        use_band = chk_band.isChecked()
        offs = [None]
        if not use_band:
            offs = offsets_or_none()
            if not offs:
                QtWidgets.QMessageBox.information(self, "Cal survey", "The tone offsets must be numbers (kHz), comma separated.")
                return
        combos = []
        for r in rates:
            for o in ([None] if use_band else valid_offsets(r, offs)):
                for g in gains:
                    combos.append((r, o, g))
        if not combos:
            self.log("CAL survey: no rate / offset combination is possible (offsets too large for the chosen rates?)")
            return
        path, _ = QtWidgets.QFileDialog.getSaveFileName(
            self, "Save calibration survey (base name; writes .html and _*.csv)", "rf_cal_survey.html", "HTML report (*.html)")
        if not path:
            return
        src = src_get()
        self._cal_source_last = src
        params = {"tone_hz": sp_tone.value() * 1e6, "vpp": src["vpp_in"], "atten": sp_att.value(), "source_text": src["text"],
                  "expected": src["dbuv"], "win_hz": sp_win.value() * 1e3,
                  "use_band": use_band, "combos": combos, "scans": sp_scans.value(), "path": path}
        self.cal_start(params)

    def cal_start(self, prm):
        if self._survey is not None or self._cal is not None:
            return
        quick = bool(prm.get("quick"))             # the CAL button: no files, just the per-rate result
        fh = None
        if not quick:
            base = os.path.splitext(prm["path"])[0]
            prm["base"] = base
            prm["html_path"] = base + ".html"
            try:
                fh = open(base + "_long.csv", "w", newline="")
            except Exception as e:
                self.log(f"CAL survey: cannot create result files: {e}")
                return
        was_running = bool(self.sweep_thread and self.sweep_thread.is_alive())
        if was_running:
            self.stop_flag.set()
            self.sweep_thread.join(timeout=3.0)
            if self.sweep_thread.is_alive():
                self.log("CAL survey: ERROR - the running sweep did not stop; try again")
                if fh is not None:
                    fh.close()
                return
        self.stop_flag.clear()
        try:
            self.init_sdr()
        except Exception as e:
            self.log(f"CAL survey: error opening the SDR: {e}")
            if fh is not None:
                fh.close()
            return
        w = csv.writer(fh) if fh is not None else None
        if w is not None:
            w.writerow(["sample_rate_hz", "tone_offset_khz", "block_offset_khz", "gain_db", "scan", "tone_freq_hz", "tone_dbuv",
                    "peak_bin_dbuv", "error_db", "implied_cal", "floor_dbuv", "snr_db", "strongest_freq_hz", "strongest_dbuv",
                    "stronger_signal_elsewhere", "n_avg"])
        prm.update({"fh": fh, "w": w, "results": [], "records": [], "idx": -1, "skipped": 0, "t_start": time.time(),
                    "was_running": was_running,
                    "orig": (self.START_FREQ, self.STOP_FREQ, self.SAMPLE_RATE, self.gain_db, bool(getattr(self, "single_mode", False))),
                    "n_avg": self.avg_get_active_count(), "cal0": CAL_SDR_DB_TO_DBUV})
        self._cal = prm
        self._survey = {"cal": True}                 # makes the stitch check / single-frequency mode / scope / old survey stand aside
        self.log(f"=== {'CAL' if quick else 'CAL SURVEY'} STARTED: {len(prm['combos'])} settings x {prm['scans']} scan(s), tone {prm['tone_hz']/1e6:.4f} MHz, "
                 f"true level {prm['expected']:.1f} dBuV" + ("" if quick else f" -> {prm['html_path']}") + " ===")
        self.survey_set_lock(True)
        self.sweep_thread = threading.Thread(target=self.cal_loop, daemon=True)
        self.sweep_thread.start()

    def cal_point_setup(self, rate, off_khz):
        """Worker thread: aim the sweep for this point. Single-block mode: one block centred so the tone is off_khz above
        the centre (START = STOP = centre; single_mode makes the whole captured bandwidth count). MODE-band mode: unchanged."""
        c = self._cal
        if c["use_band"]:
            return
        centre = c["tone_hz"] - off_khz * 1e3
        self.START_FREQ = self.STOP_FREQ = float(centre)
        self.centers = np.array([float(centre)], dtype=np.float64)

    def cal_loop(self):
        """Survey worker thread: for every rate / offset / gain take several complete scans with the normal sweep_loop(),
        measure the tone in each, keep the median, log and store the results."""
        c = self._cal
        aborted = False
        try:
            if not c["use_band"]:
                self.single_mode = True              # no edge masking, no stitching: the whole captured block is shown
            rate0, off0, gain0 = c["combos"][0]
            self.SAMPLE_RATE = rate0
            self.gain_db = gain0
            self.cal_point_setup(rate0, off0)
            self.cal_signals.progress.emit()
            c0 = self.centers if c["use_band"] is False else self.build_centers(self.START_FREQ, self.STOP_FREQ, rate0)
            self.sdr.sample_rate = rate0
            self._applied_sample_rate = rate0
            self.settle_radio(float(c0[0]))
            last_key = None
            n_all = len(c["combos"])
            for idx, (rate, off, gain) in enumerate(c["combos"]):
                if self.stop_flag.is_set():
                    aborted = True
                    break
                c["idx"] = idx
                self.SAMPLE_RATE = rate
                self.gain_db = gain
                self.cal_point_setup(rate, off)
                self.cal_signals.progress.emit()
                key = (rate, off)
                warm = 1 if key != last_key else 0      # the first scan after a rate / tuning change is a warm-up: thrown away
                last_key = key
                got = []
                tries = 0
                while len(got) < c["scans"] and tries < c["scans"] + warm + 3:
                    if self.stop_flag.is_set():
                        aborted = True
                        break
                    tries += 1
                    seq0 = self._pass_seq
                    self._restart_pass = False
                    self.sweep_loop()
                    lp = self._last_pass
                    if lp is None or lp["seq"] <= seq0 or lp["rate"] != rate or lp["gain"] != gain:
                        continue                        # not a clean scan: try again
                    if warm > 0:
                        warm -= 1
                        continue
                    got.append(self.cal_measure(lp, rate, off, gain, len(got) + 1))
                if aborted:
                    break
                if not got:
                    c["skipped"] += 1
                    self.log(f"CAL survey: no clean scan at {rate/1e6:.3f} MS/s / {off} kHz / {gain:.1f} dB - skipped")
                    continue
                res = self.cal_aggregate(got, rate, off, gain)
                c["results"].append(res)
                self.log(f"Cal {idx + 1}/{n_all}: {rate/1e6:.3f} MS/s, " + (f"tone at {off:+g} kHz, " if off is not None else "MODE band, ") +
                         f"{gain:.1f} dB -> {res['tone']:.1f} dBuV (error {res['err']:+.1f} dB)" +
                         ("  [stronger signal elsewhere]" if res["other"] else ""))
        except Exception as e:
            import traceback
            print(f"cal_loop: error {e!r}")
            traceback.print_exc()
            aborted = True
        finally:
            try:
                c["fh"].flush()
            except Exception:
                pass
            sf, st, sr, sg, sm = c["orig"]               # put the operator's settings back
            self.single_mode = sm
            self.START_FREQ, self.STOP_FREQ = sf, st
            self.SAMPLE_RATE, self.gain_db = sr, sg
            self.centers = (np.array([sf], dtype=np.float64) if sf == st else self.build_centers(sf, st, sr))
            # Put the RADIO back too. This is the worker thread, the only one allowed to touch the driver. Without this the radio stayed at
            # the LAST rate and gain of the CAL while the front panel showed the user's, and because the display's factor is looked up for
            # the rate the radio is applied at, the "Cal:" label and the scaling of the trace used the wrong rate's factor until something
            # re-applied the user's rate (a new sweep, or changing the sample rate).
            try:
                if self.sdr is not None:
                    self.sdr.sample_rate = sr
                    self._applied_sample_rate = sr
                    self.apply_gain_if_changed()                      # re-applies self.gain_db (= sg, restored above)
            except Exception as e:
                print(f"cal_loop: could not put the radio back to {sr/1e6:.3f} MS/s: {e!r}")
            # Drop the CAL's last spectrum too: it is one block taken at the last CAL rate, and must not be shown with the user's rate's
            # factor. (The sweep re-allocates its arrays when it next runs.)
            self.freq_axis = np.array([], dtype=np.float64)
            self.power_axis = np.array([], dtype=np.float64)
            self.cal_signals.done.emit(bool(aborted))

    @staticmethod
    def cal_tone_levels(f, p, rate, tone_hz, win_hz, ignore_hz=50e3):
        """Measure the test tone in one stitched spectrum (f in Hz, p in SDR dB, i.e. before the gain / attenuator / CAL
        conversion). Returns a dict: found, f_pk, pk_db (peak bin), tone_db (power summed over the tone's lobe, expressed as
        the on-bin peak - free of scalloping loss), floor_db (median), g_f / g_db (strongest bin anywhere) and
        other_louder (something clearly stronger than the tone, elsewhere in the spectrum)."""
        nan = float("nan")
        out = {"found": False, "f_pk": nan, "pk_db": nan, "tone_db": nan, "floor_db": nan, "g_f": nan, "g_db": nan,
               "other_louder": False}
        f = np.asarray(f, dtype=np.float64)
        p = np.asarray(p, dtype=np.float64)
        ok = np.isfinite(f) & np.isfinite(p) & (p > -190.0)
        f, p = f[ok], p[ok]
        if f.size < 20:
            return out
        order = np.argsort(f)
        f, p = f[order], p[order]
        floor = float(np.median(p))
        out["floor_db"] = floor
        # strongest bin anywhere - but not in the RF-DC region (|f| < ignore_hz): the V4 upconverter's LO feed-through and
        # the DC spike sit there and are not signals, so they must not be mistaken for a stronger signal than the tone
        away = np.flatnonzero(np.abs(f) > ignore_hz)
        ig = int(away[np.argmax(p[away])]) if away.size else int(np.argmax(p))
        out["g_f"], out["g_db"] = float(f[ig]), float(p[ig])
        sel = np.flatnonzero(np.abs(f - tone_hz) <= win_hz)
        if sel.size == 0:
            return out
        ipk = int(sel[np.argmax(p[sel])])
        df = float(rate) / NFFT
        lobe = np.abs(f - f[ipk]) <= (CAL_LOBE_BINS + 0.5) * df
        nb = int(np.count_nonzero(lobe))
        lin = float(np.sum(10.0 ** (p[lobe] / 10.0)))
        net = max(lin - nb * 10.0 ** (floor / 10.0), lin * 1e-3)      # take the noise inside the lobe out
        out["found"] = True
        out["f_pk"], out["pk_db"] = float(f[ipk]), float(p[ipk])
        out["tone_db"] = 10.0 * math.log10(net) - CAL_WINDOW_ENBW_DB
        out["other_louder"] = bool(out["g_db"] > out["pk_db"] + 1.0 and abs(out["g_f"] - out["f_pk"]) > 5.0 * df)
        return out

    def cal_measure(self, lp, rate, off, gain, scan_no):
        """Worker thread: measure the tone in one finished scan, convert to dBuV, store the record and one CSV row."""
        c = self._cal
        m = self.cal_tone_levels(lp["f"], lp["p"], rate, c["tone_hz"], c["win_hz"])
        cal_r = cal_for_rate(rate)                                         # step 1: the factor of THIS sample rate
        conv = cal_model_offset_db(cal_r, gain, c["atten"])               # step 2: SDR dB -> dBuV at the attenuator input (no step 3)
        nan = float("nan")
        blk = nan
        if c["use_band"]:
            cen = self.build_centers(self.START_FREQ, self.STOP_FREQ, rate)
            blk = (c["tone_hz"] - float(cen[int(np.argmin(np.abs(cen - c["tone_hz"])))])) / 1e3
        elif off is not None:
            blk = float(off)
        tone = m["tone_db"] + conv
        rec = {"rate": rate, "off": off, "blk": blk, "gain": gain, "scan": scan_no, "found": m["found"], "f_pk": m["f_pk"],
               "tone": tone, "pk": m["pk_db"] + conv, "err": tone - c["expected"], "implied": cal_solve_factor(c["expected"], m["tone_db"], gain, c["atten"]), "cal": cal_r, "tone_p": m["tone_db"],
               "floor": m["floor_db"] + conv, "snr": m["tone_db"] - m["floor_db"], "g_f": m["g_f"], "g_db": m["g_db"] + conv,
               "other": m["other_louder"], "n_avg": lp["n_avg"]}
        c["records"].append(rec)
        try:
            if c.get("w") is None:
                return rec
            c["w"].writerow([f"{rate:.0f}", "" if off is None else f"{off:g}", "" if blk != blk else f"{blk:.1f}", f"{gain:.1f}", scan_no,
                             f"{rec['f_pk']:.1f}", f"{rec['tone']:.2f}", f"{rec['pk']:.2f}", f"{rec['err']:.2f}", f"{rec['implied']:.2f}",
                             f"{rec['floor']:.2f}", f"{rec['snr']:.1f}", f"{rec['g_f']:.1f}", f"{rec['g_db']:.2f}",
                             int(rec["other"]), rec["n_avg"]])
            c["fh"].flush()
        except Exception as e:
            print("cal_measure: could not write CSV row:", e)
        return rec

    @staticmethod
    def cal_aggregate(recs, rate, off, gain):
        """Median of the scans of one setting (a tone that is not found gives NaN)."""
        def med(key):
            v = [r[key] for r in recs if r["found"] and r[key] == r[key]]
            return float(np.median(v)) if v else float("nan")
        tones = [r["tone"] for r in recs if r["found"] and r["tone"] == r["tone"]]
        return {"rate": rate, "off": off, "gain": gain, "n": len(recs), "tone": med("tone"), "pk": med("pk"), "err": med("err"),
                "implied": med("implied"), "cal": med("cal"), "tone_p": med("tone_p"), "floor": med("floor"), "snr": med("snr"), "blk": med("blk"),
                "spread": (max(tones) - min(tones)) if len(tones) > 1 else 0.0,
                "other": any(r["other"] for r in recs), "found": bool(tones)}

    # ---- analysis ----
    def cal_analyse(self, c):
        """Reduce the results to findings. Returns a list of (level, text) and, per tone offset, a dict of tables."""
        res = [r for r in c["results"] if r["found"]]
        exp = c["expected"]
        finds = []
        per_off = {}
        offs = sorted({r["off"] for r in res}, key=lambda o: (o is not None, o if o is not None else 0.0))
        for off in offs:
            rs = [r for r in res if r["off"] == off]
            rates = sorted({r["rate"] for r in rs})
            gains = sorted({r["gain"] for r in rs})
            M = {(r["rate"], r["gain"]): r for r in rs}
            name = "MODE band (stitched)" if off is None else f"tone {off:+g} kHz from block centre"
            # per rate: its own low-gain level and the gain at which it first moves away from it
            per_rate = []
            for r in rates:
                gs = [g for g in gains if (r, g) in M]
                if not gs:
                    continue
                ref = float(np.median([M[(r, g)]["tone"] for g in gs[:3]]))
                depart = next((g for g in gs if abs(M[(r, g)]["tone"] - ref) > CAL_DEPART_DB), None)
                errs = [M[(r, g)]["err"] for g in gs]
                per_rate.append({"rate": r, "ref": ref, "ref_err": ref - exp, "depart": depart, "emin": min(errs), "emax": max(errs),
                                 "implied_low": float(np.median([M[(r, g)]["cal"] for g in gs])) - (ref - exp)})
            refs = {p_["rate"]: p_["ref"] for p_ in per_rate}
            # gain range over which ONE constant works: every rate agrees AND each rate still holds its low-gain level
            # (contiguous from the lowest gain)
            gflat = None
            flat_cells = []
            for g in gains:
                cells = [M[(r, g)] for r in rates if (r, g) in M]
                vals = [x["tone"] for x in cells]
                if (len(cells) == len(rates) and (max(vals) - min(vals)) <= CAL_AGREE_DB
                        and all(abs(M[(r, g)]["tone"] - refs[r]) <= CAL_DEPART_DB for r in rates if r in refs)):
                    gflat = g
                    flat_cells += cells
                else:
                    break
            spread_by_gain = {g: (max(M[(r, g)]["tone"] for r in rates if (r, g) in M) - min(M[(r, g)]["tone"] for r in rates if (r, g) in M))
                              for g in gains if any((r, g) in M for r in rates)}
            per_off[off] = {"name": name, "rates": rates, "gains": gains, "M": M, "gflat": gflat, "per_rate": per_rate,
                            "spread_by_gain": spread_by_gain}
            if flat_cells:
                imp = [x["implied"] for x in flat_cells]
                finds.append(("ok", f"{name}: all {len(rates)} sample rate(s) agree within {CAL_AGREE_DB:g} dB AND hold their low-gain level, for RF gain {gains[0]:g} to {gflat:g} dB. "
                                    f"Over that region the dB-to-dBuV factor {np.median(imp):.1f} would read the tone correctly "
                                    f"(range {min(imp):.1f} to {max(imp):.1f})."))
            elif per_rate:
                lows = [p_["implied_low"] for p_ in per_rate]
                finds.append(("bad", f"{name}: no gain range where one constant works - at the lowest gain tested the rates already disagree by more than {CAL_AGREE_DB:g} dB (or the level moves at once). "
                                     f"The CAL value that would be right for each rate at low gain runs from {min(lows):.1f} to {max(lows):.1f} "
                                     f"(a spread of {max(lows) - min(lows):.1f} dB) - one constant cannot be right for all of them at this tone position."))
            for p_ in per_rate:
                if p_["depart"] is not None:
                    finds.append(("warn", f"{name}, {p_['rate']/1e6:.3f} MS/s: the level holds within {CAL_DEPART_DB:g} dB of its low-gain value until the gain reaches "
                                          f"{p_['depart']:g} dB, then departs "
                                          f"(range of error over all gains {p_['emin']:+.1f} to {p_['emax']:+.1f} dB)."))
        # does the tone position matter? compare offsets at the same rate and a low gain
        if len([o for o in offs if o is not None]) > 1:
            worst = 0.0
            for r in sorted({x["rate"] for x in res}):
                lv = []
                for o in offs:
                    if o is None:
                        continue
                    cell = per_off[o]["M"]
                    g0 = per_off[o]["gains"][0] if per_off[o]["gains"] else None
                    if g0 is not None and (r, g0) in cell:
                        lv.append(cell[(r, g0)]["tone"])
                if len(lv) > 1:
                    worst = max(worst, max(lv) - min(lv))
            finds.append(("warn" if worst > CAL_AGREE_DB else "ok",
                          f"Tone position in the block: at the lowest gain the level changes by up to {worst:.1f} dB between the offsets tested "
                          f"({'the tuner passband is not flat enough to ignore' if worst > CAL_AGREE_DB else 'passband flat to within the agreement limit'})."))
        n_other = sum(1 for r in c["results"] if r["other"])
        if n_other:
            finds.append(("warn", f"{n_other} setting(s) had a signal stronger than the tone elsewhere in the spectrum (spurs / birdies): the tone is "
                                  f"not the dominant signal there - see the * marks in the tables."))
        n_unst = sum(1 for r in c["results"] if r["spread"] > CAL_SPREAD_WARN_DB)
        if n_unst:
            finds.append(("warn", f"{n_unst} setting(s): repeat scans differed by more than {CAL_SPREAD_WARN_DB:g} dB (marked ~ in the tables)."))
        n_miss = sum(1 for r in c["results"] if not r["found"])
        if n_miss:
            finds.append(("bad", f"{n_miss} setting(s): no tone found within the search window."))
        if not finds:
            finds.append(("warn", "No results."))
        return finds, per_off

    @staticmethod
    def cal_svg_chart(title, gains, series, expected, w=940, h=320):
        """Inline SVG line chart: level against RF gain, one line per sample rate, dashed line = true level."""
        vals = [v for _n, _c, vs in series for v in vs if v is not None and v == v] + [expected]
        lo, hi = min(vals) - 2.0, max(vals) + 2.0
        if hi - lo < 6:
            hi, lo = hi + 3, lo - 3
        L, R, T, B = 58, 150, 26, 36
        gmin, gmax = min(gains), max(gains)
        span = (gmax - gmin) or 1.0
        X = lambda g: L + (g - gmin) / span * (w - L - R)
        Y = lambda v: T + (1.0 - (v - lo) / (hi - lo)) * (h - T - B)
        o = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{w}" height="{h}" viewBox="0 0 {w} {h}" '
             f'style="background:#14161a;border:1px solid #2a2d33;max-width:100%">',
             f'<text x="{L}" y="16" fill="#d8dbe0" font-size="13" font-family="sans-serif">{title}</text>']
        step = 5.0 if (hi - lo) <= 60 else 10.0
        v = math.ceil(lo / step) * step
        while v <= hi:
            o.append(f'<line x1="{L}" y1="{Y(v):.1f}" x2="{w - R}" y2="{Y(v):.1f}" stroke="#2a2d33"/>'
                     f'<text x="{L - 6}" y="{Y(v) + 4:.1f}" fill="#aaa" font-size="11" text-anchor="end" font-family="sans-serif">{v:g}</text>')
            v += step
        g = math.ceil(gmin / 5.0) * 5.0
        while g <= gmax:
            o.append(f'<line x1="{X(g):.1f}" y1="{T}" x2="{X(g):.1f}" y2="{h - B}" stroke="#23262c"/>'
                     f'<text x="{X(g):.1f}" y="{h - B + 14}" fill="#aaa" font-size="11" text-anchor="middle" font-family="sans-serif">{g:g}</text>')
            g += 5.0
        o.append(f'<text x="{(L + w - R) / 2:.0f}" y="{h - 6}" fill="#aaa" font-size="11" text-anchor="middle" font-family="sans-serif">RF gain (dB)</text>')
        o.append(f'<line x1="{L}" y1="{Y(expected):.1f}" x2="{w - R}" y2="{Y(expected):.1f}" stroke="#e0e0e0" stroke-dasharray="6 5"/>')
        o.append(f'<text x="{w - R + 8}" y="{Y(expected) + 4:.1f}" fill="#e0e0e0" font-size="11" font-family="sans-serif">true {expected:.1f} dBuV</text>')
        for k, (name, col, vs) in enumerate(series):
            pts = [(X(gg), Y(vv)) for gg, vv in zip(gains, vs) if vv is not None and vv == vv]
            if len(pts) > 1:
                o.append(f'<polyline fill="none" stroke="{col}" stroke-width="1.6" points="' + " ".join(f"{a:.1f},{b:.1f}" for a, b in pts) + '"/>')
            for a, b in pts:
                o.append(f'<circle cx="{a:.1f}" cy="{b:.1f}" r="2.4" fill="{col}"/>')
            o.append(f'<text x="{w - R + 8}" y="{T + 16 + 15 * (k + 1)}" fill="{col}" font-size="12" font-family="sans-serif">{name}</text>')
        o.append('</svg>')
        return "".join(o)

    def cal_build_html(self, c, finds, per_off, partial):
        cols = ['#ffd54a', '#4fc3f7', '#ff7a7a', '#9ccc65', '#ba68c8', '#ffb74d', '#4db6ac', '#f06292', '#b0bec5', '#dce775']
        css = ("body{background:#16181c;color:#d8dbe0;font:14px/1.45 Segoe UI,Arial,sans-serif;margin:0 auto;max-width:1180px;padding:16px 20px 60px}"
               "h1{font-size:22px;margin:4px 0 2px}h2{font-size:17px;margin:26px 0 6px;border-bottom:1px solid #333;padding-bottom:3px}"
               ".sub,.note{color:#9aa0a8}.note{font-size:13px;margin:2px 0 8px}"
               ".find{margin:0;padding:0;list-style:none}.find li{padding:7px 10px;margin:4px 0;border-left:4px solid #4a90d9;background:#1d2026}"
               ".find li.warn{border-left-color:#e0a030}.find li.bad{border-left-color:#e05050}.find li.ok{border-left-color:#5aa469}"
               "table{border-collapse:collapse;font-size:13px;margin:6px 0}th,td{padding:3px 8px;border:1px solid #2a2d33;text-align:right}"
               "th{background:#1d2026}td.g{background:#1d2026;font-weight:600}.scroll{overflow:auto;max-width:100%}")
        h = ['<!DOCTYPE html><html lang="en"><head><meta charset="utf-8"><title>RF calibration survey</title><style>' + css + '</style></head><body>',
             '<h1>RF scan - calibration survey (known tone)</h1>']
        h.append(f'<div class="sub">tone {c["tone_hz"]/1e6:.4f} MHz, {c.get("source_text", f'{c["vpp"]:g} Vpp at the attenuator input')} = <b>{c["expected"]:.2f} dBuV</b> at the attenuator input, '
                 f'attenuator {c["atten"]:g} dB, dB-to-dBuV: default {c["cal0"]:g}, {len(CAL_BY_RATE)} per-rate value(s) in use, averaging {c["n_avg"]}, '
                 f'{len(c["results"])} setting(s) x {c["scans"]} scan(s)' + (' | <b>PARTIAL RESULT - stopped early</b>' if partial else '') +
                 f' | {time.strftime("%Y-%m-%d %H:%M")}</div>')
        h.append('<h2>Findings</h2><ul class="find">' + "".join(f'<li class="{lv}">{tx}</li>' for lv, tx in finds) + '</ul>')
        h.append('<p class="note">The level shown is the tone converted with the main display\'s formula (SDR dB - RF gain + attenuator + CAL_SDR_DB_TO_DBUV), '
                 'taken from the power summed over the tone\'s lobe so FFT scalloping loss does not enter. If the CAL constant were right at every setting '
                 'each table cell would equal the true level. To re-calibrate: CAL_SDR_DB_TO_DBUV = current value + (true - displayed) for a setting you trust. '
                 'If the generator was set for a 50 ohm load but measured unterminated, the real level into 50 ohm is 6 dB lower than the Vpp read - check that first.</p>')
        for off, d in per_off.items():
            h.append(f'<h2>{d["name"]}</h2>')
            series = []
            for k, r in enumerate(d["rates"]):
                series.append((f'{r/1e6:.3f} MS/s', cols[k % len(cols)], [d["M"][(r, g)]["tone"] if (r, g) in d["M"] else None for g in d["gains"]]))
            h.append(self.cal_svg_chart("Displayed tone level (dBuV) against RF gain", d["gains"], series, c["expected"]))
            h.append('<h3 style="font-size:15px">Per sample rate</h3><table><tr><th>Sample rate</th><th>Level at lowest gains</th><th>Error there</th>'
                     '<th>CAL right for it (at low gain)</th><th>Follows gain up to</th><th>Error range, all gains</th></tr>')
            for p_ in d["per_rate"]:
                dep = "all gains tested" if p_["depart"] is None else f'below {p_["depart"]:g} dB'
                h.append(f'<tr><td>{p_["rate"]/1e6:.3f} MS/s</td><td>{p_["ref"]:.1f}</td><td>{p_["ref_err"]:+.1f}</td><td>{p_["implied_low"]:.1f}</td>'
                         f'<td>{dep}</td><td>{p_["emin"]:+.1f} to {p_["emax"]:+.1f}</td></tr>')
            h.append('</table>')
            h.append('<h3 style="font-size:15px">Level (dBuV) - rows RF gain, columns sample rate</h3><div class="scroll"><table><tr><th>RF gain</th>' +
                     "".join(f'<th>{r/1e6:.3f}</th>' for r in d["rates"]) + '<th>spread</th></tr>')
            for g in d["gains"]:
                row = f'<tr><td class="g">{g:g} dB</td>'
                for r in d["rates"]:
                    x = d["M"].get((r, g))
                    if x is None:
                        row += '<td></td>'
                        continue
                    e = abs(x["err"])
                    bg = '#2f6d3f' if e <= 1.0 else ('#b8741c' if e <= 3.0 else '#a03a3a')
                    row += (f'<td style="background:{bg};color:#fff" title="error {x["err"]:+.1f} dB, SNR {x["snr"]:.0f} dB, peak-bin {x["pk"]:.1f}">'
                            f'{x["tone"]:.1f}{"*" if x["other"] else ""}{"~" if x["spread"] > CAL_SPREAD_WARN_DB else ""}</td>')
                sp = d["spread_by_gain"].get(g)
                row += f'<td>{"" if sp is None else f"{sp:.1f}"}</td></tr>'
                h.append(row)
            h.append('</table></div><p class="note">Green: within 1 dB of the true level; amber: within 3 dB; red: further. '
                     '* a stronger signal exists elsewhere in the spectrum; ~ repeat scans differed. Hover a cell for error, SNR and peak-bin level.</p>')
        h.append('</body></html>')
        return "".join(h)

    def cal_write_outputs(self, c, partial):
        """Write the matrix CSV and the HTML report (the long CSV was written scan by scan). Returns the report path."""
        finds, per_off = self.cal_analyse(c)
        base = c["base"]
        with open(base + "_matrix.csv", "w", newline="") as fh:
            w = csv.writer(fh)
            for off, d in per_off.items():
                w.writerow([d["name"], f"true {c['expected']:.2f} dBuV"])
                w.writerow(["rf_gain_db"] + [f"{r:.0f}" for r in d["rates"]])
                for g in d["gains"]:
                    w.writerow([f"{g:g}"] + ["" if (r, g) not in d["M"] else f"{d['M'][(r, g)]['tone']:.2f}" for r in d["rates"]])
                w.writerow([])
        html_text = self.cal_build_html(c, finds, per_off, partial)
        with open(c["html_path"], "w", encoding="utf-8") as fh:
            fh.write(html_text)
        for lv, tx in finds:
            self.log(("CAL: " if lv == "ok" else "CAL (check): ") + tx)
        return c["html_path"]

    def cal_finish(self, aborted):
        """GUI thread, after the cal worker has finished: write the report (CAL SURVEY) or show the per-rate result (CAL),
        give the controls back, resume CONTINUOUS."""
        c = self._cal
        if c is None:
            return
        quick = bool(c.get("quick"))
        path = None
        try:
            c["fh"].close()
        except Exception:
            pass
        if not quick:
            try:
                path = self.cal_write_outputs(c, bool(aborted))
            except Exception as e:
                import traceback
                traceback.print_exc()
                self.log(f"CAL survey: could not write the report: {e}")
        self.sync_sample_rate_combo()
        self.sync_gain_slider()
        try:
            self.curve.setData([], [])                       # the CAL's last spectrum is gone (see cal_loop): do not leave it on screen
        except Exception:
            pass
        self._plot_freq_axis = np.array([], dtype=np.float64)
        self._plot_power_axis = np.array([], dtype=np.float64)
        self.update_status_panel()
        self.survey_set_lock(False)
        self._cal = None
        self._survey = None
        try:
            self.draw_block_markers()
        except Exception:
            pass
        self.log(f"=== {'CAL' if quick else 'CAL SURVEY'} {'STOPPED EARLY' if aborted else 'FINISHED'}: {len(c['results'])} settings, {len(c['records'])} scans"
                 f"{', ' + str(c['skipped']) + ' skipped' if c['skipped'] else ''}{' -> ' + path if path else ''} ===")
        if quick:
            if aborted:
                self.log("CAL stopped early - nothing was changed")
            else:
                try:
                    self.cal_quick_report(c)
                except Exception as e:
                    import traceback
                    traceback.print_exc()
                    self.log(f"CAL: could not show the result: {e}")
        elif path and SURVEY_OPEN_REPORT:
            try:
                QtGui.QDesktopServices.openUrl(QtCore.QUrl.fromLocalFile(os.path.abspath(path)))
            except Exception as e:
                self.log(f"CAL survey: could not open the report automatically: {e}")
        if c["was_running"] and not aborted:
            self.continuous_when_idle()

    # ------------------------------------------------------------------------------------
    # CAL button: one dB -> dBuV factor PER SAMPLE RATE, from a sine of known frequency (default 1.5 MHz) and amplitude, at the RF gain set on the front panel
    # ------------------------------------------------------------------------------------
    # Each rate is tuned so the tone sits at several fixed positions in the block (+/-15 % and +/-30 % of the sample
    # rate from the block centre, inside the part the display keeps) and measured at the RF gain that is set on the
    # front panel when CAL is pressed - the user sets it to about the value to be used for measurements. The factor for
    # the rate is the median over those positions:
    #       factor = true dBuV at the attenuator input - attenuator - tone level (SDR dB) + RF gain
    # so that  displayed = SDR dB - RF gain + attenuator + factor  reads the tone correctly AT THAT GAIN. Because the
    # real tuner gain differs from the nominal gain table, the factor also absorbs that difference at this gain, which
    # is why it is only right close to the gain it was made at. The spread over position is
    # reported (tuner passband ripple); a rate is only accepted if the tone is well clear of ADC full scale and of the
    # noise. Nothing is applied until the user says so, and settings.csv is only written if the user chooses that.

    def cal_quick_open_dialog(self):
        if self.cal_busy():
            self.log("CAL: not available while a survey, the stitch check or the scope is running")
            return
        if self.hf_mode:
            QtWidgets.QMessageBox.information(self, "CAL", "Not available in an HF (direct-sampling) mode - pick a tuner MODE first.")
            return
        dlg = QtWidgets.QDialog(self)
        dlg.setWindowTitle("CAL - dB to dBuV factor for every sample rate")
        lay = QtWidgets.QVBoxLayout(dlg)
        lay.addWidget(QtWidgets.QLabel(
            "Connect a SINE WAVE of known amplitude and frequency to the SDR input (through the attenuator).\n"
            "The CAL is made at the RF gain set on the front panel, so set the RF Gain slider to about the value you will use\n"
            "for measurements BEFORE pressing CAL. Choose a sine level small enough not to overload the SDR at that gain: with\n"
            "the sine connected the tone should sit well below the red saturation line on the display (check with CONTINUOUS).\n"
            "Enter the frequency and amplitude you are feeding in, and the attenuator that is fitted. The factor you get applies at\n"
            "that frequency, so calibrate at a frequency in the part of the spectrum you will measure."))
        form = QtWidgets.QFormLayout()
        cal_gain = float(self.gain_db)                 # the CAL is made at this RF gain (front panel), not at 0 dB
        lbl_gain = QtWidgets.QLabel(f"<b>{cal_gain:g} dB</b>   (front-panel RF Gain - the CAL is made at this gain)")
        sp_freq = QtWidgets.QDoubleSpinBox()
        sp_freq.setLocale(SCOPE_LOCALE)
        sp_freq.setRange(0.1, 1700.0)
        sp_freq.setDecimals(4)
        sp_freq.setSuffix(" MHz")
        sp_freq.setValue(CAL_QUICK_TONE_HZ / 1e6)
        sp_freq.setToolTip("Frequency of the test sine. Each sample rate is measured with the tone at several positions in the block,\n"
                           "so the tuner must be able to tune below the tone: a low tone frequency cannot be used with the widest sample rates.")
        sp_att = QtWidgets.QDoubleSpinBox()
        sp_att.setLocale(SCOPE_LOCALE)
        sp_att.setRange(0.0, 100.0)
        sp_att.setDecimals(1)
        sp_att.setSuffix(" dB")
        sp_att.setValue(float(self.input_atten_db))
        sp_att.setToolTip("The external attenuator between the signal and the SDR (20 dB for example). Defaults to the front-panel Attenuator setting. "
                          "Levels are calibrated at ITS INPUT. Leave the front-panel Attenuator at this value afterwards.")
        form.addRow("RF gain for the CAL", lbl_gain)
        form.addRow("Frequency of the sine", sp_freq)
        src_get = self.cal_source_rows(form, lambda: refresh(), (CAL_QUICK_VPP_DEFAULT, CAL_QUICK_RSERIES_DEFAULT, 50.0))
        form.addRow("Attenuator fitted", sp_att)
        lay.addLayout(form)
        lay.addWidget(QtWidgets.QLabel("Sample rates to calibrate:"))
        lst = QtWidgets.QListWidget()
        for r in SAMPLE_RATE_CHOICES:
            it = QtWidgets.QListWidgetItem(f"{r/1e6:.3f} MS/s" + ("   (calibrated: %.2f)" % cal_for_rate(r) if cal_has_rate(r) else ""))
            it.setData(QtCore.Qt.UserRole, float(r))
            it.setFlags(it.flags() | QtCore.Qt.ItemIsUserCheckable)
            it.setCheckState(QtCore.Qt.Checked)
            lst.addItem(it)
        lst.setMinimumHeight(150)
        lay.addWidget(lst)
        info = QtWidgets.QLabel("")
        lay.addWidget(info)

        def chosen():
            return [lst.item(i).data(QtCore.Qt.UserRole) for i in range(lst.count()) if lst.item(i).checkState() == QtCore.Qt.Checked]

        def positions(rate):
            """Tone positions (fractions of the sample rate from the block centre) whose block centre is a legal tuner frequency."""
            tone = sp_freq.value() * 1e6
            return [fr for fr in CAL_POS_FRACTIONS if 50e3 < tone - fr * rate < 1.7e9]

        def usable(rates):
            return [r for r in rates if len(positions(r)) >= 2]

        def refresh(*_):
            exp = src_get()["dbuv"]
            at_sdr = exp - sp_att.value()
            # rough headroom below ADC full scale at this gain, from the calibration in use (the measurement itself checks properly):
            # the displayed level at which a tone reaches full scale (formula of step 2, most sensitive factor) minus the tone's level
            head = self.sat_full_scale_p() + cal_model_offset_db(cal_reference_db(), cal_gain, sp_att.value()) - exp
            warn = ""
            if head < CAL_MIN_HEADROOM_DB + 2.0:
                warn = f"   WARNING: only about {head:.0f} dB below ADC full scale at {cal_gain:g} dB gain - probably too large, lower the level"
            n_avg = self.avg_get_active_count()
            ok_rates = usable(chosen())
            skipped = [r for r in chosen() if r not in ok_rates]
            tot = sum((0.12 + 0.15 + n_avg * (NFFT / r + 0.004)) * len(positions(r)) * (CAL_QUICK_SCANS + 1) for r in ok_rates)
            note = ("" if not skipped else "\nNot possible at this tone frequency (the block would reach below 50 kHz): " +
                    ", ".join(f"{r/1e6:.3f}" for r in skipped) + " MS/s - they will be skipped.")
            info.setText(f"About {at_sdr:.0f} dBuV reaches the SDR; estimated headroom {head:.0f} dB at {cal_gain:g} dB RF gain.{warn}\n"
                         f"{len(ok_rates)} rate(s) to calibrate, roughly {self.survey_fmt_seconds(tot)}. Averaging setting is used as-is.{note}")

        sp_freq.valueChanged.connect(refresh)
        sp_att.valueChanged.connect(refresh)
        lst.itemChanged.connect(refresh)
        refresh()
        bb = QtWidgets.QDialogButtonBox(QtWidgets.QDialogButtonBox.Ok | QtWidgets.QDialogButtonBox.Cancel)
        bb.accepted.connect(dlg.accept)
        bb.rejected.connect(dlg.reject)
        lay.addWidget(bb)
        if dlg.exec_() != QtWidgets.QDialog.Accepted:
            return
        rates = chosen()
        if not rates:
            self.log("CAL: no sample rate selected")
            return
        tone = sp_freq.value() * 1e6
        for r in rates:
            if r not in usable(rates):
                self.log(f"CAL: {r/1e6:.3f} MS/s skipped - at a {tone/1e6:g} MHz tone the block would reach below 50 kHz")
        rates = usable(rates)
        if not rates:
            self.log("CAL: none of the chosen sample rates can be calibrated at this tone frequency")
            return
        combos = [(r, round(fr * r / 1e3, 3), cal_gain) for r in rates for fr in positions(r)]
        src = src_get()
        self._cal_source_last = src                  # offered again next time
        self.cal_start({"quick": True, "tone_hz": tone, "vpp": src["vpp_in"], "atten": sp_att.value(), "source_text": src["text"],
                        "expected": src["dbuv"], "win_hz": max(CAL_WINDOW_HZ_DEFAULT, 20e-6 * tone),
                        "use_band": False, "combos": combos, "scans": CAL_QUICK_SCANS, "path": "", "gain": cal_gain})

    def cal_quick_evaluate(self, c):
        """Reduce the CAL results to one row per sample rate: the factor, its spread over tone position, the headroom below
        ADC full scale, the SNR and an accept / reject verdict."""
        fs_p = self.sat_full_scale_p()
        rows = []
        for r in sorted({x["rate"] for x in c["results"]}):
            rs = [x for x in c["results"] if x["rate"] == r and x["found"] and x["tone_p"] == x["tone_p"]]
            row = {"rate": r, "old": cal_for_rate(r), "had": cal_has_rate(r), "new": None, "spread": None, "head": None,
                   "snr": None, "ok": False, "note": "no tone found"}
            if rs:
                vals = [cal_solve_factor(c["expected"], x["tone_p"], c["gain"], c["atten"]) for x in rs]   # factor implied at each tone position
                row["new"] = float(np.median(vals))
                row["spread"] = float(max(vals) - min(vals))
                row["head"] = float(fs_p - max(x["tone_p"] for x in rs))
                row["snr"] = float(min(x["snr"] for x in rs))
                row["npos"] = len(rs)
                if row["head"] < CAL_MIN_HEADROOM_DB:
                    row["note"] = f"REJECTED: tone only {row['head']:.0f} dB below ADC full scale - reduce the input level"
                elif row["snr"] < CAL_MIN_SNR_DB:
                    row["note"] = f"REJECTED: tone only {row['snr']:.0f} dB above the noise - increase the input level"
                else:
                    row["ok"] = True
                    row["note"] = "ok" if row["spread"] <= CAL_POS_SPREAD_WARN_DB else \
                        f"ok - level varies {row['spread']:.1f} dB with tone position in the block"
                    if row["npos"] < len(CAL_POS_FRACTIONS):
                        row["note"] += f" (only {row['npos']} of {len(CAL_POS_FRACTIONS)} tone positions possible at this frequency)"
            rows.append(row)
        return rows

    def cal_quick_report(self, c):
        """GUI thread: show the result and ask what to do - save to settings.csv, use for this session only, or discard."""
        rows = self.cal_quick_evaluate(c)
        head = f"{'Rate (MS/s)':>11} {'Old':>8} {'New':>8} {'Change':>7} {'Pos. spread':>11} {'Headroom':>9} {'SNR':>5}  Result"
        lines = [head]
        for x in rows:
            if x["new"] is None:
                lines.append(f"{x['rate']/1e6:>11.3f} {x['old']:>8.2f} {'-':>8} {'-':>7} {'-':>11} {'-':>9} {'-':>5}  {x['note']}")
            else:
                lines.append(f"{x['rate']/1e6:>11.3f} {x['old']:>8.2f} {x['new']:>8.2f} {x['new'] - x['old']:>+7.2f} {x['spread']:>10.1f}  "
                             f"{x['head']:>8.0f} {x['snr']:>5.0f}  {x['note']}")
        table = "\n".join(lines)
        for ln in lines:
            self.log("CAL " + ln)
        good = [x for x in rows if x["ok"]]
        dlg = QtWidgets.QDialog(self)
        dlg.setWindowTitle("CAL result")
        lay = QtWidgets.QVBoxLayout(dlg)
        lay.addWidget(QtWidgets.QLabel(
            f"Sine {c['tone_hz']/1e6:g} MHz: {c.get('source_text', '')} = {c['expected']:.2f} dBuV at the attenuator input.\n"
            f"Attenuator {c['atten']:g} dB, RF gain {c['gain']:g} dB.\n"
            f"'Old' is the factor in use now (the default {CAL_SDR_DB_TO_DBUV:g} where a rate has none of its own). "
            f"Headroom = dB below ADC full scale; SNR = dB above the noise."))
        txt = QtWidgets.QPlainTextEdit(table)
        txt.setReadOnly(True)
        txt.setFont(QtGui.QFont("Consolas", 9))
        txt.setLineWrapMode(QtWidgets.QPlainTextEdit.NoWrap)
        txt.setMinimumSize(760, 40 + 22 * len(lines))
        lay.addWidget(txt)
        choice = {"v": None}
        bb = QtWidgets.QHBoxLayout()
        if good:
            lay.addWidget(QtWidgets.QLabel(f"{len(good)} of {len(rows)} rate(s) are good. What do you want to do with them?"))
            b_save = QtWidgets.QPushButton("Use and SAVE to settings.csv")
            b_use = QtWidgets.QPushButton("Use for this session only")
            for b, v in ((b_save, "save"), (b_use, "use")):
                b.clicked.connect(lambda _=False, v=v: (choice.__setitem__("v", v), dlg.accept()))
                bb.addWidget(b)
        else:
            lay.addWidget(QtWidgets.QLabel("No rate gave a usable result - nothing can be applied. Check the signal level and connections."))
        b_no = QtWidgets.QPushButton("Discard (change nothing)")
        b_no.clicked.connect(dlg.reject)
        bb.addWidget(b_no)
        lay.addLayout(bb)
        dlg.exec_()
        v = choice["v"]
        if v is None:
            self.log("CAL: result discarded - nothing was changed")
            return
        for x in good:
            CAL_BY_RATE[float(x["rate"])] = round(x["new"], 2)
        self.log(f"CAL: {len(good)} per-rate factor(s) applied for this session (made at RF gain {c['gain']:g} dB)")
        if v == "save":
            try:
                path = settings_write_cal(CAL_BY_RATE, c["gain"], c["tone_hz"])
                self.log(f"CAL: saved to {path} (the previous file is kept as {path}.bak)")
            except Exception as e:
                self.log(f"CAL: ERROR - could not write settings.csv: {e}")
                QtWidgets.QMessageBox.warning(self, "CAL", f"The factors are in use for this session but settings.csv could not be written:\n{e}")
        self.cispr_refresh_offset_ui()
        self.update_status_panel()
        self.update_plot()

    # ------------------------------------------------------------------------------------
    # ADC saturation lines on the display
    # ------------------------------------------------------------------------------------
    # Red line: the displayed level at which a single tone reaches ADC full scale at the current RF gain and attenuator.
    # Orange dashed line: SATURATION_MARGIN_DB below it - above that the front end starts to compress, so a peak that is
    # up to or beyond it is not a trustworthy level. The line is the same for every sample rate and is set by the most
    # sensitive calibrated rate (the smallest factor), so for a less sensitive rate real clipping can start a little
    # higher than shown: the line errs on the safe side. It is a guide for single spikes; a very strong signal outside the
    # displayed band, or the sum of many signals, can overload the SDR without any one spike reaching the line.

    def sat_build_ui(self):
        self.sat_fs_line = pg.InfiniteLine(pos=0.0, angle=0, movable=False, pen=pg.mkPen((255, 70, 70), width=2))
        self.sat_margin_line = pg.InfiniteLine(pos=0.0, angle=0, movable=False,
                                               pen=pg.mkPen((255, 170, 40), width=1, style=QtCore.Qt.DashLine))
        for ln in (self.sat_fs_line, self.sat_margin_line):
            ln.setZValue(5)
            self.plot.addItem(ln, ignoreBounds=True)
        self.chk_sat = QtWidgets.QCheckBox("Show ADC saturation line")
        self.chk_sat.setChecked(bool(SATURATION_LINE))
        self.chk_sat.setToolTip(
            "Red: displayed level at which a single tone reaches ADC full scale (clipping) at the current RF gain and attenuator.\n"
            f"Orange dashed: {SATURATION_MARGIN_DB:g} dB lower - above it the front end starts to compress, so levels there are unreliable.\n"
            "Default on/off: settings.csv  setting,saturation_line,on|off   (margin: setting,saturation_margin_db,10)")
        sub = self.find_layout_with(self.chk_markers)
        if sub is not None:
            sub.addWidget(self.chk_sat)
        else:
            self.centralWidget().layout().addWidget(self.chk_sat)
        self.chk_sat.toggled.connect(lambda _on: self.sat_update_lines())
        self.sat_update_lines()

    def sat_full_scale_p(self):
        """Stored spectrum level (SDR dB) of a sine at ADC full scale: |FFT/N|^2 of a Hann-windowed tone of amplitude 1/128
        (this program divides the driver's already-normalised samples by 128 again - the survey plateau confirmed it: 92.3
        predicted, 93.2 measured at clipping once the conversion is added back) plus the PSD constant."""
        return 10.0 * math.log10(0.25 / (128.0 ** 2)) + float(self._psd_to_dbuv_const)

    def sat_full_scale_display(self):
        """Displayed level at which a single tone reaches ADC full scale: 0 dBFS in dBFS mode, else dBuV at the current RF gain
        and attenuator."""
        if getattr(self, "display_dbfs", False):
            return 0.0
        applied = getattr(self, "_applied_gain_db", None)
        gain = float(applied if applied is not None else self.gain_db)
        if self.hf_mode:
            gain = 0.0
        return (self.sat_full_scale_p() + cal_model_offset_db(cal_reference_db(), gain, self.input_atten_db)      # steps 1-2, most sensitive factor
                + float(getattr(self, "cispr_offset_db", 0.0)))                                                     # step 3

    def sat_update_lines(self):
        """GUI thread: place (or hide) the saturation lines. Called on every plot update and when the box is ticked."""
        try:
            on = bool(self.chk_sat.isChecked())
            self.sat_fs_line.setVisible(on)
            self.sat_margin_line.setVisible(on)
            if on:
                fs = self.sat_full_scale_display()
                self.sat_fs_line.setValue(fs)
                self.sat_margin_line.setValue(fs - float(SATURATION_MARGIN_DB))
        except Exception:
            pass

    def cal_label_text(self):
        """Text of the front-panel 'Cal:' label for the sample rate in use."""
        if getattr(self, "display_dbfs", False):
            return "Cal: off (dBFS)"
        rate = getattr(self, "_applied_sample_rate", None) or self.SAMPLE_RATE
        return f"Cal: {cal_for_rate(rate):+.1f} dB" + ("" if cal_has_rate(rate) else " (default)")

    # ------------------------------------------------------------------------------------
    # WATERFALL: spectrum history below the main graph, to show periodic / intermittent effects
    # ------------------------------------------------------------------------------------
    # Tick "Show waterfall" and a panel appears under the graph; its frequency axis is linked to the main graph (zoom or
    # pan either one). Colours are dBuV, with the same calibration as the trace. Newest row at the top.
    # Two sources of rows:
    #   * SWEEP rows (default): one row per completed pass - a whole stitched sweep in the normal modes, one spectrum in
    #     single-frequency mode. Shows what comes and goes from sweep to sweep. Time per row = the sweep time (seconds).
    #   * FAST rows (single-frequency mode, "Fast rows" ticked): after each normal pass the program reads one CONTIGUOUS
    #     record of 128 x 4096 samples and shows a row per FFT (about 1.7 ms each at 2.4 MS/s), unaveraged. A burst that
    #     repeats every 200 ms then shows as a bright line every ~117 rows. Batches are separated by a dark line (the
    #     sweep pass between them is not in the picture), so periods longer than one batch (0.2 s at 2.4 MS/s) show
    #     within a batch only.

    def wf_build_ui(self):
        self.wf_enabled = False                 # read by the worker thread
        self.wf_fast = False
        self._wf_f = None
        self._wf_buf = None
        self._wf_kind = ""
        self._wf_dt = 1.0
        self._wf_last_t = None
        self._wf_dt_ema = None
        self._wf_levels = None
        self.wf_signals = WaterfallSignals()
        self.wf_signals.rows.connect(self.wf_on_rows)

        self.wf_box = QtWidgets.QWidget()
        v = QtWidgets.QVBoxLayout(self.wf_box)
        v.setContentsMargins(0, 0, 0, 0)
        v.setSpacing(2)
        ctl = QtWidgets.QHBoxLayout()
        ctl.setContentsMargins(0, 0, 0, 0)
        ctl.addWidget(QtWidgets.QLabel("Waterfall  rows:"))
        self.wf_spin_rows = QtWidgets.QSpinBox()
        self.wf_spin_rows.setRange(20, 2000)
        self.wf_spin_rows.setValue(int(WATERFALL_ROWS))
        self.wf_spin_rows.setToolTip("How many rows of history to keep.")
        self.wf_spin_rows.valueChanged.connect(lambda _v: self.wf_redraw())
        ctl.addWidget(self.wf_spin_rows)
        self.wf_chk_fast = QtWidgets.QCheckBox("Fast rows (single-frequency mode)")
        self.wf_chk_fast.setToolTip(
            "Single-frequency mode only: after each normal pass, record 128 x 4096 contiguous samples and show every FFT as a row\n"
            "(about 1.7 ms each at 2.4 MS/s, unaveraged). This is what shows short bursts that repeat every few hundred ms.\n"
            "Batches are separated by a dark line. Unticked: one row per sweep pass (seconds per row).")
        self.wf_chk_fast.toggled.connect(self.wf_on_fast_toggled)
        ctl.addWidget(self.wf_chk_fast)
        self.wf_chk_auto = QtWidgets.QCheckBox("Auto colours")
        self.wf_chk_auto.setChecked(True)
        self.wf_chk_auto.toggled.connect(lambda _on: self.wf_redraw())
        ctl.addWidget(self.wf_chk_auto)
        self.wf_sp_lo = QtWidgets.QDoubleSpinBox()
        self.wf_sp_hi = QtWidgets.QDoubleSpinBox()
        for sp, val in ((self.wf_sp_lo, 20.0), (self.wf_sp_hi, 80.0)):
            sp.setRange(-100.0, 200.0)
            sp.setDecimals(0)
            sp.setSuffix(" dBuV")
            sp.setValue(val)
            sp.valueChanged.connect(lambda _v: self.wf_redraw())
        self.wf_sp_lo.setToolTip("Colour scale bottom (used when Auto colours is off).")
        self.wf_sp_hi.setToolTip("Colour scale top (used when Auto colours is off).")
        ctl.addWidget(self.wf_sp_lo)
        ctl.addWidget(self.wf_sp_hi)
        btn_clear = QtWidgets.QPushButton("Clear")
        btn_clear.clicked.connect(self.wf_clear)
        ctl.addWidget(btn_clear)
        ctl.addStretch(1)
        v.addLayout(ctl)

        self.wf_plot = pg.PlotWidget()
        self.wf_plot.setLabel("bottom", "Frequency", units="Hz")
        self.wf_plot.setLabel("left", "Time ago", units="s")
        self.wf_plot.invertY(True)                                   # newest row at the top
        self.wf_plot.setMouseEnabled(x=True, y=False)
        self.wf_img = pg.ImageItem()
        try:
            self.wf_img.setOpts(axisOrder="col-major")               # array is indexed [frequency, row]
        except Exception:
            pass
        self.wf_plot.addItem(self.wf_img)
        self.wf_plot.setXLink(self.plot)                             # same frequency axis as the main graph
        self.wf_plot.enableAutoRange(axis=pg.ViewBox.XAxis, enable=False)   # the main graph owns the X range (a linked auto-ranging
        self.wf_plot.enableAutoRange(axis=pg.ViewBox.YAxis, enable=False)   # view would drag it around); the rows set the Y range
        pos = np.array([0.0, 0.25, 0.5, 0.75, 1.0])
        col = np.array([[0, 0, 40, 255], [0, 70, 170, 255], [0, 200, 200, 255], [240, 240, 60, 255], [255, 60, 0, 255]], dtype=np.ubyte)
        self._wf_lut = pg.ColorMap(pos, col).getLookupTable(0.0, 1.0, 256)
        v.addWidget(self.wf_plot, 1)
        self.wf_box.setVisible(False)

        lay = self.find_layout_with(self.plot)
        if lay is not None:
            idx = lay.indexOf(self.plot)
            lay.setStretchFactor(self.plot, 3)
            lay.insertWidget(idx + 1, self.wf_box, 2)
        else:
            self.centralWidget().layout().addWidget(self.wf_box)

        self.chk_wf = QtWidgets.QCheckBox("Show waterfall")
        self.chk_wf.setToolTip("Show a waterfall (spectrum history, newest at the top) below the graph, to reveal effects that repeat "
                               "or come and go. Default: settings.csv  setting,waterfall,on|off   setting,waterfall_rows,150")
        sub = self.find_layout_with(self.chk_markers)
        if sub is not None:
            sub.addWidget(self.chk_wf)
        else:
            self.centralWidget().layout().addWidget(self.chk_wf)
        self.chk_wf.toggled.connect(self.wf_on_toggled)
        if WATERFALL_ON:
            self.chk_wf.setChecked(True)

    def wf_on_toggled(self, on):
        self.wf_enabled = bool(on)
        self.wf_box.setVisible(bool(on))
        if on:
            self._wf_last_t = None
            QtCore.QTimer.singleShot(60, self.wf_sync_axes)
            QtCore.QTimer.singleShot(0, self.wf_first_sizes)
            self.log("Waterfall on" + (" (fast rows need single-frequency mode)" if self.wf_chk_fast.isChecked() else ""))

    def wf_first_sizes(self):
        """First time the waterfall is shown: give it about 40 % of the height. After that the user's splitter position is kept."""
        sp = getattr(self, "wf_split", None)
        if sp is None or getattr(self, "_wf_sized", True):
            return
        self._wf_sized = True
        total = sp.height()
        if total > 100:
            sp.setSizes([int(total * 0.6), int(total * 0.4)])

    def wf_on_fast_toggled(self, on):
        self.wf_fast = bool(on)
        self.wf_clear()
        if on and not getattr(self, "single_mode", False):
            self.log("Waterfall: fast rows are only taken in single-frequency mode - normal sweep rows are shown until you select it")

    def wf_sync_axes(self):
        """Give the waterfall the same left / right margins as the main graph so the frequency axes line up."""
        try:
            wl = int(max(self.plot.getAxis("left").width(), 60))
            wr = int(max(self.plot.getAxis("right").width(), 30))
            self.plot.getAxis("left").setWidth(wl)
            self.wf_plot.getAxis("left").setWidth(wl)
            self.wf_plot.showAxis("right")
            ax = self.wf_plot.getAxis("right")
            ax.setStyle(showValues=False, tickLength=0)
            ax.setPen(pg.mkPen(None))
            ax.setWidth(wr)
        except Exception as e:
            print("wf_sync_axes:", e)

    def wf_clear(self):
        self._wf_buf = None
        self._wf_f = None
        self._wf_levels = None
        try:
            self.wf_img.clear()
        except Exception:
            pass

    @staticmethod
    def wf_reduce(f, p):
        """Reduce to at most WF_MAX_BINS columns by max pooling (a narrow spike must survive). p may be 1-D or n x bins."""
        n = f.size
        g = int(math.ceil(n / float(WF_MAX_BINS)))
        if g <= 1:
            return f, p
        m = (n // g) * g
        f2 = f[:m].reshape(-1, g).mean(axis=1)
        if p.ndim == 1:
            return f2, np.fmax.reduce(p[:m].reshape(-1, g), axis=1)
        return f2, np.fmax.reduce(p[:, :m].reshape(p.shape[0], -1, g), axis=2)

    def wf_note_pass(self, lp):
        """WORKER thread: one finished clean pass -> one waterfall row (calibrated, in dBuV)."""
        if lp is None or self._survey is not None or self._stitch is not None:
            return                                       # survey / CAL / stitch-check passes are not wanted here
        if self.wf_fast and getattr(self, "single_mode", False):
            return                                       # fast rows replace the pass rows in single-frequency mode
        now = time.time()
        last = self._wf_last_t
        self._wf_last_t = now
        if last is not None and 0.0 < now - last < 120.0:
            self._wf_dt_ema = (now - last) if self._wf_dt_ema is None else 0.7 * self._wf_dt_ema + 0.3 * (now - last)
        f = np.asarray(lp["f"], dtype=np.float64)
        p = np.asarray(lp["p"], dtype=np.float64)
        ok = np.isfinite(f) & np.isfinite(p)
        f, p = f[ok], p[ok]
        if f.size < 8:
            return
        order = np.argsort(f)
        f, p = f[order], p[order]
        p = p + self.display_offset_for(lp["rate"], lp["gain"])                    # SDR dB -> the display's units (dBuV or dBFS)
        f, p = self.wf_reduce(f, p)
        kind = f"pass:{self.START_FREQ:.0f}-{self.STOP_FREQ:.0f}"          # a new band starts a new history; a new rate / gain does not
        self.wf_signals.rows.emit(f, p[None, :].astype(np.float32), kind, float(self._wf_dt_ema or 1.0))

    def wf_fast_capture(self):
        """WORKER thread (called after each pass in continuous mode): in single-frequency mode with Fast rows on, read one
        contiguous record and turn every 4096-sample piece into a row - the time structure the averaged pass cannot show."""
        if not (self.wf_enabled and self.wf_fast and getattr(self, "single_mode", False)):
            return
        if self.stop_flag.is_set() or self._survey is not None or self.sdr is None:
            return
        rate = float(getattr(self, "_applied_sample_rate", None) or self.SAMPLE_RATE)
        centre = float(self.centers[0]) if len(self.centers) else float(self.START_FREQ)
        try:
            x = self._read_normalized_iq(NFFT * WF_FAST_ROWS)
        except OSError as e:
            print(f"wf_fast_capture: read error: {e}")
            return
        m = (x.size // NFFT) * NFFT
        if m < NFFT:
            return
        seg = x[:m].reshape(-1, NFFT) * np.hanning(NFFT).astype(np.float32)[None, :]
        spec = np.fft.fftshift(np.fft.fft(seg, axis=1), axes=1) / NFFT
        db = 10.0 * np.log10(np.abs(spec) ** 2 + 1e-12) + self._psd_to_dbuv_const + self.cal_total_offset_db()
        f = np.fft.fftshift(np.fft.fftfreq(NFFT, d=1.0 / rate)) + centre
        f, db = self.wf_reduce(f, db)
        rows = np.vstack([db[::-1], np.full((1, db.shape[1]), np.nan)]).astype(np.float32)   # newest first, then a dark separator
        self.wf_signals.rows.emit(f, rows, f"fast:{centre:.0f}", float(NFFT) / rate)    # a new centre starts a new history; a new rate does not

    def wf_on_rows(self, f, rows, kind, dt):
        """GUI thread: add rows (newest first) to the history and redraw."""
        if not self.wf_enabled:
            return
        f = np.asarray(f, dtype=np.float64)
        rows = np.asarray(rows, dtype=np.float32)
        if rows.ndim == 1:
            rows = rows[None, :]
        cap = int(self.wf_spin_rows.value())
        if self._wf_buf is None or kind != self._wf_kind or self._wf_f is None:
            # first rows, or a different band / centre / row type: start a new history
            self._wf_buf = np.full((cap, f.size), np.nan, dtype=np.float32)
            self._wf_f = f
            self._wf_kind = kind
            self._wf_levels = None
        elif f.size != self._wf_f.size or abs(f[0] - self._wf_f[0]) > 1e-3 or abs(f[-1] - self._wf_f[-1]) > 1e-3:
            # same band, but the bins are laid out differently (the sample rate changed): fit the new rows to the history's
            # frequency grid instead of clearing it
            rows = self.wf_to_grid(f, rows, self._wf_f)
            f = self._wf_f
        nx = f.size
        n = rows.shape[0]
        old = self._wf_buf
        if old.shape[0] != cap:                                   # history depth changed: keep what fits
            keep = np.full((cap, nx), np.nan, dtype=np.float32)
            k = min(cap, old.shape[0])
            keep[:k] = old[:k]
            old = keep
        self._wf_buf = rows[:cap].copy() if n >= cap else np.concatenate([rows, old[:cap - n]], axis=0)
        if self._wf_buf.shape[0] < cap:
            pad = np.full((cap - self._wf_buf.shape[0], nx), np.nan, dtype=np.float32)
            self._wf_buf = np.concatenate([self._wf_buf, pad], axis=0)
        self._wf_dt = max(float(dt), 1e-6)
        self.wf_redraw()

    @staticmethod
    def wf_to_grid(f_new, rows, f_grid):
        """Fit rows (n x len(f_new)) to the frequency grid f_grid. Where the new bins are much finer, take the peak of each
        group first so narrow spikes survive; where the new rows do not cover the grid the result is NaN (shown dark)."""
        d_new = abs(f_new[1] - f_new[0]) if f_new.size > 1 else 1.0
        d_grid = abs(f_grid[1] - f_grid[0]) if f_grid.size > 1 else 1.0
        g = int(d_grid // d_new)
        if g >= 2:
            m = (f_new.size // g) * g
            f_new = f_new[:m].reshape(-1, g).mean(axis=1)
            rows = np.fmax.reduce(rows[:, :m].reshape(rows.shape[0], -1, g), axis=2)
        out = np.full((rows.shape[0], f_grid.size), np.nan, dtype=np.float32)
        for i in range(rows.shape[0]):
            out[i] = np.interp(f_grid, f_new, rows[i], left=np.nan, right=np.nan)
        return out

    def wf_redraw(self):
        """GUI thread: draw the history buffer (levels, image placement, time axis)."""
        buf = getattr(self, "_wf_buf", None)
        if buf is None or getattr(self, "_wf_f", None) is None:
            return
        cap = int(self.wf_spin_rows.value())
        if buf.shape[0] != cap:                                   # the spin box changed: resize the history
            new = np.full((cap, buf.shape[1]), np.nan, dtype=np.float32)
            k = min(cap, buf.shape[0])
            new[:k] = buf[:k]
            self._wf_buf = buf = new
        finite = np.isfinite(buf)
        if not finite.any():
            return
        if self.wf_chk_auto.isChecked():
            vals = buf[finite]
            if vals.size > 200000:
                vals = vals[::max(1, vals.size // 200000)]
            lo = float(np.percentile(vals, 3.0))
            hi = float(np.percentile(vals, 99.7))
            if hi - lo < 15.0:
                hi = lo + 15.0
            if self._wf_levels is not None:                       # smooth, so the colours do not flicker from row to row
                lo = 0.7 * self._wf_levels[0] + 0.3 * lo
                hi = 0.7 * self._wf_levels[1] + 0.3 * hi
            self._wf_levels = (lo, hi)
            for sp, val in ((self.wf_sp_lo, lo), (self.wf_sp_hi, hi)):
                sp.blockSignals(True)
                sp.setValue(round(val))
                sp.blockSignals(False)
        else:
            lo, hi = float(self.wf_sp_lo.value()), float(self.wf_sp_hi.value())
            if hi <= lo:
                hi = lo + 1.0
        disp = np.where(finite, buf, np.float32(lo))                # empty rows and separators show as the bottom colour
        self.wf_img.setImage(np.ascontiguousarray(disp.T), autoLevels=False, levels=(lo, hi), lut=self._wf_lut)
        f = self._wf_f
        nx = f.size
        dx = (f[-1] - f[0]) / (nx - 1) if nx > 1 else 1.0
        tr = QtGui.QTransform()
        tr.translate(float(f[0] - dx / 2.0), 0.0)
        tr.scale(float(dx), 1.0)
        self.wf_img.setTransform(tr)
        self.wf_plot.setYRange(0.0, float(cap), padding=0.0)
        self.wf_plot.getAxis("left").setScale(self._wf_dt)         # tick values in seconds ago
    # ------------------------------------------------------------------------------------
    # Front-panel tidy-up (called once at the end of __init__, after every builder has run)
    # ------------------------------------------------------------------------------------
    # * the message box is two lines high, to leave the room to the graph and the waterfall
    # * the waterfall and the graph share a vertical splitter: drag the thin bar between them to set the height
    # * the RF Gain slider is shorter; BASEBAND SCOPE joins GAIN/RATE SURVEY, CAL and CAL SURVEY on the gain row
    # * STITCH CHECK (and its overlay tick box) joins the Show Freq Markers / Reference row
    # * those buttons, and Capture Reference, are all one size

    def layout_tidy(self, left_layout):
        self.view_init()
        # 0. these three start switched off (the tick boxes can be turned on at any time; the saturation line also has a
        #    settings.csv default)
        self.chk_markers.setChecked(False)
        self.chk_stitch_show.setChecked(False)
        # 1. message box: two lines
        fm = self.debug.fontMetrics()
        h = 2 * fm.lineSpacing() + 2 * self.debug.frameWidth() + int(2 * self.debug.document().documentMargin()) + 2
        self.debug.setMinimumHeight(h)
        self.debug.setMaximumHeight(h)

        # 2. graph + waterfall in a vertical splitter
        self.wf_install_splitter(left_layout)

        # 3. gain row: shorter slider, BASEBAND SCOPE moved in from its own row
        self.gain_slider.setMinimumWidth(110)
        self.gain_slider.setMaximumWidth(170)
        rf = self.find_layout_with(self.btn_survey)
        old = self.find_layout_with(self.btn_scope)
        if rf is not None and old is not None and old is not rf:
            old.removeWidget(self.btn_scope)
            rf.addWidget(self.btn_scope)
            rf.addStretch(1)
            # the scope read-out line belonged to the scope button's row; it is only meaningful in scope mode, so it now sits
            # at the top of the scope controls (which are only visible then) and the empty row disappears
            old.removeWidget(self.scope_status)
            vb = self.scope_ctrl_widget.layout()
            if vb is not None:
                vb.insertWidget(0, self.scope_status)
            try:
                left_layout.removeItem(old)
            except Exception:
                pass

        # 4. STITCH CHECK + overlay tick box -> the Show Freq Markers row
        mk = self.find_layout_with(self.chk_markers)
        st = self.find_layout_with(self.btn_stitch)
        if mk is not None and st is not None and st is not mk:
            st.removeWidget(self.btn_stitch)
            st.removeWidget(self.chk_stitch_show)
            mk.addWidget(self.btn_stitch)
            mk.addWidget(self.chk_stitch_show)
            try:
                st.removeWidget(self.stitch_label)
                left_layout.removeItem(st)
                left_layout.addWidget(self.stitch_label)      # its own line, only has text after a stitch check
            except Exception:
                pass

        # 5. one size for all of these buttons
        btns = [self.btn_scope, self.btn_survey, self.btn_cal_quick, self.btn_cal, self.btn_stitch, self.btn_capture_ref]
        w = max(b.sizeHint().width() for b in btns)
        hh = max(b.sizeHint().height() for b in btns)
        for b in btns:
            b.setFixedSize(w, hh)

        # 6. peak detection row (detector, peak hold, peak markers) under the Show Freq Markers row
        self.peak_build_ui(left_layout)

    def wf_install_splitter(self, left_layout):
        """Put the graph and the waterfall in one vertical splitter so the waterfall's height is adjustable."""
        idx = left_layout.indexOf(self.plot)
        if idx < 0:
            return
        self.wf_split = QtWidgets.QSplitter(QtCore.Qt.Vertical)
        self.wf_split.setChildrenCollapsible(False)
        self.wf_split.setHandleWidth(7)
        self.wf_split.setToolTip("")
        self.wf_split.setStyleSheet("QSplitter::handle { background: #c4c8cc; margin: 1px 0; } QSplitter::handle:hover { background: #7f9db5; }")
        left_layout.removeWidget(self.wf_box)
        left_layout.removeWidget(self.plot)
        self.wf_split.addWidget(self.plot)
        self.wf_split.addWidget(self.wf_box)
        self.wf_split.setStretchFactor(0, 3)
        self.wf_split.setStretchFactor(1, 2)
        left_layout.insertWidget(idx, self.wf_split, 3)
        self._wf_sized = False
        # scope mode hides self.plot: hide the whole splitter then (otherwise the waterfall would stay on screen)
        self._wf_vis_filter = VisibilityFilter(lambda: self.wf_split.setVisible(not self.plot.isHidden()), self)
        self.plot.installEventFilter(self._wf_vis_filter)

    # ------------------------------------------------------------------------------------
    # Initial view of the graph when a mode is selected
    # ------------------------------------------------------------------------------------
    # The first complete pass in a new band sets the X axis to the mode's frequency range and the Y axis to start just under the
    # lowest level measured (0.5 percentile, so one odd bin does not decide it) and to reach Y_SPAN_ABOVE_FLOOR_DB above it
    # (default 60 dB; settings.csv  setting,y_span_above_floor_db,60). After that the view is the user's: zoom and pan freely.
    # In single-frequency mode a new centre or sample rate only re-sets X (the spectrum moves with the centre); Y is left alone.

    def view_init(self):
        self._view_key = None
        self.view_signals = ViewSignals()
        self.view_signals.apply.connect(self.view_apply)

    def view_band_key(self):
        single = bool(getattr(self, "single_mode", False))
        return (round(float(self.START_FREQ)), round(float(self.STOP_FREQ)), single,
                round(float(self.SAMPLE_RATE)) if single else 0)

    def view_note_pass(self, lp, key0):
        """WORKER thread: called after each finished clean pass with the band key taken before the pass started."""
        if lp is None or key0 == getattr(self, "_view_key", ()) or self._survey is not None or self._stitch is not None:
            return
        if getattr(self, "view_signals", None) is None:
            return
        f = np.asarray(lp["f"], dtype=np.float64)
        p = np.asarray(lp["p"], dtype=np.float64)
        ok = np.isfinite(f) & np.isfinite(p)
        if ok.sum() < 20:
            return
        prev = self._view_key
        self._view_key = key0
        single = key0[2]
        x0, x1 = (float(f[ok].min()), float(f[ok].max())) if single else (float(key0[0]), float(key0[1]))
        if single and prev is not None and prev[2]:
            ylo = yhi = float("nan")                 # only the centre / rate changed: keep the user's Y range
        else:
            disp = p[ok] + self.display_offset_for(lp["rate"], lp["gain"])
            floor = float(np.percentile(disp, 0.5))
            ylo, yhi = floor - Y_MARGIN_BELOW_FLOOR_DB, floor + Y_SPAN_ABOVE_FLOOR_DB
        self.view_signals.apply.emit(x0, x1, ylo, yhi)

    def view_apply(self, x0, x1, ylo, yhi):
        """GUI thread: set the graph's view (this also stops the X auto-range, so the view stays where the user puts it)."""
        if x1 > x0:
            self.plot.setXRange(x0, x1, padding=0.01)
        if ylo == ylo and yhi == yhi and yhi > ylo:
            self.plot.setYRange(ylo, yhi, padding=0)
        try:
            self.cispr_update_curve()                # bring the right-hand CISPR axis into step
        except Exception:
            pass

    # ------------------------------------------------------------------------------------
    # dBFS display option
    # ------------------------------------------------------------------------------------
    # "Display in dBFS" switches every level shown to dB relative to ADC full scale (a full-scale sine reads 0 dBFS in its peak bin),
    # ignoring the dB -> dBuV calibration, the RF gain compensation and the attenuator. The raw SDR-output level is what you see,
    # so changing the RF gain moves the trace. The saturation line becomes simply 0 dBFS (and the orange line below it). CISPR limit lines
    # are in dBuV and are therefore switched off in this mode. The waterfall history and the captured reference trace were in the old
    # units, so they are cleared when the units are changed.

    def dbfs_build_ui(self):
        self.display_dbfs = bool(DISPLAY_DBFS)
        self.chk_dbfs = QtWidgets.QCheckBox("Display in dBFS")
        self.chk_dbfs.setToolTip(
            "Show levels in dB relative to ADC full scale (0 dBFS = a full-scale sine), ignoring the dB-to-dBuV calibration,\n"
            "the RF gain compensation and the attenuator. Clips at 0 dBFS. CISPR limits are dBuV, so they are switched off.\n"
            "Default: settings.csv  setting,display_dbfs,on|off")
        sub = self.find_layout_with(self.chk_markers)
        if sub is not None:
            sub.addWidget(self.chk_dbfs)
        else:
            self.centralWidget().layout().addWidget(self.chk_dbfs)
        self.chk_dbfs.setChecked(self.display_dbfs)
        self.chk_dbfs.toggled.connect(self.dbfs_on_toggled)
        self.dbfs_apply_labels()

    def display_offset_for(self, rate, gain):
        """dB to ADD to a stored 'SDR dB' value for the display in the current units - the whole chain, see "HOW THE dB -> dBuV
        CALIBRATION FACTORS ARE USED" near the top of the file:
            dBFS mode:  -(full-scale level)                                     (no calibration of any kind)
            dBuV mode:  step 1 factor of this sample rate, step 2 formula (factor - RF gain + attenuator), step 3 + antenna factor / LISN corr."""
        if getattr(self, "display_dbfs", False):
            return -self.sat_full_scale_p()
        g = 0.0 if self.hf_mode else float(gain)
        factor = cal_for_rate(rate)                                          # step 1: the factor of THIS sample rate
        offset = cal_model_offset_db(factor, g, self.input_atten_db)         # step 2: dBuV at the attenuator input
        return offset + float(getattr(self, "cispr_offset_db", 0.0))        # step 3: antenna factor (Radiated) / LISN correction (Conducted)

    def dbfs_apply_labels(self):
        try:
            self.plot.setLabel("left", self.units_label())
            self.chk_cispr.setEnabled(not self.display_dbfs)
            self.chk_cispr.setToolTip("CISPR limits are in dBuV - not available while the display is in dBFS." if self.display_dbfs else "")
        except Exception:
            pass

    def dbfs_on_toggled(self, on):
        rate = getattr(self, "_applied_sample_rate", None) or self.SAMPLE_RATE
        applied = getattr(self, "_applied_gain_db", None)
        gain = float(applied if applied is not None else self.gain_db)
        self.display_dbfs = False
        off_dbuv = self.display_offset_for(rate, gain)
        self.display_dbfs = True
        off_dbfs = self.display_offset_for(rate, gain)
        self.display_dbfs = bool(on)
        delta = (off_dbfs - off_dbuv) if on else (off_dbuv - off_dbfs)      # how far every displayed level moves
        try:
            vb = self.plot.getViewBox()
            y0, y1 = vb.viewRange()[1]
            vb.setYRange(y0 + delta, y1 + delta, padding=0)                  # keep the same data in view
        except Exception:
            pass
        if on and self.chk_cispr.isChecked():
            self.chk_cispr.setChecked(False)
        self.dbfs_apply_labels()
        self.wf_clear()
        if hasattr(self, "peak_curve"):
            self.peak_reset()                                     # the held maxima were in the old units
            self._peak_live = None
        if getattr(self, "reference_power", None) is not None:
            self.reference_power = None
            self.reference_freq = None
            try:
                self.reference_curve.setData([], [])
                self.reference_curve.setVisible(False)
            except Exception:
                pass
            self.log("Reference trace cleared (the display units changed)")
        self.log("Display units: " + ("dBFS (no calibration applied)" if on else "dBuV at the attenuator input"))
        self.update_status_panel()
        self.sat_update_lines()
        try:
            self.update_plot()
        except Exception:
            pass

    # ------------------------------------------------------------------------------------
    # Antenna factor / LISN correction: part of the displayed values
    # ------------------------------------------------------------------------------------
    # The box next to "Show CISPR limit" is "Antenna factor" in Radiated mode and "LISN corr." in Conducted mode. Its value is ADDED to
    # every displayed level, so the trace reads in dBuV/m (Radiated) or dBuV at the LISN measuring port (Conducted), and the limit line
    # is drawn on the same scale (the right axis now coincides with the left). With 0 dB the display is plain dBuV at the attenuator
    # input. Changing the value moves the trace, the waterfall colours, the reference trace and the saturation lines by the same amount
    # (the view slides with them, so the same data stays on screen). It is ignored in dBFS mode. The CAL does not use it.

    def corr_build_ui(self):
        self._corr_last = float(self.cispr_offset_db)
        self.cispr_offset_spin.valueChanged.connect(self.corr_changed)            # these run AFTER the existing handlers, which have
        self.cispr_mode_combo.currentIndexChanged.connect(self.corr_changed)      # already stored the new value in cispr_offset_db
        self.dbfs_apply_labels()

    def units_label(self):
        """Left-axis label for the current units."""
        if getattr(self, "display_dbfs", False):
            return "Level (dBFS)"
        if abs(float(self.cispr_offset_db)) < 1e-9:
            return "Level at input (dBµV)"
        return "Level at LISN port (dBµV)" if self.cispr_mode == "conducted" else "Field strength (dBµV/m)"

    def csv_level_name(self):
        """Column name for SAVE CSV in the current units."""
        if getattr(self, "display_dbfs", False):
            return "level_dbfs"
        if abs(float(self.cispr_offset_db)) < 1e-9:
            return "level_dbuv"
        return "level_dbuv_lisn" if self.cispr_mode == "conducted" else "level_dbuv_per_m"

    def corr_changed(self, *_):
        """GUI thread: the antenna factor / LISN correction (or the Radiated / Conducted choice) changed."""
        new = float(self.cispr_offset_db)
        old = float(self._corr_last)
        if abs(new - old) < 1e-9:
            return
        self._corr_last = new
        self.dbfs_apply_labels()
        if getattr(self, "display_dbfs", False):
            return                                        # corrections are ignored in dBFS: nothing on screen moves
        delta = new - old                                 # how far every displayed level moves
        try:
            vb = self.plot.getViewBox()
            y0, y1 = vb.viewRange()[1]
            vb.setYRange(y0 + delta, y1 + delta, padding=0)
        except Exception:
            pass
        if getattr(self, "_wf_buf", None) is not None:   # waterfall history: every row moves by the same amount
            self._wf_buf = self._wf_buf + np.float32(delta)
            if self._wf_levels is not None:
                self._wf_levels = (self._wf_levels[0] + delta, self._wf_levels[1] + delta)
            for sp in (self.wf_sp_lo, self.wf_sp_hi):
                sp.blockSignals(True)
                sp.setValue(sp.value() + delta)
                sp.blockSignals(False)
            self.wf_redraw()
        if getattr(self, "_peak_live", None) is not None:           # peak hold / marker data moves too
            self._peak_live = (self._peak_live[0], self._peak_live[1] + delta)
            if self._peak_y is not None:
                self._peak_y = self._peak_y + delta
                self.peak_curve.setData(self._peak_f, self._peak_y)
            self.peak_update_markers()
        if getattr(self, "reference_power", None) is not None:      # the captured reference trace moves too
            try:
                self.reference_power = np.asarray(self.reference_power, dtype=np.float64) + delta
                self.reference_curve.setData(self.reference_freq, self.reference_power)
            except Exception:
                pass
        self.sat_update_lines()
        try:
            self.update_plot()
        except Exception:
            pass
    # ------------------------------------------------------------------------------------
    # PEAK DETECTION: peak detector, peak hold (max hold) and peak markers
    # ------------------------------------------------------------------------------------
    # Three separate tools, on their own row under the Show Freq Markers row:
    #   * Detector  Average (default) | Peak.  How the captures inside ONE block are combined. Average: the mean power (smooth trace,
    #               bursts are diluted). Peak: the largest power seen in each bin over the captures - a short burst that falls inside the
    #               averaging window shows at full height. The number of captures is the Time Averaging count (tick it and set the number;
    #               with it off there is one capture per block and the two detectors are the same). Surveys, the CAL and the stitch check
    #               always use the average detector.
    #   * Peak hold Keeps, for every frequency, the highest level seen over successive passes (sweeps) and draws it as a second trace.
    #               It restarts by itself when the band changes or the units change; use Reset after changing the gain, the attenuator
    #               or the sample rate if you do not want the old maxima to stay.
    #   * Mark peaks Marks the N strongest peaks of the trace (the peak-hold trace if Peak hold is on, otherwise the live trace) within the
    #               part of the spectrum that is on screen, with their frequency and level in the units being displayed. Zoom in and the
    #               markers re-pick the strongest peaks of the part you are looking at. Updated once per finished sweep.
    # "Save peaks..." writes the peak table and the hold / live traces to a CSV file.

    def peak_build_ui(self, left_layout):
        self.detector_peak = False                 # read by the worker thread (sweep_loop)
        self.peak_hold_on = False
        self.peak_mark_on = False
        self._peak_f = None                        # peak-hold trace
        self._peak_y = None
        self._peak_key = None
        self._peak_live = None                     # (f, y) of the last finished pass
        self.peak_signals = PeakSignals()
        self.peak_signals.update.connect(self.peak_on_pass)

        self.peak_curve = pg.PlotDataItem(pen=pg.mkPen((255, 120, 0), width=1))
        self.peak_curve.setZValue(3)
        self.peak_curve.setVisible(False)
        self.plot.addItem(self.peak_curve)
        self.peak_scatter = pg.ScatterPlotItem(size=11, symbol="d", pen=pg.mkPen((0, 0, 0)), brush=pg.mkBrush(255, 235, 59))
        self.peak_scatter.setZValue(6)
        self.plot.addItem(self.peak_scatter)
        self.peak_labels = []
        for k in range(PEAK_MAX_MARKERS):
            t = pg.TextItem(text="", color=(255, 235, 59) if k == 0 else (235, 235, 235), anchor=(0.5, 1.15))
            t.setZValue(7)
            t.setVisible(False)
            self.plot.addItem(t)
            self.peak_labels.append(t)

        row = QtWidgets.QHBoxLayout()
        row.addWidget(QtWidgets.QLabel("Peak:"))
        self.cb_detector = QtWidgets.QComboBox()
        self.cb_detector.addItems(["Average detector", "Peak detector"])
        self.cb_detector.setToolTip("How the captures of one block are combined. Average: mean power (smooth). Peak: the largest power per bin,\n"
                                    "so short bursts are not diluted. The number of captures is the Time Averaging count (tick it and set a number).\n"
                                    "Surveys, the CAL and the stitch check always use the average detector.")
        self.cb_detector.currentIndexChanged.connect(self.peak_on_detector)
        row.addWidget(self.cb_detector)
        self.chk_peak_hold = QtWidgets.QCheckBox("Peak hold")
        self.chk_peak_hold.setToolTip("Keep the highest level seen at every frequency over successive sweeps (orange trace). Restarts when the band or the units\n"
                                      "change; press Reset after changing the gain, attenuator or sample rate.")
        self.chk_peak_hold.toggled.connect(self.peak_on_hold_toggled)
        row.addWidget(self.chk_peak_hold)
        btn_reset = QtWidgets.QPushButton("Reset peak")
        btn_reset.setToolTip("Clear the peak-hold trace and start again.")
        btn_reset.clicked.connect(self.peak_reset)
        row.addWidget(btn_reset)
        self.chk_peak_mark = QtWidgets.QCheckBox("Mark peaks")
        self.chk_peak_mark.setToolTip("Mark the strongest peaks of the trace on screen (of the peak-hold trace if Peak hold is on) with their frequency and level.")
        self.chk_peak_mark.toggled.connect(self.peak_on_mark_toggled)
        row.addWidget(self.chk_peak_mark)
        self.sp_peak_n = QtWidgets.QSpinBox()
        self.sp_peak_n.setRange(1, PEAK_MAX_MARKERS)
        self.sp_peak_n.setValue(5)
        self.sp_peak_n.setPrefix("top ")
        self.sp_peak_n.setToolTip("How many peaks to mark.")
        self.sp_peak_n.valueChanged.connect(lambda _v: self.peak_update_markers())
        row.addWidget(self.sp_peak_n)
        btn_save = QtWidgets.QPushButton("Save peaks...")
        btn_save.setToolTip("Save the peak table and the peak-hold / live traces to a CSV file.")
        btn_save.clicked.connect(self.peak_save_csv)
        row.addWidget(btn_save)
        row.addStretch(1)
        mk = self.find_layout_with(self.chk_markers)
        idx = -1
        for i in range(left_layout.count()):
            if left_layout.itemAt(i).layout() is mk:
                idx = i
        if idx >= 0:
            left_layout.insertLayout(idx + 1, row)
        else:
            left_layout.addLayout(row)
        self._peak_vb_filter = None
        try:
            self.plot.getViewBox().sigXRangeChanged.connect(lambda *_: QtCore.QTimer.singleShot(120, self.peak_update_markers))
        except Exception:
            pass

    # ---- switches ----
    def peak_on_detector(self, idx):
        self.detector_peak = (idx == 1)
        self.log("Detector: " + ("PEAK (largest power per bin over the captures of a block; count = Time Averaging)" if self.detector_peak else "average"))

    def peak_on_hold_toggled(self, on):
        self.peak_hold_on = bool(on)
        if not on:
            self.peak_curve.setVisible(False)
        else:
            self.peak_curve.setVisible(self._peak_f is not None)
        self.peak_update_markers()

    def peak_on_mark_toggled(self, on):
        self.peak_mark_on = bool(on)
        self.peak_update_markers()

    def peak_reset(self):
        self._peak_f = None
        self._peak_y = None
        self._peak_key = None
        try:
            self.peak_curve.setData([], [])
        except Exception:
            pass
        self.peak_update_markers()

    def units_short(self):
        """Short unit name for the labels."""
        if getattr(self, "display_dbfs", False):
            return "dBFS"
        if abs(float(self.cispr_offset_db)) < 1e-9:
            return "dBµV"
        return "dBµV (LISN)" if self.cispr_mode == "conducted" else "dBµV/m"

    # ---- data in ----
    def peak_note_pass(self, lp):
        """WORKER thread: a finished clean pass -> displayed levels, handed to the GUI for the hold trace and the markers."""
        if lp is None or self._survey is not None or self._stitch is not None:
            return
        f = np.asarray(lp["f"], dtype=np.float64)
        p = np.asarray(lp["p"], dtype=np.float64)
        ok = np.isfinite(f) & np.isfinite(p)
        f, p = f[ok], p[ok]
        if f.size < 8:
            return
        order = np.argsort(f)
        f, p = f[order], p[order]
        y = p + self.display_offset_for(lp["rate"], lp["gain"])               # SDR dB -> the display's units
        self.peak_signals.update.emit(f, y, f"{self.START_FREQ:.0f}-{self.STOP_FREQ:.0f}")

    def peak_on_pass(self, f, y, key):
        """GUI thread: a finished pass arrived."""
        f = np.asarray(f, dtype=np.float64)
        y = np.asarray(y, dtype=np.float64)
        self._peak_live = (f, y)
        if self.peak_hold_on:
            if self._peak_f is None or self._peak_key != key:
                self._peak_f, self._peak_y, self._peak_key = f.copy(), y.copy(), key          # first pass, or a new band: start afresh
            else:
                if f.size != self._peak_f.size or abs(f[0] - self._peak_f[0]) > 1e-3 or abs(f[-1] - self._peak_f[-1]) > 1e-3:
                    y = self.wf_to_grid(f, y[None, :], self._peak_f)[0].astype(np.float64)       # the bins moved (sample rate): fit to the held grid
                self._peak_y = np.fmax(self._peak_y, y)                                          # the highest level seen so far
            self.peak_curve.setData(self._peak_f, self._peak_y)
            self.peak_curve.setVisible(True)
        self.peak_update_markers()

    # ---- peak picking and markers ----
    @staticmethod
    def peak_find(f, y, n):
        """The n strongest local maxima of y(f), at least 2 % of the span apart. The strongest is always returned; the others must
        stand PEAK_MIN_ABOVE_MEDIAN_DB above the median level. Returns a list of (frequency, level), strongest first."""
        f = np.asarray(f, dtype=np.float64)
        y = np.asarray(y, dtype=np.float64)
        ok = np.isfinite(y)
        f, y = f[ok], y[ok]
        if y.size < 3:
            return []
        inner = np.flatnonzero((y[1:-1] > y[:-2]) & (y[1:-1] >= y[2:])) + 1
        cand = inner if inner.size else np.array([int(np.argmax(y))])
        cand = cand[np.argsort(y[cand])[::-1]]
        span = float(f[-1] - f[0]) or 1.0
        sep = max(8.0 * span / y.size, 0.02 * span)
        floor = float(np.median(y)) + PEAK_MIN_ABOVE_MEDIAN_DB
        out = []
        for i in cand:
            if out and (y[i] < floor or any(abs(f[i] - fo) < sep for fo, _ in out)):
                continue
            out.append((float(f[i]), float(y[i])))
            if len(out) >= n:
                break
        return out

    def peak_source(self):
        """The (f, y) the markers read: the peak-hold trace if it is on and exists, else the last finished pass."""
        if self.peak_hold_on and self._peak_f is not None:
            return self._peak_f, self._peak_y
        return self._peak_live if self._peak_live is not None else (None, None)

    def peak_update_markers(self):
        """GUI thread: place the peak markers and labels (strongest N peaks of what is on screen)."""
        try:
            for t in self.peak_labels:
                t.setVisible(False)
            self.peak_scatter.setData([], [])
            if not self.peak_mark_on:
                return
            f, y = self.peak_source()
            if f is None:
                return
            x0, x1 = self.plot.getViewBox().viewRange()[0]
            sel = (f >= x0) & (f <= x1)
            if int(sel.sum()) < 3:
                sel = np.ones(f.size, dtype=bool)
            peaks = self.peak_find(f[sel], y[sel], int(self.sp_peak_n.value()))
            if not peaks:
                return
            self.peak_scatter.setData([p[0] for p in peaks], [p[1] for p in peaks])
            unit = self.units_short()
            for k, (pf, pl) in enumerate(peaks):
                t = self.peak_labels[k]
                t.setText(f"{pf / 1e6:.4f} MHz\n{pl:.1f} {unit}")
                t.setPos(pf, pl)
                t.setVisible(True)
        except Exception as e:
            print("peak_update_markers:", e)

    def peak_save_csv(self):
        """Save the peak table (whole spectrum) and the peak-hold and live traces."""
        f, y = self.peak_source()
        if f is None:
            self.log("Peaks: nothing to save yet - run a sweep first")
            return
        path, _ = QtWidgets.QFileDialog.getSaveFileName(self, "Save peaks", "peaks.csv", "CSV (*.csv)")
        if not path:
            return
        unit = self.units_short()
        name = self.csv_level_name().replace("level_", "")
        live = None
        if self._peak_live is not None:
            lf, ly = self._peak_live
            live = ly if (lf.size == f.size and abs(lf[0] - f[0]) < 1e-3 and abs(lf[-1] - f[-1]) < 1e-3) else self.wf_to_grid(lf, ly[None, :], f)[0]
        hold = self._peak_y if (self.peak_hold_on and self._peak_f is not None) else None
        try:
            with open(path, "w", newline="") as fh:
                fh.write(f"# peaks of the {'peak-hold' if hold is not None else 'live'} trace, strongest first ({unit}); saved {time.strftime('%Y-%m-%d %H:%M')}\n")
                fh.write(f"# rank, frequency_hz, level_{name}\n")
                for k, (pf, pl) in enumerate(self.peak_find(f, y, int(self.sp_peak_n.value())), 1):
                    fh.write(f"# {k}, {pf:.0f}, {pl:.2f}\n")
                cols = ["freq_hz"] + ([f"hold_{name}"] if hold is not None else []) + ([f"live_{name}"] if live is not None else [])
                fh.write(",".join(cols) + "\n")
                for i in range(f.size):
                    row = [f"{f[i]:.0f}"]
                    if hold is not None:
                        row.append(f"{hold[i]:.2f}")
                    if live is not None:
                        row.append(f"{live[i]:.2f}")
                    fh.write(",".join(row) + "\n")
            self.log(f"Peaks saved to {path}")
        except Exception as e:
            self.log(f"Peaks: could not save: {e}")

    def survey_connect_signals(self):
        self.survey_signals = SurveySignals()
        self.survey_signals.done.connect(self.survey_finish)
        self.survey_signals.progress.connect(self.survey_sync_controls)
        self.btn_survey.setToolTip(
            "Automatically step through sample rates and RF gains on the current band (any width - wide "
            "bands are swept in several blocks) and write ONE HTML report: findings on sample-rate effects and "
            "gain-stage artefacts, charts and exportable data. The Sample Rate box and RF Gain slider follow the "
            "test being run. Press STOP to end early (a partial report is saved).")
        self.cal_build_ui()                # CAL and CAL SURVEY buttons + signals: known-tone calibration
        self.sat_build_ui()                # optional ADC saturation lines on the display
        self.wf_build_ui()                 # optional waterfall below the graph
        self.dbfs_build_ui()               # optional dBFS display (no calibration)
        self.corr_build_ui()               # antenna factor / LISN correction applied to the displayed values

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
        for r in SAMPLE_RATE_RECOMMENDED:
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
            # ...and the radio too (this thread is the only one allowed to touch the driver): it was left at the last survey rate and
            # gain, so the display used that rate's dB->dBuV factor until something re-applied the user's rate
            try:
                if self.sdr is not None:
                    self.sdr.sample_rate = self.SAMPLE_RATE
                    self._applied_sample_rate = self.SAMPLE_RATE
                    self.apply_gain_if_changed()
            except Exception as e:
                print(f"survey_loop: could not put the radio back: {e!r}")
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
        refuses to be narrower than its text, which would push the whole program window wider -
        off the screen. So it ignores its text width, wraps onto at most three lines and has a
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
        tails across the whole trace. The lower edge is a smooth high-pass that stops that
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

    def continuous_loop(self):
        try:
            self.settle_radio(float(self.centers[0]) if len(self.centers) else float(self.START_FREQ))
        except Exception as e:
            print(f"continuous_loop: settle_radio failed: {e!r}")
        while not self.stop_flag.is_set():
            self.single_apply_pending()     # single-frequency mode: a new centre chosen on the front panel starts here
            try:
                self.sweep_loop()   # run one sweep
                self.wf_fast_capture()   # waterfall, single-frequency fast rows (does nothing otherwise)
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
        p = p[valid] + self.cal_total_offset_db()   # SDR dB -> dBuV at attenuator input

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
        """
        self.curve.setData(f, p)
        self.cispr_update_curve()
        self.sat_update_lines()      # keep the ADC saturation lines in step with gain / attenuator / calibration

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
        p = self.power_axis[idx] + self.cal_total_offset_db()   # dBuV at attenuator input

        have_ref = (getattr(self, "reference_freq", None) is not None
                    and getattr(self, "reference_power", None) is not None
                    and len(self.reference_freq) > 0)

        if not have_ref:
            # No reference captured: write the live trace only.
            data = np.column_stack((f, p))
            np.savetxt(path, data, delimiter=",", header="freq_hz," + self.csv_level_name(), comments="")
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
            fh.write("freq_hz,%s,ref_freq_hz,ref_%s\n" % (self.csv_level_name(), self.csv_level_name()))
            for i in range(n):
                a = f"{f[i]:.6f}" if i < len(f) else ""
                b = f"{p[i]:.3f}" if i < len(p) else ""
                c = f"{rf[i]:.6f}" if i < len(rf) else ""
                d = f"{rp[i]:.3f}" if i < len(rp) else ""
                fh.write(f"{a},{b},{c},{d}\n")
        self.log(f"Saved live ({len(f)} pts) + reference ({len(rf)} pts) trace to {path}")

    # ---- closeEvent(): the version before the decoder was added is kept, commented out, just below the new one ----
    def closeEvent(self, event):
        self.dec_active = False
        self.decoder_audio_close()
        self.stop_flag.set()
        if self.sweep_thread and self.sweep_thread.is_alive():
            self.sweep_thread.join(timeout=1.0)
        if self.sdr is not None:
            try:
                self.sdr.close()
            except Exception:
                pass
        event.accept()

    #OLD# def closeEvent(self, event):
    #OLD#     self.stop_flag.set()
    #OLD#     if self.sweep_thread and self.sweep_thread.is_alive():
    #OLD#         self.sweep_thread.join(timeout=1.0)
    #OLD#     if self.sdr is not None:
    #OLD#         try:
    #OLD#             self.sdr.close()
    #OLD#         except Exception:
    #OLD#             pass
    #OLD#     event.accept()

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