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
SAMPLE_RATE_CHOICES = [0.25e6, 0.3e6,
                       1.0e6, 1.024e6, 1.2e6, 1.4e6, 1.5e6, 1.6e6, 1.8e6, 1.92e6, 2.0e6, 2.048e6,
                       2.4e6, 2.56e6, 2.8e6, 2.88e6, 3.0e6, 3.2e6]
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


def load_settings():
    """Read settings.csv. Returns (defaults_dict, presets_list, path_used).
    defaults_dict may hold start_freq, stop_freq, sample_rate, gain_db (any that were
    given). presets_list uses the same tuple layout as _FALLBACK_PRESETS. On any
    problem with the file as a whole, falls back to the built-in presets; a bad single
    row is reported and skipped."""
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
                    raise ValueError(f"unknown type {kind!r} (use 'default' or 'preset')")
            except Exception as e:
                print(f"settings: {os.path.basename(path)} row {rownum} skipped: {e}")
    except Exception as e:
        print(f"settings: could not read {path}: {e} - using built-in presets")
        return {}, list(_FALLBACK_PRESETS), None

    if not presets:
        print(f"settings: no valid presets in {path} - using built-in presets")
        presets = list(_FALLBACK_PRESETS)
    print(f"settings: loaded {len(presets)} presets from {path}")
    return defaults, presets, path


_SETTINGS_DEFAULTS, PRESETS, SETTINGS_PATH = load_settings()
START_FREQ = _SETTINGS_DEFAULTS.get("start_freq", START_FREQ)
STOP_FREQ = _SETTINGS_DEFAULTS.get("stop_freq", STOP_FREQ)
SAMPLE_RATE = _SETTINGS_DEFAULTS.get("sample_rate", SAMPLE_RATE)
GAIN_DB = _SETTINGS_DEFAULTS.get("gain_db", GAIN_DB)
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

# ---- Baseband SCOPE defaults (see the EmcScanner.scope_* methods) ----
SCOPE_MAX_SAMPLES = 2097152     # most samples captured per scope frame (2**21)
SCOPE_MIN_SAMPLES = 512         # fewest samples ever displayed per frame
SCOPE_CLIP_LEVEL = 0.99         # |I| or |Q| at/above this (1.0 = ADC full scale) counts as clipped
SCOPE_COMPLEX_DIVISOR = 128.0   # same normalisation _read_normalized_iq() applies to complex samples
                                # (if the scope's peak readout never gets above ~0.01 even on a strong
                                #  signal, your driver already returns +/-1.0 data: set this to 1.0)
SCOPE_TOP_TAPER = 0.10          # top 10 % below Nyquist rolls off smoothly (anti-alias skirt, less ringing)
SCOPE_WINDOWS_MS = [0.05, 0.1, 0.2, 0.5, 1, 2, 5, 10, 20, 50, 100, 200, 500, 1000]   # capture windows offered
# Scope spin boxes always use '.' as the decimal point, whatever the PC's regional settings are.
SCOPE_LOCALE = QtCore.QLocale(QtCore.QLocale.English, QtCore.QLocale.UnitedKingdom)


class ScopeSignals(QtCore.QObject):
    # worker thread -> GUI thread: (time_s, trace1, trace2_or_None, info_dict)
    frame = QtCore.pyqtSignal(object, object, object, object)


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
        self.dc_removal_enabled = True
        self.iq_balance_enabled = False

        # Label of whichever preset/MODE is currently active, shown on the stable status
        # panel (see update_status_panel()). "-" until a MODE button has been pressed.
        self.current_mode_label = "-"


        # ---- Driver selection at startup ----
        driver_dirs = {
            "Default (MSVC RTL-SDR Blog V4)": DRIVER_ROOT + "/default",
            "MS64 (libusb alt)": DRIVER_ROOT + "/ms64",
            "SDRSharp drivers Copy": DRIVER_ROOT + "/sdrsharp",
            "Experimental": DRIVER_ROOT + "/experimental"
        }

        if ENABLE_DRIVER_SELECTION:
            items = list(driver_dirs.keys())
            choice, ok = QtWidgets.QInputDialog.getItem(
                self,
                "Select RTL-SDR Driver",
                "Choose driver set:",
                items,
                0,
                False
            )
            if ok:
                self.driver_path = driver_dirs[choice]
            else:
                self.driver_path = driver_dirs["Default (MSVC RTL-SDR Blog V4)"]
        else:
            # Deterministic default path when selection is disabled
            self.driver_path = driver_dirs["Default (MSVC RTL-SDR Blog V4)"]

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
            self.sample_rate_combo.addItem(f"{rate_hz/1e6:.3f} MS/s", rate_hz)
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

        # ---- Signal-processing toggle row (artefact correction) ----
        # DC Offset Removal and IQ Balance Correction each target a specific
        # direct-conversion-receiver artefact that tracks center frequency / sample rate
        # rather than being a real signal - see _remove_dc_offset() / _correct_iq_imbalance().
        # DC Offset Removal defaults ON (confirmed to be the larger effect on this
        # hardware); IQ Balance Correction still defaults off - switch it on to see
        # whether it accounts for any remaining spike once DC removal is active.
        ctrl_layout3 = QtWidgets.QHBoxLayout()
        left_layout.addLayout(ctrl_layout3)

        self.chk_dc_removal = QtWidgets.QCheckBox("DC Offset Removal")
        self.chk_dc_removal.setChecked(self.dc_removal_enabled)
        self.chk_dc_removal.setToolTip(
            "Subtract each block's own mean IQ value before windowing/FFT. Targets the "
            "repeated spike at each block's center frequency caused by LO leakage / ADC "
            "DC bias - not a real signal if it moves when you change range or sample rate."
        )
        ctrl_layout3.addWidget(self.chk_dc_removal)
        self.chk_dc_removal.toggled.connect(self.dc_removal_on_toggle)

        self.chk_iq_balance = QtWidgets.QCheckBox("IQ Balance Correction")
        self.chk_iq_balance.setChecked(self.iq_balance_enabled)
        self.chk_iq_balance.setToolTip(
            "Blind per-block correction of I/Q gain and phase mismatch. Targets a mirrored "
            "spike reflected around each block's center frequency, caused by receiver I/Q "
            "imbalance rather than a real signal."
        )
        ctrl_layout3.addWidget(self.chk_iq_balance)
        self.chk_iq_balance.toggled.connect(self.iq_balance_on_toggle)

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
            btn.clicked.connect(lambda _, i=idx: self.set_preset(i))

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
        self.log("Setup complete")
        self.update_status_panel()   # show initial (module-default) settings until a MODE is picked
        self.sync_sample_rate_combo()
        self.scope_build_ui(left_layout)   # IQ / ENVELOPE SCOPE: second display, hidden until its button is pressed
        self.scope_build_squelch_ui()      # squelch row of the scope controls



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

    def stitch_get_edge_bins(self):
        """Number of FFT bins discarded from EACH edge of a block before stitching."""
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

    def survey_band_ok(self):
        """Check the current band is allowed for a survey; returns an error text or ''."""
        if self.START_FREQ == self.STOP_FREQ:
            return "pick a MODE with a frequency range first (not a single frequency)"
        if self.STOP_FREQ > SURVEY_MAX_STOP_HZ:
            return (f"the survey only runs on bands that end at or below "
                    f"{SURVEY_MAX_STOP_HZ/1e6:g} MHz (this band ends at {self.STOP_FREQ/1e6:g} MHz)")
        if not any(self.survey_rate_fits(r) for r in SAMPLE_RATE_CHOICES):
            return "no sample rate covers this band in a single block - choose a narrower band"
        return ""

    def survey_set_lock(self, locked):
        """Disable the controls that would fight the survey while it runs (STOP stays live)."""
        for wdg in ([self.sample_rate_combo, self.gain_slider, self.btn_survey,
                     self.btn_cont, self.btn_start] + list(self.mode_buttons)):
            wdg.setEnabled(not locked)

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
            f"Band: {self.START_FREQ/1e6:.3f} - {self.STOP_FREQ/1e6:.3f} MHz "
            f"(the current MODE). Averaging setting is used as-is.\n"
            f"Only sample rates that cover the whole band in one block can be selected."))

        form = QtWidgets.QFormLayout()
        gain_texts = [f"{g:.1f} dB" for g in RTL_GAIN_STEPS_DB]
        cb_lo, cb_hi = QtWidgets.QComboBox(), QtWidgets.QComboBox()
        cb_lo.addItems(gain_texts)
        cb_hi.addItems(gain_texts)
        cb_lo.setCurrentIndex(RTL_GAIN_STEPS_DB.index(12.5))
        cb_hi.setCurrentIndex(RTL_GAIN_STEPS_DB.index(33.8))
        sp_stride = QtWidgets.QSpinBox()
        sp_stride.setRange(1, 10)
        sp_stride.setValue(1)
        sp_rep = QtWidgets.QSpinBox()
        sp_rep.setRange(1, 5)
        sp_rep.setValue(1)
        form.addRow("Gain from", cb_lo)
        form.addRow("Gain to", cb_hi)
        form.addRow("Use every Nth gain step", sp_stride)
        form.addRow("Repeats per combination", sp_rep)
        lay.addLayout(form)

        lay.addWidget(QtWidgets.QLabel("Sample rates to test:"))
        lst = QtWidgets.QListWidget()
        for r in SAMPLE_RATE_CHOICES:
            fits = self.survey_rate_fits(r)
            it = QtWidgets.QListWidgetItem(f"{r/1e6:.3f} MS/s" + ("" if fits else "   (band too wide for one block)"))
            it.setData(QtCore.Qt.UserRole, float(r))
            if fits:
                it.setFlags(it.flags() | QtCore.Qt.ItemIsUserCheckable)
                it.setCheckState(QtCore.Qt.Checked)
            else:
                it.setFlags(it.flags() & ~(QtCore.Qt.ItemIsEnabled | QtCore.Qt.ItemIsUserCheckable | QtCore.Qt.ItemIsSelectable))
            lst.addItem(it)
        lst.setMinimumHeight(180)
        lay.addWidget(lst)

        info = QtWidgets.QLabel("")
        lay.addWidget(info)

        def chosen():
            lo, hi = sorted((cb_lo.currentIndex(), cb_hi.currentIndex()))
            gains = [float(g) for g in RTL_GAIN_STEPS_DB[lo:hi + 1:sp_stride.value()]]
            rates = [lst.item(i).data(QtCore.Qt.UserRole) for i in range(lst.count())
                     if (lst.item(i).flags() & QtCore.Qt.ItemIsUserCheckable)
                     and lst.item(i).checkState() == QtCore.Qt.Checked]
            return rates, gains, sp_rep.value()

        def refresh_info(*_):
            rates, gains, reps = chosen()
            info.setText(f"{len(rates)} rates x {len(gains)} gains x {reps} repeat(s) = "
                         f"{len(rates) * len(gains) * reps} passes")

        for wdg in (cb_lo, cb_hi):
            wdg.currentIndexChanged.connect(refresh_info)
        sp_stride.valueChanged.connect(refresh_info)
        sp_rep.valueChanged.connect(refresh_info)
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
        path, _ = QtWidgets.QFileDialog.getSaveFileName(
            self, "Survey results - base file name", "rf_survey.csv", "CSV Files (*.csv)")
        if not path:
            return
        self.survey_start(rates, gains, reps, path)

    def survey_start(self, rates, gains, repeats, path):
        if self._survey is not None:
            return
        problem = self.survey_band_ok()
        if problem:
            self.log(f"Survey not started: {problem}")
            return
        rates = [r for r in rates if self.survey_rate_fits(r)]
        if not rates:
            self.log("Survey not started: none of the chosen sample rates covers the band in one block")
            return
        base = os.path.splitext(path)[0]
        try:
            spikes_fh = open(base + "_spikes.csv", "w", newline="")
            summary_fh = open(base + "_summary.csv", "w", newline="")
        except Exception as e:
            self.log(f"Survey: cannot create result files: {e}")
            return
        spikes_w, summary_w = csv.writer(spikes_fh), csv.writer(summary_fh)
        spikes_w.writerow(["repeat", "sample_rate_hz", "gain_db", "freq_hz", "level_db",
                           "prominence_db", "baseline_db"])
        summary_w.writerow(["repeat", "sample_rate_hz", "gain_db", "n_avg", "bins",
                            "floor_db_median", f"spikes_over_{SURVEY_SPIKE_THRESH_DB:g}dB",
                            "max_prominence_db", "max_spike_freq_hz", "pass_seconds"])
        alive = bool(self.sweep_thread and self.sweep_thread.is_alive())
        self._survey = {
            "combos": [(r, g, k) for r in rates for g in gains for k in range(repeats)],
            "idx": -1, "base": base, "spikes_fh": spikes_fh, "summary_fh": summary_fh,
            "spikes_w": spikes_w, "summary_w": summary_w, "spectra": [],
            "t_start": time.time(), "orig_rate": self.SAMPLE_RATE, "orig_gain": self.gain_db,
            "was_running": alive, "pass_dur": 10.0, "seq_floor": 0, "t_set": time.time(),
            "timeout": 90.0, "skipped": 0,
        }
        self.log(f"=== SURVEY STARTED: {len(self._survey['combos'])} passes -> {base}_*.csv/.npz ===")
        self.survey_set_lock(True)
        self.survey_next_combo()           # sets rate/gain for the first combination
        if not alive:
            self.start_continuous()
        self._survey_timer.start()

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

    def survey_record(self, lp):
        s = self._survey
        rate, gain, rep = s["combos"][s["idx"]]
        f, p, floor, spikes = self.survey_analyse(lp["f"], lp["p"], SURVEY_SPIKE_THRESH_DB)
        s["pass_dur"] = lp["dur"]
        for fr, lv, pr, bs in spikes:
            s["spikes_w"].writerow([rep, f"{rate:.0f}", f"{gain:.1f}", f"{fr:.1f}",
                                    f"{lv:.2f}", f"{pr:.2f}", f"{bs:.2f}"])
        top = max(spikes, key=lambda x: x[2]) if spikes else None
        s["summary_w"].writerow([rep, f"{rate:.0f}", f"{gain:.1f}", lp["n_avg"], len(p),
                                 f"{floor:.2f}", len(spikes),
                                 f"{top[2]:.2f}" if top else "", f"{top[0]:.1f}" if top else "",
                                 f"{lp['dur']:.1f}"])
        s["spikes_fh"].flush()
        s["summary_fh"].flush()
        s["spectra"].append((rate, gain, rep, f, p.astype(np.float32)))
        n_done, n_all = s["idx"] + 1, len(s["combos"])
        elapsed = time.time() - s["t_start"]
        eta = elapsed / n_done * (n_all - n_done)
        self.log(f"Survey {n_done}/{n_all}: {rate/1e6:.3f} MS/s, {gain:.1f} dB -> "
                 f"{len(spikes)} spikes (ETA {int(eta // 60)}m{int(eta % 60):02d}s)")

    def survey_finish(self, aborted):
        s = self._survey
        if s is None:
            return
        self._survey_timer.stop()
        for k in ("spikes_fh", "summary_fh"):
            try:
                s[k].close()
            except Exception:
                pass
        specs = s["spectra"]
        if specs:
            try:
                lengths = np.array([len(x[3]) for x in specs])
                np.savez_compressed(
                    s["base"] + "_spectra.npz",
                    rate_hz=np.array([x[0] for x in specs]),
                    gain_db=np.array([x[1] for x in specs]),
                    repeat=np.array([x[2] for x in specs]),
                    start=np.concatenate(([0], np.cumsum(lengths)[:-1])),
                    length=lengths,
                    freq_hz=np.concatenate([x[3] for x in specs]),
                    level_db=np.concatenate([x[4] for x in specs]))
            except Exception as e:
                self.log(f"Survey: could not write spectra file: {e}")
        # put the operator's settings back
        self.survey_apply(s["orig_rate"], s["orig_gain"])
        self.survey_set_lock(False)
        if not s["was_running"] and not self.stop_flag.is_set():
            self.stop_sweep()
        self._survey = None
        self.log(f"=== SURVEY {'STOPPED EARLY' if aborted else 'FINISHED'}: {len(specs)} passes saved"
                 f"{', ' + str(s['skipped']) + ' skipped' if s['skipped'] else ''} -> "
                 f"{s['base']}_spikes.csv / _summary.csv / _spectra.npz ===")

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
        self.scope_auto_y = True
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
        self.scope_trig_spin.setDecimals(4)
        self.scope_trig_spin.setSingleStep(0.01)
        self.scope_trig_spin.setPrefix("Level ")
        self.scope_trig_spin.setToolTip("Trigger level as a fraction of ADC full scale. Also draggable as the "
                                         "orange line on the plot.")
        self.scope_trig_spin.valueChanged.connect(self.scope_on_trig_level_changed)

        self.chk_scope_auto_y = QtWidgets.QCheckBox("Auto Y")
        self.chk_scope_auto_y.setChecked(True)
        self.chk_scope_auto_y.toggled.connect(self.scope_on_auto_y_toggled)

        self.scope_y_spin = QtWidgets.QDoubleSpinBox()
        self.scope_y_spin.setLocale(SCOPE_LOCALE)
        self.scope_y_spin.setRange(0.0001, 10.0)
        self.scope_y_spin.setDecimals(4)
        self.scope_y_spin.setSingleStep(0.05)
        self.scope_y_spin.setValue(0.5)
        self.scope_y_spin.setPrefix("Y +/- ")
        self.scope_y_spin.setEnabled(False)
        self.scope_y_spin.setToolTip("Fixed vertical range (used when Auto Y is off).")
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
        """Auto Y on -> let pyqtgraph range it; off -> fixed +/- range (0..+ for the envelope alone)."""
        try:
            if self.scope_auto_y:
                self.scope_plot.enableAutoRange(axis=pg.ViewBox.YAxis, enable=True)
            else:
                v = float(self.scope_y_spin.value())
                lo = 0.0 if self.scope_channel == 1 else -v
                self.scope_plot.setYRange(lo, v, padding=0)
        except Exception:
            pass

    # ---- mode entry / exit (GUI thread) ----

    def scope_toggle(self, checked):
        if checked:
            if not self.scope_enter():
                self.btn_scope.blockSignals(True)
                self.btn_scope.setChecked(False)
                self.btn_scope.blockSignals(False)
        else:
            self.scope_exit()

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
        self.sweep_thread = threading.Thread(target=self.scope_loop, daemon=True)
        self.sweep_thread.start()
        return True

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

    def scope_process_capture(self, x, rate, n_disp, fc, gain):
        """
        Turn one raw IQ capture (tuned at fc = rate/4) into (time_s, trace1, trace2_or_None,
        info) ready to plot. Steps: metrics on the RAW capture (peak / clipping / DC) ->
        optional removal of the tuner's DC spike -> keep only RF 0..Nyquist (rejects the mirror
        image and anything above Nyquist; optional BW-limit low-pass) -> shift back to true RF
        and take the real part -> edge trigger -> slice n_disp samples for display.
        """
        n_total = int(x.size)
        n_disp = min(int(n_disp), n_total)

        # ---- metrics on the raw capture ----
        i_raw = x.real
        q_raw = x.imag
        peak = float(max(np.max(np.abs(i_raw)), np.max(np.abs(q_raw))))
        clipped = bool(np.any((np.abs(i_raw) >= SCOPE_CLIP_LEVEL) | (np.abs(q_raw) >= SCOPE_CLIP_LEVEL)))

        # ---- the tuner's own DC spike sits at the exact centre = RF Fs/4 ----
        if self.scope_remove_lo_spike:
            x = x - np.mean(x)

        # ---- band select: keep RF 0 .. Nyquist (mask cached; it only depends on these four) ----
        bw = float(self.scope_bw_hz)
        nyq = rate / 2.0
        key = (n_total, float(rate), float(fc), bw)
        cache = self._scope_mask_cache
        if cache is None or cache[0] != key:
            nu = np.fft.fftfreq(n_total, d=1.0 / rate) + fc          # RF frequency of every FFT bin
            h = ((nu >= 0.0) & (nu <= nyq)).astype(np.float64)
            t_edge = (1.0 - SCOPE_TOP_TAPER) * nyq                   # smooth skirt just below Nyquist
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
        sig = (z * lut[np.arange(n_total) & 3]).real.astype(np.float32)
        env = np.abs(z).astype(np.float32)

        ch = int(self.scope_channel)
        tr2 = None
        if ch == 0:
            tr1 = sig
        elif ch == 1:
            tr1 = env
        else:
            tr1, tr2 = sig, env

        # ---- edge trigger on trace 1 (10 % pre-trigger) ----
        mode = int(self.scope_trig_mode)
        pre = int(0.1 * n_disp)
        start = 0
        t0_idx = 0
        trig_text = "free run"
        if mode in (1, 2):
            lvl = float(self.scope_trig_level)
            a = tr1[:-1]
            b = tr1[1:]
            if mode == 1:
                hits = np.nonzero((a < lvl) & (b >= lvl))[0] + 1
            else:
                hits = np.nonzero((a > lvl) & (b <= lvl))[0] + 1
            hits = hits[(hits >= pre) & (hits <= n_total - n_disp + pre)]
            if hits.size:
                start = int(hits[0]) - pre
                t0_idx = pre
                trig_text = "triggered"
            else:
                trig_text = "NO TRIGGER (showing free run)"

        sl = slice(start, start + n_disp)
        tr1 = tr1[sl]
        if tr2 is not None:
            tr2 = tr2[sl]
        t = (np.arange(n_disp, dtype=np.float64) - t0_idx) / rate

        vpp = float(np.max(sig[sl]) - np.min(sig[sl])) if n_disp else 0.0
        kpk = int(np.argmax(np.abs(band)))
        text = (f"Input DC-{self._scope_fmt_hz(nyq)} (Fs {rate / 1e6:.3f} MS/s) | "
                f"{n_disp / rate * 1e3:.3f} ms | gain {gain:.1f} dB | "
                f"peak {peak:.2f} FS{'  ** CLIPPING **' if clipped else ''} | "
                f"pk-pk {vpp:.3g} FS | strongest {self._scope_fmt_hz(nu[kpk])} | {trig_text}")
        info = {"text": text, "clip": clipped, "trig": trig_text, "channel": ch}
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

                n_disp = int(round(self.scope_window_s * rate))
                n_disp = max(SCOPE_MIN_SAMPLES, ((n_disp + 255) // 256) * 256)
                n_total = n_disp if int(self.scope_trig_mode) == 0 else 2 * n_disp   # headroom to find an edge
                n_total = min(n_total, SCOPE_MAX_SAMPLES)
                n_total = max(256, (n_total // 256) * 256)
                n_disp = min(n_disp, n_total)

                x = self._scope_read_iq(n_total)
                gain = self.actual_gain_db if self.actual_gain_db is not None else self.gain_db
                res = self.scope_process_capture(x, rate, n_disp, fc, float(gain))
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
#     def scope_on_frame(self, t, tr1, tr2, info):
#         try:
#             self.scope_curve1.setData(t, tr1)
#             if tr2 is None:
#                 self.scope_curve2.hide()
#             else:
#                 self.scope_curve2.setData(t, tr2)
#                 self.scope_curve2.show()
#             self.scope_status.setText(info.get("text", ""))
#             self.scope_status.setStyleSheet(
#                 "QLabel { color: #FF4040; font-weight: bold; }" if info.get("clip") else "")
#             self._scope_last = (t, tr1, tr2, info)
#         finally:
#             self._scope_pending = False

    def scope_build_squelch_ui(self):
        """Adds the squelch row to the scope control panel (called once from __init__)."""
        from collections import deque
        self.scope_squelch = 0.0                    # FS; 0 = squelch off
        self._scope_level_hist = deque(maxlen=30)   # recent frames' trace peaks (for LEARN)
        self._scope_sq_hang = 0                     # frames to stay open after a signal drops
        r3 = QtWidgets.QHBoxLayout()
        self.scope_sq_spin = QtWidgets.QDoubleSpinBox()
        self.scope_sq_spin.setLocale(SCOPE_LOCALE)
        self.scope_sq_spin.setRange(0.0, 1.0)
        self.scope_sq_spin.setDecimals(4)
        self.scope_sq_spin.setSingleStep(0.001)
        self.scope_sq_spin.setSuffix(" FS")
        self.scope_sq_spin.setSpecialValueText("Off")
        self.scope_sq_spin.setKeyboardTracking(False)
        self.scope_sq_spin.setToolTip(
            "Squelch: when the peak of the displayed trace stays below this level the trace is "
            "blanked to a flat line, so receiver noise and tiny artefacts are tuned out. The "
            "readout shows the trace peak, so you can see what level to set. Re-learn after "
            "changing gain, sample rate or BW limit (the noise level changes with each).")
        self.scope_sq_spin.valueChanged.connect(self.scope_on_squelch_changed)
        self.btn_scope_learn = QtWidgets.QPushButton("LEARN NOISE FLOOR")
        self.btn_scope_learn.setToolTip(
            "With no signal connected, press this after a second or two of running: the squelch is "
            "set 30 % above the largest trace peak seen in the last ~30 frames.")
        self.btn_scope_learn.clicked.connect(self.scope_learn_squelch)
        r3.addWidget(QtWidgets.QLabel("Squelch"))
        r3.addWidget(self.scope_sq_spin)
        r3.addWidget(self.btn_scope_learn)
        r3.addStretch()
        self.scope_ctrl_widget.layout().addLayout(r3)

    def scope_on_squelch_changed(self, val):
        self.scope_squelch = float(val)
        self._scope_sq_hang = 0

    def scope_learn_squelch(self):
        hist = list(getattr(self, "_scope_level_hist", []))
        if not hist:
            self.log("Squelch: no frames captured yet - start the scope first")
            return
        peak = max(hist)
        thr = min(1.0, round(peak * 1.3, 4))
        self.scope_sq_spin.setValue(thr)           # also updates self.scope_squelch
        self.log(f"Squelch learned: largest trace peak {peak:.4f} FS over {len(hist)} frames "
                 f"-> squelch {thr:.4f} FS")

    def scope_on_frame(self, t, tr1, tr2, info):
        """
        Draw a frame (GUI thread). Squelch: if the peak of the displayed trace(s) is below the
        squelch level (with a 3-frame hang so an intermittent signal doesn't chatter), the
        trace is drawn as a flat line instead. The ORIGINAL data is what _scope_last keeps, so
        SAVE SCOPE CSV and LEARN NOISE FLOOR always see the real trace.
        """
        try:
            level = float(np.max(np.abs(tr1))) if len(tr1) else 0.0
            if tr2 is not None and len(tr2):
                level = max(level, float(np.max(np.abs(tr2))))
            hist = getattr(self, "_scope_level_hist", None)
            if hist is not None:
                hist.append(level)

            thr = float(getattr(self, "scope_squelch", 0.0))
            squelched = False
            if thr > 0.0:
                if level >= thr:
                    self._scope_sq_hang = 3
                elif self._scope_sq_hang > 0:
                    self._scope_sq_hang -= 1
                else:
                    squelched = True

            show1 = np.zeros_like(tr1) if squelched else tr1
            show2 = None if tr2 is None else (np.zeros_like(tr2) if squelched else tr2)
            self.scope_curve1.setData(t, show1)
            if show2 is None:
                self.scope_curve2.hide()
            else:
                self.scope_curve2.setData(t, show2)
                self.scope_curve2.show()

            text = info.get("text", "") + f" | trace peak {level:.4f} FS"
            if squelched:
                text = f"SQUELCHED (below {thr:.4f} FS) | " + text
            self.scope_status.setText(text)
            if info.get("clip"):
                self.scope_status.setStyleSheet("QLabel { color: #FF4040; font-weight: bold; }")
            elif squelched:
                self.scope_status.setStyleSheet("QLabel { color: #909090; }")
            else:
                self.scope_status.setStyleSheet("")
            self._scope_last = (t, tr1, tr2, info)
        finally:
            self._scope_pending = False

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
        while not self.stop_flag.is_set():
            self.sweep_loop()
            if not self._restart_pass:
                break

    def continuous_loop(self):
        while not self.stop_flag.is_set():
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