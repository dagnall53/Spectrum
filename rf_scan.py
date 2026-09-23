import sys
import threading
import numpy as np
import os, ctypes
import time
import math
target_dir = r"C:\Spectrum"
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
GAIN        = 120     # or numeric (e.g. 30)

# ---- Stitching / edge-masking ----
# Each FFT block's outer edges show visible artefacts (front-end filter roll-off and/or
# window-shape effects), which show up as a regular "wave" pattern once many blocks are
# stitched edge-to-edge into one wideband trace. STITCH_EDGE_MASK_FRACTION is the fraction
# of each block's bandwidth discarded from EACH edge before it is stored/displayed; only
# the middle (1 - 2*STITCH_EDGE_MASK_FRACTION) of every block is kept. Centers are hopped
# by that same "kept" width (see build_centers()) so the kept slices tile the sweep with
# no gaps and no overlap. Adjust this constant if the FFT windowing/filtering changes.
STITCH_EDGE_MASK_FRACTION = 0.15   # 15% discarded off each edge (30% of each block total)

class PlotSignals(QtCore.QObject):
    update = QtCore.pyqtSignal(object, object)

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
        text = (
            f"Mode: {mode_label}\n"
            f"Start: {self.START_FREQ/1e6:.3f} MHz    Stop: {self.STOP_FREQ/1e6:.3f} MHz\n"
            f"Sample Rate: {self.SAMPLE_RATE/1e6:.3f} MS/s    Gain: {self.sdr_gain}\n"
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
        # stop is exclusive; np.arange will stop before stop_hz
        return np.arange(first, stop_hz, display_width, dtype=np.float64)

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
        # Both default OFF so existing behaviour is unchanged until switched on.
        self.dc_removal_enabled = True
        self.iq_balance_enabled = False

        # Label of whichever preset/MODE is currently active, shown on the stable status
        # panel (see update_status_panel()). "-" until a MODE button has been pressed.
        self.current_mode_label = "-"


        # ---- Driver selection at startup ----
        driver_dirs = {
            "Default (MSVC RTL-SDR Blog V4)": "C:/Spectrum/drivers/default",
            "MS64 (libusb alt)": "C:/Spectrum/drivers/ms64",
            "SDRSharp drivers Copy": "C:/Spectrum/drivers/sdrsharp",
            "Experimental": "C:/Spectrum/drivers/experimental"
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
        os.add_dll_directory(self.driver_path)

        # ---- Window title ----
        self.setWindowTitle("EMC Sweep")

        # RTL-SDR handle
        self.sdr = None

        # ---- Current sweep profile (mutable) ----
        # These start as copies of the module-level defaults, but from here on they are
        # the live settings actually used to sweep. set_preset() updates these when a MODE
        # button is pressed; start_sweep()/start_continuous()/init_sdr()/sweep_loop() all
        # read self.START_FREQ/self.STOP_FREQ/self.SAMPLE_RATE/self.sdr_gain, never the
        # bare module-level constants, so a preset change actually takes effect.
        self.START_FREQ = START_FREQ
        self.STOP_FREQ = STOP_FREQ
        self.SAMPLE_RATE = SAMPLE_RATE
        self.sdr_gain = GAIN

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
        # Both default OFF; switch them on independently to see which (if either) accounts
        # for a given spike.
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
        # Each entry: (internal_name, label, start_hz, stop_hz, sample_rate, gain, hf_mode_flag,
        #              dbuv_calibration)
        # dbuv_calibration is a fixed, mode-specific dB value (e.g. from comparing a known
        # reference signal's true dBuV level against what this mode reads). It is added to
        # the operator's live CISPR Offset spinbox value - see cispr_get_effective_offset().
        # All zero for now; fill these in as calibration data becomes available per mode.
        self.presets = [
            ("SF1", "4.5 2.048", 4.5e6, 4.5e6, 2.048e6, 60, False, 0.0),
            ("SF1", "4.5 2.8", 4.5e6, 4.5e6, 2.8e6, 60, False, 0.0),
            ("SF3", "2Mh B", 1e6, 1e6, 3.2e6, 60, True, 0.0),
            ("SF4", "2M B", 1e6, 1e6, 3.2e6, 60, False, 0.0),
            ("LF1", "2M HF", 150e3, 2e6, 2.4e6, 60, True, 0.0),
            ("LF2", "2M 3.2", 150e3, 2e6, 3.2e6, 60, False, 0.0),
            ("30M", "150k-30Mhz 1", 150e3, 30e6, 1.8e6, 60, False, 0.0),
            ("30M2", "2-30Mhz 2  ", 150e3, 30e6, 2.4e6, 60, False, 0.0),
            ("30M3", "2-30Mhz 3", 150e3, 30e6, 3.2e6, 60, False, 0.0),
            ("30-200M3", "30-200Mhz 3", 30e6, 200e6, 3.2e6, 60, False, 0.0),
            ("200-900M3", "200-900Mhz 3", 200e6, 900e6, 3.2e6, 60, False, 0.0),
            ("MVHF", "Marine VHF", 156e6, 162e6, 2.4e6, 60, False, 0.0),
            ("VHF", "Broadcast VHF", 88e6, 108e6, 2.4e6, 60, False, 0.0),
            ("VHF HG", "High Gain  Bcst VHF", 88e6, 108e6, 2.4e6, 60, False, 0.0),
            ("FM_WB", "100.3M Wideband", 100.3e6, 100.3e6, 2.4e6, 60, False, 0.0),
            ("MVHF_WB", "Marine VHF Wideband", 156.875e6, 156.875e6, 2.4e6, 60, False, 0.0),
        ]

        # Create buttons in a loop so adding presets is trivial
        for idx, (_, label, *_rest) in enumerate(self.presets):
            btn = QtWidgets.QPushButton(label)
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
        os.add_dll_directory(self.driver_path)
        
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
        # self.sdr_gain), which set_preset() updates when a MODE button is pressed.
        # Using the module-level SAMPLE_RATE/GAIN constants here would silently ignore
        # whichever mode the user last selected.
        self.sdr.sample_rate = self.SAMPLE_RATE
        self.sdr.set_agc_mode(False)
        self.sdr.gain = self.sdr_gain
        print("Gain set to:", self.sdr.get_gain())
            
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
        all read (self.START_FREQ, self.STOP_FREQ, self.SAMPLE_RATE, self.sdr_gain,
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
            # gain, hf_mode_flag, dbuv_calibration)
            _, _label, start_hz, stop_hz, sample_rate, gain, hf_flag, dbuv_cal = self.presets[preset_index]

            # Store preset values as the LIVE settings for other code to use
            self.START_FREQ = float(start_hz)
            self.STOP_FREQ = float(stop_hz)
            self.SAMPLE_RATE = float(sample_rate)

            # Preserve CISPR calibration; do not overwrite it with SDR preset gain.
            # Store SDR/preset gain separately so it cannot change CISPR mapping accidentally.
            self.sdr_gain = float(gain)

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
                     f"{self.SAMPLE_RATE/1e6:.3f} MS/s, gain {self.sdr_gain}, HF={self.hf_mode}, "
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

    def cispr_build_reference_curve(self):
        # CISPR 16-1-1 style placeholder curve (dBµV)
        f = np.array([9e3, 150e3, 500e3, 1e6, 30e6, 100e6, 200e6, 300e6, 600e6, 1e9
        ], dtype=np.float64)

        L= np.array([70, 66, 56, 56, 50, 40, 37, 33, 30, 28
        ], dtype=np.float64)
         ## very roughly with Tiny point antenna psd_dBuV = psd + 158.0

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
        samples = self.sdr.read_samples(n)
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
        # (self.START_FREQ/self.STOP_FREQ/self.SAMPLE_RATE/self.sdr_gain) - i.e. whichever
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
        # re-applies self.SAMPLE_RATE/self.sdr_gain/self.hf_mode to the hardware.
        self.init_sdr()
        self.sweep_thread = threading.Thread(target=self.sweep_loop, daemon=True)
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
        # re-applies self.SAMPLE_RATE/self.sdr_gain/self.hf_mode to the hardware.
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
        # Defensive checks at start
        if self.centers is None or len(self.centers) == 0:
            print("sweep_loop: no centers defined, exiting")
            return

        # Number of bins actually stored per block AFTER edge masking (see
        # stitch_get_kept_bins()) - this, not NFFT, is the real per-block stride into
        # freq_axis/power_axis, since the masked-off edge bins are never stored.
        edge_bins = self.stitch_get_edge_bins()
        kept_bins = NFFT - 2 * edge_bins

        expected_bins = len(self.centers) * kept_bins
        if self.freq_axis is None or self.power_axis is None or self.freq_axis.size != expected_bins:
            print(f"sweep_loop: stitched arrays missing or wrong size; reallocating to {expected_bins}")
            self.freq_axis = np.full(expected_bins, np.nan, dtype=np.float64)
            self.power_axis = np.full(expected_bins, -200.0, dtype=np.float64)

        # Diagnostic: show SDR object identity so we can detect mid-sweep reinitialisation
        try:
            print("sweep_loop: sdr object id:", id(self.sdr))
        except Exception:
            print("sweep_loop: sdr object not present")

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
            for _k in range(n_avg):
                if self.stop_flag.is_set():
                    break
                samples = self._read_normalized_iq(NFFT)
                spec = np.fft.fftshift(np.fft.fft(samples * window))
                spec = spec / NFFT
                power = np.abs(spec) ** 2
                power_acc = power if power_acc is None else power_acc + power
                captures_done += 1

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

    def stop_sweep(self):
        self.log(" STOP SWEEP ")
        self.stop_flag.set()

    def continuous_loop(self):
        while not self.stop_flag.is_set():
            self.sweep_loop()   # run one sweep

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


def main():
    app = QtWidgets.QApplication(sys.argv)
    win = EmcScanner()
    win.resize(1200, 600)
    win.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()