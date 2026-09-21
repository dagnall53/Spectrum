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


from PyQt5 import QtWidgets, QtCore
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
    
    # Use this to build centers across the sweep band
    # start_hz, stop_hz, sample_rate are floats
    def build_centers(self, start_hz, stop_hz, sample_rate):
                # first center is half a block above start
                first = start_hz + (sample_rate / 2.0)
                # stop is exclusive; np.arange will stop before stop_hz
                return np.arange(first, stop_hz, sample_rate, dtype=np.float64)

    def draw_block_markers(self):
            # remove old markers
            for item in self._block_markers:
                    try:
                            self.plot.removeItem(item)
                    except Exception:
                            pass
            self._block_markers.clear()
            # draw new markers for each center
            half = self.SAMPLE_RATE / 2.0
            for c in self.centers:
                    left = c - half
                    right = c + half
                    # vertical lines at left and right
                    line_l = pg.InfiniteLine(pos=left, angle=90, pen=pg.mkPen('g', width=1))
                    line_r = pg.InfiniteLine(pos=right, angle=90, pen=pg.mkPen('g', width=1))
                    self.plot.addItem(line_l)
                    self.plot.addItem(line_r)
                    self._block_markers.append(line_l)
                    self._block_markers.append(line_r)
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
        self.setWindowTitle("EMC FAST Sweep 150 kHz – 30 MHz")

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

        # ---- Debug console ----
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

        self.btn_start = QtWidgets.QPushButton("START FAST SWEEP")
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
            ("30M", "30M HF", 150e3, 30e6, 2.4e6, 120, True, 0.0),
            ("30M2", "30M HF2  ", 2e6, 30e6, 2.4e6, 120, True, 0.0),
            ("30M3", "30M NOT HF ", 2e6, 30e6, 2.4e6, 120, False, 0.0),
            ("MVHF", "Marine VHF", 156e6, 162e6, 2.4e6, 30, False, 0.0),
            ("VHF", "Broadcast VHF", 88e6, 108e6, 2.4e6, 37, False, 0.0),
            ("VHF HG", "High Gain  Bcst VHF", 88e6, 108e6, 2.4e6, 120, False, 0.0),
            ("FM_WB", "100.3M Wideband", 100.3e6, 100.3e6, 2.4e6, 37, False, 0.0),
            ("MVHF_WB", "Marine VHF Wideband", 156.875e6, 156.875e6, 2.4e6, 37, False, 0.0),
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
        import builtins
        real_print = builtins.print

        def gui_print(*args, **kwargs):
            text = " ".join(str(a) for a in args)
            self.log(text)
            real_print(*args, **kwargs)

        builtins.print = gui_print

        # ---- Final startup log ----
        self.log(f"Using driver path: {self.driver_path}")
        self.log("Setup complete")



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
        the same "effective" value in this one function, so they can't drift
        apart the way they could when two different code paths each redrew
        the curve with their own offset (as set_plot_data() used to, with its
        own hard-coded -80.0 "temporary visual calibration").
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

    def start_sweep(self):
        # Start a single fast stitched sweep (one-shot), using the LIVE preset settings
        # (self.START_FREQ/self.STOP_FREQ/self.SAMPLE_RATE/self.sdr_gain) - i.e. whichever
        # MODE button was last pressed, not the module-level defaults.
        if self.sweep_thread and self.sweep_thread.is_alive():
            return
        self.log("=== FAST SWEEP STARTED ===")
        self.log(f"Range: {self.START_FREQ/1e6:.3f} → {self.STOP_FREQ/1e6:.3f} MHz")
        self.log(f"Sample rate: {self.SAMPLE_RATE/1e6:.3f} MS/s")
        self.log(f"Gain: {self.sdr_gain} dB")

        # Build centers from current preset (same logic as start_continuous)
        if self.START_FREQ == self.STOP_FREQ:
            self.centers = np.array([self.START_FREQ], dtype=np.float64)
        else:
            self.centers = self.build_centers(self.START_FREQ, self.STOP_FREQ, self.SAMPLE_RATE)

        # Diagnostic: show centers and expected block ranges (MHz)
        print("start_sweep Diagnostic: centers (MHz):", (self.centers / 1e6).tolist())
        block_width_hz = self.SAMPLE_RATE
        ranges = [(c - block_width_hz/2.0, c + block_width_hz/2.0) for c in self.centers]
        print("     Diagnostic: expected block ranges (MHz):",
            [f"{a/1e6:.6f}-{b/1e6:.6f}" for a, b in ranges])
        self.log(f"Total hops: {len(self.centers)}")

        # Allocate and clear stitched arrays for this sweep
        total_bins = len(self.centers) * NFFT
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
        total_bins = len(self.centers) * NFFT
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

        expected_bins = len(self.centers) * NFFT
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

            # allow tuner/AGC to settle and flush driver buffers
            time.sleep(0.12)
            try:
                _ = self.sdr.read_samples(512)
                _ = self.sdr.read_samples(512)
            except Exception as e:
                print("sweep_loop: flush read_samples error:", e)

            # Read samples and enforce length
            samples = self.sdr.read_samples(NFFT)
            samples = samples[:NFFT]

            # Preserve complex IQ and normalize
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

            # FFT
            spec = np.fft.fftshift(np.fft.fft(samples * window))
            spec = spec / NFFT
            psd = 20 * np.log10(np.abs(spec) + 1e-12) + self._psd_to_dbuv_const

            # Frequency axis for this FFT block (use same fc). Use the LIVE sample rate
            # (self.SAMPLE_RATE), which set_preset() updates - not the module-level default.
            freqs = np.fft.fftshift(np.fft.fftfreq(NFFT, d=1.0 / self.SAMPLE_RATE)) + float(fc)

            # Diagnostic: block peak info and sample checksum
            peak_idx = np.nanargmax(psd)
            peak_freq = freqs[peak_idx]
            peak_level = psd[peak_idx]
            psd_checksum = np.sum(np.round(psd, 3))
            print(f"DIAG block {i}: peak {peak_freq/1e6:.6f} MHz @ {peak_level:.1f} dB, checksum {psd_checksum:.3f}")
            print("DIAG block sample freqs[0:6] (MHz):", (freqs[:6]/1e6).tolist())
            print("DIAG block sample psd[0:6]:", psd[:6].tolist())

            # Diagnostic: show block frequency span and planned storage indices
            fmin = freqs.min()
            fmax = freqs.max()
            start = i * NFFT
            stop = start + NFFT
            print(f"sweep_loop: block {i} freq span {fmin/1e6:.6f}-{fmax/1e6:.6f} MHz planned store [{start}:{stop}]")

            # Safety checks before writing into stitched arrays
            if self.freq_axis is None or self.power_axis is None:
                print("sweep_loop: ERROR - stitched arrays not allocated")
                break

            if stop > self.freq_axis.size:
                print(f"sweep_loop: ERROR - block {i} stop index {stop} exceeds array length {self.freq_axis.size}")
                break

            # Detect accidental overwrite (indicates logic bug)
            existing_mask = ~np.isnan(self.freq_axis[start:stop])
            if existing_mask.any():
                print(f"sweep_loop: WARNING - block {i} would overwrite existing data at indices {start}:{stop}")

            # Diagnostic: compute checksums and peaks for comparison
            peak_idx = np.nanargmax(psd)
            peak_freq = freqs[peak_idx]
            peak_level = psd[peak_idx]
            psd_checksum = float(np.sum(np.round(psd, 6)))
            print(f"sweep_loop DIAG: block {i} peak {peak_freq/1e6:.6f} MHz @ {peak_level:.2f} dB checksum {psd_checksum:.6f}")

            # Compare with previous block if present and non-empty
            if i > 0:
                prev = self.power_axis[start-NFFT:start]
                if not np.all(prev == -200.0):
                    prev_checksum = float(np.sum(np.round(prev, 6)))
                    prev_peak_idx = int(np.nanargmax(prev))
                    prev_peak_freq = float(self.freq_axis[start-NFFT:start][prev_peak_idx])
                    print(f"sweep_loop DIAG: prev block {i-1} peak {prev_peak_freq/1e6:.6f} MHz checksum {prev_checksum:.6f}")
                    if abs(psd_checksum - prev_checksum) < 1e-6:
                        print(f"sweep_loop: WARNING - block {i} PSD checksum equals previous block -> skipping write")
                        continue

            # Store block in stitched spectrum (single atomic write)
            self.freq_axis[start:stop] = freqs
            self.power_axis[start:stop] = psd

            # show current block overlay (debug only)
            try:
                if hasattr(self, "debug_block_curve"):
                    self.debug_block_curve.setData(freqs, psd)
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
            return

        path, _ = QtWidgets.QFileDialog.getSaveFileName(
            self, "Save Spectrum CSV", "spectrum.csv", "CSV Files (*.csv)"
        )
        if not path:
            return

        idx = np.argsort(self.freq_axis)
        f = self.freq_axis[idx]
        p = self.power_axis[idx]

        data = np.column_stack((f, p))
        np.savetxt(path, data, delimiter=",", header="freq_hz,level_db", comments="")

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