import sys
import threading
import numpy as np
import os, ctypes
import time
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
            half = SAMPLE_RATE / 2.0
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
        self.cispr_offset_db = 0.0
        self.cispr_gain_comp = 0.0
        self.cispr_effective_offset = 0.0
        
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

        # Sweep centers
        self.centers = self.build_centers(START_FREQ, STOP_FREQ, SAMPLE_RATE)
                # Diagnostic: show centers and expected block ranges (MHz)
        print("INIT Diagnostic: centers (MHz):", (self.centers / 1e6).tolist())
        block_width_hz = SAMPLE_RATE
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
        
        self.curve = self.plot.plot(pen='y')
        # ---- Right-hand CISPR axis ----
        self.plot.showAxis('right')
        self.plot.setLabel('right', 'CISPR (dBµV)')
        self.plot.getAxis('right').setPen(pg.mkPen(color='red', width=2))

        # CISPR curve (drawn using right axis)
        self.cispr_curve = pg.PlotCurveItem(
            pen=pg.mkPen(color='red', style=QtCore.Qt.DashLine, width=2),
            name="CISPR"
        )
        self.plot.addItem(self.cispr_curve)
        self.cispr_curve.hide()

        left_layout.addWidget(self.plot)
        # ---- Fixed Y-axis (no autoscale) ----
        self.plot.setYRange(-120, 20)
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
        self.btn_stop  = QtWidgets.QPushButton("STOP")
        self.btn_save  = QtWidgets.QPushButton("SAVE CSV")
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
        self.chk_cispr.toggled.connect(self.toggle_cispr)

        # ---- CISPR Offset Control (user calibration) ----
        self.cispr_offset_spin = QtWidgets.QDoubleSpinBox()
        self.cispr_offset_spin.setRange(-200.0, 200.0)
        self.cispr_offset_spin.setSingleStep(1.0)
        self.cispr_offset_spin.setDecimals(1)
        self.cispr_offset_spin.setValue(self.cispr_offset_db)   # initial value from __init__
        self.cispr_offset_spin.setSuffix(" dB")
        self.cispr_offset_spin.setToolTip("Visual offset to align CISPR (accounts for antenna/attenuation)")

        # place the control in the same control row
        ctrl_layout.addWidget(QtWidgets.QLabel("CISPR Offset"))
        ctrl_layout.addWidget(self.cispr_offset_spin)

        # connect to handler that updates effective offset and redraws CISPR curve
        self.cispr_offset_spin.valueChanged.connect(self.update_cispr_offset)


        # ---- Right-hand preset panel ----
        preset_panel = QtWidgets.QVBoxLayout()
        main_layout.addLayout(preset_panel)

        # Define presets as a list so we can index them and extend easily
        # Each entry: (internal_name, label, start_hz, stop_hz, sample_rate, gain, hf_mode_flag)
        self.presets = [
                ("30M", "30M HF",      150e3,      30e6,   2.4e6, 120, True),
                ("MVHF", "Marine VHF", 156e6,      162e6,  2.4e6, 30,  False),
                ("VHF",  "Broadcast VHF", 88e6,    108e6,  2.4e6, 37,  False),
                ("VHF HG",  "High Gain  Bcst VHF", 88e6,    108e6,  2.4e6, 120,  False),
                ("FM_WB","100.3M Wideband", 100.3e6, 100.3e6, 2.4e6, 37, False),
                ("MVHF_WB","Marine VHF Wideband", 156.875e6, 156.875e6, 2.4e6, 37, False),
        ]

        # Create buttons in a loop so adding presets is trivial
        for idx, (_, label, *_rest) in enumerate(self.presets):
                btn = QtWidgets.QPushButton(label)
                preset_panel.addWidget(btn)
                # bind the index into the lambda to avoid late-binding trap
                btn.clicked.connect(lambda _, i=idx: self.set_preset(i))

        preset_panel.addStretch()


        # ---- Sweep state ----
        self.stop_flag = threading.Event()
        self.sweep_thread = None
        self.freq_axis = None
        self.power_axis = None

        # ---- CISPR curve ----
        self.cispr_freqs, self.cispr_limits = self.build_cispr_curve()

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
        # Always reapply parameters
        self.sdr.sample_rate = SAMPLE_RATE
        self.sdr.set_agc_mode(False)
        self.sdr.gain = GAIN
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


    def set_preset(self, mode_index):
        """
        Set preset by index (preferred) or by name (string).
        Accepts:
        - integer index into self.presets (if defined)
        - string internal name (e.g., "MVHF")
        Falls back to built-in preset table if self.presets is not present.
        """
        global START_FREQ, STOP_FREQ, SAMPLE_RATE, GAIN

        # Built-in preset table (used if self.presets not defined)
        builtin_presets = [
            ("30M",     "30M HF",      150e3,      30e6,     2.4e6, 120, True),
            ("MVHF",    "Marine VHF",  156e6,      162e6,    2.4e6, 30,  False),
            ("VHF",     "Broadcast VHF", 88e6,     108e6,    2.4e6, 37,  False),
            ("FM_WB",   "100.3M Wideband", 100.3e6, 100.3e6, 2.4e6, 37, False),
            ("MVHF_WB", "Marine VHF Wideband", 156.875e6, 156.875e6, 2.4e6, 37, False),
        ]

        # Resolve active presets list (prefer self.presets if present)
        presets = getattr(self, "presets", builtin_presets)

        # Resolve index if a name was passed
        if isinstance(mode_index, str):
            found = [i for i, p in enumerate(presets) if p[0] == mode_index]
            if not found:
                print(f"set_preset: unknown preset name {mode_index!r}")
                return
            idx = found[0]
        else:
            try:
                idx = int(mode_index)
            except Exception:
                print(f"set_preset: invalid preset identifier {mode_index!r}")
                return

        # Validate index
        if idx < 0 or idx >= len(presets):
            print(f"set_preset: preset index {idx} out of range")
            return

        # Unpack preset tuple
        name, label, start_hz, stop_hz, sample_rate, gain, hf_mode_flag = presets[idx]

        # Apply to globals and instance state
        START_FREQ = float(start_hz)
        STOP_FREQ  = float(stop_hz)
        SAMPLE_RATE = float(sample_rate)
        GAIN = float(gain)
        self.hf_mode = bool(hf_mode_flag)
        # ---- CISPR OFFSET UPDATE (ALWAYS EXECUTED) ----
        # Gain compensation: CISPR curve must visually track mode gain
        self.cispr_gain_comp = GAIN

        # Effective offset = user offset - gain compensation
        self.cispr_effective_offset = self.cispr_offset_db - self.cispr_gain_comp
        # ------------------------------------------------
        # Log selection
        print(f"Preset selected: {label} (index {idx}, name {name})")
        print(f"Range: {START_FREQ/1e6:.6f} MHz → {STOP_FREQ/1e6:.6f} MHz")
        print(f"SAMPLE_RATE: {SAMPLE_RATE/1e6:.3f} MS/s, GAIN: {GAIN}")

        # Rebuild centers using canonical builder
        if START_FREQ == STOP_FREQ:
            self.centers = np.array([START_FREQ], dtype=np.float64)
        else:
            self.centers = self.build_centers(START_FREQ, STOP_FREQ, SAMPLE_RATE)

        # Diagnostic: show centers and expected block ranges (MHz)
        print("PRESET Diagnostic: centers (MHz):", (self.centers / 1e6).tolist())
        block_width_hz = SAMPLE_RATE
        ranges = [(c - block_width_hz/2.0, c + block_width_hz/2.0) for c in self.centers]
        print("       Diagnostic: expected block ranges (MHz):",
            [f"{a/1e6:.6f}-{b/1e6:.6f}" for a, b in ranges])

        # Update visual markers (safe call)
        try:
            self.draw_block_markers()
        except Exception:
            pass

        # Reapply SDR settings (without re-opening device)
        if self.sdr is not None:
            try:
                self.sdr.sample_rate = SAMPLE_RATE
                self.sdr.set_agc_mode(False)
                self.sdr.gain = GAIN
                # set direct sampling / tuner mode according to hf_mode
                try:
                    if self.hf_mode:
                        self.sdr.set_direct_sampling(2)
                        print("HF mode enabled (direct sampling input 2)")
                    else:
                        self.sdr.set_direct_sampling(0)
                        print("VHF/UHF tuner mode enabled")
                except Exception:
                    # some drivers may not support set_direct_sampling; ignore errors
                    pass
            except Exception as e:
                print("Error applying preset SDR settings:", e)

    def update_cispr_offset(self, val):
        """
        Slot for the CISPR offset spinbox.
        Updates the stored user offset, recomputes the effective offset
        (user offset minus current gain compensation) and refreshes the CISPR curve.
        """
        self.cispr_offset_db = float(val)
        # cispr_gain_comp is updated in set_preset(); ensure it exists
        if not hasattr(self, "cispr_gain_comp"):
            self.cispr_gain_comp = 0.0
        self.cispr_effective_offset = self.cispr_offset_db - self.cispr_gain_comp

        # Redraw CISPR curve (safe no-op if CISPR data not present)
        try:
            self.update_cispr_curve()
        except Exception:
            pass


    def build_cispr_curve(self):
        # CISPR 16-1-1 style placeholder curve (dBµV)
        f = np.array([9e3, 150e3, 500e3, 1e6, 30e6, 100e6, 200e6, 300e6, 600e6, 1e9
        ], dtype=np.float64)

        L= np.array([70, 66, 56, 56, 50, 40, 37, 33, 30, 28
        ], dtype=np.float64)
         ## very roughly with Tiny point antenna psd_dBuV = psd + 158.0

        return f, L

    def toggle_cispr(self, checked):
        if checked:
            self.cispr_curve.show()
        else:
            self.cispr_curve.hide()

    def start_sweep(self):
        # Start a single fast stitched sweep (one-shot)
        if self.sweep_thread and self.sweep_thread.is_alive():
            return
        self.log("=== FAST SWEEP STARTED ===")
        self.log(f"Range: {START_FREQ/1e6:.3f} → {STOP_FREQ/1e6:.3f} MHz")
        self.log(f"Sample rate: {SAMPLE_RATE/1e6:.3f} MS/s")
        self.log(f"Gain: {GAIN} dB")

        # Build centers from current preset (same logic as start_continuous)
        if START_FREQ == STOP_FREQ:
            self.centers = np.array([START_FREQ], dtype=np.float64)
        else:
            self.centers = self.build_centers(START_FREQ, STOP_FREQ, SAMPLE_RATE)

        # Diagnostic: show centers and expected block ranges (MHz)
        print("start_sweep Diagnostic: centers (MHz):", (self.centers / 1e6).tolist())
        block_width_hz = SAMPLE_RATE
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
        # init_sdr should be idempotent; it will skip reopen if already open
        self.init_sdr()
        self.sweep_thread = threading.Thread(target=self.sweep_loop, daemon=True)
        self.sweep_thread.start()

    def start_continuous(self):
        # Start continuous repeating stitched sweep
        if self.sweep_thread and self.sweep_thread.is_alive():
            return

        self.log("=== CONTINUOUS MODE STARTED ===")
        self.stop_flag.clear()
        # init_sdr should be idempotent; it will skip reopen if already open
        self.init_sdr()

        # Build centers from current preset (same logic as start_sweep)
        if START_FREQ == STOP_FREQ:
            self.centers = np.array([START_FREQ], dtype=np.float64)
        else:
            self.centers = self.build_centers(START_FREQ, STOP_FREQ, SAMPLE_RATE)

        # Diagnostic: show centers and expected block ranges (MHz)
        print("start_continuous Diagnostic: centers (MHz):", (self.centers / 1e6).tolist())
        block_width_hz = SAMPLE_RATE
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
            psd = 20 * np.log10(np.abs(spec) + 1e-12) + 33.5

            # Frequency axis for this FFT block (use same fc)
            freqs = np.fft.fftshift(np.fft.fftfreq(NFFT, d=1.0 / SAMPLE_RATE)) + float(fc)

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

    def update_cispr_curve(self):
        """
        Interpolate CISPR limits to the current freq axis and apply effective offset.
        Safe no-op if required data is missing.
        """
        # Guards
        if self.freq_axis is None:
            return
        if not hasattr(self, "cispr_freqs") or not hasattr(self, "cispr_limits"):
            return

        # Ensure numpy arrays
        try:
            fa = np.asarray(self.freq_axis, dtype=np.float64)
            freqs = np.asarray(self.cispr_freqs, dtype=np.float64)
            limits = np.asarray(self.cispr_limits, dtype=np.float64)
        except Exception:
            return

        # Interpolate CISPR limits to the current frequency axis
        try:
            L_interp = np.interp(fa, freqs, limits)
        except Exception:
            return

        # Ensure gain compensation and effective offset exist
        gain_comp = float(getattr(self, "cispr_gain_comp", 0.0))
        user_offset = float(getattr(self, "cispr_offset_db", 0.0))
        self.cispr_effective_offset = user_offset - gain_comp

        # Apply effective offset (visual calibration)
        L_interp = L_interp + float(self.cispr_effective_offset)

        # Update the CISPR curve item
        try:
            self.cispr_curve.setData(fa, L_interp)
        except Exception as e:
            # If the plot item isn't ready, ignore but log
            print("update_cispr_curve: setData error:", e)

        # Keep visibility in sync with checkbox
        try:
            if hasattr(self, "chk_cispr") and not self.chk_cispr.isChecked():
                self.cispr_curve.hide()
            else:
                self.cispr_curve.show()
        except Exception:
            pass

    def update_plot(self):

        # Basic guards
        if self.freq_axis is None or self.power_axis is None:
            return

        # Verify frequency uniqueness before plotting
        idx = np.argsort(self.freq_axis)
        f = self.freq_axis[idx].copy()
        p = self.power_axis[idx].copy()

        # Diagnostic: check for duplicate frequency bins
        if np.any(np.diff(f[~np.isnan(f)]) == 0):
            print("update_plot: WARNING - duplicate frequency values detected in freq_axis")

        idx = np.argsort(self.freq_axis)

        f = self.freq_axis[idx].copy()
        p = self.power_axis[idx].copy()
        valid = ~np.isnan(self.freq_axis)

        # --- Minimal additions for CISPR support ---
        # store the cleaned axis so CISPR interpolation has a stable reference
        # (this does not change your emitted data or sweep logic)
        try:
            self.freq_axis = f
            self.power_axis = p
        except Exception:
            pass
        # --------------------------------------------

        # Emit the plot update (unchanged)
        self.plot_signals.update.emit(f, p)

        # --- draw/update CISPR curve (safe no-op if data missing) ---
        try:
            self.update_cispr_curve()
        except Exception as e:
            print("update_plot: update_cispr_curve error:", e)
        # ----------------------------------------------------------

    @QtCore.pyqtSlot(object, object)
    def set_plot_data(self, f, p):
        self.curve.setData(f, p)

        if self.chk_cispr.isChecked():
            # Sweep band
            fmin = np.nanmin(self.freq_axis)
            fmax = np.nanmax(self.freq_axis)

            # Interpolate CISPR limits across sweep band
            cispr_f = self.cispr_freqs
            cispr_L = self.cispr_limits

            # If sweep band is outside CISPR table, clamp
            if fmin < cispr_f.min():
                fmin = cispr_f.min()
            if fmax > cispr_f.max():
                fmax = cispr_f.max()

            # Build interpolated CISPR curve across sweep band
            f_interp = np.linspace(fmin, fmax, 500)
            L_interp = np.interp(f_interp, cispr_f, cispr_L)
            # Apply temporary visual calibration
            L_interp = L_interp - 80.0
            
            # Display interpolated CISPR curve
            self.cispr_curve.setData(f_interp, L_interp)

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
