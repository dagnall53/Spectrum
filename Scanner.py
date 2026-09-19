import sys
import threading
import numpy as np
import os, ctypes
target_dir = r"C:\Spectrum"
# add the Current directory as it contains the crucial dll and retry
# os.add_dll_directory(target_dir)
# print(f"Called os.add_dll_directory({target_dir!r})")


from PyQt5 import QtWidgets, QtCore
import pyqtgraph as pg

init_done = False

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
    
    def __init__(self):
        super().__init__()

        # ---- Driver selection at startup ----
        driver_dirs = {
            "Default (MSVC RTL-SDR Blog V4)": "C:/Spectrum/drivers/default",
            "MS64 (libusb alt)": "C:/Spectrum/drivers/ms64",
            "SDRSharp drivers Copy": "C:/Spectrum/drivers/sdrsharp",
            "Experimental": "C:/Spectrum/drivers/experimental"
        }

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

        os.add_dll_directory(self.driver_path)
        # Now import the driver AFTER the DLL path is set


        # ---- Window title ----
        self.setWindowTitle("EMC FAST Sweep 150 kHz – 30 MHz")

        # RTL-SDR handle
        self.sdr = None

        # Sweep centers
        self.centers = np.arange(START_FREQ, STOP_FREQ, SAMPLE_RATE, dtype=np.float64)

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
        self.cispr_curve = self.plot.plot(pen=pg.mkPen('r', style=QtCore.Qt.DashLine))
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

        # ---- Right-hand preset panel ----
        preset_panel = QtWidgets.QVBoxLayout()
        main_layout.addLayout(preset_panel)

        btn_30m = QtWidgets.QPushButton("30M HF")
        btn_mvfh = QtWidgets.QPushButton("Marine VHF")
        btn_vhf  = QtWidgets.QPushButton("Broadcast VHF")
        btn_fmwb = QtWidgets.QPushButton("100.3M Wideband")
        btn_mvfhwb = QtWidgets.QPushButton("Marine VHF Wideband")


        preset_panel.addWidget(btn_30m)
        preset_panel.addWidget(btn_mvfh)
        preset_panel.addWidget(btn_vhf)
        preset_panel.addWidget(btn_fmwb)
        preset_panel.addWidget(btn_mvfhwb)

        preset_panel.addStretch()

        btn_30m.clicked.connect(lambda: self.set_preset("30M"))
        btn_mvfh.clicked.connect(lambda: self.set_preset("MVHF"))
        btn_vhf.clicked.connect(lambda: self.set_preset("VHF"))
        btn_fmwb.clicked.connect(lambda: self.set_preset("FM_WB"))
        btn_mvfhwb.clicked.connect(lambda: self.set_preset("MVHF_WB"))


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


    def set_preset(self, mode):
        global START_FREQ, STOP_FREQ, SAMPLE_RATE, GAIN

        if mode == "30M":
            START_FREQ = 150e3
            STOP_FREQ  = 30e6
            #STEP_HZ    = 2000
            SAMPLE_RATE = 2.4e6
            GAIN = 120
            self.hf_mode = True
            self.log("Setup 30M range")

        elif mode == "MVHF":
            START_FREQ = 156e6
            STOP_FREQ  = 162e6
            #STEP_HZ    = 2.4e6
            SAMPLE_RATE = 2.4e6
            GAIN = 30
            self.hf_mode = False
            self.log("Setup Marine VHF range")

        elif mode == "VHF":
            START_FREQ = 88e6
            STOP_FREQ  = 108e6
            #STEP_HZ    = 2.4e6
            SAMPLE_RATE = 2.4e6
            GAIN = 37
            self.hf_mode = False
            self.log("Setup VHF FM range")
            
        elif mode == "FM_WB":
            START_FREQ = 100.3e6
            STOP_FREQ  = 100.3e6   # single capture
            #STEP_HZ    = 1         # no stepping
            SAMPLE_RATE = 2.4e6
            GAIN = 37
            self.hf_mode = False
            self.log("Setup 100.3 MHz wideband view")
        elif mode == "MVHF_WB":
            START_FREQ = 156.875e6
            STOP_FREQ  = 156.875e6    # single capture
            #STEP_HZ    = 1            # no stepping
            SAMPLE_RATE = 2.4e6
            GAIN = 37                 # similar to FM_WB
            self.hf_mode = False
            self.log("Setup Marine VHF Channel 77 wideband view")


        print(f"Preset selected: {mode}")
        print(f"Range: {START_FREQ/1e6:.3f} MHz → {STOP_FREQ/1e6:.3f} MHz")

        # if mode == "FM_WB" or mode == "MVHF_WB":
        if START_FREQ == STOP_FREQ:   
            self.centers = np.array([START_FREQ], dtype=np.float64)
        else:
            self.centers = np.arange(START_FREQ, STOP_FREQ, SAMPLE_RATE, dtype=np.float64)

    
        # Reapply SDR settings (without re‑opening device)
        if self.sdr is not None:
            try:
                self.sdr.sample_rate = SAMPLE_RATE
                self.sdr.set_agc_mode(False)
                self.sdr.gain = GAIN
            except Exception as e:
                print("Error applying preset SDR settings:", e)

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
        if self.sweep_thread and self.sweep_thread.is_alive():
            return
        self.log("=== FAST SWEEP STARTED ===")
        self.log(f"Range: {START_FREQ/1e6:.3f} → {STOP_FREQ/1e6:.3f} MHz")
        # self.log(f"Step: {STEP_HZ} Hz")
        self.log(f"Sample rate: {SAMPLE_RATE/1e6:.3f} MS/s")
        self.log(f"Gain: {GAIN} dB")
        self.log(f"Total hops: {len(self.centers)}")

        self.stop_flag.clear()
        self.init_sdr()

        total_bins = len(self.centers) * NFFT
        self.freq_axis = np.full(total_bins, np.nan, dtype=np.float64)
        self.power_axis = np.full(total_bins, -200.0, dtype=np.float64)

        self.sweep_thread = threading.Thread(target=self.sweep_loop, daemon=True)
        self.sweep_thread.start()

    def stop_sweep(self):
        self.log(" STOP SWEEP ")
        self.stop_flag.set()


    def sweep_loop(self):
        window = np.hanning(NFFT)

        fft_bw = SAMPLE_RATE
        half_bw = fft_bw / 2.0

        for i, fc in enumerate(self.centers):
            if self.stop_flag.is_set():
                break

            # Tune
            self.sdr.center_freq = fc

            # Read samples
            samples = self.sdr.read_samples(NFFT)
            # enforce correct length
            samples = samples[:NFFT]
            samples = samples.real.astype(np.float32) / 128.0

            # FFT
            spec = np.fft.fftshift(np.fft.fft(samples * window))
            spec = spec / NFFT
            psd = 20 * np.log10(np.abs(spec) + 1e-12)
            psd = psd + 33.5

            # Frequency axis for this FFT block
            freqs = np.fft.fftshift(np.fft.fftfreq(NFFT, d=1.0 / SAMPLE_RATE))
            freqs = freqs + fc

            # Store block in stitched spectrum
            start = i * NFFT
            stop = start + NFFT

            self.freq_axis[start:stop] = freqs
            self.power_axis[start:stop] = psd

            if i % 5 == 0:
                self.update_plot()

        self.update_plot()
    def start_continuous(self):
        if self.sweep_thread and self.sweep_thread.is_alive():
            return

        self.log("=== CONTINUOUS MODE STARTED ===")
        self.stop_flag.clear()
        self.init_sdr()

        # Build centers from current preset (same logic as start_sweep)
        if START_FREQ == STOP_FREQ:
            self.centers = np.array([START_FREQ], dtype=np.float64)
        else:
            self.centers = np.arange(START_FREQ, STOP_FREQ, SAMPLE_RATE, dtype=np.float64)

        # If single-center, tune SDR to that center; if multi-center, tune to first center
        if len(self.centers) >= 1:
            self.sdr.center_freq = float(self.centers[0])

        # Allocate buffers sized to the number of FFT blocks we will stitch
        total_bins = len(self.centers) * NFFT
        self.freq_axis = np.full(total_bins, np.nan, dtype=np.float64)
        self.power_axis = np.full(total_bins, -200.0, dtype=np.float64)

        # Start repeating sweep in a thread
        self.sweep_thread = threading.Thread(target=self.continuous_loop, daemon=True)
        self.sweep_thread.start()

    # def start_continuous(self):

        # if self.sweep_thread and self.sweep_thread.is_alive():
            # return

        # self.log("=== CONTINUOUS MODE STARTED ===")

        # self.stop_flag.clear()
        # self.init_sdr()

        # # Force tuner to the current preset start frequency
        # self.sdr.center_freq = START_FREQ

        # # Continuous mode = single FFT at that frequency
        # self.centers = np.array([START_FREQ], dtype=np.float64)

        # # Allocate arrays for single capture

        # self.centers = np.arange(START_FREQ, STOP_FREQ, SAMPLE_RATE, dtype=np.float64)

        # total_bins = len(self.centers) * NFFT
        # self.freq_axis = np.full(total_bins, np.nan, dtype=np.float64)
        # self.power_axis = np.full(total_bins, -200.0, dtype=np.float64)


        # self.sweep_thread = threading.Thread(target=self.continuous_loop, daemon=True)
        # self.sweep_thread.start()


    def continuous_loop(self):
        while not self.stop_flag.is_set():
            self.sweep_loop()   # run one sweep

    def update_plot(self):
        if self.freq_axis is None or self.power_axis is None:
            return

        idx = np.argsort(self.freq_axis)
    
        f = self.freq_axis[idx].copy()
        p = self.power_axis[idx].copy()
        valid = ~np.isnan(self.freq_axis)
  
        self.plot_signals.update.emit(f, p)


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
