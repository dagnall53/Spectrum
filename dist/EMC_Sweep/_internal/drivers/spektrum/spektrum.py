import sys
sys.path.append(r"C:\SDR")

from pyrtlsdr_local.rtlsdr import RtlSdr
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Button

# -------- CONFIG --------
START_FREQ = 150e3
STOP_FREQ  = 30e6          # Spektrum EMC LF 30M preset
SAMPLE_RATE = 2.4e6
FFT_SIZE = 4096
GAIN_DB = 30

N_AVG = 4                  # Spektrum-style averaging
SMOOTH = 5                 # bin smoothing width
DSP_SCALE = 12             # Spektrum DSP scaling
HF_COMP = 22               # HF compensation
CROP = 0.10                # 10% crop

# -------- INIT --------
sdr = RtlSdr()
sdr.set_direct_sampling(0)     # tuner path (Spektrum mode)
sdr.sample_rate = SAMPLE_RATE
sdr.set_agc_mode(0)
sdr.gain = GAIN_DB

step_hz = SAMPLE_RATE / 2
freqs = np.arange(START_FREQ, STOP_FREQ, step_hz)

# -------- STOP BUTTON --------
stop_flag = False

def stop_callback(event):
    global stop_flag
    stop_flag = True

plt.ion()
fig, ax = plt.subplots(figsize=(12,6))

ax_button = plt.axes([0.85, 0.01, 0.12, 0.05])
btn_stop = Button(ax_button, 'STOP')
btn_stop.on_clicked(stop_callback)

# -------- MAIN LOOP --------
while not stop_flag:
    spectrum = []

    for f in freqs:
        if stop_flag:
            break

        sdr.center_freq = f

        # --- Spektrum-style averaging ---
        acc = np.zeros(FFT_SIZE)
        for _ in range(N_AVG):
            samples = sdr.read_samples(FFT_SIZE)
            samples -= np.mean(samples)
            window = np.hanning(FFT_SIZE)
            spec = np.fft.fftshift(np.fft.fft(samples * window))
            acc += np.abs(spec)

        psd = acc / N_AVG

        # --- Spektrum-style bin smoothing ---
        psd = np.convolve(psd, np.ones(SMOOTH)/SMOOTH, mode='same')

        # --- Convert to dB ---
        psd = 20 * np.log10(psd + 1e-12)

        # --- Spektrum DSP scaling ---
        psd += DSP_SCALE

        # --- HF compensation ---
        psd += HF_COMP

        # --- Crop percent ---
        crop_bins = int(FFT_SIZE * CROP)
        psd = psd[crop_bins:-crop_bins]

        spectrum.append(psd)

    if stop_flag:
        break

    spectrum = np.array(spectrum).flatten()
    freq_axis = np.linspace(START_FREQ, STOP_FREQ, len(spectrum))

    ax.clear()

    # --- Filled graph mode ---
    # ax.fill_between(freq_axis/1e6, spectrum, -120, color='yellow', alpha=0.3)
    ax.plot(freq_axis/1e6, spectrum, color='yellow')

    ax.set_xlabel("Frequency (MHz)")
    ax.set_ylabel("Power (dB)")
    ax.set_title("Spektrum-Style Sweep with Full DSP Smoothing")
    ax.grid(True)
    plt.pause(0.01)

# -------- CLEAN EXIT --------
sdr.close()
plt.ioff()
plt.show()
