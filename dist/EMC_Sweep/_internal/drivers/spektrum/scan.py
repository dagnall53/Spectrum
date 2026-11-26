import sys
sys.path.append(r"C:\SDR")

from pyrtlsdr_local.rtlsdr import RtlSdr
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Button
import csv
import os
import ctypes
import os

# Load your local librtlsdr.dll
dll_path = os.path.join(r"C:\SDR\pyrtlsdr_local", "librtlsdr.dll")
rtl = ctypes.CDLL(dll_path)

# Function prototypes
rtl.rtlsdr_set_direct_sampling.argtypes = [ctypes.c_void_p, ctypes.c_int]
rtl.rtlsdr_set_offset_tuning.argtypes   = [ctypes.c_void_p, ctypes.c_int]
rtl.rtlsdr_set_bias_tee.argtypes        = [ctypes.c_void_p, ctypes.c_int]
rtl.rtlsdr_set_tuner_gain_mode.argtypes = [ctypes.c_void_p, ctypes.c_int]
rtl.rtlsdr_set_agc_mode.argtypes        = [ctypes.c_void_p, ctypes.c_int]
rtl.rtlsdr_set_if_freq.argtypes         = [ctypes.c_void_p, ctypes.c_uint32]



# -------- CONFIG --------
START_FREQ = 150e3
STOP_FREQ  = 10e6
SAMPLE_RATE = 2.4e6
FFT_SIZE = 4096*8
GAIN_DB = 0

N_AVG = 8
SMOOTH = 20
DSP_SCALE = 6
HF_COMP = 12
CROP = 0.10

# -------- SDR INIT --------
# sdr = RtlSdr()
# sdr.set_direct_sampling(2)
# sdr.sample_rate = SAMPLE_RATE
# sdr.set_agc_mode(0)
# sdr.gain = GAIN_DB

# -------- SDR INIT --------
sdr = RtlSdr()

# --- minimal HF-mode patch for RTL-SDR Blog V4 ---
sdr.set_direct_sampling(2)      # HF path
sdr.set_offset_tuning(0)        # required for V4 HF
sdr.set_agc_mode(0)             # disable tuner AGC
sdr.set_tuner_gain_mode(0)      # disable tuner gain control
sdr.set_bias_tee(0)             # ensure no bias tee
sdr.set_if_freq(0)              # force zero-IF mode
sdr.gain = 0                    # HF path has no tuner gain

sdr.sample_rate = SAMPLE_RATE


step_hz = SAMPLE_RATE

freqs = np.arange(START_FREQ, STOP_FREQ, step_hz)

# -------- STATE --------
stop_flag = False
live_mode = True
reference_fft = None
reference_freq = None

use_reference = True
got_reference = False

# ------------pure‑NumPy median filter
def median_filter_1d(x, k):
    k2 = k // 2
    y = np.zeros_like(x)
    for i in range(len(x)):
        i0 = max(0, i - k2)
        i1 = min(len(x), i + k2 + 1)
        y[i] = np.median(x[i0:i1])
    return y


# -------- DSP BLOCK --------
# def compute_psd_block(center_freq):
    # sdr.center_freq = center_freq

    # acc = np.zeros(FFT_SIZE)
    # window = np.hanning(FFT_SIZE)

    # for _ in range(N_AVG):
        # samples = sdr.read_samples(FFT_SIZE)
        # samples -= np.mean(samples)

        # w_samples = samples * window
        # spec = np.fft.fftshift(np.fft.fft(w_samples))

        # acc += np.abs(spec)   # magnitude, not power

    # psd = acc / N_AVG
    # psd = 20 * np.log10(psd + 1e-12)

    # return psd

def compute_psd_block(center_freq):
    sdr.center_freq = center_freq

    acc = np.zeros(FFT_SIZE)
    window = np.hanning(FFT_SIZE)

    for _ in range(N_AVG):
        samples = sdr.read_samples(FFT_SIZE)
        samples -= np.mean(samples)

        w_samples = samples * window
        spec = np.fft.fftshift(np.fft.fft(w_samples))

        # --- minimal spur masking ---
        low = int(FFT_SIZE * 0.02)   # lowest 2% of bins
        spec[:low] = 0               # kills the big IF spur

        acc += np.abs(spec)

    psd = acc / N_AVG
    psd = 20 * np.log10(psd + 1e-12)

    return psd

# -------- FFT STITCH + OPTIONAL SUBTRACT --------
def stitch_spectrum(blocks):
    spectrum = np.array(blocks).flatten()
    freq_axis = np.linspace(START_FREQ, STOP_FREQ, len(spectrum))
    return freq_axis, spectrum

def apply_reference(freq_axis, spectrum):
    global reference_fft, reference_freq, use_reference, got_reference

    if not use_reference or not got_reference:
        return freq_axis, spectrum

    if reference_fft is None:
        return freq_axis, spectrum

    if len(reference_fft) != len(spectrum):
        return freq_axis, spectrum

    return freq_axis, spectrum - reference_fft

# -------- CSV SAVE / LOAD --------
def save_fft_csv(filename, freq_axis, spectrum):
    with open(filename, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['freq_hz', 'psd_db'])
        for f_hz, p_db in zip(freq_axis, spectrum):
            w.writerow([f_hz, p_db])
            
def run_save_reference():
    global reference_fft, reference_freq, use_reference, got_reference
   
    blocks = []
    for f in freqs:
        psd = compute_psd_block(f)
        blocks.append(psd)

    freq_axis, spectrum = stitch_spectrum(blocks)

    filename = os.path.join(r"C:\SDR", "reference_fft.csv")
    save_fft_csv(filename, freq_axis, spectrum)
    print("Saved reference FFT to", filename)

    reference_freq = np.array(freq_axis)
    reference_fft = np.array(spectrum)
 
    # FORCE RAW DISPLAY HERE
    prev_use_reference = use_reference
    use_reference = False
    print("SAVE REF display: use_reference =", use_reference,
          "got_reference =", got_reference)
    print("REF length:", len(spectrum),
        "min:", np.min(spectrum),
        "max:", np.max(spectrum))

    display_fft(freq_axis, spectrum, " (Reference Saved)")
    
    use_reference = prev_use_reference

def load_reference_csv(filename):
    global reference_fft, reference_freq, use_reference, got_reference 
    freqs = []
    psd = []
    with open(filename, 'r', newline='') as f:
        r = csv.reader(f)
        header = next(r, None)
        for row in r:
            freqs.append(float(row[0]))
            psd.append(float(row[1]))
    reference_freq = np.array(freqs)
    reference_fft = np.array(psd)
    use_reference = True
    got_reference = True

# -------- MATPLOTLIB SETUP --------
plt.ion()
fig, ax = plt.subplots(figsize=(12,7.5))
# --- mouse readout (frequency + dB) ---
def format_coord(x, y):
    # x is in MHz because you plot freq_axis/1e6
    freq_hz = x * 1e6
    return f"Freq: {freq_hz:,.0f} Hz   Level: {y:.1f} dB"

ax.format_coord = format_coord

ax_stop = plt.axes([0.80, 0.01, 0.08, 0.05])
ax_scan = plt.axes([0.70, 0.01, 0.08, 0.05])
ax_live = plt.axes([0.60, 0.01, 0.08, 0.05])
ax_ref  = plt.axes([0.50, 0.01, 0.08, 0.05])
ax_save_ref = plt.axes([0.40, 0.01, 0.08, 0.05])
ax_ref_toggle = plt.axes([0.30, 0.01, 0.08, 0.05])
plt.subplots_adjust(bottom=0.18)

btn_stop = Button(ax_stop, 'STOP')
btn_scan = Button(ax_scan, 'SCAN')
btn_live = Button(ax_live, 'LIVE')
btn_ref  = Button(ax_ref, 'LOAD REF')
btn_save_ref = Button(ax_save_ref, 'SAVE REF')
btn_ref_toggle = Button(ax_ref_toggle, 'REF ON/OFF')

def save_ref_callback(event):
    run_save_reference()

# -------- BUTTON CALLBACKS --------
def ref_toggle_callback(event):
    global use_reference
    use_reference = not use_reference
    print("Reference subtraction:", "ON" if use_reference else "OFF")

def stop_callback(event):
    global stop_flag, live_mode
    stop_flag = True
    live_mode = False

def scan_callback(event):
    global live_mode
    live_mode = False
    run_single_scan()

def live_callback(event):
    global stop_flag, live_mode
    stop_flag = False
    live_mode = True
    run_live_sweep()

def ref_callback(event):
    # simple fixed filename for now
    filename = os.path.join(r"C:\SDR", "reference_fft.csv")
    if os.path.exists(filename):
        load_reference_csv(filename)
        print("Loaded reference FFT from", filename)
    else:
        print("Reference file not found:", filename)

btn_stop.on_clicked(stop_callback)
btn_scan.on_clicked(scan_callback)
btn_live.on_clicked(live_callback)
btn_ref.on_clicked(ref_callback)
btn_save_ref.on_clicked(save_ref_callback)
btn_ref_toggle.on_clicked(ref_toggle_callback)



# -------- DISPLAY --------
def display_fft(freq_axis, spectrum, title_suffix=""):
    global reference_fft, use_reference, got_reference
    # print("FFT display: use_reference =", use_reference,
    #      "got_reference =", got_reference)
    ax.cla()                     # safer than clear()
    ax.set_facecolor('black')    # restore axes background

    # restore spines (critical!)
    for spine in ax.spines.values():
        spine.set_visible(True)

    # restore ticks (critical!)
    ax.tick_params(axis='both', which='both', labelsize=10, color='white')

    ax.plot(freq_axis/1e6, spectrum, color='yellow')
    ax.set_xlabel("Frequency (MHz)")

    if use_reference and got_reference:
        ax.set_ylabel("Power above reference (dB)")
        ax.set_ylim(-10, 40)
    else:
        ax.set_ylabel("Power (dB)")
        ax.set_ylim(-180, 80)

    ax.set_xlim(START_FREQ/1e6, STOP_FREQ/1e6)
    ax.set_title(f"Spektrum-Style Sweep{title_suffix}")
    ax.grid(True)

    fig.canvas.draw_idle()
    plt.pause(0.01)

# -------- SINGLE SCAN --------
def run_single_scan():

    blocks = []
    for f in freqs:
        psd = compute_psd_block(f)
        blocks.append(psd)

    freq_axis, spectrum = stitch_spectrum(blocks)
    freq_axis, spectrum = apply_reference(freq_axis, spectrum)

    display_fft(freq_axis, spectrum, " (Single Scan)")

    # save current scan as CSV
    filename = os.path.join(r"C:\SDR", "last_scan_fft.csv")
    save_fft_csv(filename, freq_axis, spectrum)
    print("Saved FFT to", filename)

# -------- LIVE SWEEP --------
def run_live_sweep():
    global stop_flag, live_mode
    while live_mode and not stop_flag:
        blocks = []
        for f in freqs:
            if stop_flag or not live_mode:
                break
            psd = compute_psd_block(f)
            blocks.append(psd)

        if stop_flag or not live_mode:
            break

        freq_axis, spectrum = stitch_spectrum(blocks)
        freq_axis, spectrum = apply_reference(freq_axis, spectrum)

        display_fft(freq_axis, spectrum, " (Live)")

# -------- MAIN IDLE DISPLAY --------
display_fft(np.linspace(START_FREQ, STOP_FREQ, 1000),
            np.full(1000, -80.0),
            " (Idle)")
# --- START LIVE SWEEP IMMEDIATELY ---
if live_mode:
    run_live_sweep()
plt.ioff()
plt.show()

sdr.close()
