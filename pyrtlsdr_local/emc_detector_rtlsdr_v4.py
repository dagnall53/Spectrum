import sys
sys.path.insert(0, r"C:\Users\dagna\AppData\Roaming\Python\Python314\site-packages")
sys.path.append(r"C:\SDR")

from pyrtlsdr_local.rtlsdr import RtlSdr

import time
import numpy as np
import matplotlib.pyplot as plt



# -------- CONFIGURATION --------
START_FREQ_HZ = 100e3      # 100 kHz
STOP_FREQ_HZ  = 30e6       # 30 MHz
SPAN_HZ       = 2.4e6      # per sweep chunk (must <= sample_rate)
SAMPLE_RATE   = 2.4e6
FFT_SIZE      = 16384
GAIN_DB       = 30         # adjust for your environment
INTEGRATION_SECONDS = 2.0  # per chunk
NOISE_FLOOR_ALPHA   = 0.1  # EWMA factor for noise-floor trending

# -------- RTL-SDR SETUP --------
sdr = RtlSdr()

# RTL-SDR V4 HF mode: direct sampling on Q-branch (2)
# If this fails, comment it out and configure via rtl_tcp/SDR++ instead.
try:
    sdr.set_direct_sampling(2)
except Exception as e:
    print("Direct sampling config failed, continuing anyway:", e)

sdr.sample_rate = SAMPLE_RATE
sdr.gain = GAIN_DB

# -------- FREQUENCY GRID --------
span_bins = int(FFT_SIZE)
freqs_rel = np.linspace(-SAMPLE_RATE/2, SAMPLE_RATE/2, span_bins)

# We will sweep center frequencies across HF band
center_freqs = []
cf = START_FREQ_HZ + SPAN_HZ / 2
while cf <= STOP_FREQ_HZ - SPAN_HZ / 2:
    center_freqs.append(cf)
    cf += SPAN_HZ

# Global noise-floor trend (per absolute frequency bin)
global_freqs = None
global_noise_floor = None

plt.ion()
fig, (ax_spec, ax_nf) = plt.subplots(2, 1, figsize=(10, 8))

def process_chunk(center_freq_hz):
    global global_freqs, global_noise_floor

    sdr.center_freq = center_freq_hz

    n_samples = int(SAMPLE_RATE * INTEGRATION_SECONDS)
    # Round to multiple of FFT_SIZE
    n_samples = (n_samples // FFT_SIZE) * FFT_SIZE
    if n_samples < FFT_SIZE:
        n_samples = FFT_SIZE

    samples = sdr.read_samples(n_samples)

    # FFT-based PSD
    samples = samples[:FFT_SIZE]
    window = np.blackman(FFT_SIZE)
    samples_win = samples * window
    spectrum = np.fft.fftshift(np.fft.fft(samples_win))
    psd = 20 * np.log10(np.abs(spectrum) + 1e-12)

    # Absolute frequency axis for this chunk
    freqs_abs = center_freq_hz + freqs_rel

    # Initialize global arrays on first run
    if global_freqs is None:
        global_freqs = freqs_abs.copy()
        global_noise_floor = psd.copy()
    else:
        # Merge: for overlapping bins, update EWMA noise floor
        # Simple nearest-neighbour mapping
        for f, p in zip(freqs_abs, psd):
            idx = np.argmin(np.abs(global_freqs - f))
            global_noise_floor[idx] = (
                (1.0 - NOISE_FLOOR_ALPHA) * global_noise_floor[idx]
                + NOISE_FLOOR_ALPHA * p
            )

    # Plot current spectrum chunk
    ax_spec.clear()
    ax_spec.plot(freqs_abs / 1e6, psd, label="Current spectrum")
    ax_spec.set_xlabel("Frequency (MHz)")
    ax_spec.set_ylabel("Power (dB)")
    ax_spec.set_title(f"EMC Spectrum (center {center_freq_hz/1e6:.3f} MHz)")
    ax_spec.grid(True)
    ax_spec.legend()

    # Plot global noise-floor trend
    ax_nf.clear()
    ax_nf.plot(global_freqs / 1e6, global_noise_floor, color="orange",
               label="Noise-floor trend (EWMA)")
    ax_nf.set_xlabel("Frequency (MHz)")
    ax_nf.set_ylabel("Power (dB)")
    ax_nf.set_title("Long-term Noise Floor")
    ax_nf.grid(True)
    ax_nf.legend()

    plt.tight_layout()
    plt.pause(0.01)


def main():
    print("Starting EMC detector sweep...")
    try:
        while True:
            for cf in center_freqs:
                process_chunk(cf)
            # Optional pause between full sweeps
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("Stopping...")
    finally:
        sdr.close()
        plt.ioff()
        plt.show()


if __name__ == "__main__":
    main()
