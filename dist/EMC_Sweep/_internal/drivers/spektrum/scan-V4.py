import numpy as np
from rtlsdr import RtlSdr
import time

#
# RTL‑SDR Blog V4 HF Scanner (Correct API Only)
# ---------------------------------------------
# - Uses direct sampling mode 2 (Q-branch)
# - Sweeps HF by retuning the ADC front-end
# - No calls to non-existent IF functions
# - Works with RTL‑SDR Blog Windows Release V1.4.0
#

# Sweep configuration
START_FREQ = 100_000      # 100 kHz
STOP_FREQ  = 30_000_000   # 30 MHz
STEP       = 200_000      # 200 kHz per hop
SAMPLE_RATE = 2_400_000   # 2.4 MSPS
N_SAMPLES   = 256 * 1024  # FFT block size

def scan_hf():
    sdr = RtlSdr()

    # HF mode: direct sampling on Q-branch
    sdr.set_direct_sampling(2)

    # Disable tuner AGC (not used in HF mode)
    sdr.set_agc_mode(False)

    # Set sample rate
    sdr.sample_rate = SAMPLE_RATE

    print("HF scanner started (RTL‑SDR Blog V4, direct sampling mode 2)")
    print(f"Sweeping {START_FREQ/1e6:.2f} MHz to {STOP_FREQ/1e6:.2f} MHz")

    freqs = np.arange(START_FREQ, STOP_FREQ, STEP)

    for f in freqs:
        # In HF mode, center_freq is a digital offset only
        sdr.center_freq = f

        # Read samples
        samples = sdr.read_samples(N_SAMPLES)

        # FFT
        window = np.hanning(len(samples))
        spectrum = np.fft.fftshift(np.fft.fft(samples * window))
        power_db = 20 * np.log10(np.abs(spectrum))

        # Display simple peak info
        peak = np.max(power_db)
        print(f"{f/1e6:6.3f} MHz  peak={peak:5.1f} dB")

        # Small delay to avoid USB overload
        time.sleep(0.05)

    sdr.close()
    print("HF scan complete.")

if __name__ == "__main__":
    scan_hf()
