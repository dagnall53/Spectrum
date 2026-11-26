<img width="1196" height="594" alt="Screenshot of the RF scanner" src="https://github.com/user-attachments/assets/3c855afe-d875-4537-927c-740117cbf734" />

# RF Scan Portable

An experimental Python spectrum scanner for the **RTL-SDR Blog V4**. It sweeps a much wider frequency range than programs such as SDR#, stitches the blocks into one trace, shows it in dBµV, and lets you save the spectrum to CSV.

It was written for investigating emissions (for example from marine electronics) and for finding out which spectrum features are real and which are artefacts of the receiver.

> **Not for scientific research or compliance measurements.** The displays can still show artefacts caused by the IF, the sample rate and the gain stages. The program uses spectrum masks and averaging to reduce them, but care is needed when measuring. The CISPR limit line and the dBµV scale are for reference only unless you have calibrated them for your own antenna, attenuator and gain settings (see [Calibration](#calibration-dbµv)).

## Features

- Wide-range sweeps made of many FFT blocks (4096 bins), with the outer 15% of each block masked and the rest stitched edge to edge.
- MODE preset buttons (band, sample rate and gain), loaded from `settings.csv`.
- Live **RF Gain** slider and **Sample Rate** box that act immediately, even during a sweep.
- Time averaging, a frequency-marker overlay and a reference trace.
- Optional CISPR limit line (radiated or conducted).
- **Per-sample-rate dB to dBµV calibration** using the CAL button.
- Optional **ADC saturation line** that warns when the input is close to overloading the SDR.
- Optional **dBFS display** that ignores all calibration and shows the raw level relative to ADC full scale.
- Optional **waterfall** below the graph, to show effects that repeat or come and go.
- Test modes: **Single Frequency**, **Stitch Check**, **Gain/Rate Survey**, **CAL Survey** and a **Baseband Scope**.

## Requirements

- Windows PC (the program loads the RTL-SDR driver DLLs from a folder next to the script).
- An RTL-SDR Blog V4 dongle.
- Python 3 with:

  ```
  pip install numpy PyQt5 pyqtgraph
  ```

- The RTL-SDR driver installed for the dongle using **Zadig** (WinUSB driver). Follow the RTL-SDR Blog and SDR# instructions for this step.
- Next to `rf_scan_portable.py`:
  - the `pyrtlsdr_local` folder;
  - a `drivers` folder holding the driver DLL sub-folders (`default`, `ms64`, `sdrsharp`, `experimental`). The program also looks in `C:/Spectrum/drivers`.

## Starting the program

```
python rf_scan_portable.py
```

The DLL and driver choice is made in `settings.csv` (see below). The code was originally set up to switch between several driver locations while getting the dongle to work, and that option is still there if you need to experiment.

## Quick start

1. Plug in the dongle and start the program.
2. Click one of the **MODE** preset buttons (for example *VHF* or *150k-2M*). This sets the band, sample rate and RF gain.
3. Click **START CONTINUOUS** to sweep repeatedly, or **START SINGLE SWEEP** for one pass. **STOP** ends a sweep.
4. Adjust the **RF Gain** slider and **Sample Rate** box if needed. Both take effect straight away.
5. Tick **Time Averaging** and set the number of averages for a smoother trace (slower sweep).
6. Click **SAVE CSV** to save the current spectrum (`freq_hz,level_dbuv`).

### Controls at a glance

| Control | What it does |
|---|---|
| MODE buttons | Load a band / sample rate / gain preset from `settings.csv` |
| Single Frequency | Test mode: one centre frequency, one spectrum of the whole captured bandwidth. Set the frequency digit by digit (click a digit, scroll the mouse wheel to change it). Press any MODE button to go back to sweeping |
| START CONTINUOUS / START SINGLE SWEEP / STOP | Run or stop sweeps |
| SAVE CSV | Save the spectrum |
| Show CISPR limit | Show the CISPR limit line (radiated dBµV/m at 10 m, or conducted dBµV) |
| Antenna factor / LISN corr. | The box next to the Radiated / Conducted choice. Its value is **added to every displayed level**. See [Antenna factor and LISN correction](#antenna-factor-and-lisn-correction) |
| Time Averaging | Average several captures per block |
| Show Freq Markers | Show the green (raw block) and cyan dashed (kept block) edges. Off at start-up |
| Capture Reference / Show Reference | Store the current trace (for example the system noise with the equipment off) and show it for comparison only. Nothing is subtracted |
| Show ADC saturation line | See [Saturation line](#adc-saturation-line). Off at start-up |
| Display in dBFS | Show levels relative to ADC full scale, with no calibration. See [dBFS display](#dbfs-display) |
| Show waterfall | Show a spectrum history below the graph. See [Waterfall](#waterfall) |
| BASEBAND SCOPE | Time-domain view of the input from DC to half the sample rate. The button changes to *BACK TO SPECTRUM* |

### Sample rates

Only the sample rates the RTL2832U handles well are offered: **1.024, 1.8, 2.048, 2.4, 2.56 and 3.2 MS/s**. The chip does not accept 0.3 to 0.9 MS/s. Rates of about 2.4 to 2.56 MS/s give the most stable USB transfer. 3.2 MS/s can drop samples on many PCs and USB ports, so treat results at that rate with caution.

## settings.csv

Presets, defaults and program options are read at start-up from `settings.csv`. The file is searched for next to the program (or `.exe`), then in the current folder, then in `C:/Spectrum`. If it is missing, built-in presets are used.

Columns: `type,name,label,start,stop,sample_rate,gain_db,hf_mode,dbuv_cal`. Lines starting with `#` are comments. Frequencies accept `k`, `M` and `G` suffixes (`150k`, `2.4M`, `1G`).

```csv
# Sweep defaults used at start-up
type,name,label,start,stop,sample_rate,gain_db,hf_mode,dbuv_cal
default,,,88M,108M,2.4M,40.2,,
# MODE buttons: the label is the button text
preset,VHF,Broadcast VHF,88M,108M,2.4M,40.2,off,-47
preset,LF2,150k-2M 3.2,150k,2M,3.2M,40.2,off,-47
# Program options (the value goes in the label column)
setting,dc_offset_removal,on
setting,iq_balance_correction,on
setting,driver,default
setting,saturation_line,off
setting,display_dbfs,off
setting,y_span_above_floor_db,60
setting,saturation_margin_db,10
```

| Setting | Meaning |
|---|---|
| `dc_offset_removal` | Remove the DC offset from each capture (default on) |
| `iq_balance_correction` | Correct IQ imbalance (default on) |
| `driver` | `default`, `ms64`, `sdrsharp`, `experimental`, a folder name under the driver root, or a full path |
| `driver_root` | Folder that holds the driver sub-folders (optional) |
| `driver_dialog` | `on` to be asked which driver to use at start-up |
| `saturation_line` | Show the ADC saturation line at start-up (default off) |
| `display_dbfs` | Start with the display in dBFS (default off, meaning dBµV) |
| `y_span_above_floor_db` | Height of the graph, in dB, above the lowest level measured when a mode is selected (default 60) |
| `saturation_margin_db` | Distance of the orange warning line below full scale (default 10) |
| `waterfall` | Show the waterfall at start-up (default off) |
| `waterfall_rows` | Rows of history kept in the waterfall (default 150) |
| `cal_<sample rate in Hz>` | dB to dBµV factor for one sample rate, written by the CAL button (see below) |

## Calibration (dBµV)

The display converts the raw SDR level to dBµV at the input of your external attenuator:

```
dBµV = SDR level (dB) - RF gain (dB) + attenuator (dB) + dB-to-dBµV factor
```

The factor is **different for each sample rate** (and the CAL tone can be at any frequency; the factor applies at the frequency it was made at, which is noted in the comment line the CAL writes to `settings.csv`), because the receiver's response changes with sample rate. Rates with no calibrated value use the default `CAL_SDR_DB_TO_DBUV` at the top of the script.

### Calibrating with the CAL button

1. Set the **RF Gain** slider to about the value you will use for your measurements. The CAL is made at this gain, so the factor is right close to it.
2. Feed a **sine wave** of known frequency (1.5 MHz by default) and amplitude into the SDR through an attenuator. Use a level small enough not to overload the SDR at that gain. With the sine connected and CONTINUOUS running, the tone should sit well below the red saturation line.
3. Click **CAL...**. The dialog shows the RF gain that will be used. Enter the sine frequency, its amplitude (V peak-to-peak at the attenuator input) and the attenuator you fitted, and choose the sample rates to calibrate. The dialog also estimates how far below ADC full scale the tone will be.
4. The program measures every rate at the front-panel RF gain, with the tone at several positions in the block, and works out the best factor for each rate.
5. A table shows the old factor, the new factor, how much the level varies with tone position, the headroom below ADC full scale and the signal-to-noise ratio. A rate is rejected if the tone is too close to full scale or too close to the noise.
6. You are then asked what to do:
   - **Use and SAVE to settings.csv** keeps the factors permanently. The RF gain used is noted in a comment line in the file, and your previous file is copied to `settings.csv.bak` first.
   - **Use for this session only** applies them until you close the program.
   - **Discard** changes nothing. Use this to run CAL purely as a check.

The **Cal:** label on the front panel shows the factor in use for the current sample rate.

**Limits of the calibration:**

- The factor is made at one RF gain. The RTL-SDR's real gain differs from the nominal gain table (by up to about 14 dB at low settings), and the factor absorbs that difference at the gain it was made at. Displayed levels at other gains can therefore read high or low. Calibrate at, and measure at, about the same RF gain, and repeat the CAL if you change it.
- Levels are only reliable well below ADC full scale (see below).
- The level of the test tone at the SDR depends on your generator, so check that the amplitude you enter is really what reaches the attenuator (a generator set for a 50 ohm load read on an unterminated scope shows twice the real voltage).

### CAL SURVEY

**CAL SURVEY...** is a diagnostic. It feeds a known tone and measures how its displayed level changes with every sample rate and RF gain. It writes an HTML report (levels, charts and findings) plus CSV files. Use it to see where the readings can be trusted. You can enter several tone positions to map the tuner's passband at each rate.

### ADC saturation line

When **Show ADC saturation line** is ticked:

- The **red line** is the displayed level at which a single tone reaches ADC full scale, at the current RF gain and attenuator.
- The **orange dashed line** is 10 dB lower, where the front end starts to compress.

A peak at or above the orange line is not a trustworthy level, and above the red line the SDR is clipping. Reduce the RF gain or add attenuation. The line is a guide for single spikes: a very strong signal outside the displayed band, or many signals adding together, can overload the SDR without any one spike reaching it.

## Antenna factor and LISN correction

The box next to the Radiated / Conducted choice is **Antenna factor** in Radiated mode and **LISN corr.** in Conducted mode. Its value (in dB) is **added to every displayed level**, so the trace reads in:

- dBµV/m in Radiated mode (the antenna factor, plus any cable loss);
- dBµV at the LISN measuring port in Conducted mode (the correction for the LISN or coupling network).

With 0 dB the display is plain dBµV at the attenuator input, and the axis label tells you which units are shown. The CISPR limit line is drawn on the same scale as the trace, so you compare them directly. Each mode remembers its own value. Changing the value moves the trace, the waterfall colours, the reference trace and the saturation lines by the same amount, and the graph slides with them so the same data stays in view. The correction is ignored in dBFS mode. The CAL does not use it, so the per-rate factors never depend on it. **SAVE CSV** names its column `level_dbuv_per_m` or `level_dbuv_lisn` when a correction is in use.

## dBFS display

Tick **Display in dBFS** to show every level in dB relative to ADC full scale. A full-scale sine reads 0 dBFS in its peak bin. In this mode the dB-to-dBµV factors, the RF gain compensation and the attenuator are all **ignored**, so you see the raw level at the SDR output, and changing the RF gain moves the trace. The saturation line becomes simply 0 dBFS (red), with the orange line 10 dB below it. The CISPR limit is in dBµV, so it is switched off in this mode.

When you switch units the graph keeps showing the same data, the waterfall history and any captured reference trace are cleared (they were in the old units), and **SAVE CSV** labels its column `level_dbfs`.

## Graph view

When you select a mode, the first complete sweep sets the graph's view. The frequency axis covers the mode's range, and the level axis starts just under the lowest level measured and reaches `y_span_above_floor_db` (60 dB by default) above it. After that the view is yours, and zooming or panning is not undone until you pick another mode.

## Waterfall

Tick **Show waterfall** and a panel appears below the graph. **Drag the thin bar between the graph and the waterfall** to change how tall each one is. It shows the spectrum history with the **newest row at the top**, coloured by level in dBµV (same calibration as the trace). Its frequency axis is linked to the main graph, so zooming or panning either one moves both.

There are two kinds of row:

- **Sweep rows (default):** one row per finished sweep. In the normal modes a row is a whole stitched sweep. In single-frequency mode a row is one spectrum. Use this to see what comes and goes from sweep to sweep. The time axis is the sweep time, which is seconds per row.
- **Fast rows (single-frequency mode only):** tick **Fast rows**. After each normal pass the program records 128 x 4096 contiguous samples and shows every FFT as its own unaveraged row, about 1.7 ms per row at 2.4 MS/s. A burst that repeats every 200 ms then appears as a bright line every 117 rows or so. Batches are separated by a dark line, because the sweep pass between them is not in the picture. So a repeat period longer than one batch (about 0.2 s at 2.4 MS/s) only shows if more than one burst falls inside a batch.

Other controls:

| Control | What it does |
|---|---|
| Rows | How many rows of history to keep (20 to 2000) |
| Auto colours | Picks the colour range from the data. Untick it to set the bottom and top dBµV yourself |
| Clear | Empties the history. It also clears itself when the band (or, for fast rows, the centre frequency) changes, or when you switch between dBµV and dBFS. Changing the sample rate or the RF gain does **not** clear it |

Very wide bands are reduced to at most 4096 columns by taking the peak of each group of bins, so narrow spikes still show. Set the defaults with `setting,waterfall,on|off` and `setting,waterfall_rows,150` in `settings.csv`.

## Other test modes

- **STITCH CHECK** (the overlay tick box next to it is off at start-up) sweeps the band twice, the second time with every block shifted by half a block, and compares the two. Real signals look the same both times. Anything that depends on where a frequency falls in a block (LO or DC leakage, images, filter edges) differs and is shaded magenta.
- **GAIN/RATE SURVEY...** steps automatically through sample rates and RF gains on the current band and writes one HTML report. It is for finding which spectrum features are caused by the sample rate and which by the gain stages, and which signals are consistently sensed.
- **BASEBAND SCOPE** shows the time-domain signal. Because the receiver has a high-pass response, treat it as a "pseudo-scope" for signals from about 100 kHz up to half the sample rate.

## Known issues

- Birdies (spurious spikes) appear in the 150 kHz to 2 MHz range. They move with sample rate and become more obvious at higher RF gain (roughly above 20 dB). They appear to be internal to the receiver, not external signals. Compare two sample rates: a real signal stays put, an artefact moves.
- Stitching wide ranges can show discontinuities at block edges (for example near the DC/LO region of a block). The edge masking reduces this but does not remove it.
- 3.0 to 3.2 MS/s can show a regular pulse train and dropped USB samples that are not seen at 2.4 MS/s.
- Calibration is valid only at the gain it was done at, and only below ADC full scale (see above).
- Frequencies below about 28.8 MHz go through the V4's built-in upconverter. The scope and HF behaviour are not as well tested as the VHF/UHF modes.

## Notes on HF and direct sampling

In direct-sampling mode the R820T tuner is bypassed and the antenna feeds the RTL2832U ADC directly. There is then no tuner gain, and the "centre frequency" does not retune anything. A stock dongle is quite insensitive in this mode and usually needs an external preamp.

The **RTL-SDR Blog V4 does not need this mode for HF**: it routes anything below about 28.8 MHz through its upconverter automatically, so the normal tuned modes work. The tuner gain control is the only gain in the path.

## Development

The code was written with the help of Claude and ChatGPT/Microsoft Copilot. Claude was much more useful. If you want to add features or change the code, I recommend using Claude: Copilot tended to add spurious code, introduce bugs and suggest unwanted extra features.
