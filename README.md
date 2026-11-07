<img width="1196" height="594" alt="image" src="https://github.com/user-attachments/assets/3c855afe-d875-4537-927c-740117cbf734" />

This program is intended as an experimental Python tool to use with RTL_SDR.COM V4.
The intention is that it can be used to scan and view RF emissions over a wider frequency range than programs like SDRSHARP, and to allow the spectrum to be recorded.
The code was originally set up to select between a number of SDR DLL locations, whilst trying to "get it to work". 
The correct DLLs are set using ZADIG, and you should read the RTL and SDRSharp instructions on how to set these up.

There are still many notable "issues"- The displays can show artefacts related to IF and other issues. The program should not be relied upon for scientific research in its current form. 
Currently the program employs spectrum masks and averaging to help reduce the visual effect of these artefacts, but care should be used when "measuring".

A Crispr spectrum in dBuV can be displayed, but this will need to be calibrated (offest) for the exact antenna/gain settings used, and is mainly for reference.  

It has been written with assistance by Claude and ChatGPT/Microsoft Co-pilot. But the input from Claude has been much more useful.
 
If you wish to add features or modify the code I would recommend using Claude for this, as Copilot has a tendency to add spurious code, add bugs and suggest unwanted additional features.


Notes:
What set_direct_sampling(2) actually does

Your normal (non-HF) scans go: antenna → R820T tuner (mixer + LNA + local oscillator) → tuner mixes your chosen frequency down to baseband → ADC digitizes that baseband signal. The tuner is what makes center_freq mean anything.

HF/direct-sampling mode disconnects the R820T tuner from the signal path entirely and feeds the antenna straight into the RTL2832U's ADC (the "Q-branch" input, hence mode 2). There is no mixer, no LO, and — critically — no R820T at all in that path. You're just digitizing the raw antenna voltage.

That has two consequences that line up exactly with what you're seeing:

gain_db does nothing in HF mode. Your self.sdr.gain = self.gain_db call sets the R820T tuner's gain — but the tuner is bypassed in direct sampling. There is no gain stage at all on a stock dongle in this mode; sensitivity is fixed and it's quite low. If you need to actually detect a weak HF signal, a passive dongle alone usually isn't enough — you typically need an external LNA/preamp ahead of it (or a proper HF upconverter/SDRplay-class front end).
center_freq is essentially a label, not a tuning command, in this mode. Since there's no mixer, setting self.sdr.center_freq = fc before each HF block doesn't retune anything physically — there's nothing to retune. What you're actually capturing is always the same fixed baseband window from 0 Hz up to sample_rate/2 (so at 2.4 MS/s, roughly 0–1.2 MHz), no matter what fc your code requested. Your sweep code then labels that captured spectrum using freqs_full = fftfreq(...) + fc, which in HF mode is putting a frequency axis on data that wasn't actually tuned to that point — so blocks at different "planned centers" in HF mode may just be re-labelling the same raw capture rather than showing different parts of the spectrum.

BUT V4 is different, and The V4 should auto-route through its upconverter for anything below ~28.8 MHz.

Has a Python RTL-SDR spectrum scanner program, rf_scan_portable.py (PyQt5 + pyqtgraph), with MODE preset buttons; presets and sweep defaults now load from a settings.csv file
Sees multiple birdies (spikes) in the 150 kHz to 2 MHz band that change frequency with sample rate and appear at higher RF gain (roughly above 20-21 dB); a 10 dB attenuator at the antenna made no significant difference, so believes it is an internal artefact
Asked for a live RF Gain slider, with Sample Rate moved onto the same row and both acting immediately
Asked for an automatic gain/sample-rate survey mode that saves data for analysis; wants it limited to bands up to 1.2 MHz (single block per pass); uploaded first survey results (2.8 and 3.2 MS/s)
Wants a scope-like time-domain display mode that switches away from the spectrum view; chose the tuned-IQ / envelope scope approach (option 2) and asked for an appropriately labelled selection button
Wants the scope display to show exactly what the probe sees, like a normal scope: signals only up to half the sample rate, with no centre-frequency or bandwidth controls (the Sample Rate sets the range); also reported spin boxes showing decimal commas (French notation) and wants them to use a decimal point
Scope tests: at 3.0 and 3.2 MS/s a regular pulse train appears (also with a 1 ms window, not probe-related) and disappears at 2.4 MS/s; injected a 3.3 V 200 kHz sine through a 20 dB attenuator: RF gain slider changed the amplitude only slightly, 200 kHz read 0.0099 FS pk-pk and 100 kHz about 0.002 FS, and square/triangle waves looked unrecognisable
The scope Auto phase (mirror-band phase recovery) made a sine wave look like a square wave, so it was reverted; wants switching Scope to Spectrum, and changing spectrum presets, to start CONTINUOUS automatically; noticed the spectrum needs the RF gain swept up and back down after returning from the scope before it looks clean at low gain
Has changed the scope's label to "Pseudo-SCOPE (100KHz- to Fs/2)" because of the receiver's high-pass response; code is to be left as is for now, with the new wording used at the next update
Wants the gain/rate survey to work on all frequency ranges (not just bands up to 1.2 MHz), with the output as an Excel-usable surface chart: frequency along the horizontal axis, signal level vertical, gain as the depth axis with the lowest gain at the front
The survey's purpose: mainly to find which spectrum features are caused by sample-rate effects, secondarily whether and when the gain stages introduce artefacts (clipping or distortion); the multiple output files did not work for them, wants a single summary output that is easy to view and review
DC offset removal and IQ balance correction should both default to ON, with no front-panel tick boxes, controlled from settings.csv; the RTL-SDR driver selection should also be set in settings.csv so alternate drivers can be tried
Wants the survey report biased towards identifying which signals are likely from the device under test: features constant over sample rate and tracking gain are "reliably sensed"; sample-rate artefacts (aliasing images, spurs) and gain artefacts (non-linearity) listed separately so they can be ignored; wants a maximum usable gain identified (their own visual estimate is that gains above roughly 20 dB add disproportionate distortion, to be confirmed or not by the analysis) and a best rate/gain spectrum setting
Wants the survey report to open with the two maps (sample-rate effects and gain effects) and to drop the artefact discussion; each rate/gain setting is to be scanned at least twice, with a third (and further) scan when two disagree, until two consecutive scans match, and settings needing more than two scans listed after the maps; suspects the 3.2 MS/s data differs from 3 MS/s because the SDR is not settling; default lowest gain in the survey should be 0 dB
Concerned that wide-range spectrum stitching may be wrong: with a 1.5 MS/s sample rate over 12-30 MHz and no input there is an obvious discontinuity from about 27.7 to 28.7 MHz that looks like a reflection of the DC response region, independent of averaging, with similar block-to-block discontinuities at other sample rates; this made them distrust the overall wide-range spectra
Comparison run in SDR# showed spurious signals around 26-28 MHz that move when the sample rate changes; SDR# offers only 3.2, 2.8, 2.56, 2.4, 2.048, 1.92, 1.8, 1.4, 1.024, 0.900001 and 0.25 MS/s; wants the program's sample-rate choices limited to 1.024, 1.8, 2.048, 2.4, 2.56 and 3.2 MS/s (250 kS/s omitted), avoiding 0.3-0.9 MS/s, and suspects 3.2 MS/s has USB issues
Wants a new single-frequency test mode: the centre frequency is selected and the SDR shows a single spectrum; the front panel needs a digital centre-frequency selector where individual digits can be selected and incremented or decremented, shown below the gain selection when the mode is selected; the mode button goes above the range selection buttons, same size and shape