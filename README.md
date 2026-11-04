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