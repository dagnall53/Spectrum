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



