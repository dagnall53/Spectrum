@echo off
setlocal EnableExtensions
REM ============================================================
REM  Install.bat - install RF Scan Portable (the Python version)
REM  into C:\Spectrum on this PC.
REM
REM  Put these in the SAME folder as this .bat:
REM     rf_scan_portable.py
REM     pyrtlsdr_local\   (the local RtlSdr wrapper package)
REM     drivers\          (driver DLL folders: default, ms64, ...)
REM  Optional: settings.csv, README.md
REM  (loose librtlsdr.dll / libusb-1.0.dll / pthreadVC2.dll next to
REM   this file are put in drivers\default if there is no drivers folder)
REM
REM  It needs Python 3 already installed (python.org, with
REM  "Add python.exe to PATH" ticked).
REM ============================================================

REM ---- run from the folder this .bat is in, then ask for admin rights ----
cd /d "%~dp0"
>nul 2>&1 net session
if %errorlevel% neq 0 (
    echo Requesting administrator privileges...
    powershell -NoProfile -Command "Start-Process -FilePath '%~f0' -Verb RunAs"
    exit /b
)
REM (after the restart as administrator the working folder is System32, so go back)
cd /d "%~dp0"

echo ============================================
echo   RF SCAN PORTABLE INSTALLER  (C:\Spectrum)
echo ============================================
echo.

REM ---- find Python ----
set "PYEXE="
where py >nul 2>&1
if not errorlevel 1 set "PYEXE=py -3"
if not defined PYEXE (
    where python >nul 2>&1
    if not errorlevel 1 set "PYEXE=python"
)
if not defined PYEXE (
    echo ERROR: Python was not found. Install Python 3 from python.org
    echo and tick "Add python.exe to PATH", then run this installer again.
    goto fail
)
echo Using Python: %PYEXE%
echo.

if not exist rf_scan_portable.py (
    echo ERROR: rf_scan_portable.py is not in the folder this installer is in.
    goto fail
)

REM ---- create the target folder ----
if not exist C:\Spectrum (
    mkdir C:\Spectrum
    echo Created folder C:\Spectrum
) else (
    echo Folder C:\Spectrum already exists
)
echo.

REM ---- copy the program (not when the installer is already running from C:\Spectrum) ----
if /I "%CD%"=="C:\Spectrum" (
    echo Installer is running inside C:\Spectrum - no files to copy.
) else (
    echo Copying the program...
    copy /Y rf_scan_portable.py C:\Spectrum\rf_scan_portable.py >nul
    if exist README.md copy /Y README.md C:\Spectrum\README.md >nul
    if exist pyrtlsdr_local (
        xcopy /E /I /Y /Q pyrtlsdr_local C:\Spectrum\pyrtlsdr_local >nul
    ) else (
        if not exist C:\Spectrum\pyrtlsdr_local echo WARNING: pyrtlsdr_local folder not found - the program will not start without it.
    )
    if exist drivers (
        xcopy /E /I /Y /Q drivers C:\Spectrum\drivers >nul
    ) else (
        if exist librtlsdr.dll (
            if not exist C:\Spectrum\drivers\default mkdir C:\Spectrum\drivers\default
            copy /Y librtlsdr.dll C:\Spectrum\drivers\default\librtlsdr.dll >nul
            if exist libusb-1.0.dll copy /Y libusb-1.0.dll C:\Spectrum\drivers\default\libusb-1.0.dll >nul
            if exist pthreadVC2.dll copy /Y pthreadVC2.dll C:\Spectrum\drivers\default\pthreadVC2.dll >nul
            echo Loose driver DLLs copied to C:\Spectrum\drivers\default
        )
    )
    REM settings.csv holds your presets and CAL factors: never overwrite an existing one
    if exist settings.csv (
        if exist C:\Spectrum\settings.csv (
            echo Kept the existing C:\Spectrum\settings.csv
        ) else (
            copy /Y settings.csv C:\Spectrum\settings.csv >nul
            echo Copied settings.csv
        )
    )
)
echo.

REM ---- Python packages (for all users) ----
echo Installing the Python packages the program needs...
%PYEXE% -m pip install numpy PyQt5 pyqtgraph pyrtlsdr
if errorlevel 1 (
    echo ERROR: the package installation failed - see the messages above.
    goto fail
)
echo.

REM ---- launcher ----
echo Creating the launcher...
> "C:\Spectrum\run_rf_scan.bat" echo @echo off
>> "C:\Spectrum\run_rf_scan.bat" echo cd /d "%%~dp0"
>> "C:\Spectrum\run_rf_scan.bat" echo %PYEXE% rf_scan_portable.py
>> "C:\Spectrum\run_rf_scan.bat" echo if errorlevel 1 pause

REM ---- desktop shortcut (all users) ----
powershell -NoProfile -ExecutionPolicy Bypass -Command "$d=[Environment]::GetFolderPath('CommonDesktopDirectory'); $s=(New-Object -ComObject WScript.Shell).CreateShortcut($d+'\RF Scan.lnk'); $s.TargetPath='C:\Spectrum\run_rf_scan.bat'; $s.WorkingDirectory='C:\Spectrum'; $s.Save()"

REM ---- let normal users change files here (the program saves settings.csv when you use CAL) ----
icacls "C:\Spectrum" /grant *S-1-5-32-545:(OI)(CI)M /T /C /Q >nul

echo.
echo ============================================
echo   INSTALL COMPLETE
echo ============================================
echo  Start the program with the "RF Scan" icon on the desktop,
echo  or run  C:\Spectrum\run_rf_scan.bat
echo.
echo  The RTL-SDR dongle needs the WinUSB driver: run Zadig, choose the
echo  dongle (Bulk-In, Interface 0) and install WinUSB, as for SDR#.
echo.
pause
exit /b 0

:fail
echo.
echo Install failed - see the messages above.
pause
exit /b 1
