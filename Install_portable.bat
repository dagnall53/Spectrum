@echo off
setlocal EnableExtensions
REM ============================================================
REM  Install_portable.bat - install the standalone (exe) version
REM  on a PC that has no Python.
REM
REM  Run it from INSIDE the EMC_Sweep folder that build_exe.bat made
REM  (the folder that holds EMC_Sweep.exe), after copying that folder
REM  to this PC. It copies the program to C:\Spectrum\EMC_Sweep and
REM  puts a shortcut on the desktop. An existing settings.csv there
REM  (your presets and CAL factors) is kept.
REM
REM  You do not have to run this: EMC_Sweep.exe also runs straight
REM  from wherever the folder is, as long as the folder stays writable
REM  (the program saves settings.csv next to the exe when you use CAL).
REM ============================================================
cd /d "%~dp0"
>nul 2>&1 net session
if %errorlevel% neq 0 (
    echo Requesting administrator privileges...
    powershell -NoProfile -Command "Start-Process -FilePath '%~f0' -Verb RunAs"
    exit /b
)
cd /d "%~dp0"

set "APPNAME=EMC_Sweep"
set "TARGET=C:\Spectrum\%APPNAME%"

echo ============================================
echo   RF SCAN PORTABLE - install to %TARGET%
echo ============================================
echo.

if not exist "%APPNAME%.exe" (
    echo ERROR: %APPNAME%.exe is not in the folder this installer is in.
    echo Run Install_portable.bat from inside the %APPNAME% folder.
    goto fail
)

if not exist C:\Spectrum mkdir C:\Spectrum

REM ---- keep an existing settings.csv ----
set "KEEP="
if /I "%CD%"=="%TARGET%" goto skipkeep
if exist "%TARGET%\settings.csv" (
    copy /Y "%TARGET%\settings.csv" "%TEMP%\rf_scan_settings_keep.csv" >nul
    set "KEEP=1"
)

:skipkeep
if /I "%CD%"=="%TARGET%" (
    echo Already running from %TARGET% - no files to copy.
) else (
    echo Copying the program...
    xcopy /E /I /Y /Q "%~dp0." "%TARGET%" >nul
    if errorlevel 1 (
        echo ERROR: copying failed.
        goto fail
    )
)

if defined KEEP (
    copy /Y "%TEMP%\rf_scan_settings_keep.csv" "%TARGET%\settings.csv" >nul
    del "%TEMP%\rf_scan_settings_keep.csv" >nul 2>&1
    echo Kept your existing settings.csv
)

REM ---- desktop shortcut (all users) ----
powershell -NoProfile -ExecutionPolicy Bypass -Command "$d=[Environment]::GetFolderPath('CommonDesktopDirectory'); $s=(New-Object -ComObject WScript.Shell).CreateShortcut($d+'\RF Scan.lnk'); $s.TargetPath='%TARGET%\%APPNAME%.exe'; $s.WorkingDirectory='%TARGET%'; $s.Save()"

REM ---- let normal users change files here (CAL saves settings.csv) ----
icacls "C:\Spectrum" /grant *S-1-5-32-545:(OI)(CI)M /T /C /Q >nul

echo.
echo ============================================
echo   INSTALL COMPLETE
echo ============================================
echo  Start the program with the "RF Scan" icon on the desktop,
echo  or run  %TARGET%\%APPNAME%.exe
echo.
echo  The RTL-SDR dongle needs the WinUSB driver: run Zadig, choose the
echo  dongle (Bulk-In, Interface 0) and install WinUSB, as for SDR#.
echo  If the program reports a missing DLL, install the Microsoft
echo  Visual C++ Redistributable 2015-2022 (x64) from microsoft.com.
echo.
pause
exit /b 0

:fail
echo.
echo Install failed - see the messages above.
pause
exit /b 1
