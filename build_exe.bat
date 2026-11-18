@echo off
setlocal EnableExtensions
REM ============================================================
REM  build_exe.bat - build rf_scan_portable.py into a standalone,
REM  portable Windows program (PyInstaller, "one folder" build).
REM
REM  Run this on the PC that already runs rf_scan_portable.py.
REM  Put these in the SAME folder as this .bat:
REM     rf_scan_portable.py
REM     pyrtlsdr_local\   (the local RtlSdr wrapper package)
REM     drivers\          (the driver DLL folders: default, ms64, ...)
REM  Optional, copied into the finished folder if present:
REM     settings.csv, README.md, Install_portable.bat
REM  If "drivers" or "pyrtlsdr_local" are missing they are copied
REM  from C:\Spectrum.
REM
REM  Usage:   build_exe.bat            normal build (no console window)
REM           build_exe.bat console    build WITH a console window,
REM                                    for finding out why it will not start
REM
REM  Result:  dist\EMC_Sweep\EMC_Sweep.exe and dist\EMC_Sweep.zip
REM ============================================================
cd /d "%~dp0"

set "APPNAME=EMC_Sweep"
set "WINFLAG=--windowed"
if /I "%~1"=="console" set "WINFLAG=--console"

REM ---- find Python (the py launcher first, then python) ----
set "PYEXE="
where py >nul 2>&1
if not errorlevel 1 set "PYEXE=py -3"
if not defined PYEXE (
    where python >nul 2>&1
    if not errorlevel 1 set "PYEXE=python"
)
if not defined PYEXE (
    echo ERROR: Python was not found. Install Python 3 from python.org
    echo and tick "Add python.exe to PATH" in its installer.
    goto fail
)
echo Using Python: %PYEXE%

REM ---- check the files that must be next to this script ----
if not exist rf_scan_portable.py (
    echo ERROR: rf_scan_portable.py is not in this folder.
    goto fail
)

if not exist drivers (
    if exist C:\Spectrum\drivers (
        echo Copying C:\Spectrum\drivers to .\drivers ...
        xcopy /E /I /Y /Q C:\Spectrum\drivers drivers >nul
    ) else (
        echo ERROR: no drivers folder found. Copy C:\Spectrum\drivers here.
        goto fail
    )
)

if not exist pyrtlsdr_local (
    if exist C:\Spectrum\pyrtlsdr_local (
        echo Copying C:\Spectrum\pyrtlsdr_local to .\pyrtlsdr_local ...
        xcopy /E /I /Y /Q C:\Spectrum\pyrtlsdr_local pyrtlsdr_local >nul
    ) else (
        echo ERROR: the pyrtlsdr_local folder was not found next to this script.
        goto fail
    )
)

REM ---- build tools and the packages the program imports ----
%PYEXE% -m pip install --upgrade pyinstaller numpy PyQt5 pyqtgraph pyrtlsdr
if errorlevel 1 goto fail

REM ---- build ----
%PYEXE% -m PyInstaller --noconfirm --clean %WINFLAG% --name %APPNAME% ^
    --paths . ^
    --hidden-import pyrtlsdr_local.rtlsdr ^
    --collect-submodules pyqtgraph ^
    --collect-data pyqtgraph ^
    --add-data "drivers;drivers" ^
    rf_scan_portable.py
if errorlevel 1 goto fail

if not exist "dist\%APPNAME%\%APPNAME%.exe" (
    echo ERROR: the build finished but dist\%APPNAME%\%APPNAME%.exe is missing.
    goto fail
)

REM ---- put the extras next to the exe ----
REM  settings.csv is read from the exe's folder (presets, CAL factors, options).
if exist settings.csv copy /Y settings.csv "dist\%APPNAME%\settings.csv" >nul
if exist README.md copy /Y README.md "dist\%APPNAME%\README.md" >nul
if exist Install_portable.bat copy /Y Install_portable.bat "dist\%APPNAME%\Install_portable.bat" >nul

REM ---- one zip file, convenient for moving to another PC ----
powershell -NoProfile -ExecutionPolicy Bypass -Command "Compress-Archive -Path 'dist\%APPNAME%' -DestinationPath 'dist\%APPNAME%.zip' -Force"

echo.
echo ============================================================
echo  Build finished.
echo    Program folder: dist\%APPNAME%    (run %APPNAME%.exe from inside it)
echo    Zip file:       dist\%APPNAME%.zip
echo  Copy the whole folder, or unzip the zip, on the other PC.
echo  There, run Install_portable.bat to put it in C:\Spectrum with a
echo  desktop shortcut, or just run %APPNAME%.exe where it is.
echo  The dongle needs the WinUSB driver (Zadig) on that PC.
echo ============================================================
pause
exit /b 0

:fail
echo.
echo Build failed - see the messages above.
pause
exit /b 1
