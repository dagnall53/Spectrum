@echo off
REM ============================================================
REM  Build rf_scan_portable.py into a standalone Windows program
REM  Run this on the PC that already runs rf_scan.py successfully.
REM
REM  Put in the SAME folder as this .bat:
REM     rf_scan_portable.py
REM     pyrtlsdr_local\        (your local RtlSdr wrapper package)
REM     drivers\               (copy of C:\Spectrum\drivers - default, ms64, ...)
REM  If "drivers" is missing, it is copied from C:\Spectrum\drivers.
REM ============================================================
cd /d "%~dp0"

if not exist drivers (
    if exist C:\Spectrum\drivers (
        echo Copying C:\Spectrum\drivers to .\drivers ...
        xcopy /E /I /Y C:\Spectrum\drivers drivers
    ) else (
        echo ERROR: no drivers folder found. Copy C:\Spectrum\drivers here.
        pause
        exit /b 1
    )
)

if not exist pyrtlsdr_local (
    if exist C:\Spectrum\pyrtlsdr_local (
        xcopy /E /I /Y C:\Spectrum\pyrtlsdr_local pyrtlsdr_local
    ) else (
        echo ERROR: pyrtlsdr_local folder not found next to this script.
        pause
        exit /b 1
    )
)

python -m pip install --upgrade pyinstaller
if errorlevel 1 goto fail

python -m PyInstaller --noconfirm --clean --windowed --name EMC_Sweep ^
    --paths . ^
    --hidden-import pyrtlsdr_local.rtlsdr ^
    --collect-submodules pyqtgraph ^
    --add-data "drivers;drivers" ^
    rf_scan_portable.py
if errorlevel 1 goto fail

echo.
echo Done. Copy the whole folder  dist\EMC_Sweep  to the other PC
echo and run EMC_Sweep.exe from inside it.
pause
exit /b 0

:fail
echo Build failed - see messages above.
pause
exit /b 1
