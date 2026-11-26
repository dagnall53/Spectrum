@echo off
setlocal

REM === Spektrum installation directory ===
REM Change this if your Spektrum folder is different
set SPEKTRUM_DIR=C:\Users\dagna\Downloads\Spektrum

REM === Your Python rtl-sdr local folder ===
set PYRTL=C:\SDR\pyrtlsdr_local

echo Copying RTL-SDR Blog V4 DLLs from Spektrum to %PYRTL%
echo.

REM === Ensure destination exists ===
if not exist "%PYRTL%" (
    echo Creating %PYRTL%
    mkdir "%PYRTL%"
)

REM === List of DLLs Spektrum provides ===
set DLL1=librtlsdr.dll
set DLL2=rtlsdr.dll

REM === Backup existing DLLs ===
for %%F in (%DLL1% %DLL2%) do (
    if exist "%PYRTL%\%%F" (
        echo Backing up %%F to %%F.old
        ren "%PYRTL%\%%F" "%%F.old"
    )
)

REM === Copy Spektrum DLLs ===
echo Copying new DLLs...
copy /Y "%SPEKTRUM_DIR%\%DLL1%" "%PYRTL%\%DLL1%"
copy /Y "%SPEKTRUM_DIR%\%DLL2%" "%PYRTL%\%DLL2%"

echo.
echo Done. Your Python environment now uses the same HF-capable V4 DLLs as Spektrum.
echo.

pause
