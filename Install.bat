@echo off
:: ============================================================
:: AUTO-ELEVATE TO ADMIN
:: ============================================================
:: Check if running as admin
>nul 2>&1 net session
if %errorlevel% neq 0 (
    echo Requesting administrator privileges...
    powershell -Command "Start-Process '%~f0' -Verb RunAs"
    exit /b
)

echo ============================================
echo   EMC SCANNER INSTALLER  (C:\Spectrum)
echo ============================================
echo.

:: ============================================================
:: CREATE TARGET FOLDER
:: ============================================================
if not exist C:\Spectrum (
    mkdir C:\Spectrum
    echo Created folder C:\Spectrum
) else (
    echo Folder C:\Spectrum already exists
)

echo.

:: ============================================================
:: COPY FILES (ONLY IF NOT ALREADY IN C:\Spectrum)
:: ============================================================
if /I not "%CD%"=="C:\Spectrum" (
    echo Copying EMC scanner Python files...
    copy /Y emc_gui.py C:\Spectrum\emc_gui.py
) else (
    echo Skipping copy: installer is running inside C:\Spectrum
)

if exist librtlsdr.dll copy /Y librtlsdr.dll C:\Spectrum\librtlsdr.dll
if exist libusb-1.0.dll copy /Y libusb-1.0.dll C:\Spectrum\libusb-1.0.dll
if exist pthreadVC2.dll copy /Y pthreadVC2.dll C:\Spectrum\pthreadVC2.dll

echo Files copied.
echo.

:: ============================================================
:: INSTALL PYTHON PACKAGES (USER SITE)
:: ============================================================
echo Installing required Python packages...
python -m pip install --user pyrtlsdr pyqt5 pyqtgraph numpy
echo.

:: ============================================================
:: DETECT USER SITE-PACKAGES
:: ============================================================
for /f "delims=" %%i in ('python -m site --user-site') do set USER_SITE=%%i

echo User site-packages detected:
echo     %USER_SITE%
echo.

:: ============================================================
:: DETECT SYSTEM SITE-PACKAGES (CORRECT DIRECTORY)
:: ============================================================
for /f "usebackq delims=" %%i in (
    `python -c "import site; print([p for p in site.getsitepackages() if p.endswith('site-packages')][0])"`
) do set SYSTEM_SITE=%%i

echo System site-packages detected:
echo     %SYSTEM_SITE%
echo.

:: ============================================================
:: CREATE SITECUSTOMIZE.PY
:: ============================================================
echo Creating sitecustomize.py...
echo import sys > "%SYSTEM_SITE%\sitecustomize.py"
echo sys.path.append(r"%USER_SITE%") >> "%SYSTEM_SITE%\sitecustomize.py"

echo sitecustomize.py created at:
echo     %SYSTEM_SITE%\sitecustomize.py
echo.

:: ============================================================
:: CREATE LAUNCHER
:: ============================================================
echo Creating launcher...
echo @echo off > C:\Spectrum\run_emc_scanner.bat
echo C:\Python314\python.exe C:\Spectrum\emc_gui.py >> C:\Spectrum\run_emc_scanner.bat
echo pause >> C:\Spectrum\run_emc_scanner.bat

echo Launcher created:
echo     C:\Spectrum\run_emc_scanner.bat
echo.

echo ============================================
echo   INSTALL COMPLETE
echo ============================================
echo To run the EMC scanner:
echo     C:\Spectrum\run_emc_scanner.bat
echo.
pause
