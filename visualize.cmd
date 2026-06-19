@echo off
setlocal

set "ROOT=%~dp0"
set "PY=%ROOT%.venv\Scripts\python.exe"

if "%~1"=="" (
    set "CSV=simulation\logs\test_ic_steady_flight\telemetry.csv"
) else (
    set "CSV=%~1"
)

if not exist "%PY%" (
    echo Python venv not found: %PY%
    exit /b 1
)

pushd "%ROOT%" >nul
start "RAWES visualization" cmd /k ""%PY%" simulation\viz3d\visualize_3d.py "%CSV%""
popd >nul
