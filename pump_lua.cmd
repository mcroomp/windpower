@echo off
setlocal

REM Run the Lua pumping-cycle simtest (test_lua_pumping_unified) and then open the
REM 3D visualization of its telemetry.
REM
REM Usage:
REM   pump_lua.cmd                 run with the default aero (or existing RAWES_AERO)
REM   pump_lua.cmd vpm             run with the VPM (free-wake) aero model
REM   pump_lua.cmd oye --novis     run with Oye aero, skip the visualization
REM
REM Aero models: quasi_static (default) | pitt_peters | oye | vpm
REM The aero model is passed to the test via the RAWES_AERO environment variable.

set "ROOT=%~dp0"
set "PY=%ROOT%.venv\Scripts\python.exe"
if not exist "%PY%" set "PY=%ROOT%simulation\.venv\Scripts\python.exe"

if not exist "%PY%" (
    echo Python venv not found. Tried:
    echo   %ROOT%.venv\Scripts\python.exe
    echo   %ROOT%simulation\.venv\Scripts\python.exe
    exit /b 1
)

REM Parse args: --novis is a flag; any other token selects the aero model.
set "NOVIS="
:argloop
if "%~1"=="" goto argdone
if /I "%~1"=="--novis" (set "NOVIS=1") else (set "RAWES_AERO=%~1")
shift
goto argloop
:argdone

set "CSV=%ROOT%simulation\logs\test_lua_pumping_unified\telemetry.csv"

pushd "%ROOT%" >nul

if defined RAWES_AERO (
    echo Aero model: %RAWES_AERO%  ^(RAWES_AERO^)
) else (
    echo Aero model: quasi_static  ^(default; set RAWES_AERO or pass an arg to change^)
)

REM Test timeout [s].  The pytest.mark.timeout marker overrides the --timeout CLI,
REM so the test reads RAWES_PUMP_TIMEOUT (default 600 in-test).  The slow VPM
REM free-wake model needs a much longer budget, so auto-bump it unless the caller
REM already set one.
if not defined RAWES_PUMP_TIMEOUT (
    if /I "%RAWES_AERO%"=="vpm" set "RAWES_PUMP_TIMEOUT=7200"
)
if defined RAWES_PUMP_TIMEOUT echo Test timeout: %RAWES_PUMP_TIMEOUT% s  ^(RAWES_PUMP_TIMEOUT^)

echo Running Lua pumping simtest (test_lua_pumping_unified) ...
"%PY%" simulation\run_tests.py simulation\tests\simtests -k test_lua_pumping_unified -m simtest -s
set "RC=%ERRORLEVEL%"

if not defined NOVIS (
    if exist "%CSV%" (
        echo Launching visualization: %CSV%
        start "RAWES pumping (Lua)" cmd /k ""%PY%" simulation\viz3d\visualize_3d.py "%CSV%""
    ) else (
        echo Telemetry not found: %CSV%
    )
)

popd >nul
exit /b %RC%
