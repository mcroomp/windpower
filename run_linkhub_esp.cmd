@echo off
setlocal

if exist "%~dp0loadcell\launch_config.cmd" call "%~dp0loadcell\launch_config.cmd"
if not defined LOADCELL_DEVICE_URL (
    echo ERROR: Set LOADCELL_DEVICE_URL or create loadcell\launch_config.cmd from its example.
    exit /b 2
)
if not defined LINKHUB_CONNECTION (
    echo ERROR: Set LINKHUB_CONNECTION or create loadcell\launch_config.cmd from its example.
    exit /b 2
)

set "LINKHUB_CSV_MAP=%~dp0linkhub\csv-map.json"

call powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0scripts\configure_esp_csv.ps1" -DeviceUrl "%LOADCELL_DEVICE_URL%" -HttpPort 8999
if errorlevel 1 (
    echo ERROR: ESP CSV configuration failed. LinkHub was not started.
    exit /b 1
)

call "%~dp0run_linkhub_ui.cmd" %* "%LINKHUB_CONNECTION%" auto 8999
exit /b %ERRORLEVEL%
