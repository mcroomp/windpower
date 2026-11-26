@echo off
setlocal

set "REPO=%~dp0"
set "REPO=%REPO:~0,-1%"
set "BUILD=0"
set "CONFIGURE_FIREWALL=0"
set "SETUP_DONE=0"
set "CONNECTION="
set "BAUD="
set "PORT="

:parse_args
if "%~1"=="" goto args_done
if /i "%~1"=="--build" (
    set "BUILD=1"
    shift
    goto parse_args
)
if /i "%~1"=="--configure-firewall" (
    set "CONFIGURE_FIREWALL=1"
    shift
    goto parse_args
)
if not defined CONNECTION (
    set "CONNECTION=%~1"
) else if not defined BAUD (
    set "BAUD=%~1"
) else if not defined PORT (
    set "PORT=%~1"
) else (
    echo ERROR: Unexpected argument: %~1
    exit /b 2
)
shift
goto parse_args

:args_done

if not defined CONNECTION set "CONNECTION=%LINKHUB_CONNECTION%"
if not defined BAUD set "BAUD=%LINKHUB_BAUD%"
if not defined BAUD set "BAUD=auto"
if not defined PORT set "PORT=%LINKHUB_PORT%"
if not defined PORT set "PORT=8999"
if not defined LINKHUB_HTTPS_PORT set "LINKHUB_HTTPS_PORT=9443"

if not defined CONNECTION (
    echo Usage: %~nx0 [--build] [--configure-firewall] ^<connection^> [baud] [port]
    echo.
    echo Hardware:
    echo   %~nx0 auto
    echo   %~nx0 COM7
    echo   %~nx0 COM7 57600 8999
    echo   %~nx0 --build auto
    echo   %~nx0 --configure-firewall auto
    echo   %~nx0 ble:DroneBridge
    echo.
    echo SITL:
    echo   %~nx0 tcp:127.0.0.1:5760
    echo.
    echo Baud defaults to auto-discovery. Set it explicitly only to restrict scanning.
    echo Connection and baud may also be set with LINKHUB_CONNECTION and LINKHUB_BAUD.
    exit /b 2
)

set "BAUD_ARG="
if /i not "%BAUD%"=="auto" set BAUD_ARG=--baud "%BAUD%"

if "%BUILD%"=="1" goto setup
if not exist "%REPO%\linkhub\target\release\linkhub.exe" goto setup
if not exist "%REPO%\linkhub-ui\dist\index.html" goto setup
goto verify_build

:setup
if "%BUILD%"=="1" (
    call "%REPO%\setup.cmd" --linkhub-only
) else (
    call "%REPO%\setup.cmd"
)
if errorlevel 1 (
    echo ERROR: Project setup failed. LinkHub was not started.
    exit /b 1
)
set "SETUP_DONE=1"

:verify_build
if not exist "%REPO%\linkhub\target\release\linkhub.exe" (
    echo ERROR: LinkHub executable not found. Run %~nx0 --build %CONNECTION%
    exit /b 1
)

if not exist "%REPO%\linkhub-ui\dist\index.html" (
    echo ERROR: Compiled LinkHub UI not found. Run %~nx0 --build %CONNECTION%
    exit /b 1
)

call :run_linkhub --help >nul
if not errorlevel 1 goto start
if "%SETUP_DONE%"=="1" (
    echo ERROR: Built LinkHub does not accept the launcher arguments. LinkHub was not started.
    exit /b 1
)
echo [INFO] Existing LinkHub does not accept the launcher arguments. Rebuilding through setup.cmd ...
goto setup

:start
if "%CONFIGURE_FIREWALL%"=="1" (
    call powershell -NoProfile -ExecutionPolicy Bypass -File "%REPO%\scripts\configure_linkhub_firewall.ps1" -ProgramPath "%REPO%\linkhub\target\release\linkhub.exe" -HttpPort "%PORT%" -HttpsPort "%LINKHUB_HTTPS_PORT%"
    if errorlevel 1 (
        echo ERROR: Firewall configuration failed or was cancelled. LinkHub was not started.
        exit /b 1
    )
)
echo [INFO] Starting LinkHub on http://0.0.0.0:%PORT%/ (all network interfaces)
echo [INFO] LAN access: http://%COMPUTERNAME%.local:%PORT%/ or https://%COMPUTERNAME%.local:%LINKHUB_HTTPS_PORT%/ (phone cameras)
echo [INFO] Connection: %CONNECTION%  Baud: %BAUD%
echo [INFO] Press Ctrl+C to stop.
call :run_linkhub

set "EXIT_CODE=%ERRORLEVEL%"
if not "%EXIT_CODE%"=="0" echo ERROR: LinkHub exited with code %EXIT_CODE%.
exit /b %EXIT_CODE%

:run_linkhub
call "%REPO%\linkhub\target\release\linkhub.exe" serve ^
    --connection "%CONNECTION%" ^
    %BAUD_ARG% ^
    --listen 0.0.0.0 ^
    --port "%PORT%" ^
    --https-port "%LINKHUB_HTTPS_PORT%" ^
    --data-dir "%REPO%\simulation\logs\linkhub" ^
    --static-dir "%REPO%\linkhub-ui\dist" ^
    --motor-name-prefix BLDC ^
    --no-cache %*
exit /b %ERRORLEVEL%
