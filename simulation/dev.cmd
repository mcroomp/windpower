@echo off
::
:: dev.cmd — manage the persistent rawes-dev container
::
:: Usage:
::   dev.cmd start            start the container (idempotent)
::   dev.cmd stop             stop and remove the container
::   dev.cmd shell            interactive bash shell
::   dev.cmd exec <cmd...>    run a command in the container
::   dev.cmd test-stack [...]  run stack integration tests (extra args forwarded)
::   dev.cmd test-unit  [...]  run unit tests (extra args forwarded)
::
:: The container is started once and reused for all subsequent commands.
:: The simulation\ directory is mounted at /rawes/simulation.
::
setlocal enabledelayedexpansion

:: Prevent Git Bash / MSYS2 from translating Unix-style paths (e.g. /rawes/...)
:: to Windows paths when docker commands are invoked from a bash shell.
set MSYS_NO_PATHCONV=1
set MSYS2_ARG_CONV_EXCL=*

set IMAGE=rawes-sim
set CONTAINER=rawes-dev
set SIM_DIR=%~dp0
if "%SIM_DIR:~-1%"=="\" set SIM_DIR=%SIM_DIR:~0,-1%

if "%~1"=="" (
    echo Usage: dev.cmd start ^| stop ^| shell ^| exec ^<cmd...^> ^| test-stack ^| test-unit
    exit /b 0
)

set CMD=%~1
shift

:: ── start ────────────────────────────────────────────────────────────────────
if /i "%CMD%"=="start" goto do_start
if /i "%CMD%"=="stop"  goto do_stop
if /i "%CMD%"=="shell" goto do_shell
if /i "%CMD%"=="exec"  goto do_exec
if /i "%CMD%"=="test-stack" goto do_test_stack
if /i "%CMD%"=="test-unit"  goto do_test_unit

echo [ERROR] Unknown command: %CMD%
exit /b 1

:: ─────────────────────────────────────────────────────────────────────────────
:do_start
call :ensure_running
exit /b %errorlevel%

:do_stop
echo [INFO] Stopping and removing container '%CONTAINER%' ...
docker rm -f %CONTAINER% >nul 2>&1
echo [INFO] Done.
exit /b 0

:do_shell
call :ensure_running || exit /b 1
docker exec -it %CONTAINER% bash
exit /b %errorlevel%

:do_exec
call :ensure_running || exit /b 1
set ARGS=%*
docker exec %CONTAINER% bash -c "%ARGS%"
exit /b %errorlevel%

:do_test_stack
call :ensure_running || exit /b 1
set EXTRA=%*
if "%EXTRA%"=="" (
    docker exec %CONTAINER% bash /rawes/simulation/test_stack.sh
) else (
    docker exec %CONTAINER% bash /rawes/simulation/test_stack.sh %EXTRA%
)
exit /b %errorlevel%

:do_test_unit
call :ensure_running || exit /b 1
set EXTRA=%*
if "%EXTRA%"=="" (
    docker exec %CONTAINER% bash -c "cd /rawes && .venv/bin/python -m pytest simulation/tests/unit"
) else (
    docker exec %CONTAINER% bash -c "cd /rawes && .venv/bin/python -m pytest simulation/tests/unit %EXTRA%"
)
exit /b %errorlevel%

:: ─────────────────────────────────────────────────────────────────────────────
:ensure_running
:: Check if the container is already running
docker inspect --format "{{.State.Running}}" %CONTAINER% 2>nul | findstr /c:"true" >nul
if not errorlevel 1 exit /b 0

:: Container exists but is not running (or doesn't exist) — remove and recreate
docker rm -f %CONTAINER% >nul 2>&1

echo [INFO] Starting container '%CONTAINER%' (simulation\ mounted at /rawes/simulation) ...
docker run -d --name %CONTAINER% -v "%SIM_DIR%:/rawes/simulation" %IMAGE% sleep infinity >nul
if errorlevel 1 (
    echo [ERROR] Failed to start container.
    exit /b 1
)
echo [INFO] Container '%CONTAINER%' is ready.
exit /b 0
