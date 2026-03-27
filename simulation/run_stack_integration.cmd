@echo off
setlocal enabledelayedexpansion

set IMAGE=rawes-sim
set SCRIPT_DIR=%~dp0
if "%SCRIPT_DIR:~-1%"=="\" set SCRIPT_DIR=%SCRIPT_DIR:~0,-1%

where docker >nul 2>&1
if errorlevel 1 (
    echo [ERROR] Docker not found. Install Docker Desktop first.
    exit /b 1
)

docker image inspect %IMAGE% >nul 2>&1
if errorlevel 1 (
    echo [ERROR] Docker image '%IMAGE%' not found.
    echo [INFO] Build it first with:
    echo        simulation\build.cmd ardupilot
    exit /b 1
)

docker run --rm %IMAGE% python -m pytest --version >nul 2>&1
if errorlevel 1 (
    echo [ERROR] Docker image '%IMAGE%' is missing stack integration test dependencies.
    echo [INFO] Rebuild it first with:
    echo        simulation\build.cmd ardupilot
    exit /b 1
)

echo [INFO] Running stack integration tests in Docker ...
docker run --rm -v "%SCRIPT_DIR%:/rawes/simulation" %IMAGE% bash /rawes/simulation/test_stack.sh %*
exit /b %errorlevel%