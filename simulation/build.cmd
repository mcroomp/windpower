@echo off
setlocal enabledelayedexpansion

set IMAGE=rawes-sim
set SIM_DIR=%~dp0
if "%SIM_DIR:~-1%"=="\" set SIM_DIR=%SIM_DIR:~0,-1%

set DO_ARDUPILOT=0
set DO_RUN=0
set DO_BUILD=1

:parse_args
if "%~1"=="" goto done_args
if /i "%~1"=="ardupilot" (
    set DO_ARDUPILOT=1
    shift
    goto parse_args
)
if /i "%~1"=="run" (
    set DO_RUN=1
    shift
    goto parse_args
)
if /i "%~1"=="nobuild" (
    set DO_BUILD=0
    shift
    goto parse_args
)
echo [WARN]  Unknown argument: %~1
shift
goto parse_args
:done_args

where docker >nul 2>&1
if errorlevel 1 (
    echo [ERROR] Docker not found. Install Docker Desktop: https://www.docker.com/products/docker-desktop
    exit /b 1
)

if "%DO_BUILD%"=="1" (
    set BUILD_CMD=docker build "%SIM_DIR%" -t %IMAGE%

    if "%DO_ARDUPILOT%"=="1" (
        set BUILD_CMD=!BUILD_CMD! --build-arg INSTALL_ARDUPILOT=true
        echo [WARN]  ArduPilot SITL build enabled -- expect ~30 minutes.
    )

    echo [INFO]  Building image '%IMAGE%' from %SIM_DIR% ...
    echo [INFO]  Command: !BUILD_CMD!
    echo.

    call !BUILD_CMD!
    if errorlevel 1 (
        echo [ERROR] Docker build failed.
        exit /b 1
    )

    echo.
    echo [INFO]  Build complete: %IMAGE%
)

if "%DO_RUN%"=="1" (
    echo.
    echo [INFO]  Starting container (simulation\ mounted at /rawes/simulation) ...
    echo [INFO]  Press Ctrl+C or type 'exit' to stop.
    echo.

    docker run --rm -it -v "%SIM_DIR%:/rawes/simulation" %IMAGE%
    if errorlevel 1 exit /b 1
)

if "%DO_BUILD%"=="1" (
    if "%DO_RUN%"=="0" (
    echo.
    echo [INFO]  To start a shell in the container:
    echo         docker run --rm -it -v "%SIM_DIR%:/rawes/simulation" %IMAGE%
    echo.
    echo [INFO]  Or use this script:
    echo         simulation\build.cmd nobuild run
    echo.
    )
)

endlocal
