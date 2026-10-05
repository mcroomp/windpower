@echo off
setlocal

set "REPO=%~dp0"
set "REPO=%REPO:~0,-1%"
set "BUILD=0"
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
if not defined BAUD set "BAUD=115200"
if not defined PORT set "PORT=%LINKHUB_PORT%"
if not defined PORT set "PORT=8999"

if not defined CONNECTION (
    echo Usage: %~nx0 [--build] ^<connection^> [baud] [port]
    echo.
    echo Hardware:
    echo   %~nx0 COM7 115200 8999
    echo   %~nx0 --build COM7 115200 8999
    echo.
    echo SITL:
    echo   %~nx0 tcp:127.0.0.1:5760
    echo.
    echo The connection may also be set with LINKHUB_CONNECTION.
    exit /b 2
)

if "%BUILD%"=="1" (
    where npm >nul 2>nul
    if errorlevel 1 (
        echo ERROR: npm was not found on PATH.
        exit /b 1
    )

    where cargo >nul 2>nul
    if errorlevel 1 (
        echo ERROR: cargo was not found on PATH.
        exit /b 1
    )

    echo [INFO] Installing LinkHub UI dependencies...
    pushd "%REPO%\linkhub-ui"
    call npm install --no-audit --no-fund
    if errorlevel 1 (
        popd
        echo ERROR: LinkHub UI dependency installation failed.
        exit /b 1
    )

    echo [INFO] Building LinkHub UI...
    call npm run build
    if errorlevel 1 (
        popd
        echo ERROR: LinkHub UI build failed.
        exit /b 1
    )
    popd

    echo [INFO] Building LinkHub...
    cargo build --manifest-path "%REPO%\linkhub\Cargo.toml" --release --features bluetooth
    if errorlevel 1 (
        echo ERROR: LinkHub build failed.
        exit /b 1
    )
)

if not exist "%REPO%\linkhub\target\release\linkhub.exe" (
    echo ERROR: LinkHub executable not found. Run %~nx0 --build %CONNECTION% %BAUD% %PORT%
    exit /b 1
)

if not exist "%REPO%\linkhub-ui\dist\index.html" (
    echo ERROR: Compiled LinkHub UI not found. Run %~nx0 --build %CONNECTION% %BAUD% %PORT%
    exit /b 1
)

echo [INFO] Starting LinkHub on http://127.0.0.1:%PORT%/
echo [INFO] Connection: %CONNECTION%  Baud: %BAUD%
echo [INFO] Press Ctrl+C to stop.
"%REPO%\linkhub\target\release\linkhub.exe" serve ^
    --connection "%CONNECTION%" ^
    --baud "%BAUD%" ^
    --port "%PORT%" ^
    --data-dir "%REPO%\simulation\logs\linkhub" ^
    --static-dir "%REPO%\linkhub-ui\dist" ^
    --no-cache

set "EXIT_CODE=%ERRORLEVEL%"
if not "%EXIT_CODE%"=="0" echo ERROR: LinkHub exited with code %EXIT_CODE%.
exit /b %EXIT_CODE%
