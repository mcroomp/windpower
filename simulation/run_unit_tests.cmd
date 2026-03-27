@echo off
setlocal enabledelayedexpansion

set SCRIPT_DIR=%~dp0
if "%SCRIPT_DIR:~-1%"=="\" set SCRIPT_DIR=%SCRIPT_DIR:~0,-1%

set VENV_DIR=%SCRIPT_DIR%\tests\unit\.venv
set PYTHON_EXE=%VENV_DIR%\Scripts\python.exe

if not exist "%PYTHON_EXE%" (
    echo [INFO] Creating unit test venv at "%VENV_DIR%" ...
    where py >nul 2>&1
    if not errorlevel 1 (
        py -3 -m venv "%VENV_DIR%"
    ) else (
        python -m venv "%VENV_DIR%"
    )
    if errorlevel 1 (
        echo [ERROR] Failed to create simulation virtual environment.
        exit /b 1
    )
)

echo [INFO] Installing local test dependencies into simulation\tests\unit\.venv ...
"%PYTHON_EXE%" -m pip install --upgrade pip >nul
if errorlevel 1 (
    echo [ERROR] Failed to upgrade pip in simulation\tests\unit\.venv.
    exit /b 1
)

"%PYTHON_EXE%" -m pip install numpy pytest >nul
if errorlevel 1 (
    echo [ERROR] Failed to install test dependencies.
    exit /b 1
)

echo [INFO] Running simulation unit tests ...
"%PYTHON_EXE%" -m pytest "%SCRIPT_DIR%\tests\unit" %*
exit /b %errorlevel%