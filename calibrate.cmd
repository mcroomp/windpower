@echo off
setlocal

set VENV=%~dp0simulation\.venv
set PYTHON=%VENV%\Scripts\python.exe
set PIP=%VENV%\Scripts\pip.exe
set REQ=%~dp0simulation\requirements.txt
set STAMP=%VENV%\.req_stamp

if not exist "%VENV%\Scripts\activate.bat" (
    echo Creating venv at %VENV% ...
    python -m venv "%VENV%"
    if errorlevel 1 (
        echo ERROR: Failed to create venv. Is Python 3 on PATH?
        exit /b 1
    )
)

if not exist "%PYTHON%" (
    echo ERROR: venv python not found at %PYTHON%
    exit /b 1
)

:: Only reinstall if requirements.txt changed since last install
for /f "delims=" %%H in ('certutil -hashfile "%REQ%" MD5 ^| findstr /v "hash"') do set REQ_HASH=%%H
set REQ_HASH=%REQ_HASH: =%

set STAMP_HASH=
if exist "%STAMP%" (
    set /p STAMP_HASH=<"%STAMP%"
)

if not "%REQ_HASH%"=="%STAMP_HASH%" (
    echo Installing/verifying requirements ...
    "%PIP%" install -q -r "%REQ%"
    if errorlevel 1 (
        echo ERROR: pip install failed.
        exit /b 1
    )
    echo %REQ_HASH%>"%STAMP%"
)

"%PYTHON%" "%~dp0simulation\scripts\calibrate.py" %*
