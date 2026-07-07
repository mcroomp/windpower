@echo off
setlocal

set "REPO=%~dp0"
set "REPO=%REPO:~0,-1%"
set "VENV=%REPO%\.venv"
set "PYTHON=%VENV%\Scripts\python.exe"
set "REQS=%REPO%\simulation\requirements.txt"
set "STAMP=%VENV%\Scripts\.requirements_hash"

:: Create venv if missing
if exist "%PYTHON%" goto :check_reqs
if exist "%VENV%" (
    echo [WARN] %VENV% exists but has no python.exe -- recreating
    rmdir /s /q "%VENV%"
)
echo [INFO] Creating venv at %VENV% ...
py -3 -m venv "%VENV%"
"%PYTHON%" -m pip install --upgrade pip --quiet

:check_reqs
if not exist "%REQS%" (
    echo [WARN] requirements.txt not found -- skipping install
    goto :done
)

:: Hash-gated install: only reinstall when requirements.txt changes.
:: certutil writes "SHA256 hash of <file>:" then the hash then "CertUtil: -hashfile command completed successfully."
:: We extract just the hash line (line 2).
for /f "skip=1 tokens=1" %%H in ('certutil -hashfile "%REQS%" SHA256 2^>nul') do (
    set "DIGEST=%%H"
    goto :got_hash
)
:got_hash

set "CACHED="
if exist "%STAMP%" set /p CACHED=<"%STAMP%"

if "%DIGEST%"=="%CACHED%" (
    echo [INFO] requirements.txt unchanged -- skipping pip install
    goto :done
)

echo [INFO] Installing requirements ...
"%PYTHON%" -m pip install -r "%REQS%"
echo %DIGEST%>"%STAMP%"

:done
echo [INFO] Done.
"%PYTHON%" --version
endlocal
