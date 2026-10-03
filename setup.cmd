@echo off
setlocal
set "PIP_INDEX_URL=https://packagefeedproxy.microsoft.io/pypi/simple/"

set "REPO=%~dp0"
set "REPO=%REPO:~0,-1%"
set "VENV=%REPO%\.venv"
set "PYTHON=%VENV%\Scripts\python.exe"
set "REQS=%REPO%\simulation\requirements.txt"
set "STAMP=%VENV%\Scripts\.requirements_hash"
set "PYPROJECT=%REPO%\pyproject.toml"
set "PKG_STAMP=%VENV%\Scripts\.editable_install_hash"

:: Create venv if missing
if exist "%PYTHON%" goto :check_reqs
if exist "%VENV%" (
    echo [WARN] %VENV% exists but has no python.exe -- recreating
    rmdir /s /q "%VENV%"
)
echo [INFO] Creating venv at %VENV% ...
py -3 -m venv "%VENV%"
if errorlevel 1 (
    echo ERROR: Failed to create venv. Is Python 3 on PATH?
    exit /b 1
)
"%PYTHON%" -m pip install --upgrade pip --quiet

:check_reqs
if not exist "%PYTHON%" (
    echo ERROR: venv python not found at %PYTHON%
    exit /b 1
)
if not exist "%REQS%" (
    echo [WARN] requirements.txt not found -- skipping install
    goto :install_pkg
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
    goto :install_pkg
)

echo [INFO] Installing requirements ...
"%PYTHON%" -m pip install -r "%REQS%"
if errorlevel 1 (
    echo ERROR: pip install -r failed.
    exit /b 1
)
echo %DIGEST%>"%STAMP%"

:install_pkg
:: Install the editable package only when its distribution is missing.
:: Re-running pip install -e can invoke the build backend and appear to hang,
:: while an existing editable install remains valid as source files change.
if not exist "%PYPROJECT%" (
    echo [WARN] pyproject.toml not found -- skipping editable install
    goto :done
)
set "PKG_PRESENT="
"%PYTHON%" -c "import importlib.metadata as m; m.version('rawes')" >nul 2>nul
if not errorlevel 1 set "PKG_PRESENT=1"
if defined PKG_PRESENT (
    echo [INFO] rawes editable package already installed -- skipping pip install -e
    goto :done
)
for /f "skip=1 tokens=1" %%H in ('certutil -hashfile "%PYPROJECT%" SHA256 2^>nul') do (
    set "PKG_DIGEST=%%H"
    goto :got_pkg_hash
)
:got_pkg_hash

set "PKG_CACHED="
if exist "%PKG_STAMP%" set /p PKG_CACHED=<"%PKG_STAMP%"

if "%PKG_DIGEST%"=="%PKG_CACHED%" (
    echo [INFO] pyproject.toml unchanged -- skipping pip install -e
    goto :done
)

echo [INFO] Installing rawes package (editable) ...
"%PYTHON%" -m pip install -e "%REPO%" --no-deps --quiet
if errorlevel 1 (
    echo ERROR: pip install -e failed.
    exit /b 1
)
echo %PKG_DIGEST%>"%PKG_STAMP%"

:done
set "LINKHUB=%REPO%\linkhub\target\release\linkhub.exe"
if not exist "%LINKHUB%" (
    echo [INFO] Building LinkHub with Bluetooth support ...
    cargo build --manifest-path "%REPO%\linkhub\Cargo.toml" --release --features bluetooth
    if errorlevel 1 (
        echo ERROR: LinkHub build failed.
        exit /b 1
    )
)
set "LINKHUB_CLIENT=%REPO%\linkhub_client"
"%PYTHON%" -c "import linkhub_client" >nul 2>nul
if errorlevel 1 (
    echo [INFO] Installing linkhub-client package ^(editable^) ...
    "%PYTHON%" -m pip install -e "%LINKHUB_CLIENT%" --no-deps --quiet
    if errorlevel 1 (
        echo ERROR: linkhub-client editable install failed.
        exit /b 1
    )
)
echo [INFO] Done.
"%PYTHON%" --version
endlocal
