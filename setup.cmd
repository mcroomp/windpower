@echo off
setlocal

set "REPO=%~dp0"
set "REPO=%REPO:~0,-1%"
set "VS_INSTALLER=%ProgramFiles(x86)%\Microsoft Visual Studio\Installer"
set "PATH=%LOCALAPPDATA%\Microsoft\WinGet\Links;%ProgramFiles%\WinGet\Links;%ProgramFiles%\nodejs;%USERPROFILE%\.cargo\bin;%VS_INSTALLER%;%PATH%"

call :ensure_tool uv astral-sh.uv
if errorlevel 1 exit /b 1
call :ensure_node
if errorlevel 1 exit /b 1
set "PYTHON_VERSION="
set /p PYTHON_VERSION=<"%REPO%\.python-version"
if not defined PYTHON_VERSION (
    echo ERROR: .python-version must specify the supported Python minor.
    exit /b 1
)
set "UV_PYTHON_INSTALL_DIR=%REPO%\.tools\python"
echo [INFO] Upgrading repository-local Python %PYTHON_VERSION% ...
call uv python install --upgrade --no-bin --no-registry "%PYTHON_VERSION%"
if errorlevel 1 (
    echo ERROR: uv could not upgrade Python %PYTHON_VERSION%.
    exit /b 1
)
set "PYTHON_EXE="
for /f "delims=" %%P in ('uv python find --managed-python --system --no-project "%PYTHON_VERSION%"') do set "PYTHON_EXE=%%P"
if not defined PYTHON_EXE (
    echo ERROR: The repository-local Python installation could not be located.
    exit /b 1
)
echo [INFO] Synchronizing the project environment ...
call uv sync --project "%REPO%" --python "%PYTHON_EXE%" --managed-python --locked --dev --extra simulation
if errorlevel 1 (
    echo ERROR: uv sync failed.
    exit /b 1
)

echo [INFO] Checking Python with Ruff ...
call uv run --no-sync --project "%REPO%" ruff check "%REPO%"
if errorlevel 1 (
    echo ERROR: Ruff checks failed.
    exit /b 1
)

echo [INFO] Checking Python with Pyright ...
call uv run --no-sync --project "%REPO%" python -m pyright --project "%REPO%\pyrightconfig.json" --pythonpath "%REPO%\.venv\Scripts\python.exe" --warnings
if errorlevel 1 (
    echo ERROR: Pyright checks failed.
    exit /b 1
)

call :ensure_msvc
if errorlevel 1 exit /b 1
call :ensure_tool cargo Rustlang.Rustup
if errorlevel 1 exit /b 1

pushd "%REPO%\linkhub-ui"
if errorlevel 1 (
    echo ERROR: LinkHub UI directory was not found.
    exit /b 1
)
echo [INFO] Installing LinkHub UI dependencies ...
call npm ci --no-audit --no-fund
if errorlevel 1 (
    popd
    echo ERROR: LinkHub UI dependency installation failed.
    exit /b 1
)
echo [INFO] Building LinkHub UI ...
call npm run build
if errorlevel 1 (
    popd
    echo ERROR: LinkHub UI build failed.
    exit /b 1
)
popd

echo [INFO] Building LinkHub ...
call cargo build --manifest-path "%REPO%\linkhub\Cargo.toml" --release
if errorlevel 1 (
    echo ERROR: LinkHub build failed.
    exit /b 1
)

call uv run --no-sync --project "%REPO%" python --version
if errorlevel 1 (
    echo ERROR: The managed Python environment could not be verified.
    exit /b 1
)
echo [INFO] Done.
endlocal
exit /b 0

:ensure_tool
where %~1 >nul 2>nul
if not errorlevel 1 exit /b 0
call :install_package %~2
if errorlevel 1 exit /b 1
where %~1 >nul 2>nul
if not errorlevel 1 exit /b 0
echo ERROR: %~1 is not available after installation. Open a new terminal
echo        and run setup.cmd again.
exit /b 1

:ensure_node
set "NODE_MAJOR="
set /p NODE_MAJOR=<"%REPO%\.node-version"
if not defined NODE_MAJOR (
    echo ERROR: .node-version must specify the supported Node.js major.
    exit /b 1
)
call powershell -NoProfile -ExecutionPolicy Bypass -File "%REPO%\scripts\setup_node.ps1" -RepoRoot "%REPO%" -NodeMajor "%NODE_MAJOR%"
if errorlevel 1 (
    echo ERROR: Repository-local Node.js setup failed.
    exit /b 1
)
set "PATH=%REPO%\.tools\node;%PATH%"
call "%REPO%\.tools\node\node" -e "if (process.versions.node.split('.')[0] !== process.argv[1]) process.exit(1)" "%NODE_MAJOR%"
if errorlevel 1 (
    echo ERROR: The repository-local Node.js version could not be verified.
    exit /b 1
)
if not exist "%REPO%\.tools\node\npm.cmd" (
    echo ERROR: npm is missing from the repository-local Node.js installation.
    exit /b 1
)
exit /b 0

:install_package
where winget >nul 2>nul
if errorlevel 1 (
    echo ERROR: WinGet is required to install %~1. Install or update App Installer
    echo        from Microsoft Store, then run setup.cmd again.
    exit /b 1
)
echo [INFO] Installing %~1 with WinGet ...
call winget install --exact --source winget --accept-package-agreements --accept-source-agreements --disable-interactivity --id %*
if not errorlevel 1 exit /b 0
echo ERROR: WinGet could not install %~1. If a restart is required,
echo        restart Windows and run setup.cmd again.
exit /b 1

:check_msvc
set "VC_INSTALL="
set "SDK_FOUND="
where vswhere >nul 2>nul
if errorlevel 1 exit /b 1
for /f "delims=" %%I in ('vswhere -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath') do set "VC_INSTALL=%%I"
if not defined VC_INSTALL exit /b 1
for /d %%D in ("%ProgramFiles(x86)%\Windows Kits\10\Lib\*") do (
    if exist "%%~fD\um\x64\kernel32.lib" if exist "%%~fD\ucrt\x64\ucrt.lib" set "SDK_FOUND=1"
)
if not defined SDK_FOUND exit /b 1
exit /b 0

:ensure_msvc
call :check_msvc
if not errorlevel 1 exit /b 0
set "BUILD_TOOLS="
where vswhere >nul 2>nul
if errorlevel 1 goto :install_msvc
for /f "delims=" %%I in ('vswhere -latest -products Microsoft.VisualStudio.Product.BuildTools -property installationPath') do set "BUILD_TOOLS=%%I"
if not defined BUILD_TOOLS goto :install_msvc
echo [INFO] Adding the C++ build workload to existing Visual Studio Build Tools ...
call "%VS_INSTALLER%\setup" modify --installPath "%BUILD_TOOLS%" --add Microsoft.VisualStudio.Workload.VCTools --includeRecommended --passive --norestart
if errorlevel 1 (
    echo ERROR: C++ workload installation failed. Restart Windows if required,
    echo        then run setup.cmd again.
    exit /b 1
)
goto :verify_msvc

:install_msvc
call :install_package Microsoft.VisualStudio.2022.BuildTools --override "--wait --passive --norestart --add Microsoft.VisualStudio.Workload.VCTools --includeRecommended"
if errorlevel 1 exit /b 1

:verify_msvc
call :check_msvc
if not errorlevel 1 exit /b 0
echo ERROR: Visual Studio C++ tools or the x64 Windows SDK are still missing.
echo        Restart Windows if required, then run setup.cmd again.
exit /b 1
