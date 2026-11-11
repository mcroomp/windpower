from __future__ import annotations

import os
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys
import textwrap
import tomllib

import pytest


REPO = Path(__file__).resolve().parents[2]
WINDOWS_ONLY = pytest.mark.skipif(sys.platform != "win32", reason="Windows bootstrap")


def write_cmd(path: Path, source: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(textwrap.dedent(source).lstrip(), encoding="ascii", newline="\r\n")


@pytest.fixture
def bootstrap(tmp_path: Path) -> tuple[Path, Path, Path, dict[str, str]]:
    project = tmp_path / "project with spaces"
    project.mkdir()
    for name in ("setup.cmd", "setup.sh", "run_linkhub_ui.cmd", ".node-version", ".python-version"):
        shutil.copy2(REPO / name, project / name)
    (project / "scripts").mkdir()
    shutil.copy2(REPO / "scripts" / "setup_node.ps1", project / "scripts" / "setup_node.ps1")
    (project / "linkhub-ui").mkdir()
    (project / "linkhub" / "target" / "release").mkdir(parents=True)
    bin_dir = tmp_path / "bin"
    bin_dir.mkdir()
    log = tmp_path / "commands.log"
    env = os.environ.copy()
    env.update({
        "PATH": str(bin_dir) + os.pathsep + str(Path(env["SYSTEMROOT"]) / "System32"),
        "LOCALAPPDATA": str(tmp_path / "local"),
        "PROGRAMFILES": str(tmp_path / "program files"),
        # Native CMD restores ProgramFiles from ProgramW6432.
        "PROGRAMW6432": str(tmp_path / "program files"),
        "PROGRAMFILES(X86)": str(tmp_path / "program files x86"),
        "USERPROFILE": str(tmp_path / "user"),
        "MOCK_LOG": str(log),
        "MOCK_BIN": str(bin_dir),
        "MOCK_PROJECT": str(project),
        "MOCK_CPP_READY": str(tmp_path / "cpp-ready"),
        "MOCK_CPP_ROOT": str(tmp_path / "visual studio"),
        "MOCK_PYTHON": str(project / ".tools" / "python" / "cpython-3.14" / "python.exe"),
        "UV_NO_SYNC": "1",
    })
    for name in ("LINKHUB_CONNECTION", "LINKHUB_BAUD", "LINKHUB_PORT"):
        env.pop(name, None)
    make_cpp = bin_dir / "make-cpp.cmd"
    write_cmd(make_cpp, r"""
        @echo off
        type nul > "%MOCK_CPP_READY%"
        mkdir "%ProgramFiles(x86)%\Windows Kits\10\Lib\test\um\x64" 2>nul
        mkdir "%ProgramFiles(x86)%\Windows Kits\10\Lib\test\ucrt\x64" 2>nul
        type nul > "%ProgramFiles(x86)%\Windows Kits\10\Lib\test\um\x64\kernel32.lib"
        type nul > "%ProgramFiles(x86)%\Windows Kits\10\Lib\test\ucrt\x64\ucrt.lib"
        exit /b 0
    """)
    write_cmd(bin_dir / "uv.template", r"""
        @echo off
        echo uv %*>>"%MOCK_LOG%"
        if "%~1 %~2"=="python install" echo python-install-dir %UV_PYTHON_INSTALL_DIR%>>"%MOCK_LOG%"
        if "%MOCK_FAIL%"=="ruff" if "%~5"=="ruff" exit /b 19
        if "%MOCK_FAIL%"=="%~1" exit /b 13
        if "%MOCK_FAIL%"=="python-find" if "%~2"=="find" exit /b 13
        if "%~2"=="find" echo %MOCK_PYTHON%
        exit /b 0
    """)
    write_cmd(bin_dir / "npm.template", r"""
        @echo off
        echo npm %*>>"%MOCK_LOG%"
        if "%MOCK_FAIL%"=="npm-%~1" exit /b 14
        exit /b 0
    """)
    write_cmd(bin_dir / "node.template", r"""
        @echo off
        if "%MOCK_FAIL%"=="node-version" exit /b 1
        exit /b 0
    """)
    write_cmd(bin_dir / "powershell.cmd", r"""
        @echo off
        echo powershell %*>>"%MOCK_LOG%"
        if "%MOCK_FAIL%"=="node-setup" exit /b 1
        mkdir "%MOCK_PROJECT%\.tools\node" 2>nul
        copy /y "%MOCK_BIN%\node.template" "%MOCK_PROJECT%\.tools\node\node.cmd" >nul
        if "%MOCK_FAIL%"=="node-npm" exit /b 0
        copy /y "%MOCK_BIN%\npm.template" "%MOCK_PROJECT%\.tools\node\npm.cmd" >nul
        exit /b 0
    """)
    write_cmd(bin_dir / "cargo.template", r"""
        @echo off
        echo cargo %*>>"%MOCK_LOG%"
        if "%MOCK_FAIL%"=="cargo" exit /b 15
        if not defined MOCK_BUILD_ARTIFACTS exit /b 0
        copy /y "%MOCK_SERVER_EXE%" "%MOCK_PROJECT%\linkhub\target\release\linkhub.exe" >nul
        mkdir "%MOCK_PROJECT%\linkhub-ui\dist" 2>nul
        echo built>"%MOCK_PROJECT%\linkhub-ui\dist\index.html"
        exit /b 0
    """)
    write_cmd(bin_dir / "vswhere.template", r"""
        @echo off
        for %%A in (%*) do if "%%~A"=="Microsoft.VisualStudio.Component.VC.Tools.x86.x64" goto cpp_tools
        if defined MOCK_EXISTING_VS echo %MOCK_CPP_ROOT%
        exit /b 0
        :cpp_tools
        if exist "%MOCK_CPP_READY%" echo %MOCK_CPP_ROOT%
        exit /b 0
    """)
    write_cmd(bin_dir / "winget.cmd", r"""
        @echo off
        echo winget %*>>"%MOCK_LOG%"
        if "%MOCK_FAIL%"=="winget" exit /b 16
        if defined MOCK_INSTALL_NOOP exit /b 0
        :find_id
        if "%~1"=="--id" goto install
        if "%~1"=="" exit /b 17
        shift
        goto find_id
        :install
        if "%~2"=="astral-sh.uv" goto uv
        if "%~2"=="Rustlang.Rustup" goto rust
        if "%~2"=="Microsoft.VisualStudio.2022.BuildTools" goto cpp
        exit /b 18
        :uv
        mkdir "%LOCALAPPDATA%\Microsoft\WinGet\Links" 2>nul
        copy /y "%MOCK_BIN%\uv.template" "%LOCALAPPDATA%\Microsoft\WinGet\Links\uv.cmd" >nul
        exit /b 0
        :rust
        mkdir "%USERPROFILE%\.cargo\bin" 2>nul
        copy /y "%MOCK_BIN%\cargo.template" "%USERPROFILE%\.cargo\bin\cargo.cmd" >nul
        exit /b 0
        :cpp
        copy /y "%MOCK_BIN%\vswhere.template" "%MOCK_BIN%\vswhere.cmd" >nul
        call "%MOCK_BIN%\make-cpp.cmd"
        exit /b %errorlevel%
    """)
    return project, bin_dir, log, env


def installed_tools(
    bootstrap: tuple[Path, Path, Path, dict[str, str]], *, sdk: bool = True,
) -> None:
    _, bin_dir, _, env = bootstrap
    for name in ("uv", "node", "npm", "cargo", "vswhere"):
        shutil.copy2(bin_dir / f"{name}.template", bin_dir / f"{name}.cmd")
    Path(env["MOCK_CPP_READY"]).touch()
    if sdk:
        for directory, filename in (("um", "kernel32.lib"), ("ucrt", "ucrt.lib")):
            path = Path(env["PROGRAMFILES(X86)"]) / "Windows Kits" / "10" / "Lib" / "test" / directory / "x64" / filename
            path.parent.mkdir(parents=True, exist_ok=True)
            path.touch()


def run_script(
    bootstrap: tuple[Path, Path, Path, dict[str, str]],
    script: str = "setup.cmd",
    *args: str,
):
    project, _, _, env = bootstrap
    command = subprocess.list2cmdline([str(project / script), *args])
    return subprocess.run(
        f'"{env["COMSPEC"]}" /d /s /c "call {command}"',
        env=env,
        cwd=project.parent,
        capture_output=True,
        text=True,
        input="echo MOCK_SERVER %cmdcmdline%\nexit /b 7\n",
        timeout=20,
    )


def commands(bootstrap: tuple[Path, Path, Path, dict[str, str]]) -> list[str]:
    log = bootstrap[2]
    return log.read_text().splitlines() if log.exists() else []


@WINDOWS_ONLY
def test_setup_bootstraps_all_missing_tools_without_python(
    bootstrap: tuple[Path, Path, Path, dict[str, str]],
):
    result = run_script(bootstrap)
    assert result.returncode == 0, result.stdout + result.stderr
    calls = commands(bootstrap)
    installs = [line for line in calls if line.startswith("winget install ")]
    assert len(installs) == 3
    for package in ("astral-sh.uv", "Rustlang.Rustup", "Microsoft.VisualStudio.2022.BuildTools"):
        assert any(f"--id {package}" in line for line in installs)
    assert any("--add Microsoft.VisualStudio.Workload.VCTools --includeRecommended" in line for line in installs)
    assert not any("OpenJS.NodeJS" in line for line in calls)
    assert any('setup_node.ps1" -RepoRoot ' in line and '-NodeMajor "24"' in line for line in calls)
    assert calls.index('uv python install --upgrade --no-bin --no-registry "3.14"') < next(i for i, line in enumerate(calls) if line.startswith("uv sync "))
    assert any("--managed-python --locked --dev --extra simulation" in line for line in calls)
    assert f"python-install-dir {bootstrap[0] / '.tools' / 'python'}" in calls
    assert "npm ci --no-audit --no-fund" in calls
    assert "npm run build" in calls
    assert any(line.startswith("cargo build ") and line.endswith("--release") for line in calls)
    assert "[INFO] Done." in result.stdout


def test_node_version_policy_is_shared_by_npm_projects():
    major = (REPO / ".node-version").read_text().strip()
    assert major == "24"
    for name in ("linkhub-ui", "presentations", "am32config"):
        project = REPO / name
        package = json.loads((project / "package.json").read_text())
        lock = json.loads((project / "package-lock.json").read_text())
        assert package["engines"]["node"] == f"{major}.x"
        assert lock["packages"][""]["engines"] == package["engines"]
        npm_config = (project / ".npmrc").read_text().splitlines()
        assert "engine-strict=true" in npm_config
        assert "registry=https://packagefeedproxy.microsoft.io/npm/" in npm_config
        assert all(
            record["resolved"].startswith("https://packagefeedproxy.microsoft.io/npm/")
            for record in lock["packages"].values()
            if "resolved" in record
        )


@WINDOWS_ONLY
def test_setup_reuses_tools_without_winget_and_handles_spaces(
    bootstrap: tuple[Path, Path, Path, dict[str, str]],
):
    installed_tools(bootstrap)
    project, bin_dir, _, _ = bootstrap
    (bin_dir / "winget.cmd").unlink()
    result = run_script(bootstrap)
    assert result.returncode == 0, result.stdout + result.stderr
    calls = commands(bootstrap)
    assert not any(line.startswith("winget ") for line in calls)
    assert f'uv sync --project "{project}" --python "{bootstrap[3]["MOCK_PYTHON"]}" --managed-python --locked --dev --extra simulation' in calls
    assert f'uv run --no-sync --project "{project}" ruff check "{project}"' in calls
    assert f'uv run --no-sync --project "{project}" python --version' in calls


@WINDOWS_ONLY
def test_setup_uses_local_node_and_npm_despite_incompatible_shared_versions(
    bootstrap: tuple[Path, Path, Path, dict[str, str]],
):
    installed_tools(bootstrap)
    project, bin_dir, _, env = bootstrap
    (bin_dir / "winget.cmd").unlink()
    write_cmd(bin_dir / "node.cmd", r"""
        @echo off
        exit /b 1
    """)
    write_cmd(bin_dir / "npm.cmd", r"""
        @echo off
        echo ERROR: wrong npm selected
        exit /b 1
    """)
    shared = Path(env["PROGRAMFILES"]) / "nodejs"
    write_cmd(shared / "node.cmd", r"""
        @echo off
        exit /b 1
    """)
    shutil.copy2(bin_dir / "npm.cmd", shared / "npm.cmd")
    write_cmd(bin_dir / "npm.template", r"""
        @echo off
        call node -e "if (process.versions.node.split('.')[0] !== process.argv[1]) process.exit(1)" 24
        if errorlevel 1 exit /b 1
        echo local-npm %*>>"%MOCK_LOG%"
        exit /b 0
    """)
    node_before = (shared / "node.cmd").read_bytes()
    original_path = env["PATH"]
    result = run_script(bootstrap)
    assert result.returncode == 0, result.stdout + result.stderr
    assert "local-npm ci --no-audit --no-fund" in commands(bootstrap)
    assert "local-npm run build" in commands(bootstrap)
    assert not any(line.startswith("winget ") for line in commands(bootstrap))
    assert (shared / "node.cmd").read_bytes() == node_before
    assert env["PATH"] == original_path
    assert (project / ".tools" / "node" / "npm.cmd").is_file()


@WINDOWS_ONLY
@pytest.mark.parametrize("scenario", ["install", "upgrade", "current", "checksum", "no-release", "network"])
def test_local_node_installer(
    tmp_path: Path, scenario: str,
):
    local_node = REPO / ".tools" / "node" / "node.exe"
    node = str(local_node) if local_node.is_file() else shutil.which("node.exe")
    if node is None:
        pytest.skip("Node.js is required for installer integration tests")
    version = subprocess.check_output([node, "--version"], text=True).strip()
    major = version.removeprefix("v").split(".")[0]
    project = tmp_path / "node project with spaces"
    project.mkdir()
    install = project / ".tools" / "node"
    install.mkdir(parents=True)
    if scenario == "current":
        os.link(node, install / "node.exe")
        (install / "npm.cmd").write_text("@exit /b 0")
    if scenario in ("upgrade", "checksum", "no-release", "network"):
        (install / "old-version").write_text("previous installation")
    files = ["win-x64-zip", "win-arm64-zip"]
    releases = [
        {"version": f"v{major}.0.0", "files": files},
        {"version": f"v{int(major) + 1}.0.0", "files": files},
        {"version": version, "files": files},
    ]
    index = project / "index.json"
    index.write_text(json.dumps([] if scenario == "no-release" else releases))
    payload = project / "download.zip"
    payload.write_bytes(b"test archive")
    checksum = hashlib.sha256(payload.read_bytes()).hexdigest()
    env = os.environ.copy()
    env.update({
        "TEST_NODE": node,
        "TEST_NODE_VERSION": version,
        "TEST_NODE_INDEX": str(index),
        "TEST_NODE_PAYLOAD": str(payload),
        "TEST_NODE_HASH": "0" * 64 if scenario == "checksum" else checksum,
        "TEST_NODE_SCENARIO": scenario,
    })
    wrapper = project / "mock-downloads.ps1"
    wrapper.write_text(textwrap.dedent(r"""
        function Invoke-RestMethod {
            param($Uri, $TimeoutSec)
            if ($env:TEST_NODE_SCENARIO -eq 'network') { throw 'Test network failure' }
            Get-Content $env:TEST_NODE_INDEX -Raw | ConvertFrom-Json
        }
        function Invoke-WebRequest {
            param($Uri, $OutFile, $TimeoutSec, [switch]$UseBasicParsing)
            if ($env:TEST_NODE_SCENARIO -eq 'current') { throw 'Unexpected download' }
            if ($Uri -notlike "*/$env:TEST_NODE_VERSION/*") { throw 'Wrong release selected' }
            if ($Uri.EndsWith('/SHASUMS256.txt')) {
                "$env:TEST_NODE_HASH  node-$($env:TEST_NODE_VERSION)-win-x64.zip" | Set-Content $OutFile
                "$env:TEST_NODE_HASH  node-$($env:TEST_NODE_VERSION)-win-arm64.zip" | Add-Content $OutFile
            } else {
                Copy-Item $env:TEST_NODE_PAYLOAD $OutFile
            }
        }
        function Expand-Archive {
            param($LiteralPath, $DestinationPath)
            $name = [IO.Path]::GetFileNameWithoutExtension($LiteralPath)
            $extracted = Join-Path $DestinationPath $name
            New-Item -ItemType Directory -Path $extracted | Out-Null
            New-Item -ItemType HardLink -Path (Join-Path $extracted 'node.exe') -Target $env:TEST_NODE | Out-Null
            '@exit /b 0' | Set-Content (Join-Path $extracted 'npm.cmd')
        }
        & $args[0] -RepoRoot $args[1] -NodeMajor $args[2]
    """), encoding="ascii")
    result = subprocess.run(
        ["powershell", "-NoProfile", "-ExecutionPolicy", "Bypass", "-File", str(wrapper),
         str(REPO / "scripts" / "setup_node.ps1"), str(project), major],
        capture_output=True, text=True, env=env, timeout=20,
    )
    if scenario in ("install", "upgrade", "current"):
        assert result.returncode == 0, result.stdout + result.stderr
        assert subprocess.check_output([str(install / "node.exe"), "--version"], text=True).strip() == version
        assert (install / "npm.cmd").is_file()
        assert not (install / "old-version").exists()
    else:
        assert result.returncode != 0
        expected = {
            "checksum": "SHA256 verification failed",
            "no-release": "No Node.js",
            "network": "Test network failure",
        }[scenario]
        assert expected in result.stderr
        assert not (install / "node.exe").exists()
        assert (install / "old-version").read_text() == "previous installation"
    assert not list((project / "tmp").glob("node-setup-*"))


@WINDOWS_ONLY
@pytest.mark.parametrize("stage", ["winget", "node-setup", "node-version", "node-npm", "python", "python-find", "sync", "ruff", "npm-ci", "npm-run", "cargo", "run"])
def test_setup_stops_on_install_or_build_failure(
    bootstrap: tuple[Path, Path, Path, dict[str, str]], stage: str,
):
    if stage != "winget":
        installed_tools(bootstrap)
    bootstrap[3]["MOCK_FAIL"] = stage
    result = run_script(bootstrap)
    assert result.returncode != 0
    assert "ERROR:" in result.stdout
    assert "[INFO] Done." not in result.stdout
    if stage in ("winget", "node-setup", "node-version", "node-npm", "python", "python-find", "sync", "ruff"):
        assert not any(line.startswith("npm ") or line.startswith("cargo ") for line in commands(bootstrap))


@WINDOWS_ONLY
def test_setup_reports_missing_winget(
    bootstrap: tuple[Path, Path, Path, dict[str, str]],
):
    (bootstrap[1] / "winget.cmd").unlink()
    result = run_script(bootstrap)
    assert result.returncode != 0
    assert "App Installer" in result.stdout
    assert not commands(bootstrap)


@WINDOWS_ONLY
def test_setup_rejects_install_success_without_an_available_tool(
    bootstrap: tuple[Path, Path, Path, dict[str, str]],
):
    bootstrap[3]["MOCK_INSTALL_NOOP"] = "1"
    result = run_script(bootstrap)
    assert result.returncode != 0
    assert "uv is not available after installation" in result.stdout
    assert not any(line.startswith("uv ") for line in commands(bootstrap))


@WINDOWS_ONLY
def test_setup_repairs_an_existing_build_tools_installation_missing_sdk(
    bootstrap: tuple[Path, Path, Path, dict[str, str]],
):
    installed_tools(bootstrap, sdk=False)
    _, bin_dir, _, env = bootstrap
    env["MOCK_EXISTING_VS"] = "1"
    installer = Path(env["PROGRAMFILES(X86)"]) / "Microsoft Visual Studio" / "Installer" / "setup.cmd"
    write_cmd(installer, r"""
        @echo off
        echo visual-studio %*>>"%MOCK_LOG%"
        call "%MOCK_BIN%\make-cpp.cmd"
        exit /b %errorlevel%
    """)
    (bin_dir / "winget.cmd").unlink()
    result = run_script(bootstrap)
    assert result.returncode == 0, result.stdout + result.stderr
    assert any(line.startswith("visual-studio modify ") for line in commands(bootstrap))


def server_artifacts(bootstrap: tuple[Path, Path, Path, dict[str, str]]) -> None:
    project, _, _, env = bootstrap
    env["MOCK_SERVER_EXE"] = env["COMSPEC"]
    shutil.copy2(env["COMSPEC"], project / "linkhub" / "target" / "release" / "linkhub.exe")
    (project / "linkhub-ui" / "dist").mkdir(exist_ok=True)
    (project / "linkhub-ui" / "dist" / "index.html").write_text("built")


@WINDOWS_ONLY
@pytest.mark.parametrize("missing", [None, "binary", "ui"])
def test_launcher_delegates_setup_and_preserves_connection_arguments(
    bootstrap: tuple[Path, Path, Path, dict[str, str]], missing: str | None,
):
    installed_tools(bootstrap)
    server_artifacts(bootstrap)
    project, _, _, env = bootstrap
    env["MOCK_BUILD_ARTIFACTS"] = "1"
    if missing == "binary":
        (project / "linkhub" / "target" / "release" / "linkhub.exe").unlink()
    elif missing == "ui":
        (project / "linkhub-ui" / "dist" / "index.html").unlink()
    args = ["tcp:127.0.0.1:5760", "auto", "9123"]
    if missing is None:
        args.insert(0, "--build")
    result = run_script(bootstrap, "run_linkhub_ui.cmd", *args)
    assert result.returncode == 7
    assert 'uv python install --upgrade --no-bin --no-registry "3.14"' in commands(bootstrap)
    assert "http://127.0.0.1:9123/" in result.stdout
    assert "Connection: tcp:127.0.0.1:5760  Baud: auto" in result.stdout
    assert "MOCK_SERVER" in result.stdout
    assert 'serve     --connection "tcp:127.0.0.1:5760"' in result.stdout
    assert '--port "9123"' in result.stdout
    assert "--baud" not in result.stdout


@WINDOWS_ONLY
def test_prebuilt_launcher_does_not_require_python_uv_or_winget(
    bootstrap: tuple[Path, Path, Path, dict[str, str]],
):
    server_artifacts(bootstrap)
    result = run_script(bootstrap, "run_linkhub_ui.cmd", "auto")
    assert result.returncode == 7
    assert "Starting LinkHub" in result.stdout
    assert not commands(bootstrap)
    assert "MOCK_SERVER" in result.stdout


@WINDOWS_ONLY
def test_launcher_preserves_environment_defaults_and_explicit_baud(
    bootstrap: tuple[Path, Path, Path, dict[str, str]],
):
    server_artifacts(bootstrap)
    bootstrap[3].update({
        "LINKHUB_CONNECTION": "COM42",
        "LINKHUB_BAUD": "115200",
        "LINKHUB_PORT": "9124",
    })
    result = run_script(bootstrap, "run_linkhub_ui.cmd")
    assert result.returncode == 7
    assert '--connection "COM42"' in result.stdout
    assert '--baud "115200"' in result.stdout
    assert '--port "9124"' in result.stdout
    assert not commands(bootstrap)


@WINDOWS_ONLY
def test_launcher_does_not_start_after_setup_failure(
    bootstrap: tuple[Path, Path, Path, dict[str, str]],
):
    installed_tools(bootstrap)
    server_artifacts(bootstrap)
    bootstrap[3]["MOCK_FAIL"] = "sync"
    result = run_script(bootstrap, "run_linkhub_ui.cmd", "--build", "auto")
    assert result.returncode == 1
    assert "Project setup failed. LinkHub was not started." in result.stdout
    assert "Starting LinkHub" not in result.stdout


@WINDOWS_ONLY
def test_git_bash_delegates_to_the_windows_bootstrap(
    bootstrap: tuple[Path, Path, Path, dict[str, str]],
):
    bash = Path(os.environ["PROGRAMFILES"]) / "Git" / "bin" / "bash.exe"
    if not bash.is_file():
        pytest.skip("Git Bash is not installed")
    installed_tools(bootstrap)
    project, _, _, env = bootstrap
    env["MSYSTEM"] = "MINGW64"
    env["PATH"] = str(bash.parent.parent / "usr" / "bin") + os.pathsep + env["PATH"]
    result = subprocess.run(
        [str(bash), "--noprofile", "--norc", "-c", 'bash "$(cygpath -u "$1")"', "setup-test", str(project / "setup.sh")],
        env=env,
        capture_output=True,
        text=True,
        timeout=20,
    )
    assert result.returncode == 0, result.stdout + result.stderr
    assert 'uv python install --upgrade --no-bin --no-registry "3.14"' in commands(bootstrap)


def test_uv_sync_includes_the_editable_linkhub_client():
    project = tomllib.loads((REPO / "pyproject.toml").read_text())
    lock = tomllib.loads((REPO / "uv.lock").read_text())
    assert "linkhub-client" in project["project"]["dependencies"]
    assert project["tool"]["uv"]["sources"]["linkhub-client"] == {
        "path": "linkhub_client",
        "editable": True,
    }
    packages = {package["name"]: package for package in lock["package"]}
    assert packages["linkhub-client"]["source"] == {"editable": "linkhub_client"}
    assert {"name": "linkhub-client"} in packages["rawes"]["dependencies"]
