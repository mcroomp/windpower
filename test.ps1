param(
    [Parameter(ValueFromRemainingArguments = $true)]
    [string[]]$CliArgs
)

$ErrorActionPreference = "Stop"
$PSNativeCommandUseErrorActionPreference = $false

$RepoDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$SimDir = Join-Path $RepoDir "simulation"
$VenvPy = Join-Path $RepoDir ".venv\Scripts\python.exe"

$Image = "rawes-sim"
$Container = "rawes-dev"

function Write-Info([string]$msg) {
    $ts = Get-Date -Format "HH:mm:ss"
    Write-Host "$ts [INFO] $msg"
}

function Require-Venv {
    if (-not (Test-Path $VenvPy)) {
        throw "Python venv not found at $VenvPy. Run setup.cmd first."
    }
}

function Invoke-External([string]$exe, [string[]]$cmdArgs) {
    & $exe @cmdArgs
    $rc = $LASTEXITCODE
    if ($rc -ne 0) {
        throw "Command failed ($rc): $exe $($cmdArgs -join ' ')"
    }
}

function Docker-RunNoThrow([string[]]$dockerArgs) {
    & docker @dockerArgs 2>$null | Out-Null
    return $LASTEXITCODE
}

function Sync-Code([string]$name) {
    Write-Info "Syncing code to container $name..."
    & docker exec $name mkdir -p /rawes/simulation/logs | Out-Null
    # Simple, PowerShell-native sync (no WSL/Git-Bash tar pipeline required)
    & docker cp "$SimDir\." "$name`:/rawes/simulation"
    if ($LASTEXITCODE -ne 0) {
        throw "docker cp to $name failed"
    }

    $aeroDir = Join-Path $RepoDir "..\aero"
    if (Test-Path $aeroDir) {
        & docker exec $name mkdir -p /rawes/aero | Out-Null
        & docker cp "$aeroDir\." "$name`:/rawes/aero"
        if ($LASTEXITCODE -ne 0) {
            Write-Host "[WARN] Failed to sync ../aero; continuing."
        }
    }
    Write-Info "Code sync complete."
}

function Ensure-Running {
    $runningName = & docker ps --filter "name=^$Container$" --format "{{.Names}}"
    if ($LASTEXITCODE -eq 0 -and ($runningName -join "`n") -match "^$([Regex]::Escape($Container))$") {
        Sync-Code $Container
        return
    }

    $existing = & docker ps -a --filter "name=^$Container$" --format "{{.Names}}"
    if ($LASTEXITCODE -eq 0 -and ($existing -join "`n") -match "^$([Regex]::Escape($Container))$") {
        Docker-RunNoThrow @("rm", "-f", $Container) | Out-Null
    }
    Write-Info "Starting container '$Container'..."
    Invoke-External "docker" @("run", "-d", "--cap-add=SYS_PTRACE", "--name", $Container, $Image, "sleep", "infinity")
    Sync-Code $Container
}

function Parse-StackArgs([string[]]$args) {
    $nWorkers = 4
    $pass = New-Object System.Collections.Generic.List[string]

    for ($i = 0; $i -lt $args.Count; $i++) {
        $a = $args[$i]
        if ($a -eq "-n" -and ($i + 1) -lt $args.Count) {
            $i++
            $nWorkers = [int]$args[$i]
            continue
        }
        if ($a -match "^-n\d+$") {
            $nWorkers = [int]($a.Substring(2))
            continue
        }
        $pass.Add($a)
    }

    return @{ N = $nWorkers; Pass = $pass.ToArray() }
}

function Get-KExpr([string[]]$passArgs) {
    for ($i = 0; $i -lt $passArgs.Count; $i++) {
        if ($passArgs[$i] -eq "-k" -and ($i + 1) -lt $passArgs.Count) {
            return $passArgs[$i + 1]
        }
    }
    return ""
}

function Run-Stack([string[]]$stackArgs) {
    $parsed = Parse-StackArgs $stackArgs
    $passArgs = $parsed.Pass

    $allFiles = Get-ChildItem -Path (Join-Path $SimDir "tests\sitl") -Recurse -Filter "test_*.py" |
        Sort-Object FullName

    $kExpr = Get-KExpr $passArgs
    if ($kExpr -ne "") {
        $matched = New-Object System.Collections.Generic.List[System.IO.FileInfo]
        foreach ($f in $allFiles) {
            $base = [System.IO.Path]::GetFileNameWithoutExtension($f.Name)
            if ($base -like "*$kExpr*") {
                $matched.Add($f)
                continue
            }
            $txt = Get-Content -Path $f.FullName -Raw
            if ($txt -match "def\s+test_[A-Za-z0-9_]*$([Regex]::Escape($kExpr))[A-Za-z0-9_]*\s*\(" -or
                $txt -match "def\s+$([Regex]::Escape($kExpr))\s*\(") {
                $matched.Add($f)
            }
        }
        if ($matched.Count -gt 0) {
            $allFiles = $matched
            Write-Info "-k '$kExpr': pre-filtered to $($allFiles.Count) file(s)"
        }
    }

    $runId = [int][double]::Parse((Get-Date -UFormat %s))
    Write-Info "$($allFiles.Count) tests, sequential worker mode (run=$runId)"

    $anyFail = $false
    $logsRoot = Join-Path $SimDir "logs"
    New-Item -ItemType Directory -Path $logsRoot -Force | Out-Null

    for ($j = 0; $j -lt $allFiles.Count; $j++) {
        $f = $allFiles[$j]
        $label = [System.IO.Path]::GetFileNameWithoutExtension($f.Name)
        $ctr = "rawes-parallel-$runId-$j"

        Write-Info "[t$j] starting: $label"

        try {
            Invoke-External "docker" @("run", "-d", "--cap-add=SYS_PTRACE", "--name", $ctr, $Image, "sleep", "infinity")
            Sync-Code $ctr
            & docker exec $ctr bash -lc "rm -rf /rawes/simulation/logs && mkdir -p /rawes/simulation/logs" | Out-Null

            $rel = $f.FullName.Substring($SimDir.Length).TrimStart('\\').Replace('\\', '/')
            $inside = "/rawes/simulation/$rel"

            $dockerArgs = @(
                "exec",
                "-e", "RAWES_RUN_STACK_INTEGRATION=1",
                "-e", "RAWES_SIM_VEHICLE=/ardupilot/Tools/autotest/sim_vehicle.py",
                "-e", "PYTHONPATH=/rawes",
                $ctr,
                "/rawes/.venv/bin/python",
                "-m", "pytest",
                $inside,
                "-s", "-v"
            ) + $passArgs

            & docker @dockerArgs
            if ($LASTEXITCODE -ne 0) {
                $anyFail = $true
            }

            $labelDir = Join-Path $logsRoot $label
            New-Item -ItemType Directory -Path $labelDir -Force | Out-Null
            & docker cp "$ctr`:/rawes/simulation/logs/." "$logsRoot" 2>$null | Out-Null
        }
        finally {
            Docker-RunNoThrow @("rm", "-f", $ctr) | Out-Null
        }
    }

    Write-Info "[LOGS] simulation\\logs"
    if ($anyFail) {
        throw "One or more stack tests failed."
    }
}

function Show-Help {
    @"
test.ps1 -- PowerShell entry point for RAWES tests and Docker workflows.

Commands:
  unit [pytest args...]      Windows venv unit tests
  simtest [args...]          Windows venv simtests
  stack [-n N] [args...]     Docker SITL stack tests (PowerShell-native orchestration)
  hil [pytest args...]       HIL pytest
  sitl [args...]             Start SITL bench container and run sitl_bench.py
  start                      Start rawes-dev container + sync code
  stop                       Stop/remove rawes-dev container
  sync                       Sync simulation code into running rawes-dev
  shell                      Interactive bash in rawes-dev
  exec <cmd...>              Run command in rawes-dev
"@ | Write-Host
}

try {
    if (-not $CliArgs -or $CliArgs.Count -eq 0) {
        Show-Help
        exit 1
    }

    $cmd = $CliArgs[0]
    $rest = @()
    if ($CliArgs.Count -gt 1) {
        $rest = $CliArgs[1..($CliArgs.Count - 1)]
    }

    switch ($cmd) {
        "unit" {
            Require-Venv
            Invoke-External $VenvPy @("-m", "pytest", (Join-Path $SimDir "tests\unit"), "-m", "not simtest") + $rest
        }
        "simtest" {
            Require-Venv
            Invoke-External $VenvPy @((Join-Path $SimDir "run_tests.py"), (Join-Path $SimDir "tests\simtests"), "-m", "simtest") + $rest
        }
        "stack" {
            Run-Stack $rest
        }
        "hil" {
            Require-Venv
            Invoke-External $VenvPy @("-m", "pytest", (Join-Path $SimDir "tests\hil")) + $rest
        }
        "sitl" {
            $sitlCtr = "rawes-sitl"
            Write-Info "Starting SITL bench environment ..."
            Docker-RunNoThrow @("rm", "-f", $sitlCtr) | Out-Null
            Invoke-External "docker" @("run", "-d", "--name", $sitlCtr, "-p", "5760:5760", $Image, "sleep", "infinity")
            Sync-Code $sitlCtr
            try {
                Invoke-External "docker" @("exec", "-it", $sitlCtr, "/rawes/.venv/bin/python3", "/rawes/simulation/scripts/sitl_bench.py") + $rest
            }
            finally {
                Docker-RunNoThrow @("rm", "-f", $sitlCtr) | Out-Null
            }
        }
        "start" {
            Ensure-Running
        }
        "stop" {
            Write-Info "Stopping and removing container '$Container' ..."
            $existing = & docker ps -a --filter "name=^$Container$" --format "{{.Names}}"
            if ($LASTEXITCODE -eq 0 -and ($existing -join "`n") -match "^$([Regex]::Escape($Container))$") {
                Docker-RunNoThrow @("rm", "-f", $Container) | Out-Null
            }
            Write-Info "Done."
        }
        "sync" {
            Sync-Code $Container
        }
        "shell" {
            Ensure-Running
            & docker exec -it $Container bash
            exit $LASTEXITCODE
        }
        "exec" {
            Ensure-Running
            $cmdText = ($rest -join " ")
            Invoke-External "docker" @("exec", $Container, "bash", "-lc", $cmdText)
        }
        "-h" { Show-Help }
        "--help" { Show-Help }
        default {
            throw "Unknown command: $cmd"
        }
    }
}
catch {
    Write-Host "[ERROR] $($_.Exception.Message)" -ForegroundColor Red
    exit 1
}
