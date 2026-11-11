param(
    [Parameter(Mandatory = $true)]
    [string]$RepoRoot,
    [Parameter(Mandatory = $true)]
    [ValidatePattern('^[1-9][0-9]*$')]
    [string]$NodeMajor
)

$ErrorActionPreference = 'Stop'
$ProgressPreference = 'SilentlyContinue'
# Python and CMD can pass PowerShell 7 module paths to Windows PowerShell.
$env:PSModulePath = (Join-Path $PSHOME 'Modules') + [IO.Path]::PathSeparator + $env:PSModulePath
$installDir = Join-Path $RepoRoot '.tools\node'
$node = Join-Path $installDir 'node.exe'
$npm = Join-Path $installDir 'npm.cmd'

$architecture = $env:PROCESSOR_ARCHITEW6432
if (-not $architecture) {
    $architecture = $env:PROCESSOR_ARCHITECTURE
}
$arch = switch ($architecture) {
    'AMD64' { 'x64' }
    'ARM64' { 'arm64' }
    default { throw "Unsupported Windows architecture: $architecture" }
}

$staging = Join-Path $RepoRoot ("tmp\node-setup-" + [guid]::NewGuid().ToString('N'))
try {
    Write-Host "[INFO] Checking for repository-local Node.js $NodeMajor.x updates ..."
    $releases = Invoke-RestMethod -Uri 'https://nodejs.org/dist/index.json' -TimeoutSec 30
    $release = $releases | Where-Object {
        $_.version -match "^v$NodeMajor\.\d+\.\d+$" -and $_.files -contains "win-$arch-zip"
    } | Sort-Object { [version]$_.version.Substring(1) } -Descending | Select-Object -First 1
    if (-not $release) {
        throw "No Node.js $NodeMajor.x release is available for Windows $arch."
    }
    if ((Test-Path $node) -and (Test-Path $npm)) {
        $version = & $node --version
        if ($LASTEXITCODE -eq 0 -and $version -ceq $release.version) {
            Write-Host "[INFO] Repository-local Node.js $version is up to date"
            exit 0
        }
    }

    Write-Host "[INFO] Installing repository-local Node.js $($release.version) ..."
    $name = "node-$($release.version)-win-$arch"
    $archiveName = "$name.zip"
    $baseUri = "https://nodejs.org/dist/$($release.version)"
    New-Item -ItemType Directory -Path $staging -Force | Out-Null
    $archive = Join-Path $staging $archiveName
    $checksums = Join-Path $staging 'SHASUMS256.txt'
    Invoke-WebRequest -UseBasicParsing -Uri "$baseUri/SHASUMS256.txt" -OutFile $checksums -TimeoutSec 30
    Invoke-WebRequest -UseBasicParsing -Uri "$baseUri/$archiveName" -OutFile $archive -TimeoutSec 180
    $expectedHash = $null
    foreach ($line in Get-Content $checksums) {
        if ($line -match "^([0-9a-fA-F]{64})\s+$([regex]::Escape($archiveName))$") {
            $expectedHash = $Matches[1]
            break
        }
    }
    if (-not $expectedHash -or (Get-FileHash $archive -Algorithm SHA256).Hash -ne $expectedHash) {
        throw "SHA256 verification failed for $archiveName."
    }

    Expand-Archive -LiteralPath $archive -DestinationPath $staging
    $extracted = Join-Path $staging $name
    $version = & (Join-Path $extracted 'node.exe') --version
    if ($LASTEXITCODE -ne 0 -or $version -cne $release.version -or
        -not (Test-Path (Join-Path $extracted 'npm.cmd'))) {
        throw 'The downloaded Node.js installation could not be verified.'
    }
    New-Item -ItemType Directory -Path (Split-Path $installDir) -Force | Out-Null
    if (Test-Path $installDir) {
        Remove-Item -LiteralPath $installDir -Recurse -Force
    }
    Move-Item -LiteralPath $extracted -Destination $installDir
    Write-Host "[INFO] Installed repository-local Node.js $version"
} catch {
    Write-Error $_
    exit 1
} finally {
    if (Test-Path $staging) {
        Remove-Item -LiteralPath $staging -Recurse -Force
    }
}
