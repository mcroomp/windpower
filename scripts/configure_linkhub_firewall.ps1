[CmdletBinding(SupportsShouldProcess = $true)]
param(
    [Parameter(Mandatory = $true)]
    [string]$ProgramPath,
    [ValidateRange(1, 65535)]
    [int]$HttpPort = 8999,
    [ValidateRange(1, 65535)]
    [int]$HttpsPort = 9443
)

$ErrorActionPreference = 'Stop'
$env:PSModulePath = (Join-Path $PSHOME 'Modules') + [IO.Path]::PathSeparator + $env:PSModulePath
$program = (Resolve-Path -LiteralPath $ProgramPath).ProviderPath
if (-not (Test-Path -LiteralPath $program -PathType Leaf)) {
    throw "LinkHub executable not found: $program"
}

$principal = [Security.Principal.WindowsPrincipal]::new(
    [Security.Principal.WindowsIdentity]::GetCurrent()
)
if (-not $WhatIfPreference -and -not $principal.IsInRole(
    [Security.Principal.WindowsBuiltInRole]::Administrator
)) {
    Write-Host '[INFO] Requesting administrator approval to configure Private LAN firewall access ...'
    $arguments = '-NoProfile -ExecutionPolicy Bypass -File "{0}" -ProgramPath "{1}" -HttpPort {2} -HttpsPort {3}' -f `
        $PSCommandPath, $program, $HttpPort, $HttpsPort
    $process = Start-Process -FilePath "$PSHOME\powershell.exe" -Verb RunAs `
        -ArgumentList $arguments -Wait -PassThru
    if ($process.ExitCode -ne 0) {
        throw "Elevated firewall configuration failed with exit code $($process.ExitCode)."
    }
    exit 0
}

$ruleName = 'RAWES-LinkHub-Private-LAN'
$existing = Get-NetFirewallRule -PolicyStore PersistentStore |
    Where-Object { $_.Name -eq $ruleName }
$settings = @{
    Name = $ruleName
    DisplayName = 'RAWES LinkHub (Private LAN)'
    Description = 'Allow local-subnet access to LinkHub HTTP and HTTPS on Private networks only.'
    PolicyStore = 'PersistentStore'
    Enabled = 'True'
    Direction = 'Inbound'
    Action = 'Allow'
    Profile = 'Private'
    Program = $program
    Protocol = 'TCP'
    LocalPort = @($HttpPort, $HttpsPort) | Sort-Object -Unique
    RemoteAddress = 'LocalSubnet'
    EdgeTraversalPolicy = 'Block'
    WhatIf = [bool]$WhatIfPreference
}
if ($existing) {
    Set-NetFirewallRule @settings | Out-Null
} else {
    New-NetFirewallRule @settings | Out-Null
}
if (-not $WhatIfPreference) {
    Write-Host "[INFO] LinkHub firewall rule configured for Private LAN TCP ports $HttpPort and $HttpsPort."
    Write-Host '[INFO] Public networks remain unchanged. Explicit block rules or managed policy may still prevent access.'
}
