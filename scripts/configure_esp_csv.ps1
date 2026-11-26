param(
    [Parameter(Mandatory = $true)]
    [uri]$DeviceUrl,
    [ValidateRange(1, 65535)]
    [int]$HttpPort = 8999
)

$ErrorActionPreference = 'Stop'
$route = [Net.Sockets.Socket]::new(
    [Net.Sockets.AddressFamily]::InterNetwork,
    [Net.Sockets.SocketType]::Dgram,
    [Net.Sockets.ProtocolType]::Udp
)
try {
    $route.Connect($DeviceUrl.Host, $DeviceUrl.Port)
    $localAddress = $route.LocalEndPoint.Address.ToString()
}
finally {
    $route.Dispose()
}

$target = "http://${localAddress}:${HttpPort}/v1/csv"

function Get-CsvDestination {
    $page = Invoke-WebRequest -UseBasicParsing -Uri ([uri]::new($DeviceUrl, '/')) -TimeoutSec 15
    $match = [regex]::Match(
        $page.Content,
        '<input\b[^>]*\bid=[''"]CSVStorageURL[''"][^>]*\bvalue=[''"]([^''"]*)[''"]'
    )
    if (-not $match.Success) {
        throw 'ESP configuration page did not contain the CSV destination.'
    }
    return [Net.WebUtility]::HtmlDecode($match.Groups[1].Value)
}

if ((Get-CsvDestination) -ne $target) {
    $commandUrl = [uri]::new($DeviceUrl, '/create_new_csv').AbsoluteUri +
        '?url=' + [uri]::EscapeDataString($target)
    Invoke-WebRequest -UseBasicParsing -Uri $commandUrl -TimeoutSec 15 | Out-Null
}
else {
    Write-Host '[INFO] ESP CSV destination is already correct; preserving its current file and pending samples.'
}

if ((Get-CsvDestination) -ne $target) {
    throw "ESP CSV destination read-back did not match $target."
}

$modeUrl = [uri]::new($DeviceUrl, '/set_hx711').AbsoluteUri + '?mode=4'
$modeResponse = Invoke-WebRequest -UseBasicParsing -Uri $modeUrl -TimeoutSec 15
if ($modeResponse.StatusCode -ne 202 -or
        ($modeResponse.Content | ConvertFrom-Json).requested_mode -ne 4) {
    throw 'ESP did not accept four-input capture mode.'
}
$status = Invoke-RestMethod -Uri ([uri]::new($DeviceUrl, '/hx711')) -TimeoutSec 15
if ($status.requested_mode -ne 4) {
    throw 'ESP four-input capture mode read-back did not match.'
}

Write-Host "[INFO] Verified ESP CSV destination: $target"
Write-Host '[INFO] Verified ESP capture mode: 4 inputs (A1, A2, B1, B2).'
Write-Host '[INFO] Destination and capture mode remain configured after LinkHub stops. Motion and sample interval are unchanged.'
