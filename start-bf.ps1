# One-click launcher for the Betaflight SITL + local web configurator.
#
# 1. Starts betaflight_SITL.exe (low-CPU mode) if it is not already running.
# 2. Starts the local web server on http://127.0.0.1:8080 (with the manual
#    connection preset to ws://127.0.0.1:6761) if it is not already running.
# 3. Opens the configurator in the default browser.

$ErrorActionPreference = "Stop"
$root = $PSScriptRoot
$exe = Join-Path $root "build-win-cmake\betaflight_SITL.exe"
$serverScript = Join-Path $root "bfweb-server.mjs"
$port = 8080

function Test-PortListening([int]$Port) {
    $conn = Get-NetTCPConnection -LocalPort $Port -State Listen -ErrorAction SilentlyContinue
    return ($null -ne $conn)
}

function Get-PortOwner([int]$Port) {
    $conn = Get-NetTCPConnection -LocalPort $Port -State Listen -ErrorAction SilentlyContinue |
        Select-Object -First 1
    if ($null -eq $conn) {
        return $null
    }
    try {
        return Get-Process -Id $conn.OwningProcess -ErrorAction Stop
    } catch {
        return $null
    }
}

# 1. SITL
$sitl = Get-Process -Name betaflight_SITL -ErrorAction SilentlyContinue
if (-not $sitl) {
    if (-not (Test-Path $exe)) {
        throw "betaflight_SITL.exe not found at $exe - build it first."
    }
    $err = Join-Path $root "build-win-cmake\sitl-launch.err.log"
    $out = Join-Path $root "build-win-cmake\sitl-launch.out.log"
    Start-Process -FilePath $exe -WorkingDirectory (Split-Path $exe) `
        -RedirectStandardOutput $out -RedirectStandardError $err -WindowStyle Hidden | Out-Null
    Write-Host "Started betaflight_SITL.exe"
    # Wait until the MSP/WebSocket ports are up (a few seconds max).
    for ($i = 0; $i -lt 20; $i++) {
        if ((Test-PortListening 5761) -and (Test-PortListening 6761)) {
            break
        }
        Start-Sleep -Milliseconds 500
    }
} else {
    Write-Host "betaflight_SITL.exe already running (PID $($sitl.Id))"
}

# 2. Web server on 127.0.0.1:8080
$owner = Get-PortOwner $port
if ($owner) {
    $cmdline = (Get-CimInstance Win32_Process -Filter "ProcessId=$($owner.Id)" -ErrorAction SilentlyContinue).CommandLine
    if ($cmdline -and $cmdline -like '*bfweb-server.mjs*') {
        Write-Host "Web server already running on port $port (PID $($owner.Id))"
    } else {
        Write-Host "Port $port is used by $($owner.ProcessName) (PID $($owner.Id)) - replacing it with our web server."
        Stop-Process -Id $owner.Id -Force
        Start-Sleep -Milliseconds 500
        $owner = $null
    }
}
if (-not $owner) {
    $webErr = Join-Path $root "bfweb-server.err.log"
    $webOut = Join-Path $root "bfweb-server.out.log"
    Start-Process -FilePath "node" -ArgumentList @($serverScript) `
        -WorkingDirectory $root -RedirectStandardOutput $webOut -RedirectStandardError $webErr `
        -WindowStyle Hidden | Out-Null
    Start-Sleep -Seconds 2
    if (-not (Test-PortListening $port)) {
        throw "Web server failed to start - see $webErr"
    }
    Write-Host "Started web server on http://127.0.0.1:$port"
}

# 3. Open the browser
Start-Process "http://127.0.0.1:$port/"
Write-Host "Opening http://127.0.0.1:$port/"
Write-Host "The Connect menu -> Manual entry is preset to ws://127.0.0.1:6761 (injected by bfweb-server.mjs, bf-configurator sources are untouched)."
