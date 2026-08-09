$root = $PSScriptRoot
$cfg = Join-Path $root 'bf-configurator'
$p = Start-Process -FilePath 'node.exe' -ArgumentList @('node_modules/vite/bin/vite.js','preview','--host','127.0.0.1','--port','8080') -WorkingDirectory $cfg -RedirectStandardOutput (Join-Path $cfg 'preview.out.log') -RedirectStandardError (Join-Path $cfg 'preview.err.log') -WindowStyle Hidden -PassThru
Start-Sleep -Seconds 4
Write-Output ('preview PID=' + $p.Id)
$r = Invoke-WebRequest -Uri 'http://127.0.0.1:8080/' -UseBasicParsing -TimeoutSec 15
Write-Output ('HTTP ' + $r.StatusCode)
Write-Output $r.Content.Substring(0, [Math]::Min(600, $r.Content.Length))
