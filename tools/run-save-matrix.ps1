<#
Runs the save/consistency matrix for the LOCAL DLL and prints a summary.

Every case boots the FC fresh, records one deterministic stick scenario, performs
the requested configurator-like operation and records the same scenario again.
The motor outputs are compared sample by sample (see sitl_local_save_compare.c).

Usage:
    pwsh -File tools\run-save-matrix.ps1
    pwsh -File tools\run-save-matrix.ps1 -Scenarios roll-step,snap -Quick

Exit code 0 = every case behaved as expected.
#>

param(
    [string]$Exe = (Join-Path $PSScriptRoot '..\build-win-local\sitl_local_save_compare.exe'),
    [string]$OutDir = (Join-Path $PSScriptRoot '..\build-win-local\matrix'),
    [string[]]$Scenarios = @('roll-step', 'pitch-step', 'yaw-step', 'snap', 'throttle', 'combined'),
    [switch]$Quick
)

$ErrorActionPreference = 'Stop'
New-Item -ItemType Directory -Force -Path $OutDir | Out-Null

# op name -> what the two comparisons must show
#   keep   the two traces must be identical (a no-op save must change nothing)
#   change the first comparison must differ (a real edit must still apply) and
#          the second one must be identical again (reverting returns to baseline)
$saveOps = @('save-only', 'save-reboot', 'cfg-save', 'cfg-save-reboot', 'aux-save')
$editOps = @{
    'cfg-change'        = @('DIVERGES', 'identical')
    'cfg-change-filter' = @('DIVERGES', 'identical')
    'cfg-change-rate'   = @('DIVERGES', 'identical')
    'cfg-change-pid'    = @('DIVERGES', 'identical')
    'twice'             = @('identical', 'identical')
    'reinit'            = @('identical', 'identical')
    'repeat'            = @('identical', 'identical')
    'armed-save'        = @('identical', 'identical')
}

function Invoke-Case {
    param([string]$Op, [string]$Scenario)

    $log = Join-Path $OutDir "$Op-$Scenario.txt"
    $null = & cmd /c "`"$Exe`" $Op $Scenario 2> `"$log`""
    $text = if (Test-Path $log) { Get-Content $log -Raw } else { '' }
    $matches = [regex]::Matches($text, 'VERDICT \(([^)]+)\): (identical|DIVERGES)')
    $verdicts = @()
    foreach ($m in $matches) {
        $verdicts += [pscustomobject]@{ Label = $m.Groups[1].Value; Result = $m.Groups[2].Value }
    }
    return [pscustomobject]@{ Op = $Op; Scenario = $Scenario; Log = $log; Verdicts = $verdicts }
}

$results = @()

foreach ($op in $saveOps) {
    foreach ($scenario in $Scenarios) {
        $results += Invoke-Case -Op $op -Scenario $scenario
    }
}
foreach ($op in $editOps.Keys) {
    $results += Invoke-Case -Op $op -Scenario 'roll-step'
    if (-not $Quick) {
        $results += Invoke-Case -Op $op -Scenario 'combined'
    }
}

$pass = 0
$fail = 0
$rows = @()

foreach ($r in $results) {
    $expected = if ($saveOps -contains $r.Op) { @('identical', 'identical') }
                else { $editOps[$r.Op] }
    $ok = $true
    $summary = @()
    for ($i = 0; $i -lt $expected.Count; $i++) {
        $got = if ($i -lt $r.Verdicts.Count) { $r.Verdicts[$i].Result } else { 'MISSING' }
        if ($got -ne $expected[$i]) { $ok = $false }
        $summary += $got
    }
    if ($ok) { $pass++ } else { $fail++ }
    $rows += [pscustomobject]@{
        Result   = if ($ok) { 'PASS' } else { 'FAIL' }
        Op       = $r.Op
        Scenario = $r.Scenario
        Traces   = ($summary -join ' / ')
        Expected = ($expected -join ' / ')
    }
}

$rows | Format-Table -AutoSize | Out-String -Width 200 | Write-Host
Write-Host "cases: $($rows.Count)  pass: $pass  fail: $fail  logs: $OutDir"

if ($fail -gt 0) {
    Write-Host "`nFailing cases:"
    $rows | Where-Object { $_.Result -eq 'FAIL' } | ForEach-Object { Write-Host "  $($_.Op) / $($_.Scenario)" }
    exit 1
}
exit 0
