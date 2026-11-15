# The multi-GPU test on win5090, for the lead's GPU window: a CARLA 1.0 primary (-nullrhi) and one rendering
# secondary on the RTX 5090, compared with the single packaged server; plus the VR eye-camera cost on the RTX 5090.
#
#   powershell -ExecutionPolicy Bypass -File tools\multigpu_win5090.ps1 -Plan      # print the steps, run nothing
#   powershell -ExecutionPolicy Bypass -File tools\multigpu_win5090.ps1            # run them (about 25 minutes)
#
# Before: the GPU window is on (ssh wsl5090 'tmux new -d -s gpuwin ~/cc58/gpu_window.sh'), nobody uses the desktop
# (the yield scripts would pause WSL and C:\cc58), the host checkout in WSL (~/cc58/coupled-carla) is at the commit
# that has tools/multigpu_check.py and tools/vr_bench.py. Everything it starts it stops; at the end the task
# cc58-pkgserver runs again as before. Results: C:\cc58\mgpu-test\<time>\ (and the WSL logs it names).
#
# Steps
#   1. baseline: the dry run (three local bot stations, 40 s) against the single server of cc58-pkgserver
#   2. stop cc58-pkgserver; start the package as primary (-nullrhi, RPC 2000) and as secondary (RPC 3000, primary
#      port 2002, -graphicsadapter=0 -RenderOffScreen), both with the same -carla-packs
#   3. protocol check (tools/multigpu_check.py): the secondary switched the primary to synchronous 1/20 s and
#      guard_settings puts 0.02 back; a camera gets its images from the secondary
#   4. the dry run again with server.multigpu.secondaries [{name: gpu0, launch: false}] (experiments/_mgpu_test.yaml,
#      written from the study file, not committed)
#   5. the VR cost on the RTX 5090 (tools/vr_bench.py, simulated head): two eye cameras at 960x1056, 1440x1584,
#      1832x1920 on the single server
#   6. stop both, start cc58-pkgserver again, print the comparison
param(
  [string]$Package = "C:\cc58\carla\Build\Package\Carla-1.0-Win64-Shipping",
  [string]$Packs = "C:\cc58\carla_packs",
  [string]$Study = "experiments/eindhoven_crossing_study.yaml",
  [string]$Distro = "Ubuntu-24.04",
  [string]$WslUser = "crowd",
  [int]$Duration = 40,
  [switch]$Plan,
  [switch]$SkipVr
)
$ErrorActionPreference = "Stop"
$stamp = Get-Date -Format "yyyyMMdd-HHmm"
$out = "C:\cc58\mgpu-test\$stamp"
$exe = Join-Path $Package "CarlaUnreal.exe"
$common = @("-nosound", "-unattended", "-quality-level=Epic", "-carla-packs=$Packs")
$primaryArgs = @("-nullrhi", "-carla-rpc-port=2000", "-carla-streaming-port=2001", "-carla-secondary-port=2002") + $common
$secondaryArgs = @("-carla-rpc-port=3000", "-carla-streaming-port=3001", "-carla-primary-host=127.0.0.1",
                   "-carla-primary-port=2002", "-graphicsadapter=0", "-RenderOffScreen") + $common
$wslPrefix = "cd ~/cc58/coupled-carla && source ~/cc58/env58.sh && "

function Step($text) { Write-Host ""; Write-Host "== $text" -ForegroundColor Cyan }
function Wsl($cmd, $log) {
  Write-Host "   wsl: $cmd"
  if ($Plan) { return }
  & wsl -d $Distro -u $WslUser -- bash -lc ($wslPrefix + $cmd) 2>&1 | Tee-Object -FilePath (Join-Path $out $log)
}
function WaitPort($port, $seconds) {
  if ($Plan) { Write-Host "   wait for port $port"; return }
  $t0 = Get-Date
  while (-not (Test-NetConnection 127.0.0.1 -Port $port -InformationLevel Quiet -WarningAction SilentlyContinue)) {
    if (((Get-Date) - $t0).TotalSeconds -gt $seconds) { throw "port $port not up after $seconds s" }
    Start-Sleep 3
  }
  Write-Host ("   port ${port}: up after " + [int]((Get-Date) - $t0).TotalSeconds + " s")
}
function Gpu($tag) {
  if ($Plan) { return }
  $q = & nvidia-smi --query-gpu=memory.used,memory.total,utilization.gpu --format=csv,noheader
  "$tag $q" | Tee-Object -FilePath (Join-Path $out "gpu.txt") -Append
}
function StopCarla {
  if ($Plan) { Write-Host "   stop every CarlaUnreal process this script started"; return }
  Get-Process | Where-Object { $_.Path -like "$Package*" } | Stop-Process -Force -ErrorAction SilentlyContinue
  Start-Sleep 5
}

if (-not $Plan) {
  New-Item -ItemType Directory -Force -Path $out | Out-Null
  if (-not (Test-Path $exe)) { throw "no package at $exe" }
  Start-Transcript -Path (Join-Path $out "transcript.txt") | Out-Null
}
Write-Host "multi-GPU test on win5090 ($stamp); results in $out"
Write-Host "primary:   $exe $($primaryArgs -join ' ')"
Write-Host "secondary: $exe $($secondaryArgs -join ' ')"

Step "0. checks: GPU window, free GPU memory, the single server"
Wsl "test -e ~/cc58/gpu_flag && echo 'GPU window: on' || echo 'GPU window: OFF (start it first)'" "check.txt"
Gpu "before"

Step "1. baseline: the dry run against the single packaged server (cc58-pkgserver)"
if (-not $Plan) { & schtasks /run /tn cc58-pkgserver | Out-Null }
WaitPort 2000 180
Gpu "single server"
Wsl "python tools/station_dryrun.py $Study --duration $Duration --out logs/mgpu-$stamp/single" "dryrun_single.txt"

if (-not $SkipVr) {
  Step "5 (first, on the single server). the VR eye cameras' cost on the RTX 5090"
  Wsl "python tools/vr_bench.py --port 2000 --runtime sim --sizes 0 1280x720:1 960x1056 1440x1584 1832x1920 --ticks 300 --out logs/mgpu-$stamp/vr_bench.json" "vr_bench.txt"
}

Step "2. primary (-nullrhi) and secondary (RTX 5090)"
if (-not $Plan) { & schtasks /end /tn cc58-pkgserver | Out-Null }
StopCarla
if (-not $Plan) { Start-Process -FilePath $exe -ArgumentList $primaryArgs -WindowStyle Hidden | Out-Null }
WaitPort 2000 240
if (-not $Plan) { Start-Process -FilePath $exe -ArgumentList $secondaryArgs -WindowStyle Hidden | Out-Null }
WaitPort 3000 240
Start-Sleep -Seconds ($(if ($Plan) { 0 } else { 20 }))
Gpu "primary + secondary"

Step "3. protocol: synchronous 1/20 s after the secondary connects, guard_settings, a camera rendered by the secondary"
Wsl "python tools/multigpu_check.py settings --primary 127.0.0.1 --port 2000 --step 0.02 --ticks 40" "check_settings.txt"
Wsl "python tools/multigpu_check.py station --primary 127.0.0.1 --port 2000 --role pedestrian --timeout 10 --plan gpu0@server:3001/pedestrian" "check_station.txt"

Step "4. the dry run with server.multigpu (experiments/_mgpu_test.yaml)"
Wsl ("python tools/multigpu_check.py experiment --base $Study --out experiments/_mgpu_test.yaml --secondary gpu0@127.0.0.1:3000 && " +
     "python -m coupledcarla.multigpu experiments/_mgpu_test.yaml; " +
     "python tools/station_dryrun.py experiments/_mgpu_test.yaml --duration $Duration --out logs/mgpu-$stamp/multigpu") "dryrun_multigpu.txt"
Gpu "after the multi-GPU dry run"

Step "6. back to the single server; the comparison"
StopCarla
if (-not $Plan) { & schtasks /run /tn cc58-pkgserver | Out-Null }
WaitPort 2000 180
Wsl "rm -f experiments/_mgpu_test.yaml" "cleanup.txt"
if (-not $Plan) {
  foreach ($f in @("dryrun_single.txt", "dryrun_multigpu.txt")) {
    Write-Host "== $f"
    Get-Content (Join-Path $out $f) -Tail 25
  }
  Stop-Transcript | Out-Null
}
Write-Host ""
Write-Host "Compare in $out : dryrun_single.txt vs dryrun_multigpu.txt (RTF, station fps, tick-to-image p95), gpu.txt,"
Write-Host "check_settings.txt (expect synchronous 0.05 then 0.02), check_station.txt (expect images > 0), vr_bench.txt."
