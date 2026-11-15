# Upload the new / changed programs and docs to the Jetson.
# Run from PowerShell on this PC, in this folder:   .\upload_to_jetson.ps1
# You will be asked for the Jetson password TWICE (scp, then ssh).
#
# Files that already exist on the Jetson are backed up first as
# <name>.bak_<yyyymmdd_hhmmss>, so nothing newer on the Jetson is lost.
# (This file is kept ASCII-only on purpose: Windows PowerShell 5.1 misreads
#  non-ASCII characters in a script saved without a BOM.)

$JetsonUser = 'scott'
$JetsonHost = '192.168.55.1'
$RemoteDir  = '/home/scott/claude-project/gpsless_VO_run1/gpsless_VO_image'
$RemoteStage = '/home/scott/upload_staging'
$RunDir = 'photo_obtained/20261114_092057'

$Here = Split-Path -Parent $MyInvocation.MyCommand.Path
Set-Location $Here

$Files = @(
    # new programs + launchers
    'vo_flight_capture_fix1.py', 'vo_flight_capture_fix2.py', 'run_vo_flight_capture_fix2.sh',
    'vo_flight_capture_fix3.py', 'run_vo_flight_capture_fix3.sh',
    'photo_capture.py', 'run_photo_capture.sh',
    # changed programs (added "[frame N] photo: <file name>" log line)
    'vo_flight_capture.py', 'vo_flight_capture_always.py', 'vo_flight_capture_fix.py',
    # docs
    'vo_flight_capture_fix_comparison.md', 'vo_flight_capture_fix_comparison.html'
)
$Files += (Get-ChildItem -File 'photo_capture_*.md' | ForEach-Object { $_.Name })   # the Chinese user guide
$Files += (Get-ChildItem -File 'vo_flight_capture_fix3_*.md' | ForEach-Object { $_.Name })   # fix3 Chinese user guide
$RunFiles = @('flight_20261114_analysis.png', 'flight_20261114_analysis_gps_tilt.png', 'replay_gps_tilt.py')

# Build a local staging folder: top level -> program folder, run/ -> the 11/14 photo folder.
$LocalStage = Join-Path $env:TEMP 'jetson_upload_staging'
if (Test-Path $LocalStage) { Remove-Item -Recurse -Force $LocalStage }
New-Item -ItemType Directory -Force (Join-Path $LocalStage 'run') | Out-Null
foreach ($f in $Files) { Copy-Item -LiteralPath $f -Destination $LocalStage -ErrorAction Stop }
foreach ($f in $RunFiles) { Copy-Item -LiteralPath "$RunDir/$f" -Destination (Join-Path $LocalStage 'run') -ErrorAction Stop }
Write-Host "Staged $($Files.Count) program/doc files + $($RunFiles.Count) analysis files."

$Target = "$JetsonUser@$JetsonHost"
Write-Host "1/2  Uploading (enter the Jetson password) ..."
scp -r -o ConnectTimeout=10 $LocalStage "${Target}:$RemoteStage"
if ($LASTEXITCODE -ne 0) { throw "scp failed (exit $LASTEXITCODE)" }

Write-Host "2/2  Backing up existing files and installing (enter the password again) ..."
$remote = @"
set -e
ts=`$(date +%Y%m%d_%H%M%S)
cd $RemoteStage
for f in *; do
  [ -f "`$f" ] || continue
  if [ -f "$RemoteDir/`$f" ]; then cp -p "$RemoteDir/`$f" "$RemoteDir/`$f.bak_`$ts"; echo "backup:    `$f.bak_`$ts"; fi
  case "`$f" in *.py|*.sh) sed -i 's/\r`$//' "`$f" ;; esac
  mv -f "`$f" "$RemoteDir/`$f"; echo "installed: `$f"
done
mkdir -p "$RemoteDir/$RunDir"
for f in run/*; do mv -f "`$f" "$RemoteDir/$RunDir/"; echo "installed: $RunDir/`$(basename "`$f")"; done
chmod +x "$RemoteDir/run_vo_flight_capture_fix2.sh" "$RemoteDir/run_vo_flight_capture_fix3.sh""$RemoteDir/run_photo_capture.sh"
rmdir "$RemoteStage/run" "$RemoteStage"
echo "DONE -- files are in $RemoteDir"
"@
($remote -replace "`r", '') | ssh -o ConnectTimeout=10 $Target "bash -s"
if ($LASTEXITCODE -ne 0) { throw "remote install failed (exit $LASTEXITCODE)" }
Remove-Item -Recurse -Force $LocalStage
Write-Host "Upload complete."
