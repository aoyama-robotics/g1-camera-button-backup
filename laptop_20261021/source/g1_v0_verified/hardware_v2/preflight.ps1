# No installation, network setting changes, or robot connection.
$ErrorActionPreference = 'Stop'
Set-Location $PSScriptRoot
python .\g1_arm.py preflight
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
python .\g1_arm.py nudge --joint 22 --delta-deg -1
exit $LASTEXITCODE
