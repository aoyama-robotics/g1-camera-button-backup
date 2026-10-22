$ErrorActionPreference = 'Stop'
$env:PYTHONPATH = 'E:\sandrone\.deps;E:\sandrone'
$env:PYTHONDONTWRITEBYTECODE = '1'
$taskTmp = Join-Path $PSScriptRoot 'output\tmp'
New-Item -ItemType Directory -Force $taskTmp | Out-Null
$env:TEMP = $taskTmp
$env:TMP = $taskTmp
Push-Location $PSScriptRoot
try {
    python scripts\build_mount.py
    if ($LASTEXITCODE -ne 0) { throw 'CAD generation or validation failed' }
} finally { Pop-Location }
