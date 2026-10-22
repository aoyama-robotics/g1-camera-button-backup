$ErrorActionPreference = 'Stop'
$env:PYTHONPATH = 'E:\sandrone\.deps_cae_precision;E:\sandrone\.deps;E:\sandrone'
$env:PYTHONDONTWRITEBYTECODE = '1'
$env:OMP_NUM_THREADS = '1'
Push-Location $PSScriptRoot
try {
    python scripts/build_camera_attachment.py
    if ($LASTEXITCODE -ne 0) { throw 'Camera clamp CAD validation failed' }
    python scripts/finalize_camera_attachment.py
    if ($LASTEXITCODE -ne 0) { throw 'Camera clamp packaging failed' }
} finally { Pop-Location }
