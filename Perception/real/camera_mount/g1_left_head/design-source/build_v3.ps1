$ErrorActionPreference = 'Stop'
$env:PYTHONPATH = 'E:\sandrone\.deps_cae_precision;E:\sandrone\.deps;E:\sandrone'
$env:PYTHONDONTWRITEBYTECODE = '1'
$env:OMP_NUM_THREADS = '1'
$env:OPENBLAS_NUM_THREADS = '1'
$env:TEMP = Join-Path $PSScriptRoot 'output\tmp'
$env:TMP = $env:TEMP
New-Item -ItemType Directory -Force $env:TEMP | Out-Null
Push-Location $PSScriptRoot
try {
    python scripts/build_v3.py
    if ($LASTEXITCODE -ne 0) { throw 'v3 CAD failed' }
    foreach ($taskMesh in @(5,3.5)) {
        python scripts/fea_v2.py monolithic --version v3 --h $taskMesh
        if ($LASTEXITCODE -ne 0) { throw 'v3 FEA failed' }
    }
    python scripts/finalize_v3.py
    if ($LASTEXITCODE -ne 0) { throw 'v3 finalization failed' }
} finally { Pop-Location }
