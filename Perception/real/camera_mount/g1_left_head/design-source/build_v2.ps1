$ErrorActionPreference = 'Stop'
$env:PYTHONPATH = 'E:\sandrone\.deps_cae_precision;E:\sandrone\.deps;E:\sandrone'
$env:PYTHONDONTWRITEBYTECODE = '1'
$env:OMP_NUM_THREADS = '1'
$env:OPENBLAS_NUM_THREADS = '1'
$taskTmp = Join-Path $PSScriptRoot 'output\tmp'
New-Item -ItemType Directory -Force $taskTmp | Out-Null
$env:TEMP = $taskTmp
$env:TMP = $taskTmp
Push-Location $PSScriptRoot
try {
    python scripts\build_structural_v2.py
    if ($LASTEXITCODE -ne 0) { throw 'v2 geometry generation failed' }
    python scripts\check_elasticity_v2.py
    if ($LASTEXITCODE -ne 0) { throw 'Elasticity known-solution check failed' }
    python scripts\run_fea_v2.py --h 5
    if ($LASTEXITCODE -ne 0) { throw 'Coarse FEA failed' }
    python scripts\select_v2.py
    if ($LASTEXITCODE -ne 0) { throw 'Candidate selection failed' }
    $taskSelected = (Get-Content output\v2\selection.json -Raw | ConvertFrom-Json).selected
    python scripts\run_fea_v2.py --h 3.5 forward_plain baseline_v1 $taskSelected
    if ($LASTEXITCODE -ne 0) { throw 'Fine FEA failed' }
    python scripts\select_v2.py
    if ($LASTEXITCODE -ne 0) { throw 'Fine selection verification failed' }
    python scripts\finalize_v2.py
    if ($LASTEXITCODE -ne 0) { throw 'v2 deliverable verification failed' }
} finally { Pop-Location }
