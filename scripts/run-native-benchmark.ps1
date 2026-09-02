param(
    [int]$Frames = 2000,
    [uint32]$Seed = 1,
    [int]$Boids = 5000,
    [int]$Tasks = 1
)

$ErrorActionPreference = 'Stop'
$executable = Join-Path $PSScriptRoot '..\build-native\wasm_boids_native.exe'
if (-not (Test-Path $executable)) {
    throw "Native benchmark executable not found. Run npm run build-native first."
}

& $executable --bench $Frames --seed $Seed --boids $Boids --tasks $Tasks
exit $LASTEXITCODE
