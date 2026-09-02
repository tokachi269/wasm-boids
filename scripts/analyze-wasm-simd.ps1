$ErrorActionPreference = 'Stop'

$wasmDis = if ($env:EMSDK) {
    Join-Path $env:EMSDK 'upstream\bin\wasm-dis.exe'
} else {
    ''
}
if (-not $wasmDis -or -not (Test-Path $wasmDis)) {
    $wasmDis = 'D:\GitHub\emsdk\upstream\bin\wasm-dis.exe'
}
if (-not $wasmDis -or -not (Test-Path $wasmDis)) {
    throw 'wasm-dis.exe not found. Activate emsdk or set EMSDK.'
}

$wasm = Join-Path $PSScriptRoot '..\src\wasm\build\prod\wasm_boids.wasm'
if (-not (Test-Path $wasm)) {
    throw 'Release WASM not found. Run npm run build-wasm:prod first.'
}

& $wasmDis $wasm -o - |
    Select-String -AllMatches 'v128\.load|v128\.store|f32x4\.add|f32x4\.mul|f32x4\.sub|f32x4\.div|f32x4\.sqrt' |
    ForEach-Object { $_.Matches.Value } |
    Group-Object |
    Sort-Object Count -Descending |
    Select-Object Count, Name
