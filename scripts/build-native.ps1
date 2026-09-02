$ErrorActionPreference = 'Stop'

$vswhere = Join-Path ${env:ProgramFiles(x86)} 'Microsoft Visual Studio\Installer\vswhere.exe'
if (Test-Path $vswhere) {
    $installationPath = & $vswhere -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath
    if ($installationPath) {
        $vsDevCmd = Join-Path $installationPath 'Common7\Tools\VsDevCmd.bat'
        cmd /d /s /c "`"$vsDevCmd`" -arch=x64 -host_arch=x64 >nul && set" | ForEach-Object {
            if ($_ -match '^([^=]+)=(.*)$') {
                Set-Item -Path "Env:$($matches[1])" -Value $matches[2]
            }
        }
    }
}

cmake --fresh -S . -B build-native -G Ninja -DCMAKE_BUILD_TYPE=Release '-DCMAKE_CXX_FLAGS=/utf-8 /EHsc'
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
cmake --build build-native
exit $LASTEXITCODE
