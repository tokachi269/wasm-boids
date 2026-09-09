# Command cheatsheet

workdir: `D:\GitHub\wasm-boids`

このリポジトリでは `package.json` のnpm scriptsを通常の入口とする。下記にないraw `cmake`、`ninja`、生成物の直接リンクを通常のbuild成功として扱わない。

## Native Release

```powershell
npm run build-native
```

固定条件の既定benchmark:

```powershell
npm run benchmark
```

個体数などを指定する場合:

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File scripts/run-native-benchmark.ps1 -Frames 2000 -Seed 1 -Boids 50000 -Tasks 1
```

正当性比較はRelease、seed 1、fixed dt 1/60、1 taskで行う。詳細は `scripts/bench.md` を参照する。

## WASM

```powershell
npm run build-wasm:dev
npm run build-wasm:prod
```

通常のproduction build:

```powershell
npm run build
```

## Browser benchmark

```powershell
npm run benchmark:build
npm run benchmark:serve
```

測定URLと条件は `scripts/bench.md` を参照する。

## Development server

```powershell
npm run serve
```

## Deploy

```powershell
npm run deploy
```

deployはユーザーが明示した場合だけ実行する。実行前後の確認条件は `docs/testing.md` を参照する。

## 注意事項

- Native buildは `scripts/build-native.ps1` がVisual Studio環境を初期化するため、通常は別途Developer PowerShellを開く必要はない。
- build内部のgeneratorを手動実行しない。npm scriptが停止した場合は、別コマンドへ迂回せず停止箇所と残留processを確認する。
- C++またはWASM binding変更では、native buildだけで完了にせず該当するWASM buildも通す。
- docsだけの変更では全buildを実行せず、`git diff --check` とリンク・コマンドの目視確認に留める。
