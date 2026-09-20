# Command cheatsheet

workdir: `D:\GitHub\wasm-boids`

このリポジトリでは `package.json` のnpm scriptsを通常の入口とする。下記にないraw `cmake`、`ninja`、生成物の直接リンクを通常のbuild成功として扱わない。

## Native Release

```powershell
npm run build-native
```

Nativeの小さな契約テスト:

```powershell
npm run test-native
```

固定条件の既定benchmark:

```powershell
npm run benchmark
```

Release最適化を維持したままWindows CPU sampling用PDBを生成する場合:

```powershell
npm run build-native:profile
```

相互作用経路の回数を集計する診断専用native build:

```powershell
npm run build-native:diagnostics
powershell -NoProfile -ExecutionPolicy Bypass -File scripts/run-native-benchmark.ps1 -Frames 1300 -Warmup 300 -Seed 1 -Boids 50000 -Tasks 1
```

診断counterは通常のnative/WASM buildには含まれない。

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

通常のDebug WASM:

```powershell
npm run serve
```

Release最適化（`-O3`）したWASMで、Vueのhot reloadとWASMの自動再buildを使う場合:

```powershell
npm run serve:release
```

いずれも起動は1コマンドでよい。`App.vue`などのJavaScript/Vue変更はhot reloadされ、C++変更時は監視中のWASM buildだけが再実行される。

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
