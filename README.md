# wasm-boids

C++ / WebAssemblyで群れを計算し、Three.jsで描画するブラウザ向け魚群シミュレーション。
数万匹規模での実行を対象とし、近傍探索、データ配置、インスタンシングに重点を置く。

[デモ](https://tokachi269.github.io/wasm-boids/)

## 概要

- 分離・整列・凝集によるBoidsモデル
- 捕食者からの逃避とストレス応答
- 小クラスターと大クラスターの推定
- C++側のSoA形式データとダブルバッファ
- 空間階層、近傍キャッシュ、空間順への並べ替え
- Three.js `InstancedMesh`による描画とLOD
- 水中フォグ、SSAO、Bloom
- 固定seedによるnative / browser benchmark

## 構成

| 領域 | 主な責務 |
| --- | --- |
| `src/wasm` | Boids、近傍探索、tree、cluster、WASM bindings |
| `src/simulation` | WASMの初期化、buffer view、simulation step |
| `src/rendering` | InstancedMesh、LOD、水中表現、post-process |
| `src/components` | 設定UI |
| `src/benchmark` | browser benchmark |
| `scripts` | ビルド、ネイティブベンチマーク、配信サーバー |

実行時のデータ経路:

```text
C++ simulation
  -> WASM buffers
  -> JavaScript typed-array views
  -> Three.js InstancedMesh
  -> post-process
```

## 性能計測

性能評価には、同一seed・個体数・task数で再実行できるネイティブベンチマークを使用する。
ブラウザベンチマークでは、simulation、instance packing、render submission、GPU時間を個別に計測する。

ベンチマーク値は変更前後の比較用であり、実画面のフレームレートとは直接対応しない。
測定条件と結果の読み方は [`scripts/bench.md`](scripts/bench.md) を参照。

## セットアップ

必要環境:

- Node.js / npm
- Emscripten SDK
- CMake

依存パッケージ:

```sh
npm ci
```

開発用WASMビルド:

```sh
npm run build-wasm:dev
```

開発サーバー:

```sh
npm run serve
```

本番ビルド:

```sh
npm run build
```

ネイティブベンチマーク:

```sh
npm run benchmark
```

利用可能なコマンドと前提条件は [`docs/command_cheatsheet.md`](docs/command_cheatsheet.md) を参照。

## 開発資料

- [`docs/architecture.md`](docs/architecture.md): 実装の責務境界
- [`docs/testing.md`](docs/testing.md): 変更内容ごとの検証方針
- [`docs/engineering/agent_harness.md`](docs/engineering/agent_harness.md): バグ修正と画面確認の手順
- [`scripts/bench.md`](scripts/bench.md): benchmarkの条件と計測項目

## 参考文献

- 石橋ら「大規模な魚群シミュレーションのための階層的Boidアルゴリズム」情報処理学会 CG-133 (2008)
- Ito & Uchida, J. Phys. Soc. Jpn., 91, 064806 (2022)

## ライセンス

[MIT License](LICENSE)
