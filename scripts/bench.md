# 性能ベンチマーク

## 固定シードのnative基準

`npm run benchmark` はnative Releaseをビルドし、5,000個体、seed 1、固定dt 1/60、warm-up 1,000＋測定1,000フレームを1タスクで実行する。標準出力はJSON 1行で、最適化比較では `frame_ms.p95` を見て、同じ条件の `checksum` が一致する結果だけを採用する。

引数を変える場合は次のように実行する。

```powershell
npm run build-native
powershell -NoProfile -ExecutionPolicy Bypass -File scripts/run-native-benchmark.ps1 -Frames 2000 -Seed 1 -Boids 50000 -Tasks 1
```

`-Tasks 1` は決定論A/B比較用。`-Tasks 0` は現在の通常上限（nativeは最大8タスク）を使うが、現実装では同一seedでもchecksumが安定しないため性能参考値としてのみ扱う。

## Release WASM＋JS＋GPU

```powershell
npm run benchmark:build
npm run benchmark:serve
```

Chrome 151で `http://127.0.0.1:4173/?bench=1&boids=20000&seed=1&tasks=1&warmup=300&samples=1000` を開く。COOP/COEP付きでRelease成果物（`-O3 -msimd128 -pthread`）を実行し、終了後に画面とConsoleへJSON 1行を出す。`tasks=0` はブラウザ通常上限の4タスク比較、`tasks=1` はchecksum比較用。GPU値は `EXT_disjoint_timer_query_webgl2` が利用できる場合だけ真のGPU時間として出し、`render_submission_cpu` とは非同期に重なるため足し合わせない。

## SIMD確認

```powershell
npm run build-wasm:prod
npm run benchmark:simd
```

これは最終WASM内のSIMD命令数を静的集計する。主要処理がどの程度SIMD化されたかの厳密な帰属には、関数単位の逆アセンブルとプロファイルを追加で使う。

## 2026-09-02 現環境の基準

環境は Chrome 151、8 logical CPUs、NVIDIA GeForce RTX 3060 / ANGLE D3D11。全件Release、seed 1、固定dt 1/60、warm-up 300、測定1,000、1タスク。

| Boids | frame mean | median | p95 | p99 | max | WASM mean | JS packing mean | GPU mean | checksum |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|:---|
| 5,000 | 8.12 | 8.03 | 11.07 | 12.51 | 24.16 | 6.43 | 0.39 | 2.52 | `e2cccbd6a0d970e8` |
| 10,000 | 14.94 | 14.54 | 19.94 | 22.56 | 26.29 | 13.08 | 0.71 | 3.40 | `3faa28f182dfa868` |
| 20,000 | 29.55 | 28.72 | 39.06 | 44.47 | 56.80 | 27.02 | 1.25 | 5.61 | `20620cc15d01707d` |
| 30,000 | 52.83 | 48.59 | 80.31 | 93.51 | 131.75 | 49.20 | 2.11 | 7.85 | `e2d4209dfe2eaf6c` |
| 50,000 | 84.00 | 78.41 | 122.14 | 154.63 | 249.17 | 79.68 | 2.71 | 10.35 | `304150aef216683b` |

単位はms。5,000個体・1タスクは同じ条件で2回実行し、checksum `e2cccbd6a0d970e8` の一致を確認した。

50,000個体のフレーム平均に対する累積内訳は `computeBoidInteraction` 66.1%、kinematics 24.2%、tree rebuild 2.6%、cluster更新 0.6%、JS instance packing 3.2%、render submission CPU 1.8%。WASM全体が平均79.68msでフレーム平均84.00msの94.9%を占める。GPU平均10.35msはCPUと並行するため割合の合計には含めない。

tree rebuildは100回/1,000フレームで、50,000個体では合計2,223ms。定期的なrebuildとWASM相互作用時間の変動がp95〜maxのスパイクに寄与している。split/mergeは合計1.29msで支配的ではない。

20,000個体の通常4タスク条件は frame mean 16.33ms、p95 26.57ms。相互作用チャンクの最悪 `max/mean` は2.16、kinematicsは2.84で負荷偏りがある。ただし同条件を2回実行したchecksumは `cd0a0c876ab92dba` と `1de129282bcf6939` で不一致だったため、4タスク条件をビット一致判定には使わない。

局所性診断はwarm-up中の1フレームだけ採取した。50,000個体では `abs(neighborIndex-selfIndex)>256` が同一leafで99.0%、external neighborで98.9%、平均差はそれぞれ16,838と17,048だった。現在の物理配列indexは空間的近傍とほぼ局所化していない。

最終WASMの静的命令数は `v128.load` 207、`v128.store` 303、`f32x4.add` 8、`f32x4.mul` 22、`f32x4.sub` 13、`f32x4.div` 1。`boid_unit.cpp.o` だけではSIMD算術が add 3 / mul 8 / sub 8なのに対し、scalarはload 456 / store 147 / add 372 / mul 686 / sub 201 / div 121 / sqrt 46で、主要Boid処理はscalar中心と判断できる。

## 次に調査する候補（未実装）

| 候補 | 期待効果 | 変更量 | 挙動変更リスク | 根拠 |
|:---|:---|:---|:---|:---|
| `computeBoidInteraction` 内の検索・ループをさらにプロファイル | 最大。相互作用内部の真の支配箇所を絞る | 小〜中 | なし（計測のみ） | 50,000個体でフレーム平均の66.1% |
| 空間近傍とSoA indexの局所化方式を試作 | キャッシュミス削減の可能性 | 大 | 高。並べ替えで乱数・演算順・外部参照が変わり得る | index差>256が約99% |
| チャンク分割をleaf数ではなく推定仕事量で均等化 | thread待ち時間の削減 | 中 | 中。実行順と競合の扱いが必要 | 4タスクの最悪max/mean 2.16〜2.84 |
| コンパイラSIMD化を妨げる依存関係の調査 | 相互作用・kinematicsの演算短縮 | 中〜大 | 中〜高。浮動小数点順序が変わる | boid_unitはscalar命令が大多数 |

まず相互作用内部の読み取り専用プロファイルを追加し、その後に各候補を別ブランチ・同一checksum条件で比較する。通常4タスクの非決定性は最適化より先に原因を診断する必要がある。
