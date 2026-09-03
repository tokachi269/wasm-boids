# Architecture

この文書は、変更時に越えてはいけない責務境界だけを定める。Boidsの詳細仕様はREADMEとC++実装、性能基準は `scripts/bench.md` を参照する。

## Runtime flow

```text
Species settings / seed / dt
  -> C++ BoidSimulation
  -> WASM exported buffers
  -> WasmtimeBridge views
  -> BoidInstancing / Three.js scene
  -> FogPipeline
  -> screen
```

## Decision owners

| Decision | Owner | Consumers |
|---|---|---|
| 個体位置、速度、姿勢、近傍相互作用、tree、cluster | `src/wasm/*` | WASM bindings、bridge、描画 |
| WASM bufferの取得とJS viewの寿命 | `src/simulation/WasmtimeBridge.js` | `App.vue`、描画 |
| 個体のLOD、instance packing、material patch | `src/rendering/BoidInstancing.js` | Three.js renderer |
| 水中媒質、SSAO、Bloom、post-process順序 | `src/rendering/FogPipeline.js` | screen output |
| scene構成、camera、light、ground/background、UI連携 | `src/App.vue` | renderer、UI |
| 再現可能な性能計測 | native/browser benchmark | 人間による比較判断 |

下流は上流の意味を再判定しない。描画都合の変更をC++のシミュレーションへ持ち込まず、見た目の調整で物理状態を書き換えない。

## Simulation invariants

- read/write bufferのswap位置、update phase順序、条件、定数は挙動仕様である。
- seed、dt、task数が同じ1 task実行では最終position checksumが一致することを基準とする。
- benchmark timingとdiagnosticsは観測専用であり、neighbor選択や更新頻度を変えない。
- pthreadの通常経路と決定論比較用1 taskを混同しない。

## Rendering and depth

- Three.jsのcolor、alpha、depth bufferは別の情報である。半透明alphaは単一のdepth値では表現できない。
- depth-based post-processへ透明meshを書き込む場合、透明部分を含む面全体が実在する不透明面として扱われ得る。
- 透明meshに水中媒質が必要な場合は、目的に応じてmaterial内で媒質を適用するか、専用compositeを使う。背景色の変更で境界を隠さない。
- 背景meshは見た目の背景であり、実在geometryとしてdepth-based effectへ参加させる場合は、その意味を明示する。
- Bloomはcamera側の効果、媒質はsceneとcamera間の効果として描画順を決める。

## Build and delivery

- Debug WASMは開発確認用。性能と公開物はRelease (`-O3`, `-msimd128`, pthread) を基準にする。
- `dist/`は生成物であり、正本は `src/`, `public/`, build設定である。
- GitHub Pagesへのdeployは、Release buildとローカル実画面確認の後に行う。
