# wasm-boids

C++ / WebAssemblyで魚群シミュレーションを実行し、Three.jsで描画するブラウザ向けアプリケーション。

Boidsの分離・整列・凝集を基礎に、少数近傍、近傍記憶、捕食者応答、群れ全体の補助的な追跡を組み合わせる。数万匹規模での実行を前提とし、空間階層、近傍キャッシュ、SoA、空間順reorder、InstancedMesh / LODを利用する。

[デモ](https://tokachi269.github.io/wasm-boids/)

<!-- hero image / short movie -->

| 項目 | 内容 |
| --- | --- |
| シミュレーション | C++ / WebAssembly |
| 描画 | Three.js / InstancedMesh / LOD |
| 対象 | 3D魚群、数万匹規模 |
| 主な要素 | 局所相互作用、近傍記憶、捕食者応答、空間階層、再現可能な性能検証 |

## シミュレーション

### 基本モデル

各個体は、視野と距離の条件を満たす少数の近傍から分離・整列・凝集の影響を受ける。通常の群泳に全個体の重心や平均方向は使用しない。

1個体の操舵は、概念的には次の要素から構成される。

\[
\mathbf{a}_i =
\mathbf{S}_i +
\mathbf{A}_i +
\mathbf{C}_i +
\mathbf{R}_i +
\mathbf{E}_i +
\mathbf{P}_i
\]

| 項 | 内容 |
| --- | --- |
| \(\mathbf{S}\) | separation: 近傍から離れる |
| \(\mathbf{A}\) | alignment: 近傍の進行方向へ揃える |
| \(\mathbf{C}\) | cohesion: 近傍側へ寄る |
| \(\mathbf{R}\) | 魚体寸法を基準にした近距離反発 |
| \(\mathbf{E}\) | 捕食者からの逃避 |
| \(\mathbf{P}\) | 群れから大きく外れた個体への復帰補助 |

群れ内部の形状と回転は主に局所相互作用から生じる。大クラスターへの復帰力は通常の群れ形状を生成する主規則ではなく、離脱個体や過度に広がった状態を戻すための補助として扱う。

### 近傍選択と近傍記憶

各個体が参照する近傍数は `maxNeighbors` で制限する。近傍は毎フレーム完全に選び直すのではなく、個体IDと経過時間を持つキャッシュとして保持する。

保持中の近傍は各stepで検証し、無効または寿命切れになった要素を取り除く。不足した分だけleaf内または周辺の空間検索から補充する。記憶中の寄与は経過時間に応じて減衰する。

近傍のidentityにはphysical indexとは別のstable IDを使用する。これにより、物理配列を並べ替えても同じ個体への参照を維持できる。

### 群れ全体の追跡

空間階層のleafから小クラスターを推定し、近接する小クラスターをまとめて大クラスターとして追跡する。

| 局所相互作用 | 群れ全体の追跡 |
| --- | --- |
| 分離・整列・凝集 | small / school cluster |
| 視野・距離・近傍記憶 | 群れ中心・半径・追跡状態 |
| 近距離反発 | 離脱個体への復帰補助 |
| 捕食者への直接反応 | デバッグ表示・カメラ基準 |

clusterは群れ全体の状態推定に使用し、個体間の通常の群泳規則とは分離する。

### 捕食者

通常魚は `predatorAlertRadius` 内の捕食者を検知し、逃避方向と脅威状態を更新する。脅威は時間とともに減衰し、逃避中は通常の群泳より回避を優先する。

捕食者側は対象の追跡と休止状態を持つ。通常魚の局所相互作用とは別経路で処理する。

### 運動と姿勢

相互作用から得た操舵を速度と位置へ反映し、姿勢を更新する。

姿勢制御は主に次の要素から構成される。

- 目標方向への回転応答
- 移動距離あたりの最大曲率
- 水平化トルク
- 最小 / 最大速度

`maxTurnAngle` は単純な角速度上限ではなく、移動距離に対する最大曲率として扱う。

時間に意味を持つ状態は経過時間を基準に更新する。近傍寿命、脅威・ストレスの減衰、cluster追跡などが該当する。tree rebuild、leaf再収集、reorderなどの保守処理は物理状態とは分離し、所定のframe間隔で実行する。

## 大規模化

### 空間階層

全個体間を直接比較する場合、候補数はおおよそ

\[
N(N-1)
\]

となる。

本実装では個体群を空間的なunitへ分割し、leafを近傍探索の基本単位とする。通常は同一leaf内から近傍を補充し、不足時のみ周辺のleafを検索する。

近傍が確定した後の相互作用計算は、各個体が参照する近傍数を \(k\) とすると概ね

\[
N \times k,\qquad k \leq \texttt{maxNeighbors}
\]

となる。これは近傍候補探索やtree maintenanceの計算量を含まない。

treeは完全rebuildと局所的なsplit / mergeを組み合わせて維持する。完全rebuildを毎frame実行せず、保守処理の集中を避ける。

### データ配置

シミュレーション状態はC++側のSoAに保持する。

```cpp
positions[i]
velocities[i]
orientations[i]
speciesIds[i]
stresses[i]
```

位置・速度・姿勢はread/write bufferを分離し、1 step中は全個体が同じ時点の状態を参照する。step完了後にbufferをswapする。

近傍キャッシュは個体ごとに `maxNeighbors` 分の領域を持ち、有効要素を先頭側へ詰める。近傍index、age、stable slotをまとめて保持する。

tree更新後は、種族ごとの領域を維持したままleaf順へphysical storageを並べ替える。空間的に近い個体をメモリ上でも近づけるための処理である。

reorderではstable IDとphysical indexを分離し、次の参照を同じ対応表で更新する。

- SoAの各buffer
- neighbor reference
- predator target
- tree / leaf index
- inspector / debug reference

### 描画

C++側のシミュレーション状態はWebAssembly memoryを通してJavaScriptから参照する。

| 層 | 主な責務 |
| --- | --- |
| C++ | 個体状態、相互作用、空間階層、cluster、運動更新 |
| WebAssembly | C++の状態と処理をブラウザへ公開 |
| JavaScript | WASM buffer view、simulation step、描画データの受け渡し |
| Three.js | InstancedMesh、LOD、camera、post-process |

通常魚は `InstancedMesh` で描画し、距離に応じてLODを切り替える。水中fog、SSAO、Bloomなどの描画処理はシミュレーション状態を変更しない。

## 性能と検証

性能変更は、同一条件での計測とシミュレーション状態の検証を分けて評価する。

| 変更 | 確認 |
| --- | --- |
| 挙動を変えない最適化 | fixed seed / fixed dt / 1 taskでchecksum一致 |
| physical reorder | stable ID基準の状態一致、参照index validation |
| 相互作用・時間処理の変更 | native Releaseで挙動差を確認 |
| 性能改善 | 同一条件のA/B benchmark |
| hotspotの特定 | 区間計測、CPU sampling、必要に応じてPMC |
| 描画変更 | Release相当の実画面、Console / WebGL error |
| deploy | production build後に公開URLを確認 |

### Native benchmark

native benchmarkでは、CPU側の処理を区間ごとに計測する。

| 区間 | 内容 |
| --- | --- |
| interaction | 近傍選択と相互作用 |
| kinematics | 速度・位置・姿勢の更新 |
| tree build | 空間階層の再構築 |
| reorder | physical storageの並べ替え |
| cluster update | small / school clusterの更新 |

固定seed、個体数、task数、warmup、measurement frame数を指定できる。挙動同値の最適化では、性能値とは別にsimulation checksumを比較する。

### Browser benchmark

browser benchmarkでは、次の処理を分離して計測する。

- WASM simulation
- JavaScript側のinstance packing
- render submission
- GPU timer query
- 実フレーム間隔

実画面のFPSは最終的な体感指標として使用するが、CPU simulation、JavaScript、GPUのどこが律速しているかの判断には区間別の計測を使用する。

測定条件とコマンドは [`scripts/bench.md`](scripts/bench.md)、変更ごとの検証基準は [`docs/testing.md`](docs/testing.md) を参照。

## 主なパラメータ

| 対象 | 主なparameter |
| --- | --- |
| 基本の群泳 | `cohesion`, `separation`, `alignment` |
| 作用距離 | `cohesionRange`, `separationRange`, `alignmentRange` |
| 近傍 | `maxNeighbors`, `tau`, field of view |
| 移動 | `minSpeed`, `maxSpeed` |
| 旋回・姿勢 | `maxTurnAngle`, `torqueStrength`, `horizontalTorque` |
| 捕食者 | `predatorAlertRadius`, escape / threat系 |
| 群れへの復帰補助 | school pull系 |

ワールド座標の1 unitは1 mとして扱う。各parameterは独立ではなく、近傍数、作用範囲、旋回能力の組み合わせによって群れの形状が変化する。

## プロジェクト構成

| パス | 内容 |
| --- | --- |
| `src/wasm` | 個体更新、空間階層、近傍cache、cluster、WASM binding |
| `src/simulation` | WASM初期化、buffer view、simulation step |
| `src/rendering` | InstancedMesh、LOD、水中表現、post-process |
| `src/components` | 設定UI |
| `src/benchmark` | browser benchmark |
| `scripts` | build、native benchmark、deploy関連 |

責務境界は [`docs/architecture.md`](docs/architecture.md) を参照。

## セットアップ

必要環境:

- Node.js / npm
- Emscripten SDK
- CMake

```powershell
npm ci
npm run build-wasm:dev
npm run serve
```

production build:

```powershell
npm run build
```

利用可能なコマンドは [`docs/command_cheatsheet.md`](docs/command_cheatsheet.md) を参照。

## 開発資料

- [`docs/architecture.md`](docs/architecture.md) — 実装の責務境界
- [`docs/testing.md`](docs/testing.md) — 変更内容ごとの検証方針
- [`docs/engineering/agent_harness.md`](docs/engineering/agent_harness.md) — バグ修正と画面確認
- [`scripts/bench.md`](scripts/bench.md) — benchmarkの条件と計測項目

## 参考

本実装は以下の研究を参考にするが、論文実装の忠実な再現ではない。

| 参考 | 参照している点 |
| --- | --- |
| Yoshiaki Ishibashi, Norimasa Yoshida, 「大規模な魚群シミュレーションのための階層的Boidアルゴリズム」 | 個体群を階層的に扱う考え方 |
| Susumu Ito, Nariya Uchida, “Emergence of a Giant Rotating Cluster of Fish in Three Dimensions by Local Interactions” | 少数近傍との局所相互作用から回転群が生じる考え方 |

- [大規模な魚群シミュレーションのための階層的Boidアルゴリズム](https://ipsj.ixsq.nii.ac.jp/records/37917)
- [Emergence of a Giant Rotating Cluster of Fish in Three Dimensions by Local Interactions](https://doi.org/10.7566/JPSJ.91.064806)

## ライセンス

[MIT License](LICENSE)