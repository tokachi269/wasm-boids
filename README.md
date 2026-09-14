# wasm-boids

本プロジェクトは、階層的Boidアルゴリズムと、局所相互作用から三次元の回転魚群が生じる研究を参考にしたリアルタイムシミュレーションである。

Boidsの分離・整列・凝集を基礎に、視野、少数近傍、近傍記憶、捕食者応答、旋回・姿勢制御を組み合わせる。論文実装の忠実な再現ではなく、魚群らしい集団運動、数万匹規模での実行、変更後の再現性を同時に扱うための独自実装である。

C++でシミュレーションを計算し、WebAssemblyの線形メモリに保持した位置・速度・姿勢をJavaScriptのtyped array viewから参照する。描画はThree.jsの `InstancedMesh` / LODが担当する。

[デモ](https://tokachi269.github.io/wasm-boids/)

| 項目 | 内容 |
| --- | --- |
| シミュレーション | C++ / WebAssembly |
| 描画 | Three.js / InstancedMesh / LOD |
| 対象 | 三次元魚群、数万匹規模 |
| 主な設計 | 局所相互作用、空間階層、近傍キャッシュ、SoA、空間順reorder |

## シミュレーション

### 基本モデル

各個体は、視野と距離の条件を満たす少数の近傍から分離・整列・凝集の影響を受ける。通常の群泳に全個体の重心や平均方向は使用しない。

1個体の操舵は、概ね次の要素から構成される。

| 要素 | 役割 |
| --- | --- |
| 分離 | 近すぎる個体から離れる |
| 整列 | 近傍の進行方向へ揃える |
| 凝集 | 近傍側へ寄る |
| 近距離反発 | 魚体寸法を基準に重なりを避ける |
| 捕食者応答 | 捕食者から逃避し、脅威状態を更新する |
| 群れへの復帰補助 | 群れから離れた個体や過度な広がりを戻す |

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

### 1ステップの処理

概ね次の順序でシミュレーションを進める。clusterはデバッグ表示だけでなく、群れへの復帰補助にも使用する。

| 順序 | 処理 |
| ---: | --- |
| 1 | `dt` を検証・制限し、シミュレーション時刻を進める |
| 2 | 現在のread bufferを参照して、近傍相互作用、捕食者応答、速度・位置・姿勢を計算する |
| 3 | 次状態をwrite bufferへ書き、read / write bufferをswapする |
| 4 | 3 frameごとに、蓄積した経過時間を使ってsmall clusterとschool clusterを更新する |
| 5 | 所定のframe間隔でleaf cacheの再収集、treeの再構築、空間順reorderを行う |
| 6 | 1 frameあたりの処理量を制限しながらtreeのsplit / mergeを進める |

この順序により、1ステップ中の全個体は同じ時点の状態を参照する。treeやreorderの保守頻度は描画負荷に応じて下がる一方、近傍寿命やcluster追跡など挙動上の時間は実際の経過時間で扱う。

## 大規模化

### 空間階層

全個体間を直接比較する場合、候補数はおおよそ

$$
N(N-1)
$$

となる。

本実装では個体群を空間的なunitへ分割し、leafを近傍探索の基本単位とする。通常は同一leaf内から近傍を補充し、不足時のみ周辺のleafを検索する。

近傍が確定した後の相互作用計算は、各個体が参照する近傍数を $k$ とすると概ね

$$
N \times k,\qquad k \leq \texttt{maxNeighbors}
$$

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

## パラメータ

UIには、魚種ごとの挙動を決める `SpeciesParams` と、全体へ作用する調整値がある。通常のパラメータ変更は実行中のWASMへ逐次反映され、個体数または捕食者フラグの変更時は群れを再初期化する。設定はブラウザの `localStorage` に保存される。

### 魚種ごとの主要項目

| 名前 | 役割 |
| --- | --- |
| `cohesion`, `cohesionRange` | 近傍側へ寄る強さと、その参照距離 |
| `separation`, `separationRange` | 近すぎる個体から離れる強さと、作用距離 |
| `alignment`, `alignmentRange` | 近傍の速度方向へ揃える強さと、その参照距離 |
| `maxSpeed` | 速度の上限 |
| `maxNeighbors` | 近傍として保持する最大数。挙動と探索量の両方に影響する |
| `maxTurnAngle` | 最大曲率。移動距離あたりの回転量を制限する |
| `torqueStrength` | 目標方向へ姿勢を合わせる反応の強さ |
| `horizontalTorque` | 上下方向の傾きを水平へ戻す強さ |
| `lambda` | 速度の減衰係数 |
| `tau` | 通常魚が同じ近傍IDを保持する時間。寄与は寿命まで徐々に減衰する |
| `predatorAlertRadius` | 捕食者を検知して逃避を始める距離 |
| `schoolPullEnabled` | その魚種へschool clusterによる復帰補助を適用するか |
| `isPredator` | 捕食者として扱うか |

### 全体調整

| 名前 | 役割 |
| --- | --- |
| `threatDecay` | 捕食者から離れた後に脅威状態が減衰する速さ |
| `maxEscapeWeight` | 通常の群泳に対して逃避をどこまで優先するか |
| `baseEscapeStrength` | 捕食者から離れる操舵の強さ |
| `schoolPullCoefficient` | 関連付けられたschool clusterへ戻す力の基準値 |
| `schoolPullStartDistance` | cluster中心から、この距離までは復帰力を掛けない |
| `schoolPullFullDistance` | 設定した復帰力へ到達する中心距離 |
| `schoolPullDenseScale` | 近傍が十分いる個体にも残す復帰力の倍率 |

ワールド座標の1 unitは1 mとして扱う。各値は独立ではなく、近傍数、作用範囲、速度、旋回能力の組み合わせによって群れの密度・形状・回転が変化する。

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
npm run serve
```

production build:

```powershell
npm run build
```

GitHub Pagesへの公開は `npm run deploy` を使用する。

利用可能なコマンドは [`docs/command_cheatsheet.md`](docs/command_cheatsheet.md) を参照。

## 開発資料

- [`docs/architecture.md`](docs/architecture.md) — 実装の責務境界
- [`docs/testing.md`](docs/testing.md) — 変更内容ごとの検証方針
- [`docs/engineering/agent_harness.md`](docs/engineering/agent_harness.md) — バグ修正と画面確認
- [`scripts/bench.md`](scripts/bench.md) — benchmarkの条件と計測項目

## 参考

本実装は以下の研究を参考にするが、論文実装の忠実な再現ではない。

| 参考 | 本実装で参照している観点 |
| --- | --- |
| [大規模な魚群シミュレーションのための階層的Boidアルゴリズム](https://ipsj.ixsq.nii.ac.jp/records/37917) | 個体群を空間的・階層的に扱い、相互作用の候補を絞る考え方 |
| [Emergence of a Giant Rotating Cluster of Fish in Three Dimensions by Local Interactions](https://doi.org/10.7566/JPSJ.91.064806) | 少数の近傍との局所相互作用から三次元の回転群が生じる考え方 |

## ライセンス

[MIT License](LICENSE)
