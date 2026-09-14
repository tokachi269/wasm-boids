# wasm-boids

C++ / WebAssemblyで魚群の挙動を計算し、Three.jsで描画するブラウザ向けシミュレーションです。

単純にBoidsを大量表示するのではなく、**少数の近傍との局所相互作用から魚群らしい集団運動を作ること**と、**数万匹規模でもその挙動を崩さず動かすこと**の両方を試しています。

[デモ](https://tokachi269.github.io/wasm-boids/)

<!--
ここに後でスクリーンショットまたは短い動画を置く。
READMEを開いた直後に、魚群の密度・回転・奥行きが分かる画を1枚置く想定。
-->

| | |
| --- | --- |
| シミュレーション | C++ / WebAssembly |
| 描画 | Three.js / InstancedMesh / LOD |
| 対象 | 数万匹規模の3D魚群 |
| 主な関心 | 局所相互作用、時間方向の安定性、再現可能な検証、実測に基づく性能改善 |

## 何を作ろうとしているか

Boidsの基本は分離・整列・凝集です。この実装でもそこが中心ですが、全個体の平均方向や重心を使って群れをまとめるのではなく、各個体が実際に参照している少数の近傍から運動を作る方を重視しています。

一方、局所相互作用だけでは、いったん大きく離れた個体が群れへ戻れないことがあります。そのため群れ全体のクラスターも追跡していますが、これは通常の群れ形状を作る主役ではなく、離脱時の補助や群れ全体の推定に使います。

概念的には、1個体の加速度は次のような要素の合成です。

\[
\mathbf{a}_i =
\mathbf{S}_i +
\mathbf{A}_i +
\mathbf{C}_i +
\mathbf{R}_i +
\mathbf{E}_i +
\mathbf{P}_i
\]

| 項 | 役割 |
| --- | --- |
| \(\mathbf{S}\) | 近傍から離れる separation |
| \(\mathbf{A}\) | 近傍の進行方向へ揃える alignment |
| \(\mathbf{C}\) | 近傍側へ寄る cohesion |
| \(\mathbf{R}\) | 魚体が深く重ならないための近距離反発 |
| \(\mathbf{E}\) | 捕食者からの逃避 |
| \(\mathbf{P}\) | 群れから大きく外れた場合の弱い復帰補助 |

\(\mathbf{P}\) を強くして円形の塊を作ることはできますが、それを主な群泳規則にはしません。群れ内部の形や回転は、できるだけ \(\mathbf{S}\)、\(\mathbf{A}\)、\(\mathbf{C}\) と近傍の選び方から生じるようにしています。

### 局所で決めるもの / 群れ全体を見るもの

| 局所相互作用 | 群れ全体の推定 |
| --- | --- |
| 分離・整列・凝集 | small / school cluster |
| 視野と距離による近傍選択 | 群れ中心・半径・追跡状態 |
| 時間幅を持つ近傍記憶 | 大きく離れた個体の復帰補助 |
| 近距離反発 | デバッグ表示やカメラ基準 |
| 捕食者への直接反応 | 通常の群れ形状生成には使わない |

## 近傍は毎フレーム捨てない

単純な近傍探索では、境界をまたぐたびに相手が入れ替わり、操舵が細かく不連続になりやすくなります。

この実装では、近傍を「そのフレームで見つけた点」ではなく、短い寿命を持つ状態として扱います。

```text
保持中の近傍を検証
    ↓
まだ有効なら年齢を更新
    ↓
不足分だけ同じleafや周辺から補充
    ↓
記憶年齢に応じて寄与を減衰
    ↓
分離・整列・凝集へ使う
```

各個体が保持する近傍数には `maxNeighbors` の上限があります。少数の相手を継続して見ることで、計算量を抑えるだけでなく、群れの局所的な関係が毎フレーム完全に組み替わらないようにしています。

近傍記憶には格納位置とは別のstable IDを使います。後述する物理配列の並べ替えを行っても、「同じ個体を覚えている」という意味が変わらないようにするためです。

## 数万匹に広げる

全個体どうしを直接比較すると、相互作用の組み合わせはおおよそ

\[
N(N-1)
\]

となり、個体数を増やすほど急激に重くなります。

この実装では、力を計算する相手を少数近傍に制限します。近傍がすでに得られている部分だけを見れば、相互作用の仕事量は概ね

\[
N \times k,\qquad k \leq \texttt{maxNeighbors}
\]

です。

もちろん近傍候補を見つける処理自体は別に必要です。そこを含めて、次の構造を組み合わせています。

| 問題 | この実装で使っているもの | 狙い |
| --- | --- | --- |
| 全個体探索が重い | 空間階層 / BoidUnit | 近い個体を小さな単位へ分ける |
| 毎回近傍を探し直すと重い | packed neighbor cache | 有効な近傍を継続利用する |
| 近傍アクセスが広く飛ぶ | leaf順のspatial reorder | 空間的に近い個体を物理配列上でも近づける |
| 状態を一括で扱いたい | SoA | 計算とWASM buffer viewを単純化する |
| 完全rebuildのスパイク | rebuildの間引き + split / merge | 保守処理を一度に集中させない |
| 描画数が多い | InstancedMesh + LOD | 魚1匹ごとのdraw callを避ける |

### 空間階層

魚群を空間的なunitへ分け、leaf内で詳細な近傍処理を行います。通常は同じleafから近傍を補充し、十分な近傍が得られない場合だけ周辺の空間検索を使います。

treeを毎フレーム完全に作り直すのではなく、完全rebuildを間引きながらsplit / mergeを進めます。これはアルゴリズム上の状態更新というより、実行コストを平準化するための保守処理です。

### データ配置

シミュレーション状態はC++側のSoAに保持します。

```cpp
positions[i]
velocities[i]
orientations[i]
speciesIds[i]
stresses[i]
...
```

物理インデックスは固定しません。treeを更新した後、近い個体がメモリ上でも近くなるようleaf順へ並べ替えます。

ただし個体のidentityまで並べ替えで変えてしまうと、近傍記憶や捕食対象が壊れます。そのため、

- stable ID: 個体そのもののidentity
- physical index: 現在の配列上の位置

を分けています。

reorder時は位置・速度などのSoAだけでなく、近傍参照、捕食対象、tree側のindexも同じ対応関係で更新します。

## 時間の扱い

魚群はカオス系なので、dtが変われば軌跡まで完全一致するわけではありません。ただし、描画フレームレートが変わっただけで群れ全体が系統的に縮んだり膨らんだりする状態は避けたいと考えています。

そのため、時間に意味がある状態は経過時間を基準に扱います。

- 近傍記憶の寿命
- 脅威やストレスの減衰
- clusterの追跡と平滑化
- 捕食者の追跡 / 休止時間

一方で、tree rebuild、leaf再収集、reorderなどの保守作業は、物理現象そのものではありません。これらまで低FPS時に追いつこうとして処理回数を増やすと、負荷がさらに増えるため、挙動側の時間と保守処理の頻度は分けて扱います。

姿勢についても、単に速度ベクトルを瞬時に向けるのではなく、方向応答、最大曲率、水平化を別々に扱っています。ここは厳密な魚体力学の再現というより、速度を変えても旋回の見え方が大きく崩れないことを優先した設計です。

## 捕食者と群れの崩れ方

捕食者は単なる「逆向きのcohesion」ではなく、通常の群泳とは別の脅威として扱います。

通常魚は捕食者を検知すると逃避方向と脅威状態を持ち、脅威が高い間は通常の群泳より逃避を優先します。脅威は時間とともに減衰するため、捕食者が離れたあとも1フレームで通常状態へ戻るわけではありません。

捕食者側には追跡と休止があります。目的は捕食ゲームを作ることではなく、外乱を入れたときに魚群がどう崩れ、どう再形成されるかを見ることです。

<!--
ここに後で、
- 捕食者なし
- 捕食者接近
- 分裂
- 再形成
が分かる連続画像または短い動画を置くと、この節はかなり分かりやすくなる。
-->

## C++ / WASM / Three.js

シミュレーションと描画は分離しています。

| 層 | 主な責務 |
| --- | --- |
| C++ | 個体状態、近傍相互作用、空間階層、cluster、運動更新 |
| WebAssembly | C++の状態と処理をブラウザへ公開 |
| JavaScript | WASM memoryのview管理、描画用データの受け渡し |
| Three.js | InstancedMesh、LOD、camera、post-process |
| Fog / SSAO / Bloom | 見た目のみ。シミュレーション状態は変更しない |

位置・速度・姿勢はWASM memory上のbufferをJavaScriptからviewし、描画側へ渡します。描画都合の情報からC++の群泳規則を再判定しないことを責務境界にしています。

## 品質と性能

このプロジェクトでは、性能改善もシミュレーションの変更として扱います。

「速そうだから採用」ではなく、何を壊し得る変更なのかを先に決め、その契約に対応する証拠を取ります。

| 変更 | 主な確認 |
| --- | --- |
| 挙動を変えない最適化 | 同条件のsimulation checksum一致 |
| 物理配列のreorder | stable ID基準の状態一致 + 参照indexのvalidation |
| 相互作用や時間処理の変更 | native Releaseで挙動差を確認 |
| 性能改善 | 同一条件のA/B benchmark |
| hotspotの特定 | 区間計測 / CPU sampling / 必要に応じてPMC |
| 描画変更 | Release相当の実画面 + Console / WebGL error |
| deploy | build後に公開URLを確認 |

### 性能は区間を分けて見る

native benchmarkでは、シミュレーション全体だけでなく、たとえば次の区間を分けて計測します。

| 区間 | 内容 |
| --- | --- |
| interaction | 近傍選択と相互作用 |
| kinematics | 速度・位置・姿勢の更新 |
| tree build | 空間階層の再構築 |
| reorder | 物理配列の並べ替え |
| cluster update | small / school clusterの更新 |

browser側ではさらに、WASM simulation、JavaScript側のinstance packing、render submission、GPU timer queryを分けます。

実画面のFPSは便利な最終指標ですが、「simulationが重い」「JavaScriptが重い」「GPUが重い」を区別できません。そのため最適化の判断には、それぞれ別の計測を使います。

最近の最適化でも、近似計算やデータ配置変更を先に採用するのではなく、CPU samplingと診断カウンタでhot pathを絞り、同じ計算結果を再利用できる箇所を減らしてからA/Bしています。

測定条件とコマンドは [`scripts/bench.md`](scripts/bench.md)、変更ごとの検証基準は [`docs/testing.md`](docs/testing.md) に分けています。

## 主な調整項目

すべてのparameterをREADMEに列挙するのではなく、群れの見え方を決める主要なグループだけ示します。

| 対象 | 主なparameter |
| --- | --- |
| 基本の群泳 | `cohesion`, `separation`, `alignment` |
| 作用距離 | `cohesionRange`, `separationRange`, `alignmentRange` |
| 近傍 | `maxNeighbors`, `tau`, field of view |
| 移動 | `minSpeed`, `maxSpeed` |
| 旋回・姿勢 | `maxTurnAngle`, `torqueStrength`, `horizontalTorque` |
| 捕食者への反応 | `predatorAlertRadius`, escape / threat系 |
| 群れへの復帰補助 | school pull系 |

値は独立ではありません。たとえば `maxNeighbors` を増やすと参照する局所関係そのものが変わるため、CPU負荷だけでなく群れの形も変わります。

## 構成

| パス | 内容 |
| --- | --- |
| `src/wasm` | 個体更新、空間階層、近傍cache、cluster、WASM binding |
| `src/simulation` | WASM初期化、buffer view、simulation step |
| `src/rendering` | InstancedMesh、LOD、水中表現、post-process |
| `src/components` | 設定UI |
| `src/benchmark` | browser benchmark |
| `scripts` | build、native benchmark、deploy関連 |

責務境界は [`docs/architecture.md`](docs/architecture.md) にまとめています。

## セットアップ

必要なもの:

- Node.js / npm
- Emscripten SDK
- CMake

```powershell
npm ci
npm run build-wasm:dev
npm run serve
```

本番build:

```powershell
npm run build
```

利用可能なコマンドは [`docs/command_cheatsheet.md`](docs/command_cheatsheet.md) を参照してください。

## 開発資料

- [`docs/architecture.md`](docs/architecture.md) — 実装の責務境界
- [`docs/testing.md`](docs/testing.md) — 変更内容ごとの検証方針
- [`docs/engineering/agent_harness.md`](docs/engineering/agent_harness.md) — バグ修正と画面確認
- [`scripts/bench.md`](scripts/bench.md) — benchmarkの条件と計測項目

## 参考

大規模な魚群の扱いと、少数近傍による3Dの回転群について、次の研究を参考にしています。いずれも論文実装の忠実な再現ではありません。

| 参考 | この実装で参照している点 |
| --- | --- |
| Yoshiaki Ishibashi, Norimasa Yoshida, 「大規模な魚群シミュレーションのための階層的Boidアルゴリズム」 | 個体群を階層的に扱う考え方 |
| Susumu Ito, Nariya Uchida, “Emergence of a Giant Rotating Cluster of Fish in Three Dimensions by Local Interactions” | 少数近傍との局所相互作用から回転群が生じる考え方 |

- [大規模な魚群シミュレーションのための階層的Boidアルゴリズム](https://ipsj.ixsq.nii.ac.jp/records/37917)
- [Emergence of a Giant Rotating Cluster of Fish in Three Dimensions by Local Interactions](https://doi.org/10.7566/JPSJ.91.064806)

## ライセンス

[MIT License](LICENSE)