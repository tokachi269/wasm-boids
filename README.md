# wasm-boids

C++ / WebAssemblyで群れを計算し、Three.jsで描画する魚群シミュレーション。
階層的Boidアルゴリズムと魚群の集団運動に関する研究を参考に、数万匹規模の挙動と描画を扱う。

[デモ](https://tokachi269.github.io/wasm-boids/)

## 目的

基本的なBoidsの分離・整列・凝集に加え、次の要素を扱う。

- 空間階層を利用した近傍探索
- 時間幅を持つ近傍記憶
- 小クラスターと大クラスターの推定
- 大クラスターから外れた個体の復帰
- 捕食者に対する逃避とストレス応答
- 最大曲率、回転トルク、水平化による姿勢制御

石橋・吉田による階層的Boidアルゴリズムを、大規模な個体群を扱うための出発点とする。Ito・Uchidaによる三次元回転クラスターの研究は、少数の近傍との局所相互作用からmillingが形成されるモデルとして参照する。ただし論文実装の忠実な再現ではなく、ブラウザでの対話的な描画、捕食者応答、近傍記憶、クラスター追跡を組み合わせた独自実装である。

シミュレーション状態はC++側のSoAに保持し、WASMのbuffer viewを介してThree.jsの `InstancedMesh` へ渡す。
描画側は魚のLOD、水中フォグ、SSAO、Bloomを担当する。

## 性能設計と評価

大規模化に向けた主な実装:

- read/writeバッファによる更新中の状態分離
- 有効な近傍だけを保持するpacked neighbor cache
- treeのleaf順を利用したphysical storageの空間順reorder
- 再利用可能なscratch buffer
- `InstancedMesh` とLODによる描画負荷の制御

性能評価には、固定seed、個体数、task数を指定できるnative benchmarkとbrowser benchmarkを使用する。
native側はinteraction、kinematics、tree build、reorder、cluster updateを個別に計測する。browser側はsimulation、instance packing、render submission、GPU時間を分離する。

測定値は変更前後の比較を目的とし、ブラウザ上のフレームレートとは直接対応しない。条件と手順は [`scripts/bench.md`](scripts/bench.md) を参照。

---

## 構成

### フロントエンド（Vue + Three.js）

- `src/App.vue`: UI/レンダリング/デバッグ表示の統合
- `src/components/Settings.vue`: 種族パラメータの編集UI
- `src/rendering/*`: InstancedMesh・フォグ・粒子
- `src/simulation/WasmtimeBridge.js`: WASMバッファのビュー取得と step 呼び出し


### シミュレーション（C++ / WASM）

- `src/wasm/boids_simulation.cpp`: 空間階層（BoidSimulation）、種族エンベロープ、クラスター推定
- `src/wasm/boid_unit.cpp`: 個体更新（近傍相互作用、捕食者、旋回制限など）
- `src/wasm/species_params.h`: 種族パラメータ
- `src/wasm/simulation_tuning.*`: システム調整値（逃避など）

---

## データ設計

### SoA（Structure of Arrays）

位置・速度・姿勢・speciesIdなどは配列として保持し、キャッシュ効率とbuffer viewの単純さを優先する。

### 読み/書きバッファの分離

フレーム内で参照中の値を上書きしないようread/write bufferを分け、step完了後にswapする。
すべての個体が同じ時点の状態を参照するための構造である。

---

## 設計とアルゴリズム

主な対象は `src/wasm/boids_simulation.cpp` と `src/wasm/boid_unit.cpp`。
Boidsの基本3規則ではなく、大規模化のための近似と分割を中心に記載する。

### 1) 空間階層（BoidTree / BoidUnit）

Boidsを直接全探索すると $O(N^2)$ になるため、個体群を空間的な塊（unit）として扱い、木構造で管理する。

- 木は再帰的に分割して構築する。
- 葉（BoidUnit）はインデックス集合を持ち、葉の中で詳細計算を行う。
- 内部ノードは子の包絡球（中心/半径）などの代表量を持ち、探索や近似に使う。

更新中は木を辿りながら候補を集め、葉で詳細計算を行う。

#### BoidUnitが持つ代表量（近似の核）

葉/内部ノード（どちらも `BoidUnit`）は、少なくとも次の代表量を持つ。

- `center` / `radius`: バウンディング球。空間探索の枝刈りに使う。
- `averageVelocity`: 速度の代表値。遠方unitの影響を粗く評価する。
- `indices`: 葉に含まれる個体index。SoA bufferへ引き当てて詳細計算する。

空間queryの単位を個体ではなくunitとすることで、探索回数とmemory localityを改善する。

### 2) フレームパイプライン（概略）

概ね次の処理順で1 stepを進める。

1. `dt` の決定と取り扱い
2. 個体更新（`updateRecursive`）
   - 近傍相互作用（分離・整列・凝集）
   - 捕食者の影響、ストレス/逃避などの補助項
   - 旋回制限、水平化トルク、速度クランプ
   - 書き込みバッファへ反映
3. 読み/書きバッファの swap
4. デバッグ用集計（間引き）
   - 種族エンベロープ（中心/半径/個体数）
   - 小クラスター推定、さらに「群れクラスター」推定
5. 木構造の再構築/調整
   - 定期的に `build()` で再構築
   - さらに葉キャッシュを使って、分割/結合を少量ずつ進める

処理の対応関係を概念的な擬似コードで示す。厳密な実装の全分岐は省略する。

```text
BoidTree::update(dt):
  (定期) 種族エンベロープ更新

  if root:
    個体更新（木を辿って葉で詳細計算）
    dt > 0 なら read/write を swap

  (定期) 小クラスター更新
  (定期) 群れクラスター更新

  (定期) 葉一覧（キャッシュ）を再収集
  (定期) 木を再構築

  (毎フレーム少量) split/merge を進める
```

重い集計（envelope/cluster/rebuild）は、毎frame必須の個体更新から分離して間引く。処理時間のspikeを抑えながら、空間構造を追従させるためである。

#### split/merge を“少量ずつ”進める理由

木の完全rebuildは間引き、その合間に局所的なsplit/mergeを少量ずつ進める。

- 定期的に葉一覧（leafCache）を収集し、分割/結合候補を順に処理する
- 1 frameで処理する候補数を制限し、処理時間のspikeを避ける

#### 分割判定/結合判定の目安

分割/結合は `BoidUnit::needsSplit()` / `BoidUnit::canMergeWith()` を基準とする。

- split: ユニット半径が大きい、または向き/密度のばらつきが大きい、かつユニット内個体数が多い
- merge: ユニット同士が近い、速度が十分揃っている、半径が過大でない、かつ合算個体数が上限以内

異なる `speciesId` のunitは結合しない。

### 3) 近傍探索（球交差クエリ + 葉内詳細）

葉の中では `indices` を使って個体同士の詳細相互作用を計算する。
影響範囲に入る別の葉は、木のbounding sphereで枝刈りしながら走査する。

`BoidTree` は球交差queryを提供する。

- `forEachLeafIntersectingSphere(...)`: 交差する葉を列挙
- `forEachLeafIntersectingSphereCancelable(...)`: 途中で打ち切れる版（必要数が集まれば十分な用途向け）

Cancelable版は、必要数が揃った時点で探索を止める。

#### 葉内の個体更新（BoidUnit側の概要）

葉では、`indices` で参照できる個体群について、分離・整列・凝集のsteeringを合成して加速度または目標速度を作る。
近傍として参照する上限は `maxNeighbors` で制限する。これは挙動だけでなく探索costにも影響する。

主に次の項目を適用する。

- 近傍相互作用（分離・整列・凝集）: 各レンジ（`*Range`）内の近傍を集計し、強度（`cohesion/alignment/separation`）で重み付け
- 捕食者影響: 非捕食者は捕食者を検知すると回避方向を加算（警戒距離は種族ごと）
- 姿勢の追従: `torqueStrength` による方向合わせ、`horizontalTorque` による水平化
- 旋回と速度の上限: `maxTurnAngle`（曲率）による旋回クランプ、`minSpeed/maxSpeed` による速度クランプ

捕食者の警戒距離は、種族parameter更新時に探索用の値を前計算してcacheする。

### 4) クラスター推定（species clusters / school clusters）

clusterは個体全数ではなく、leaf（BoidUnit）単位で近似集計する。

- 小クラスター: leaf を素材に中心・半径・速度整列などを推定
- 大クラスター（群れ）: 小クラスター同士のリンク（距離閾値）を辿り、群れ中心を推定
- 時間方向はEMA（指数移動平均）で平滑化する

推定結果は、大クラスターへの復帰力、debug表示、起動直後のcamera注視点に利用する。

#### 小クラスターの半径推定が“中心のズレ”に強い理由

小クラスターはleafを寄せ集めた近似である。中心からの最大距離だけで半径を作ると、中心のわずかなずれによって半径が大きく変動する。

そこで実装では、位置の二乗和（`E[x^2]`）も保持し、

$$\mathrm{Var}(x) = E[x^2] - (E[x])^2$$

の形でRMS半径を求め、中心の変動に対する広がりの推定を安定させる。
さらにleaf自体の半径を加算し、leaf単位近似による過小評価を抑える。

#### 群れ（大クラスター）の作り方

群れは小クラスター同士をlinkし、連結成分としてまとめる。
リンク判定は

```
dist <= linkScale * (r_i + r_j)
```

のように、半径の和を基準に密集を判定する。
中心はEMAで追跡し、frameごとの対応付けが多少ずれても注視点が飛ばないようにする。

### 5) 旋回制限（maxTurnAngle の意味）

`maxTurnAngle` は角速度（/sec）ではなく、最大曲率（移動距離あたりの回転量）として扱う。
角速度上限では、速度の変更に伴って同じ設定値でも曲率が変わるためである。

1 stepの旋回上限角は概ね次で決まる。

```
maxTurnStep ≈ clamp(maxTurnAngle * speed * dt, 0, stepLimit)
```

これにより、速度を上げても曲がりやすさを維持する。

補足:
この設計は視覚的な旋回の滑らかさと安定性を優先する。厳密な運動モデルの再現ではない。

---

## パラメータ

画面の種族パネルは、種族ごとの運動則を変更する。値は相互に作用するため、強度だけでなく範囲、旋回能力、近傍数を合わせて調整する必要がある。

| パラメータ | 作用 | 値を上げた場合 |
| --- | --- | --- |
| `cohesion` | 参照中の近傍中心へ向かう強さ | 群れがまとまりやすくなる。過大な値では中心への収縮が強くなる |
| `cohesionRange` | 凝集の参照距離 | 離れた近傍にも凝集が働く |
| `separation` | 近すぎる個体から離れる強さ | 個体間隔が広がり、衝突を避けやすくなる |
| `separationRange` | 分離が働く距離 | より早い段階から距離を取る |
| `alignment` | 近傍の進行方向へ揃える強さ | 流れが揃いやすくなる。過大な値では局所的な変形が減る |
| `alignmentRange` | 整列の参照距離 | より離れた近傍の向きも参照する |
| `maxSpeed` | 移動速度の上限 | 群れの移動速度域が上がる |
| `maxNeighbors` | 記憶・参照する近傍数の上限（0–32） | 多くの個体を参照する一方、interactionの計算量も増える |
| `maxTurnAngle` | 移動距離あたりの最大曲率 | 小さい半径で旋回できる |
| `torqueStrength` | 目標方向へ姿勢を追従させる強さ | 方向転換への応答が速くなる |
| `horizontalTorque` | 上下の傾きを水平へ戻す強さ | 水平姿勢へ戻りやすくなり、上下方向の動きが抑えられる |
| `lambda` | 速度の減衰係数 | 慣性が弱まり、速度が落ちやすくなる |
| `tau` | 近傍identityを保持する時間 | 近傍の入れ替わりが遅くなり、反応が滑らかになる |
| `predatorAlertRadius` | 捕食者を検知する距離 | より遠い段階から逃避を始める |
| `schoolPullEnabled` | 大クラスターへの復帰力を種族に適用するか | ONでは後述の大クラスター引力が働く |
| `densityReturnStrength` | UIとbindingに残る旧設定 | 現行の相互作用計算では参照されず、値を変えても挙動は変化しない |
| `isPredator` | 種族を捕食者として扱う | 他種の逃避対象になり、捕食者用の更新則が働く |

画面のAdjustmentパネルは、全種族に共通する逃避と大クラスター復帰を変更する。

| パラメータ | 作用 | 値を上げた場合 |
| --- | --- | --- |
| `threatDecay` | 脅威状態の減衰速度（1/sec） | 捕食者から離れた後、通常状態へ早く戻る |
| `maxEscapeWeight` | 操舵に占める逃避方向の最大割合（0–1） | 危険時に通常の群泳より逃避を優先する |
| `baseEscapeStrength` | 逃避方向の目標速度へ寄せる強さ | 捕食者への反応が強くなる |
| `schoolPullCoefficient` | 大クラスター中心へ戻す力の基礎係数 | 離脱個体が戻りやすくなる。過大な値では群れが正円に近づきやすい |
| `schoolPullStartDistance` | 中心から引力が働き始める距離 | 中心付近で引力を受けない領域が広がる |
| `schoolPullFullDistance` | 引力が最大になる中心距離 | startから最大値までの立ち上がりが緩やかになる |
| `schoolPullDenseScale` | 近傍が十分いる個体に残す引力倍率（0–1） | 密集した個体にも中心引力が残る。小さくすると形成済みの群れの局所運動を妨げにくい |

### 調整の目安

1. `cohesion`、`separation`、`alignment` と各rangeで局所的な群泳を作る。
2. `maxTurnAngle` と `torqueStrength` で旋回半径と方向応答を合わせる。急な反転や振動が出る場合は、まずこの2値を確認する。
3. `horizontalTorque` で上下方向の広がりを調整する。
4. `schoolPullCoefficient` と距離gateは、群れの形を作る主力ではなく、離脱や過度な膨張を戻す範囲に留める。
5. `maxNeighbors` は挙動とCPU負荷の両方を変えるため、最後に増減する。

ワールド座標の1単位は1 mとして扱う。range、引力距離、魚体寸法は同じ尺度で指定する。

---

## 実行時の挙動メモ

- 起動直後、camera操作前はcluster中心へ注視点を滑らかに合わせる。
- tabが非アクティブ（非表示/非focus）の間は背景音をmuteする。

---

## セットアップ / ビルド

### 必要環境

- Node.js / npm
- Emscripten SDK（WASMビルド）
- CMake（WASM/ネイティブどちらにも使用）

### 依存関係

```bash
npm ci
```

### WASMビルド

```bash
npm run build-wasm:dev
```

リポジトリ内のCMake buildには `build-dev/` を使う。

### 開発起動

```bash
npm run serve
```

`serve` はWASMの再build（watch）とVue dev serverを並列起動する。

### 本番ビルド

```bash
npm run build
```

---

## 開発資料

- [`docs/architecture.md`](docs/architecture.md): 実装の責務分担
- [`docs/testing.md`](docs/testing.md): 変更内容ごとの確認方法
- [`docs/engineering/agent_harness.md`](docs/engineering/agent_harness.md): バグ修正と画面確認の手順
- [`docs/command_cheatsheet.md`](docs/command_cheatsheet.md): build、benchmark、deployのコマンド
- [`scripts/bench.md`](scripts/bench.md): benchmarkの条件と読み方

---

## ライセンス

[MIT License](LICENSE)

---

## 参考

- Yoshiaki Ishibashi and Norimasa Yoshida, [「大規模な魚群シミュレーションのための階層的Boidアルゴリズム」](https://ipsj.ixsq.nii.ac.jp/records/37917), 情報処理学会研究報告 2008-CG-133 (2008)
- Susumu Ito and Nariya Uchida, [“Emergence of a Giant Rotating Cluster of Fish in Three Dimensions by Local Interactions”](https://doi.org/10.7566/JPSJ.91.064806), J. Phys. Soc. Jpn. 91, 064806 (2022)
