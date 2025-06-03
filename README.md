# wasm-boids

本プロジェクトは、**階層的Boidアルゴリズム（Hierarchical Boid Algorithm）**と、**魚群の回転運動に関する生物学的モデル**を組み合わせたリアルタイムシミュレーションです。  
C++ と Three.js を WebAssembly (WASM) を介して連携し、大規模な魚群の自然な群れ行動（整列・分離・凝集・回避・回転）をブラウザ上で描画・制御します。

---

## 🐟 プロジェクトの目的

- **生物的妥当性を重視**  
  単純なハック的実装や強制的な方向補正を避け、魚が実際に持つ「視界」「回転トルク」「近傍記憶」などの局所ルールのみで、自然な群れ行動を実現します。

- **大規模シミュレーション性能**  
  Boid を**階層的に管理する木構造（BoidTree）**を導入し、数千～数万体の魚を高速に更新。  
  WebAssembly 化した C++ 側で近傍探索と加速度計算を行い、JavaScript/Three.js 側では頂点バッファを受け取って効率的に可視化します。

- **回転群れの自然発生**  
  “回転トルク”や“視界の非対称性”“密度スクリーニング”など、生物論文に基づく局所ルールを導入し、環状（リング）や球状（バイトボール）の自発的な回転集団を生み出します。  
  これにより、目標の強制指定なしで魚群が自然に円環状・渦状に回転する挙動を再現します。

---

## ⭐ 主な特徴

1. **階層的Boid木構造 (BoidTree)**  
   - Boid を空間的に再帰分割し、**葉ノードでは個体ごとの詳細相互作用**、  
     **中間ノードでは代表値（中心・平均速度）による近似相互作用**を行うことで、O(N log N) レベルの性能を実現。  
   - BoidUnit（ノード）は k-means 風クラスタや分割・結合を通じて動的に最適化され、局所解像度を維持しつつ柔軟に構造を更新します。

2. **生物学的に裏付けられた回転ルール**  
   - 魚群が実際に示す「視界制限（FOV）」「近傍数上限 (Nu)」「回転トルク」「慣性モーメント的効果」を Boid レベルで実装。  
   - 局所的な非対称配置（片側に近傍が偏るなど）から**自然に回転力（トルク）が発生**し、群全体としてリングやボール型の渦を形成します。  
   - 魚種ごとの異なる「吸引強度 (λ)」や「近傍記憶 (τ)」などのパラメータを用いて、多様な回転パターンを再現。

3. **WebAssembly × Three.js による高速可視化**  
   - C++ 側で Boid（魚）の位置・速度・加速度計算を行い、WebAssembly でブラウザに渡すことで、**JavaScript 実行コストを最小化**。  
   - Three.js の `InstancedMesh` を使い、数千～数万体の魚モデルを効率的に描画。グラデーションや影、フォグもサポート。

4. **パラメータリアルタイム調整**  
   - Boid の **分離 (Separation)、整列 (Alignment)、凝集 (Cohesion)、視界角度 (FOV)、最大近傍数 (Nu)、最大旋回角、トルク強度、速度範囲** などを GUI スライダーで動的に変更可能。  
   - 次フレームから即時に反映されるため、群れ挙動の実験・チューニングが容易。

5. **LOD (Level of Detail) 対応**  
   - カメラ距離に応じて木構造の粗いノード代表を用いて描画・更新し、遠景の計算／描画負荷を削減。  
   - 視点に近い魚程、解像度の高い葉ノード単位で更新することで、性能と品質を両立。

---

## 📂 ディレクトリ構成

wasm-boids/ ├── src/ │   ├── wasm/                      # C++ ソース一式 (WebAssembly ビルド対象) │   │   ├── boid_unit.*            # BoidUnit の挙動・近傍計算 │   │   ├── boids_tree.*           # BoidTree (階層的木構造管理) │   │   ├── species_params.h        # 群れ行動パラメータを定義 │   │   ├── entry.*                 # extern "C" バインディング用インターフェース │   │   └── wasm_bindings.cpp       # Emscripten バインディング定義 │   ├── main.js                    # Vue.js アプリのエントリーポイント │   ├── App.vue                    # メインレイアウト＆GUI コントロール │   └── components/                # Vue コンポーネント │       └── BoidsScene.vue         # Three.js + WASM 統合ビュー ├── public/                        # 静的アセット (index.html, favicon, etc.) │   └── index.html ├── package.json                   # npm スクリプト＆依存パッケージ定義 ├── vue.config.js                  # Vue CLI 設定 (WASM 負荷軽減のためのヘッダー付与など) └── README.md                      # （このファイル）

---

## 🛠 必要環境とセットアップ

1. ### Emscripten SDK
   WASM ビルドに必要です。  
   ```bash
   git clone https://github.com/emscripten-core/emsdk.git
   cd emsdk
   ./emsdk install latest
   ./emsdk activate latest
   source ./emsdk_env.sh   # Linux/macOS
   # Windows (PowerShell) なら: ./emsdk_env.ps1

emcc --version が表示されれば OK。


2. Node.js / npm

Vue CLI ベースのフロントエンド構築に必要です（v16.x 以上推奨）。

# 公式サイト https://nodejs.org/ からインストール
node -v
npm -v


3. リポジトリをローカルにクローン

git clone https://github.com/your-username/wasm-boids.git
cd wasm-boids




---

🚀 開発モードで起動 (リアルタイム連携)

npm install
npm run serve

serve スクリプトの中で以下を並列実行します：

1. nodemon が src/wasm/ 以下を監視し、変更があると npm run build-wasm:dev（-O0 -g）を自動実行して src/wasm/build/wasm_boids.js と .wasm を生成。


2. Emscripten 出力後、vue-cli-service serve が Vue アプリを起動し、https://localhost:8080/ などでブラウザプレビュー。



ブラウザにアクセスすると、CUDABoost (SharedArrayBuffer) を有効にするためのヘッダーが自動付与されており、FPS を維持しつつ大規模群をシミュレートできます。

C++ 側を編集すると自動的に WASM が再ビルドされ、Vue アプリがホットリロードされます。



---

📦 本番ビルドとデプロイ

1. 本番向け WASM (最適化 -O3) + Vue アプリをビルド

npm run build

build-wasm:prod → Emscripten で最適化版 WASM を出力。

vue-cli-service build → dist/ ディレクトリに静的ファイルを生成。



2. GitHub Pages などにデプロイ

npm run deploy

dist/ を gh-pages ブランチへプッシュし、GitHub Pages で公開。

リポジトリの「Settings → Pages」から gh-pages ブランチを公開対象として設定してください。





---

🎛 主要なパラメータ（SpeciesParams）

パラメータ名	説明	デフォルト 値例

separation	分離（近すぎる Boid を遠ざける力）の強度	1.0
alignment	整列（近傍 Boid の平均速度方向に合わせる力）の強度	1.0
cohesion	凝集（群れの中心に近づく力）の強度	1.0
separationRange	分離が作用する距離 (単位: メートル相当)	50.0
cohesionRange	凝集・視野判定が作用する距離	50.0
maxNeighbors (Nu)	1 Boid が同時に参照する最大近傍数 (Topological Interaction Capacity)	32
fieldOfViewDeg	視野角 (Degrees)	270
maxTurnAngle	1 フレームあたりの最大旋回角 (Radians)	0.5
torqueStrength	回転トルク強度 (整列方向への回転を促進するモーメント)	0.1
horizontalTorque	傾き補正トルク強度 (上下成分を抑えて水平回転を優先)	0.05
minSpeed	最低速度 (群れの最小移動速度)	1.0
maxSpeed	最高速度 (群れの最大移動速度)	5.0
tau	近傍記憶 (cohesionMemories) の有効継続時間 (秒)	0.5


Vue の GUI スライダーで上記パラメータを変更すると、WASM 側の setSpeciesParams が即時呼び出され、次フレームから反映されます。


---

🎮 操作方法

1. 画面左側の GUI コントロール

Boid 数、パラメータ (分離・整列・凝集・視野・近傍数上限・トルクなど) をスライダーでリアルタイムに変更可能。

変更後は自動的に WASM 側のパラメータを書き換え、群れ挙動を更新します。



2. 画面右側の Three.js シーン

マウス操作:

左ドラッグ: 回転 (OrbitControls)

右ドラッグ: 平行移動（パン）

ホイール: ズーム


HUD/Stats: 画面右上に FPS カウンターを表示。

視覚効果: 環境ライト・影・フォグ設定済みで、臨場感ある魚群を表現。



3. シミュレーション開始

起動直後、自動で initBoids(N, posRange, velRange) → build(...) → update(dt) の一連フローを開始。

Boid の初期位置・速度はランダムに配置。

GUI から Boid 数やパラメータを調整すると、再度 initBoids や build を実行せずとも即座に挙動変化を確認できます。





---

🔍 アルゴリズム概要（ミドル／ハイレベル）

1. BoidTree (KD-tree 類似) の構築

N 個の Boid（魚）を、空間的に分割して階層木を構成。

各ノード (BoidUnit) は内部に Boid の配列を持ち、さらに「子ノード」を持つことが可能。

build(maxPerUnit, level) により、木の深さや各ユニットの最大 Boid 数を指定して動的に分割・結合。



2. BoidUnit の更新 (updateRecursive(dt))

再帰的に木をたどり、葉ノードレベルでは Boid 間の詳細な相互作用（分離・整列・凝集・トルク・メモリ）を計算。

中間ノードレベルでは「代表値近似」（中心・平均速度をただ一つの仮想 Boid とみなす）を使って他ユニットとの相互作用を高速近似。

最後に、全 Boid に対して速度・位置を更新し、リセットした加速度をクリア。



3. 生物的回転ルールの導入

各 Boid が「視界内」「距離内」の近傍を調べる際、視界角 (FOV) と 近傍数上限 (Nu) を同時に満たす相互作用のみを行う。

局所的な非対称配置 が生じたとき、回転トルク（torqueStrength） と 水平トルク（horizontalTorque） によって Boid はその局所群方向に捻じれながら動き、
結果として群全体で自然なリング状・トーラス状・バイトボール状の回転構造を形成する。



4. パラメータ・群特性

吸引強度 (λ): 回転群れのサイズ・形状を左右。小さめにすると球状回転、大きめにすると環状回転など。

近傍記憶 (τ): Boid が一度近傍として認識した相手を一定時間「記憶」し続け、群れの一体感を高める。

速度レンジ (minSpeed, maxSpeed): 外縁の Boid はより速く動くように設定すると、群れが安定して回転しやすい。





---

🔮 今後の拡張・改良ポイント

1. 回転モデルの洗練

現状は原論文 (Ito & Uchida, JPSJ 2022) を参考に局所トルクを実装済み。

「視界制限」「密度依存トルク」「慣性モーメント」 をさらに組み合わせ、より生物に近い回転パターンを模索予定。

魚種ごとに「回転特性」の異なるパラメータセットを用意し、バラエティ豊かな回転群れを生成。



2. 近傍探索と更新のさらなる最適化

Morton コード や パーティクル GPU 化 (WebGPU / compute shader) を検討して、大規模群 (数万～10万体) にも対応できるアーキテクチャを目指します。



3. 衝突・重なり回避の強化

現在、Boid 同士が時々重なってしまうことがあるため、衝突判定や物理的なバウンディングを追加し、重なりを防止します。



4. 外敵・捕食者モデルの組み込み

魚群が捕食者に追われた際の**高速逃避行動（fast-start）**を追加し、実際の群れの急反転・散開を再現。

捕食者を動的に配置し、魚群が自発的に回転シールドを形成する挙動を模倣。



5. 多人数同時シミュレーション(マルチプレイヤー)

Web ソケット経由で各クライアントの視点を共有し、ブラウザ間で群れの同期を実現することを検討。

教育用途やインタラクティブ展示の一部として、複数ユーザ間で群れ挙動を観察・操作できるように。





---

📄 ライセンス

本リポジトリは MIT License のもとで公開されています。詳細は LICENSE ファイルをご参照ください。


---

参考論文・資料

1. 石橋ら「大規模な魚群シミュレーションのための階層的Boidアルゴリズム」情報処理学会 CG-133 (2008)


2. Ito & Uchida “Emergence of a Giant Rotating Cluster of Fish in Three Dimensions by Local Interactions” (J. Phys. Soc. Jpn. 91, 064806, 2022)


3. その他、Boid や群れ運動に関する各種文献／生物学的データ




---

ご質問・フィードバックは Issue や Pull Request、あるいは GitHub Discussions までお気軽にどうぞ！

Happy Flocking! 🐟






[![DeepWiki](https://img.shields.io/badge/DeepWiki-tokachi269%2Fwasm--boids-blue.svg?logo=data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAACwAAAAyCAYAAAAnWDnqAAAAAXNSR0IArs4c6QAAA05JREFUaEPtmUtyEzEQhtWTQyQLHNak2AB7ZnyXZMEjXMGeK/AIi+QuHrMnbChYY7MIh8g01fJoopFb0uhhEqqcbWTp06/uv1saEDv4O3n3dV60RfP947Mm9/SQc0ICFQgzfc4CYZoTPAswgSJCCUJUnAAoRHOAUOcATwbmVLWdGoH//PB8mnKqScAhsD0kYP3j/Yt5LPQe2KvcXmGvRHcDnpxfL2zOYJ1mFwrryWTz0advv1Ut4CJgf5uhDuDj5eUcAUoahrdY/56ebRWeraTjMt/00Sh3UDtjgHtQNHwcRGOC98BJEAEymycmYcWwOprTgcB6VZ5JK5TAJ+fXGLBm3FDAmn6oPPjR4rKCAoJCal2eAiQp2x0vxTPB3ALO2CRkwmDy5WohzBDwSEFKRwPbknEggCPB/imwrycgxX2NzoMCHhPkDwqYMr9tRcP5qNrMZHkVnOjRMWwLCcr8ohBVb1OMjxLwGCvjTikrsBOiA6fNyCrm8V1rP93iVPpwaE+gO0SsWmPiXB+jikdf6SizrT5qKasx5j8ABbHpFTx+vFXp9EnYQmLx02h1QTTrl6eDqxLnGjporxl3NL3agEvXdT0WmEost648sQOYAeJS9Q7bfUVoMGnjo4AZdUMQku50McDcMWcBPvr0SzbTAFDfvJqwLzgxwATnCgnp4wDl6Aa+Ax283gghmj+vj7feE2KBBRMW3FzOpLOADl0Isb5587h/U4gGvkt5v60Z1VLG8BhYjbzRwyQZemwAd6cCR5/XFWLYZRIMpX39AR0tjaGGiGzLVyhse5C9RKC6ai42ppWPKiBagOvaYk8lO7DajerabOZP46Lby5wKjw1HCRx7p9sVMOWGzb/vA1hwiWc6jm3MvQDTogQkiqIhJV0nBQBTU+3okKCFDy9WwferkHjtxib7t3xIUQtHxnIwtx4mpg26/HfwVNVDb4oI9RHmx5WGelRVlrtiw43zboCLaxv46AZeB3IlTkwouebTr1y2NjSpHz68WNFjHvupy3q8TFn3Hos2IAk4Ju5dCo8B3wP7VPr/FGaKiG+T+v+TQqIrOqMTL1VdWV1DdmcbO8KXBz6esmYWYKPwDL5b5FA1a0hwapHiom0r/cKaoqr+27/XcrS5UwSMbQAAAABJRU5ErkJggg==)](https://deepwiki.com/tokachi269/wasm-boids)
<!-- DeepWiki badge generated by https://deepwiki.ryoppippi.com/ -->