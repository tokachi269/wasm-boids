# wasm-boids 🐟️

本プロジェクトは、\*\*階層的Boidアルゴリズム（Hierarchical Boid Algorithm）\*\*と、**魚群の回転運動に関する生物学的モデル**を組み合わせたリアルタイムシミュレーションです。  
C++とThree.jsをWebAssembly (WASM) を介して連携し、大規模な魚群の群れ行動（整列・分離・凝集・回避・回転）をブラウザ上で再現します。


## プロジェクトの目的

-   **生物的な振る舞いを尊重**  
    必要以上に手を加えず、魚が実際に示す「視界」「回転トルク」「近傍記憶」などの局所ルールのみを使って群れ行動を構築します。
    
-   **ある程度の数のBoidを扱える構造**  
    Boidを\*\*階層的に管理する木構造（BoidTree）\*\*に分割することで、数千〜数万体規模でも処理を続けられるようにしています。  
    C++側で近傍探索と加速度計算を行い、JavaScript/Three.js側は頂点バッファを受け取って描画に集中します。
    
-   **回転群れの発生を確認**  
    生物学論文に基づく「回転トルク」「視界の非対称性」「密度スクリーニング」といった局所ルールを取り入れ、環状（リング）や球状（バイトボール）の回転構造が条件次第で現れることを確かめます。  
    群れの中心や外側を指定せず、あくまで局所ルールだけで回転構造が現れるかを試します。
    


## 主な特徴

1.  **階層的Boid木構造 (BoidTree)**
    
    -   Boidを空間的に分割し、**葉ノードでは各Boid間の詳細相互作用**、  
        **中間ノードでは代表値（中心・平均速度）による近似相互作用**という形で処理を切り替えています。  
        これにより、全体で見たときに処理量をある程度抑えつつ動作させることができます。
        
    -   BoidUnit（ノード）は k-means 風クラスタリングのように動的に分割・結合し、構造を更新します。  
        必要な範囲で詳細な情報を持たせつつ、他はまとめて扱う仕組みです。
        
2.  **生物学的に想定された回転ルール**
    
    -   魚群で観察される「視界制限（FOV）」「近傍数上限 (Nu)」「回転トルク」「慣性モーメント的効果」をBoidレベルで取り入れています。
        
    -   周囲のBoidが片側に偏るなどの非対称な配置が起こると、\*\*回転トルク（torqueStrength）**や**水平トルク（horizontalTorque）\*\*によって、その局所群方向にBoidが向きを変えます。  
        これを積み重ねると、群れ全体としてリング状・トーラス状・バイトボール状の回転構造ができる可能性があります。
        
    -   魚種ごとに想定される「吸引強度 (λ)」や「近傍記憶 (τ)」といったパラメータを変えることで、いくつかの回転パターンを試せます。
        
3.  **WebAssembly × Three.js での描画**
    
    -   C++側でBoid（魚）の位置・速度・加速度を計算し、その結果をWebAssemblyでブラウザに渡します。
        
    -   Three.js の `InstancedMesh` を使い、複数の魚モデルをまとめて描画することで、数千体を一定のフレームレートで表示可能にしています。
        
4.  **パラメータのリアルタイム調整**
    
    -   Boidの**分離 (Separation)、整列 (Alignment)、凝集 (Cohesion)、視界角度 (FOV)、最大近傍数 (Nu)、最大旋回角、トルク強度、速度範囲**などをGUIスライダーで調整できます。
        
    -   GUIで変更するとすぐにWASM側の `setSpeciesParams` が呼ばれ、次フレームから設定が反映されます。
        


## ディレクトリ構成

```text
wasm-boids/
├── src/
│   ├── wasm/                      # C++ ソース (WASM ビルド対象)
│   │   ├── boid_unit.*            # BoidUnit の挙動・近傍計算
│   │   ├── boids_tree.*           # BoidTree (階層的木構造管理)
│   │   ├── species_params.h       # 群れ行動パラメータ定義
│   │   ├── entry.*                # extern "C" バインディング用インターフェース
│   │   ├── wasm_bindings.cpp      # Emscripten バインディング定義
│   │   └── main.cpp               # WASM エントリーポイント
│   ├── main.js                    # Vue.js アプリのエントリーポイント
│   ├── App.vue                    # メインレイアウト＆GUIコントロール
│   └── components/                # Vue コンポーネント
│       └── Settings.vue           # 設定用 GUI コンポーネント
├── public/                        # 静的アセット (index.html, favicon, etc.)
├── package.json                   # npm スクリプト＆依存パッケージ定義
├── vue.config.js                  # Vue CLI 設定 (WASM 読み込み設定)
├── CMakeLists.txt                 # CMake ビルド設定
└── README.md                      # （このファイル）
```


## 必要環境とセットアップ

### 1\. Emscripten SDK

WASMビルドに必要です。

```bash
git clone https://github.com/emscripten-core/emsdk.git
cd emsdk
./emsdk install latest
./emsdk activate latest
source ./emsdk_env.sh   # Linux/macOS
# Windows (PowerShell) なら: ./emsdk_env.ps1
```

-   `emcc --version` が表示されればビルド環境は整っています。
    

### 2\. Node.js / npm

フロントエンドのビルドに使います（v16.x以上推奨）。

```bash
# https://nodejs.org/ からインストール
node -v
npm -v
```

### 3\. リポジトリをローカルにクローン

```bash
git clone https://github.com/tokachi269/wasm-boids.git
cd wasm-boids
npm install
```


## 開発モードで起動 (リアルタイム連携)

```bash
npm run serve
```

-   `serve` スクリプトの動き
    
    1.  `nodemon` が `src/wasm/` 以下を監視し、変更があると `npm run build-wasm:dev` を自動で実行。
        
    2.  `build-wasm:dev` によって `src/wasm/build/` に WASM ビルド成果物を出力。
        
    3.  Emscripten の出力完了後、`vue-cli-service serve` が Vue アプリを起動し、ブラウザ (※https://localhost:8080/ など) でプレビュー。
        
-   C++を編集すると WASM が再ビルドされ、変更はすぐブラウザに反映されます。
    
-   COOP/COEP ヘッダーが有効なため、**SharedArrayBuffer** を使った高速メモリアクセスが可能です。
    


## 本番ビルドとデプロイ

### 1\. 本番向けWASM（`-O3`）+ Vueアプリ ビルド

```bash
npm run build
```

-   `build-wasm:prod` → Emscriptenで最適化したWASMを出力します。
    
-   `vue-cli-service build` → `dist/` に静的ファイルを生成します。
    

### 2\. GitHub Pagesなどにデプロイ

```bash
npm run deploy
```

-   `dist/` を `gh-pages` ブランチへプッシュし、GitHub Pagesで公開します。
    
-   リポジトリの「Settings → Pages」で `gh-pages` ブランチを公開対象に設定してください。
    


## 主要なパラメータ（SpeciesParams）

| パラメータ名 | 説明 | デフォルト値例 |
| --- | --- | --- |
| `separation` | 分離（近すぎるBoidを遠ざける力）の強度 | 1.0 |
| `alignment` | 整列（近傍Boidの平均速度方向に合わせる力）の強度 | 1.0 |
| `cohesion` | 凝集（群れの中心に近づく力）の強度 | 1.0 |
| `separationRange` | 分離が作用する距離 (単位: メートル相当) | 50.0 |
| `cohesionRange` | 凝集・視界判定が作用する距離 | 50.0 |
| `maxNeighbors` (Nu) | 1 Boid が同時に参照する最大近傍数 (Topological Interaction Capacity) | 32 |
| `fieldOfViewDeg` | 視野角 (Degrees) | 270 |
| `maxTurnAngle` | 1フレームあたりの最大旋回角 (Radians) | 0.5 |
| `torqueStrength` | 回転トルク強度 (整列方向への回転を促進するモーメント) | 0.1 |
| `horizontalTorque` | 傾き補正トルク強度 (上下成分を抑えて水平回転を優先) | 0.05 |
| `minSpeed` | 最低速度 (群れの最小移動速度) | 1.0 |
| `maxSpeed` | 最高速度 (群れの最大移動速度) | 5.0 |
| `tau` | 近傍記憶 (cohesionMemories) の有効継続時間 (秒) | 0.5 |
| `enableLOD` | LOD (Level of Detail) を有効化するかどうか (true/false) | true |
| `enableCollisionCheck` | Boid 同士の衝突判定を行うかどうか (現状は false にしてパフォーマンス優先) | false |

> GUIスライダーでこれらを調整すると、WASM 側の `setSpeciesParams` が即座に呼ばれ、次フレームから反映されます。


## 操作方法

1.  **画面左側の GUI コントロール**
    
    -   **Boid 数**、**分離/整列/凝集/視野/近傍数上限/トルク**などをスライダーで変更できます。
        
    -   設定が変わると、WASM 側のパラメータが更新され、次フレームから反映されます。
        
2.  **画面右側の Three.js シーン**
    
    -   **マウス操作**
        
        -   左ドラッグ: 回転 (OrbitControls)
            
        -   右ドラッグ: 平行移動（パン）
            
        -   ホイール: ズーム
            
    -   **HUD/Stats**: 画面右上に FPS カウンターを表示します。
        
    -   **視覚効果**: 環境ライト・影・フォグなどを適宜設定しています。
        
3.  **シミュレーション開始**
    
    -   起動すると自動で `initBoids(N, posRange, velRange)` → `build(...)` → `update(dt)` の流れが始まります。
        
    -   Boid の初期位置・速度はランダムに配置されます。
        
    -   GUI から Boid 数やパラメータを変えると、その場で挙動に変化が現れます（再ビルドは不要です）。
        


## アルゴリズム概要（中～高レベル）

1.  **BoidTree (KD-tree に似た構造) の構築**
    
    -   N 個の Boid（魚）を空間的に分割し、階層的な木構造を作成します。
        
    -   各ノード（BoidUnit）は内部に Boid の配列を持ち、必要ならさらに子ノードを持ちます。
        
    -   `build(maxPerUnit, level)` を使って、木の深さや各ユニットが持つ Boid 数の上限を指定しながら動的に分割・結合します。
        
2.  **BoidUnit の更新 (`updateRecursive(dt)`)**
    
    -   再帰的に木をたどり、**葉ノードレベルでは各 Boid 間の詳細相互作用**（分離・整列・凝集・回転トルク・近傍記憶など）を計算します。
        
    -   **中間ノードレベルでは代表値近似**（中心・平均速度をひとつの「代表Boid」のように扱う）で、他ユニットとの相互作用を高速化しています。
        
    -   最後に、全 Boid の速度・位置を更新し、加速度をリセットします。
        
3.  **生物的回転ルールの適用**
    
    -   各 Boid が「視界内」「距離内」の近傍を調べる際、**視界角 (FOV)** と **近傍数上限 (Nu)** の条件を満たすBoidのみを相互作用対象とします。
        
    -   局所的に非対称な配置が生じると、**回転トルク (torqueStrength)** と **水平トルク (horizontalTorque)** によって Boid がわずかに捻じれ、その力を積み重ねることで群れ全体でリング状・トーラス状・バイトボール状の回転構造を形成する可能性があります。
        
4.  **パラメータが群れの性質に与える影響**
    
    -   **吸引強度 (λ)**: 回転群れのサイズや形状に影響します。小さいと球状回転が出やすく、大きいと環状回転が出やすい、といった傾向があります。
        
    -   **近傍記憶 (τ)**: Boid が一度近傍と認識した相手を一定時間「記憶」することで、一体感を向上させます。
        
    -   **速度レンジ (minSpeed, maxSpeed)**: 外側の Boid が速くなるほど群れが滑らかに回転しやすい傾向があります。
        


## 今後の拡張・改良ポイント

1.  **回転モデルの改善**
    
    -   現在の「視界制限」「密度依存トルク」「慣性モーメント」などをさらに組み合わせて調整し、より実際の魚群に近い振る舞いを探ります。
        
    -   魚種ごとに異なるパラメータセットを用意し、さまざまな回転群れが出るかを確認します。
        
2.  **近傍探索と更新の最適化**
    
    -   **Mortonコード** や **GPUパーティクル化 (WebGPU / compute shader)** の検討を通じて、数万〜十万体規模でも動かせる仕組みを探ります。
        
3.  **衝突・重なり回避の強化**
    
    -   現在、一部の Boid が重なってしまう場合があるため、**衝突判定** や **物理的なバウンディング** を導入して重なりを減らす予定です。
        
4.  **外敵・捕食者モデルの追加**
    
    -   魚群が捕食者に追われたときの **高速逃避行動（fast-start）** を組み込み、実際の群れの急反転や散開を再現してみます。
        
    -   捕食者を動的に配置し、魚群が自発的に回転シールドを作るような挙動を観察します。
        


## ライセンス

本リポジトリは **MIT License** のもとで公開されています。詳細は `LICENSE` ファイルをご確認ください。


## 参考論文・資料

-   石橋ら「大規模な魚群シミュレーションのための階層的Boidアルゴリズム」情報処理学会 CG-133 (2008)
    
-   Ito & Uchida “Emergence of a Giant Rotating Cluster of Fish in Three Dimensions by Local Interactions” (J. Phys. Soc. Jpn. 91, 064806, 2022)
    
-   その他、Boidや群れ運動に関する各種文献・生物学的データ
    


ご質問・フィードバックは Issue や Pull Request、あるいは GitHub Discussions までお気軽にどうぞ！