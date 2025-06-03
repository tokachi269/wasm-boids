# wasm-boids

本プロジェクトは、\*\*階層的Boidアルゴリズム（Hierarchical Boid Algorithm）\*\*と、**魚群の回転運動に関する生物学的モデル**を組み合わせたリアルタイムシミュレーションです。  
C++とThree.jsをWebAssembly (WASM) を介して連携し、大規模な魚群の自然な群れ行動（整列・分離・凝集・回避・回転）をブラウザ上で描画・制御します。

---

## 🐟 プロジェクトの目的

-   **生物的妥当性を重視**  
    単純なハック的実装や強制的な方向補正を避け、魚が実際に持つ「視界」「回転トルク」「近傍記憶」などの局所ルールのみで、自然な群れ行動を実現します。
    
-   **大規模シミュレーション性能**  
    Boidを\*\*階層的に管理する木構造（BoidTree）\*\*を導入し、数千～数万体の魚を高速に更新。  
    WebAssembly化したC++側で近傍探索と加速度計算を行い、JavaScript/Three.js側では頂点バッファを受け取って効率的に可視化します。
    
-   **回転群れの自然発生**  
    “回転トルク”や“視界の非対称性”“密度スクリーニング”など、生物論文に基づく局所ルールを導入し、環状（リング）や球状（バイトボール）の自発的な回転集団を生み出します。  
    これにより、目標の強制指定なしで魚群が自然に円環状・渦状に回転する挙動を再現します。
    

---

## ⭐ 主な特徴

1.  **階層的Boid木構造 (BoidTree)**
    
    -   Boidを空間的に再帰分割し、**葉ノードでは個体ごとの詳細相互作用**、  
        **中間ノードでは代表値（中心・平均速度）による近似相互作用**を行うことで、O(N log N)レベルの性能を実現。
        
    -   BoidUnit（ノード）はk-means風クラスタや分割・結合を通じて動的に最適化され、局所解像度を維持しつつ柔軟に構造を更新します。
        
2.  **生物学的に裏付けられた回転ルール**
    
    -   魚群が実際に示す「視界制限（FOV）」「近傍数上限 (Nu)」「回転トルク」「慣性モーメント的効果」をBoidレベルで実装。
        
    -   局所的な非対称配置（片側に近傍が偏るなど）が生じたとき、\*\*回転トルク（torqueStrength）**と**水平トルク（horizontalTorque）\*\*によってBoidはその局所群方向に捻じれながら動き、  
        結果として群全体で自然なリング状・トーラス状・バイトボール状の回転構造を形成します。
        
    -   魚種ごとの異なる「吸引強度 (λ)」や「近傍記憶 (τ)」などのパラメータを用いて、多様な回転パターンを再現。
        
3.  **WebAssembly × Three.jsによる高速可視化**
    
    -   C++側でBoid（魚）の位置・速度・加速度計算を行い、WebAssemblyでブラウザに渡すことで、**JavaScript実行コストを最小化**。
        
    -   Three.jsの`InstancedMesh`を使い、数千～数万体の魚モデルを効率的に描画。グラデーションや影、フォグもサポート。
        
4.  **パラメータリアルタイム調整**
    
    -   Boidの**分離 (Separation)、整列 (Alignment)、凝集 (Cohesion)、視界角度 (FOV)、最大近傍数 (Nu)、最大旋回角、トルク強度、速度範囲**などをGUIスライダーで動的に変更可能。
        
    -   変更後は自動的にWASM側の`setSpeciesParams`が呼び出され、次フレームから反映。
        
5.  **LOD (Level of Detail)対応**
    
    -   カメラ距離に応じて木構造の粗いノード代表を用いて描画・更新し、遠景の計算／描画負荷を削減。
        
    -   視点に近い魚ほど解像度の高い葉ノード単位で更新することで、性能と品質を両立。
        

---

## 📂 ディレクトリ構成

```csharp
wasm-boids/
├── src/
│   ├── wasm/                      # C++ソース一式 (WebAssemblyビルド対象)
│   │   ├── boid_unit.*            # BoidUnitの挙動・近傍計算
│   │   ├── boids_tree.*           # BoidTree (階層的木構造管理)
│   │   ├── species_params.h        # 群れ行動パラメータ定義
│   │   ├── entry.*                 # extern "C" バインディング用インターフェース
│   │   └── wasm_bindings.cpp       # Emscriptenバインディング定義
│   ├── main.js                    # Vue.jsアプリのエントリーポイント
│   ├── App.vue                    # メインレイアウト＆GUIコントロール
│   └── components/                # Vueコンポーネント
│       └── BoidsScene.vue         # Three.js + WASM統合ビュー
├── public/                        # 静的アセット (index.html, favicon, etc.)
│   └── index.html
├── package.json                   # npmスクリプト＆依存パッケージ定義
├── vue.config.js                  # Vue CLI設定 (WASM読み込み最適化)
└── README.md                      # （このファイル）
```

---

## 🛠 必要環境とセットアップ

1.  ### Emscripten SDK
    
    WASMビルドに必要です。
    
    ```bash
    git clone https://github.com/emscripten-core/emsdk.git
    cd emsdk
    ./emsdk install latest
    ./emsdk activate latest
    source ./emsdk_env.sh   # Linux/macOS
    # Windows (PowerShell) なら: ./emsdk_env.ps1
    ```
    
    -   `emcc --version`が表示されればOKです。
        
2.  ### Node.js / npm
    
    Vue CLIベースのフロントエンド構築に必要です（v16.x以上推奨）。
    
    ```bash
    # https://nodejs.org/ からインストール
    node -v
    npm -v
    ```
    
3.  ### リポジトリをローカルにクローン
    
    ```bash
    git clone https://github.com/your-username/wasm-boids.git
    cd wasm-boids
    npm install
    ```
    

---

## 🚀 開発モードで起動 (リアルタイム連携)

```bash
npm run serve
```

-   `serve`スクリプト内で並列実行:
    
    1.  `nodemon` が `src/wasm/` 以下を監視し、変更があると `npm run build-wasm:dev`（`-O0 -g`）を自動実行して `src/wasm/build/wasm_boids.js` と `.wasm` を生成。
        
    2.  Emscripten出力後、`vue-cli-service serve` が Vueアプリを起動し、`https://localhost:8080/`などでブラウザプレビュー。
        
-   C++を編集すると自動でWASMが再ビルドされ、Vue側に直ちに反映されるため、迅速に挙動を確認できます。
    
-   ブラウザはCOOP/COEPヘッダー付きのため、**SharedArrayBuffer**による高速メモリアクセスが可能です。
    

---

## 📦 本番ビルドとデプロイ

1.  **本番向けWASM（`-O3`）+ Vueアプリ ビルド**
    
    ```bash
    npm run build
    ```
    
    -   `build-wasm:prod` → Emscriptenで最適化版WASMを出力。
        
    -   `vue-cli-service build` → `dist/`ディレクトリに静的ファイルを生成。
        
2.  **GitHub Pagesなどにデプロイ**
    
    ```bash
    npm run deploy
    ```
    
    -   `dist/`を`gh-pages`ブランチへプッシュし、GitHub Pagesで公開します。
        
    -   リポジトリの「Settings → Pages」で`gh-pages`ブランチを公開対象に設定してください。
        

---

## 🎛 主要なパラメータ（SpeciesParams）

| パラメータ名 | 説明 | デフォルト 値例 |
| --- | --- | --- |
| `separation` | 分離（近すぎるBoidを遠ざける力）の強度 | 1.0 |
| `alignment` | 整列（近傍Boidの平均速度方向に合わせる力）の強度 | 1.0 |
| `cohesion` | 凝集（群れの中心に近づく力）の強度 | 1.0 |
| `separationRange` | 分離が作用する距離 (単位: メートル相当) | 50.0 |
| `cohesionRange` | 凝集・視界判定が作用する距離 | 50.0 |
| `maxNeighbors` (Nu) | 1 Boidが同時に参照する最大近傍数 (Topological Interaction Capacity) | 32 |
| `fieldOfViewDeg` | 視野角 (Degrees) | 270 |
| `maxTurnAngle` | 1フレームあたりの最大旋回角 (Radians) | 0.5 |
| `torqueStrength` | 回転トルク強度 (整列方向への回転を促進するモーメント) | 0.1 |
| `horizontalTorque` | 傾き補正トルク強度 (上下成分を抑えて水平回転を優先) | 0.05 |
| `minSpeed` | 最低速度 (群れの最小移動速度) | 1.0 |
| `maxSpeed` | 最高速度 (群れの最大移動速度) | 5.0 |
| `tau` | 近傍記憶 (cohesionMemories) の有効継続時間 (秒) | 0.5 |

> VueのGUIスライダーで上記パラメータを調整すると、WASM側の`setSpeciesParams`が即時呼び出され、次フレームから反映されます。

---

## 🎮 操作方法

1.  **画面左側のGUIコントロール**
    
    -   **Boid数**、**分離/整列/凝集/視野/近傍数上限/トルク**などをスライダーでリアルタイムに変更可能。
        
    -   変更後は自動的にWASM側のパラメータを書き換え、群れ挙動を更新します。
        
2.  **画面右側のThree.jsシーン**
    
    -   **マウス操作**
        
        -   左ドラッグ: 回転 (OrbitControls)
            
        -   右ドラッグ: 平行移動（パン）
            
        -   ホイール: ズーム
            
    -   **HUD/Stats**: 画面右上にFPSカウンターを表示。
        
    -   **視覚効果**: 環境ライト・影・フォグ設定済みで、臨場感ある魚群を表現。
        
3.  **シミュレーション開始**
    
    -   起動直後、自動で`initBoids(N, posRange, velRange)` → `build(...)` → `update(dt)`の一連フローを開始。
        
    -   Boidの初期位置・速度はランダムに配置。
        
    -   GUIからBoid数やパラメータを調整すると、再度`initBoids`や`build`を実行せずとも即座に挙動変化を確認できます。
        

---

## 🔍 アルゴリズム概要（ミドル／ハイレベル）

1.  **BoidTree (KD-tree類似) の構築**
    
    -   N個のBoid（魚）を、空間的に分割して階層木を構成。
        
    -   各ノード (BoidUnit) は内部にBoidの配列を持ち、さらに「子ノード」を持つことが可能。
        
    -   `build(maxPerUnit, level)` により、木の深さや各ユニットの最大Boid数を指定して動的に分割・結合。
        
2.  **BoidUnit の更新 (`updateRecursive(dt)`)**
    
    -   再帰的に木をたどり、**葉ノードレベルではBoid間の詳細な相互作用**（分離・整列・凝集・トルク・メモリ）を計算。
        
    -   **中間ノードレベルでは「代表値近似」**（中心・平均速度をただ一つの仮想Boidとみなす）を使って他ユニットとの相互作用を高速近似。
        
    -   最後に、全Boidに対して速度・位置を更新し、リセットした加速度をクリア。
        
3.  **生物的回転ルールの導入**
    
    -   各Boidが「視界内」「距離内」の近傍を調べる際、**視界角 (FOV)** と **近傍数上限 (Nu)** を同時に満たす相互作用のみを行う。
        
    -   局所的な非対称配置が生じたとき、**回転トルク（torqueStrength）** と **水平トルク（horizontalTorque）** によってBoidはその局所群方向に捻じれながら動き、  
        結果として群全体で自然なリング状・トーラス状・バイトボール状の回転構造を形成する。
        
4.  **パラメータ・群特性**
    
    -   **吸引強度 (λ)**: 回転群れのサイズ・形状を左右。小さめにすると球状回転、大きめにすると環状回転など。
        
    -   **近傍記憶 (τ)**: Boidが一度近傍として認識した相手を一定時間「記憶」し続け、群れの一体感を高める。
        
    -   **速度レンジ (minSpeed, maxSpeed)**: 外縁のBoidはより速く動くように設定すると、群れが安定して回転しやすい。
        

---

## 🔮 今後の拡張・改良ポイント

1.  **回転モデルの洗練**
    
    -   原論文 (Ito & Uchida, JPSJ 2022) を参考に局所トルクを実装済み。
        
    -   **「視界制限」「密度依存トルク」「慣性モーメント」** をさらに組み合わせ、より生物に近い回転パターンを模索予定。
        
    -   魚種ごとに「回転特性」の異なるパラメータセットを用意し、バラエティ豊かな回転群れを生成。
        
2.  **近傍探索と更新のさらなる最適化**
    
    -   **Mortonコード**や\*\*GPUパーティクル化 (WebGPU/compute shader)\*\*を検討して、大規模群 (数万～10万体) にも対応できるアーキテクチャを目指します。
        
3.  **衝突・重なり回避の強化**
    
    -   現在、Boid同士が時々重なってしまう問題があるため、**衝突判定**や**物理的バウンディング**を追加して重なりを防止します。
        
4.  **外敵・捕食者モデルの組み込み**
    
    -   魚群が捕食者に追われた際の\*\*高速逃避行動（fast-start）\*\*を追加し、実際の群れの急反転・散開を再現。
        
    -   捕食者を動的に配置し、魚群が自発的に回転シールドを形成する挙動を模倣。
        
5.  **多人数同時シミュレーション (マルチプレイヤー)**
    
    -   Webソケット経由で各クライアントの視点を共有し、**ブラウザ間で群れの同期**を実現することを検討。
        
    -   教育用途やインタラクティブ展示として、複数ユーザ間で群れ挙動を観察・操作できるように。
        

---

## 📄 ライセンス

本リポジトリは **MIT License** のもとで公開されています。詳細は `LICENSE` ファイルをご参照ください。

---

## 参考論文・資料

-   石橋ら「大規模な魚群シミュレーションのための階層的Boidアルゴリズム」情報処理学会 CG-133 (2008)
    
-   Ito & Uchida “Emergence of a Giant Rotating Cluster of Fish in Three Dimensions by Local Interactions” (J. Phys. Soc. Jpn. 91, 064806, 2022)
    
-   その他、Boidや群れ運動に関する各種文献／生物学的データ
    

---

ご質問・フィードバックは Issue や Pull Request、あるいは GitHub Discussions までお気軽にどうぞ！

Happy Flocking! 🐟