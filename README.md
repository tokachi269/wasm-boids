[![Ask DeepWiki](https://deepwiki.com/badge.svg)](https://deepwiki.com/tokachi269/wasm-boids)

[Demo](https://tokachi269.github.io/wasm-boids/)


# wasm-boids 🐟️  

本プロジェクトは、**階層的Boidアルゴリズム（Hierarchical Boid Algorithm）**と、**魚群の回転運動に関する生物学的モデル**を組み合わせたリアルタイムシミュレーションです。  
C++とThree.jsをWebAssembly (WASM) を介して連携し、大規模な魚群の群れ行動（整列・分離・凝集・回避・回転）をブラウザ上で再現します。

---

## プロジェクトの目的

- **生物的な振る舞いを尊重**  
  魚の実際の「視界」「回転トルク」「近傍記憶」などの局所ルールを使用して、群れ行動を構築します。

- **大規模シミュレーション対応**  
  Boidを階層的に管理する木構造（BoidTree）を使用し、数千〜数万体規模の処理を可能にします。C++で計算し、Three.jsで描画に集中します。

- **回転群れの発生を確認**  
  「回転トルク」「視界の非対称性」「密度スクリーニング」を取り入れ、群れがどのように回転構造を形成するかを確認します。

---

## 主な特徴

- **階層的Boid木構造 (BoidTree)**  
  Boidを空間的に分割し、葉ノードでは詳細な相互作用、中間ノードでは代表値を使用して処理を最適化します。これにより、パフォーマンスを維持しつつ大量のBoidを扱えます。

- **回転ルールの適用**  
  魚群で観察される回転トルクや視界制限をBoid単位で適用し、非対称な配置から自然な回転構造が現れるようにします。魚種ごとに異なる回転パターンを設定可能です。

- **WebAssembly × Three.jsでの描画**  
  C++側で位置・速度・加速度を計算し、WebAssemblyを通じて効率的にブラウザに渡します。Three.jsの`InstancedMesh`を使用して、数千体の魚を描画します。

- **パラメータのリアルタイム調整**  
  GUIスライダーで分離・整列・凝集・視界角度などのパラメータを調整可能。変更は即座にWASMに反映されます。

---

## ディレクトリ構成

```text
wasm-boids/
├── src/
│   ├── wasm/                      # C++ ソース (WASM ビルド対象)
│   │   ├── boid_unit.*            # BoidUnit の挙動・近傍計算
│   │   ├── boids_tree.*           # BoidTree (階層的木構造管理)
│   │   ├── species_params.h       # 群れ行動パラメータ定義
│   │   ├── entry.*                # extern "C" バインディング用インターフェース
│   │   └── wasm_bindings.cpp      # Emscripten バインディング定義
│   ├── main.js                    # Vue.js アプリのエントリーポイント
│   ├── App.vue                    # メインレイアウト＆GUIコントロール
│   └── components/                # Vue コンポーネント
│       └── Settings.vue           # 設定用 GUI コンポーネント
├── public/                        # 静的アセット (index.html, favicon, etc.)
├── package.json                   # npm スクリプト＆依存パッケージ定義
├── vue.config.js                  # Vue CLI 設定 (WASM 読み込み設定)
└── CMakeLists.txt                 # CMake ビルド設定
```

---

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

`emcc --version` が表示されればビルド環境は整っています。

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

---

## 開発モードで起動

```bash
npm run serve
```

-   `serve` スクリプトで自動ビルドが実行され、即座にブラウザで確認できます。
    

---

## 主要なパラメータ（SpeciesParams）

| パラメータ名 | 説明 | デフォルト値例 |
| --- | --- | --- |
| `separation` | 分離（近すぎるBoidを遠ざける力）の強度 | 1.0 |
| `alignment` | 整列（近傍Boidの平均速度方向に合わせる力）の強度 | 1.0 |
| `cohesion` | 凝集（群れの中心に近づく力）の強度 | 1.0 |
| `separationRange` | 分離が作用する距離 (メートル相当) | 50.0 |
| `cohesionRange` | 凝集・視界判定が作用する距離 | 50.0 |
| `maxNeighbors` (Nu) | 1 Boid が参照する最大近傍数 | 32 |
| `fieldOfViewDeg` | 視野角 (Degrees) | 270 |
| `maxTurnAngle` | 1フレームの最大旋回角 (Radians) | 0.5 |
| `torqueStrength` | 回転トルク強度 | 0.1 |
| `horizontalTorque` | 傾き補正トルク強度 | 0.05 |
| `minSpeed` | 最低速度 | 1.0 |
| `maxSpeed` | 最高速度 | 5.0 |
| `tau` | 近傍記憶の有効時間 (秒) | 0.5 |

---

## 操作方法

1.  **画面左側の GUI コントロール**
    
    -   **Boid数**や**分離/整列/凝集**などをスライダーで調整。
        
    -   設定変更後、即座にWASM側で反映されます。
        
2.  **画面右側の Three.js シーン**
    
    -   **マウス操作**
        
        -   左ドラッグ: 回転
            
        -   右ドラッグ: 平行移動（パン）
            
        -   ホイール: ズーム
            
    -   **HUD/Stats**: 画面右上にFPSカウンター表示
        

---

## アルゴリズム概要

1.  **BoidTree構築**  
    空間を再帰分割し、木構造を構築。葉ノードでは詳細な計算、中間ノードでは代表値で近似。
    
2.  **BoidUnit更新**  
    再帰的に木をたどり、葉ノードで詳細相互作用を計算し、中間ノードで近似計算。
    
3.  **回転ルールの適用**  
    非対称配置が生じた際、トルクによって群れ全体で回転構造を形成。
    

---

## 今後の予定

1.  回転モデルの改善
    
2.  近傍探索の最適化（WebGPU/compute shader）
    
3.  衝突・重なり回避の強化
    
4.  捕食者モデルの追加
    

---

## ライセンス

MIT License

---

## 参考文献

-   石橋ら「大規模な魚群シミュレーションのための階層的Boidアルゴリズム」情報処理学会 CG-133 (2008)
    
-   Ito & Uchida, *J. Phys. Soc. Jpn.*, **91**, 064806 (2022)
    
