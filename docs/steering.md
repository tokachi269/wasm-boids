# Guide / Obstacle steering

GuideとObstacleは、個体の現在位置と速度から追加steeringを計算します。近傍探索や描画には依存しません。

## Guide

`freeRadius`内では力を加えません。外側では、境界からの距離が`responseDistance`に達するまでsmoothstepで`strength`へ立ち上がります。遠距離でも力を無制限に増やさないため、群れ全体を強く圧縮する中心力にはなりません。

```cpp
boids::Guide guide;
guide.target = {10.0f, 4.0f, -2.0f};
guide.freeRadius = 12.0f;
guide.responseDistance = 20.0f;
guide.strength = 0.02f;
world.setGuides({guide});
```

`setGuides()`を再度呼ぶことで、映像側の目標などへtargetを追従させられます。

## Obstacle

対応形状はPlane、Sphere、Capsule、Boxです。全形状が表面までのsigned distanceと外向きnormalを返し、`influenceDistance`内だけ回避steeringを加えます。

| shape | `size` | transform |
|---|---|---|
| Plane | 未使用 | local +Yが許可側normal、`position`が面上の点 |
| Sphere | `x`: radius | `position`が中心 |
| Capsule | `x`: radius、`y`: local Y方向のhalf length | `position`, `rotation` |
| Box | xyz: half extents | `position`, `rotation` |

`strength`は法線方向の事前回避、`damping`は面へ向かう速度成分の抑制、`tangentStrength`は接線方向へ流す強さです。正面衝突で接線方向を決められない場合は、stable IDとObstacle IDから左右を決め、影響範囲を抜けるまで個体ごとに保持します。そのためObstacle IDは一意にし、transform更新中も変えないでください。

penetration correctionは通常steeringの後に残り、面を越えた個体だけを表面外へ戻します。

各個体は有効なGuide/Obstacleを直接評価するため、コストは個体数と登録数に比例します。現段階では少数の大きな誘導領域・障害物を対象とし、FieldやNavMeshの代替にはしません。

## JavaScript / WASM

`WasmtimeBridge`では配列単位で更新します。

```js
bridge.setGuides([{ target: [10, 4, -2], freeRadius: 12, responseDistance: 20, strength: 0.02 }]);

bridge.setObstacles([{
  id: 1,
  shape: 'sphere',
  position: [0, 3, 0],
  size: [2, 1, 1],
  influenceDistance: 4,
  strength: 8,
  damping: 4,
  tangentStrength: 3,
}]);
```

Bridgeは配列をWASM共有入力バッファへまとめて書き、Guide/Obstacleごとの個別呼び出しを行いません。transformを動かす場合も、更新する配列ごとに1回のcommitです。

既存のGroundPlaneは水平Plane型Obstacleと同じ計算契約を使います。ただし追加Obstacleが空の場合の性能を維持するため、水平面専用の軽量経路を通ります。
