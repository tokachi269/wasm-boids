# Testing policy

検証はtest数ではなく、今回壊し得る契約に直接対応する証拠を選ぶ。build成功だけで見た目やシミュレーション一致を証明したことにしない。

## Change matrix

| 変更 | 最低限の検証 |
|---|---|
| 文書のみ | `git diff --check`、link/commandの目視確認 |
| Vue / Three.js / shader | `npm run vue:build`、Release相当のローカル画面、Console/WebGL error確認 |
| WASM interface / build設定 | `npm run build-wasm:dev` と `npm run build-wasm:prod`、ブラウザ起動 |
| シミュレーション挙動 | native Release build、変更前後の1 task checksum一致、WASM build |
| 性能最適化 | 上記checksumに加え `scripts/bench.md` と同条件のA/B測定 |
| deploy | `npm run build`、ローカル画面確認、deploy後の公開URL確認 |

`vue:build`は既存のWASM生成物を使うため、C++やbindingを変更した場合の最終証拠にはならない。

## Bug fix

1. 現行状態で症状を再現し、観測条件を記録する。
2. root cause候補をコードまたはbuffer/pixel/計測値で切り分ける。
3. 変更前に、何が変われば直ったと判断するか決める。
4. 最小の因果的修正を行う。
5. 同じ条件で再確認し、反対方向や隣接状態に新しい破綻がないか見る。
6. 完全なdiffから補償変更とscope拡大がないことを確認する。

新しい破綻が出た場合、さらに調整を足す前に直前の変更を戻した状態と比較する。

## Visual proof

見た目変更では、可能な限り変更前と変更後を次の条件で揃える。

- 同じRelease build条件
- 同じbrowser、viewport、device quality判定
- 同じcamera位置と向き
- 同じseed、個体数、経過条件
- 同じUI/debug toggle

最低限、依頼された視点と通常視点のスクリーンショットを人間が比較する。アニメーション中の偶然を避けるため、魚群が邪魔ならUIで一時的に対象speciesを削除して背景・地面だけを確認してよい。ただし、それだけで通常状態の確認を代替しない。

### Water / ground checklist

- 水面方向に背景mesh由来の水平線がない。
- 海底方向にPlaneの四角いdepth境界またはalphaTestの切断線がない。
- Plane中心は見え、外周はalphaで背景へ連続的に遷移する。
- 海底色にも距離・深度に応じた水中媒質が適用される。
- 魚、粒子、地面、背景の前後関係が破綻しない。
- Consoleに新しいerrorやshader compile/link errorがない。

## Deterministic simulation proof

正式な正当性条件はRelease、seed 1、fixed dt 1/60、1 taskとする。

```powershell
npm run build-native
powershell -NoProfile -ExecutionPolicy Bypass -File scripts/run-native-benchmark.ps1 -Frames 2000 -Seed 1 -Boids 20000 -Tasks 1
```

変更前後でchecksumが一致しない場合、性能が改善していても採用しない。演算順変更を「誤差」として自動承認しない。4 taskは性能参考値であり、現状のビット一致判定には使わない。

## Release browser proof

```powershell
npm run benchmark:build
npm run benchmark:serve
```

通常画面は `http://127.0.0.1:4173/wasm-boids/`、browser benchmarkは `scripts/bench.md` のURLを使う。ローカルserverはCOOP/COEP付きで配信する。

## Stop condition

次を満たした時だけ完了とする。

- 依頼された挙動または見た目を同じ条件で確認した。
- 対応するbuild/checkが成功した。
- 新しいConsole/WebGL errorがない。
- シミュレーション不変が必要ならchecksumが一致した。
- diffが要求範囲内で、無関係な変更を含まない。
- deploy依頼がある場合、公開URLで反映を確認した。
