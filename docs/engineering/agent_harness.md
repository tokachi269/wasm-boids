# Agent engineering harness

`wire`のportable harnessから、wasm-boidsで実際に必要な変更手順だけを移したもの。architecture lint、domain ledger、汎用manifestは、この規模では維持費が勝るため導入しない。

## Small change record

実装前に、会話または作業メモで次を短く確定する。分からない欄を推測で埋めない。

```text
Observed problem:
Acceptance condition:
Root cause / hypothesis:
In scope:
Out of scope:
Forbidden compensation:
Primary proof:
Counter-check:
Stop condition:
```

## Bug-fix flow

```text
reproduce
  -> isolate the responsible stage
  -> define the expected observable result
  -> minimal causal fix
  -> same-condition comparison
  -> adjacent/counter-view check
  -> full diff review
```

修正後に別の違和感が出た場合は、追加の補償を入れる前に変更を縮小またはrevertして比較する。たとえば魚の遠景色が問題なら背景色で相殺せず、魚material、媒質、tone mappingのどこで差が生じたかを切り分ける。

## Visual-change flow

```text
capture current deployed/local behavior
  -> identify color/alpha/depth/post-process ownership
  -> change one owner
  -> Release build
  -> same-view before/after
  -> water-surface and seabed counter-views
  -> Console/WebGL check
  -> commit
  -> deploy and verify public URL
```

スクリーンショット取得は証拠の収集であり、判定ではない。境界、色の連続性、背景との馴染み、通常状態での見え方を実際に確認する。

## Simulation/performance flow

```text
baseline JSON and checksum
  -> one bounded change
  -> short Release A/B
  -> checksum equality
  -> retain or revert
  -> formal benchmark when warranted
```

計測と最適化を同じ変更へ混ぜない。性能差を主張するときは平均だけでなくmedian、p95、p99、maxを見て、OSやbrowserの揺れがある短縮runはbaseline/variantを交互に複数回測る。

## Independent review

変更が複数ownerへまたがる、または最初の仮説と実際の変更範囲が大きく異なる場合は、実装を続ける前に完全なdiffを別contextで監査する。確認対象は因果上必要なedit、scope外の変更、テスト弱体化、第二の正本やspecial pathの追加である。

## Escalation

次の場合は勝手に設計を広げず停止して報告する。

- 正確な検証にシミュレーション仕様変更が必要。
- 見た目を直すために無関係な背景、camera、物理parameterの変更が必要に見える。
- 小修正のはずがC++、binding、JS、rendererを横断する第二経路を要求する。
- 同一条件を作れず、変更前後を信用できる形で比較できない。
