`npm run build-native` で native バイナリをビルドする。
`build-native\wasm_boids_native.exe --bench 2000 --seed 1 --boids 50000` で固定条件のベンチを実行する。
出力 JSON の `frame_ms.p95` を比較し、`checksum` が一致する結果だけを採用する。
