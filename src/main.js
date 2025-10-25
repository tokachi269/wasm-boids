import { createApp } from 'vue';
import App from './App.vue';

// COI対応によりSharedArrayBuffer等を利用する関係で、WASMファイルは固定パスにコピーしている
const publicBase = (process.env.BASE_URL || '/').replace(/\/*$/, '/');
const wasmPublicBase = `${publicBase}static/js`;
const wasmJsUrl = `${wasmPublicBase}/wasm_boids.js`;
const wasmBinaryUrl = `${wasmPublicBase}/wasm_boids.wasm`;

import(/* webpackIgnore: true */ wasmJsUrl)
    .then((BoidsModule) => {
        if (!BoidsModule?.default) {
            throw new Error('WASM module loader not found at: ' + wasmJsUrl);
        }
            return BoidsModule.default({
                locateFile: (path) => (path.endsWith('.wasm') ? wasmBinaryUrl : path),
            });
    })
    .then((Module) => {
        // デバッグ確認用にグローバルへ公開しつつ初期化ログを出力
        if (typeof window !== 'undefined') {
            window.wasmModule = Module;
        }
        console.log('Wasm module initialized:', Module);
        const app = createApp(App);
        app.provide('wasmModule', Module);
        app.mount('#app');
    })
    .catch((error) => {
        console.error('Failed to initialise WASM module:', error);
    });