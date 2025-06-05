import { createApp } from 'vue';
import App from './App.vue';
import * as BoidsModule from './wasm/build/wasm_boids.js';
import wasmUrl from './wasm/build/wasm_boids.wasm';

let wasmModule = null;
console.log("wasmUrl:", wasmUrl); // ここで URL を確認

BoidsModule.default({
    locateFile: (path) => {
        if (path.endsWith('.wasm')) {
            return wasmUrl; // ハッシュ付きの正しいURLを返す
        }
        return path;
    },
}).then(Module => {
    wasmModule = Module;
    console.log('Wasm module initialized:', Module);

    // Vue アプリケーションに WebAssembly モジュールを渡す
    const app = createApp(App);
    app.provide('wasmModule', Module);
    app.mount('#app');
});