import { createApp } from 'vue';
import App from './App.vue';
import createBoidsModule from './wasm/build/wasm_boids.js';

createBoidsModule({
    locateFile: (path) => {
        if (path.endsWith('.wasm')) {
            // 本番環境では '/wasm-boids/' を付与
            return (process.env.NODE_ENV === 'production' ? '/wasm-boids' : '') + `/static/js/${path}`;
        }
        return path;
    },
}).then((Module) => {
    console.log('Wasm module initialized:', Module);

    // Vue アプリケーションに WebAssembly モジュールを渡す
    const app = createApp(App);
    app.provide('wasmModule', Module); // provide を使用してモジュールを渡す
    app.mount('#app');
});