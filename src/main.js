import { createApp } from 'vue';
import App from './App.vue';

// SharedArrayBuffer を利用するため COOP/COEP が成立する固定パスから wasm をロードする
const publicBase = (process.env.BASE_URL || '/').replace(/\/*$/, '/');
const wasmPublicBase = `${publicBase}static/js`;
const wasmJsUrl = `${wasmPublicBase}/wasm_boids.js`;
const wasmBinaryUrl = `${wasmPublicBase}/wasm_boids.wasm`;

async function bootstrap() {
    try {
        const BoidsModule = await import(/* webpackIgnore: true */ wasmJsUrl);
        if (!BoidsModule?.default) {
            throw new Error(`WASM module loader not found at: ${wasmJsUrl}`);
        }

        const Module = await BoidsModule.default({
            locateFile: (path) => (path.endsWith('.wasm') ? wasmBinaryUrl : path),
        });

        // デバッグ確認用にグローバルへ公開しつつ初期化ログを出力
        if (typeof window !== 'undefined') {
            window.wasmModule = Module;
        }
        console.log('Wasm module initialized:', Module);

        const app = createApp(App);
        app.provide('wasmModule', Module);
        app.mount('#app');
    } catch (error) {
        console.error('Failed to initialise WASM module:', error);
    }
}

bootstrap();