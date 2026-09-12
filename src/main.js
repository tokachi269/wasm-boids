import { createApp } from 'vue';
import App from './App.vue';

// SharedArrayBuffer を利用するため COOP/COEP が成立する固定パスから wasm をロードする
const publicBase = (process.env.BASE_URL || '/').replace(/\/*$/, '/');
const wasmPublicBase = `${publicBase}static/js`;
const wasmJsUrl = `${wasmPublicBase}/wasm_boids.js`;
const wasmBinaryUrl = `${wasmPublicBase}/wasm_boids.wasm`;
const controlledIsolationReloadKey = 'wasmBoidsControlledIsolationReloadAttempted';

function waitForServiceWorkerController() {
    if (navigator.serviceWorker.controller) {
        return Promise.resolve();
    }

    return new Promise((resolve, reject) => {
        const timeoutId = window.setTimeout(() => {
            navigator.serviceWorker.removeEventListener('controllerchange', onControllerChange);
            reject(new Error('The COOP/COEP service worker did not take control of the page.'));
        }, 10000);
        const onControllerChange = () => {
            window.clearTimeout(timeoutId);
            navigator.serviceWorker.removeEventListener('controllerchange', onControllerChange);
            resolve();
        };
        navigator.serviceWorker.addEventListener('controllerchange', onControllerChange);
    });
}

async function ensureCrossOriginIsolation() {
    if (window.crossOriginIsolated) {
        window.sessionStorage.removeItem(controlledIsolationReloadKey);
        return true;
    }

    if (!window.isSecureContext || !('serviceWorker' in navigator)) {
        throw new Error('SharedArrayBuffer requires a secure, cross-origin-isolated context.');
    }

    const registration = await navigator.serviceWorker.ready;
    if (!registration.active) {
        throw new Error('The COOP/COEP service worker did not become active.');
    }
    await waitForServiceWorkerController();

    if (window.sessionStorage.getItem(controlledIsolationReloadKey)) {
        window.sessionStorage.removeItem(controlledIsolationReloadKey);
        throw new Error('Cross-origin isolation was not established after a controlled reload.');
    }

    window.sessionStorage.setItem(controlledIsolationReloadKey, '1');
    window.location.reload();
    return false;
}

async function bootstrap() {
    try {
        if (!(await ensureCrossOriginIsolation())) {
            return;
        }

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
