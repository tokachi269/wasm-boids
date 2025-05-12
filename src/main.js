import { createApp } from 'vue';
import App from './App.vue';
import createBoidsModule from './wasm/build/wasm_boids.js';

createBoidsModule({
    locateFile: (path) => {
        if (path.endsWith('.wasm')) {
            return `/static/js/${path}`;
        }
        return path;
    },
}).then((Module) => {
    console.log('Wasm module initialized:', Module);
});

createApp(App).mount('#app');