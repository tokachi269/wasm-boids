const { defineConfig } = require('@vue/cli-service')
const CopyWebpackPlugin = require('copy-webpack-plugin');
const path = require('path');

module.exports = defineConfig({
    filenameHashing: true,
    transpileDependencies: true,
    devServer: {
        https: true,
        headers: {
            'Cross-Origin-Opener-Policy': 'same-origin',
            'Cross-Origin-Embedder-Policy': 'require-corp',
        },
    },
    configureWebpack: {
        devtool: 'source-map',
        plugins: [
            new CopyWebpackPlugin({
                patterns: [
                    {
                        from: path.resolve(__dirname, 'src/wasm/build/wasm_boids.wasm'),
                         to: 'static/js/wasm_boids.wasm'
                    },
                ],
            }),
        ],
    },

    pages: {
    index: {
        entry: 'src/main.js', // ここは変えないで
        title: 'TimeLeaf', // 好きな文字列をいれてください
    }
},
    assetsDir: 'static',
    publicPath: process.env.NODE_ENV === 'production'
    ? '/wasm-boids/'
    : '/'
})
