const { defineConfig } = require('@vue/cli-service')
const CopyWebpackPlugin = require('copy-webpack-plugin');
const path = require('path');

module.exports = defineConfig({
    filenameHashing: true,
    transpileDependencies: true,
    devServer: {
        https: true,
        // COOP/COEPでSharedArrayBuffer等を有効化
        headers: {
            'Cross-Origin-Opener-Policy': 'same-origin',
            'Cross-Origin-Embedder-Policy': 'require-corp',
        },
    },
    configureWebpack: {
        devtool: 'source-map',
        experiments: {
            asyncWebAssembly: true, // wasmのasync importを有効化
        },
        module: {
            rules: [
                {
                    test: /\.wasm$/,
                    type: 'asset/resource' // wasmファイルをリソースとして扱う
                }
            ]
        },
        plugins: [
            // wasm/jsビルド成果物をビルド時にコピー
            new CopyWebpackPlugin({
                patterns: [
                    {
                        from: path.resolve(__dirname, 'src/wasm/build/wasm_boids.wasm'),
                        to: 'static/js/[name].[contenthash:8][ext]',
                    },
                    {
                        from: path.resolve(__dirname, 'src/wasm/build/wasm_boids.js'),
                        to: 'static/js/[name].[contenthash:8][ext]',
                    },
                ],
            }),
        ],
    },

    pages: {
        index: {
            entry: 'src/main.js', // ここは変えないで
            title: 'wasm-boids', // 好きな文字列をいれる
        }
    },
    assetsDir: 'static',
    publicPath: process.env.NODE_ENV === 'production'
        ? '/wasm-boids/'
        : '/'
})
