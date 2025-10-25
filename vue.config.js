const { defineConfig } = require('@vue/cli-service')
const CopyWebpackPlugin = require('copy-webpack-plugin');
const path = require('path');
// 環境変数からWASMビルドディレクトリを取得（serve時はdev、build時はprod）
const wasmBuildDir = process.env.WASM_BUILD_DIR || 'prod';
const wasmOutputDir = path.resolve(__dirname, 'src/wasm/build', wasmBuildDir);

module.exports = defineConfig({
    filenameHashing: true,
    transpileDependencies: true,
    devServer: {
        server: {
            type: 'https',
        },
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
        // Workerをmoduleスクリプトとして読み込むため、importScriptsを使わないローダーに切り替える
        output: {
            workerChunkLoading: 'import',
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
                        from: path.resolve(wasmOutputDir, 'wasm_boids.wasm'),
                        to: 'static/js/wasm_boids.wasm',
                        noErrorOnMissing: true,
                    },
                    {
                        from: path.resolve(wasmOutputDir, 'wasm_boids.js'),
                        to: 'static/js/wasm_boids.js', // pthread workerが固定パスを期待するためハッシュを付与しない
                        noErrorOnMissing: true,
                    },
                    {
                        from: path.resolve(wasmOutputDir, 'wasm_boids.wasm.map'),
                        to: 'static/js/wasm_boids.wasm.map',
                        noErrorOnMissing: true,
                    },
                ],
            }),
        ],
        resolve: {
            fallback: {
                fs: false,
                path: require.resolve("path-browserify"),
                crypto: require.resolve("crypto-browserify"),
                stream: require.resolve("stream-browserify"),
                vm: require.resolve("vm-browserify"),
                module: false,
                worker_threads: false,
            },
        },
    },
    chainWebpack: (config) => {
        config.output
            .filename('static/js/[name].[contenthash:8].js')
            .chunkFilename('static/js/[name].[contenthash:8].js');

        config.optimization
            .minimizer('terser')
            .tap((args) => {
                const options = args[0] || {};
                const existingExclude = options.exclude || [];
                options.exclude = Array.isArray(existingExclude)
                    ? existingExclude
                    : [existingExclude];
                options.exclude.push(/wasm_boids\..*\.js$/i);
                options.terserOptions = options.terserOptions || {};
                options.terserOptions.ecma = options.terserOptions.ecma || 2020;
                options.terserOptions.module = true;
                args[0] = options;
                return args;
            });
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
