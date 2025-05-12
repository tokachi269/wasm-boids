const path = require('path');

module.exports = {
    // ...既存の設定...
    devServer: {
        static: {
            directory: path.join(__dirname, 'src/assets'),
        },
    },
};