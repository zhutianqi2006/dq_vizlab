import { defineConfig } from 'vite';

const isProd = process.env.NODE_ENV === 'production';

export default defineConfig({
  base: isProd ? '/dq_vizlab/' : '/',
  server: {
    port: 5173,
    open: true,
    headers: {
      // 确保 WASM 文件可以被正确加载
      'Cross-Origin-Opener-Policy': 'same-origin',
      'Cross-Origin-Embedder-Policy': 'require-corp'
    }
  },
  build: {
    target: 'esnext',
    outDir: 'dist'
  },
  publicDir: 'public',
  assetsInclude: ['**/*.wasm']
});

