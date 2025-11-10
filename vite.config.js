import { defineConfig } from 'vite';

export default defineConfig({
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

