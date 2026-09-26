import react from '@vitejs/plugin-react'
import { defineConfig } from 'vite'

// https://vite.dev/config/
export default defineConfig({
  plugins: [react()],
  server: {
    // send /api requests to Django so the session and csrf cookies work
    proxy: {
      '/api': 'http://127.0.0.1:8000',
    },
  },
})
