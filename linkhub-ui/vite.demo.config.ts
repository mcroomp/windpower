import { defineConfig } from "vite";
import { fileURLToPath } from "node:url";

export default defineConfig({
  base: "./",
  resolve: {
    alias: {
      "./runtime-api": fileURLToPath(new URL("./src/demo-runtime-api.ts", import.meta.url)),
    },
  },
  build: {
    outDir: fileURLToPath(new URL("./dist-demo", import.meta.url)),
    emptyOutDir: true,
  },
});
