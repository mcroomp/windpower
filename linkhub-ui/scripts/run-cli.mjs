import { mkdir, readdir, stat } from "node:fs/promises";
import { fileURLToPath, pathToFileURL } from "node:url";
import path from "node:path";

const repoRoot = fileURLToPath(new URL("../../", import.meta.url));
const outputDirectory = path.join(repoRoot, "tmp", "linkhub-cli");
const outputFile = path.join(outputDirectory, "rawes-cli.mjs");
const forceRebuild = process.argv[2] === "--rebuild";
if (forceRebuild) {
  process.argv.splice(2, 1);
}

await mkdir(outputDirectory, { recursive: true });

async function sourceFiles(directory) {
  const entries = await readdir(directory, { withFileTypes: true });
  const nested = await Promise.all(entries.map((entry) => {
    const entryPath = path.join(directory, entry.name);
    if (entry.isDirectory()) {
      return sourceFiles(entryPath);
    }
    return entry.name.endsWith(".ts") ? [entryPath] : [];
  }));
  return nested.flat();
}

async function needsBuild() {
  if (forceRebuild) {
    return true;
  }
  let outputModified;
  try {
    outputModified = (await stat(outputFile)).mtimeMs;
  } catch (error) {
    if (error?.code === "ENOENT") {
      return true;
    }
    throw error;
  }
  const watched = [
    ...(await sourceFiles(path.join(repoRoot, "linkhub-ui", "src"))),
    fileURLToPath(import.meta.url),
    path.join(repoRoot, "linkhub-ui", "package.json"),
    path.join(repoRoot, "linkhub-ui", "package-lock.json"),
  ];
  const modified = await Promise.all(watched.map(async (file) => (await stat(file)).mtimeMs));
  return modified.some((mtime) => mtime > outputModified);
}

if (await needsBuild()) {
  const { build } = await import("esbuild");
  await build({
    entryPoints: [path.join(repoRoot, "linkhub-ui", "src", "cli.ts")],
    outfile: outputFile,
    bundle: true,
    platform: "node",
    format: "esm",
    target: "node24",
    sourcemap: "inline",
    logLevel: "silent",
  });
}

process.env.RAWES_REPO_ROOT = repoRoot;
await import(`${pathToFileURL(outputFile).href}?built=${Date.now()}`);
