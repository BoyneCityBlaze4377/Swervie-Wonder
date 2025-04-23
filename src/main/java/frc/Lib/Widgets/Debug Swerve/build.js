const esbuild = require("esbuild");

esbuild.build({
  entryPoints: ["src/DebugSwerve.js"],
  bundle: true,
  outfile: "dist/DebugSwerve.js",
  minify: true,
  target: ["es2020"]
}).catch(() => process.exit(1));