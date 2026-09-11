# coal WASM demo

This directory builds the WebAssembly module behind `../index.html` (the
"coal Solver Playground" — pick two coal primitive shapes, pose one in 6 DoF,
and see `coal::collide`/`coal::distance`'s witness points, normal, signed
gap/depth, and — for GJK/EPA pairs — the exact final simplex/EPA face,
straight from coal's own solver running in-browser).

## Files

- `coal_wasm.cpp` — the Emscripten/embind binding. Exposes one function,
  `solvePair(specA, specB, tx, ty, tz, r00..r22)`, which builds two shapes
  from JS spec objects and calls the real `coal::collide()`/`coal::distance()`
  (through `ShapeBase*`, exactly like a normal user would), plus — for shape
  pairs that aren't Plane/Halfspace — runs coal's own templated
  `GJKSolver::shapeDistance<S1,S2>()` a second time purely to read back its
  public `gjk`/`epa` members afterward (the final simplex and, if the pair
  penetrates, EPA's closest polytope face).
- `build.sh` — builds `coal_wasm.js` (a single self-contained file, WASM
  binary included as base64 — no separate `.wasm` fetch, so `index.html`
  works straight off `file://` or any static host, GitHub Pages included).
- `test_node.js` — a quick Node-only smoke test of the built module (a few
  hand-picked shape pairs, run directly, no browser).
- `cdp_test.js` — a headless-browser regression test that drives the actual
  `index.html` page (via the Chrome DevTools Protocol against a
  `--headless=new` Chromium/Edge instance you point it at on port 9333):
  clicks Solve, Random pose, Snap to touching, toggles every overlay chip,
  and fuzzes shape B through several kinds — asserting zero console errors
  and zero thrown exceptions.

## Building

Prereqs: an activated [emsdk](https://emscripten.org/docs/getting_started/downloads.html)
(`source <emsdk>/emsdk_env.sh`), and Eigen3 + Boost headers on disk (header-only
use — `libboost_serialization` is never linked, this demo never serializes
anything). Then:

```sh
./build.sh
```

The three CMake-generated headers (`coal/config.hh`, `deprecated.hh`,
`warning.hh`) are reused from a native CMake configure (default
`../../build_verify/include`, override with `GEN_INC=...`) rather than
re-running CMake just for those three tiny files — point `GEN_INC` at any
configured coal build directory's `include/`, or generate them by hand from
the `.hh.in` templates in `include/coal/` if you don't have one.

`build.sh` compiles coal's actual sources directly (not the prebuilt native
`coal.dll`/`.so` — WASM is a different target architecture entirely), minus
`mesh_loader/*` (needs assimp), `octree.cpp` (needs octomap,
`COAL_HAS_OCTOMAP` is left undefined), and `serialization/serialization.cpp`
(needs `libboost_serialization`) — none of which shape-vs-shape
collide/distance/GJK/EPA needs.

## Testing

```sh
node test_node.js                 # quick sanity check of the wasm module alone
# in another terminal: launch a headless browser with remote debugging, e.g.
#   msedge --headless=new --disable-gpu --remote-debugging-port=9333 --user-data-dir=<tmp dir> about:blank
node cdp_test.js                  # drives the real page through that browser
```
