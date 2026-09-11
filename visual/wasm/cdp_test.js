// Minimal CDP driver (no npm deps -- Node 24's built-in fetch + WebSocket)
// to load visual/index.html in a headless Edge/Chrome instance already
// listening on localhost:9333 (e.g. `msedge --headless=new --disable-gpu
// --remote-debugging-port=9333 --user-data-dir=<tmp dir> about:blank`), and
// exercise it like a real user would. Reports console errors / thrown
// exceptions; exits non-zero if any occurred.
const path = require("path");
const fileUrl = "file:///" + path.resolve(__dirname, "../index.html").replace(/\\/g, "/");

async function main() {
  const newTabRes = await fetch("http://localhost:9333/json/new?" + encodeURIComponent(fileUrl), { method: "PUT" });
  const tab = await newTabRes.json();
  const ws = new WebSocket(tab.webSocketDebuggerUrl);

  let msgId = 1;
  const pending = new Map();
  const consoleMessages = [];
  const exceptions = [];

  function send(method, params) {
    return new Promise((resolve, reject) => {
      const id = msgId++;
      pending.set(id, { resolve, reject });
      ws.send(JSON.stringify({ id, method, params: params || {} }));
    });
  }

  await new Promise((resolve, reject) => {
    ws.addEventListener("open", resolve);
    ws.addEventListener("error", reject);
  });

  ws.addEventListener("message", (ev) => {
    const msg = JSON.parse(ev.data);
    if (msg.id !== undefined && pending.has(msg.id)) {
      const { resolve, reject } = pending.get(msg.id);
      pending.delete(msg.id);
      if (msg.error) reject(new Error(JSON.stringify(msg.error)));
      else resolve(msg.result);
    } else if (msg.method === "Runtime.consoleAPICalled") {
      const args = (msg.params.args || []).map((a) => a.value !== undefined ? a.value : a.description);
      consoleMessages.push({ type: msg.params.type, args });
    } else if (msg.method === "Runtime.exceptionThrown") {
      exceptions.push(msg.params.exceptionDetails);
    }
  });

  await send("Page.enable");
  await send("Runtime.enable");
  await send("Page.navigate", { url: fileUrl });

  // Wait for load + wasm module init (CoalModule().then(...)).
  await new Promise((r) => setTimeout(r, 4000));

  async function evalJs(expr) {
    const res = await send("Runtime.evaluate", { expression: expr, returnByValue: true, awaitPromise: true });
    if (res.exceptionDetails) throw new Error(JSON.stringify(res.exceptionDetails));
    return res.result.value;
  }

  console.log("Module ready:", await evalJs("Module !== null"));

  // Convexity self-check: for 300 random point sets, verify every point of
  // the set is on the non-positive side of every computed hull face's plane
  // (the defining property of a convex hull) -- catches exactly the "looks
  // non-convex" bug once reported against the fixed-topology renderer.
  const convexityCheck = await evalJs(`
    (function () {
      var EPS = 1e-6, failures = [];
      for (var trial = 0; trial < 300; trial++) {
        var pts = randomConvexPoints();
        var faces = convexHullFaces(pts);
        if (!faces) { failures.push({trial: trial, reason: "degenerate"}); continue; }
        for (var fi = 0; fi < faces.length; fi++) {
          var f = faces[fi];
          var a = pts[f[0]], b = pts[f[1]], c = pts[f[2]];
          var e1 = [b[0]-a[0],b[1]-a[1],b[2]-a[2]], e2 = [c[0]-a[0],c[1]-a[1],c[2]-a[2]];
          var n = [e1[1]*e2[2]-e1[2]*e2[1], e1[2]*e2[0]-e1[0]*e2[2], e1[0]*e2[1]-e1[1]*e2[0]];
          for (var pi = 0; pi < pts.length; pi++) {
            var v = [pts[pi][0]-a[0], pts[pi][1]-a[1], pts[pi][2]-a[2]];
            var d = n[0]*v[0]+n[1]*v[1]+n[2]*v[2];
            if (d > EPS) { failures.push({trial: trial, face: fi, point: pi, violation: d}); }
          }
        }
      }
      return JSON.stringify({trials: 300, failures: failures.length, sample: failures.slice(0, 5)});
    })()
  `);
  console.log("Convexity self-check (300 random point sets):", convexityCheck);

  // Default pair (TruncatedCone under a Box, flush) -- expect a 4-point
  // SURFACE manifold straight out of the box.
  await evalJs("doSolve(); true");
  console.log("Default pair:", await evalJs(
    "JSON.stringify({dist:lastResult.dist, kind:lastResult.manifoldKind, n:lastResult.manifoldPoints.length})"));

  // Snap-to-touching precision: separate, then snap. `poseOverride` (see
  // index.html) must make this land at ~machine-epsilon, not the ~1e-2
  // residual that <input type=range>'s step-snapping used to leave.
  await evalJs("setPose({tx:0,ty:0,tz:3,roll:0,pitch:0,yaw:0}); doSolve(); snapToTouching(); true");
  const snapDist = await evalJs("lastResult.dist");
  console.log("Snap-to-touching residual (expect ~1e-10 or smaller):", snapDist);
  if (Math.abs(snapDist) > 1e-6) {
    console.error("FAIL: snap-to-touching residual too large:", snapDist);
    exceptions.push({ text: "snap-to-touching precision regression" });
  }

  // Cone/Cylinder flat-cap-to-flat-cap (curved shapes, GJK/EPA path, not the
  // analytic Plane/Halfspace shortcut) -- also expect a 4-point manifold.
  await evalJs(`
    document.getElementById('shapeA').value = kindIndex('cone');
    document.getElementById('shapeA').dispatchEvent(new Event('change'));
    document.getElementById('shapeB').value = kindIndex('cylinder');
    document.getElementById('shapeB').dispatchEvent(new Event('change'));
    setPose({tx:0, ty:0, tz:-2, roll:0, pitch:0, yaw:0});
    doSolve();
    snapToTouching();
    true
  `);
  console.log("Cone/Cylinder base-to-cap after snap:", await evalJs(
    "JSON.stringify({dist:lastResult.dist, kind:lastResult.manifoldKind, n:lastResult.manifoldPoints.length})"));

  // Toggle all overlay chips off and back on, re-render, make sure render() doesn't throw.
  await evalJs("document.querySelectorAll('button.chip[data-ov]').forEach(function(b){b.click();}); render(); true");
  await evalJs("document.querySelectorAll('button.chip[data-ov]').forEach(function(b){b.click();}); render(); true");

  // Randomize both shapes' parameters a few times and re-solve, across
  // every shape kind, to fuzz for JS-side exceptions.
  const kinds = ["sphere", "ellipsoid", "capsule", "cylinder", "truncatedcone", "cone", "box", "convex", "plane", "halfspace"];
  let fuzzOk = true;
  for (const kb of kinds) {
    await evalJs(`document.getElementById('shapeB').value = kindIndex(${JSON.stringify(kb)}); document.getElementById('shapeB').dispatchEvent(new Event('change')); true`);
    await evalJs("document.getElementById('randA').click(); document.getElementById('randB').click(); randomPose(); doSolve(); render(); true");
    const ok = await evalJs("lastResult && lastResult.ok");
    if (!ok) fuzzOk = false;
    console.log("kind B =", kb, "-> ok:", ok);
  }
  console.log("Fuzz all-ok:", fuzzOk);

  console.log("\n--- console messages (" + consoleMessages.length + ") ---");
  consoleMessages.forEach((m) => console.log(m.type, m.args));
  console.log("\n--- exceptions (" + exceptions.length + ") ---");
  exceptions.forEach((e) => console.log(JSON.stringify(e)));

  ws.close();
  process.exit(exceptions.length > 0 || !fuzzOk ? 1 : 0);
}

main().catch((e) => { console.error("TEST DRIVER ERROR:", e); process.exit(1); });
