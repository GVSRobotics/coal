const CoalModule = require("./coal_wasm.js");

CoalModule().then((Module) => {
  function run(label, specA, specB, tx, ty, tz, R) {
    const r = Module.solvePair(
      specA, specB, tx, ty, tz,
      R[0][0], R[0][1], R[0][2],
      R[1][0], R[1][1], R[1][2],
      R[2][0], R[2][1], R[2][2]
    );
    console.log("---", label, "---");
    console.log(JSON.stringify(r, null, 1));
  }

  const I = [[1,0,0],[0,1,0],[0,0,1]];

  // Sphere vs Sphere, clearly separated -> pure GJK, no penetration.
  run("sphere-sphere separated", {kind:"sphere", R:0.5}, {kind:"sphere", R:0.5}, 3, 0, 0, I);

  // Sphere vs Sphere, overlapping -> GJK + EPA.
  run("sphere-sphere overlapping", {kind:"sphere", R:0.5}, {kind:"sphere", R:0.5}, 0.5, 0, 0, I);

  // Box vs TruncatedCone, exact touch (from earlier probe: halfLength=0.5).
  run("box-truncatedcone exact touch", {kind:"box", sx:1,sy:1,sz:1}, {kind:"truncatedcone", Rb:0.25, Rt:0.15, L:1.0}, 0, 0, 1.0, I);

  // Halfspace vs TruncatedCone -> analytic path, no GJK.
  run("halfspace-truncatedcone", {kind:"halfspace", n:[0,0,1], d:0}, {kind:"truncatedcone", Rb:0.25, Rt:0.15, L:1.0}, 0, 0, 0.499, I);

  // Plane vs Cone
  run("plane-cone", {kind:"plane", n:[0,0,1], d:0}, {kind:"cone", R:0.3, L:1.0}, 0, 0, 0.5, I);

  // Box resting flush on a halfspace -> expect a 4-point SURFACE manifold.
  run("halfspace-box (surface manifold)", {kind:"halfspace", n:[0,0,1], d:0}, {kind:"box", sx:1,sy:1,sz:1}, 0, 0, 0.49, I);

  // Box edge resting on a halfspace (rotated 45deg about y) -> expect a
  // 2-point LINE manifold.
  const c = Math.SQRT1_2;
  run("halfspace-box edge (line manifold)", {kind:"halfspace", n:[0,0,1], d:0}, {kind:"box", sx:1,sy:1,sz:1}, 0, 0, c - 0.005, [[c,0,c],[0,1,0],[-c,0,c]]);

  // Sphere resting on halfspace -> expect a 1-point manifold.
  run("halfspace-sphere (point manifold)", {kind:"halfspace", n:[0,0,1], d:0}, {kind:"sphere", R:0.5}, 0, 0, 0.49, I);
});
