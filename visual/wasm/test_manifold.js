// Run after building coal_wasm.js: node visual/wasm/test_manifold.js
const assert = require("node:assert/strict");
const CoalModule = require("./coal_wasm.js");

const identity = [1, 0, 0, 0, 1, 0, 0, 0, 1];
const box = { kind: "box", sx: 1, sy: 1, sz: 1 };
const sphere = { kind: "sphere", R: 0.5 };
const halfspace = { kind: "halfspace", n: [0, 0, 1], d: 0 };
const depth = 0.01;
const edgeTilt = Math.PI / 4;

function rotateX(angle) {
  const c = Math.cos(angle), s = Math.sin(angle);
  return [1, 0, 0, 0, c, -s, 0, s, c];
}

function rotateY(angle) {
  const c = Math.cos(angle), s = Math.sin(angle);
  return [c, 0, s, 0, 1, 0, -s, 0, c];
}

function rotateZ(angle) {
  const c = Math.cos(angle), s = Math.sin(angle);
  return [c, -s, 0, s, c, 0, 0, 0, 1];
}

function multiply(a, b) {
  return Array.from({ length: 9 }, (_, i) =>
    [0, 1, 2].reduce((sum, k) =>
      sum + a[Math.floor(i / 3) * 3 + k] * b[k * 3 + i % 3], 0));
}

function transform(rotation, point) {
  return [0, 1, 2].map(i =>
    rotation[3 * i] * point[0] + rotation[3 * i + 1] * point[1] +
    rotation[3 * i + 2] * point[2]);
}

const corners = [
  [-0.5, -0.5, -0.5], [0.5, -0.5, -0.5],
  [0.5, 0.5, -0.5], [-0.5, 0.5, -0.5],
  [-0.5, -0.5, 0.5], [0.5, -0.5, 0.5],
  [0.5, 0.5, 0.5], [-0.5, 0.5, 0.5],
];
const tiltedBox = {
  kind: "convex",
  points: corners.map(point => transform(rotateX(edgeTilt), point)),
};

// Patch vertices have no guaranteed starting point or winding.
function expectPointSet(actual, expected, tolerance = 1e-6) {
  assert.equal(actual.length, expected.length);
  const remaining = actual.slice();
  for (const point of expected) {
    const index = remaining.findIndex(candidate =>
      Math.hypot(...candidate.map((value, i) => value - point[i])) <= tolerance);
    assert.notEqual(index, -1,
      `Missing point ${JSON.stringify(point)} in ${JSON.stringify(actual)}`);
    remaining.splice(index, 1);
  }
}

function expectManifold(result, kind, onA, onB, tolerance = 1e-6) {
  assert.equal(result.ok, true);
  assert.equal(result.isCollision, true);
  assert.equal(result.manifoldKind, kind);
  expectPointSet(result.manifoldPoints.map(point => point.onA), onA, tolerance);
  expectPointSet(result.manifoldPoints.map(point => point.onB), onB, tolerance);
  assert.ok(result.manifoldNormal.every(Number.isFinite));
  assert.ok(Math.abs(Math.hypot(...result.manifoldNormal) - 1) < 1e-6);
}

async function main() {
  const Module = await CoalModule();
  const solve = (a, b, translation, rotation = identity) =>
    Module.solvePair(a, b, ...translation, ...rotation);
  let passed = 0, failed = 0;
  function check(label, run) {
    try {
      run();
      ++passed;
      console.log(`PASS ${label}`);
    } catch (error) {
      ++failed;
      console.error(`FAIL ${label}: ${error.message}`);
    }
  }

  // Each box presents an edge to the common contact normal. The four angles
  // cover both signs of the segment determinant, including the sign that used
  // to project the crossing edges into a spurious two-point patch.
  for (const degrees of [-135, -45, 45, 135]) {
    const rotation = multiply(rotateZ(degrees * Math.PI / 180),
      rotateY(edgeTilt));
    const translation = [0, 0, Math.SQRT2 - depth];
    for (const useBox of [false, true]) {
      check(`crossing edges ${degrees} deg (${useBox ? "box" : "convex"}-box)`, () => {
        // Rotate the entire scene so shape A can also be an ordinary box.
        const frame = useBox ? rotateX(-edgeTilt) : identity;
        const result = solve(useBox ? box : tiltedBox, box,
          transform(frame, translation), multiply(frame, rotation));
        expectManifold(result, "point",
          [transform(frame, [0, 0, Math.SQRT1_2])],
          [transform(frame, [0, 0, Math.SQRT1_2 - depth])]);
      });
    }
  }

  // Parallel capsules have an analytic contact normal and two segment support
  // sets, isolating segment-segment overlap. Shift B along z so the patch is
  // the partial overlap [-0.25, 0.5], not either original support segment.
  for (const penetration of [0, depth]) {
    check(`parallel segments preserve overlap endpoints (depth ${penetration})`, () => {
      const capsule = { kind: "capsule", R: 0.5, L: 1 };
      const result = solve(capsule, capsule, [1 - penetration, 0, 0.25]);
      const endpoints = [[0.5, 0, -0.25], [0.5, 0, 0.5]];
      expectManifold(result, "line", endpoints,
        endpoints.map(([x, y, z]) => [x - penetration, y, z]));
    });
  }

  // Rotate a true line in its tangent plane: dimension must stay one even
  // when its bounding box has nonzero width and height.
  for (const degrees of [0, 30, 60]) {
    const angle = degrees * Math.PI / 180;
    check(`edge-face line (${degrees} deg)`, () => {
      const result = solve(box, box, [0.5 + Math.SQRT1_2 - depth, 0, 0],
        multiply(rotateX(angle), rotateZ(edgeTilt)));
      const endpoints = [-0.5, 0.5].map(t =>
        [0.5, -t * Math.sin(angle), t * Math.cos(angle)]);
      expectManifold(result, "line", endpoints,
        endpoints.map(([x, y, z]) => [x - depth, y, z]));
    });
    check(`halfspace-edge line (${degrees} deg)`, () => {
      const result = solve(halfspace, box, [0, 0, Math.SQRT1_2 - depth],
        multiply(rotateZ(angle), rotateX(edgeTilt)));
      const endpoints = [-0.5, 0.5].map(t =>
        [t * Math.cos(angle), t * Math.sin(angle), 0]);
      expectManifold(result, "line", endpoints,
        endpoints.map(([x, y]) => [x, y, -depth]));
    });
  }

  for (const degrees of [0, 45]) {
    check(`thin diamond reduces to full-span endpoints (${degrees} deg)`, () => {
      const face = [[0, -0.0005], [1, 0], [0, 0.0005], [-1, 0]];
      const prism = {
        kind: "convex",
        points: [-0.5, 0.5].flatMap(z => face.map(([x, y]) => [x, y, z])),
      };
      const angle = degrees * Math.PI / 180;
      const result = solve(halfspace, prism, [0, 0, 0.5 - depth],
        rotateZ(angle));
      // The four-vertex patch is thin enough to display as a line. Reduction
      // must preserve the diameter rather than choosing a shorter vertex pair.
      const endpoints = [-1, 1].map(t =>
        [t * Math.cos(angle), t * Math.sin(angle), 0]);
      expectManifold(result, "line", endpoints,
        endpoints.map(([x, y]) => [x, y, -depth]));
    });
  }

  check("box-box surface", () => {
    const result = solve(box, box, [0, 0, 1 - depth]);
    const face = corners.slice(4);
    expectManifold(result, "surface", face,
      face.map(([x, y, z]) => [x, y, z - depth]));
  });
  check("halfspace-box surface", () => {
    const result = solve(halfspace, box, [0, 0, 0.5 - depth]);
    const face = corners.slice(4).map(([x, y]) => [x, y, 0]);
    expectManifold(result, "surface", face,
      face.map(([x, y]) => [x, y, -depth]));
  });
  check("sphere-sphere point", () => {
    expectManifold(solve(sphere, sphere, [0.5, 0, 0]), "point",
      [[0.5, 0, 0]], [[0, 0, 0]]);
  });
  check("halfspace-sphere point", () => {
    expectManifold(solve(halfspace, sphere, [0, 0, 0.5 - depth]), "point",
      [[0, 0, 0]], [[0, 0, -depth]]);
  });
  check("separated spheres have no manifold", () => {
    const result = solve(sphere, sphere, [3, 0, 0]);
    assert.equal(result.ok, true);
    assert.equal(result.isCollision, false);
    assert.equal(result.manifoldKind, "none");
    assert.deepEqual(result.manifoldPoints, []);
  });

  console.log(`${passed} passed, ${failed} failed`);
  if (failed) process.exitCode = 1;
}

main().catch(error => {
  console.error(error);
  process.exitCode = 1;
});
