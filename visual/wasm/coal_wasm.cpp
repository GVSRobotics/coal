// visual/wasm/coal_wasm.cpp
//
// Emscripten/embind bindings exposing the REAL coal collision library
// (coal::collide / coal::distance -- the exact same entry points a normal
// C++ or Python user calls) to the interactive demo on visual/index.html.
//
// Two things are exposed for a given shape pair + relative pose:
//   1. The "ground truth" result: coal::collide()/coal::distance() called
//      through the ShapeBase base pointer, exactly as a real user would --
//      no reimplementation of any collision logic here, and this works for
//      EVERY shape pair including Plane/Halfspace (which go through coal's
//      internal closed-form routines, not GJK).
//   2. A "final simplex" diagnostic: for shape pairs that don't involve
//      Plane/Halfspace, we ALSO run coal's GJKSolver ourselves (the same
//      templated GJK/EPA entry point coal's own dispatch matrix uses
//      internally) so we can read back its public `gjk`/`epa` members
//      afterward -- the final GJK simplex (up to 4 vertices, each mapped to
//      its support point on shape A and shape B), and, if GJK found
//      enclosure, EPA's closest polytope face. This is purely a read of
//      already-public state (GJKSolver::gjk / GJKSolver::epa are public
//      members, see include/coal/narrowphase/narrowphase.h) -- nothing here
//      duplicates coal's actual collision math.
//   3. When colliding: the actual contact PATCH (coal::computeContactPatch,
//      include/coal/contact_patch.h) -- a point contact (0D), an edge/line
//      contact (1D, reduced to its 2 endpoints), or a face/face contact
//      (2D, reduced to 4 points), using the real
//      ContactPatchSimplifierMaxArea to pick the best subset -- rather than
//      always showing the single closest-point pair, which understates a
//      pair that's actually touching along a line or over a whole face
//      (same normal at every point, per the ContactPatch API's own design).
//

// coal has no type-erased "concrete shape" value type usable as a template
// argument (ShapeBase is only useful through virtual dispatch), so getting
// the *diagnostic* simplex -- which needs GJKSolver::shapeDistance<S1,S2>,
// a template -- needs an explicit double dispatch over the 7 solid
// primitives, exactly like the sibling DCOLpp project's own WASM binding
// (visual/wasm/dcolpp_wasm.cpp there) does for its 8 shapes via
// std::variant + nested std::visit.
//
// Build (from visual/wasm/, after activating emsdk; see build.sh):
//   see build.sh in this directory.

#include <emscripten/bind.h>
#include <emscripten/val.h>
#include <Eigen/Dense>
#include <memory>
#include <variant>

#include "coal/shape/geometric_shapes.h"
#include "coal/shape/convex.h"
#include "coal/narrowphase/narrowphase.h"
#include "coal/collision.h"
#include "coal/distance.h"
#include "coal/collision_data.h"
#include "coal/contact_patch.h"
#include "coal/contact_patch/contact_patch_simplifier.h"

using namespace coal;
using namespace emscripten;

namespace {

// ---------------------------------------------------------------------
// Shape construction from a JS spec object. All shape geometry lives in
// visual/index.html (SHAPES + DEFAULTS + randomShape); this file just
// materialises whatever spec it is handed. Spec shapes:
//   {kind:"box",         sx, sy, sz}        (full side lengths)
//   {kind:"sphere",      R}
//   {kind:"ellipsoid",   a, b, c}
//   {kind:"capsule",     R, L}
//   {kind:"cylinder",    R, L}
//   {kind:"cone",        R, L}
//   {kind:"truncatedcone", Rb, Rt, L}
//   {kind:"plane",       n:[x,y,z], d}      (n . x = d)
//   {kind:"halfspace",   n:[x,y,z], d}      (n . x <= d is solid)
// ---------------------------------------------------------------------
Vec3s vec3FromVal(const val& v) {
  return Vec3s(v[0].as<double>(), v[1].as<double>(), v[2].as<double>());
}

// {kind:"convex", points:[[x,y,z] x 8]} -- an arbitrary (not necessarily
// box-shaped) convex polytope over 8 vertices, so the demo has a
// "polytope"-style shape (coal's ConvexBase/Convex) alongside the named
// primitives. The 8 vertices are given a fixed box-like triangle topology
// (12 triangles); note this doesn't actually constrain the collision math
// at all -- ConvexBase's GJK/EPA support function (getShapeSupportLinear,
// src/narrowphase/support_functions.cpp) only ever reads `points`, never
// the triangle list, so the support -- and therefore collide()/distance()
// -- is exactly the support of the convex hull of the 8 points regardless
// of the (possibly non-planar, for a heavily perturbed point set) supplied
// faces. The triangle list only matters for things this demo never calls
// (contact-patch face sampling, BVH mesh export), so a fixed topology is
// fine; visual/index.html independently triangulates the SAME 8 points the
// SAME way for its own rendering, so the two stay visually consistent.
const int kConvexTriIdx[12][3] = {
    {0, 1, 2}, {0, 2, 3}, {4, 5, 6}, {4, 6, 7}, {0, 1, 5}, {0, 5, 4},
    {3, 2, 6}, {3, 6, 7}, {0, 3, 7}, {0, 7, 4}, {1, 2, 6}, {1, 6, 5}};

ConvexTpl<Triangle32> buildConvex(const val& s) {
  const val ptsVal = s["points"];
  auto points = std::make_shared<std::vector<Vec3s>>();
  points->reserve(8);
  for (int i = 0; i < 8; ++i) points->push_back(vec3FromVal(ptsVal[i]));
  auto polys = std::make_shared<std::vector<Triangle32>>();
  polys->reserve(12);
  for (const auto& t : kConvexTriIdx)
    polys->emplace_back(Triangle32::IndexType(t[0]), Triangle32::IndexType(t[1]),
                        Triangle32::IndexType(t[2]));
  return ConvexTpl<Triangle32>(points, 8, polys, 12);
}

// The 8 "solid volume" primitives usable directly with coal's templated
// GJK/EPA solver (GJKSolver::shapeDistance<S1,S2>). Plane/Halfspace are
// deliberately excluded: their support function is degenerate (they are
// infinite), and coal itself never runs GJK on them -- it always resolves
// them analytically (see src/narrowphase/details.h::planeDistance /
// halfspaceDistance), which is exactly why this demo treats them
// separately below.
using GjkShapeVariant = std::variant<Box, Sphere, Ellipsoid, Capsule, Cone,
                                     Cylinder, TruncatedCone, ConvexTpl<Triangle32>>;

bool isPlaneOrHalfspace(const val& s) {
  const std::string kind = s["kind"].as<std::string>();
  return kind == "plane" || kind == "halfspace";
}

// Builds a heap-allocated, type-erased shape for the "ground truth"
// coal::collide()/coal::distance() calls -- works for every kind.
std::unique_ptr<ShapeBase> buildShapeBase(const val& s) {
  const std::string kind = s["kind"].as<std::string>();
  if (kind == "box")
    return std::make_unique<Box>(s["sx"].as<double>(), s["sy"].as<double>(),
                                 s["sz"].as<double>());
  if (kind == "sphere") return std::make_unique<Sphere>(s["R"].as<double>());
  if (kind == "ellipsoid")
    return std::make_unique<Ellipsoid>(s["a"].as<double>(), s["b"].as<double>(),
                                       s["c"].as<double>());
  if (kind == "capsule")
    return std::make_unique<Capsule>(s["R"].as<double>(), s["L"].as<double>());
  if (kind == "cylinder")
    return std::make_unique<Cylinder>(s["R"].as<double>(), s["L"].as<double>());
  if (kind == "cone")
    return std::make_unique<Cone>(s["R"].as<double>(), s["L"].as<double>());
  if (kind == "truncatedcone")
    return std::make_unique<TruncatedCone>(
        s["Rb"].as<double>(), s["Rt"].as<double>(), s["L"].as<double>());
  if (kind == "plane")
    return std::make_unique<Plane>(vec3FromVal(s["n"]), s["d"].as<double>());
  if (kind == "halfspace")
    return std::make_unique<Halfspace>(vec3FromVal(s["n"]), s["d"].as<double>());
  // convex
  return std::make_unique<ConvexTpl<Triangle32>>(buildConvex(s));
}

// Builds the same shape as a GjkShapeVariant, for shapes where that is
// possible (i.e. never Plane/Halfspace -- checked by the caller via
// isPlaneOrHalfspace before calling this).
GjkShapeVariant buildGjkShape(const val& s) {
  const std::string kind = s["kind"].as<std::string>();
  if (kind == "box")
    return Box(s["sx"].as<double>(), s["sy"].as<double>(), s["sz"].as<double>());
  if (kind == "sphere") return Sphere(s["R"].as<double>());
  if (kind == "ellipsoid")
    return Ellipsoid(s["a"].as<double>(), s["b"].as<double>(), s["c"].as<double>());
  if (kind == "capsule") return Capsule(s["R"].as<double>(), s["L"].as<double>());
  if (kind == "cylinder") return Cylinder(s["R"].as<double>(), s["L"].as<double>());
  if (kind == "truncatedcone")
    return TruncatedCone(s["Rb"].as<double>(), s["Rt"].as<double>(),
                         s["L"].as<double>());
  if (kind == "cone") return Cone(s["R"].as<double>(), s["L"].as<double>());
  // convex
  return buildConvex(s);
}

Transform3s makeTf(double tx, double ty, double tz, double r00, double r01,
                   double r02, double r10, double r11, double r12, double r20,
                   double r21, double r22) {
  Matrix3s R;
  R << r00, r01, r02, r10, r11, r12, r20, r21, r22;
  return Transform3s(R, Vec3s(tx, ty, tz));
}

val vecToVal(const Vec3s& v) {
  return val::array(std::vector<double>{v[0], v[1], v[2]});
}

const char* gjkStatusName(details::GJK::Status s) {
  switch (s) {
    case details::GJK::DidNotRun:
      return "DidNotRun";
    case details::GJK::Failed:
      return "Failed (ran out of iterations)";
    case details::GJK::NoCollisionEarlyStopped:
      return "NoCollisionEarlyStopped";
    case details::GJK::NoCollision:
      return "NoCollision";
    case details::GJK::CollisionWithPenetrationInformation:
      return "CollisionWithPenetrationInformation";
    case details::GJK::Collision:
      return "Collision";
    default:
      return "?";
  }
}

const char* epaStatusName(details::EPA::Status s) {
  switch (s) {
    case details::EPA::DidNotRun:
      return "DidNotRun";
    case details::EPA::Failed:
      return "Failed (ran out of iterations)";
    case details::EPA::Valid:
      return "Valid";
    case details::EPA::AccuracyReached:
      return "AccuracyReached";
    case details::EPA::Degenerated:
      return "Degenerated (degenerate face)";
    case details::EPA::NonConvex:
      return "NonConvex";
    case details::EPA::InvalidHull:
      return "InvalidHull";
    case details::EPA::OutOfFaces:
      return "OutOfFaces";
    case details::EPA::OutOfVertices:
      return "OutOfVertices";
    case details::EPA::FallBack:
      return "FallBack";
    default:
      return "?";
  }
}

// Runs coal's own templated GJK/EPA solver directly (bypassing the
// analytic Plane/Halfspace shortcut, which is why this is only called for
// non-Plane/Halfspace pairs) purely to read back its internal `gjk`/`epa`
// state afterward for display. The numeric result (p1/p2/normal/distance)
// this also produces is NOT used for the headline numbers -- those always
// come from the real coal::collide()/coal::distance() calls below, so the
// two independent computations act as a cross-check of each other too.
template <typename S1, typename S2>
void runDiagnosticGJK(const S1& s1, const Transform3s& tf1, const S2& s2,
                      const Transform3s& tf2, val& out) {
  GJKSolver solver;
  Vec3s p1, p2, normal;
  solver.shapeDistance(s1, tf1, s2, tf2, /*compute_penetration=*/true, p1, p2,
                       normal);

  out.set("gjkApplicable", true);
  out.set("gjkStatus", std::string(gjkStatusName(solver.gjk.status)));
  out.set("epaStatus", std::string(epaStatusName(solver.epa.status)));

  const details::GJK::Simplex* simplex = solver.gjk.getSimplex();
  val simplexArr = val::array();
  if (simplex != nullptr) {
    for (int i = 0; i < simplex->rank; ++i) {
      const details::GJK::SimplexV* v = simplex->vertex[i];
      val vv = val::object();
      // v->w0/w1 are expressed in shape1's local frame (coal's Minkowski
      // difference reference frame) -- map to world with tf1, exactly how
      // coal's own GJKExtractWitnessPointsAndNormal does internally.
      vv.set("onA", vecToVal(tf1.transform(v->w0.template cast<Scalar>())));
      vv.set("onB", vecToVal(tf1.transform(v->w1.template cast<Scalar>())));
      simplexArr.call<void>("push", vv);
    }
  }
  out.set("simplex", simplexArr);

  if (solver.epa.status != details::EPA::DidNotRun) {
    val face = val::object();
    val faceVerts = val::array();
    for (int i = 0; i < solver.epa.result.rank; ++i) {
      const details::GJK::SimplexV* v = solver.epa.result.vertex[i];
      val vv = val::object();
      vv.set("onA", vecToVal(tf1.transform(v->w0.template cast<Scalar>())));
      vv.set("onB", vecToVal(tf1.transform(v->w1.template cast<Scalar>())));
      faceVerts.call<void>("push", vv);
    }
    face.set("vertices", faceVerts);
    face.set("normal", vecToVal(tf1.getRotation() *
                                solver.epa.normal.cast<Scalar>()));
    face.set("depth", double(solver.epa.depth));
    out.set("epaFace", face);
  } else {
    out.set("epaFace", val::null());
  }
}

// Computes the contact patch (only meaningful when colliding) and reduces
// it to the points worth actually drawing:
//   - 0D (point contact): 1 point.
//   - 1D (edge/line contact): its 2 endpoints.
//   - 2D (face/face contact): 4 points (the ContactPatchSimplifierMaxArea
//     subset maximizing the area of the reduced quad -- so the 4 points
//     drawn are as representative of the true patch's extent as possible,
//     not an arbitrary subset).
// Dimension is read directly off the RAW (unsimplified) patch's own 2D
// local-frame point cloud (ContactPatch::points(), already expressed in the
// patch's own (x, y) tangent basis -- see ContactPatch::addPoint) via its
// axis-aligned bounding extent: negligible in both axes -> point; negligible
// in one axis only -> line; otherwise -> a genuine 2D surface.
void addManifold(const ShapeBase* shapeA, const Transform3s& tf1,
                 const ShapeBase* shapeB, const Transform3s& tf2,
                 const CollisionResult& colres, val& out) {
  out.set("manifoldKind", std::string("none"));
  out.set("manifoldPoints", val::array());
  if (!colres.isCollision()) return;

  ContactPatchRequest patchReq;
  ContactPatchResult patchRes(patchReq);
  computeContactPatch(shapeA, tf1, shapeB, tf2, colres, patchReq, patchRes);
  if (patchRes.numContactPatches() == 0) return;

  const ContactPatch& patch = patchRes.getContactPatch(0);
  const size_t n = patch.size();
  if (n == 0) return;

  const std::vector<Vec2s>& pts2d = patch.points();
  Scalar minX = pts2d[0].x(), maxX = pts2d[0].x();
  Scalar minY = pts2d[0].y(), maxY = pts2d[0].y();
  for (const auto& p : pts2d) {
    minX = (std::min)(minX, p.x()); maxX = (std::max)(maxX, p.x());
    minY = (std::min)(minY, p.y()); maxY = (std::max)(maxY, p.y());
  }
  const Scalar extentX = maxX - minX, extentY = maxY - minY;
  const Scalar maxExtent = (std::max)(extentX, extentY);
  const Scalar minExtent = (std::min)(extentX, extentY);
  const Scalar relTol = Scalar(1e-3);

  std::string kind;
  size_t target;
  if (n == 1 || maxExtent < Scalar(1e-9)) {
    kind = "point"; target = 1;
  } else if (minExtent < relTol * maxExtent) {
    kind = "line"; target = 2;
  } else {
    kind = "surface"; target = 4;
  }
  target = (std::min)(target, n);

  ContactPatch reduced(target);
  if (target < n) {
    ContactPatchSimplifierMaxArea simplifier;
    simplifier.compute(patch, target, reduced);
  } else {
    reduced = patch;
  }

  out.set("manifoldKind", kind);
  val pts = val::array();
  for (size_t i = 0; i < reduced.size(); ++i) {
    val vv = val::object();
    vv.set("onA", vecToVal(reduced.getPointShape1(i)));
    vv.set("onB", vecToVal(reduced.getPointShape2(i)));
    pts.call<void>("push", vv);
  }
  out.set("manifoldPoints", pts);
  // Single normal for the whole patch -- same convention as
  // DistanceResult::normal (points from shape A to shape B).
  // ContactPatch::getNormal() already accounts for `direction` internally.
  out.set("manifoldNormal", vecToVal(reduced.getNormal()));
}

// ---------------------------------------------------------------------
// JS-facing API: two shape spec objects + body B's pose (body A is always
// at the identity pose) as translation + row-major rotation, matching
// coal::collide(o1, tf1, o2, tf2, ...)'s own convention (o1 = shape A at
// tf1 = identity, o2 = shape B at tf2).
// ---------------------------------------------------------------------
val solvePair(val specA, val specB, double tx, double ty, double tz,
             double r00, double r01, double r02, double r10, double r11,
             double r12, double r20, double r21, double r22) {
  const Transform3s tf1;  // identity
  const Transform3s tf2 = makeTf(tx, ty, tz, r00, r01, r02, r10, r11, r12, r20,
                                 r21, r22);

  std::unique_ptr<ShapeBase> shapeA = buildShapeBase(specA);
  std::unique_ptr<ShapeBase> shapeB = buildShapeBase(specB);

  val out = val::object();
  out.set("ok", true);

  // 1. Ground truth: the REAL coal::collide()/coal::distance(), exactly as
  // a real user would call them, via the type-erased ShapeBase pointers.
  // This works uniformly for every shape pair, including Plane/Halfspace.
  // CONTACT is requested so the CollisionResult carries what
  // computeContactPatch() (step 3 below) needs.
  CollisionRequest colreq(CollisionRequestFlag::CONTACT, 1);
  colreq.distance_upper_bound = (std::numeric_limits<Scalar>::max)();
  CollisionResult colres;
  size_t numContacts =
      collide(shapeA.get(), tf1, shapeB.get(), tf2, colreq, colres);

  DistanceRequest distreq;
  DistanceResult distres;
  Scalar dist = distance(shapeA.get(), tf1, shapeB.get(), tf2, distreq, distres);

  out.set("isCollision", colres.isCollision());
  out.set("numContacts", double(numContacts));
  out.set("dist", double(dist));
  out.set("p1", vecToVal(distres.nearest_points[0]));
  out.set("p2", vecToVal(distres.nearest_points[1]));
  out.set("normal", vecToVal(distres.normal));

  // 2. Diagnostic: the raw GJK/EPA simplex, only meaningful (and only run)
  // when neither shape is Plane/Halfspace -- coal itself never runs GJK on
  // those, it always uses a closed-form formula (a single support-function
  // query against the shape's own known normal), so there is no simplex.
  if (isPlaneOrHalfspace(specA) || isPlaneOrHalfspace(specB)) {
    out.set("gjkApplicable", false);
    out.set("gjkNote",
            std::string("coal resolves this pair analytically (closed-form "
                        "Plane/Halfspace distance) -- no GJK/EPA runs, so "
                        "there is no simplex to show."));
  } else {
    GjkShapeVariant vA = buildGjkShape(specA);
    GjkShapeVariant vB = buildGjkShape(specB);
    std::visit(
        [&](const auto& a) {
          std::visit([&](const auto& b) { runDiagnosticGJK(a, tf1, b, tf2, out); },
                     vB);
        },
        vA);
  }

  // 3. Contact patch / manifold: when colliding, classify + reduce it to
  // 1 (point), 2 (line endpoints) or 4 (surface) points -- see addManifold.
  addManifold(shapeA.get(), tf1, shapeB.get(), tf2, colres, out);

  return out;
}

}  // namespace

EMSCRIPTEN_BINDINGS(coal_module) { function("solvePair", &solvePair); }
