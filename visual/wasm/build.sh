#!/usr/bin/env bash
# Build visual/wasm/coal_wasm.js from source. Run from anywhere; paths are
# resolved relative to this script's location.
#
# Prereqs: emsdk activated (source <emsdk>/emsdk_env.sh), Eigen3 headers
# available (EIGEN_INC below), Boost headers available (BOOST_INC below;
# header-only usage only -- boost_serialization is NOT linked).
set -euo pipefail
cd "$(dirname "$0")"

SRC_ROOT=../..
EIGEN_INC="${EIGEN_INC:-/c/vcpkg/installed/x64-windows/include/eigen3}"
BOOST_INC="${BOOST_INC:-/c/vcpkg/installed/x64-windows/include}"
# CMake-generated headers (coal/config.hh, deprecated.hh, warning.hh) --
# reused from the native build_verify/ CMake configure so we don't need to
# run CMake again just for these three tiny generated files.
GEN_INC="${GEN_INC:-$SRC_ROOT/build_verify/include}"

# coal sources needed for shape-vs-shape collide()/distance()/GJK/EPA.
# Deliberately excludes: mesh_loader/* (assimp dependency, unused here),
# octree.cpp (needs COAL_HAS_OCTOMAP + octomap, unused here),
# serialization/serialization.cpp (needs libboost_serialization compiled
# for wasm32, unused here -- this demo never serializes anything).
COAL_SOURCES=(
  collision.cpp
  contact_patch.cpp
  contact_patch/contact_patch_solver.cpp
  contact_patch/contact_patch_simplifier.cpp
  contact_patch/polygon_convex_hull.cpp
  contact_patch_func_matrix.cpp
  distance_func_matrix.cpp
  collision_data.cpp
  collision_node.cpp
  collision_object.cpp
  BV/RSS.cpp
  BV/AABB.cpp
  BV/kIOS.cpp
  BV/kDOP.cpp
  BV/OBBRSS.cpp
  BV/OBB.cpp
  narrowphase/gjk.cpp
  narrowphase/minkowski_difference.cpp
  narrowphase/support_functions.cpp
  shape/geometric_shapes.cpp
  shape/geometric_shapes_utility.cpp
  distance/box_halfspace.cpp
  distance/box_plane.cpp
  distance/box_sphere.cpp
  distance/capsule_capsule.cpp
  distance/capsule_halfspace.cpp
  distance/capsule_plane.cpp
  distance/cone_halfspace.cpp
  distance/cone_plane.cpp
  distance/cylinder_halfspace.cpp
  distance/cylinder_plane.cpp
  distance/truncated_cone_halfspace.cpp
  distance/truncated_cone_plane.cpp
  distance/sphere_sphere.cpp
  distance/sphere_cylinder.cpp
  distance/sphere_halfspace.cpp
  distance/sphere_plane.cpp
  distance/sphere_capsule.cpp
  distance/ellipsoid_halfspace.cpp
  distance/ellipsoid_plane.cpp
  distance/convex_halfspace.cpp
  distance/convex_plane.cpp
  distance/triangle_halfspace.cpp
  distance/triangle_plane.cpp
  distance/triangle_triangle.cpp
  distance/triangle_sphere.cpp
  distance/halfspace_plane.cpp
  distance/plane_plane.cpp
  distance/halfspace_halfspace.cpp
  intersect.cpp
  math/transform.cpp
  traversal/traversal_recurse.cpp
  distance.cpp
  BVH/BVH_utility.cpp
  BVH/BV_fitter.cpp
  BVH/BVH_model.cpp
  BVH/BV_splitter.cpp
  collision_func_matrix.cpp
  collision_utility.cpp
  hfield.cpp
)

SRC_ARGS=()
for f in "${COAL_SOURCES[@]}"; do SRC_ARGS+=("$SRC_ROOT/src/$f"); done

em++ coal_wasm.cpp "${SRC_ARGS[@]}" \
  -I "$SRC_ROOT/include" \
  -I "$GEN_INC" \
  -I "$EIGEN_INC" \
  -I "$BOOST_INC" \
  -std=c++17 -O2 \
  -DNDEBUG -DCOAL_STATIC \
  -lembind \
  -s MODULARIZE=1 -s EXPORT_NAME=CoalModule -s ENVIRONMENT=web \
  -s ALLOW_MEMORY_GROWTH=1 -s SINGLE_FILE=1 \
  -o coal_wasm.js \
  "$@"

echo "Built visual/wasm/coal_wasm.js"
