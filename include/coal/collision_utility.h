// Copyright (c) 2017 CNRS
// Copyright (c) 2022 INRIA
// Authors: Joseph Mirabel (joseph.mirabel@laas.fr)
//
// This file is part of Coal.
// Coal is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// Coal is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Lesser Public License for more details.  You should have
// received a copy of the GNU Lesser General Public License along with
// Coal. If not, see <http://www.gnu.org/licenses/>.

#ifndef COAL_COLLISION_UTILITY_H
#define COAL_COLLISION_UTILITY_H

#include "coal/collision_object.h"

namespace coal {

COAL_DLLAPI CollisionGeometry* extract(const CollisionGeometry* model,
                                       const Transform3s& pose,
                                       const AABB& aabb);

/**
 * \brief Returns the name associated to a NODE_TYPE
 */
inline const char* get_node_type_name(NODE_TYPE node_type) {
  // Note: this array is indexed positionally by the NODE_TYPE enum value, so
  // it must have exactly one entry per enumerator (GEOM_CONVEX16 and
  // GEOM_CONVEX32 included separately, even though GEOM_CONVEX is an alias
  // for GEOM_CONVEX32) or every name after a missing entry silently shifts.
  static const char* node_type_name_all[] = {
      "BV_UNKNOWN",       "BV_AABB",         "BV_OBB",        "BV_RSS",
      "BV_kIOS",          "BV_OBBRSS",       "BV_KDOP16",     "BV_KDOP18",
      "BV_KDOP24",        "GEOM_BOX",        "GEOM_SPHERE",   "GEOM_CAPSULE",
      "GEOM_CONE",        "GEOM_CYLINDER",   "GEOM_CONVEX16", "GEOM_CONVEX",
      "GEOM_PLANE",       "GEOM_HALFSPACE",  "GEOM_TRIANGLE", "GEOM_OCTREE",
      "GEOM_ELLIPSOID",   "GEOM_TRUNCATEDCONE",
      "HF_AABB",          "HF_OBBRSS",       "NODE_COUNT"};

  return node_type_name_all[node_type];
}

/**
 * \brief Returns the name associated to a OBJECT_TYPE
 */
inline const char* get_object_type_name(OBJECT_TYPE object_type) {
  static const char* object_type_name_all[] = {
      "OT_UNKNOWN", "OT_BVH", "OT_GEOM", "OT_OCTREE", "OT_HFIELD", "OT_COUNT"};

  return object_type_name_all[object_type];
}

}  // namespace coal

#endif  // COAL_COLLISION_UTILITY_H
