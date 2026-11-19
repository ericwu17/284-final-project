#ifndef CGL_INTERSECT_H
#define CGL_INTERSECT_H

#include <vector>

#include "CGL/vector3D.h"
#include "CGL/misc.h"

#include "pathtracer/bsdf.h"

namespace CGL {
    
namespace SceneObjects {

class Primitive;

/**
 * A record of an intersection point which includes the time of intersection
 * and other information needed for shading
 */
struct Intersection {

  Intersection() : t (INF_D), primitive(NULL), bsdf(NULL) { }

  double t;    ///< time of intersection

  const Primitive* primitive;  ///< the primitive intersected

  Vector3D n;  ///< shading normal at point of intersection (interpolated for meshes)

  /// Geometric normal: the triangle's face normal, oriented to the same side
  /// as its vertex normals (outward for a closed, outward-wound mesh such as
  /// the jello), or the sphere's normal. Never interpolated, so it tells which
  /// side of the surface a ray is on even where the smooth normal is bent.
  Vector3D ng;

  BSDF* bsdf; ///< BSDF of the surface at point of intersection

  // More to follow.
};

} // namespace SceneObjects
} // namespace CGL

#endif // CGL_INTERSECT_H
