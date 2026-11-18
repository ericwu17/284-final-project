#include "colliders.h"

namespace Jello {

void resolveContactsVelocityLevel(std::vector<Vector3D> &x, std::vector<Vector3D> &v,
                                  const std::vector<const Collider *> &colliders, double t) {
  for (const Collider *c : colliders) {
    for (size_t i = 0; i < x.size(); i++) {
      ContactQuery q = c->query(x[i], t);
      if (q.phi >= 0) continue;
      x[i] -= q.phi * q.normal;  // back onto the surface
      double vn = dot(v[i] - q.velocity, q.normal);
      if (vn < 0) v[i] -= vn * q.normal;  // stop moving into the surface
    }
  }
}

} // namespace Jello
