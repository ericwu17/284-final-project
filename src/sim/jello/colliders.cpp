#include "colliders.h"

#include <algorithm>

namespace Jello {

namespace {

// Shrinks the tangential vector `vt` by `amount` (not past zero): Coulomb
// friction with the bound mu * (normal correction) already in `amount`.
Vector3D applyFriction(const Vector3D &vt, double amount) {
  double len = vt.norm();
  if (len <= amount || len == 0) return Vector3D(0, 0, 0);  // sticks
  return vt * (1.0 - amount / len);                        // slides, slower
}

} // namespace

void resolveContactsVelocityLevel(std::vector<Vector3D> &x, std::vector<Vector3D> &v,
                                  const std::vector<const Collider *> &colliders, double t) {
  for (const Collider *c : colliders) {
    for (size_t i = 0; i < x.size(); i++) {
      ContactQuery q = c->query(x[i], t);
      if (q.phi >= 0) continue;
      x[i] -= q.phi * q.normal;  // back onto the surface

      Vector3D vr = v[i] - q.velocity;  // relative to the collider
      double vn = dot(vr, q.normal);
      if (vn >= 0) continue;  // already separating: no impulse, no friction
      Vector3D vt = vr - vn * q.normal;
      // Normal velocity -> 0 (inelastic); tangential reduced by mu * |vn|.
      v[i] = q.velocity + applyFriction(vt, c->friction * -vn);
    }
  }
}

void resolveContactsPositionLevel(std::vector<Vector3D> &x, const std::vector<Vector3D> &prev,
                                  const std::vector<const Collider *> &colliders, double t,
                                  double h) {
  for (const Collider *c : colliders) {
    for (size_t i = 0; i < x.size(); i++) {
      ContactQuery q = c->query(x[i], t);
      if (q.phi >= 0) continue;
      x[i] -= q.phi * q.normal;  // back onto the surface
      if (c->friction <= 0) continue;

      // Tangential motion this substep, relative to the collider's motion.
      Vector3D d = (x[i] - prev[i]) - h * q.velocity;
      Vector3D dt = d - dot(d, q.normal) * q.normal;
      x[i] += applyFriction(dt, c->friction * -q.phi) - dt;
    }
  }
}

} // namespace Jello
