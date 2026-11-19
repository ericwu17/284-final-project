#include "colliders.h"

#include <algorithm>
#include <cmath>

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

Vector3D KinematicPlate::centerAt(double t) const {
  double tau = t - shake.start;
  if (shake.amplitude == 0 || tau <= 0) return center0;
  double r = shake.ramp > 0 ? std::min(1.0, tau / shake.ramp) : 1.0;
  return center0 + shake.amplitude * r * std::sin(2 * M_PI * shake.frequency * tau) * shake.axis;
}

Vector3D KinematicPlate::velocityAt(double t) const {
  double tau = t - shake.start;
  if (shake.amplitude == 0 || tau <= 0) return Vector3D(0, 0, 0);
  const double w = 2 * M_PI * shake.frequency;
  double r = 1, dr = 0;
  if (shake.ramp > 0 && tau < shake.ramp) {
    r = tau / shake.ramp;
    dr = 1 / shake.ramp;
  }
  // d/dt [A r sin(w tau)] = A (r' sin(w tau) + r w cos(w tau))
  return shake.amplitude * (dr * std::sin(w * tau) + r * w * std::cos(w * tau)) * shake.axis;
}

ContactQuery KinematicPlate::query(const Vector3D &p, double t) const {
  ContactQuery out;
  out.velocity = velocityAt(t);
  Vector3D d = p - centerAt(t);
  Vector3D q(std::abs(d.x) - halfSize.x, std::abs(d.y) - halfSize.y, std::abs(d.z) - halfSize.z);
  auto sign = [](double v) { return v < 0 ? -1.0 : 1.0; };

  if (q.x > 0 || q.y > 0 || q.z > 0) {
    // Outside: distance to the nearest point of the box.
    Vector3D o(std::max(q.x, 0.0), std::max(q.y, 0.0), std::max(q.z, 0.0));
    out.phi = o.norm();
    out.normal = Vector3D(sign(d.x) * o.x, sign(d.y) * o.y, sign(d.z) * o.z) / out.phi;
    return out;
  }
  // Inside: upper half goes out through the top; otherwise the nearest face.
  if (d.y >= 0) {
    out.phi = q.y;
    out.normal = Vector3D(0, 1, 0);
    return out;
  }
  int axis = (q.x >= q.y && q.x >= q.z) ? 0 : (q.y >= q.z ? 1 : 2);
  out.phi = q[axis];
  out.normal = Vector3D(0, 0, 0);
  out.normal[axis] = sign(d[axis]);
  return out;
}

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
