#ifndef JELLO_COLLIDERS_H
#define JELLO_COLLIDERS_H

#include <string>
#include <vector>

#include "CGL/vector3D.h"

namespace Jello {

using CGL::Vector3D;

// What a collider reports about a point p at time t.
struct ContactQuery {
  double phi;         // signed distance: < 0 means p is inside (penetrating)
  Vector3D normal;    // unit outward normal at the closest surface point
  Vector3D velocity;  // velocity of the collider surface there (0 if static)
};

// A kinematic (scripted, not simulated) obstacle. Both solvers only talk to
// colliders through query(), so a moving plate (Stage C2) plugs in like the
// ground does.
class Collider {
 public:
  explicit Collider(double friction) : friction(friction) {}
  virtual ~Collider() = default;
  virtual std::string name() const = 0;
  virtual ContactQuery query(const Vector3D &p, double t) const = 0;

  double friction;  // Coulomb coefficient (used from Stage C1)
};

// Infinite horizontal plane y = height, solid below.
class GroundPlane : public Collider {
 public:
  GroundPlane(double height, double friction) : Collider(friction), height(height) {}
  std::string name() const override { return "ground"; }
  ContactQuery query(const Vector3D &p, double) const override {
    return {p.y - height, Vector3D(0, 1, 0), Vector3D(0, 0, 0)};
  }

  double height;
};

// Velocity-level contact for explicit integrators (mass-spring), applied
// after the position update. For each penetrating vertex: move it back to the
// surface along the normal, then remove the part of its velocity (relative to
// the collider) that points into the surface. That makes contact perfectly
// inelastic in the normal direction; the bounce comes from the springs.
// No friction yet (Stage C1).
void resolveContactsVelocityLevel(std::vector<Vector3D> &x, std::vector<Vector3D> &v,
                                  const std::vector<const Collider *> &colliders, double t);

} // namespace Jello

#endif // JELLO_COLLIDERS_H
