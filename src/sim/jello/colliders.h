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

  double friction;  // Coulomb coefficient mu (static = kinetic)
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

// The shared contact model. Both solvers apply the same rule, at the level
// their integrator works at, for every vertex with phi < 0:
//   1. Move the vertex back onto the surface along the normal.
//   2. Remove its normal motion into the surface (relative to the collider's
//      own motion), which makes contact inelastic; the bounce comes from the
//      elastic forces.
//   3. Coulomb friction: reduce its tangential motion (relative to the
//      collider) by at most mu times the normal correction from step 2;
//      if that is enough to stop it, it sticks (static friction).
// mu = 0 gives frictionless sliding. One coefficient serves as both the
// static and the kinetic one.

// Velocity level, for explicit integrators (mass-spring), after the position
// update. The normal correction is the removed normal velocity |dv_n|.
void resolveContactsVelocityLevel(std::vector<Vector3D> &x, std::vector<Vector3D> &v,
                                  const std::vector<const Collider *> &colliders, double t);

// Position level, for XPBD, inside each substep after the constraint solve.
// `prev` holds the positions at the start of the substep (length h); the
// motion over the substep is x - prev, and the collider moves by u*h. The
// normal correction is the penetration depth -phi that was projected away.
void resolveContactsPositionLevel(std::vector<Vector3D> &x, const std::vector<Vector3D> &prev,
                                  const std::vector<const Collider *> &colliders, double t,
                                  double h);

} // namespace Jello

#endif // JELLO_COLLIDERS_H
