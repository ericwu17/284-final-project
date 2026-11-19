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

// An axis-aligned box that moves without rotating: the shaking plate.
// Its center follows
//   c(t) = c0 + A r(tau) sin(2 pi f tau) axis,  tau = t - start (0 before start),
// where r ramps linearly from 0 to 1 over `ramp` seconds. With the ramp the
// velocity starts at 0 instead of jumping to 2 pi f A, so the block is not
// hit by an impulse when shaking begins.
struct PlateShake {
  Vector3D axis = Vector3D(1, 0, 0);  // unit direction of motion
  double amplitude = 0;               // A (m); 0 = a static plate
  double frequency = 3;               // f (Hz)
  double start = 1;                   // seconds
  double ramp = 0.5;                  // seconds to reach full amplitude (0 = none)
};

class KinematicPlate : public Collider {
 public:
  KinematicPlate(const Vector3D &center, const Vector3D &size, const PlateShake &shake,
                 double friction)
      : Collider(friction), center0(center), halfSize(0.5 * size), shake(shake) {}
  std::string name() const override { return "plate"; }

  // Signed distance to the box, outward normal, and the plate's velocity.
  // Inside the box, the normal is that of the nearest face, except that any
  // point inside the upper half of the box, within the footprint, is pushed
  // out through the top face. A vertex resting near the edge of the plate
  // then cannot be shoved out sideways through a side face.
  ContactQuery query(const Vector3D &p, double t) const override;

  Vector3D centerAt(double t) const;
  Vector3D velocityAt(double t) const;
  Vector3D size() const { return 2 * halfSize; }

  Vector3D center0;
  Vector3D halfSize;
  PlateShake shake;
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
