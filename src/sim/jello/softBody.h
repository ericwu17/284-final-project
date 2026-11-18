#ifndef JELLO_SOFT_BODY_H
#define JELLO_SOFT_BODY_H

#include <memory>
#include <string>
#include <vector>

#include "tetMesh.h"

namespace Jello {

// Everything outside the body that a solver step needs.
struct SimContext {
  Vector3D gravity = Vector3D(0, -9.8, 0);
  double time = 0;  // simulated time at the start of the step (seconds)
};

// A deformable body on a tet mesh. Solvers (mass-spring, XPBD) subclass this.
// State is index-based: x[i], v[i], mass[i] belong to mesh().rest[i].
class SoftBody {
 public:
  SoftBody(const TetMesh &mesh, double density);
  virtual ~SoftBody() = default;

  virtual std::string name() const = 0;
  // Advance the state by one output frame (frameDt seconds). Solvers choose
  // their own substeps inside.
  virtual void step(double frameDt, const SimContext &ctx) = 0;
  // Back to the rest pose, at rest.
  virtual void reset();

  const TetMesh &mesh() const { return mesh_; }
  double density() const { return density_; }
  double totalMass() const;

  std::vector<Vector3D> x;       // positions
  std::vector<Vector3D> v;       // velocities
  std::vector<double> mass;      // lumped: each tet gives density * V / 4 to each vertex
  std::vector<double> invMass;   // 1 / mass (0 would mean pinned)

 protected:
  TetMesh mesh_;
  double density_;
};

// Placeholder solver: holds the rest pose. Used to check the viewer and the
// export pipeline before the real solvers exist.
class StaticBody : public SoftBody {
 public:
  using SoftBody::SoftBody;
  std::string name() const override { return "static"; }
  void step(double, const SimContext &) override {}
};

// Creates the solver named in a scene file ("static" for now; "mass_spring"
// and "xpbd" come in later stages). Returns nullptr for an unknown name.
std::unique_ptr<SoftBody> makeSoftBody(const std::string &solver, const TetMesh &mesh,
                                       double density);

} // namespace Jello

#endif // JELLO_SOFT_BODY_H
