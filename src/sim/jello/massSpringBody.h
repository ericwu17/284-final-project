#ifndef JELLO_MASS_SPRING_BODY_H
#define JELLO_MASS_SPRING_BODY_H

#include "softBody.h"

namespace Jello {

struct MassSpringParams {
  double ks = 300;    // spring stiffness, N/m (same for every tet edge)
  double kd = 0.02;   // spring damping along the spring axis, N s/m
  int substeps = 100; // integration steps per output frame
};

// Mass-spring soft body: one spring per tet edge (cell edges and face
// diagonals), lumped masses, semi-implicit (symplectic) Euler:
//   v += h f / m,  x += h v,  then contacts.
// Explicit, so the time step h = frameDt / substeps must stay below about
// 2 / omega_max ~ 2 sqrt(m / k_eff): stiffer springs need more substeps.
class MassSpringBody : public SoftBody {
 public:
  MassSpringBody(const TetMesh &mesh, double density, const MassSpringParams &params);

  std::string name() const override { return "mass_spring"; }
  void step(double frameDt, const SimContext &ctx) override;
  std::vector<SolverParam> params() override;

  MassSpringParams p;

 private:
  void substep(double h, double t, const SimContext &ctx);

  std::vector<double> restLength;  // one per mesh edge
  std::vector<Vector3D> force;     // scratch, one per vertex
};

} // namespace Jello

#endif // JELLO_MASS_SPRING_BODY_H
