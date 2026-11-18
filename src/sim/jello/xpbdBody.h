#ifndef JELLO_XPBD_BODY_H
#define JELLO_XPBD_BODY_H

#include "softBody.h"

namespace Jello {

struct XPBDParams {
  // Edge compliance alpha = 1 / stiffness (m/N). 1/300 is the same material as
  // the mass-spring default ks = 300 N/m; 0 makes every edge rigid.
  double edgeCompliance = 1.0 / 300.0;
  // Edge damping beta (N s/m): damps relative motion along each edge only,
  // the XPBD counterpart of the mass-spring kd.
  double edgeDamping = 0.02;
  int substeps = 100; // substeps per output frame, one constraint pass each
};

// XPBD soft body (Macklin, Mueller, Chentanez 2016) in the "small steps" form
// (Macklin et al. 2019): many substeps, one Gauss-Seidel pass over the
// constraints per substep, Lagrange multipliers reset every substep.
// Per substep h:
//   v += h g;  p = x;  x += h v;            predict
//   for each edge: C = |xa - xb| - L0        project distance constraints
//     dlambda = (-C - gamma gradC.(x - p)) / ((1 + gamma)(wa + wb) + alpha/h^2)
//     xa += wa dlambda n,  xb -= wb dlambda n,  gamma = alpha beta / h
//   push penetrating vertices out of colliders
//   v = (x - p) / h
// The update is a position projection, so it stays stable for any
// compliance >= 0 and any h; accuracy (not stability) depends on substeps.
// Volume constraints are added in Stage X2.
class XPBDBody : public SoftBody {
 public:
  XPBDBody(const TetMesh &mesh, double density, const XPBDParams &params);

  std::string name() const override { return "xpbd"; }
  void step(double frameDt, const SimContext &ctx) override;
  std::vector<SolverParam> params() override;

  XPBDParams p;

 private:
  void substep(double h, double t, const SimContext &ctx);
  void solveEdges(double h);

  std::vector<double> restLength;  // one per mesh edge
  std::vector<Vector3D> prev;      // positions at the start of the substep
  bool forwardSweep = false;       // flips every substep
};

} // namespace Jello

#endif // JELLO_XPBD_BODY_H
