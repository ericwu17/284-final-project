#include "xpbdBody.h"

#include <algorithm>

namespace Jello {

XPBDBody::XPBDBody(const TetMesh &mesh, double density, const XPBDParams &params)
    : SoftBody(mesh, density), p(params) {
  restLength.reserve(mesh_.edges.size());
  for (const auto &e : mesh_.edges) restLength.push_back((mesh_.rest[e[1]] - mesh_.rest[e[0]]).norm());
  prev.resize(mesh_.numVerts());
}

std::vector<SolverParam> XPBDBody::params() {
  std::vector<SolverParam> out(5);
  out[0].name = "edge compliance"; out[0].units = "m/N";   out[0].value = &p.edgeCompliance;
  out[1].name = "edge damping";    out[1].units = "N s/m"; out[1].value = &p.edgeDamping;
  out[2].name = "substeps";        out[2].intValue = &p.substeps; out[2].minValue = 1;
  out[3].name = "volume";          out[3].boolValue = &p.volume;
  out[4].name = "vol compliance";  out[4].units = "1/Pa";  out[4].value = &p.volumeCompliance;
  return out;
}

void XPBDBody::step(double frameDt, const SimContext &ctx) {
  const int n = std::max(1, p.substeps);
  const double h = frameDt / n;
  for (int s = 0; s < n; s++) substep(h, ctx.time + s * h, ctx);
}

void XPBDBody::substep(double h, double t, const SimContext &ctx) {
  const int nv = (int)x.size();

  // Predict: explicit step under gravity alone.
  for (int i = 0; i < nv; i++) {
    v[i] += h * ctx.gravity;
    prev[i] = x[i];
    x[i] += h * v[i];
  }

  solveEdges(h);
  if (p.volume) solveVolumes(h);

  // Contacts (shared model, colliders.h): project penetrating vertices back
  // onto the surface, then position-level Coulomb friction. The velocity
  // update below then has no component into the surface, so contact is
  // inelastic, as in the mass-spring solver.
  resolveContactsPositionLevel(x, prev, ctx.colliders, t + h, h);

  for (int i = 0; i < nv; i++) v[i] = (x[i] - prev[i]) / h;
}

void XPBDBody::solveVolumes(double h) {
  // Same sweep direction as the edge pass that just ran (solveEdges flipped it).
  const size_t m = mesh_.tets.size();
  for (size_t k = 0; k < m; k++) {
    const size_t t = forwardSweep ? k : m - 1 - k;
    const auto &tet = mesh_.tets[t];
    Vector3D grad[4];
    tetVolumeGradients(x[tet[0]], x[tet[1]], x[tet[2]], x[tet[3]], grad);

    double denom = p.volumeCompliance * mesh_.restVolumes[t] / (h * h);
    for (int i = 0; i < 4; i++) denom += invMass[tet[i]] * grad[i].norm2();
    if (denom <= 0) continue;

    double C = tetVolume(x[tet[0]], x[tet[1]], x[tet[2]], x[tet[3]]) - mesh_.restVolumes[t];
    double dLambda = -C / denom;
    for (int i = 0; i < 4; i++) x[tet[i]] += invMass[tet[i]] * dLambda * grad[i];
  }
}

void XPBDBody::solveEdges(double h) {
  const double alphaTilde = p.edgeCompliance / (h * h);
  const double gamma = p.edgeCompliance * p.edgeDamping / h;

  // Alternate the sweep direction every substep (symmetric Gauss-Seidel). A
  // fixed order biases each pass toward the last-solved edges; on a symmetric
  // drop that bias shows up as a spurious spin on a frictionless ground.
  const size_t m = mesh_.edges.size();
  forwardSweep = !forwardSweep;
  for (size_t k = 0; k < m; k++) {
    const size_t e = forwardSweep ? k : m - 1 - k;
    const int a = mesh_.edges[e][0], b = mesh_.edges[e][1];
    const double wa = invMass[a], wb = invMass[b];
    const double wSum = wa + wb;
    if (wSum == 0) continue;

    Vector3D d = x[a] - x[b];
    double len = d.norm();
    if (len <= 0) continue;
    Vector3D n = d / len;  // gradient of C with respect to x[a]; -n for x[b]

    double C = len - restLength[e];
    double dampTerm = gamma * dot(n, (x[a] - prev[a]) - (x[b] - prev[b]));
    double dLambda = (-C - dampTerm) / ((1 + gamma) * wSum + alphaTilde);

    x[a] += wa * dLambda * n;
    x[b] -= wb * dLambda * n;
  }
}

} // namespace Jello
