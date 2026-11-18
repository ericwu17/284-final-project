#include "massSpringBody.h"

#include <algorithm>

namespace Jello {

MassSpringBody::MassSpringBody(const TetMesh &mesh, double density, const MassSpringParams &params)
    : SoftBody(mesh, density), p(params) {
  restLength.reserve(mesh_.edges.size());
  for (const auto &e : mesh_.edges) restLength.push_back((mesh_.rest[e[1]] - mesh_.rest[e[0]]).norm());
  force.resize(mesh_.numVerts());
}

std::vector<SolverParam> MassSpringBody::params() {
  std::vector<SolverParam> out(3);
  out[0].name = "ks";       out[0].units = "N/m";   out[0].value = &p.ks;
  out[1].name = "kd";       out[1].units = "N s/m"; out[1].value = &p.kd;
  out[2].name = "substeps"; out[2].intValue = &p.substeps; out[2].minValue = 1;
  return out;
}

void MassSpringBody::step(double frameDt, const SimContext &ctx) {
  const int n = std::max(1, p.substeps);
  const double h = frameDt / n;
  for (int s = 0; s < n; s++) substep(h, ctx.time + s * h, ctx);
}

void MassSpringBody::substep(double h, double t, const SimContext &ctx) {
  const int nv = (int)x.size();

  // Gravity.
  for (int i = 0; i < nv; i++) force[i] = mass[i] * ctx.gravity;

  // Springs: Hooke's law plus damping of the relative velocity along the
  // spring. Positive when stretched, so a is pulled toward b and b toward a.
  for (size_t e = 0; e < mesh_.edges.size(); e++) {
    const int a = mesh_.edges[e][0], b = mesh_.edges[e][1];
    Vector3D d = x[b] - x[a];
    double len = d.norm();
    if (len <= 0) continue;
    Vector3D dir = d / len;
    double magnitude = p.ks * (len - restLength[e]) + p.kd * dot(v[b] - v[a], dir);
    Vector3D f = magnitude * dir;
    force[a] += f;
    force[b] -= f;
  }

  // Semi-implicit Euler: new velocity first, then position from it.
  for (int i = 0; i < nv; i++) {
    v[i] += h * invMass[i] * force[i];
    x[i] += h * v[i];
  }

  resolveContactsVelocityLevel(x, v, ctx.colliders, t + h);
}

} // namespace Jello
