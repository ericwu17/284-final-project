#include "softBody.h"

namespace Jello {

SoftBody::SoftBody(const TetMesh &mesh, double density) : mesh_(mesh), density_(density) {
  const int n = mesh_.numVerts();
  mass.assign(n, 0.0);
  for (size_t t = 0; t < mesh_.tets.size(); t++) {
    double share = density_ * mesh_.restVolumes[t] / 4.0;
    for (int i : mesh_.tets[t]) mass[i] += share;
  }
  invMass.resize(n);
  for (int i = 0; i < n; i++) invMass[i] = mass[i] > 0 ? 1.0 / mass[i] : 0.0;
  reset();
}

void SoftBody::reset() {
  x = mesh_.rest;
  v.assign(mesh_.numVerts(), Vector3D(0, 0, 0));
}

double SoftBody::totalMass() const {
  double m = 0;
  for (double mi : mass) m += mi;
  return m;
}

} // namespace Jello
