#include "metrics.h"

#include <algorithm>
#include <cmath>
#include <sstream>

namespace Jello {

Stability checkStability(const SoftBody &body, double maxBBoxRatio) {
  Stability s;
  const TetMesh &mesh = body.mesh();

  Vector3D lo(INFINITY, INFINITY, INFINITY), hi(-INFINITY, -INFINITY, -INFINITY);
  for (const Vector3D &p : body.x) {
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
      s.hasNaN = true;
      continue;
    }
    for (int a = 0; a < 3; a++) {
      lo[a] = std::min(lo[a], p[a]);
      hi[a] = std::max(hi[a], p[a]);
    }
  }
  if (!s.hasNaN) {
    for (int a = 0; a < 3; a++) s.bboxRatio = std::max(s.bboxRatio, (hi[a] - lo[a]) / mesh.size[a]);
    for (const auto &t : mesh.tets)
      if (tetVolume(body.x[t[0]], body.x[t[1]], body.x[t[2]], body.x[t[3]]) <= 0) s.invertedTets++;
  }

  std::ostringstream why;
  if (s.hasNaN) why << "position is NaN/inf";
  else if (s.invertedTets > 0) why << s.invertedTets << " inverted tet(s)";
  else if (s.bboxRatio > maxBBoxRatio) why << "bounding box " << s.bboxRatio << "x rest size";
  s.reason = why.str();
  s.ok = s.reason.empty();
  return s;
}

} // namespace Jello
