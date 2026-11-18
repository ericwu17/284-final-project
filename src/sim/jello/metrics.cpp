#include "metrics.h"

#include <sys/stat.h>

#include <algorithm>
#include <cerrno>
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

FrameMetrics measureFrame(const SoftBody &body, int frame, double t, double stepMs) {
  FrameMetrics m;
  m.frame = frame;
  m.t = t;
  m.stepMs = stepMs;
  const TetMesh &mesh = body.mesh();
  const std::vector<Vector3D> &x = body.x;

  // Volumes.
  m.minTetVolRatio = INFINITY;
  for (size_t i = 0; i < mesh.tets.size(); i++) {
    const auto &tet = mesh.tets[i];
    double V = tetVolume(x[tet[0]], x[tet[1]], x[tet[2]], x[tet[3]]);
    m.volume += V;
    m.minTetVolRatio = std::min(m.minTetVolRatio, V / mesh.restVolumes[i]);
    if (V <= 0) m.nInverted++;
  }
  double restVolume = 0;  // the same sum at rest, so a body at rest reads exactly 0
  for (double v0 : mesh.restVolumes) restVolume += v0;
  m.volErrPct = 100.0 * (m.volume - restVolume) / restVolume;

  // Center of mass, extent, NaN.
  Vector3D com(0, 0, 0);
  Vector3D lo(INFINITY, INFINITY, INFINITY), hi(-INFINITY, -INFINITY, -INFINITY);
  for (size_t i = 0; i < x.size(); i++) {
    const Vector3D &p = x[i];
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) m.hasNaN = true;
    com += body.mass[i] * p;
    for (int a = 0; a < 3; a++) {
      lo[a] = std::min(lo[a], p[a]);
      hi[a] = std::max(hi[a], p[a]);
    }
  }
  m.com = com / body.totalMass();
  m.height = hi.y - lo.y;
  for (int a = 0; a < 3; a++) m.bboxRatio = std::max(m.bboxRatio, (hi[a] - lo[a]) / mesh.size[a]);
  if (m.hasNaN) m.bboxRatio = NAN;

  // Top face: vertices with grid index j == ny, tracked by rest index.
  double topSum = 0;
  for (int k = 0; k <= mesh.nz; k++)
    for (int i = 0; i <= mesh.nx; i++) topSum += x[mesh.vertexId(i, mesh.ny, k)].y;
  m.topY = topSum / ((mesh.nx + 1) * (mesh.nz + 1));

  Vector3D u = x[mesh.vertexId(mesh.nx, mesh.ny, mesh.nz)] - x[mesh.vertexId(0, mesh.ny, mesh.nz)];
  m.yawDeg = atan2(-u.z, u.x) * 180 / M_PI;
  m.tiltDeg = asin(u.y / u.norm()) * 180 / M_PI;
  return m;
}

bool makeDirs(const std::string &dir) {
  if (dir.empty()) return true;
  std::string partial;
  std::stringstream ss(dir);
  std::string part;
  if (dir[0] == '/') partial = "/";
  while (std::getline(ss, part, '/')) {
    if (part.empty()) continue;
    partial += part + "/";
    if (mkdir(partial.c_str(), 0755) != 0 && errno != EEXIST) return false;
  }
  return true;
}

bool MetricsLogger::open(const std::string &path) {
  close();
  size_t slash = path.find_last_of('/');
  if (slash != std::string::npos && !makeDirs(path.substr(0, slash))) return false;
  file = fopen(path.c_str(), "w");
  if (!file) return false;
  path_ = path;
  rows_ = 0;
  fprintf(file, "frame,t,volume,vol_err_pct,com_x,com_y,com_z,top_y,height,"
                "min_tet_vol_ratio,n_inverted,bbox_ratio,has_nan,step_ms,yaw_deg,tilt_deg\n");
  return true;
}

void MetricsLogger::record(const FrameMetrics &m) {
  if (!file) return;
  fprintf(file, "%d,%.6f,%.9e,%.6f,%.7f,%.7f,%.7f,%.7f,%.7f,%.6f,%d,%.6f,%d,%.4f,%.4f,%.4f\n",
          m.frame, m.t, m.volume, m.volErrPct, m.com.x, m.com.y, m.com.z, m.topY, m.height,
          m.minTetVolRatio, m.nInverted, m.bboxRatio, m.hasNaN ? 1 : 0, m.stepMs, m.yawDeg,
          m.tiltDeg);
  rows_++;
}

void MetricsLogger::close() {
  if (file) fclose(file);
  file = nullptr;
}

} // namespace Jello
