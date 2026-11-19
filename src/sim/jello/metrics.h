#ifndef JELLO_METRICS_H
#define JELLO_METRICS_H

#include <cstdio>
#include <string>

#include "softBody.h"

namespace Jello {

// The proposal's stability rule: a run is unstable if any position is NaN,
// any tet is inverted (signed volume <= 0), or the bounding box grows past
// twice its rest size along any axis.
struct Stability {
  bool ok = true;
  std::string reason;      // empty when ok
  bool hasNaN = false;
  int invertedTets = 0;
  double bboxRatio = 1;    // max over axes of current extent / rest extent
};

Stability checkStability(const SoftBody &body, double maxBBoxRatio = 2.0);

// Everything measured about one output frame (one row of metrics.csv).
struct FrameMetrics {
  int frame = 0;               // 0 is the initial state
  double t = 0;                // simulated time (s)
  double volume = 0;           // sum of signed tet volumes (m^3)
  double volErrPct = 0;        // (volume - rest volume) / rest volume, percent
  Vector3D com;                // mass-weighted center of mass (m)
  double topY = 0;             // mean y of the vertices that start on the top face (m)
  double topX = 0, topZ = 0;   // mean x, z of those vertices (m)
  double bottomX = 0, bottomZ = 0;  // mean x, z of the bottom-face vertices (m)
  double height = 0;           // vertical extent, max y - min y (m)
  double minTetVolRatio = 1;   // min over tets of V / V0 (<= 0 means inverted)
  int nInverted = 0;           // tets with V <= 0
  double bboxRatio = 1;        // as in Stability
  bool hasNaN = false;
  double stepMs = 0;           // wall-clock time to simulate this frame (0 for frame 0)
  // Orientation of the top front edge (rest direction +x): yaw is its rotation
  // about the vertical axis, tilt its angle above horizontal. Shows the
  // spurious spin XPBD picks up at low substep counts (Stage X2).
  double yawDeg = 0, tiltDeg = 0;
};

// Measures the body as it is now. NaN positions give NaN sums; hasNaN says so.
FrameMetrics measureFrame(const SoftBody &body, int frame, double t, double stepMs);

// Writes FrameMetrics rows to a CSV file, one per output frame, with a header:
//   frame,t,volume,vol_err_pct,com_x,com_y,com_z,top_y,height,
//   min_tet_vol_ratio,n_inverted,bbox_ratio,has_nan,step_ms,yaw_deg,tilt_deg,
//   top_x,top_z,bottom_x,bottom_z
// Used by jellosim --out and by the viewer's metrics toggle (key M).
class MetricsLogger {
 public:
  ~MetricsLogger() { close(); }
  // Creates the parent directories if needed. Returns false if the file
  // cannot be opened.
  bool open(const std::string &path);
  void record(const FrameMetrics &m);
  void close();
  bool isOpen() const { return file != nullptr; }
  const std::string &path() const { return path_; }
  int rows() const { return rows_; }

 private:
  FILE *file = nullptr;
  std::string path_;
  int rows_ = 0;
};

// mkdir -p. Returns false on failure.
bool makeDirs(const std::string &dir);

} // namespace Jello

#endif // JELLO_METRICS_H
