#ifndef JELLO_METRICS_H
#define JELLO_METRICS_H

#include <string>

#include "softBody.h"

namespace Jello {

// The proposal's stability rule: a run is unstable if any position is NaN,
// any tet is inverted (signed volume <= 0), or the bounding box grows past
// twice its rest size along any axis.
// (The per-frame metrics logger is added here in Stage Q1.)
struct Stability {
  bool ok = true;
  std::string reason;      // empty when ok
  bool hasNaN = false;
  int invertedTets = 0;
  double bboxRatio = 1;    // max over axes of current extent / rest extent
};

Stability checkStability(const SoftBody &body, double maxBBoxRatio = 2.0);

} // namespace Jello

#endif // JELLO_METRICS_H
