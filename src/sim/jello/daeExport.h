#ifndef JELLO_DAE_EXPORT_H
#define JELLO_DAE_EXPORT_H

#include <string>

#include "colliders.h"
#include "softBody.h"

namespace Jello {

// Writes one COLLADA (.dae) file per frame for the path tracer, by filling in
// a template scene (dae/jello/template.dae) with the jello's surface and the
// plate's pose. Everything is in sim coordinates (meters, Y-up); the template's
// "sim" node maps them into the render scene, so the simulator never needs to
// know the scene's units.
//
// The jello is written as its boundary surface: the surface vertices only,
// re-indexed 0..n-1 in ascending mesh order (the path tracer's halfedge
// builder needs every index in 0..n-1 used), and the outward-wound surface
// triangles. Positions are shared between triangles, so the loader's smooth
// vertex normals are continuous across the surface.
//
// Template placeholders, each of which must appear exactly once:
//   {{JELLO_FLOAT_COUNT}}  3 * number of surface vertices
//   {{JELLO_POS_COUNT}}    number of surface vertices
//   {{JELLO_POSITIONS}}    x y z per vertex, %.6f
//   {{JELLO_TRI_COUNT}}    number of surface triangles
//   {{JELLO_VCOUNT}}       "3 3 3 ..." (one per triangle)
//   {{JELLO_INDICES}}      three vertex indices per triangle
//   {{PLATE_MATRIX}}       row-major translate(center) * scale(size)
// For scenes without a plate, everything from the line containing
// "PLATE_BEGIN" through the line containing "PLATE_END" is removed.
class DaeExporter {
 public:
  // Reads and checks the template. On failure returns false and sets *error.
  bool loadTemplate(const std::string &path, std::string *error);

  // Writes the scene at time t to `path`. plate may be null.
  bool writeFrame(const std::string &path, const SoftBody &body, const KinematicPlate *plate,
                  double t, std::string *error) const;

  // The filled-in document (writeFrame without the file), for tests.
  std::string fill(const SoftBody &body, const KinematicPlate *plate, double t) const;

 private:
  std::string tmpl;
};

} // namespace Jello

#endif // JELLO_DAE_EXPORT_H
