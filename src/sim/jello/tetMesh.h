#ifndef JELLO_TET_MESH_H
#define JELLO_TET_MESH_H

#include <array>
#include <string>
#include <vector>

#include "CGL/vector3D.h"

namespace Jello {

using CGL::Vector3D;

// Signed volume of the tet (x0, x1, x2, x3):
//   V = (1/6) (x1 - x0) . ((x2 - x0) x (x3 - x0))
// Positive when (x1 - x0, x2 - x0, x3 - x0) is a right-handed frame.
double tetVolume(const Vector3D &x0, const Vector3D &x1, const Vector3D &x2,
                 const Vector3D &x3);

// A tetrahedral mesh of a box, built on a regular grid.
//
// Every grid cell is split into 5 tets: 4 corner tets plus 1 central tet.
// The split is mirrored between neighboring cells (by the parity of i+j+k) so
// the face diagonals of neighboring cells match and the mesh is conforming.
// Edges are the cell edges plus one diagonal per cell face; there are no body
// diagonals. 8x8x8 cells give 729 vertices and 2,560 tets.
//
// All indices refer to `rest` (and to any same-sized array of current
// positions), so solvers can use them directly.
struct TetMesh {
  // Grid the mesh was built from.
  int nx = 0, ny = 0, nz = 0;
  Vector3D size;    // box side lengths (meters)
  Vector3D origin;  // minimum corner of the box

  std::vector<Vector3D> rest;                   // rest position of each vertex
  std::vector<std::array<int, 4>> tets;         // all positively oriented
  std::vector<double> restVolumes;              // one per tet
  std::vector<std::array<int, 2>> edges;        // unique, edge[0] < edge[1]
  std::vector<std::array<int, 3>> surfaceTris;  // boundary faces, counter-clockwise seen from outside
  std::vector<int> surfaceVerts;                // vertices used by surfaceTris, ascending

  void buildGrid(int nx, int ny, int nz, const Vector3D &size, const Vector3D &origin);

  int numVerts() const { return (int)rest.size(); }
  int vertexId(int i, int j, int k) const { return i + (nx + 1) * (j + (ny + 1) * k); }

  // Sum of signed tet volumes at positions x (x.size() == numVerts()).
  double volume(const std::vector<Vector3D> &x) const;
  // Volume enclosed by surfaceTris at positions x, by the divergence theorem.
  // Equals volume(x) whenever the surface is closed and outward-wound.
  double surfaceVolume(const std::vector<Vector3D> &x) const;
};

// Structural checks for a mesh built by TetMesh::buildGrid: counts, volumes,
// conformity, and that the surface is a closed, outward-wound 2-manifold.
// The path tracer's inside/outside test depends on that last property.
struct MeshCheck {
  std::vector<std::string> failures;  // empty means every check passed
  // Summary numbers, for the selftest table.
  double volumeRelError = 0;
  double surfaceVolumeRelError = 0;
  double minTetVolume = 0, maxTetVolume = 0;
  int eulerCharacteristic = 0;
};
MeshCheck checkGridMesh(const TetMesh &mesh);

} // namespace Jello

#endif // JELLO_TET_MESH_H
