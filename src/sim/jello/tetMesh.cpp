#include "tetMesh.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <sstream>
#include <unordered_map>

namespace Jello {

double tetVolume(const Vector3D &x0, const Vector3D &x1, const Vector3D &x2,
                 const Vector3D &x3) {
  return dot(x1 - x0, cross(x2 - x0, x3 - x0)) / 6.0;
}

void tetVolumeGradients(const Vector3D &x0, const Vector3D &x1, const Vector3D &x2,
                        const Vector3D &x3, Vector3D grad[4]) {
  const Vector3D e1 = x1 - x0, e2 = x2 - x0, e3 = x3 - x0;
  grad[1] = cross(e2, e3) / 6.0;
  grad[2] = cross(e3, e1) / 6.0;
  grad[3] = cross(e1, e2) / 6.0;
  grad[0] = -(grad[1] + grad[2] + grad[3]);
}

namespace {

// Cell corners are numbered c = dx + 2*dy + 4*dz, with dx, dy, dz in {0, 1}.
//
// Even cells (i+j+k even): the central tet uses the 4 corners with an odd
// number of set bits {1, 2, 4, 7}; each of the other 4 corners gets a corner
// tet with its 3 axis neighbors. Odd cells use the mirror image. On a shared
// face, both cells then pick the same diagonal, so the mesh is conforming.
// Vertex order here is arbitrary; buildGrid fixes the orientation.
const int EVEN_CELL_TETS[5][4] = {
    {0, 1, 2, 4}, {3, 1, 2, 7}, {5, 1, 4, 7}, {6, 2, 4, 7}, {1, 2, 4, 7}};
const int ODD_CELL_TETS[5][4] = {
    {1, 0, 3, 5}, {2, 0, 3, 6}, {4, 0, 5, 6}, {7, 3, 5, 6}, {0, 3, 5, 6}};

uint64_t edgeKey(int a, int b) {
  if (a > b) std::swap(a, b);
  return ((uint64_t)a << 32) | (uint32_t)b;
}

// Key for an unordered triangle. 21 bits per index (up to ~2M vertices).
uint64_t faceKey(int a, int b, int c) {
  int v[3] = {a, b, c};
  std::sort(v, v + 3);
  return ((uint64_t)v[0] << 42) | ((uint64_t)v[1] << 21) | (uint64_t)v[2];
}

// The 4 faces of a tet, each paired with the opposite vertex.
const int TET_FACES[4][4] = {{1, 2, 3, 0}, {0, 2, 3, 1}, {0, 1, 3, 2}, {0, 1, 2, 3}};

} // namespace

std::vector<std::array<int, 3>> boundaryFaces(const std::vector<Vector3D> &pos,
                                              const std::vector<std::array<int, 4>> &tets) {
  // Count every tet face, then collect the ones seen once, in tet order.
  std::unordered_map<uint64_t, int> faceCount;
  faceCount.reserve(tets.size() * 4);
  for (const auto &t : tets)
    for (const auto &f : TET_FACES) faceCount[faceKey(t[f[0]], t[f[1]], t[f[2]])]++;

  std::vector<std::array<int, 3>> out;
  for (const auto &t : tets) {
    for (const auto &f : TET_FACES) {
      int a = t[f[0]], b = t[f[1]], c = t[f[2]], opposite = t[f[3]];
      if (faceCount[faceKey(a, b, c)] != 1) continue;
      // Wind the triangle so its normal points away from the tet's 4th vertex.
      Vector3D n = cross(pos[b] - pos[a], pos[c] - pos[a]);
      if (dot(n, pos[opposite] - pos[a]) > 0) std::swap(b, c);
      out.push_back({a, b, c});
    }
  }
  return out;
}

void TetMesh::buildGrid(int nx_, int ny_, int nz_, const Vector3D &size_,
                        const Vector3D &origin_) {
  nx = nx_; ny = ny_; nz = nz_;
  size = size_; origin = origin_;
  rest.clear(); tets.clear(); restVolumes.clear();
  edges.clear(); surfaceTris.clear(); surfaceVerts.clear();

  // Vertices, x fastest (matches vertexId).
  const Vector3D h(size.x / nx, size.y / ny, size.z / nz);
  rest.reserve((size_t)(nx + 1) * (ny + 1) * (nz + 1));
  for (int k = 0; k <= nz; k++)
    for (int j = 0; j <= ny; j++)
      for (int i = 0; i <= nx; i++)
        rest.push_back(origin + Vector3D(i * h.x, j * h.y, k * h.z));

  // Tets, 5 per cell, flipped to positive orientation where needed.
  tets.reserve((size_t)5 * nx * ny * nz);
  for (int k = 0; k < nz; k++) {
    for (int j = 0; j < ny; j++) {
      for (int i = 0; i < nx; i++) {
        int corner[8];
        for (int c = 0; c < 8; c++)
          corner[c] = vertexId(i + (c & 1), j + ((c >> 1) & 1), k + ((c >> 2) & 1));
        const int(*pattern)[4] = ((i + j + k) % 2 == 0) ? EVEN_CELL_TETS : ODD_CELL_TETS;
        for (int t = 0; t < 5; t++) {
          std::array<int, 4> tet = {corner[pattern[t][0]], corner[pattern[t][1]],
                                    corner[pattern[t][2]], corner[pattern[t][3]]};
          if (tetVolume(rest[tet[0]], rest[tet[1]], rest[tet[2]], rest[tet[3]]) < 0)
            std::swap(tet[2], tet[3]);
          tets.push_back(tet);
        }
      }
    }
  }

  restVolumes.reserve(tets.size());
  for (const auto &t : tets)
    restVolumes.push_back(tetVolume(rest[t[0]], rest[t[1]], rest[t[2]], rest[t[3]]));

  // Unique edges, in ascending (a, b) order.
  std::vector<uint64_t> edgeKeys;
  edgeKeys.reserve(tets.size() * 6);
  for (const auto &t : tets)
    for (int a = 0; a < 4; a++)
      for (int b = a + 1; b < 4; b++) edgeKeys.push_back(edgeKey(t[a], t[b]));
  std::sort(edgeKeys.begin(), edgeKeys.end());
  edgeKeys.erase(std::unique(edgeKeys.begin(), edgeKeys.end()), edgeKeys.end());
  edges.reserve(edgeKeys.size());
  for (uint64_t key : edgeKeys) edges.push_back({(int)(key >> 32), (int)(key & 0xffffffffu)});

  surfaceTris = boundaryFaces(rest, tets);

  for (const auto &tri : surfaceTris)
    for (int v : tri) surfaceVerts.push_back(v);
  std::sort(surfaceVerts.begin(), surfaceVerts.end());
  surfaceVerts.erase(std::unique(surfaceVerts.begin(), surfaceVerts.end()), surfaceVerts.end());
}

double TetMesh::volume(const std::vector<Vector3D> &x) const {
  double v = 0;
  for (const auto &t : tets) v += tetVolume(x[t[0]], x[t[1]], x[t[2]], x[t[3]]);
  return v;
}

double TetMesh::surfaceVolume(const std::vector<Vector3D> &x) const {
  // Sum of signed volumes of tets formed by each face and the origin.
  double v = 0;
  for (const auto &tri : surfaceTris) v += dot(x[tri[0]], cross(x[tri[1]], x[tri[2]])) / 6.0;
  return v;
}

MeshCheck checkGridMesh(const TetMesh &m) {
  MeshCheck r;
  auto expect = [&](bool ok, const std::string &what) {
    if (!ok) r.failures.push_back(what);
  };
  auto num = [](double v) { std::ostringstream s; s << v; return s.str(); };

  const long nx = m.nx, ny = m.ny, nz = m.nz;
  const long V = (long)m.rest.size(), T = (long)m.tets.size();

  // Counts.
  expect(V == (nx + 1) * (ny + 1) * (nz + 1), "vertex count");
  expect(T == 5 * nx * ny * nz, "tet count != 5 per cell");
  // Cell edges plus exactly one diagonal per lattice face (conforming split, no body diagonals).
  long axisEdges = nx * (ny + 1) * (nz + 1) + (nx + 1) * ny * (nz + 1) + (nx + 1) * (ny + 1) * nz;
  long latticeFaces = nx * ny * (nz + 1) + nx * (ny + 1) * nz + (nx + 1) * ny * nz;
  expect((long)m.edges.size() == axisEdges + latticeFaces,
         "edge count " + num(m.edges.size()) + " != " + num(axisEdges + latticeFaces) +
             " (non-conforming face diagonals?)");
  expect((long)m.surfaceTris.size() == 4 * (nx * ny + ny * nz + nz * nx),
         "surface triangle count " + num(m.surfaceTris.size()));
  long interior = std::max(0L, nx - 1) * std::max(0L, ny - 1) * std::max(0L, nz - 1);
  expect((long)m.surfaceVerts.size() == V - interior, "surface vertex count");

  // Volumes.
  double boxVolume = m.size.x * m.size.y * m.size.z;
  double cellVolume = boxVolume / (nx * ny * nz);
  r.minTetVolume = 1e300; r.maxTetVolume = -1e300;
  for (size_t t = 0; t < m.tets.size(); t++) {
    r.minTetVolume = std::min(r.minTetVolume, m.restVolumes[t]);
    r.maxTetVolume = std::max(r.maxTetVolume, m.restVolumes[t]);
  }
  expect(r.minTetVolume > 0, "a tet has non-positive volume");
  // In the 5-tet split, corner tets are 1/6 of a cell and the central tet 1/3.
  expect(std::abs(r.minTetVolume - cellVolume / 6) < 1e-9 * cellVolume &&
             std::abs(r.maxTetVolume - cellVolume / 3) < 1e-9 * cellVolume,
         "tet volumes are not cell/6 and cell/3");
  double tetSum = m.volume(m.rest);
  r.volumeRelError = std::abs(tetSum - boxVolume) / boxVolume;
  expect(r.volumeRelError < 1e-10, "sum of tet volumes != box volume");
  r.surfaceVolumeRelError = std::abs(m.surfaceVolume(m.rest) - tetSum) / boxVolume;
  expect(r.surfaceVolumeRelError < 1e-10, "surface (divergence) volume != tet volume");

  // Closed, consistently wound 2-manifold: every directed edge appears exactly
  // once, and so does its reverse.
  std::unordered_map<uint64_t, int> directed;  // (a << 32 | b) -> count
  for (const auto &tri : m.surfaceTris)
    for (int e = 0; e < 3; e++)
      directed[((uint64_t)tri[e] << 32) | (uint32_t)tri[(e + 1) % 3]]++;
  bool manifold = true;
  for (const auto &kv : directed) {
    uint64_t a = kv.first >> 32, b = kv.first & 0xffffffffu;
    auto rev = directed.find((b << 32) | a);
    if (kv.second != 1 || rev == directed.end() || rev->second != 1) manifold = false;
  }
  expect(manifold, "surface is not a closed, consistently wound 2-manifold");

  long surfaceEdges = (long)directed.size() / 2;
  r.eulerCharacteristic = (int)((long)m.surfaceVerts.size() - surfaceEdges + (long)m.surfaceTris.size());
  expect(r.eulerCharacteristic == 2, "surface Euler characteristic != 2");

  // Outward: each face normal points away from the box center.
  Vector3D center = m.origin + 0.5 * m.size;
  bool outward = true;
  for (const auto &tri : m.surfaceTris) {
    const Vector3D &a = m.rest[tri[0]], &b = m.rest[tri[1]], &c = m.rest[tri[2]];
    Vector3D n = cross(b - a, c - a);
    if (dot(n, (a + b + c) / 3.0 - center) <= 0) outward = false;
  }
  expect(outward, "a surface triangle faces inward");

  return r;
}

} // namespace Jello
