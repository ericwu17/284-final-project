// jellosim: headless command-line driver for the jello simulator.
//
// Only mesh tools for now (Stage P1). Simulation and per-frame export are
// added in later stages; see docs/milestone-plan.md.

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "jello/tetMesh.h"

using namespace Jello;

namespace {

void usage(const char *bin) {
  printf("Usage:\n");
  printf("  %s --selftest\n", bin);
  printf("      Build tet meshes at several resolutions and check counts, volumes,\n");
  printf("      conformity, and that the surface is closed and outward-wound.\n");
  printf("      Exits with status 1 if any check fails.\n");
  printf("  %s --dump-mesh NX NY NZ FILE.json\n", bin);
  printf("      Write a unit-cube mesh with NX x NY x NZ cells as JSON\n");
  printf("      (for scripts/plot_tetmesh.py).\n");
}

int selftest() {
  struct Case { int nx, ny, nz; };
  // Single cell, both parities, non-cubic grids, the 8^3 demo mesh, and the
  // ~20k-tet mesh planned for the performance study.
  const std::vector<Case> cases = {{1, 1, 1}, {2, 2, 2}, {3, 5, 2}, {4, 8, 6},
                                   {8, 8, 8}, {12, 12, 12}, {16, 16, 16}, {20, 20, 20}};
  // A 10 cm jello block; the checks are relative, so the size is only cosmetic.
  const Vector3D size(0.1, 0.1, 0.1), origin(-0.05, 0.0, -0.05);

  printf("Tet mesh selftest: 10 cm box, 5-tet split per cell (mirrored by cell parity)\n\n");
  printf("%-9s %7s %7s %7s %9s %9s %7s %9s %9s %8s  %s\n", "grid", "verts", "tets", "edges",
         "surf-tri", "surf-vert", "V-E+F", "vol err", "surf err", "ms", "result");
  printf("%s\n", std::string(104, '-').c_str());

  int failed = 0;
  for (const Case &c : cases) {
    TetMesh mesh;
    auto t0 = std::chrono::steady_clock::now();
    mesh.buildGrid(c.nx, c.ny, c.nz, size, origin);
    double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    MeshCheck check = checkGridMesh(mesh);

    char grid[32];
    snprintf(grid, sizeof(grid), "%dx%dx%d", c.nx, c.ny, c.nz);
    printf("%-9s %7d %7zu %7zu %9zu %9zu %7d %9.1e %9.1e %8.2f  %s\n", grid, mesh.numVerts(),
           mesh.tets.size(), mesh.edges.size(), mesh.surfaceTris.size(), mesh.surfaceVerts.size(),
           check.eulerCharacteristic, check.volumeRelError, check.surfaceVolumeRelError, ms,
           check.failures.empty() ? "PASS" : "FAIL");
    for (const std::string &f : check.failures) printf("          ! %s\n", f.c_str());
    if (!check.failures.empty()) failed++;
  }

  printf("\nChecks per mesh: vertex/tet/edge/surface counts match the grid formulas\n");
  printf("(edges = cell edges + one diagonal per lattice face, i.e. conforming);\n");
  printf("every tet has positive volume (cell/6 corner, cell/3 central); tet volumes\n");
  printf("sum to the box volume; the surface encloses the same volume (divergence\n");
  printf("theorem); every directed surface edge appears once with its reverse\n");
  printf("(closed, consistently wound 2-manifold); Euler characteristic is 2; and\n");
  printf("every surface normal points out of the box.\n");
  printf("\n%s: %zu of %zu meshes passed\n", failed ? "FAILED" : "OK", cases.size() - failed,
         cases.size());
  return failed ? 1 : 0;
}

int dumpMesh(int nx, int ny, int nz, const char *path) {
  TetMesh mesh;
  mesh.buildGrid(nx, ny, nz, Vector3D(1, 1, 1), Vector3D(0, 0, 0));
  std::ofstream out(path);
  if (!out) {
    fprintf(stderr, "Could not open %s\n", path);
    return 1;
  }
  auto writeList = [&](const char *name, const auto &items, bool last = false) {
    out << "  \"" << name << "\": [";
    for (size_t i = 0; i < items.size(); i++) {
      out << (i ? ", " : "") << "[";
      for (size_t j = 0; j < items[i].size(); j++) out << (j ? ", " : "") << items[i][j];
      out << "]";
    }
    out << "]" << (last ? "\n" : ",\n");
  };
  std::vector<std::array<double, 3>> positions;
  for (const Vector3D &p : mesh.rest) positions.push_back({p.x, p.y, p.z});

  out << "{\n  \"cells\": [" << nx << ", " << ny << ", " << nz << "],\n";
  writeList("positions", positions);
  writeList("tets", mesh.tets);
  writeList("edges", mesh.edges);
  writeList("surface_tris", mesh.surfaceTris, true);
  out << "}\n";
  printf("Wrote %s: %d verts, %zu tets, %zu edges, %zu surface triangles\n", path,
         mesh.numVerts(), mesh.tets.size(), mesh.edges.size(), mesh.surfaceTris.size());
  return 0;
}

} // namespace

int main(int argc, char **argv) {
  if (argc >= 2 && !strcmp(argv[1], "--selftest")) return selftest();
  if (argc == 6 && !strcmp(argv[1], "--dump-mesh"))
    return dumpMesh(atoi(argv[2]), atoi(argv[3]), atoi(argv[4]), argv[5]);
  usage(argv[0]);
  return 2;
}
