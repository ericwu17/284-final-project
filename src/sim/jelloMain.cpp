// jellosim: headless command-line driver for the jello simulator.
//
// Mesh selftest and dump (Stage P1), headless simulation with the stability
// check (Stage M1), per-frame metrics CSV and run.json (Stage Q1). Per-frame
// mesh export comes in Stage P3; see docs/milestone-plan.md.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "jello/jelloScene.h"
#include "json.hpp"
#include "jello/metrics.h"
#include "jello/tetMesh.h"

using namespace Jello;
using CGL::Vector3D;

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
  printf("  %s --run SCENE.json [--seconds T] [--set KEY=VALUE]... [--every N] [--quiet]\n", bin);
  printf("                 [--out DIR] [--label TEXT]\n");
  printf("      Simulate T seconds (default 3) at 30 output frames/s, checking\n");
  printf("      stability every frame (NaN, inverted tet, bbox > 2x rest).\n");
  printf("      --set overrides a scene value, e.g. --set mass_spring.ks=1000.\n");
  printf("      --every N prints a row every N frames (default 15).\n");
  printf("      --out DIR writes DIR/metrics.csv (one row per output frame, frame 0\n");
  printf("      is the initial state) and DIR/run.json (scene, overrides, final\n");
  printf("      parameters, git commit, result). --label names the run in run.json\n");
  printf("      (scripts/plot_metrics.py uses it as the legend entry).\n");
  printf("      Ends with one 'RESULT key=value ...' line. Exit status 0 if\n");
  printf("      stable, 3 if unstable.\n");
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

  // Volume gradients vs central finite differences on random (non-degenerate) tets.
  std::srand(1234);
  auto rnd = []() { return 2.0 * std::rand() / RAND_MAX - 1.0; };
  double worst = 0;
  const int trials = 1000;
  for (int t = 0; t < trials; t++) {
    Vector3D x[4];
    for (auto &p : x) p = 0.05 * Vector3D(rnd(), rnd(), rnd());
    Vector3D g[4];
    tetVolumeGradients(x[0], x[1], x[2], x[3], g);
    double scale = 0;
    for (auto &gi : g) scale = std::max(scale, gi.norm());
    if (scale < 1e-6) continue;  // skip near-degenerate tets
    const double h = 1e-7;
    for (int i = 0; i < 4; i++) {
      for (int a = 0; a < 3; a++) {
        Vector3D xp[4] = {x[0], x[1], x[2], x[3]}, xm[4] = {x[0], x[1], x[2], x[3]};
        xp[i][a] += h;
        xm[i][a] -= h;
        double fd = (tetVolume(xp[0], xp[1], xp[2], xp[3]) - tetVolume(xm[0], xm[1], xm[2], xm[3])) / (2 * h);
        worst = std::max(worst, std::abs(fd - g[i][a]) / scale);
      }
    }
  }
  bool gradOk = worst < 1e-6;
  printf("Volume gradient vs finite differences, %d random tets: max relative error %.1e  %s\n",
         trials, worst, gradOk ? "PASS" : "FAIL");
  if (!gradOk) failed++;
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

// First line of a shell command's output, or "" if it fails.
std::string shellLine(const char *cmd) {
  FILE *p = popen(cmd, "r");
  if (!p) return "";
  char buf[256] = {0};
  std::string out = fgets(buf, sizeof(buf), p) ? buf : "";
  pclose(p);
  while (!out.empty() && (out.back() == '\n' || out.back() == '\r')) out.pop_back();
  return out;
}

int run(int argc, char **argv) {
  std::string scenePath;
  double seconds = 3.0;
  bool quiet = false;
  int every = 15;  // print a row every this many frames
  std::string outDir, label;
  std::vector<std::pair<std::string, std::string>> overrides;
  for (int i = 2; i < argc; i++) {
    std::string a = argv[i];
    if (a == "--seconds" && i + 1 < argc) {
      seconds = atof(argv[++i]);
    } else if (a == "--set" && i + 1 < argc) {
      std::string kv = argv[++i];
      size_t eq = kv.find('=');
      if (eq == std::string::npos) {
        fprintf(stderr, "--set expects KEY=VALUE, got %s\n", kv.c_str());
        return 2;
      }
      overrides.push_back({kv.substr(0, eq), kv.substr(eq + 1)});
    } else if (a == "--every" && i + 1 < argc) {
      every = std::max(1, atoi(argv[++i]));
    } else if (a == "--out" && i + 1 < argc) {
      outDir = argv[++i];
    } else if (a == "--label" && i + 1 < argc) {
      label = argv[++i];
    } else if (a == "--quiet") {
      quiet = true;
    } else if (scenePath.empty() && a[0] != '-') {
      scenePath = a;
    } else {
      fprintf(stderr, "unknown argument %s\n", a.c_str());
      return 2;
    }
  }
  if (scenePath.empty()) {
    fprintf(stderr, "--run needs a scene file\n");
    return 2;
  }

  JelloScene scene;
  std::string error;
  std::unique_ptr<SoftBody> body;
  if (!scene.load(scenePath, &error, overrides) || !(body = scene.buildBody(&error))) {
    fprintf(stderr, "Error: %s\n", error.c_str());
    return 2;
  }
  auto colliders = scene.buildColliders();
  SimContext ctx;
  ctx.gravity = scene.gravity;
  for (const auto &c : colliders) ctx.colliders.push_back(c.get());
  double maxVolErr = 0;  // max |V - V0| / V0 over output frames, percent

  const double fps = 30.0;
  const int frames = (int)std::lround(seconds * fps);
  int substeps = 1;
  if (scene.solver == "mass_spring") substeps = scene.massSpring.substeps;
  if (scene.solver == "xpbd") substeps = scene.xpbd.substeps;
  const double dt = 1.0 / fps / substeps;

  MetricsLogger log;
  if (!outDir.empty()) {
    if (!log.open(outDir + "/metrics.csv")) {
      fprintf(stderr, "Error: cannot write %s/metrics.csv\n", outDir.c_str());
      return 2;
    }
    log.record(measureFrame(*body, 0, 0.0, 0.0));
  }

  if (!quiet) {
    printf("%s: solver %s, %zu tets, mass %.3f kg, dt %.3g s\n", scenePath.c_str(),
           body->name().c_str(), body->mesh().tets.size(), body->totalMass(), dt);
    printf("%8s %10s %10s %10s %10s %10s %9s %9s %11s\n", "t (s)", "com y (m)", "height (m)",
           "bbox ratio", "com x (mm)", "com z (mm)", "yaw (deg)", "tilt (deg)", "vol err (%)");
  }

  Stability st;
  double simMs = 0;
  int frame = 0;
  for (; frame < frames; frame++) {
    ctx.time = frame / fps;
    auto t0 = std::chrono::steady_clock::now();
    body->step(1.0 / fps, ctx);
    double stepMs = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    simMs += stepMs;
    st = checkStability(*body);
    FrameMetrics m = measureFrame(*body, frame + 1, (frame + 1) / fps, stepMs);
    log.record(m);
    if (st.ok) maxVolErr = std::max(maxVolErr, std::abs(m.volErrPct));
    if (!quiet && ((frame + 1) % every == 0 || !st.ok)) {
      printf("%8.3f %10.4f %10.4f %10.3f %10.3f %10.3f %9.3f %9.3f %11.4f\n", m.t, m.com.y,
             m.height, st.bboxRatio, m.com.x * 1000, m.com.z * 1000, m.yawDeg, m.tiltDeg,
             m.volErrPct);
    }
    if (!st.ok) break;
  }
  int done = std::min(frame + 1, frames);
  std::string failT = st.ok ? "none" : std::to_string((frame + 1) / fps);

  printf("RESULT solver=%s stable=%d seconds=%.4f fail_t=%s dt=%.6g ms_per_frame=%.4f "
         "max_vol_err_pct=%.4f reason=\"%s\"\n",
         body->name().c_str(), st.ok ? 1 : 0, done / fps, failT.c_str(), dt, simMs / done,
         maxVolErr, st.reason.c_str());

  if (!outDir.empty()) {
    log.close();
    using nlohmann::json;
    json r;
    r["label"] = label.empty() ? scenePath : label;
    r["scene_file"] = scenePath;
    json ov = json::object();
    for (const auto &o : overrides) ov[o.first] = o.second;
    r["overrides"] = ov;
    r["scene"] = json::parse(scene.toJson());
    r["seconds_requested"] = seconds;
    r["output_fps"] = fps;
    r["substeps"] = substeps;
    r["dt"] = dt;
    r["mesh"] = {{"verts", body->mesh().numVerts()}, {"tets", body->mesh().tets.size()},
                 {"edges", body->mesh().edges.size()}, {"mass_kg", body->totalMass()}};
    r["result"] = {{"stable", st.ok}, {"seconds", done / fps}, {"fail_t", failT},
                   {"reason", st.reason}, {"ms_per_frame", simMs / done},
                   {"max_vol_err_pct", maxVolErr}, {"frames_logged", log.rows()}};
    r["git_commit"] = shellLine("git rev-parse --short HEAD 2>/dev/null");
    r["git_dirty"] = !shellLine("git status --porcelain --untracked-files=no 2>/dev/null").empty();
    std::ofstream out(outDir + "/run.json");
    out << r.dump(2) << "\n";
    if (!quiet) printf("wrote %s/metrics.csv (%d rows) and %s/run.json\n", outDir.c_str(),
                       log.rows(), outDir.c_str());
  }
  return st.ok ? 0 : 3;
}

} // namespace

int main(int argc, char **argv) {
  if (argc >= 2 && !strcmp(argv[1], "--selftest")) return selftest();
  if (argc >= 2 && !strcmp(argv[1], "--run")) return run(argc, argv);
  if (argc == 6 && !strcmp(argv[1], "--dump-mesh"))
    return dumpMesh(atoi(argv[2]), atoi(argv[3]), atoi(argv[4]), argv[5]);
  usage(argv[0]);
  return 2;
}
