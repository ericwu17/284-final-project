# Milestone Plan: Weeks 1–2 (through the Nov 22 checkpoint)

This plan covers everything the [proposal](https://ericwu17.github.io/cs284-final-project-proposal/) schedules for Weeks 1 and 2. The work is split into small stages. Each stage ends with something you can run and see, plus a set of captures (screenshots, recordings, plots) saved in a fixed way so the milestone report, slides, video, and final paper can reuse them.

**Milestone goal (from the proposal):** both scenarios (jello dropped onto a plate; jello on a shaking plate) run end to end with both solvers, with volume terms, contact, metrics logging, and first path-traced sequences. Deliverables: SIGGRAPH-template report, 1-minute video, 2–3 slides.

---

## 0. Where the code stands today (commit `8ab0378`)

| Area | State | What it means for us |
|---|---|---|
| `src/sim/` (HW4) | Complete: Verlet, springs, plane/sphere collision, self-collision. | Good reference, but it is built around `Cloth`/`PointMass` (pointers, `last_position`, Provot clamp). The jello solvers need a cleaner particle layout (indices, explicit velocity). |
| `src/sim` viewer | NanoGUI viewer draws `cloth->clothMesh`. Keys in use: `R`, `P`, `N`, `Space`, `Esc`. No way to take screenshots. `png.cpp` is not compiled, but CGL ships `lodepng`. | We need to add screenshot and frame-recording keys (Stage 0). |
| `src/render/` (HW3-1) | Complete: BVH, Möller–Trumbore, direct lighting (hemisphere and importance sampling), global illumination, Russian roulette, adaptive sampling. | Usable as is for diffuse scenes. |
| `src/render/` (HW3-2) | **Not implemented.** `reflect`, `refract`, `MirrorBSDF`, `RefractionBSDF`, `GlassBSDF`, and `MicrofacetBSDF` in [advanced_bsdf.cpp](../src/render/pathtracer/advanced_bsdf.cpp) are all TODO stubs. | The dielectric BSDF is written from scratch (Stage R1). |
| Integrator and specular paths | [pathtracer.cpp](../src/render/pathtracer/pathtracer.cpp) multiplies by `w_in.z` (negative for transmitted rays), never adds emission after a specular bounce, and applies Russian roulette at 0.65 on every bounce. | Must be fixed, or the jello renders black or very noisy (Stage R1). |
| `Intersection` | Has only the interpolated shading normal `n`. No geometric normal. | Add a geometric normal `ng` to tell inside from outside (Stage R3). |
| Mesh normals | The `.dae` loader **ignores file normals**. [gl_scene/mesh.cpp](../src/render/scene/gl_scene/mesh.cpp) builds a `HalfedgeMesh` and recomputes area-weighted vertex normals. | The exporter must write **shared, indexed vertices** on a **closed, consistently outward-wound** surface. Smooth normals then come for free. |
| `.dae` scenes | Cornell boxes are `Z_UP`. Each `<node>` has a row-major `<matrix>`. `<polylist>` and `<triangles>` are both parsed. | Use the node matrix to map sim coordinates (Y-up, meters) into the scene, so the sim never deals with scene units. |
| Build | `build/` contains only `clothsim`. `pathtracer` has not been built yet. | Stage 0 builds both. |

**Calendar reality:** the Week 1 items (tet meshing, both solvers, dielectric) are not in the repo yet, so all of Weeks 1–2 fits into **Wed Nov 18 – Sun Nov 22**. The schedule in §5 runs four tracks in parallel to make that work. Section 6 lists what to cut if we fall behind.

---

## 1. Architecture (agree on this on Day 1)

```
scene/jello/*.json ──► jello_core (C++, no GL) ──────────► out/sim/<run>/frame_####.dae ──► pathtracer ──► out/render/<run>/####.png ──► ffmpeg ──► media/
                         TetMesh, SoftBody,                 out/sim/<run>/metrics.csv  ──► scripts/plot_metrics.py ──────────────────────────► media/
                         MassSpringBody, XPBDBody,
                         Colliders, Metrics, DaeExporter
                              ▲                 ▲
                   clothsim (GUI viewer)   jellosim (headless CLI)
```

**New files** (all physics code stays free of nanogui and OpenGL so the headless runner can link it):

| File | Contents |
|---|---|
| `src/sim/jello/tetMesh.{h,cpp}` | `TetMesh`: rest positions, `tets` (int4), unique `edges` (int2), `surfaceTris` (int3, outward), `surfaceVerts` (map to compact indices), rest volumes. `buildGrid(nx, ny, nz, size, origin)`. |
| `src/sim/jello/softBody.h` | Abstract `SoftBody`: `x`, `v`, `invMass` (`std::vector`, index based), `step(double frameDt, const SimContext&)`, `reset()`, `params()`. |
| `src/sim/jello/massSpringBody.{h,cpp}` | Springs on tet edges, spring damping, per-tet volume penalty, semi-implicit Euler. |
| `src/sim/jello/xpbdBody.{h,cpp}` | Edge-distance and tet-volume constraints with compliance, substepping. |
| `src/sim/jello/colliders.{h,cpp}` | `Collider::query(p, t) → {phi, normal, velocity}`. `GroundPlane` and `KinematicPlate` (a box moving sinusoidally). |
| `src/sim/jello/metrics.{h,cpp}` | Per-frame volume, center of mass, top height, inversions, NaN, bounding box, timing. Writes CSV. Includes the stability check. |
| `src/sim/jello/daeExporter.{h,cpp}` | Fills the template `.dae` with the surface mesh and the plate matrix for each frame. |
| `src/sim/jello/jelloScene.{h,cpp}` | Parses the `"jello"`, `"solver"`, `"plate"`, `"ground"`, and `"export"` JSON blocks (reuses `json.hpp`). |
| `src/sim/jelloMain.cpp` | Headless `jellosim` CLI: `jellosim -f scene/jello/drop.json --solver xpbd -n 120 -o out/sim/drop_xpbd`. |
| `dae/jello/template.dae` | Studio scene (start from `CBempty.dae`) with `{{...}}` placeholders. |
| `scripts/render_sequence.py`, `scripts/make_video.sh`, `scripts/plot_metrics.py`, `scripts/log_capture.py` | Batch rendering, video assembly, plots, and the capture log. |

CMake: add a `jello_core` static library in `src/sim/CMakeLists.txt` and link it into both `clothsim` and a new `jellosim` target (CGL only, no nanogui). It picks up `CGL_CXX_FLAGS` from the top level, which matters because the AVX flags change the layout of `Vector3D`.

**Conventions:** SI units (meters, kg, seconds), Y-up, gravity (0, −9.8, 0). Particles are referenced by index, never by pointer (the cloth code needed `reserve()` to keep pointers valid; avoid that trap). The sim is deterministic: fixed timestep and no `rand()`. Every headless run writes `out/sim/<run>/run.json` with a copy of the scene JSON, the final parameters, the git hash, and a dirty flag.

**Shared JSON parameters** (starting values; tune in the GUI):

```json
{
  "jello":  { "size": [0.1, 0.1, 0.1], "cells": [8, 8, 8], "origin": [-0.05, 0.15, -0.05], "density": 1050 },
  "solver": "xpbd",
  "mass_spring": { "ks": 300, "kd": 0.02, "kv": 1e5, "volume": true, "substeps": 100 },
  "xpbd":   { "edge_compliance": 0.00333, "edge_damping": 0.02, "volume_compliance": 0.0, "volume": true, "substeps": 100 },
  "ground": { "y": 0.0, "friction": 0.5 },
  "plate":  { "size": [0.25, 0.01, 0.25], "center": [0, 0.005, 0], "friction": 0.6,
              "shake": { "axis": [1, 0, 0], "amplitude": 0.015, "frequency": 3.0, "start": 1.0 } },
  "export": { "fps": 30, "frames": 120, "template": "dae/jello/template.dae" }
}
```

`scene/jello/drop.json` uses `amplitude: 0`. `scene/jello/shake.json` starts the block about 2 mm above the plate and begins shaking at `t = 1 s`, after the block settles.

---

## 2. Capture system (applies to every stage)

### Folder layout

```
media/
  INDEX.md                       one row per capture (the master list)
  S0_baseline/                   one folder per stage, named <StageID>_<slug>
    S0-01_cbspheres-lambertian.png
  P1_tetmesh/
    P1-01_selftest-output.txt
    P1-02_tet-wireframe.png
  ...
out/                             (git-ignored) raw sim frames, raw PNG sequences, scratch renders
```

### Rules

1. **Capture ID** = `<StageID>-<NN>` (for example `X1-03`). File name = `<ID>_<kebab-slug>.<ext>`. Never rename after logging; the report cites IDs.
2. **Log every capture right away** with
   `scripts/log_capture.py media/X1_xpbd/X1-03_drop-wireframe.mp4 --caption "..." --cmd "..." --use report,video`.
   The script appends a row to `media/INDEX.md` with: ID, file, caption, date, command or scene, and intended use (`report` / `slides` / `video` / `final`). Write the caption as if it were the figure caption. That makes the report quick to write.
3. **Reproducibility:** if a capture comes from a headless run, copy that run's `run.json` next to it as `<ID>_run.json`.
4. **What goes in git:** PNG stills, plots (PNG plus PDF for LaTeX), short MP4s (H.264, CRF 23, under about 20 MB). Track `*.mp4` with Git LFS if the repo grows. Raw frame sequences and `.dae` frames stay in `out/`.
5. **Fixed framing:** add 2–3 named camera presets to the GUI (key `1`/`2`/`3`: front, 3/4 view, top) so before/after captures across stages line up.

### Tooling (built in Stage 0)

- **GUI screenshot:** key `S` saves `out/capture/shot_<timestamp>.png`. Use `glReadPixels` at **framebuffer** size (Retina is 2× window size; use `glfwGetFramebufferSize`), flip vertically, and save with CGL's `lodepng`.
- **GUI recording:** key `V` toggles recording. While on, it saves one PNG per drawn frame to `out/capture/rec_<timestamp>/%05d.png`. While recording, force `frames_per_sec = 30` so each saved frame is exactly 1/30 s of sim time. The 30 fps video then plays in real time no matter how slowly the sim runs.
- **`scripts/make_video.sh <png_dir> <out.mp4> [fps]`**: `ffmpeg -framerate 30 -i %05d.png -c:v libx264 -pix_fmt yuv420p -crf 23`. Add a `--hstack a.mp4 b.mp4` mode for side-by-side solver comparisons, using `drawtext` labels such as "Mass-spring" and "XPBD".
- **`scripts/plot_metrics.py run1/metrics.csv run2/metrics.csv ... -o media/Q1_metrics/Q1-02`** writes `.png` and `.pdf`.

---

## 3. Stages

Each stage lists: **Build** (what to write), **Verify** (how you know it's right), **Capture** (what to save), and **Done when**. Stage IDs are grouped by track:
**P** = pipeline/meshing, **M** = mass-spring, **X** = XPBD, **C** = contact, **Q** = metrics, **R** = rendering, **I** = integration.

### Stage S0: Baseline, build, capture tooling (half a day, 1 person, do first)

- **Build:** build both binaries (`cmake .. && make -j`). Add `S`/`V` capture keys, `make_video.sh`, `log_capture.py`, `media/INDEX.md`, and the `media/` folders. Add `out/` to `.gitignore` (it already is).
- **Verify:** `pathtracer` renders `CBspheres_lambertian.dae`. `clothsim` runs `pinned2.json`. Pressing `V` for 3 s produces about 90 frames and a video that plays in real time.
- **Capture:**
  - `S0-01` CBspheres_lambertian at 480×360, 256 spp (baseline renderer).
  - `S0-02` `CBspheres.dae` before R1, showing the broken (black) glass and mirror spheres. This is the "before" for R1.
  - `S0-03` 5 s recording of HW4 cloth on a sphere (shows the starting point).
- **Done when:** a teammate can clone, build, and make a logged capture in under 10 minutes.

### Stage P1: Tet mesh and boundary surface (Day 1)

- **Build:** `TetMesh::buildGrid`.
  - Vertices on an (nx+1)(ny+1)(nz+1) lattice.
  - **5-tet split** per cell, mirrored by parity `(i+j+k) % 2` so face diagonals match between neighbors. 8×8×8 cells gives 2,560 tets (the proposal's number) with face diagonals and no body diagonals.
  - Orient each tet to positive signed volume `V = (1/6) (x1−x0)·((x2−x0)×(x3−x0))` by swapping two vertices if needed.
  - Unique edges come from sorted vertex pairs over all tets.
  - **Boundary faces** are tet faces that appear exactly once (hash the sorted triple). Wind each one so its normal points away from the tet's fourth vertex.
- **Verify:** add a `jellosim --selftest` that checks and prints:
  - total volume = Lx·Ly·Lz (to 1e-12);
  - all tet volumes > 0;
  - surface triangle count = 4(nx·ny + ny·nz + nz·nx) (= 768 for 8³);
  - every surface edge is shared by exactly 2 triangles;
  - Euler characteristic V − E + F = 2;
  - the divergence-theorem volume of the surface equals the sum of tet volumes;
  - all surface normals point outward.
- **Capture:**
  - `P1-01` selftest output (text) for grids from 1³ to 20³ (16³ = 20,480 tets is the mesh for the later performance study).
  - `P1-02` decomposition figure from `scripts/plot_tetmesh.py`: exploded even and odd cells, the boundary surface, and a cutaway.
- **Done when:** the selftest passes for several resolutions, including non-cubic ones such as 4×8×6.

### Stage P2: Viewer shows the soft body (Day 1–2)

- **Build:** add a jello mode to `ClothSimulator`. Scenes with a top-level `"jello"` object (`scene/jello/*.json`, parsed by `JelloScene`) load a `SoftBody`; other scenes still load the HW4 cloth, so the S0 captures stay reproducible.
  - Wireframe draws tet edges.
  - Normal and Phong shaders draw `surfaceTris` with per-vertex normals computed each frame (area-weighted, same as the path tracer).
  - Keys: camera presets `1`/`2`/`3` (front, three-quarter, top), `C` cutaway, `E` edge overlay, `R` reset.
  - Flags for scripted captures: `-S <shader>`, `-C <preset>`, `-K` (cutaway), `-E` (edges), `-P <file.png>` (screenshot, then exit).
  - A placeholder `static` solver holds the rest pose until M1/X1. Solver parameter controls (ks, kd, kv / compliances, volume toggle, substeps, plate) arrive with the solvers that use them.
  - The ground is drawn by the viewer, outside `jello_core`; the plate follows in C2.
- **Verify:** the static block looks right in all three shaders. Toggling between wireframe and surface works. Cloth scenes still run.
- **Capture:**
  - `P2-01` tet wireframe of a 4³ block (sparse enough to read).
  - `P2-02` 8³ surface with the Normal shader (shows outward normals).
  - `P2-03` cutaway: hide the tets in the octant facing the camera, draw the rest flat-shaded with edges, to show the interior tets.
  - `P2-04`/`P2-05` 8³ Phong with smooth normals, without and with edges (the rounded look the path tracer will also show).
- **Done when:** both solvers (once they exist) plug in without viewer changes.

### Stage M1: Mass-spring with gravity and ground (Day 2)

- **Build:** `MassSpringBody`.
  - Lumped mass: each tet gives ρV/4 to each of its 4 vertices.
  - One spring per tet edge: `f = ks (|d| − L0) d̂` plus damping along the spring, `kd ((v_b − v_a)·d̂) d̂`.
  - Semi-implicit Euler per substep: `v += dt·f/m`, then `x += dt·v`.
  - Ground contact through the `Collider` interface (`query(p, t)` → signed distance, normal, collider velocity): project out, remove the approaching normal velocity. Friction comes in C1.
  - Stability guard (`checkStability`: NaN, inverted tet, bbox > 2× rest): the viewer pauses and prints the reason; `-U` keeps going so a blow-up can be recorded.
  - Solvers list their tunable parameters (`SoftBody::params()`); the viewer builds its controls from that list.
  - `jellosim --run SCENE --set key=value` (and `clothsim -O key=value`) for headless runs and parameter overrides; `scripts/stability_sweep.py` bisects the substep count to find the largest stable dt.
- **Defaults (scene/jello/drop_ms.json):** ks = 300 N/m, kd = 0.02 N s/m, 100 substeps per 1/30 s frame. The first guesses were wrong in instructive ways:
  - kd = 0.5 is unstable at dt = 0.33 ms. Explicit damping needs dt < 2m/c, and the block-corner vertices weigh only 1/17 of the mean vertex (one small tet each), so the limit is ~0.1 ms there.
  - ks ≤ 200 inverts tets on landing from a 10 cm drop, whatever the time step. A spring lattice has no inversion resistance; M2's volume term is the fix.
  - ks = 500 gives a ~15 Hz bounce and 0.5% sag; ks = 300 wobbles visibly. Calibration against real jello is Week 3.
- **Verify:** the block falls, bounces and settles. Stable dt follows dt ∝ ks^(−1/2) (measured slope −0.48).
- **Capture:**
  - `M1-01` drop, Phong, 3/4 view, 4 s; `M1-02` same drop in wireframe.
  - `M1-03` dt too large (30 substeps): corner vertices diverge first, then the mesh explodes (slow motion).
  - `M1-04` ks too soft (150): stable integration, but the bottom layer inverts on landing.
  - `M1-05` largest stable dt vs ks (log-log, with fit).
  - `M1-06`/`M1-07` filmstrips of the drop and of the two failure modes, for the report.
- **Done when:** an 8³ block drops and settles stably at 30 fps with a fixed substep count.

### Stage X1: XPBD with distance constraints (Day 2)

- **Build:** `XPBDBody`, small-steps form: per frame, `n` substeps with `h = frameDt / n`:
  1. `v += h·g`; `p = x`; `x += h·v`.
  2. One Gauss-Seidel pass over the edges, alternating direction every substep (symmetric, removes ordering bias): `C = |x_a − x_b| − L0`, `Δλ = (−C − γ ∇C·(x − p)) / ((1 + γ)(w_a + w_b) + α/h²)`, with constraint damping `γ = αβ/h` (Macklin et al. 2016) instead of global velocity damping, which would also slow the free fall.
  3. Collision projection (ground) in the same substep.
  4. `v = (x − p)/h`.
- **Matched material:** compliance α = 1/ks and damping β = kd describe the same material as the mass-spring solver, so differences come from the solver: `scene/jello/drop_xpbd.json` uses α = 1/300, β = 0.02.
- **Findings:**
  - With 400 substeps XPBD reproduces the mass-spring bounces; with fewer it dissipates energy numerically (bounces shrink, settles ~0.9 s vs ~1.5 s).
  - It never produces NaN, but distance constraints alone need ≥ 40 substeps for the drop (about the mass-spring limit of 38). Below that, the bottom crushes and the block folds into a wad: folding keeps edge lengths, so nothing preserves volume. Default: 100 substeps.
  - The stable step vs stiffness is not a power law (best near k = 1000 N/m). XPBD wins on substeps at high stiffness, but costs ~5× more per substep (sequential Gauss-Seidel), so it is still slower per frame. X2's volume constraints should change this picture; re-run the sweep then.
- **Capture:**
  - `X1-01` XPBD drop (same scene, camera and duration as `M1-01`).
  - `X1-02` too few substeps (10): collapse and folding, the opposite failure to `M1-03`.
  - `X1-03` side by side, mass-spring vs XPBD.
  - `X1-04` center-of-mass and height traces: XPBD at 400/100/40 substeps vs mass-spring.
  - `X1-05` largest stable time step vs stiffness, XPBD vs mass-spring.
- **Done when:** the drop runs without NaN for every compliance tried, and its behavior vs substeps is understood.

### Stage M2 / X2: Volume terms (Day 3)

Both use the same geometric quantities (`tetVolume`, `tetVolumeGradients` in `tetMesh`):
`V = (1/6)(x1−x0)·((x2−x0)×(x3−x0))`, with gradients
`∇₁V = (1/6)(x2−x0)×(x3−x0)`, `∇₂V = (1/6)(x3−x0)×(x1−x0)`, `∇₃V = (1/6)(x1−x0)×(x2−x0)`, `∇₀V = −(∇₁V + ∇₂V + ∇₃V)`.
`jellosim --selftest` checks them against finite differences on 1,000 random tets (max relative error 1.4e-10).

- **M2 build:** per-tet penalty energy `E = ½ kv (V − V0)² / V0`, so `f_i = −kv ((V − V0)/V0) ∇ᵢV`; kv in Pa (bulk-modulus-like). `"volume": true/false` for the ablation.
- **X2 build:** per-tet constraint `C = V − V0`, `Δλ = −C / (Σ wᵢ|∇ᵢV|² + α_t/h²)`, `xᵢ += wᵢ Δλ ∇ᵢV`, with `α_t = V0 · volume_compliance`. `volume_compliance = 1/kv` is the same energy as the penalty, so the solvers can be matched. Solved right after the edge pass, same sweep direction.
- **Defaults are off** (`"volume": false`) so the M1/X1 scenes keep their behavior; `drop_ms_volume.json` (kv = 1e5 Pa, 150 substeps) and `drop_xpbd_volume.json` (compliance 1e-6, 50 substeps) turn them on. A `"gravity"` scene key was added for the squash test.
- **Findings:**
  - Worst volume error in the drop: mass-spring 7.2% → 0.84%; XPBD 7.7% → 1.18% at matched stiffness, 0.90% at compliance 1e-6 (meets the 1% target).
  - The penalty fixes the soft-spring inversion (M1-04): ks = 150 survives the drop.
  - Mass-spring pays in time step: min substeps 38 → 60 (kv 1e5) → 159 (kv 1e6). XPBD gains: 39 → 24 (1e-5) → 22 (1e-6). At equal accuracy (~0.3%) the cost per frame is similar (MS 400 substeps 22 ms, XPBD 100 substeps 20 ms).
  - Stable dt vs stiffness (X2-03, ks/k from 250 to 16,000 N/m): mass-spring + penalty keeps the k^-1/2 slope (58 → 261 substeps), lifted only at soft springs, where the penalty is the stiffest force. XPBD + volume (1e-5) needs 23–33 substeps at every k (dt ≈ 1.0–1.45 ms, no trend): volume constraints remove the tet inversions that limit edges-only XPBD.
  - Hard volume constraints (compliance 0) invert tets with one Gauss-Seidel pass per substep: they fight the edge and contact constraints.
  - Squash test (4 g, ks = 150): springs only lose 5.9% volume; with the penalty 0.7%, and the sides bulge past the rest outline.
  - Known XPBD artifact: below ~50 substeps the impact injects spurious spin (up to ~25 deg/s), kept by the frictionless ground. Mass-spring on the same mesh shows none. Friction (C1) will mask it; a graph-colored (Jacobi-per-color) ordering is the real fix (Week 3).
- **Capture:**
  - `M2-01` soft springs (ks = 150), volume off vs on, side by side.
  - `M2-02` mass-spring volume error vs time, off vs on.
  - `M2-03` squash test stills: rest / springs only / with penalty, with the rest outline marked.
  - `M2-04` largest stable dt vs ks with the penalty.
  - `X2-01` XPBD at 10 substeps, volume off (folds into a wad) vs on (survives, tumbles).
  - `X2-02` volume error vs time for all five configurations.
  - `X2-03` largest stable dt vs stiffness: both solvers, with and without volume terms.
- **Done when:** XPBD with volume constraints stays within 1% of rest volume during the drop (done: 0.90%). Mass-spring result recorded (0.84% at kv = 1e5).

### Stage Q1: Metrics logger and plots (Day 3, in parallel)

- **Build:** `Metrics::record(frame)` writes one CSV row per output frame:
  `frame, t, volume, vol_err_pct, com_x, com_y, com_z, top_y, min_tet_vol_ratio, n_inverted, bbox_ratio, has_nan, step_ms`.
  - `volume` = sum of signed tet volumes.
  - `top_y` = mean y of the vertices that start on the top face (track them by rest index).
  - `step_ms` = wall-clock time to simulate that frame (exclude export).
  - The stability rule matches the proposal: any inverted tet, any NaN, or `bbox_ratio > 2` within 5 s means unstable.
  - Write the CSV from both `jellosim` and the GUI (a GUI toggle).
  - `plot_metrics.py` makes: volume error vs t; COM y vs t; top y vs t; step_ms (bar or box plot). Label each run from its `run.json`.
- **Verify:** at rest (gravity off), volume error is 0 and COM is constant.
- **Capture:**
  - `Q1-01` the first combined plot: 4 runs (MS ± volume, XPBD ± volume), drop scenario, volume error vs t.
  - `Q1-02` COM y / top y vs t for both solvers (an early look at the oscillation comparison).
  - `Q1-03` ms per frame for both solvers at 8³.
- **Done when:** one command turns a set of runs into report-ready figures.

### Stage C1: Ground friction and a shared contact model (Day 3)

- **Build:** the `Collider` returns `phi` (signed distance), `n`, and the collider velocity `u`.
  - **Mass-spring** (after the position update): if `phi < 0`, project `x -= phi·n`. Then, with relative velocity `v_r = v − u`: `vn = v_r·n`, `vt = v_r − vn·n`. If `vn < 0`, set `vn = 0` and reduce `vt` by `μ|Δvn|` (clamped at zero, Coulomb). Then `v = u + vn·n + vt`.
  - **XPBD** (inside each substep): project, then apply position-level friction. Let `Δt_disp` = tangential displacement of `(x − p)` relative to the collider's motion `u·h`. If `|Δt_disp| < μ_s·|phi|`, cancel it (static). Otherwise scale it down by `μ_k·|phi| / |Δt_disp|`.
- **Verify:** a block dropped with sideways initial velocity slides and stops sooner as μ increases. μ = 0 slides forever.
- **Capture:** `C1-01`: three clips with μ = 0, 0.3, 0.8, combined with `--hstack`.
- **Done when:** both solvers use the same `Collider` API, which lets C2 plug in directly.

### Stage C2: Kinematic shaking plate (Day 4)

- **Build:** `KinematicPlate`: a box with center `c(t) = c0 + A sin(2πf(t − t0)) â` for `t > t0`, and velocity `u(t) = 2πfA cos(…) â`.
  - `query()` returns the signed distance to the box (top-face normal when above the footprint) and `u(t)`. The contact code already handles moving colliders through `u`.
  - The ground stays as a backup collider.
  - The viewer draws the plate at `c(t)`; the exporter writes the plate's node matrix.
- **Verify:** the jello rides the plate at low amplitude (friction holds) and wobbles, slips, or walks at high amplitude. There is no penetration at the plate edges.
- **Capture:**
  - `C2-01` GUI clip of the shake scenario, mass-spring.
  - `C2-02` same for XPBD.
  - `C2-03` the two clips side by side with `--hstack`. **This is the centerpiece for the milestone video.**
- **Done when:** both scenarios run 5 s stably with both solvers.

### Stage P3: Per-frame `.dae` export and template scene (Day 2–3)

- **Template:** copy `dae/sky/CBempty.dae` to `dae/jello/template.dae`.
  - Add a `jello-material` effect (the `<jello>` CGL profile, see R2) and a plate mesh with a diffuse material.
  - Add a `jello` node and a `plate` node. Each node's `<matrix>` maps sim (Y-up, meters) into the box's Z-up frame and scale. Keep that matrix in the template only, so the sim never knows about scene units.
  - Placeholders: `{{JELLO_POS_COUNT}}`, `{{JELLO_POSITIONS}}`, `{{JELLO_TRI_COUNT}}`, `{{JELLO_VCOUNT}}`, `{{JELLO_INDICES}}`, `{{PLATE_MATRIX}}`.
  - Change the camera's `aspect_ratio` to 16:9 for 960×540.
- **Exporter:** write `<polylist>` with a `VERTEX` input only (normals are recomputed by the loader anyway). Write positions of surface vertices only, re-indexed compactly. Use `%.6f` and write one frame per file: `out/sim/<run>/frame_0000.dae`.
- **Verify:** load frame 0 in the `pathtracer` GUI. The undeformed block should sit exactly on the plate, right side up and centered. Use the diffuse fallback material until R2 lands. Check that the loader prints no halfedge or non-manifold warnings.
- **Capture:** `P3-01` frame 0 with a diffuse jello, low spp (proves the pipeline end to end before any fancy shading).
- **Done when:** `jellosim ... -o out/sim/drop_xpbd` produces 120 loadable `.dae` frames.

### Stage R1: Dielectric BSDF and integrator fixes (Day 1–2)

- **Build** (in [advanced_bsdf.cpp](../src/render/pathtracer/advanced_bsdf.cpp)):
  - `reflect`: `wi = (−wo.x, −wo.y, wo.z)`.
  - `refract`: η = 1/ior when entering (`wo.z > 0`) or ior when exiting. Return false on total internal reflection.
  - `GlassBSDF::sample_f`: exact dielectric Fresnel (or Schlick, decide once and note it). Reflect with probability F (pdf = F, return `F·reflectance/|cos|`). Otherwise refract (pdf = 1−F, return `(1−F)·transmittance/|cos|/η²`).
  - Also fill `MirrorBSDF` (trivial, useful for debugging).
- **Integrator fixes** in [pathtracer.cpp](../src/render/pathtracer/pathtracer.cpp):
  1. Use `fabs(w_in.z)` in the bounce term.
  2. Skip `one_bounce_radiance` when `isect.bsdf->is_delta()` (it is always 0).
  3. After a delta bounce, **add `zero_bounce_radiance(bounce_isect)`**, so lights seen through or reflected in the jello appear (these are the bright refracted highlights).
  4. Don't apply Russian roulette on delta bounces (paths through the jello need 2–6 specular bounces).
- **Verify:** `CBspheres.dae` matches the expected HW3-2 look: a clear glass sphere with a caustic-free shadow and a visible light reflection. Render at `-m 1, 2, 3, 4, 5, 100` (enter, exit, internal bounce). There should be no NaNs or fireflies from division by small cosines.
- **Capture:**
  - `R1-01` CBspheres after the fix (compare with `S0-02`).
  - `R1-02` a max-ray-depth sequence (a classic report figure).
  - `R1-03` ior sweep 1.0 / 1.33 / 1.5 on the glass sphere.
- **Done when:** the glass sphere is correct at 256 spp.

### Stage R2: Jello material on static meshes (Day 2–3)

- **Build:** a `JelloBSDF` (or `GlassBSDF` plus fields) with `ior` and `sigma_a` (RGB, **per scene unit**, since the node matrix scales sim meters). Parse it in [collada.cpp](../src/render/scene/collada/collada.cpp) next to the `"glass"` branch, as a `<jello>` element with `<ior>`, `<sigma_a>`, `<reflectance>`, and `<transmittance>`.
  - Add `Vector3D ng` (geometric normal from triangle winding) to `Intersection` and set it in `Triangle::intersect` (precompute it in the constructor). Spheres set `ng = n`.
  - **Smooth-normal guard:** if `sign(dot(−r.d, n)) != sign(dot(−r.d, ng))`, use `ng` as the shading normal for that hit. Otherwise refraction can pick the wrong η near block edges.
- **Verify:** an undeformed block (P3 frame 0) renders as clear glass. The smooth normals visibly round the edges and corners. That is expected and looks jello-like, but compare with a faceted version (use separate per-face vertices in a test export) to see it.
- **Capture:** `R2-01` static block, clear dielectric (σ_a = 0), 960×540, 256 spp. Optionally `R2-02`, smooth vs faceted normals side by side (good for explaining the design choice).

### Stage R3: Beer–Lambert absorbing medium (Day 3–4)

- **Build:** inside/outside is decided per segment, without any extra state. In `at_least_one_bounce_radiance`, after finding `bounce_isect`: if `bounce_isect.bsdf` is a medium and `dot(bounce_ray.d, bounce_isect.ng) > 0`, the segment ran **inside** the jello, so multiply `L_in` by `exp(−σ_a · t · |d|)`. Apply the same check to camera rays in `est_radiance_global_illumination`. This relies on the surface being watertight and outward-wound, which P1's selftest guarantees.
- **Verify:**
  - A thicker block (or a longer path) looks darker and more saturated.
  - σ_a = 0 reproduces R2 exactly.
  - Analytic check: render a slab of known thickness against a white emitter and compare the pixel ratio with `exp(−σ_a d)` (times the Fresnel factor).
- **Capture:**
  - `R3-01` absorption sweep: σ_a scaled ×0.25, ×1, ×4 (red jello, for example σ_a ≈ (0.5, 6, 8) per scene unit, tuned by eye).
  - `R3-02` the hero still: a deformed mid-impact frame from the drop, 960×540, 256 spp. **Report teaser figure.**
  - `R3-03` render time per frame vs spp (a first data point for the rendering measurements).
- **Note:** shadows under the jello will be dark because pure path tracing finds few caustic paths. That is expected. Mention it in the report as a known limitation. A transmissive-shadow hack can be a later toggle.

### Stage I1: End-to-end sequences (Day 4, renders overnight)

- **Build:** `scripts/render_sequence.py out/sim/<run> out/render/<run> --start 0 --end 120 --stride 1 -- -t 8 -s 64 -m 10 -l 4 -r 480 270`. It skips frames already rendered (so it can resume) and supports `--stride`/`--offset` so 4 laptops can split frames (`--offset k --stride 4`).
- **Runs:** {drop, shake} × {mass-spring, XPBD}, with volume on. That is 4 × 120 frames. Time one frame first. Budget: at about 20 s per frame, roughly 2.7 h on one machine, or about 40 min split four ways.
- **Capture:**
  - `I1-01`…`I1-04` the four sequence MP4s.
  - `I1-05` drop, MS | XPBD side by side.
  - `I1-06` shake, MS | XPBD side by side.
  - `I1-07` a filmstrip still (6 frames across the impact) for the report.
- **Done when:** both scenarios render end to end for both solvers. **This is the milestone bar.**

### Stage I2: Milestone deliverables (Day 5)

- **Report** (SIGGRAPH template, about 1–2 pages): pipeline diagram; P1-02 (tet decomposition); M1-01/X1-01 stills; Q1-01 (volume error ± volume term); R1-02 (depth sweep); R3-01 (absorption sweep); R3-02 (hero); a short table of the volume errors and ms per frame we have so far; progress against the plan; and the plan for Weeks 3–4 (calibration, sweeps, appearance tuning).
- **1-minute video:** 0–8 s title and tet mesh (P1-02, P2-01, P2-03) → 8–25 s GUI side-by-side drop and shake (C2-03, plus M2-01/X2-01 volume on/off) → 25–50 s rendered sequences (I1-05, I1-06) → 50–60 s plots (Q1-01) and next steps. Build it with ffmpeg concat plus drawtext captions, or any editor. Log it as `I2-01`.
- **Slides (2–3):** (1) what we built, with the pipeline diagram and hero still; (2) results so far (volume plot and side-by-side GIF); (3) what's next. Log the exported PDF as `I2-02`.

---

## 4. Dependency graph

```
S0 ──┬─► P1 ─► P2 ─┬─► M1 ─► M2 ─┐
     │             ├─► X1 ─► X2 ─┼─► C1 ─► C2 ─┐
     │             └─► Q1 ───────┘             ├─► I1 ─► I2
     │      P1 ─► P3 ────────────────────────► │
     └─► R1 ─► R2 ─► R3 ──────────────────────►┘
```

R1 depends on nothing in the sim, so it can start on Day 1. M1 and X1 can start on Day 1 against the `TetMesh`/`SoftBody` headers (agree on them in the first hour) and run headless before P2 lands.

## 5. Day-by-day schedule (4 people)

| Day | Track A: Pipeline / meshing / metrics | Track B: Mass-spring + contact | Track C: XPBD | Track D: Rendering |
|---|---|---|---|---|
| **Wed 11/18** | S0, agree on headers, P1 + selftest | M1 (headless, using a stub mesh until P1) | X1 (headless) | R1 |
| **Thu 11/19** | P2 viewer, P3 exporter + template | M1 in GUI, Collider API (C1) | X1 in GUI, X1 captures | R1 captures, R2 |
| **Fri 11/20** | Q1 logger + plots, P3 captures | M2, C1 friction | X2, XPBD friction | R3 + analytic check |
| **Sat 11/21** | I1 render script; start overnight renders | C2 plate (shared by both solvers) | Stability guard, run all 4 headless sims | R3 hero still, tune σ_a and ior, I1 renders |
| **Sun 11/22** | I2 plots and report figures | I2 video | I2 slides | I2 report rendering section |

Rule for every day: **no stage is done until its captures are logged in `media/INDEX.md`.** Do a 10-minute check-in each evening: what's captured, what's blocked, and whether we need to cut scope (§6).

## 6. What to cut if we fall behind (in this order)

1. Use 32 spp at 480×270 instead of 64 spp. Keep 960×540 / 256 spp for 1–2 hero stills only.
2. Render the shake scenario for one solver only. Show the other in GUI side-by-side clips.
3. Use simpler XPBD friction: a static-only cancel with a single μ.
4. Skip the analytic Beer–Lambert check (do it in Week 3).
5. **Never cut:** both solvers with volume terms, the drop scenario rendered end to end, and the volume-error plot. These are the milestone's core claims.

## 7. Known pitfalls

- **Watertightness matters for rendering.** Any non-manifold or flipped surface triangle breaks the inside/outside test (R3). Keep the P1 selftest in CI-like use: run it in `jellosim` on every export (it is cheap).
- **Plate/jello interpenetration** puts a surface inside the medium, which shows up as dark artifacts. Keep a small contact offset (the cloth code uses `SURFACE_OFFSET = 1e-4`).
- **`sigma_a` units are scene units**, not meters. Note the node-matrix scale in `run.json`.
- **Retina screenshots** must use the framebuffer size, or captures come out cropped or blurry.
- **Explicit mass-spring with a large kv** needs many substeps. Log the substep count in `run.json` so ms-per-frame comparisons are fair.
- **Row-major `<matrix>`** in COLLADA: verify the plate's motion direction in P3 before rendering 120 frames.
- **Thread count:** keep `-t` the same across machines when timing renders. Otherwise only use rendered frames, not their timings.
