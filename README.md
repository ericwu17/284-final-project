# Simulating and Path-Tracing Translucent Jello

CS 184/284A final project: a volumetric soft-body jello simulation (mass-spring on a tetrahedral mesh and XPBD), rendered with physically based light transport. See the [project proposal](https://ericwu17.github.io/cs284-final-project-proposal/).

This is one CMake project that builds two executables:

| Executable   | Source         | Starting point                                                     |
| ------------ | -------------- | ------------------------------------------------------------------ |
| `clothsim`   | `src/sim/`     | HW4 ClothSim: mass-spring integration, collision objects, NanoGUI viewer |
| `pathtracer` | `src/render/`  | HW3 PathTracer: BVH, `.dae` scene loading, direct and global illumination |

The planned pipeline: the simulator writes each frame's boundary surface to a `.dae` scene, and the path tracer renders it.

## Layout

```
CGL/            Shared math/graphics library (vectors, matrices, viewer). From HW3; includes GLEW.
ext/nanogui/    GUI for the simulator. Also provides GLFW for both executables.
ext/imgui/      GUI for the path tracer.
src/sim/        Simulator (HW4).
src/render/     Path tracer (HW3).
scene/          Simulator scenes (.json).
shaders/        Simulator GLSL shaders.
textures/       Simulator textures.
dae/            Path tracer scenes (.dae): Cornell boxes in dae/sky, primitives in dae/simple.
scripts/        Helper scripts (render.py: run the path tracer with a progress bar).
```

## Build

Requires CMake, a C++17 compiler, OpenGL and FreeType (`brew install freetype` on macOS).

```sh
mkdir -p build && cd build
cmake ..
make -j
```

## Run

From the `build/` directory:

```sh
# Simulator
./clothsim -f ../scene/pinned2.json

# Path tracer, interactive viewer (press R to render)
./pathtracer -t 8 ../dae/sky/CBspheres_lambertian.dae

# Path tracer, render straight to a file
./pathtracer -t 8 -s 256 -l 4 -m 5 -r 480 360 -f out.png ../dae/sky/CBbunny.dae
```

Or from the repo root, with a progress bar:

```sh
scripts/render.py -t 8 -s 256 -l 4 -m 5 -r 480 360 -f out.png dae/sky/CBbunny.dae
```

Run `./pathtracer -h` to list all options. With no arguments it opens a launcher GUI where you can pick the settings.

### Rendering the jello

`jellosim` writes one `.dae` per frame from `dae/jello/template.dae`; render them with the path tracer at **960x540** (the template's framing assumes that size, because the path tracer scales its field of view with the output resolution):

```sh
./build/jellosim --run scene/jello/drop_xpbd_volume.json --seconds 4 --export out/sim/drop_xpbd/frames
./build/pathtracer -t 10 -s 64 -l 4 -m 8 -r 960 540 -f frame0.png out/sim/drop_xpbd/frames/frame_0000.dae
```

The jello's look is set in the template's `<jello>` element: `ior` (1.34) and `sigma_a`, the absorption per scene unit (6 units = 1 m; default strawberry red `0.5 6 8`, `0 0 0` for clear). Use `-m 8` or more so rays can bounce around inside the block. The path tracer also writes a `<name>_rate.png` sampling heat map next to each image (red = pixel used all its samples). To show the translucency, export with `--template dae/jello/template_backdrop.dae` (the same scene plus three spheres behind the jello; regenerate it with `scripts/make_backdrop_template.py` after editing `template.dae`).

## Captures (screenshots, recordings, figures)

Every result we may show in the milestone or final report goes in `media/` and is logged in [media/INDEX.md](media/INDEX.md). The rules are in [docs/milestone-plan.md](docs/milestone-plan.md), section 2. Requires `ffmpeg` (`brew install ffmpeg`).

In `clothsim`, press **S** for a screenshot and **V** to start/stop recording. Both save to `out/capture/` and leave out the GUI panel. Recordings save one frame per 1/30 s of *simulated* time, so the video plays in real time however slowly the sim runs. To record without pressing keys:

```sh
./build/clothsim -f scene/sphere.json -R out/capture/my_run -T 5    # 5 sim-seconds, then exit
```

Then turn the frames into a video and log it (run from the repo root):

```sh
scripts/make_video.sh out/capture/my_run media/S0_baseline/S0-03_cloth-on-sphere.mp4
scripts/make_video.sh --hstack media/X1_xpbd/X1-04_compare.mp4 a.mp4 "Mass-spring" b.mp4 "XPBD"
MAX_WIDTH=1920 scripts/make_video.sh --hstack out.mp4 a.mp4 "mu = 0" b.mp4 "mu = 0.3" c.mp4 "mu = 0.8"  # any number
scripts/log_capture.py media/S0_baseline/S0-03_cloth-on-sphere.mp4 \
    --caption "Figure caption..." --cmd "command that made it" --use report,video
```

### Metrics and plots

Every jello run can log one CSV row per simulated frame (volume error, center of mass, top-face height, smallest tet volume, ms per frame, ...):

```sh
./build/jellosim --run scene/jello/drop_xpbd_volume.json --seconds 3 --out out/sim/my_run --label "XPBD + volume"
# writes out/sim/my_run/metrics.csv and run.json (all settings, result, git commit)
```

In `clothsim`, press **M** (or tick "log metrics CSV") to log the same columns to `out/sim/gui_<time>/metrics.csv`, or pass `-M file.csv` to log from launch.

`scripts/plot_metrics.py` runs scenes and plots them in one command (it also accepts finished runs with `--dir`):

```sh
scripts/plot_metrics.py media/Q1_metrics/Q1-01_drop-volume-error-4-solvers --panels vol,mintet \
    --run "mass-spring|scene/jello/drop_ms.json" \
    --run "XPBD, 50 substeps|scene/jello/drop_xpbd.json|xpbd.substeps=50"
scripts/plot_metrics.py out/timing --timing --run ...   # ms per frame instead
```

`out/` is git-ignored scratch space (raw frames, sim output). Only curated captures go in `media/`.
