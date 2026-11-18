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
scripts/log_capture.py media/S0_baseline/S0-03_cloth-on-sphere.mp4 \
    --caption "Figure caption..." --cmd "command that made it" --use report,video
```

`out/` is git-ignored scratch space (raw frames, sim output). Only curated captures go in `media/`.
