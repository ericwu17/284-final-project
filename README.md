# Simulating and Path-Tracing Translucent Jello

CS 184/284A final project: a volumetric soft-body jello simulation (mass-spring on a tetrahedral mesh and XPBD), rendered with physically based light transport. See the [project proposal](https://ericwu17.github.io/cs284-final-project-proposal/).

The simulator is built on the CS 184 HW4 ClothSim starter code (CGL, NanoGUI, mass-spring integration and collision objects).

## Build

Requires CMake, a C++17 compiler, OpenGL and FreeType.

```sh
mkdir build && cd build
cmake ..
make -j
```

## Run

From the `build/` directory:

```sh
./clothsim -f ../scene/pinned2.json
```
