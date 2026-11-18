# Media index

Every capture used in the milestone report, slides, video, or final paper is logged here
by `scripts/log_capture.py`. Cite captures by ID. Never rename a logged file.
Naming and folder rules: [docs/milestone-plan.md](../docs/milestone-plan.md), section 2.

| ID | File | Caption | Date | Command / scene | Use |
|----|------|---------|------|-----------------|-----|
| S0-01 | [S0-01_cbspheres-lambertian.png](S0_baseline/S0-01_cbspheres-lambertian.png) | Baseline: inherited HW3 path tracer (BVH, importance-sampled direct + global illumination) on the diffuse Cornell box spheres. 480x360, 256 spp, 4 light samples, max depth 5; 32.6 s on 10 threads. | 2026-11-18 | `./build/pathtracer -t 10 -s 256 -l 4 -m 5 -r 480 360 -f out.png dae/sky/CBspheres_lambertian.dae` | report, debug |
| S0-02 | [S0-02_cbspheres-glass-mirror-broken.png](S0_baseline/S0-02_cbspheres-glass-mirror-broken.png) | Before Stage R1: the mirror (left) and glass (right) spheres render black because MirrorBSDF/GlassBSDF are unimplemented stubs that return zero. The 'before' image for the dielectric BSDF. Same settings as S0-01; 45.7 s. | 2026-11-18 | `./build/pathtracer -t 10 -s 256 -l 4 -m 5 -r 480 360 -f out.png dae/sky/CBspheres.dae` | report, debug |
| S0-03 | [S0-03_cloth-on-sphere.mp4](S0_baseline/S0-03_cloth-on-sphere.mp4) | Starting point for the simulator: inherited HW4 mass-spring cloth (50x50, Verlet) draping over a sphere, wireframe shader. 5 s of simulated time recorded at 30 fps of sim time (151 frames), so playback is real time even though the run took 18 s wall-clock. | 2026-11-18 | `./build/clothsim -f scene/sphere.json -R out/capture/S0-03_cloth-sphere -T 5 && scripts/make_video.sh out/capture/S0-03_cloth-sphere <out.mp4>` | video, debug |
