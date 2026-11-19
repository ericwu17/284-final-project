#!/usr/bin/env python3
"""Analytic check of the path tracer's jello absorption (Stage R3).

Builds a scene with nothing in it but a uniform emitter (radiance 0.5) and,
in front of it, a jello slab of thickness d facing the camera. The center of
the image looks straight through the slab (normal incidence), so the
radiance there, divided by the radiance with no slab, should be

    T^2 exp(-sigma d) / (1 - R^2 exp(-2 sigma d)),   R = ((n-1)/(n+1))^2, T = 1 - R

(Beer-Lambert inside, Fresnel at both faces, plus light bouncing back and
forth between the faces any number of times). Renders one image per sigma,
inverts the PNG encoding (8-bit of (sqrt(2) L)^(1/2.2), see
HDRImageBuffer::toColor), averages a patch at the center, and compares.

Usage:
  scripts/beer_lambert_check.py OUT_PREFIX [--sigmas 0,0.5,1,2,4] [--d 0.5] [--ior 1.34]
Writes OUT_PREFIX.png/.pdf (measured vs predicted) and OUT_PREFIX.csv.
"""
import argparse
import csv
import math
import os
import subprocess
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.image as mpimg
import matplotlib.pyplot as plt
import numpy as np

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
PATHTRACER = os.path.join(ROOT, "build", "pathtracer")
EMIT = 0.5  # emitter radiance: sqrt(2) * 0.5 < 1, so the PNG never clips


def box(x0, x1, y0, y1, z0, z1):
    """Closed box, outward counter-clockwise triangles (Y-up coordinates)."""
    p = [(x, y, z) for z in (z0, z1) for y in (y0, y1) for x in (x0, x1)]  # index = x + 2y + 4z
    tris = [(0, 2, 3), (0, 3, 1), (4, 5, 7), (4, 7, 6),   # z0 (-z), z1 (+z)
            (0, 1, 5), (0, 5, 4), (2, 6, 7), (2, 7, 3),   # y0 (-y), y1 (+y)
            (0, 4, 6), (0, 6, 2), (1, 3, 7), (1, 7, 5)]   # x0 (-x), x1 (+x)
    return p, tris


def geometry(gid, mat, pts, tris):
    pos = " ".join(f"{c:g}" for q in pts for c in q)
    return f'''    <geometry id="{gid}" name="{gid}">
      <mesh>
        <source id="{gid}-pos">
          <float_array id="{gid}-pos-array" count="{3 * len(pts)}">{pos}</float_array>
          <technique_common>
            <accessor source="#{gid}-pos-array" count="{len(pts)}" stride="3">
              <param name="X" type="float"/><param name="Y" type="float"/><param name="Z" type="float"/>
            </accessor>
          </technique_common>
        </source>
        <vertices id="{gid}-vtx"><input semantic="POSITION" source="#{gid}-pos"/></vertices>
        <polylist material="{mat}" count="{len(tris)}">
          <input semantic="VERTEX" source="#{gid}-vtx" offset="0"/>
          <vcount>{" ".join("3" for _ in tris)}</vcount>
          <p>{" ".join(str(i) for t in tris for i in t)}</p>
        </polylist>
      </mesh>
    </geometry>
'''


def node(nid, gid, mat):
    # The file is Z_UP like the template, so geometry written in Y-up goes
    # under the matrix G that cancels the loader's Z_UP correction.
    return f'''      <node id="{nid}" name="{nid}" type="NODE">
        <matrix sid="transform">-1 0 0 0 0 0 1 0 0 1 0 0 0 0 0 1</matrix>
        <instance_geometry url="#{gid}">
          <bind_material><technique_common>
            <instance_material symbol="{mat}" target="#{mat}"/>
          </technique_common></bind_material>
        </instance_geometry>
      </node>
'''


def scene(sigma, d, ior, with_slab):
    # Emitter: a large quad at z = -1 (a thin closed box, so the mesh is closed).
    ep, et = box(-3, 3, -3, 3, -1.02, -1.0)
    sp, st = box(-1.2, 1.2, -1.2, 1.2, -d / 2, d / 2)
    # Geometry ids must differ from node ids: the loader resolves urls in one table.
    geoms = geometry("emitter-mesh", "emit-mat", ep, et)
    nodes = node("emitter", "emitter-mesh", "emit-mat")
    if with_slab:
        geoms += geometry("slab-mesh", "jello-mat", sp, st)
        nodes += node("slab", "slab-mesh", "jello-mat")
    return f'''<?xml version="1.0" encoding="utf-8"?>
<COLLADA xmlns="http://www.collada.org/2005/11/COLLADASchema" version="1.4.1">
  <asset><unit name="meter" meter="1"/><up_axis>Z_UP</up_axis></asset>
  <library_cameras>
    <camera id="cam" name="cam"><optics><technique_common><perspective>
      <xfov sid="xfov">30</xfov><aspect_ratio>1.333333</aspect_ratio>
      <znear sid="znear">0.1</znear><zfar sid="zfar">100</zfar>
    </perspective></technique_common></optics></camera>
  </library_cameras>
  <library_effects>
    <effect id="emit-effect"><profile_COMMON><technique sid="common"><phong>
      <diffuse><color sid="diffuse">0 0 0 1</color></diffuse></phong></technique></profile_COMMON>
      <extra><technique profile="CGL"><emission><radiance>{EMIT} {EMIT} {EMIT}</radiance></emission></technique></extra>
    </effect>
    <effect id="jello-effect"><profile_COMMON><technique sid="common"><phong>
      <diffuse><color sid="diffuse">0.5 0.5 0.5 1</color></diffuse></phong></technique></profile_COMMON>
      <extra><technique profile="CGL"><jello>
        <ior>{ior}</ior><sigma_a>{sigma} {sigma} {sigma}</sigma_a>
      </jello></technique></extra>
    </effect>
  </library_effects>
  <library_materials>
    <material id="emit-mat" name="emit"><instance_effect url="#emit-effect"/></material>
    <material id="jello-mat" name="jello"><instance_effect url="#jello-effect"/></material>
  </library_materials>
  <library_geometries>
{geoms}  </library_geometries>
  <library_visual_scenes>
    <visual_scene id="Scene" name="Scene">
      <node id="Camera" name="Camera" type="NODE">
        <matrix sid="transform">-1 0 0 0 0 0 1 3 0 1 0 0 0 0 0 1</matrix>
        <instance_camera url="#cam"/>
      </node>
{nodes}    </visual_scene>
  </library_visual_scenes>
  <scene><instance_visual_scene url="#Scene"/></scene>
</COLLADA>
'''


def center_radiance(png, half=8):
    img = mpimg.imread(png)[..., :3].astype(np.float64)
    h, w = img.shape[:2]
    patch = img[h // 2 - half:h // 2 + half, w // 2 - half:w // 2 + half]
    # PNG stores (sqrt(2) L)^(1/2.2) in 8 bits; invert per pixel, then average.
    lin = np.power(patch, 2.2) / math.sqrt(2)
    return lin.mean(), lin.std() / math.sqrt(lin[..., 0].size)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("out_prefix")
    ap.add_argument("--sigmas", default="0,0.5,1,2,4")
    ap.add_argument("--d", type=float, default=0.5, help="slab thickness (scene units)")
    ap.add_argument("--ior", type=float, default=1.34)
    ap.add_argument("--spp", type=int, default=64)
    args = ap.parse_args()
    work = os.path.join(ROOT, "out", "r3_check")
    os.makedirs(work, exist_ok=True)

    def render(name, text):
        dae, png = os.path.join(work, name + ".dae"), os.path.join(work, name + ".png")
        with open(dae, "w") as f:
            f.write(text)
        cmd = [PATHTRACER, "-t", "10", "-s", str(args.spp), "-l", "1", "-m", "100",
               "-r", "160", "120", "-f", png, dae]
        proc = subprocess.run(cmd, capture_output=True, text=True)
        if proc.returncode != 0:
            sys.exit(f"render failed: {' '.join(cmd)}\n{proc.stdout[-2000:]}{proc.stderr[-2000:]}")
        return center_radiance(png)

    ref, _ = render("no_slab", scene(0, args.d, args.ior, False))
    print(f"no slab: center radiance {ref:.4f} (emitter {EMIT})")
    R = ((args.ior - 1) / (args.ior + 1)) ** 2
    T = 1 - R
    rows = []
    for sigma in [float(s) for s in args.sigmas.split(",")]:
        L, err = render(f"slab_sigma{sigma:g}", scene(sigma, args.d, args.ior, True))
        a = math.exp(-sigma * args.d)
        predicted = T * T * a / (1 - R * R * a * a)
        measured = L / ref
        # Also against the exact emitter radiance: the no-slab reference itself
        # carries the 8-bit rounding of the PNG (it reads ~0.4958, not 0.5).
        vs_exact = L / EMIT
        rows.append({"sigma": sigma, "sigma_d": sigma * args.d, "predicted": predicted,
                     "measured": measured, "measured_err": err / ref,
                     "rel_diff_pct": 100 * (measured - predicted) / predicted,
                     "measured_vs_exact_emitter": vs_exact,
                     "rel_diff_vs_exact_pct": 100 * (vs_exact - predicted) / predicted})
        print(f"sigma {sigma:5g}  sigma*d {sigma * args.d:5.2f}  predicted {predicted:.4f}  "
              f"measured {measured:.4f} +- {err / ref:.4f} ({rows[-1]['rel_diff_pct']:+.2f}%)  "
              f"vs exact emitter {vs_exact:.4f} ({rows[-1]['rel_diff_vs_exact_pct']:+.2f}%)")

    fig, ax = plt.subplots(figsize=(5, 3.6))
    sd = np.linspace(0, max(r["sigma_d"] for r in rows) * 1.05, 200)
    a = np.exp(-sd)
    ax.semilogy(sd, T * T * a / (1 - R * R * a * a), color="gray", lw=1.2,
                label=f"predicted, ior {args.ior:g} (Fresnel T² = {T * T:.3f})")
    ax.semilogy(sd, np.exp(-sd), color="gray", lw=1, ls=":", label="exp(-σd) alone")
    ax.errorbar([r["sigma_d"] for r in rows], [r["measured"] for r in rows],
                yerr=[2 * r["measured_err"] for r in rows], fmt="o", color="#E45756", ms=5,
                label="path tracer (center patch)")
    ax.set_xlabel("optical depth σ_a d")
    ax.set_ylabel("radiance through slab / without")
    ax.grid(True, which="both", alpha=0.25)
    ax.legend(fontsize=7.5)
    fig.tight_layout()
    os.makedirs(os.path.dirname(os.path.abspath(args.out_prefix)), exist_ok=True)
    for ext in ("png", "pdf"):
        fig.savefig(f"{args.out_prefix}.{ext}", dpi=200)
    with open(args.out_prefix + ".csv", "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)
    print(f"wrote {args.out_prefix}.png/.pdf/.csv")


if __name__ == "__main__":
    main()
