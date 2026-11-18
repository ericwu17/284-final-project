#!/usr/bin/env python3
"""Figure of the grid tetrahedralization (Stage P1).

Usage: scripts/plot_tetmesh.py OUT_PREFIX     (writes OUT_PREFIX.png and OUT_PREFIX.pdf)

Panels:
  (a) one even cell split into 5 tets, pulled apart ("exploded")
  (b) the mirrored split used in odd cells
  (c) the boundary surface of a 4x4x4 block: face diagonals alternate cell to cell
  (d) the same block with one octant removed, showing the interior tets

Meshes come from build/jellosim --dump-mesh, so the figure shows exactly what
the C++ code builds.
"""
import json
import os
import subprocess
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
JELLOSIM = os.path.join(ROOT, "build", "jellosim")
TMP = os.path.join(ROOT, "out", "mesh")

CORNER_COLORS = ["#4C78A8", "#72B7B2", "#54A24B", "#B279A2"]
CENTER_COLOR = "#E45756"
SURFACE_COLOR = "#F2A65A"
LIGHT = np.array([0.4, -0.5, 0.75])  # in plot coordinates (z up)
LIGHT /= np.linalg.norm(LIGHT)
FACES = [(1, 2, 3, 0), (0, 2, 3, 1), (0, 1, 3, 2), (0, 1, 2, 3)]  # face + opposite vertex


def load(nx, ny, nz):
    os.makedirs(TMP, exist_ok=True)
    path = os.path.join(TMP, f"mesh_{nx}x{ny}x{nz}.json")
    subprocess.run([JELLOSIM, "--dump-mesh", str(nx), str(ny), str(nz), path],
                   check=True, stdout=subprocess.DEVNULL)
    with open(path) as f:
        m = json.load(f)
    m["positions"] = np.array(m["positions"])
    return m


def to_plot(p):
    """Sim is y-up; matplotlib 3D is z-up. (x, y, z) -> (x, z, y)."""
    p = np.asarray(p)
    return p[..., [0, 2, 1]]


def shade(color, normal):
    n = normal / (np.linalg.norm(normal) + 1e-12)
    k = 0.45 + 0.55 * max(0.0, float(n @ LIGHT))
    rgb = np.array(matplotlib.colors.to_rgb(color))
    return tuple(np.clip(rgb * k, 0, 1))


def outward_faces(P, tet):
    """The 4 faces of a tet as plot-space triangles, wound so normals point out."""
    out = []
    for a, b, c, opp in FACES:
        tri = to_plot(P[[tet[a], tet[b], tet[c]]])
        n = np.cross(tri[1] - tri[0], tri[2] - tri[0])
        if n @ (to_plot(P[tet[opp]]) - tri[0]) > 0:
            tri = tri[[0, 2, 1]]
            n = -n
        out.append((tri, n))
    return out


def setup(ax, title, lim):
    ax.set_title(title, fontsize=10, pad=0)
    ax.set_box_aspect((1, 1, 1))
    for setter, (lo, hi) in zip((ax.set_xlim, ax.set_ylim, ax.set_zlim), lim):
        setter(lo, hi)
    ax.view_init(elev=22, azim=-58)
    ax.set_axis_off()


def draw_exploded_cell(ax, mesh, cell_index, title, azim):
    P = mesh["positions"]
    tets = mesh["tets"][5 * cell_index: 5 * cell_index + 5]
    cell_center = P[np.unique(np.array(tets).ravel())].mean(axis=0)
    for t, tet in enumerate(tets):
        is_center = t == 4  # buildGrid emits the central tet last
        offset = 0.0 if is_center else 2.0 * (P[tet].mean(axis=0) - cell_center)
        faces = outward_faces(P + offset, tet)
        color = CENTER_COLOR if is_center else CORNER_COLORS[t]
        ax.add_collection3d(Poly3DCollection([f for f, _ in faces],
                                             facecolors=[shade(color, n) for _, n in faces],
                                             edgecolors="k", linewidths=0.6))
    lo = to_plot(P[np.array(tets).ravel()].min(axis=0)) - 0.6
    hi = to_plot(P[np.array(tets).ravel()].max(axis=0)) + 0.6
    setup(ax, title, list(zip(lo, hi)))
    # Look down a vertex of the central tet: no corner tet sits between it and the camera.
    ax.view_init(elev=35, azim=azim)


def draw_surface(ax, P, tris, title, color=SURFACE_COLOR):
    polys = [to_plot(P[list(t)]) for t in tris]
    # to_plot swaps two axes (a mirror), which reverses the winding, so the
    # plot-space cross product points inward; negate it to get the outward normal.
    colors = [shade(color, -np.cross(p[1] - p[0], p[2] - p[0])) for p in polys]
    ax.add_collection3d(Poly3DCollection(polys, facecolors=colors, edgecolors="k",
                                         linewidths=0.35))
    setup(ax, title, [(-0.05, 1.05)] * 3)


def cutaway_tris(mesh):
    """Boundary faces of the tets left after removing the octant nearest the viewer."""
    P = mesh["positions"]
    keep = []
    for tet in mesh["tets"]:
        c = P[tet].mean(axis=0)
        if not (c[0] > 0.5 and c[1] > 0.5 and c[2] < 0.5):  # remove the octant facing the camera
            keep.append(tet)
    count = {}
    for tet in keep:
        for a, b, c, _ in FACES:
            key = tuple(sorted((tet[a], tet[b], tet[c])))
            count[key] = count.get(key, 0) + 1
    tris = []
    for tet in keep:
        for a, b, c, opp in FACES:
            if count[tuple(sorted((tet[a], tet[b], tet[c])))] != 1:
                continue
            n = np.cross(P[tet[b]] - P[tet[a]], P[tet[c]] - P[tet[a]])
            if n @ (P[tet[opp]] - P[tet[a]]) > 0:
                tris.append((tet[a], tet[c], tet[b]))
            else:
                tris.append((tet[a], tet[b], tet[c]))
    return tris


def main():
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    prefix = sys.argv[1]
    if not os.path.exists(JELLOSIM):
        sys.exit(f"{JELLOSIM} not found; build first")

    two = load(2, 1, 1)    # cell 0 is even, cell 1 is odd
    block = load(4, 4, 4)

    fig = plt.figure(figsize=(13, 3.6))
    draw_exploded_cell(fig.add_subplot(1, 4, 1, projection="3d"), two, 0,
                       "(a) even cell: 4 corner tets + central tet", azim=45)
    draw_exploded_cell(fig.add_subplot(1, 4, 2, projection="3d"), two, 1,
                       "(b) odd cell: mirrored split", azim=-45)
    draw_surface(fig.add_subplot(1, 4, 3, projection="3d"), block["positions"],
                 block["surface_tris"], "(c) 4x4x4 boundary surface")
    draw_surface(fig.add_subplot(1, 4, 4, projection="3d"), block["positions"],
                 cutaway_tris(block), "(d) cutaway: interior tets")
    fig.subplots_adjust(left=0, right=1, top=0.92, bottom=0, wspace=0)

    os.makedirs(os.path.dirname(os.path.abspath(prefix)), exist_ok=True)
    for ext in ("png", "pdf"):
        fig.savefig(f"{prefix}.{ext}", dpi=200)
    print(f"Wrote {prefix}.png and {prefix}.pdf")


if __name__ == "__main__":
    main()
