#!/usr/bin/env python3
"""Lay out images in a labeled grid (one figure for the report).

Usage:
  scripts/image_grid.py OUT.png --cols 3 IMG "label" IMG "label" ... [--title TEXT]

Each image gets its label underneath. Images keep their pixels (no
resampling beyond matplotlib's display), so use same-size inputs.
"""
import argparse
import math

import matplotlib
matplotlib.use("Agg")
import matplotlib.image as mpimg
import matplotlib.pyplot as plt


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("out")
    ap.add_argument("items", nargs="+", help="alternating image path and label")
    ap.add_argument("--cols", type=int, default=3)
    ap.add_argument("--title", default="")
    ap.add_argument("--width", type=float, default=4.0, help="inches per image")
    args = ap.parse_args()
    if len(args.items) % 2:
        ap.error("items must alternate IMAGE LABEL")
    pairs = list(zip(args.items[0::2], args.items[1::2]))
    imgs = [mpimg.imread(p) for p, _ in pairs]
    h, w = imgs[0].shape[:2]
    cols = min(args.cols, len(pairs))
    rows = math.ceil(len(pairs) / cols)
    cell_h = args.width * h / w
    fig, axes = plt.subplots(rows, cols, figsize=(args.width * cols, (cell_h + 0.35) * rows + (0.4 if args.title else 0)),
                             squeeze=False)
    for ax in axes.flat:
        ax.axis("off")
    for ax, img, (_, label) in zip(axes.flat, imgs, pairs):
        ax.imshow(img)
        ax.text(0.5, -0.03, label, transform=ax.transAxes, ha="center", va="top", fontsize=10)
    if args.title:
        fig.suptitle(args.title, fontsize=11)
    fig.tight_layout(pad=0.4)
    # bbox_inches="tight" keeps the labels under the last row inside the image.
    fig.savefig(args.out, dpi=max(100, w / args.width), bbox_inches="tight", pad_inches=0.05)
    print(f"wrote {args.out}")


if __name__ == "__main__":
    main()
