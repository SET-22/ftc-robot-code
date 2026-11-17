"""Quick matplotlib renders of the assembly (no CAD viewer needed).
    python render_previews.py
"""
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import generate_face as g  # noqa: E402

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "exports")


def mesh(shape, tol=0.4):
    verts, tris = shape.tessellate(tol, 0.5)
    v = np.array([(p.x, p.y, p.z) for p in verts])
    return v[np.array(tris)]


def draw(ax, parts, elev, azim, hide=()):
    light = np.array([0.4, 0.9, 0.6])
    light /= np.linalg.norm(light)
    for name, shp, col, *_ in parts:
        if any(h in name for h in hide):
            continue
        tris = mesh(shp)
        c = g.COLORS[col].toTuple()[:3]
        n = np.cross(tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0])
        n /= (np.linalg.norm(n, axis=1)[:, None] + 1e-9)
        shade = 0.45 + 0.55 * np.abs(n @ light)
        fc = np.clip(np.array(c)[None, :] * shade[:, None], 0, 1)
        pc = Poly3DCollection(tris, facecolors=fc, edgecolors="none", linewidths=0)
        ax.add_collection3d(pc)
    ax.set_xlim(-75, 75)
    ax.set_ylim(-130, 20)
    ax.set_zlim(-110, 50)
    ax.set_box_aspect((150, 150, 160))
    ax.view_init(elev=elev, azim=azim)
    ax.set_axis_off()


def main():
    os.makedirs(OUT, exist_ok=True)
    parts = g.build(g.POSES["neutral"])
    fig = plt.figure(figsize=(14, 11))
    views = [("3/4 front", 15, 55), ("front", 0, 90), ("side", 0, 0), ("3/4 rear", 25, -130)]
    for i, (t, e, a) in enumerate(views):
        ax = fig.add_subplot(2, 2, i + 1, projection="3d")
        draw(ax, parts, e, a)
        ax.set_title(t)
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, "preview_views.png"), dpi=110)
    plt.close(fig)

    poses = ["neutral", "smile", "surprise", "angry", "sad", "blink"]
    fig = plt.figure(figsize=(15, 10))
    for i, p in enumerate(poses):
        ax = fig.add_subplot(2, 3, i + 1, projection="3d")
        draw(ax, g.build(g.POSES[p]), 8, 75,
             hide=("raspberry", "pca9685", "eye_servo_cradle", "servo_eye", "servo_lid", "rod_"))
        ax.set_title(p)
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, "preview_expressions.png"), dpi=110)
    print("wrote previews to", OUT)


if __name__ == "__main__":
    main()
