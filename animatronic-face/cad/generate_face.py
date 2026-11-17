"""
Animatronic Face v0.1 - parametric CAD generator (CadQuery)
===========================================================

Life-size (1:1 human) expressive robot face: 2 eyes with independent
pan/tilt (differential linkage), 4 independent eyelids, 4-point eyebrows,
jaw and 2 mouth corners. 15 servos total, one PCA9685 board.

Run:
    pip install cadquery
    python generate_face.py              # neutral pose
    python generate_face.py --pose smile  # any pose from POSES

Outputs (in ./exports):
    animatronic_face_assembly.step   <- import this into Onshape
    step/<part>.step                 <- individual printed parts
    stl/<part>.stl                   <- ready to slice
    preview_*.png                    <- quick renders

Coordinate system (mm):  origin = midpoint between the two eye centres
    +X = robot's left (viewer's right), +Y = forward (out of the face), +Z = up
"""

import argparse
import math
import os
import sys

import cadquery as cq
from cadquery import Vector as V

# --------------------------------------------------------------------------
# Global parameters (edit these)
# --------------------------------------------------------------------------
IPD = 63.0            # inter-pupil distance (adult average ~63 mm)
EYE_R = 12.5          # 25 mm eyeball (human eye ~24 mm)
EYE_WALL = 2.0
PIN_R = 1.05          # 2 mm steel rod pins (drill/ream to fit)
M2_TAP_R = 0.85       # M2 self-tapping into plastic
M3_CLR_R = 1.65

LOWER_LID = dict(r_in=13.0, r_out=14.2, sector=(-135, -45), plate_x=(12.9, 14.1))
UPPER_LID = dict(r_in=14.4, r_out=15.6, sector=(30, 115), plate_x=(14.6, 15.8))
FRAME_X = (16.1, 18.9)  # eye frame side plate (|dx| from eye centre)

FACE_PLATE_Y = (-34.0, -30.0)
BASE_Z = (-53.0, -50.0)

XE = IPD / 2  # right eye centre x (left eye = -XE)

# --------------------------------------------------------------------------
# Expression poses (degrees). These are the same numbers you'd send to the
# servos (after per-servo trim) - see ../README.md "Expression table".
# --------------------------------------------------------------------------
POSES = {
    "neutral":  dict(pan=0, tilt=0, lid_u=0, lid_l=0, brow_in=0, brow_out=0, jaw=0, corner=0),
    "smile":    dict(pan=0, tilt=0, lid_u=-10, lid_l=15, brow_in=5, brow_out=5, jaw=6, corner=25),
    "surprise": dict(pan=0, tilt=5, lid_u=8, lid_l=-5, brow_in=20, brow_out=20, jaw=18, corner=0),
    "angry":    dict(pan=0, tilt=-5, lid_u=-25, lid_l=10, brow_in=-20, brow_out=10, jaw=0, corner=-15),
    "sad":      dict(pan=0, tilt=-10, lid_u=-20, lid_l=0, brow_in=20, brow_out=-10, jaw=0, corner=-20),
    "blink":    dict(pan=0, tilt=0, lid_u=-55, lid_l=40, brow_in=0, brow_out=0, jaw=0, corner=0),
    "look_left": dict(pan=20, tilt=0, lid_u=0, lid_l=0, brow_in=0, brow_out=0, jaw=0, corner=0),
}


# --------------------------------------------------------------------------
# Small geometry helpers
# --------------------------------------------------------------------------
def box(x0, x1, y0, y1, z0, z1):
    return cq.Solid.makeBox(x1 - x0, y1 - y0, z1 - z0, V(x0, y0, z0))


def cyl(r, p0, p1):
    p0, p1 = V(*p0), V(*p1)
    d = p1 - p0
    return cq.Solid.makeCylinder(r, d.Length, p0, d.normalized())


def sphere(r, c=(0, 0, 0)):
    return cq.Workplane().sphere(r).val().translate(V(*c))


def union(*shapes):
    shapes = [s for s in shapes if s is not None]
    out = shapes[0]
    for s in shapes[1:]:
        out = out.fuse(s)
    return out.clean()


def yz_prism(pts, x0, x1):
    """Polygon given in (y, z) extruded along X from x0 to x1."""
    return (cq.Workplane("YZ").workplane(offset=x0).polyline(pts).close()
            .extrude(x1 - x0).val())


def xy_prism(pts, z0, z1):
    return (cq.Workplane("XY").workplane(offset=z0).polyline(pts).close()
            .extrude(z1 - z0).val())


def sector_pts(r0, r1, a0, a1, n=24):
    """Annular sector in a 2D plane, angles in degrees (0 = +first axis)."""
    outer = [(r1 * math.cos(math.radians(a)), r1 * math.sin(math.radians(a)))
             for a in [a0 + (a1 - a0) * i / n for i in range(n + 1)]]
    if r0 <= 0:
        return [(0.0, 0.0)] + outer
    inner = [(r0 * math.cos(math.radians(a)), r0 * math.sin(math.radians(a)))
             for a in [a1 - (a1 - a0) * i / n for i in range(n + 1)]]
    return outer + inner


def bar_yz(p0, p1, w, x0, x1):
    """Flat link in the YZ plane between 2 (y,z) points, width w, rounded ends."""
    (y0, z0), (y1, z1) = p0, p1
    L = math.hypot(y1 - y0, z1 - z0)
    ang = math.degrees(math.atan2(z1 - z0, y1 - y0))
    s = (cq.Workplane("YZ").workplane(offset=x0).slot2D(L + w, w, 0)
         .extrude(x1 - x0).val())
    s = s.rotate(V(0, 0, 0), V(1, 0, 0), ang)
    return s.translate(V(0, (y0 + y1) / 2, (z0 + z1) / 2))


def rot_x(shape, deg, cy, cz):
    return shape.rotate(V(0, cy, cz), V(1, cy, cz), deg)


def frame(origin, xdir, zdir):
    return cq.Location(cq.Plane(origin=origin, xDir=xdir, normal=zdir))


# --------------------------------------------------------------------------
# Purchased-part models (simplified envelopes, correct mounting geometry)
# Local frame: origin on shaft axis at the TOP face of the mounting tabs,
# +Z = output shaft, +X = along body length (shaft sits near the -X end).
# --------------------------------------------------------------------------
SERVOS = {
    # body x-range, half width, body z-range, tab x-range, tab holes x, spline top z
    "MG90S": dict(bx=(-5.9, 16.9), hw=6.1, bz=(-18.4, 4.3), tx=(-10.6, 21.6),
                  holes=[(-8.3, 0), (19.3, 0)], hole_r=1.0, dome_r=5.9,
                  dome_z=(4.3, 8.3), spline=(2.4, 11.3), horn_z=(11.3, 13.3)),
    "MG996R": dict(bx=(-10.0, 30.7), hw=9.85, bz=(-26.8, 10.0), tx=(-16.7, 37.4),
                   holes=[(-13.8, 5), (-13.8, -5), (34.5, 5), (34.5, -5)], hole_r=2.2,
                   dome_r=6.5, dome_z=(10.0, 14.0), spline=(3.0, 16.0), horn_z=(16.0, 18.5)),
}


def servo_body(kind):
    s = SERVOS[kind]
    body = box(s["bx"][0], s["bx"][1], -s["hw"], s["hw"], s["bz"][0], s["bz"][1])
    tabs = box(s["tx"][0], s["tx"][1], -s["hw"], s["hw"], -2.5, 0)
    for hx, hy in s["holes"]:
        tabs = tabs.cut(cyl(s["hole_r"], (hx, hy, -5), (hx, hy, 5)))
    dome = cyl(s["dome_r"], (0, 0, s["dome_z"][0]), (0, 0, s["dome_z"][1]))
    spl = cyl(s["spline"][0], (0, 0, s["dome_z"][1]), (0, 0, s["spline"][1]))
    return union(body, tabs, dome, spl)


def servo_pocket(kind, plate_t, clr=0.3):
    """Cutter (local frame) for a plate the tabs sit on (plate under the tabs)."""
    s = SERVOS[kind]
    c = box(s["bx"][0] - clr, s["bx"][1] + clr, -s["hw"] - clr, s["hw"] + clr,
            -2.5 - plate_t - 5, 0.1)
    for hx, hy in s["holes"]:
        c = c.fuse(cyl(s["hole_r"] * 0.85, (hx, hy, -2.5 - plate_t - 5), (hx, hy, 0)))
    return c


def servo_horn(kind, length, angle_deg=0):
    """Single-arm horn along local +X (rotated by angle about the shaft)."""
    s = SERVOS[kind]
    z0, z1 = s["horn_z"]
    w = 5.0 if kind == "MG90S" else 7.0
    arm = (cq.Workplane("XY").workplane(offset=z0).center(length / 2, 0)
           .slot2D(length + w, w, 0).extrude(z1 - z0).val())
    hub = cyl(3.6 if kind == "MG90S" else 5.0, (0, 0, z0), (0, 0, z1))
    return union(arm, hub).rotate(V(0, 0, 0), V(0, 0, 1), angle_deg)


class Placed:
    """A servo instance placed in the world."""

    def __init__(self, name, kind, tab_origin, xdir, zdir):
        self.name, self.kind = name, kind
        self.loc = frame(tab_origin, xdir, zdir)
        self.o, self.x, self.z = V(*tab_origin), V(*xdir), V(*zdir)

    def body(self):
        return servo_body(self.kind).moved(self.loc)

    def pocket(self, plate_t):
        return servo_pocket(self.kind, plate_t).moved(self.loc)

    def horn(self, length, angle=0):
        return servo_horn(self.kind, length, angle).moved(self.loc)

    def horn_top(self):
        return self.o + self.z * SERVOS[self.kind]["horn_z"][1]

    def horn_tip(self, length, angle, extra=0.0):
        """World point at the horn tip (optionally `extra` mm above the horn)."""
        a = math.radians(angle)
        y = self.z.cross(self.x)
        z = SERVOS[self.kind]["horn_z"][1] + extra
        return self.o + self.x * (length * math.cos(a)) + y * (length * math.sin(a)) + self.z * z


def ball_link_rod(p0, p1, name):
    """M2 threaded rod with two ball links (visual envelope)."""
    p0, p1 = V(*p0), V(*p1)
    rod = cyl(1.0, p0.toTuple(), p1.toTuple())
    b0 = sphere(2.4, p0.toTuple())
    b1 = sphere(2.4, p1.toTuple())
    return union(rod, b0, b1)


# --------------------------------------------------------------------------
# EYE MODULE (built for the RIGHT eye at the origin, then moved / mirrored)
# --------------------------------------------------------------------------
LUG_DX, LUG_Y, LUG_BALL_Z = 7.5, -11.0, 12.5


def eyeball():
    """Hollow 25 mm eyeball, internal U-joint lugs, 2 differential drive lugs.
    Print in two halves split on the XZ plane (y=0) or print upright with supports."""
    R, r = EYE_R, EYE_R - EYE_WALL
    e = sphere(R).cut(sphere(r))
    e = e.cut(cyl(10.0, (0, -20, 0), (0, -2, 0)))          # rear opening for the post
    e = e.cut(cyl(6.1, (0, R - 0.9, 0), (0, R + 2, 0)))     # iris recess: 12 mm glass cabochon
    e = e.cut(cyl(1.8, (0, 6, 0), (0, R + 2, 0)))            # pupil hole
    # internal tilt (X-axis) bearing lugs + ribs to the shell
    for s in (1, -1):
        ring = cyl(3.0, (3.6 * s, 0, 0), (5.0 * s, 0, 0))
        rib = box(*sorted((5.0 * s, 11.2 * s)), -2.5, 2.5, -2.5, 2.5)
        e = e.fuse(ring).fuse(rib)
    e = e.cut(cyl(PIN_R, (-14, 0, 0), (14, 0, 0)))           # 2 mm tilt pin
    # differential drive lugs (ball studs point up)
    for s in (1, -1):
        lug = box(*sorted((6.25 * s, 8.75 * s)), -13.0, -2.0, 6.0, 9.5)
        lug = lug.cut(cyl(M2_TAP_R, (LUG_DX * s, LUG_Y, 5), (LUG_DX * s, LUG_Y, 10)))
        e = e.fuse(lug)
    return e.clean()


def eye_cross():
    """U-joint centre block: horizontal tilt pin passes through, pan stubs from top/bottom."""
    c = cyl(3.0, (0, 0, -3.0), (0, 0, 3.0))
    c = c.cut(cyl(PIN_R, (-5, 0, 0), (5, 0, 0)))
    c = c.cut(cyl(M2_TAP_R, (0, 0, 3.1), (0, 0, 1.6)))
    c = c.cut(cyl(M2_TAP_R, (0, 0, -3.1), (0, 0, -1.6)))
    return c


def eye_frame():
    """Fixed frame: base, two side plates carrying the eyelid pivot pins,
    rear upright + post + fork carrying the eye's U-joint (pivot at eye centre)."""
    x0, x1 = FRAME_X
    base = union(box(-x1, x1, -14, 8, -27, -24), box(-8, 8, -30, -14, -27, -24))
    parts = [base]
    for s in (1, -1):
        xa, xb = sorted((x0 * s, x1 * s))
        plate = union(box(xa, xb, -6, 6, -27, 0), cyl(6, (xa, 0, 0), (xb, 0, 0)))
        plate = plate.cut(cyl(PIN_R, (xa - 1, 0, 0), (xb + 1, 0, 0)))
        parts.append(plate)
    upright = box(-4, 4, -30, -26, -27, 6)
    post = box(-2, 2, -26, -8, -5.3, 5.3)
    fork_t = union(box(-3, 3, -9, 3.5, 3.3, 5.3), cyl(3, (0, 0, 3.3), (0, 0, 5.3)))
    fork_b = union(box(-3, 3, -9, 3.5, -5.3, -3.3), cyl(3, (0, 0, -5.3), (0, 0, -3.3)))
    parts += [upright, post, fork_t, fork_b]
    f = union(*parts)
    f = f.cut(cyl(1.0, (0, 0, -6), (0, 0, 6)))                       # pan stub screws (M2)
    for z in (-15, 0):                                               # M3 into face plate
        f = f.cut(cyl(M3_CLR_R * 0.8, (0, -31, z), (0, -24, z)))
    return f


def eyelid(spec, lever_side, lever_angle, upper, lever_len=15.5):
    """Spherical-shell lid with side pivot plates and one drive lever.
    lever_side = +1 / -1 (which side the drive lever is on, in eye-local x)."""
    r_in, r_out = spec["r_in"], spec["r_out"]
    a0, a1 = spec["sector"]
    px0, px1 = spec["plate_x"]
    shell = sphere(r_out).cut(sphere(r_in))
    wedge = yz_prism(sector_pts(0, 30, a0, a1), -20, 20)
    trim = 11.0 if upper else px0
    shell = shell.intersect(wedge).intersect(box(-trim, trim, -30, 30, -30, 30))
    parts = [shell]
    for s in (1, -1):
        xa, xb = sorted((px0 * s, px1 * s))
        if upper:
            # L-arm from the trimmed shell edge out past the lower lid to the plate
            amid = (60, 85)
            ta, tb = sorted((11.0 * s - 0.4 * s, px0 * s))
            parts.append(yz_prism(sector_pts(9.5, 12.8, *amid), ta, tb))
            parts.append(yz_prism(sector_pts(0, 12.8, *amid), xa, xb))
        else:
            parts.append(yz_prism(sector_pts(0, 6.5, a0, a1), xa, xb))
        parts.append(cyl(3.5, (xa, 0, 0), (xb, 0, 0)))
    xa, xb = sorted((px0 * lever_side, px1 * lever_side))
    tip = (lever_len * math.cos(math.radians(lever_angle)),
           lever_len * math.sin(math.radians(lever_angle)))
    parts.append(bar_yz((0, 0), tip, 5.0, xa, xb))
    lid = union(*parts)
    lid = lid.cut(cyl(PIN_R + 0.1, (-20, 0, 0), (20, 0, 0)))
    lid = lid.cut(cyl(M2_TAP_R, (xa - 1, tip[0], tip[1]), (xb + 1, tip[0], tip[1])))
    return lid, tip


# Lid levers are ~perpendicular to their (fore-aft) push rods at mid-travel.
UPPER_LEVER_ANG = 117.0   # upper lid lever, temporal side, ball stud points outward
LOWER_LEVER_ANG = -110.0  # lower lid lever, temporal side, ball stud points inward
LOWER_LEVER_LEN = 19.0


def eye_module(pose):
    """Return dict of (name -> shape) for the RIGHT eye at origin, plus link points."""
    pan, tilt = pose["pan"], pose["tilt"]
    e = eyeball()
    cross = eye_cross()
    # tilt about X (through centre), then pan about Z
    e = e.rotate(V(0, 0, 0), V(1, 0, 0), -tilt)
    cross_r = cross
    e = e.rotate(V(0, 0, 0), V(0, 0, 1), pan)
    cross_r = cross_r.rotate(V(0, 0, 0), V(0, 0, 1), pan)

    up, up_tip = eyelid(UPPER_LID, +1, UPPER_LEVER_ANG, True)
    lo, lo_tip = eyelid(LOWER_LID, +1, LOWER_LEVER_ANG, False, LOWER_LEVER_LEN)
    up = rot_x(up, pose["lid_u"], 0, 0)
    lo = rot_x(lo, pose["lid_l"], 0, 0)

    def rot_pt(y, z, deg):
        a = math.radians(deg)
        return (y * math.cos(a) - z * math.sin(a), y * math.sin(a) + z * math.cos(a))

    upy, upz = rot_pt(*up_tip, pose["lid_u"])
    loy, loz = rot_pt(*lo_tip, pose["lid_l"])

    # eye lug ball centres (tilt then pan)
    def eye_pt(x, y, z):
        p = V(x, y, z)
        a = math.radians(-tilt)
        p = V(p.x, p.y * math.cos(a) - p.z * math.sin(a), p.y * math.sin(a) + p.z * math.cos(a))
        b = math.radians(pan)
        return V(p.x * math.cos(b) - p.y * math.sin(b), p.x * math.sin(b) + p.y * math.cos(b), p.z)

    links = {
        "eyeA": eye_pt(+LUG_DX, LUG_Y, LUG_BALL_Z),
        "eyeB": eye_pt(-LUG_DX, LUG_Y, LUG_BALL_Z),
        "lidU": V(UPPER_LID["plate_x"][1] + 3.0, upy, upz),
        "lidL": V(LOWER_LID["plate_x"][0] - 3.0, loy, loz),
    }
    return {"eyeball": e, "eye_cross": cross_r, "eye_frame": eye_frame(),
            "eyelid_upper": up, "eyelid_lower": lo}, links


# --------------------------------------------------------------------------
# STRUCTURE
# --------------------------------------------------------------------------
# Rod slot positions through the face plate (right side, mirrored for left)
ROD_SLOTS = [(39.0, 14.0, 6.0), (24.0, 14.0, 6.0), (50.3, 17.0, 7.0), (41.4, -17.6, 5.0)]


def face_plate():
    y0, y1 = FACE_PLATE_Y
    p = box(-62, 62, y0, y1, BASE_Z[1], 46)
    for s in (1, -1):
        for x, z, h in ROD_SLOTS:
            p = p.cut(box(s * x - 4.5, s * x + 4.5, y0 - 1, y1 + 1, z - h, z + h))
        for z in (-15, 0):  # eye frame bolts
            p = p.cut(cyl(M3_CLR_R, (s * XE, y0 - 1, z), (s * XE, y1 + 1, z)))
        for z in (24, 40):  # brow bracket bolts
            p = p.cut(cyl(M3_CLR_R, (s * 31.0, y0 - 1, z), (s * 31.0, y1 + 1, z)))
    p = p.cut(box(-6, 6, y0 - 1, y1 + 1, -46, -30))   # camera ribbon / wiring pass
    for x in (-50, -20, 20, 50):                     # base plate bolts (from below)
        p = p.cut(cyl(M3_CLR_R * 0.8, (x, -32, BASE_Z[1] - 1), (x, -32, BASE_Z[1] + 8)))
    return p.clean()


def base_plate():
    """Palate / skull base. Neck will bolt to the 4 holes at the rear later."""
    pts = [(-60, -130), (60, -130), (60, -20), (45, -5), (25, 2), (-25, 2), (-45, -5), (-60, -20)]
    p = xy_prism(pts, *BASE_Z)
    # upper lip / upper gum ridge (mouth corners pivot just behind it)
    lip_pts = [(-18, -4), (18, -4), (14, 2), (-14, 2)]
    p = p.fuse(xy_prism(lip_pts, -68, BASE_Z[0]))
    for x, y in [(-50, -32), (-20, -32), (20, -32), (50, -32)]:
        p = p.cut(cyl(M3_CLR_R, (x, y, -60), (x, y, -40)))
    for x, y in [(-30, -120), (30, -120), (-30, -95), (30, -95)]:   # neck / electronics
        p = p.cut(cyl(M3_CLR_R, (x, y, -60), (x, y, -40)))
    p = p.cut(box(-15, 15, -70, -40, -60, -40))                       # wiring pass
    for s in (1, -1):
        for x, y in [(62.5, -64), (62.5, -76)]:  # hinge brackets bolt holes
            p = p.cut(cyl(M3_CLR_R * 0.8, (s * 58, y, -60), (s * 58, y, -40)))
    return p.clean()


# ---- Eye servo cradle -----------------------------------------------------
FRONT_SHELF = dict(y=(-70.0, -37.0), z_top=-3.0, t=3.0)
BACK_SHELF = dict(y=(-104.0, -70.0), z_top=8.0, t=3.0)
FRONT_Y, BACK_Y = -48.0, -80.0
TAB = 2.5   # servo tab thickness: tab-top origin sits TAB above the plate it rests on
HORN_EYE = 10.0


def eye_servos():
    """8 MG90S for eyes+lids. Returns list of (Placed, horn_dir_sign, link_key, eye_side)."""
    out = []
    for side in (1, -1):              # +1 right eye (x>0), -1 left eye
        sx = side
        fz, bz = FRONT_SHELF["z_top"], BACK_SHELF["z_top"]
        # (link key, shaft_x, shaft_y, tab-top z, shaft dir); bodies point straight back (-Y)
        # lidL hangs upside-down under the front shelf so its rod runs below the eye.
        rows = [
            ("eyeA", 50.5, FRONT_Y, fz + TAB, 1),
            ("eyeB", 14.0, FRONT_Y, fz + TAB, 1),
            ("lidU", 40.3, BACK_Y, bz + TAB, 1),
            ("lidL", 31.0, FRONT_Y, fz - FRONT_SHELF["t"] - TAB, -1),
        ]
        for key, x, y, zt, zd in rows:
            p = Placed(f"servo_{key}_{'R' if side > 0 else 'L'}", "MG90S",
                       (sx * x, y, zt), (0, -1, 0), (0, 0, zd))
            out.append((p, key, side))
    return out


def horn_angle_for(p, key, side):
    """Point the single-arm horn at the rod line (along world x toward the link)."""
    target_x = {"eyeA": 39.0, "eyeB": 24.0, "lidU": 50.3, "lidL": 41.4}[key] * side
    yv = p.z.cross(p.x)
    return 90.0 if (target_x - p.o.x) * yv.x > 0 else -90.0


def eye_cradle(servos):
    fy, by = FRONT_SHELF["y"], BACK_SHELF["y"]
    ft, bt = FRONT_SHELF["z_top"], BACK_SHELF["z_top"]
    c = union(
        box(-68, 68, fy[0], fy[1], ft - 3, ft),
        box(-68, 68, by[0], by[1], bt - 3, bt),
        box(62, 68, by[0], fy[1], ft - 3, bt),          # side walls
        box(-68, -62, by[0], fy[1], ft - 3, bt),
        box(-68, 68, fy[1], fy[1] + 3, ft - 3, ft + 9),   # flange that bolts to face plate
    )
    for p, key, side in servos:
        c = c.cut(p.pocket(3.0))
    # wiring holes
    c = c.cut(box(-8, 8, by[0] + 4, by[1] - 4, bt - 5, bt + 1))
    for s in (1, -1):
        c = c.cut(cyl(M3_CLR_R, (s * 58, fy[1] - 1, ft + 4), (s * 58, fy[1] + 4, ft + 4)))
    return c.clean()


# ---- Eyebrows -------------------------------------------------------------
BROW_Z = 26.0
BROW_IN = dict(anchor=(14.0, 9.0), shaft_y=-8.7)
BROW_OUT = dict(anchor=(48.0, 6.0), shaft_y=-21.0)


def brow_servos():
    out = []
    for side in (1, -1):
        s = side
        # inner: shaft points to the centre (-x for right side), horn plane ~x=14
        p_in = Placed(f"servo_browIn_{'R' if s > 0 else 'L'}", "MG90S",
                      (s * 26.3, BROW_IN["shaft_y"], BROW_Z), (0, 0, 1), (-s, 0, 0))
        # outer: shaft points outward, horn plane ~x=48
        p_out = Placed(f"servo_browOut_{'R' if s > 0 else 'L'}", "MG90S",
                       (s * 35.7, BROW_OUT["shaft_y"], BROW_Z), (0, 0, 1), (s, 0, 0))
        out += [(p_in, "in", side), (p_out, "out", side)]
    return out


def brow_lever(p, kind, side, angle):
    """Printed lever bolted to the servo horn, ending in a 3 mm peg for the brow strip."""
    spec = BROW_IN if kind == "in" else BROW_OUT
    ay, az = spec["anchor"][1], BROW_Z
    sy, sz = spec["shaft_y"], BROW_Z
    L = math.hypot(ay - sy, az - sz)
    a = math.radians(angle)
    tip = (sy + L * math.cos(a), sz + L * math.sin(a))
    ht = p.horn_top()
    x0 = ht.x
    x1 = x0 + 2.5 * (1 if p.z.x > 0 else -1)
    xa, xb = sorted((x0, x1))
    lev = bar_yz((sy, sz), tip, 6.0, xa, xb)
    peg = cyl(1.5, ((xa + xb) / 2, tip[0], tip[1]), ((xa + xb) / 2, tip[0] + 6, tip[1]))
    return union(lev, peg), V((xa + xb) / 2, tip[0], tip[1])


def brow_bracket(side, servos):
    s = side
    xa, xb = sorted((s * 28.8, s * 33.2))
    flange = box(min(s * 24, s * 40), max(s * 24, s * 40), -30, -27.5, 18, 46)
    b = union(box(xa, xb, -30, 0, 18, 46), flange)
    for p, kind, sd in servos:
        if sd == side:
            b = b.cut(p.pocket(4.4))
    for z in (24, 40):
        b = b.cut(cyl(M3_CLR_R * 0.8, (s * 31, -31, z), (s * 31, -20, z)))
    return b.clean()


def brow_strip(side, tips):
    """TPU eyebrow strip (prints flat, flexes so inner/outer ends move independently)."""
    s = side
    pin_in, pin_out = tips
    xs = [10, 20, 31, 42, 52]
    ys = [9.5, 11.0, 11.5, 9.5, 6.0]
    out = []   # height linearly interpolated between the two pegs
    for i, x in enumerate(xs):
        t = (x - 10) / 42
        z = pin_in.z * (1 - t) + pin_out.z * t
        out.append((s * x, ys[i] + 4, z))
    strip = None
    for a, b in zip(out[:-1], out[1:]):
        seg = union(cyl(3.0, a, b), sphere(3.0, a), sphere(3.0, b))
        strip = seg if strip is None else strip.fuse(seg)
    for pt in (pin_in, pin_out):
        strip = strip.cut(cyl(1.6, (pt.x, pt.y - 1, pt.z), (pt.x, pt.y + 20, pt.z)))
    return strip.clean()


# ---- Jaw & mouth ----------------------------------------------------------
HINGE = (-70.0, -66.0)   # (y, z) of the jaw hinge axis (TMJ)


def jaw_servo():
    # MG996R, shaft along +X at the right hinge; body extends toward centre, long axis back (-Y)
    return Placed("servo_jaw", "MG996R", (39.5, HINGE[0], HINGE[1]), (0, -1, 0), (1, 0, 0))


def jaw(angle):
    hy, hz = HINGE
    outline = [(61, -78), (61, -35), (45, -10), (25, 3), (0, 7), (-25, 3), (-45, -10),
               (-61, -35), (-61, -78)]
    inner = [(47, -74), (47, -38), (35, -22), (18, -12), (0, -9), (-18, -12), (-35, -22),
             (-47, -38), (-47, -74)]
    floor = xy_prism(outline, -100, -96).cut(xy_prism(inner, -101, -95))
    front_outline = [(45, -10), (25, 3), (0, 7), (-25, 3), (-45, -10),
                     (-41, -13), (-23, -1), (0, 3), (23, -1), (41, -13)]
    front = xy_prism(front_outline, -100, -74)
    parts = [floor, front]
    for s in (1, -1):
        xa, xb = sorted((s * 58, s * 61))
        ram = yz_prism([(-80, -100), (-60, -100), (-60, hz), (-80, hz)], xa, xb)
        hub = cyl(9, (xa, hy, hz), (xb, hy, hz))
        parts += [ram, hub]
    j = union(*parts)
    for x in (-15, 0, 15):  # lower-lip skin anchor posts
        y = 7 - abs(x) * 0.18
        j = j.fuse(cyl(1.5, (x, y - 2, -77), (x, y + 6, -77)))
    for s in (1, -1):
        j = j.cut(cyl(1.6, (s * 55, hy, hz), (s * 66, hy, hz)))
    # horn bolt pattern on right ramus
    for a in range(0, 360, 90):
        yy = hy + 7 * math.cos(math.radians(a))
        zz = hz + 7 * math.sin(math.radians(a))
        j = j.cut(cyl(1.0, (55, yy, zz), (62, yy, zz)))
    return rot_x(j.clean(), -angle, hy, hz)


def hinge_brackets():
    hy, hz = HINGE
    out = []
    for s in (1, -1):
        xa, xb = sorted((s * 62, s * 65))
        plate = union(yz_prism([(-82, BASE_Z[0]), (-58, BASE_Z[0]), (-62, hz), (-78, hz)], xa, xb),
                      cyl(7, (xa, hy, hz), (xb, hy, hz)))
        foot = box(*sorted((s * 52, s * 65)), -82, -58, BASE_Z[0] - 3, BASE_Z[0])
        b = union(plate, foot)
        b = b.cut(cyl(1.6, (xa - 1, hy, hz), (xb + 1, hy, hz)))
        for y in (-64, -76):
            b = b.cut(cyl(M3_CLR_R * 0.8, (s * 58, y, -60), (s * 58, y, -45)))
        out.append(b.clean())
    return out


def jaw_servo_bracket(p):
    b = box(34, 37, -108, -52, -80, BASE_Z[0])
    b = b.fuse(box(26, 40, -108, -52, BASE_Z[0] - 3, BASE_Z[0]))
    b = b.cut(p.pocket(3.0))
    return b.clean()


def corner_servos():
    out = []
    for s in (1, -1):
        p = Placed(f"servo_corner_{'R' if s > 0 else 'L'}", "MG90S",
                   (s * 45, -15.0, -64.0), (s, 0, 0), (0, 1, 0))
        out.append((p, s))
    return out


CORNER_LEN = 18.0


def corner_lever(p, side, angle):
    """Lever on the corner servo horn; tip post is the mouth-corner skin anchor."""
    ht = p.horn_top()
    a = math.radians(180 - angle) if side > 0 else math.radians(angle)
    tip = V(ht.x + CORNER_LEN * math.cos(a), ht.y, ht.z + CORNER_LEN * math.sin(a))
    lev = (cq.Workplane("XY").center(CORNER_LEN / 2, 0)
           .slot2D(CORNER_LEN + 6, 6, 0).extrude(2.5).val())
    lev = lev.rotate(V(0, 0, 0), V(1, 0, 0), -90)            # extrusion now along +Y
    lev = lev.rotate(V(0, 0, 0), V(0, 1, 0), -math.degrees(a))
    lev = lev.translate(V(ht.x, ht.y, ht.z))
    post = cyl(1.8, (tip.x, ht.y, tip.z), (tip.x, ht.y + 9, tip.z))
    return union(lev, post), tip


def cheek_bracket(side, p):
    s = side
    b = box(*sorted((s * 30, s * 70)), -20.5, -17.5, -76, BASE_Z[0])
    b = b.fuse(box(*sorted((s * 30, s * 70)), -26, -17.5, BASE_Z[0] - 3, BASE_Z[0]))
    b = b.cut(p.pocket(3.0))
    return b.clean()


def camera_mount():
    m = box(-12, 12, -4, -2, BASE_Z[1], -21)
    m = m.fuse(box(-12, 12, -12, -2, BASE_Z[1], BASE_Z[1] + 3))
    for x in (-10.5, 10.5):
        for z in (-43.5, -31.0):
            m = m.cut(cyl(M2_TAP_R, (x, -5, z), (x, -1, z)))
    m = m.cut(box(-8, 8, -5, -1, -42, -26))  # sensor/connector clearance
    return m.clean()


def camera_model():
    return union(box(-12.5, 12.5, -2, -1, -46, -22), box(-4.25, 4.25, -1, 8, -38.25, -29.75))


def pi5_model():
    pi = box(-42.5, 42.5, -128, -72, -45, -43.4)
    pi = pi.fuse(box(-30, 20, -120, -80, -43.4, -30))  # components / fan envelope
    for x, y in [(-39, -124.5), (19, -124.5), (-39, -75.5), (19, -75.5)]:
        pi = pi.fuse(cyl(1.5, (x, y, BASE_Z[1]), (x, y, -45)))
    return pi


def pca9685_model():
    b = box(-31.25, 31.25, -125, -99.6, -22, -20.4)
    b = b.fuse(box(-24, 24, -112, -99.6, -20.4, -12))  # header pins
    for x, y in [(-28, -122), (28, -122), (-28, -102.6), (28, -102.6)]:
        b = b.fuse(cyl(1.5, (x, y, -43.4), (x, y, -22)))
    return b


# --------------------------------------------------------------------------
# BUILD
# --------------------------------------------------------------------------
PRINTED = "printed"
BOUGHT = "purchased"
COLORS = {
    PRINTED: cq.Color(0.85, 0.85, 0.88),
    "eye": cq.Color(0.97, 0.97, 0.95),
    "lid": cq.Color(0.95, 0.72, 0.6),
    "tpu": cq.Color(0.25, 0.18, 0.12),
    "servo": cq.Color(0.15, 0.25, 0.75),
    "servo_big": cq.Color(0.1, 0.1, 0.1),
    "horn": cq.Color(0.95, 0.95, 0.95),
    "rod": cq.Color(0.75, 0.6, 0.15),
    "pcb": cq.Color(0.1, 0.5, 0.2),
    "jaw": cq.Color(0.8, 0.82, 0.9),
}


def build(pose):
    parts = []   # (name, shape, color_key, category, export_as_part)

    def add(name, shape, color, cat=PRINTED, export=True):
        parts.append((name, shape, color, cat, export))

    # --- eyes
    mod, links = eye_module(pose)
    for side, tag in ((1, "R"), (-1, "L")):
        for k, shp in mod.items():
            s = shp
            if side < 0:
                s = s.mirror("YZ")
            s = s.translate(V(side * XE, 0, 0))
            col = {"eyeball": "eye", "eyelid_upper": "lid", "eyelid_lower": "lid"}.get(k, PRINTED)
            add(f"{k}_{tag}", s, col, PRINTED, export=(tag == "R"))

    # --- structure
    add("face_plate", face_plate(), PRINTED)
    add("base_plate", base_plate(), PRINTED)

    # --- eye/lid servos + rods
    es = eye_servos()
    add("eye_servo_cradle", eye_cradle(es), PRINTED)
    servo_offsets = {"eyeA": pose["pan"] * 0.6 - pose["tilt"] * 0.6,
                     "eyeB": -pose["pan"] * 0.6 - pose["tilt"] * 0.6,
                     "lidU": -pose["lid_u"] * 0.8, "lidL": pose["lid_l"] * 0.8}
    for p, key, side in es:
        add(p.name, p.body(), "servo", BOUGHT, False)
        ang = horn_angle_for(p, key, side)
        add(p.name + "_horn", p.horn(HORN_EYE, ang), "horn", BOUGHT, False)
        # ball stud on the horn: above it, except lidL (inverted) where it points back up
        tip = p.horn_tip(HORN_EYE, ang, extra=(-4.4 if key == "lidL" else 2.5))
        lk = links[key]
        target = V(side * (XE + lk.x), lk.y, lk.z)   # left eye is the mirror image
        add(f"rod_{key}_{'R' if side > 0 else 'L'}", ball_link_rod(tip, target, key), "rod",
            BOUGHT, False)

    # --- brows
    bs = brow_servos()
    tips = {}
    for p, kind, side in bs:
        add(p.name, p.body(), "servo", BOUGHT, False)
        yv = p.z.cross(p.x)
        add(p.name + "_horn", p.horn(8, 90 if yv.y > 0 else -90), "horn", BOUGHT, False)
        ang = pose["brow_in"] if kind == "in" else pose["brow_out"]
        lev, tip = brow_lever(p, kind, side, ang)
        tag = "R" if side > 0 else "L"
        add(f"brow_lever_{kind}_{tag}", lev, PRINTED, PRINTED, export=(side > 0))
        tips[(kind, side)] = tip
    for side, tag in ((1, "R"), (-1, "L")):
        add(f"brow_bracket_{tag}", brow_bracket(side, bs), PRINTED, PRINTED, export=(side > 0))
        add(f"brow_strip_{tag}", brow_strip(side, (tips[("in", side)], tips[("out", side)])),
            "tpu", PRINTED, export=(side > 0))

    # --- jaw
    js = jaw_servo()
    add("servo_jaw", js.body(), "servo_big", BOUGHT, False)
    add("jaw", jaw(pose["jaw"]), "jaw", PRINTED)
    add("jaw_servo_bracket", jaw_servo_bracket(js), PRINTED)
    hb = hinge_brackets()
    add("jaw_hinge_bracket_R", hb[0], PRINTED)
    add("jaw_hinge_bracket_L", hb[1], PRINTED, PRINTED, export=False)

    # --- mouth corners
    for p, side in corner_servos():
        tag = "R" if side > 0 else "L"
        add(p.name, p.body(), "servo", BOUGHT, False)
        lev, _ = corner_lever(p, side, pose["corner"])
        add(f"mouth_corner_lever_{tag}", lev, PRINTED, PRINTED, export=(side > 0))
        add(f"cheek_bracket_{tag}", cheek_bracket(side, p), PRINTED, PRINTED, export=(side > 0))

    # --- sensors & electronics
    add("camera_mount", camera_mount(), PRINTED)
    add("pi_camera_module_3", camera_model(), "pcb", BOUGHT, False)
    add("raspberry_pi_5", pi5_model(), "pcb", BOUGHT, False)
    add("pca9685_servo_driver", pca9685_model(), "pcb", BOUGHT, False)
    return parts


def interference_report(parts, ignore_pairs=()):
    """Pairwise solid intersection check (bounding-box prefilter)."""
    hits = []
    shapes = [(n, s) for n, s, *_ in parts]
    bbs = [s.BoundingBox() for _, s in shapes]
    for i in range(len(shapes)):
        for j in range(i + 1, len(shapes)):
            a, b = bbs[i], bbs[j]
            if (a.xmax < b.xmin or b.xmax < a.xmin or a.ymax < b.ymin or b.ymax < a.ymin
                    or a.zmax < b.zmin or b.zmax < a.zmin):
                continue
            ni, nj = shapes[i][0], shapes[j][0]
            if any((x in ni and y in nj) or (x in nj and y in ni) for x, y in ignore_pairs):
                continue
            try:
                v = shapes[i][1].intersect(shapes[j][1]).Volume()
            except Exception:
                v = -1
            if v > 0.5:
                hits.append((ni, nj, round(v, 1)))
    return hits


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--pose", default="neutral", choices=list(POSES))
    ap.add_argument("--out", default=os.path.join(os.path.dirname(os.path.abspath(__file__)), "exports"))
    ap.add_argument("--check", action="store_true", help="run interference check")
    ap.add_argument("--no-parts", action="store_true", help="only write the assembly")
    args = ap.parse_args()

    pose = POSES[args.pose]
    parts = build(pose)
    os.makedirs(args.out, exist_ok=True)

    assy = cq.Assembly(name="animatronic_face")
    for name, shp, col, cat, _ in parts:
        assy.add(shp, name=name, color=COLORS[col])
    suffix = "" if args.pose == "neutral" else f"_{args.pose}"
    step_path = os.path.join(args.out, f"animatronic_face_assembly{suffix}.step")
    assy.export(step_path)
    print("wrote", step_path)

    if not args.no_parts:
        os.makedirs(os.path.join(args.out, "step"), exist_ok=True)
        os.makedirs(os.path.join(args.out, "stl"), exist_ok=True)
        for name, shp, col, cat, export in parts:
            if cat != PRINTED or not export:
                continue
            base = name[:-2] if name.endswith("_R") else name
            if base == "eyeball":   # print as two halves so the U-joint cross can go inside
                halves = {"eyeball_front_half": box(-30, 30, 0, 30, -30, 30),
                          "eyeball_rear_half": box(-30, 30, -30, 0, -30, 30)}
                for hn, hb in halves.items():
                    h = shp.translate(V(-XE, 0, 0)).intersect(hb)
                    cq.exporters.export(cq.Workplane().add(h), os.path.join(args.out, "step", hn + ".step"))
                    cq.exporters.export(cq.Workplane().add(h), os.path.join(args.out, "stl", hn + ".stl"),
                                        tolerance=0.05, angularTolerance=0.15)
                continue
            cq.exporters.export(cq.Workplane().add(shp), os.path.join(args.out, "step", base + ".step"))
            cq.exporters.export(cq.Workplane().add(shp), os.path.join(args.out, "stl", base + ".stl"),
                                tolerance=0.05, angularTolerance=0.15)
        print("wrote individual parts")

    if args.check:
        ignore = [("horn", "servo_"), ("rod_", "_horn"), ("rod_", "eyeball"),
                  ("rod_", "eyelid"), ("eye_cross", "eye_frame"), ("eye_cross", "eyeball"),
                  ("brow_lever", "_horn"), ("mouth_corner_lever", "servo_corner"),
                  ("raspberry_pi_5", "base_plate"), ("pca9685", "raspberry_pi_5"),
                  ("camera_mount", "base_plate"), ("pi_camera", "camera_mount"),
                  ("brow_strip", "brow_lever")]
        hits = interference_report(parts, ignore)
        print(f"interference check ({args.pose}): {len(hits)} overlaps")
        for h in hits:
            print("  ", h)
    return parts


if __name__ == "__main__":
    main()
