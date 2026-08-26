#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
#
# Copyright (c) 2026 Warioware64
#
# Standalone NPE (Nitro Particle Entity) editor.
#
# Edit a particle effect with live preview, save as a `.npe` binary the
# NEAParticle runtime can load directly. Tkinter plus Pillow: the preview
# composites real sprites with rotation, scale, alpha and additive blending,
# none of which Tk's own drawing can do, and tools/img2ds already needs Pillow.
#
# Usage:
#     python3 tools/npe_editor/npe_editor.py [path/to/file.npe]

"""NPE particle editor (tkinter).

Left panel: tabbed parameter controls.
Right panel: 2D side-view preview running the same simulation logic as the
  C runtime in NEAParticle.c, so what you see is what the DS will show.

An effect names the material it draws with but does not contain it, so the
editor cannot know what the particle looks like. Point it at the image with
File -> Import sprite image and it is recorded in a sidecar beside the effect
(see editor_sidecar.py); the .npe itself is never touched by that.

With a sprite loaded the preview composites rather than drawing shapes, which is
what makes the two flags that matter visible: additive blending becomes a real
add instead of a colour mixed toward the background, and a sprite sheet actually
plays its cells.
"""

import json
import math
import os
import random
import sys
import tkinter as tk
from tkinter import colorchooser, filedialog, messagebox, ttk

try:
    from PIL import Image, ImageTk
except ImportError:
    sys.exit("This editor needs Pillow for its preview: pip install Pillow")

# Allow running the script from anywhere by adding its dir to sys.path.
_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)
import npe_format  # noqa: E402
import editor_sidecar as SC  # noqa: E402

# ---------------------------------------------------------------------------
# Simulator (mirrors NEAParticle.c -- keep the two in lockstep)
# ---------------------------------------------------------------------------

class Particle:
    __slots__ = ("px", "py", "pz", "vx", "vy", "vz",
                 "age", "life", "rot", "rot_vel",
                 "size", "r", "g", "b", "a", "alive")
    def __init__(self):
        self.alive = False

class Simulator:
    """Pure-Python copy of NEAParticle.c's per-frame logic."""

    def __init__(self, effect):
        self.set_effect(effect)
        self._rng = random.Random(0x12345678)
        self._emit_acc = 0.0

    def set_effect(self, effect):
        self.eff = effect
        n = max(1, int(effect.get("max_particles", 64)))
        self.pool = [Particle() for _ in range(n)]
        self._emit_acc = 0.0

    def clear(self):
        for p in self.pool:
            p.alive = False

    def _rand_range(self, lo, hi):
        if lo >= hi: return lo
        return self._rng.uniform(lo, hi)

    def _rand_int(self, lo, hi):
        if lo >= hi: return lo
        return self._rng.randint(int(lo), int(hi))

    def _random_dir(self):
        d = self.eff["initial_dir"]
        cone = int(self.eff["cone_spread"])
        if d[0] == 0 and d[1] == 0 and d[2] == 0:
            # Both angles span the full 0..511 the runtime uses. That makes the
            # polar angle cover a whole turn rather than a half, which
            # double-covers the sphere -- harmless, and what the hardware does.
            theta = self._rng.uniform(0, 511) * math.pi / 256.0
            phi   = self._rng.uniform(0, 511) * math.pi / 256.0
            return (math.sin(theta) * math.cos(phi),
                    math.cos(theta),
                    math.sin(theta) * math.sin(phi))
        # Normalize the base direction.
        n = math.sqrt(d[0]*d[0] + d[1]*d[1] + d[2]*d[2]) or 1.0
        dx, dy, dz = d[0]/n, d[1]/n, d[2]/n
        if cone == 0:
            return (dx, dy, dz)
        # Build orthonormal basis perpendicular to dir.
        if abs(dy) > 0.98:
            uh = (1.0, 0.0, 0.0)
        else:
            uh = (0.0, 1.0, 0.0)
        rx = dy*uh[2] - dz*uh[1]
        ry = dz*uh[0] - dx*uh[2]
        rz = dx*uh[1] - dy*uh[0]
        rl = math.sqrt(rx*rx + ry*ry + rz*rz) or 1.0
        rx, ry, rz = rx/rl, ry/rl, rz/rl
        ux = ry*dz - rz*dy
        uy = rz*dx - rx*dz
        uz = rx*dy - ry*dx
        # The runtime picks theta in 0..cone_spread NEA angle units and feeds
        # sinLerp(theta << 6), against a 32768-unit full circle. So one unit is
        # pi/256 radians and cone_spread 256 is a half-turn -- not the pi/511
        # this used to assume, which made the editor's cone half the width of
        # the one the DS actually produces.
        theta = self._rng.uniform(0, cone) * math.pi / 256.0
        phi   = self._rng.uniform(0, 2 * math.pi)
        st, ct = math.sin(theta), math.cos(theta)
        sp, cp = math.sin(phi),   math.cos(phi)
        return (ct*dx + st*cp*rx + st*sp*ux,
                ct*dy + st*cp*ry + st*sp*uy,
                ct*dz + st*cp*rz + st*sp*uz)

    def _free_slot(self):
        for i, p in enumerate(self.pool):
            if not p.alive:
                return i
        return -1

    def emit_one(self, origin=(0.0, 0.0, 0.0)):
        i = self._free_slot()
        if i < 0:
            return
        p = self.pool[i]
        e = self.eff
        p.px = origin[0] + self._rand_range(e["pos_min"][0], e["pos_max"][0])
        p.py = origin[1] + self._rand_range(e["pos_min"][1], e["pos_max"][1])
        p.pz = origin[2] + self._rand_range(e["pos_min"][2], e["pos_max"][2])
        dx, dy, dz = self._random_dir()
        spd = self._rand_range(e["speed_min"], e["speed_max"])
        p.vx, p.vy, p.vz = dx * spd, dy * spd, dz * spd
        p.age  = 0
        # The runtime substitutes 60 frames for a zero life, rather than
        # clamping to one.
        p.life = self._rand_int(e["life_min_frames"], e["life_max_frames"]) or 60
        p.rot  = int(e["base_rotation"])
        p.rot_vel = self._rand_int(e["ang_vel_min"], e["ang_vel_max"])
        p.size = float(e["base_size"]) if e["base_size"] else 1.0
        if e["color_keys"]:
            _, r, g, b, a = e["color_keys"][0]
            p.r, p.g, p.b, p.a = r, g, b, a
        else:
            p.r = p.g = p.b = p.a = 255
        p.alive = True

    def burst(self, count, origin=(0.0, 0.0, 0.0)):
        for _ in range(count):
            self.emit_one(origin)

    @staticmethod
    def _sample_color(keys, t):
        if not keys:                  return (255, 255, 255, 255)
        if len(keys) == 1 or t <= keys[0][0]:
            return tuple(keys[0][1:5])
        if t >= keys[-1][0]:
            return tuple(keys[-1][1:5])
        for i in range(len(keys) - 1):
            ta, ra, ga, ba, aa = keys[i]
            tb, rb, gb, bb, ab = keys[i+1]
            if ta <= t <= tb:
                u = (t - ta) / max(1, (tb - ta))
                return (int(ra + (rb - ra) * u),
                        int(ga + (gb - ga) * u),
                        int(ba + (bb - ba) * u),
                        int(aa + (ab - aa) * u))
        return tuple(keys[-1][1:5])

    @staticmethod
    def _sample_size(keys, t, fallback):
        if not keys:                  return fallback
        if len(keys) == 1 or t <= keys[0][0]:
            return keys[0][1]
        if t >= keys[-1][0]:
            return keys[-1][1]
        for i in range(len(keys) - 1):
            ta, sa = keys[i]
            tb, sb = keys[i+1]
            if ta <= t <= tb:
                u = (t - ta) / max(1, (tb - ta))
                return sa + (sb - sa) * u
        return keys[-1][1]

    def step(self, origin=(0.0, 0.0, 0.0)):
        e = self.eff
        # Continuous emission (60 fps assumed)
        if e["flags"].get("continuous") and e["emit_rate"] > 0:
            self._emit_acc += e["emit_rate"] / 60.0
            while self._emit_acc >= 1.0:
                self.emit_one(origin)
                self._emit_acc -= 1.0
        # Per-particle
        gx, gy, gz = e["gravity"]
        drag = e["drag"]
        for p in self.pool:
            if not p.alive: continue
            p.vx += gx; p.vy += gy; p.vz += gz
            if drag:
                p.vx *= (1.0 - drag); p.vy *= (1.0 - drag); p.vz *= (1.0 - drag)
            p.px += p.vx; p.py += p.vy; p.pz += p.vz
            p.rot = (p.rot + p.rot_vel) & 0x1FF
            p.age += 1
            if p.age >= p.life:
                p.alive = False
                continue
            t = int(1000 * p.age / p.life)
            r, g, b, a = self._sample_color(e["color_keys"], t)
            p.r, p.g, p.b, p.a = r, g, b, a
            p.size = self._sample_size(e["size_keys"], t, p.size)

# ---------------------------------------------------------------------------
# Keyframe graph
# ---------------------------------------------------------------------------

# The same palette and interaction the animmat editor's timeline uses, so the
# two tools feel like one set. Colour and size over life are keyframed curves
# exactly like a material animation track, and they used to be edited through a
# listbox and a chain of modal prompts -- three dialogs to move one colour key.

GRAPH_BG      = "#1b1b1f"
GRAPH_GRID_H  = "#2a2a30"
GRAPH_GRID_V  = "#26262c"
GRAPH_AXIS    = "#777"
GRAPH_CURVE   = "#4da3ff"
GRAPH_KEY     = "#ff8c42"
GRAPH_KEY_SEL = "#ffd24d"
GRAPH_HINT    = "#666"

GRAPH_HINT_TEXT = ("Click a key to select, drag to move, double-click empty "
                   "space to add, right-click a key to delete.")


class LifeGraph(tk.Canvas):
    """A keyframe curve over a particle's life, drawn like an animmat track.

    Both of the things an emitter animates over life are curves with draggable
    keys, so both get the same widget:

      size   x is t (0..1000), y is the size in world units.
      color  x is t, y is alpha. RGB is a gradient behind the curve and a
             swatch on each key, because a colour is not a height -- but alpha
             is, and it is the channel that decides whether anything is
             visible at all.
    """

    def __init__(self, parent, editor, mode):
        super().__init__(parent, height=190, background=GRAPH_BG,
                         highlightthickness=0)
        self.editor = editor
        self.mode = mode          # "size" or "color"
        self.sel = None
        self._drag = None
        self._range = None

        self.bind("<Configure>", lambda e: self.redraw())
        self.bind("<Button-1>", self._on_click)
        self.bind("<B1-Motion>", self._on_drag)
        self.bind("<ButtonRelease-1>", lambda e: self._end_drag())
        self.bind("<Double-Button-1>", self._on_double)
        self.bind("<Button-3>", self._on_right)

    # -- data ------------------------------------------------------------

    @property
    def keys(self):
        return self.editor.effect[
            "color_keys" if self.mode == "color" else "size_keys"]

    def _set_keys(self, keys):
        self.editor.effect[
            "color_keys" if self.mode == "color" else "size_keys"] = keys

    def _y_value(self, key):
        """The number this key plots at."""
        return key[4] if self.mode == "color" else key[1]

    def _with_y(self, key, t, value):
        if self.mode == "color":
            return (t, key[1], key[2], key[3], int(max(0, min(255, value))))
        return (t, max(0.0, float(value)))

    def _y_span(self):
        if self.mode == "color":
            return 0.0, 255.0
        top = max((s for _, s in self.keys), default=1.0) or 1.0
        return 0.0, top * 1.15

    def _sample(self, t):
        if self.mode == "color":
            return Simulator._sample_color(self.keys, t)[3]
        return Simulator._sample_size(self.keys, t, 0.0)

    # -- geometry ---------------------------------------------------------

    def _geom(self):
        w = self.winfo_width()
        h = self.winfo_height()
        if w <= 1:
            w = 420
        if h <= 1:
            h = int(self.cget("height"))
        return w, h, 44, 12, w - 14, h - 26

    def _to_px(self, t, value, lo, hi):
        _, _, x0, y0, x1, y1 = self._geom()
        x = x0 + (x1 - x0) * t / 1000.0
        span = (hi - lo) or 1.0
        y = y1 - (y1 - y0) * (value - lo) / span
        return x, y

    def _from_px(self, px, py, lo, hi):
        _, _, x0, y0, x1, y1 = self._geom()
        t = round((px - x0) / max(1, x1 - x0) * 1000.0)
        span = (hi - lo) or 1.0
        value = lo + (y1 - py) / max(1, y1 - y0) * span
        return max(0, min(1000, int(t))), value

    # -- drawing ----------------------------------------------------------

    def redraw(self):
        self.delete("all")
        w, h, x0, y0, x1, y1 = self._geom()
        lo, hi = self._y_span()

        if self.mode == "color":
            self._draw_gradient(x0, y0, x1, y1)

        for i in range(5):
            y = y0 + (y1 - y0) * i / 4
            self.create_line(x0, y, x1, y, fill=GRAPH_GRID_H)
            val = hi - (hi - lo) * i / 4
            label = f"{val:.0f}" if self.mode == "color" else f"{val:.2f}"
            self.create_text(x0 - 5, y, text=label, fill=GRAPH_AXIS,
                             anchor="e", font=("TkFixedFont", 7))

        for t in range(0, 1001, 100):
            x, _ = self._to_px(t, lo, lo, hi)
            self.create_line(x, y0, x, y1, fill=GRAPH_GRID_V)
            self.create_text(x, y1 + 9, text=str(t), fill=GRAPH_AXIS,
                             font=("TkFixedFont", 7))

        # The evaluated curve: sampled through the same functions the simulator
        # and the runtime use, so the graph cannot disagree with the preview.
        pts = []
        for i in range(0, 121):
            t = i * 1000 // 120
            pts.extend(self._to_px(t, self._sample(t), lo, hi))
        if len(pts) >= 4:
            self.create_line(*pts, fill=GRAPH_CURVE, width=2, joinstyle="round")

        for i, key in enumerate(self.keys):
            x, y = self._to_px(key[0], self._y_value(key), lo, hi)
            selected = (i == self.sel)
            if self.mode == "color":
                # The swatch sits on top of the gradient, so its outline has to
                # read against a light band as well as a dark one -- the
                # canvas-coloured outline the size keys use vanishes here.
                swatch = f"#{key[1]:02x}{key[2]:02x}{key[3]:02x}"
                self.create_rectangle(x - 6, y - 6, x + 6, y + 6,
                                      fill=swatch,
                                      outline=GRAPH_KEY_SEL if selected
                                      else "#e8e8ea", width=2)
            else:
                self.create_rectangle(x - 4, y - 4, x + 4, y + 4,
                                      fill=GRAPH_KEY_SEL if selected
                                      else GRAPH_KEY, outline=GRAPH_BG)

        if not self.keys:
            self.create_text((x0 + x1) / 2, (y0 + y1) / 2, fill=GRAPH_HINT,
                             text="No keyframes - double-click to add one")

    def _draw_gradient(self, x0, y0, x1, y1):
        """The colour over life, behind the alpha curve."""
        step = 2
        for x in range(int(x0), int(x1), step):
            t = int(1000 * (x - x0) / max(1, x1 - x0))
            r, g, b, a = Simulator._sample_color(self.keys, t)
            # Composited over the canvas so alpha is visible as fading, the
            # same way the preview composites it.
            af = a / 255.0
            cr = int(0x1b * (1 - af) + r * af)
            cg = int(0x1b * (1 - af) + g * af)
            cb = int(0x1f * (1 - af) + b * af)
            self.create_rectangle(x, y0, x + step, y1,
                                  fill=f"#{cr:02x}{cg:02x}{cb:02x}", width=0)

    # -- interaction ------------------------------------------------------

    def _hit(self, px, py):
        lo, hi = self._y_span()
        for i, key in enumerate(self.keys):
            x, y = self._to_px(key[0], self._y_value(key), lo, hi)
            if abs(px - x) <= 7 and abs(py - y) <= 7:
                return i
        return None

    def _on_click(self, e):
        self.sel = self._hit(e.x, e.y)
        self._drag = self.sel
        self._range = self._y_span() if self.sel is not None else None
        self.editor.on_graph_select(self)
        self.redraw()

    def _on_drag(self, e):
        if self._drag is None:
            return
        lo, hi = self._range
        t, value = self._from_px(e.x, e.y, lo, hi)

        keys = list(self.keys)

        # Two keys on one t leave a zero-length span, which the samplers skip --
        # so one of them silently stops mattering. Refuse the move sideways and
        # let the drag continue vertically, as the animmat timeline does.
        if any(j != self._drag and k[0] == t for j, k in enumerate(keys)):
            t = keys[self._drag][0]

        moved = self._with_y(keys[self._drag], t, value)
        keys[self._drag] = moved
        keys.sort(key=lambda k: k[0])

        self._set_keys(keys)
        self._drag = keys.index(moved)
        self.sel = self._drag

        self.editor.on_graph_select(self)
        self.editor._mark_dirty()
        self.redraw()

    def _end_drag(self):
        self._drag = None
        self._range = None

    def _on_double(self, e):
        lo, hi = self._y_span()
        t, value = self._from_px(e.x, e.y, lo, hi)

        keys = [k for k in self.keys if k[0] != t]
        if self.mode == "color":
            r, g, b, _a = Simulator._sample_color(self.keys, t)
            keys.append((t, r, g, b, int(max(0, min(255, value)))))
        else:
            keys.append((t, max(0.0, float(value))))

        keys.sort(key=lambda k: k[0])
        self._set_keys(keys)
        self.sel = next(i for i, k in enumerate(keys) if k[0] == t)

        self.editor.on_graph_select(self)
        self.editor._mark_dirty()
        self.redraw()

    def _on_right(self, e):
        i = self._hit(e.x, e.y)
        if i is None:
            return
        keys = list(self.keys)
        del keys[i]
        self._set_keys(keys)
        self.sel = None
        self.editor.on_graph_select(self)
        self.editor._mark_dirty()
        self.redraw()


# ---------------------------------------------------------------------------
# UI
# ---------------------------------------------------------------------------

PREVIEW_W = 520
PREVIEW_H = 520
# World units shown in the preview window.
PREVIEW_SCALE_DEFAULT = 80.0  # pixels per world unit

class FloatVar(tk.DoubleVar):
    """DoubleVar that round-trips small floats cleanly."""
    pass

class NpeEditor:
    def __init__(self, root, initial_path=None):
        self.root = root
        root.title("NPE editor -- Nitro Engine Advanced")
        root.geometry("1080x720")

        self.path = None
        self.effect = npe_format.default_effect()
        self.sim = Simulator(self.effect)
        self._preview_scale = PREVIEW_SCALE_DEFAULT

        # Sprite preview state. _sprite_cache holds transformed copies keyed by
        # (cell, rotation bucket, size bucket): rotating and scaling every
        # particle every frame is far too slow in Python at 30 fps, and the DS
        # itself does not resolve rotation any more finely than this.
        self._sprite_path = None
        self._sprite = None
        self._sprite_cache = {}
        self._preview_photo = None
        self._dirty = False

        self._build_menu()
        self._build_layout()
        if initial_path:
            self._open_file(initial_path)
        else:
            self._refresh_all()

        # Live preview tick (~30 fps is plenty for a 2D editor).
        self._tick()

    # -- File menu ---------------------------------------------------------

    def _build_menu(self):
        m = tk.Menu(self.root)
        fm = tk.Menu(m, tearoff=0)
        fm.add_command(label="New",         command=self._new,  accelerator="Ctrl+N")
        fm.add_command(label="Open .npe...", command=self._open, accelerator="Ctrl+O")
        fm.add_separator()
        fm.add_command(label="Save",        command=self._save, accelerator="Ctrl+S")
        fm.add_command(label="Save As...",  command=self._save_as)
        fm.add_separator()
        fm.add_command(label="Export JSON...", command=self._export_json)
        fm.add_separator()
        fm.add_command(label="Import sprite image...",
                       command=self._import_sprite)
        fm.add_command(label="Clear sprite image", command=self._clear_sprite)
        fm.add_separator()
        fm.add_command(label="Quit", command=self.root.destroy)
        m.add_cascade(label="File", menu=fm)

        vm = tk.Menu(m, tearoff=0)
        vm.add_command(label="Reset preview", command=self._reset_preview)
        vm.add_command(label="Burst now",     command=self._burst_now)
        m.add_cascade(label="View", menu=vm)

        self.root.config(menu=m)
        self.root.bind("<Control-n>", lambda e: self._new())
        self.root.bind("<Control-o>", lambda e: self._open())
        self.root.bind("<Control-s>", lambda e: self._save())

    # -- Layout ------------------------------------------------------------

    def _build_layout(self):
        outer = ttk.Frame(self.root)
        outer.pack(fill=tk.BOTH, expand=True)

        # Left: notebook with parameter tabs.
        left = ttk.Frame(outer, padding=4)
        left.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        self.nb = ttk.Notebook(left)
        self.nb.pack(fill=tk.BOTH, expand=True)
        self._build_emission_tab()
        self._build_physics_tab()
        self._build_appearance_tab()
        self._build_color_tab()
        self._build_size_tab()
        self._build_flags_tab()

        # Right: preview.
        right = ttk.Frame(outer, padding=4)
        right.pack(side=tk.RIGHT, fill=tk.BOTH)

        ttk.Label(right, text="Live preview (side view, Y up)",
                  font=("TkDefaultFont", 10, "bold")).pack(anchor="w")
        self.canvas = tk.Canvas(right, width=PREVIEW_W, height=PREVIEW_H,
                                background="#101820", highlightthickness=0)
        self.canvas.pack()

        info = ttk.Frame(right)
        info.pack(fill=tk.X, pady=(4, 0))
        self.alive_lbl = ttk.Label(info, text="Alive: 0")
        self.alive_lbl.pack(side=tk.LEFT)
        ttk.Button(info, text="Burst", command=self._burst_now).pack(side=tk.RIGHT)
        ttk.Button(info, text="Reset", command=self._reset_preview).pack(side=tk.RIGHT, padx=4)

    # -- Reusable widget helpers ------------------------------------------

    def _row_label(self, parent, text):
        ttk.Label(parent, text=text).grid(sticky="w", padx=2, pady=2)

    def _add_float(self, parent, row, label, key, sub=None, lo=-100.0, hi=100.0,
                   resolution=0.01):
        ttk.Label(parent, text=label).grid(row=row, column=0, sticky="w", padx=4, pady=2)
        v = FloatVar(value=self._get(key, sub))
        entry = ttk.Entry(parent, textvariable=v, width=10)
        entry.grid(row=row, column=1, sticky="we", padx=4, pady=2)
        sc = tk.Scale(parent, orient="horizontal", from_=lo, to=hi,
                      resolution=resolution, variable=v, showvalue=False, length=180)
        sc.grid(row=row, column=2, sticky="we", padx=4, pady=2)
        def cb(*_):
            try:    fv = float(v.get())
            except: return
            self._set(key, sub, fv)
        v.trace_add("write", cb)
        return v

    def _add_int(self, parent, row, label, key, sub=None, lo=0, hi=1000):
        ttk.Label(parent, text=label).grid(row=row, column=0, sticky="w", padx=4, pady=2)
        v = tk.IntVar(value=int(self._get(key, sub)))
        entry = ttk.Entry(parent, textvariable=v, width=10)
        entry.grid(row=row, column=1, sticky="we", padx=4, pady=2)
        sc = tk.Scale(parent, orient="horizontal", from_=lo, to=hi,
                      resolution=1, variable=v, showvalue=False, length=180)
        sc.grid(row=row, column=2, sticky="we", padx=4, pady=2)
        def cb(*_):
            try: iv = int(v.get())
            except: return
            self._set(key, sub, iv)
        v.trace_add("write", cb)
        return v

    def _add_check(self, parent, row, label, flag_key):
        v = tk.BooleanVar(value=bool(self.effect["flags"].get(flag_key, False)))
        ttk.Checkbutton(parent, text=label, variable=v).grid(row=row, column=0,
                            columnspan=3, sticky="w", padx=4, pady=2)
        def cb(*_):
            self.effect["flags"][flag_key] = bool(v.get())
            self._sim_reset()
            self._mark_dirty()
        v.trace_add("write", cb)
        return v

    def _get(self, key, sub):
        v = self.effect[key]
        if sub is not None:
            return v[sub]
        return v

    def _set(self, key, sub, value):
        if sub is not None:
            self.effect[key][sub] = value
        else:
            self.effect[key] = value
        # Rebuild the sim pool if the cap changed.
        if key == "max_particles":
            self._sim_reset()
        self._mark_dirty()

    # -- Tabs --------------------------------------------------------------

    def _build_emission_tab(self):
        f = ttk.Frame(self.nb, padding=8)
        self.nb.add(f, text="Emission")
        f.grid_columnconfigure(2, weight=1)
        r = 0
        self._add_int  (f, r, "Max particles", "max_particles", lo=1, hi=512); r += 1
        self._add_float(f, r, "Emit rate /s",  "emit_rate", lo=0.0, hi=120.0); r += 1
        self._add_int  (f, r, "Burst count",   "burst_count", lo=0, hi=512); r += 1
        self._add_int  (f, r, "Cone spread",   "cone_spread", lo=0, hi=511); r += 1
        self._add_int  (f, r, "Life min (fr)", "life_min_frames", lo=1, hi=600); r += 1
        self._add_int  (f, r, "Life max (fr)", "life_max_frames", lo=1, hi=600); r += 1
        self._add_float(f, r, "Speed min",     "speed_min", lo=-1.0, hi=2.0); r += 1
        self._add_float(f, r, "Speed max",     "speed_max", lo=-1.0, hi=2.0); r += 1
        ttk.Separator(f).grid(row=r, columnspan=3, sticky="we", pady=4); r += 1
        ttk.Label(f, text="Initial direction").grid(row=r, columnspan=3, sticky="w", padx=4); r += 1
        self._add_float(f, r, "dir.x", "initial_dir", sub=0, lo=-1.0, hi=1.0); r += 1
        self._add_float(f, r, "dir.y", "initial_dir", sub=1, lo=-1.0, hi=1.0); r += 1
        self._add_float(f, r, "dir.z", "initial_dir", sub=2, lo=-1.0, hi=1.0); r += 1
        ttk.Label(f, text="Spawn box (offset from emitter origin)").grid(
            row=r, columnspan=3, sticky="w", padx=4); r += 1
        self._add_float(f, r, "pos_min.x", "pos_min", sub=0, lo=-5.0, hi=5.0); r += 1
        self._add_float(f, r, "pos_min.y", "pos_min", sub=1, lo=-5.0, hi=5.0); r += 1
        self._add_float(f, r, "pos_min.z", "pos_min", sub=2, lo=-5.0, hi=5.0); r += 1
        self._add_float(f, r, "pos_max.x", "pos_max", sub=0, lo=-5.0, hi=5.0); r += 1
        self._add_float(f, r, "pos_max.y", "pos_max", sub=1, lo=-5.0, hi=5.0); r += 1
        self._add_float(f, r, "pos_max.z", "pos_max", sub=2, lo=-5.0, hi=5.0); r += 1

    def _build_physics_tab(self):
        f = ttk.Frame(self.nb, padding=8)
        self.nb.add(f, text="Physics")
        f.grid_columnconfigure(2, weight=1)
        r = 0
        ttk.Label(f, text="Gravity (per frame)").grid(row=r, columnspan=3, sticky="w", padx=4); r += 1
        self._add_float(f, r, "gravity.x", "gravity", sub=0, lo=-0.2, hi=0.2, resolution=0.001); r += 1
        self._add_float(f, r, "gravity.y", "gravity", sub=1, lo=-0.2, hi=0.2, resolution=0.001); r += 1
        self._add_float(f, r, "gravity.z", "gravity", sub=2, lo=-0.2, hi=0.2, resolution=0.001); r += 1
        self._add_float(f, r, "Drag",      "drag", lo=0.0, hi=0.5, resolution=0.001); r += 1

    def _build_appearance_tab(self):
        f = ttk.Frame(self.nb, padding=8)
        self.nb.add(f, text="Appearance")
        f.grid_columnconfigure(2, weight=1)
        r = 0
        self._add_float(f, r, "Base size",     "base_size", lo=0.05, hi=4.0); r += 1
        self._add_int  (f, r, "Base rotation", "base_rotation", lo=0, hi=511); r += 1
        self._add_int  (f, r, "Ang vel min",   "ang_vel_min", lo=-32, hi=32); r += 1
        self._add_int  (f, r, "Ang vel max",   "ang_vel_max", lo=-32, hi=32); r += 1
        ttk.Separator(f).grid(row=r, columnspan=3, sticky="we", pady=4); r += 1
        self._add_int  (f, r, "Sheet cols",    "sheet_cols", lo=1, hi=16); r += 1
        self._add_int  (f, r, "Sheet rows",    "sheet_rows", lo=1, hi=16); r += 1
        self._add_int  (f, r, "Sheet fps",     "sheet_fps", lo=0, hi=60); r += 1
        ttk.Separator(f).grid(row=r, columnspan=3, sticky="we", pady=4); r += 1
        ttk.Label(f, text="Material name (NEA_MaterialFindByName):").grid(
            row=r, columnspan=3, sticky="w", padx=4); r += 1
        self._tex_var = tk.StringVar(value=self.effect.get("mat_name", ""))
        e = ttk.Entry(f, textvariable=self._tex_var, width=44)
        e.grid(row=r, column=0, columnspan=3, sticky="we", padx=4, pady=2); r += 1
        def cb(*_):
            self.effect["mat_name"] = self._tex_var.get()[:31]
            self._mark_dirty()
        self._tex_var.trace_add("write", cb)

    def _build_color_tab(self):
        f = ttk.Frame(self.nb, padding=8)
        self.nb.add(f, text="Color over life")

        ttk.Label(f, text="Color and alpha over a particle's life. The curve is "
                          "alpha; the band behind it is the color, faded by "
                          "that alpha the way the DS composites it.",
                  wraplength=480, justify="left").pack(anchor="w")

        self.color_graph = LifeGraph(f, self, "color")
        self.color_graph.pack(fill=tk.BOTH, expand=True, pady=(6, 2))

        ttk.Label(f, text=GRAPH_HINT_TEXT, foreground=GRAPH_HINT,
                  wraplength=480, justify="left").pack(anchor="w")

        self._build_key_row(f, self.color_graph, "Alpha", with_color=True)

    def _build_size_tab(self):
        f = ttk.Frame(self.nb, padding=8)
        self.nb.add(f, text="Size over life")

        ttk.Label(f, text="Size in world units over a particle's life.",
                  justify="left").pack(anchor="w")

        self.size_graph = LifeGraph(f, self, "size")
        self.size_graph.pack(fill=tk.BOTH, expand=True, pady=(6, 2))

        ttk.Label(f, text=GRAPH_HINT_TEXT, foreground=GRAPH_HINT,
                  wraplength=480, justify="left").pack(anchor="w")

        self._build_key_row(f, self.size_graph, "Size")

    def _build_key_row(self, parent, graph, value_label, with_color=False):
        """The numeric editor under a graph, matching the animmat editor's."""
        row = ttk.Frame(parent)
        row.pack(fill=tk.X, pady=6)

        ttk.Label(row, text="t ").pack(side=tk.LEFT)
        graph.t_var = tk.StringVar()
        e1 = ttk.Entry(row, textvariable=graph.t_var, width=6)
        e1.pack(side=tk.LEFT)
        e1.bind("<Return>", lambda ev: self._apply_key_row(graph))

        ttk.Label(row, text=f"  {value_label} ").pack(side=tk.LEFT)
        graph.v_var = tk.StringVar()
        e2 = ttk.Entry(row, textvariable=graph.v_var, width=10)
        e2.pack(side=tk.LEFT)
        e2.bind("<Return>", lambda ev: self._apply_key_row(graph))

        ttk.Button(row, text="Set",
                   command=lambda: self._apply_key_row(graph)).pack(
            side=tk.LEFT, padx=4)

        if with_color:
            graph.color_btn = ttk.Button(
                row, text="Pick color...",
                command=lambda: self._pick_key_color(graph))
            graph.color_btn.pack(side=tk.LEFT)
        else:
            graph.color_btn = None

        self.on_graph_select(graph)

    def on_graph_select(self, graph):
        """Mirror the graph's selected key into its numeric boxes."""
        if not hasattr(graph, "t_var"):
            return

        keys = graph.keys
        if graph.sel is None or not (0 <= graph.sel < len(keys)):
            graph.t_var.set("")
            graph.v_var.set("")
            if graph.color_btn is not None:
                graph.color_btn.configure(state="disabled")
            return

        key = keys[graph.sel]
        graph.t_var.set(str(key[0]))
        graph.v_var.set(f"{key[4]}" if graph.mode == "color"
                        else f"{key[1]:.3f}")
        if graph.color_btn is not None:
            graph.color_btn.configure(state="normal")

    def _apply_key_row(self, graph):
        keys = list(graph.keys)
        if graph.sel is None or not (0 <= graph.sel < len(keys)):
            return

        try:
            t = max(0, min(1000, int(graph.t_var.get())))
            value = float(graph.v_var.get())
        except ValueError:
            return

        if any(j != graph.sel and k[0] == t for j, k in enumerate(keys)):
            messagebox.showwarning(
                "That time already has a key",
                f"There is already a key at t={t}.\n\nTwo keys at one time "
                "leave a zero-length span, which the sampler skips, so one of "
                "them would do nothing.")
            return

        edited = graph._with_y(keys[graph.sel], t, value)
        keys[graph.sel] = edited
        keys.sort(key=lambda k: k[0])

        graph._set_keys(keys)
        graph.sel = keys.index(edited)

        self.on_graph_select(graph)
        self._mark_dirty()
        graph.redraw()

    def _pick_key_color(self, graph):
        keys = list(graph.keys)
        if graph.sel is None or not (0 <= graph.sel < len(keys)):
            return

        t, r, g, b, a = keys[graph.sel]
        rgb, _ = colorchooser.askcolor(initialcolor=(r, g, b), parent=self.root)
        if rgb is None:
            return

        keys[graph.sel] = (t, int(rgb[0]), int(rgb[1]), int(rgb[2]), a)
        graph._set_keys(keys)

        self._mark_dirty()
        graph.redraw()

    def _build_flags_tab(self):
        f = ttk.Frame(self.nb, padding=8)
        self.nb.add(f, text="Flags")
        r = 0
        self._add_check(f, r, "Continuous emission", "continuous");   r += 1
        self._add_check(f, r, "Axis-aligned (no billboard)", "axis_aligned"); r += 1
        self._add_check(f, r, "Additive blending",  "additive");     r += 1
        self._add_check(f, r, "Sprite-sheet animation", "spritesheet"); r += 1
        self._add_check(f, r, "Velocity-stretched (reserved)", "stretch"); r += 1

    # -- File I/O ----------------------------------------------------------

    def _new(self):
        if not self._confirm_discard(): return
        self.effect = npe_format.default_effect()
        self.path = None
        self._sim_reset()
        self._refresh_all()
        self._dirty = False
        self._update_title()

    def _open(self):
        if not self._confirm_discard(): return
        p = filedialog.askopenfilename(filetypes=[("NPE", "*.npe"), ("All", "*.*")])
        if not p: return
        self._open_file(p)

    def _open_file(self, p):
        try:
            with open(p, "rb") as f:
                data = f.read()
            self.effect = npe_format.decode(data)
            self.path = p
            self._load_sidecar()
            self._sim_reset()
            self._refresh_all()
            self._dirty = False
            self._update_title()
        except Exception as ex:
            messagebox.showerror("Open failed", str(ex))

    def _save(self):
        if not self.path:
            return self._save_as()
        try:
            with open(self.path, "wb") as f:
                f.write(npe_format.encode(self.effect))
            self._save_sidecar()
            self._dirty = False
            self._update_title()
        except Exception as ex:
            messagebox.showerror("Save failed", str(ex))

    def _save_as(self):
        p = filedialog.asksaveasfilename(defaultextension=".npe",
                                         filetypes=[("NPE", "*.npe")])
        if not p: return
        self.path = p
        self._save()

    def _export_json(self):
        p = filedialog.asksaveasfilename(defaultextension=".json",
                                         filetypes=[("JSON", "*.json")])
        if not p: return
        try:
            with open(p, "w") as f:
                json.dump(self.effect, f, indent=2)
        except Exception as ex:
            messagebox.showerror("Export failed", str(ex))

    def _confirm_discard(self):
        if not self._dirty: return True
        a = messagebox.askyesnocancel("Unsaved changes",
                                      "Save changes before continuing?")
        if a is None: return False
        if a:         self._save()
        return True

    # -- Preview -----------------------------------------------------------

    def _sim_reset(self):
        self.sim = Simulator(self.effect)

    def _reset_preview(self):
        self._sim_reset()

    def _burst_now(self):
        self.sim.burst(int(self.effect["burst_count"]))

    def _tick(self):
        self.sim.set_effect(self.effect) if False else None  # cheap: shared dict
        self.sim.eff = self.effect
        self.sim.step()
        self._draw_preview()
        self.root.after(33, self._tick)  # ~30 fps

    # -- sprite ------------------------------------------------------------

    ROT_BUCKETS = 16
    SIZE_BUCKETS = 12

    def _load_sidecar(self):
        """Pick up the sprite recorded beside this effect, if any."""
        data = SC.load(self.path)
        self._set_sprite(SC.resolve_path(self.path, data.get("image")),
                         stored=data.get("image"), persist=False)

    def _save_sidecar(self):
        if self._sprite_path:
            SC.save(self.path, {"image": self._sprite_path})
        else:
            SC.delete(self.path)

    def _set_sprite(self, full_path, stored=None, persist=True):
        self._sprite_cache = {}
        self._sprite = None
        self._sprite_path = stored

        if full_path:
            try:
                self._sprite = Image.open(full_path).convert("RGBA")
            except (OSError, ValueError):
                self._sprite = None

        if persist:
            self._save_sidecar()

    def _import_sprite(self):
        p = filedialog.askopenfilename(
            title="Sprite image for this effect",
            filetypes=[("Images", "*.png *.gif *.jpg *.jpeg *.bmp"),
                       ("All files", "*")])
        if not p:
            return

        try:
            Image.open(p).close()
        except (OSError, ValueError) as e:
            messagebox.showerror("Cannot read image", str(e))
            return

        self._set_sprite(p, stored=SC.store_path(self.path, p))

    def _clear_sprite(self):
        self._set_sprite(None)

    def _sheet_cell(self, particle):
        """Which sprite-sheet cell a particle is on, or None if not a sheet."""
        eff = self.effect
        if not eff.get("spritesheet"):
            return None

        cols = max(1, int(eff.get("sheet_cols", 1)))
        rows = max(1, int(eff.get("sheet_rows", 1)))
        count = cols * rows
        if count <= 1:
            return None

        fps = int(eff.get("sheet_fps", 0))
        if fps <= 0:
            return 0

        # age is in frames at 60 Hz, as the runtime counts it.
        return int(particle.age * fps / 60.0) % count

    def _sprite_for(self, cell, rot, diameter):
        """A rotated, scaled copy of the sprite, cached."""
        size_bucket = max(1, min(256, int(diameter)))
        size_bucket -= size_bucket % max(1, 256 // self.SIZE_BUCKETS)
        size_bucket = max(2, size_bucket)

        rot_bucket = int(rot) % 512 * self.ROT_BUCKETS // 512

        key = (cell, rot_bucket, size_bucket)
        hit = self._sprite_cache.get(key)
        if hit is not None:
            return hit

        src = self._sprite
        if cell is not None:
            cols = max(1, int(self.effect.get("sheet_cols", 1)))
            rows = max(1, int(self.effect.get("sheet_rows", 1)))
            cw, ch = src.width // cols, src.height // rows
            cx, cy = (cell % cols) * cw, (cell // cols) * ch
            src = src.crop((cx, cy, cx + cw, cy + ch))

        img = src.resize((size_bucket, size_bucket), Image.BILINEAR)

        degrees = rot_bucket * 360.0 / self.ROT_BUCKETS
        if degrees:
            img = img.rotate(degrees, resample=Image.BILINEAR, expand=True)

        self._sprite_cache[key] = img
        return img

    def _draw_preview(self):
        c = self.canvas
        c.delete("all")
        cx = PREVIEW_W // 2
        cy = PREVIEW_H * 3 // 4  # emitter near the bottom
        sc = self._preview_scale

        if self._sprite is None:
            alive = self._draw_preview_shapes(c, cx, cy, sc)
        else:
            alive = self._draw_preview_sprites(c, cx, cy, sc)

        self.alive_lbl.config(text=f"Alive: {alive}")

        # Floor line and origin marker, over whatever was drawn.
        c.create_line(0, cy, PREVIEW_W, cy, fill="#283848")
        c.create_oval(cx - 3, cy - 3, cx + 3, cy + 3, outline="#5d7", width=1)

        if self._sprite is None and self._sprite_path:
            c.create_text(6, 6, anchor="nw", fill="#a66",
                          font=("TkFixedFont", 8),
                          text="sprite image missing: " + str(self._sprite_path))

    def _draw_preview_shapes(self, c, cx, cy, sc):
        """The no-sprite fallback: one oval per particle.

        Alpha is approximated by mixing toward the background, which is the best
        a solid shape can do and the reason importing a sprite is worth it.
        """
        alive = 0
        for p in self.sim.pool:
            if not p.alive:
                continue
            alive += 1
            x = cx + p.px * sc
            y = cy - p.py * sc
            rad = max(1.0, p.size * sc * 0.5)
            br, bg_, bb = 0x10, 0x18, 0x20
            pa_ = int(p.a)
            if self.effect.get("additive"):
                lum = (p.r * 77 + p.g * 151 + p.b * 28) >> 8
                pa_ = (pa_ * lum) // 255
            af = pa_ / 255.0
            cr = int(br * (1 - af) + p.r * af)
            cg = int(bg_ * (1 - af) + p.g * af)
            cb = int(bb * (1 - af) + p.b * af)
            color = f"#{cr:02x}{cg:02x}{cb:02x}"
            c.create_oval(x - rad, y - rad, x + rad, y + rad,
                          outline=color, fill=color)
        return alive

    def _draw_preview_sprites(self, c, cx, cy, sc):
        """Composite the real sprite for every particle, into one image.

        Compositing rather than drawing each particle separately is what makes
        alpha and additive blending real rather than approximated: additive in
        particular is the whole look of a spark or a flame and cannot be faked
        with a solid shape.
        """
        additive = bool(self.effect.get("additive"))

        field = Image.new("RGB", (PREVIEW_W, PREVIEW_H), (0x10, 0x18, 0x20))
        alive = 0

        for p in self.sim.pool:
            if not p.alive:
                continue
            alive += 1

            diameter = max(2.0, p.size * sc)
            sprite = self._sprite_for(self._sheet_cell(p), p.rot, diameter)

            # The DS has no additive blend, so the runtime approximates one by
            # weighting a particle's alpha with its brightness: dark particles
            # contribute almost nothing, bright ones dominate. Do exactly the
            # same here -- a preview that showed true additive would be showing
            # something the hardware cannot produce.
            p_a = int(p.a)
            if additive:
                lum = (p.r * 77 + p.g * 151 + p.b * 28) >> 8
                p_a = (p_a * lum) // 255

            # Tint by the particle colour, fade by its alpha.
            r, g, b, a = sprite.split()
            tinted = Image.merge("RGBA", (
                r.point(lambda v, k=p.r: (v * k) // 255),
                g.point(lambda v, k=p.g: (v * k) // 255),
                b.point(lambda v, k=p.b: (v * k) // 255),
                a.point(lambda v, k=p_a: (v * k) // 255)))

            x = int(cx + p.px * sc - tinted.width / 2)
            y = int(cy - p.py * sc - tinted.height / 2)

            field.paste(tinted, (x, y), tinted)

        self._preview_photo = ImageTk.PhotoImage(field)
        c.create_image(0, 0, anchor="nw", image=self._preview_photo)
        return alive


    # -- Misc --------------------------------------------------------------

    def _refresh_all(self):
        # The float/int widgets read self.effect directly via the trace
        # callbacks, but the tabs are re-built only on init -- so for now
        # rebuild the notebook contents from scratch.
        for tab_id in self.nb.tabs():
            self.nb.forget(tab_id)
        self._build_emission_tab()
        self._build_physics_tab()
        self._build_appearance_tab()
        self._build_color_tab()
        self._build_size_tab()
        self._build_flags_tab()

    def _mark_dirty(self):
        if not self._dirty:
            self._dirty = True
            self._update_title()

    def _update_title(self):
        name = os.path.basename(self.path) if self.path else "<unsaved>"
        star = " *" if self._dirty else ""
        self.root.title(f"NPE editor -- {name}{star}")

# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main():
    arg = sys.argv[1] if len(sys.argv) > 1 else None
    root = tk.Tk()
    NpeEditor(root, initial_path=arg)
    root.mainloop()

if __name__ == "__main__":
    main()
