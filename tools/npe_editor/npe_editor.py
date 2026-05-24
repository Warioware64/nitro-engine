#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
#
# Copyright (c) 2026 Warioware64
#
# Standalone NPE (Nitro Particle Entity) editor.
#
# Edit a particle effect with live preview, save as a `.npe` binary the
# NEAParticle runtime can load directly. Tkinter only -- ships with Python,
# no extra dependencies, matching the rest of the NEA Python tools.
#
# Usage:
#     python3 tools/npe_editor/npe_editor.py [path/to/file.npe]

"""NPE particle editor (tkinter).

Left panel: tabbed parameter controls.
Right panel: 2D side-view preview running the same simulation logic as the
  C runtime in NEAParticle.c, so what you see is what the DS will show.
"""

import json
import math
import os
import random
import sys
import tkinter as tk
from tkinter import colorchooser, filedialog, messagebox, ttk

# Allow running the script from anywhere by adding its dir to sys.path.
_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)
import npe_format  # noqa: E402

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
            theta = self._rng.uniform(0, math.pi)
            phi   = self._rng.uniform(0, 2 * math.pi)
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
        max_theta_rad = (cone / 511.0) * math.pi   # cone 0..511 maps to 0..pi
        theta = self._rng.uniform(0, max_theta_rad)
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
        p.life = max(1, self._rand_int(e["life_min_frames"], e["life_max_frames"]))
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
        ttk.Label(f, text="Keyframes (t = 0..1000):").pack(anchor="w")
        self.color_list = tk.Listbox(f, height=10)
        self.color_list.pack(fill=tk.BOTH, expand=True, pady=4)

        bar = ttk.Frame(f); bar.pack(fill=tk.X)
        ttk.Button(bar, text="Add",  command=self._add_color_key   ).pack(side=tk.LEFT)
        ttk.Button(bar, text="Edit", command=self._edit_color_key  ).pack(side=tk.LEFT, padx=4)
        ttk.Button(bar, text="Delete", command=self._delete_color_key).pack(side=tk.LEFT)

        self.color_strip = tk.Canvas(f, height=24, background="#000")
        self.color_strip.pack(fill=tk.X, pady=8)
        self._refresh_color_list()

    def _build_size_tab(self):
        f = ttk.Frame(self.nb, padding=8)
        self.nb.add(f, text="Size over life")
        ttk.Label(f, text="Keyframes (t = 0..1000, size in world units):").pack(anchor="w")
        self.size_list = tk.Listbox(f, height=10)
        self.size_list.pack(fill=tk.BOTH, expand=True, pady=4)

        bar = ttk.Frame(f); bar.pack(fill=tk.X)
        ttk.Button(bar, text="Add",  command=self._add_size_key   ).pack(side=tk.LEFT)
        ttk.Button(bar, text="Edit", command=self._edit_size_key  ).pack(side=tk.LEFT, padx=4)
        ttk.Button(bar, text="Delete", command=self._delete_size_key).pack(side=tk.LEFT)

        self.size_curve = tk.Canvas(f, height=120, background="#202830", highlightthickness=0)
        self.size_curve.pack(fill=tk.X, pady=8)
        self._refresh_size_list()

    def _build_flags_tab(self):
        f = ttk.Frame(self.nb, padding=8)
        self.nb.add(f, text="Flags")
        r = 0
        self._add_check(f, r, "Continuous emission", "continuous");   r += 1
        self._add_check(f, r, "Axis-aligned (no billboard)", "axis_aligned"); r += 1
        self._add_check(f, r, "Additive blending",  "additive");     r += 1
        self._add_check(f, r, "Sprite-sheet animation", "spritesheet"); r += 1
        self._add_check(f, r, "Velocity-stretched (reserved)", "stretch"); r += 1

    # -- Color keys --------------------------------------------------------

    def _refresh_color_list(self):
        self.color_list.delete(0, tk.END)
        for (t, r, g, b, a) in self.effect["color_keys"]:
            self.color_list.insert(tk.END, f"t={t:>4}   RGB=({r:3},{g:3},{b:3})   A={a:3}")
        # Strip preview.
        self.color_strip.delete("all")
        w = max(1, self.color_strip.winfo_width() or 360)
        h = 22
        for x in range(w):
            t = int(1000 * x / max(1, w - 1))
            r, g, b, a = Simulator._sample_color(self.effect["color_keys"], t)
            # Blend with dark background to show alpha
            br, bg_, bb = 16, 24, 32
            af = a / 255.0
            cr = int(br * (1 - af) + r * af)
            cg = int(bg_ * (1 - af) + g * af)
            cb = int(bb * (1 - af) + b * af)
            self.color_strip.create_line(x, 0, x, h, fill=f"#{cr:02x}{cg:02x}{cb:02x}")

    def _add_color_key(self):
        self.effect["color_keys"].append((500, 255, 255, 255, 255))
        self.effect["color_keys"].sort()
        self._refresh_color_list()
        self._mark_dirty()

    def _edit_color_key(self):
        sel = self.color_list.curselection()
        if not sel: return
        i = sel[0]
        t, r, g, b, a = self.effect["color_keys"][i]
        # Time prompt
        new_t = self._prompt_int("Time (0..1000)", t)
        if new_t is None: return
        # Color picker
        ((nr, ng, nb), _hex) = colorchooser.askcolor(initialcolor=(r, g, b),
                                                     parent=self.root) or ((None,)*3, None)
        if nr is None: return
        new_a = self._prompt_int("Alpha (0..255)", a)
        if new_a is None: return
        self.effect["color_keys"][i] = (max(0, min(1000, new_t)),
                                        int(nr), int(ng), int(nb),
                                        max(0, min(255, new_a)))
        self.effect["color_keys"].sort()
        self._refresh_color_list()
        self._mark_dirty()

    def _delete_color_key(self):
        sel = self.color_list.curselection()
        if not sel: return
        del self.effect["color_keys"][sel[0]]
        self._refresh_color_list()
        self._mark_dirty()

    # -- Size keys ---------------------------------------------------------

    def _refresh_size_list(self):
        self.size_list.delete(0, tk.END)
        for (t, sz) in self.effect["size_keys"]:
            self.size_list.insert(tk.END, f"t={t:>4}   size={sz:.3f}")
        # Curve preview.
        self.size_curve.delete("all")
        w = max(1, self.size_curve.winfo_width() or 360)
        h = 118
        max_s = max((s for _, s in self.effect["size_keys"]), default=1.0) or 1.0
        # Axis
        self.size_curve.create_line(0, h - 1, w, h - 1, fill="#555")
        pts = []
        for x in range(w):
            t = int(1000 * x / max(1, w - 1))
            s = Simulator._sample_size(self.effect["size_keys"], t, 0)
            y = h - 1 - int((s / max_s) * (h - 2))
            pts.extend([x, y])
        if len(pts) >= 4:
            self.size_curve.create_line(*pts, fill="#7fd")

    def _add_size_key(self):
        self.effect["size_keys"].append((500, 1.0))
        self.effect["size_keys"].sort()
        self._refresh_size_list()
        self._mark_dirty()

    def _edit_size_key(self):
        sel = self.size_list.curselection()
        if not sel: return
        i = sel[0]
        t, s = self.effect["size_keys"][i]
        new_t = self._prompt_int("Time (0..1000)", t)
        if new_t is None: return
        new_s = self._prompt_float("Size (world units)", s)
        if new_s is None: return
        self.effect["size_keys"][i] = (max(0, min(1000, new_t)),
                                       max(0.0, float(new_s)))
        self.effect["size_keys"].sort()
        self._refresh_size_list()
        self._mark_dirty()

    def _delete_size_key(self):
        sel = self.size_list.curselection()
        if not sel: return
        del self.effect["size_keys"][sel[0]]
        self._refresh_size_list()
        self._mark_dirty()

    # -- Small prompt dialogs ---------------------------------------------

    def _prompt_int(self, label, initial):
        return self._prompt_value(label, initial, parser=lambda s: int(s))

    def _prompt_float(self, label, initial):
        return self._prompt_value(label, initial, parser=lambda s: float(s))

    def _prompt_value(self, label, initial, parser):
        dlg = tk.Toplevel(self.root)
        dlg.title("Edit")
        dlg.transient(self.root); dlg.grab_set()
        ttk.Label(dlg, text=label).pack(padx=8, pady=(8, 4))
        var = tk.StringVar(value=str(initial))
        e = ttk.Entry(dlg, textvariable=var); e.pack(padx=8, pady=4); e.focus_set()
        result = {"value": None}
        def ok():
            try:
                result["value"] = parser(var.get())
                dlg.destroy()
            except Exception:
                messagebox.showerror("Invalid", "Could not parse value")
        ttk.Button(dlg, text="OK",     command=ok).pack(side=tk.LEFT, padx=8, pady=8)
        ttk.Button(dlg, text="Cancel", command=dlg.destroy).pack(side=tk.RIGHT, padx=8, pady=8)
        dlg.bind("<Return>", lambda e: ok())
        dlg.wait_window()
        return result["value"]

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

    def _draw_preview(self):
        c = self.canvas
        c.delete("all")
        cx = PREVIEW_W // 2
        cy = PREVIEW_H * 3 // 4  # emitter near the bottom
        # Floor line.
        c.create_line(0, cy, PREVIEW_W, cy, fill="#283848")
        # Origin marker.
        c.create_oval(cx - 3, cy - 3, cx + 3, cy + 3, outline="#5d7", width=1)
        # Particles (side view: X -> screen X, Y -> -screen Y).
        sc = self._preview_scale
        alive = 0
        for p in self.sim.pool:
            if not p.alive: continue
            alive += 1
            x = cx + p.px * sc
            y = cy - p.py * sc
            rad = max(1.0, p.size * sc * 0.5)
            # Mix particle color with background according to alpha.
            br, bg_, bb = 0x10, 0x18, 0x20
            af = p.a / 255.0
            cr = int(br * (1 - af) + p.r * af)
            cg = int(bg_ * (1 - af) + p.g * af)
            cb = int(bb * (1 - af) + p.b * af)
            color = f"#{cr:02x}{cg:02x}{cb:02x}"
            c.create_oval(x - rad, y - rad, x + rad, y + rad,
                          outline=color, fill=color)
        self.alive_lbl.config(text=f"Alive: {alive}")

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
