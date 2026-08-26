#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
#
# Copyright (c) 2026 Warioware64
#
# Standalone AnimMat (.neaanimmat) editor.
#
# Edit a material animation with live preview, save as a binary the NEAAnimMat
# runtime loads directly. Tkinter plus Pillow: the preview has to rotate, scale,
# tint and alpha-composite a real texture, none of which Tk's own PhotoImage can
# do, and tools/img2ds already depends on Pillow.
#
# Usage:
#     python3 tools/animmat_editor/animmat_editor.py [path/to/file.neaanimmat]

"""AnimMat material animation editor (tkinter).

Left panel:   targets (materials this animation drives) and their tracks.
Centre:       a timeline for the selected track. Tracks that interpolate get a
              curve with draggable keyframes; tracks the hardware can only step
              through get a strip of labelled segments, because a bitmask or a
              culling mode plotted on a numeric axis is not readable.
Right panel:  a preview quad evaluated with animmat_format.evaluate_target(),
              which is verified frame-for-frame against the C runtime by
              tests/animmat_eval. What the preview shows is what the DS does.

An animation names the materials it drives but does not contain them, so the
editor cannot know what the artwork looks like. Point it at the images with
File -> Import image, and it records them in a sidecar beside the animation --
see editor_sidecar.py. The binary itself is never touched by any of that.
"""

import math
import os
import sys
import tkinter as tk
from tkinter import colorchooser, filedialog, messagebox, ttk

try:
    from PIL import Image, ImageDraw, ImageTk
except ImportError:
    sys.exit("This editor needs Pillow for its preview: pip install Pillow")

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)
import animmat_format as A  # noqa: E402
import editor_sidecar as SC  # noqa: E402


# ---------------------------------------------------------------------------
# Track presentation
# ---------------------------------------------------------------------------

# For each track type: how a raw value is shown, and how it is parsed back.
# Keeping this in one table is what stops the timeline, the value box and the
# preview from disagreeing about what a number means.

def _fmt_f32(v):
    return f"{A.from_f32(v):.3f}"


def _parse_f32(text):
    return A.f32(float(text))


def _fmt_hexcolor(v):
    r, g, b = v & 0x1F, (v >> 5) & 0x1F, (v >> 10) & 0x1F
    return f"#{r * 255 // 31:02x}{g * 255 // 31:02x}{b * 255 // 31:02x}"


def _parse_int(text):
    text = text.strip()
    return int(text, 16) if text.lower().startswith("0x") else int(text)


TRACK_UI = {
    A.ALPHA:             ("int",   (0, 31),   str,      _parse_int),
    A.LIGHTS:            ("int",   (0, 15),   str,      _parse_int),
    A.CULLING:           ("cull",  (0, 192),  str,      _parse_int),
    A.COLOR:             ("color", (0, 0x7FFF), lambda v: f"0x{v:04X}", _parse_int),
    A.DIFFUSE_AMBIENT:   ("hex32", None,      lambda v: f"0x{v:08X}", _parse_int),
    A.SPECULAR_EMISSION: ("hex32", None,      lambda v: f"0x{v:08X}", _parse_int),
    A.MATERIAL_SWAP:     ("int",   (0, 255),  str,      _parse_int),
    A.POLYID:            ("int",   (0, 63),   str,      _parse_int),
    A.TEX_SCROLL_X:      ("f32",   None,      _fmt_f32, _parse_f32),
    A.TEX_SCROLL_Y:      ("f32",   None,      _fmt_f32, _parse_f32),
    A.TEX_ROTATE:        ("int",   (0, 511),  str,      _parse_int),
    A.TEX_SCALE_X:       ("f32",   None,      _fmt_f32, _parse_f32),
    A.TEX_SCALE_Y:       ("f32",   None,      _fmt_f32, _parse_f32),
    A.TEXPAL_SWAP:       ("hex16", None,      lambda v: f"0x{v:04X}", _parse_int),
}

CULL_NAMES = {0: "none", 64: "front", 128: "back", 192: "all"}

# Tracks the colour picker can author. The two packed ones hold a pair of RGB15
# values in one 32 bit word, so they need two swatches rather than one.
COLOR_TRACKS = (A.COLOR, A.DIFFUSE_AMBIENT, A.SPECULAR_EMISSION)
PACKED_COLOR_TRACKS = (A.DIFFUSE_AMBIENT, A.SPECULAR_EMISSION)

# What the two halves of each packed track are called, low half first.
PACKED_HALVES = {
    A.DIFFUSE_AMBIENT:   ("Diffuse", "Ambient"),
    A.SPECULAR_EMISSION: ("Specular", "Emission"),
}


def _safe_int(var, default=0):
    try:
        return int(var.get())
    except (ValueError, tk.TclError):
        return default


def rgb15_to_hex(v):
    r, g, b = v & 0x1F, (v >> 5) & 0x1F, (v >> 10) & 0x1F
    return f"#{r * 255 // 31:02x}{g * 255 // 31:02x}{b * 255 // 31:02x}"


def hex_to_rgb15(rgb):
    r, g, b = (int(c) * 31 // 255 for c in rgb)
    return r | (g << 5) | (b << 10)


def lights_label(v):
    on = [f"L{i}" for i in range(4) if v & (1 << i)]
    return " ".join(on) if on else "none"


def texpal_label(v):
    tex = (v >> 8) & 0xFF
    pal = v & 0xFF
    t = "-" if tex == 0xFF else str(tex)
    p = "-" if pal == 0xFF else str(pal)
    return f"tex {t}  pal {p}"


# How a step-only track's value reads as text in its segment strip.
SEGMENT_LABEL = {
    A.CULLING:       lambda v: CULL_NAMES.get(v, str(v)),
    A.LIGHTS:        lights_label,
    A.MATERIAL_SWAP: lambda v: f"material {v}",
    A.TEXPAL_SWAP:   texpal_label,
}


def value_range(track_type, values):
    """Vertical span for the timeline plot of this track."""
    kind, rng, _, _ = TRACK_UI[track_type]
    if rng is not None:
        return rng
    if not values:
        return (0, 1)
    lo, hi = min(values), max(values)
    if lo == hi:
        return (lo - 1, hi + 1)
    pad = (hi - lo) * 0.1
    return (lo - pad, hi + pad)


# ---------------------------------------------------------------------------
# Editor
# ---------------------------------------------------------------------------

class Editor(tk.Tk):

    def __init__(self, path=None):
        super().__init__()
        self.title("NEA AnimMat editor")
        self.geometry("1180x720")

        self.anim = A.new_animation(60)
        self.path = None
        self.target_idx = 0
        self.track_idx = None
        self.frame = 0
        self.playing = False

        # Selection and drag state. These used to be created on first use and
        # read back through getattr(), which meant three places had to guess a
        # default for state the editor always has.
        self._sel_key = None
        self._drag_key = None
        self._drag_range = None

        self._dirty = False
        self._size_job = None

        # Sidecar state: the image table, and which image each target uses.
        self._images = []        # stored paths, as written to the sidecar
        self._photos = {}        # index -> loaded PIL image, or None if gone
        self._target_image = {}  # target name -> index into _images
        self._tint_emission = True
        self._all_targets = False
        self._preview_photo = None  # kept alive; Tk drops unreferenced images

        self._build_menu()
        self._build_layout()

        if path:
            self._open_path(path)
        else:
            self._refresh_all()

        self.protocol("WM_DELETE_WINDOW", self._on_close)
        self.after(33, self._tick)

    # -- construction -------------------------------------------------------

    def _build_menu(self):
        bar = tk.Menu(self)

        filemenu = tk.Menu(bar, tearoff=0)
        filemenu.add_command(label="New", command=self._new, accelerator="Ctrl+N")
        filemenu.add_command(label="Open...", command=self._open, accelerator="Ctrl+O")
        filemenu.add_command(label="Save", command=self._save, accelerator="Ctrl+S")
        filemenu.add_command(label="Save As...", command=self._save_as)
        filemenu.add_command(label="Save As version 1...",
                             command=self._save_as_v1)
        filemenu.add_separator()
        filemenu.add_command(label="Import image...", command=self._import_image)
        filemenu.add_command(label="Manage images...", command=self._manage_images)
        filemenu.add_separator()
        filemenu.add_command(label="Quit", command=self._on_close)
        bar.add_cascade(label="File", menu=filemenu)

        editmenu = tk.Menu(bar, tearoff=0)
        editmenu.add_command(label="Optimize storage modes",
                             command=self._optimize)
        bar.add_cascade(label="Edit", menu=editmenu)

        self.config(menu=bar)
        self.bind("<Control-n>", lambda e: self._new())
        self.bind("<Control-o>", lambda e: self._open())
        self.bind("<Control-s>", lambda e: self._save())

    def _build_layout(self):
        root = ttk.Frame(self, padding=6)
        root.pack(fill="both", expand=True)

        left = ttk.Frame(root, width=260)
        left.pack(side="left", fill="y", padx=(0, 6))
        left.pack_propagate(False)

        right = ttk.Frame(root, width=250)
        right.pack(side="right", fill="y", padx=(6, 0))
        right.pack_propagate(False)

        centre = ttk.Frame(root)
        centre.pack(side="left", fill="both", expand=True)

        self._build_left(left)
        self._build_centre(centre)
        self._build_right(right)

    def _build_left(self, p):
        ttk.Label(p, text="Targets (materials)",
                  font=("TkDefaultFont", 9, "bold")).pack(anchor="w")
        ttk.Label(p, text="A target's name is matched against the\n"
                          "submesh material names of a model.",
                  foreground="#666", justify="left").pack(anchor="w", pady=(0, 2))

        self.target_list = tk.Listbox(p, height=6, exportselection=False)
        self.target_list.pack(fill="x")
        self.target_list.bind("<<ListboxSelect>>", self._on_target_select)

        row = ttk.Frame(p)
        row.pack(fill="x", pady=2)
        ttk.Button(row, text="Add", width=6, command=self._add_target).pack(side="left")
        ttk.Button(row, text="Rename", width=8, command=self._rename_target).pack(side="left")
        ttk.Button(row, text="Remove", width=8, command=self._remove_target).pack(side="left")

        ttk.Separator(p).pack(fill="x", pady=6)

        ttk.Label(p, text="Tracks", font=("TkDefaultFont", 9, "bold")).pack(anchor="w")
        self.track_list = tk.Listbox(p, height=12, exportselection=False,
                                     font=("TkFixedFont", 9))
        self.track_list.pack(fill="both", expand=True)
        self.track_list.bind("<<ListboxSelect>>", self._on_track_select)

        row = ttk.Frame(p)
        row.pack(fill="x", pady=2)
        self.new_track_type = tk.StringVar(value=A.TRACK_NAMES[A.ALPHA])
        ttk.OptionMenu(row, self.new_track_type, A.TRACK_NAMES[A.ALPHA],
                       *[A.TRACK_NAMES[t] for t in sorted(A.TRACK_NAMES)]
                       ).pack(side="left", fill="x", expand=True)
        ttk.Button(row, text="Add", width=5,
                   command=self._add_track).pack(side="left")
        ttk.Button(row, text="Del", width=5,
                   command=self._remove_track).pack(side="left")

        ttk.Separator(p).pack(fill="x", pady=6)
        self.size_label = ttk.Label(p, text="", foreground="#444",
                                    font=("TkFixedFont", 8), justify="left")
        self.size_label.pack(anchor="w")

    def _build_centre(self, p):
        head = ttk.Frame(p)
        head.pack(fill="x")

        self.track_title = ttk.Label(head, text="No track selected",
                                     font=("TkDefaultFont", 10, "bold"))
        self.track_title.pack(side="left")

        ttk.Label(head, text="  storage ").pack(side="left")
        self.storage_var = tk.StringVar(value="keys")
        self.storage_menu = ttk.OptionMenu(
            head, self.storage_var, "keys", "keys", "const", "baked",
            command=lambda *_: self._set_storage())
        self.storage_menu.pack(side="left")

        ttk.Label(head, text="  interp ").pack(side="left")
        self.interp_var = tk.StringVar(value="linear")
        self.interp_menu = ttk.OptionMenu(
            head, self.interp_var, "linear", "step", "linear",
            command=lambda *_: self._set_interp())
        self.interp_menu.pack(side="left")

        self.canvas = tk.Canvas(p, bg="#1b1b1f", highlightthickness=0)
        self.canvas.pack(fill="both", expand=True, pady=6)
        self.canvas.bind("<Configure>", lambda e: self._draw_timeline())
        self.canvas.bind("<Button-1>", self._on_canvas_click)
        self.canvas.bind("<B1-Motion>", self._on_canvas_drag)
        self.canvas.bind("<ButtonRelease-1>", lambda e: self._end_drag())
        self.canvas.bind("<Double-Button-1>", self._on_canvas_double)
        self.canvas.bind("<Button-3>", self._on_canvas_right)

        ttk.Label(p, text="Click a key to select, drag to move, double-click "
                          "empty space to add, right-click a key to delete.",
                  foreground="#666").pack(anchor="w")

        row = ttk.Frame(p)
        row.pack(fill="x", pady=4)
        ttk.Label(row, text="Key frame ").pack(side="left")
        self.key_frame_var = tk.StringVar()
        e = ttk.Entry(row, textvariable=self.key_frame_var, width=7)
        e.pack(side="left")
        e.bind("<Return>", lambda ev: self._apply_key_edit())

        ttk.Label(row, text="  value ").pack(side="left")
        self.key_value_var = tk.StringVar()
        e2 = ttk.Entry(row, textvariable=self.key_value_var, width=14)
        e2.pack(side="left")
        e2.bind("<Return>", lambda ev: self._apply_key_edit())
        ttk.Button(row, text="Set", command=self._apply_key_edit).pack(side="left", padx=4)
        self.color_btn = ttk.Button(row, text="Pick color...",
                                    command=self._pick_color)
        self.color_btn.pack(side="left")

        # Enum tracks get a control that speaks their language instead of the
        # raw number box: typing 128 to mean "back" is the same readability
        # problem as plotting it on a numeric axis.
        self.enum_row = ttk.Frame(p)
        self.enum_row.pack(fill="x", pady=(0, 4))

    def _build_right(self, p):
        ttk.Label(p, text="Preview", font=("TkDefaultFont", 9, "bold")).pack(anchor="w")
        self.preview = tk.Canvas(p, height=200, bg="#101014",
                                 highlightthickness=0)
        self.preview.pack(fill="x", pady=(2, 6))
        # The first draw happens before Tk has laid the canvas out, so its
        # reported size is a placeholder and the quad comes out tiny. Redraw
        # once the real geometry arrives, and on every resize after that.
        self.preview.bind("<Configure>", lambda e: self._draw_preview())

        row = ttk.Frame(p)
        row.pack(fill="x")
        self.play_btn = ttk.Button(row, text="Play", width=7, command=self._toggle_play)
        self.play_btn.pack(side="left")
        ttk.Label(row, text=" frame ").pack(side="left")
        self.frame_label = ttk.Label(row, text="0")
        self.frame_label.pack(side="left")

        self.frame_scale = ttk.Scale(p, from_=0, to=59, orient="horizontal",
                                     command=self._on_scrub)
        self.frame_scale.pack(fill="x", pady=4)

        row_opts = ttk.Frame(p)
        row_opts.pack(fill="x", pady=(2, 0))

        self.all_targets_var = tk.IntVar(value=0)
        ttk.Checkbutton(row_opts, text="All targets",
                        variable=self.all_targets_var,
                        command=self._toggle_all_targets).pack(side="left")

        # Which colour the preview tints with. On a mesh with normals a
        # vertex-colour track is ignored -- every NORMAL command re-runs the
        # lighting equation and overwrites it -- so a lit model animates
        # emission instead, and the preview has to be told which it is.
        self.tint_var = tk.StringVar(value="emission")
        ttk.Label(row_opts, text="  tint ").pack(side="left")
        ttk.OptionMenu(row_opts, self.tint_var, "emission",
                       "emission", "vertex color",
                       command=lambda *_: self._toggle_tint()).pack(side="left")

        row2 = ttk.Frame(p)
        row2.pack(fill="x", pady=4)
        ttk.Label(row2, text="Length (frames) ").pack(side="left")
        self.length_var = tk.StringVar(value="60")
        e = ttk.Entry(row2, textvariable=self.length_var, width=6)
        e.pack(side="left")
        e.bind("<Return>", lambda ev: self._set_length())
        ttk.Button(row2, text="Set", command=self._set_length).pack(side="left", padx=2)

        ttk.Separator(p).pack(fill="x", pady=6)
        ttk.Label(p, text="Evaluated this frame",
                  font=("TkDefaultFont", 9, "bold")).pack(anchor="w")
        self.value_text = tk.Text(p, height=16, font=("TkFixedFont", 8),
                                  bg="#f7f7f7", relief="flat", state="disabled")
        self.value_text.pack(fill="both", expand=True, pady=2)

    # -- state helpers ------------------------------------------------------

    @property
    def target(self):
        if not self.anim["targets"]:
            return None
        self.target_idx = min(self.target_idx, len(self.anim["targets"]) - 1)
        return self.anim["targets"][self.target_idx]

    @property
    def track(self):
        t = self.target
        if t is None or self.track_idx is None:
            return None
        if self.track_idx >= len(t["tracks"]):
            return None
        return t["tracks"][self.track_idx]

    def _refresh_all(self):
        self._refresh_targets()
        self._refresh_tracks()
        self._refresh_sizes()
        self.length_var.set(str(self.anim["num_frames"]))
        self.frame_scale.configure(to=max(0, self.anim["num_frames"] - 1))
        self._draw_timeline()
        self._draw_preview()

    def _refresh_targets(self):
        self.target_list.delete(0, "end")
        for i, t in enumerate(self.anim["targets"]):
            name = t["name"] or "(unnamed - drives everything)"
            self.target_list.insert("end", f"{i}: {name}")
        if self.anim["targets"]:
            self.target_list.selection_clear(0, "end")
            self.target_list.selection_set(self.target_idx)

    def _refresh_tracks(self):
        self.track_list.delete(0, "end")
        t = self.target
        if t is None:
            return
        for track in t["tracks"]:
            name = A.TRACK_NAMES.get(track["type"], f"type {track['type']}")
            store = A.STORAGE_NAMES[track.get("storage", A.STORE_KEYS)]
            size = A.track_byte_size(track, self.anim["num_frames"])
            self.track_list.insert("end", f"{name:<20} {store:<5} {size:>4}B")
        if self.track_idx is not None and self.track_idx < len(t["tracks"]):
            self.track_list.selection_clear(0, "end")
            self.track_list.selection_set(self.track_idx)
            self._sync_track_controls()

    def _refresh_sizes(self):
        """Report the file size, a moment after the edits stop.

        Measuring costs two full serialisations, one of them through
        optimize(), which bakes every dense track to a per-frame array just to
        count its bytes. Doing that on every keystroke and every drag release is
        real work for a number nobody reads mid-gesture.
        """
        if self._size_job is not None:
            self.after_cancel(self._size_job)
        self._size_job = self.after(200, self._compute_sizes)

    def _compute_sizes(self):
        self._size_job = None
        try:
            total = len(A.dumps(self.anim))
            opt = len(A.dumps(A.optimize(self.anim)))
        except A.AnimMatFormatError as e:
            self.size_label.config(text=f"cannot serialise:\n{e}")
            return
        saving = "" if opt >= total else f"\noptimized: {opt} B  (-{total - opt})"
        self.size_label.config(text=f"file size: {total} B{saving}")

    def _sync_track_controls(self):
        track = self.track
        if track is None:
            self.track_title.config(text="No track selected")
            return
        name = A.TRACK_NAMES.get(track["type"], "?")
        self.track_title.config(text=name)

        storage = track.get("storage", A.STORE_KEYS)
        self.storage_var.set(A.STORAGE_NAMES[storage])
        self.interp_var.set("step" if track.get("interp", A.INTERP_STEP)
                            == A.INTERP_STEP else "linear")

        # Interpolation is a property of keyframes. A constant has nothing to
        # interpolate and a baked track was already sampled every frame, so the
        # control does nothing in either -- and a track the hardware can only
        # step through cannot interpolate at all.
        step_only = track["type"] in A.STEP_ONLY_TRACKS
        usable = storage == A.STORE_KEYS and not step_only
        self.interp_menu.configure(state="normal" if usable else "disabled")

        # The colour picker only applies to colour tracks; it used to be always
        # enabled and explain itself with an error box after the click.
        self.color_btn.configure(
            state="normal" if track["type"] in COLOR_TRACKS else "disabled")

        self._sync_value_editor()

    # -- file ---------------------------------------------------------------

    # -- dirty state ------------------------------------------------------

    def _mark_dirty(self):
        if not self._dirty:
            self._dirty = True
            self._update_title()

    def _changed(self):
        """Something about the animation was edited."""
        self._mark_dirty()
        self._refresh_all()

    def _update_title(self):
        name = os.path.basename(self.path) if self.path else "<unsaved>"
        star = " *" if self._dirty else ""
        self.title(f"NEA AnimMat editor - {name}{star}")

    def _confirm_discard(self):
        """Ask before throwing work away. Returns False to cancel."""
        if not self._dirty:
            return True
        answer = messagebox.askyesnocancel(
            "Unsaved changes", "Save changes before continuing?")
        if answer is None:
            return False
        if answer:
            self._save()
            # A failed or cancelled save leaves the flag set; do not continue.
            return not self._dirty
        return True

    def _on_close(self):
        if self._confirm_discard():
            self.destroy()

    # -- file -------------------------------------------------------------

    def _new(self):
        if not self._confirm_discard():
            return
        self.anim = A.new_animation(60)
        self.path = None
        self.target_idx, self.track_idx = 0, None
        self._sel_key = None
        self._images, self._photos, self._target_image = [], {}, {}
        self._dirty = False
        self._update_title()
        self._refresh_all()

    def _open(self):
        if not self._confirm_discard():
            return
        p = filedialog.askopenfilename(
            filetypes=[("AnimMat", "*.neaanimmat *.bin"), ("All files", "*")])
        if p:
            self._open_path(p)

    def _open_path(self, p):
        try:
            anim = A.load(p)
        except (A.AnimMatFormatError, OSError) as e:
            messagebox.showerror("Open failed", str(e))
            return

        self.anim = anim
        self.path = p
        self.target_idx = 0
        # Land on something editable rather than an empty timeline.
        self.track_idx = 0 if self.anim["targets"][0]["tracks"] else None
        self._sel_key = None
        self._dirty = False

        self._load_sidecar()
        self._update_title()
        self._refresh_all()

    def _save(self):
        if not self.path:
            return self._save_as()
        try:
            A.dump(self.anim, self.path)
        except (A.AnimMatFormatError, OSError) as e:
            messagebox.showerror("Save failed", str(e))
            return
        self._save_sidecar()
        self._dirty = False
        self._update_title()

    def _save_as(self):
        p = filedialog.asksaveasfilename(defaultextension=".neaanimmat",
                                         filetypes=[("AnimMat", "*.neaanimmat")])
        if p:
            self.path = p
            self._save()

    def _save_as_v1(self):
        """Write the old single-material format.

        Opening a version 1 file and saving it silently produced a version 2
        one, which is a surprise if the runtime being targeted is older. Version
        1 holds exactly one material's tracks and has no room for a name, so
        this is only offered when the animation is that shape.
        """
        if len(self.anim["targets"]) != 1:
            messagebox.showwarning(
                "Cannot save as version 1",
                f"Version 1 holds one material's tracks and this animation has "
                f"{len(self.anim['targets'])} targets.\n\n"
                "Remove the extra targets first, or save as version 2.")
            return

        target = self.anim["targets"][0]
        if not target["tracks"]:
            messagebox.showwarning("Cannot save as version 1",
                                   "Version 1 needs at least one track.")
            return

        bad = [A.TRACK_NAMES.get(t["type"], "?") for t in target["tracks"]
               if t.get("storage", A.STORE_KEYS) != A.STORE_KEYS]
        if bad:
            messagebox.showwarning(
                "Cannot save as version 1",
                "Version 1 has no storage field: every track is keyframed.\n\n"
                "These are not: " + ", ".join(bad))
            return

        if target["name"]:
            if not messagebox.askokcancel(
                    "Target name will be lost",
                    f"Version 1 has nowhere to record the name '{target['name']}'."
                    "\n\nThe animation will still load, but it will drive "
                    "whichever material it is applied to rather than that one."):
                return

        p = filedialog.asksaveasfilename(defaultextension=".neaanimmat",
                                         filetypes=[("AnimMat", "*.neaanimmat")])
        if not p:
            return

        try:
            with open(p, "wb") as f:
                f.write(A.dumps_v1(self.anim))
        except (A.AnimMatFormatError, OSError) as e:
            messagebox.showerror("Save failed", str(e))

    # -- images -----------------------------------------------------------

    def _load_sidecar(self):
        """Pick up the image table recorded beside this animation, if any."""
        data = SC.load(self.path)

        self._images = [p for p in data.get("images", []) if isinstance(p, str)]
        self._photos = {}

        self._target_image = {}
        for name, entry in (data.get("targets") or {}).items():
            if isinstance(entry, dict) and isinstance(entry.get("image"), int):
                self._target_image[name] = entry["image"]

        self._tint_emission = bool(data.get("tint_emission", True))
        self._all_targets = bool(data.get("all_targets", False))

        if hasattr(self, "tint_var"):
            self.tint_var.set("emission" if self._tint_emission
                              else "vertex color")
            self.all_targets_var.set(1 if self._all_targets else 0)

    def _save_sidecar(self):
        """Write the image table, or remove the sidecar once it holds nothing.

        Leaving an empty sidecar behind after the last image is cleared would
        litter the assets directory with files that say nothing.
        """
        if not self._images and not self._target_image:
            SC.delete(self.path)
            return

        SC.save(self.path, {
            "images": self._images,
            "targets": {name: {"image": idx}
                        for name, idx in self._target_image.items()},
            "tint_emission": self._tint_emission,
            "all_targets": self._all_targets,
        })

    def _image_for(self, index):
        """The loaded PIL image at a table index, or None if it is not there.

        Loading is deferred and cached, and a path that no longer resolves
        caches as None so a missing file is not re-opened every frame.
        """
        if index is None or not (0 <= index < len(self._images)):
            return None

        if index in self._photos:
            return self._photos[index]

        full = SC.resolve_path(self.path, self._images[index])
        img = None
        if full:
            try:
                img = Image.open(full).convert("RGBA")
            except (OSError, ValueError):
                img = None

        self._photos[index] = img
        return img

    def _target_photo(self, target, values):
        """Which image a target shows this frame.

        A texture/palette swap moves the texture index, so the preview has to
        follow it; 0xFF means "leave the texture alone" and keeps whatever the
        target's own image is.
        """
        index = self._target_image.get(target.get("name", ""))

        swap = values.get(A.TEXPAL_SWAP)
        if swap is not None:
            tex = (swap >> 8) & 0xFF
            if tex != 0xFF:
                index = tex

        mat = values.get(A.MATERIAL_SWAP)
        if mat is not None and mat != 0xFF:
            index = mat

        return self._image_for(index)

    def _import_image(self):
        """Add an image to the table and give it to the selected target."""
        target = self.target
        if target is None:
            return

        p = filedialog.askopenfilename(
            title="Image for this target",
            filetypes=[("Images", "*.png *.gif *.jpg *.jpeg *.bmp"),
                       ("All files", "*")])
        if not p:
            return

        try:
            Image.open(p).close()
        except (OSError, ValueError) as e:
            messagebox.showerror("Cannot read image", str(e))
            return

        stored = SC.store_path(self.path, p)

        if stored in self._images:
            index = self._images.index(stored)
        else:
            index = len(self._images)
            self._images.append(stored)

        self._photos.pop(index, None)
        self._target_image[target.get("name", "")] = index

        self._save_sidecar()
        self._refresh_all()

    def _manage_images(self):
        """Show the image table, and let entries be reassigned or cleared.

        The table exists because a swap track's value is an index into it, the
        same way the runtime indexes its material and texture tables. Showing it
        is what makes a swap track's numbers mean something.
        """
        dlg = tk.Toplevel(self)
        dlg.title("Images")
        dlg.transient(self)

        ttk.Label(dlg, justify="left", padding=8,
                  text="Preview images, in table order. A material or\n"
                       "texture swap track uses these indices, so index 0\n"
                       "here is what value 0 in such a track selects.\n\n"
                       "These are recorded beside the animation and never\n"
                       "written into it.").pack(anchor="w")

        listbox = tk.Listbox(dlg, width=60, height=8)
        listbox.pack(padx=8, fill="both", expand=True)

        def repopulate():
            listbox.delete(0, "end")
            for i, stored in enumerate(self._images):
                users = [n for n, idx in self._target_image.items() if idx == i]
                missing = "" if SC.resolve_path(self.path, stored) else "  [missing]"
                who = ("  <- " + ", ".join(users)) if users else ""
                listbox.insert("end", f"{i}: {stored}{missing}{who}")
            if not self._images:
                listbox.insert("end", "(no images -- File > Import image)")

        def clear_target():
            target = self.target
            if target is not None:
                self._target_image.pop(target.get("name", ""), None)
                self._save_sidecar()
                repopulate()
                self._refresh_all()

        def forget_all():
            self._images, self._photos, self._target_image = [], {}, {}
            self._save_sidecar()
            repopulate()
            self._refresh_all()

        row = ttk.Frame(dlg, padding=8)
        row.pack(fill="x")
        ttk.Button(row, text="Unassign selected target",
                   command=clear_target).pack(side="left")
        ttk.Button(row, text="Forget all", command=forget_all).pack(side="left",
                                                                   padx=4)
        ttk.Button(row, text="Close", command=dlg.destroy).pack(side="right")

        repopulate()

    def _optimize(self):
        try:
            new = A.optimize(self.anim)
        except A.AnimMatFormatError as e:
            messagebox.showerror("Optimize failed", str(e))
            return
        before, after = len(A.dumps(self.anim)), len(A.dumps(new))
        self.anim = new
        self._changed()
        messagebox.showinfo("Optimized",
                            f"{before} bytes -> {after} bytes\n\n"
                            "Values are unchanged; only how they are stored is.")

    # -- targets and tracks -------------------------------------------------

    def _selected_key(self):
        """The (index, frame, value) of the selected key, or None."""
        track = self.track
        if track is None or self._sel_key is None:
            return None
        if track.get("storage", A.STORE_KEYS) != A.STORE_KEYS:
            return None
        keys = track.get("keys", [])
        if not (0 <= self._sel_key < len(keys)):
            return None
        return self._sel_key, keys[self._sel_key][0], keys[self._sel_key][1]

    def _set_selected_value(self, raw):
        """Write a new value into the selected key."""
        sel = self._selected_key()
        if sel is None:
            return
        i, frame, _ = sel
        self.track["keys"][i] = [frame, raw & 0xFFFFFFFF]
        self.key_value_var.set(TRACK_UI[self.track["type"]][2](raw))
        self._changed()

    def _sync_value_editor(self):
        """Rebuild the value control to match the selected track."""
        for child in self.enum_row.winfo_children():
            child.destroy()

        track = self.track
        if track is None:
            return

        ttype = track["type"]
        if ttype not in A.STEP_ONLY_TRACKS:
            return

        sel = self._selected_key()
        value = sel[2] if sel else 0
        state = "normal" if sel else "disabled"

        if ttype == A.CULLING:
            ttk.Label(self.enum_row, text="Culling ").pack(side="left")
            var = tk.StringVar(value=CULL_NAMES.get(value, "back"))
            names = ["none", "front", "back", "all"]
            by_name = {v: k for k, v in CULL_NAMES.items()}
            menu = ttk.OptionMenu(
                self.enum_row, var, var.get(), *names,
                command=lambda v: self._set_selected_value(by_name[v]))
            menu.configure(state=state)
            menu.pack(side="left")

        elif ttype == A.LIGHTS:
            ttk.Label(self.enum_row, text="Lights ").pack(side="left")
            self._light_vars = []
            for bit in range(4):
                v = tk.IntVar(value=1 if value & (1 << bit) else 0)
                self._light_vars.append(v)
                cb = ttk.Checkbutton(
                    self.enum_row, text=f"L{bit}", variable=v,
                    command=lambda: self._set_selected_value(
                        sum(b << i for i, b in
                            enumerate(x.get() for x in self._light_vars))))
                cb.configure(state=state)
                cb.pack(side="left")

        elif ttype == A.MATERIAL_SWAP:
            ttk.Label(self.enum_row, text="Material index ").pack(side="left")
            var = tk.StringVar(value=str(value & 0xFF))
            sb = ttk.Spinbox(
                self.enum_row, from_=0, to=255, width=5, textvariable=var,
                command=lambda: self._set_selected_value(int(var.get())))
            sb.configure(state=state)
            sb.pack(side="left")
            ttk.Label(self.enum_row,
                      text="  " + self._image_note(value & 0xFF),
                      foreground="#666").pack(side="left")

        elif ttype == A.TEXPAL_SWAP:
            tex = (value >> 8) & 0xFF
            pal = value & 0xFF
            self._texpal_vars = {}

            def push():
                t = self._texpal_vars["tex"]
                p_ = self._texpal_vars["pal"]
                tv = 0xFF if t[1].get() else max(0, min(254, _safe_int(t[0])))
                pv = 0xFF if p_[1].get() else max(0, min(254, _safe_int(p_[0])))
                self._set_selected_value((tv << 8) | pv)

            for label, raw in (("Texture", tex), ("Palette", pal)):
                ttk.Label(self.enum_row, text=f"{label} ").pack(side="left")
                num = tk.StringVar(value=str(raw if raw != 0xFF else 0))
                keep = tk.IntVar(value=1 if raw == 0xFF else 0)
                sb = ttk.Spinbox(self.enum_row, from_=0, to=254, width=4,
                                 textvariable=num, command=push)
                sb.configure(state=state)
                sb.pack(side="left")
                cb = ttk.Checkbutton(self.enum_row, text="unchanged",
                                     variable=keep, command=push)
                cb.configure(state=state)
                cb.pack(side="left", padx=(2, 10))
                self._texpal_vars["tex" if label == "Texture" else "pal"] = \
                    (num, keep)

    def _image_note(self, index):
        """How an image-table index reads in the value editor."""
        if not (0 <= index < len(self._images)):
            return "(no image at this index)"
        return os.path.basename(self._images[index])

    def _clear_key_editor(self):
        """Drop the selected key and empty the value boxes.

        They used to keep whatever the previous track held, so pressing Set
        after switching tracks wrote a value belonging to a different property
        into the new one.
        """
        self._sel_key = None
        self.key_frame_var.set("")
        self.key_value_var.set("")
        self._sync_value_editor()

    def _on_target_select(self, _):
        sel = self.target_list.curselection()
        if sel:
            self.target_idx = sel[0]
            self.track_idx = None
            self._clear_key_editor()
            self._refresh_tracks()
            self._sync_track_controls()
            self._draw_timeline()
            self._draw_preview()

    def _add_target(self):
        if len(self.anim["targets"]) >= A.MAX_TARGETS:
            messagebox.showwarning("Limit", f"At most {A.MAX_TARGETS} targets.")
            return
        # Names have to be unique or only the first of them ever binds.
        used = {t["name"] for t in self.anim["targets"]}
        name = "material"
        n = 2
        while name in used:
            name = f"material{n}"
            n += 1

        self.anim["targets"].append({"name": name, "tracks": []})
        self.target_idx = len(self.anim["targets"]) - 1
        self.track_idx = None
        self._changed()

    def _rename_target(self):
        t = self.target
        if t is None:
            return
        dlg = tk.Toplevel(self)
        dlg.title("Rename target")
        dlg.transient(self)
        ttk.Label(dlg, text="Material name (max 31 characters).\n"
                            "Leave empty to drive every material.",
                  justify="left").pack(padx=10, pady=8)
        var = tk.StringVar(value=t["name"])
        e = ttk.Entry(dlg, textvariable=var, width=34)
        e.pack(padx=10)
        e.focus_set()

        def ok():
            wanted = var.get()[:A.NAME_LEN - 1]

            # NEA_AnimMatFindTarget() returns the first match, so a second
            # target with the same name would never bind and never say so.
            clash = any(other is not t and other["name"] == wanted
                        for other in self.anim["targets"])
            if clash:
                messagebox.showwarning(
                    "Name already used",
                    f"Another target is already called '{wanted}'.\n\n"
                    "The runtime binds the first match, so the second would "
                    "never drive anything.")
                return

            old_name = t["name"]
            t["name"] = wanted

            # Follow the image assignment across the rename.
            if old_name in self._target_image:
                self._target_image[wanted] = self._target_image.pop(old_name)
                self._save_sidecar()

            dlg.destroy()
            self._changed()

        ttk.Button(dlg, text="OK", command=ok).pack(pady=8)
        dlg.bind("<Return>", lambda ev: ok())

    def _remove_target(self):
        if len(self.anim["targets"]) <= 1:
            messagebox.showwarning("Limit", "An animation needs one target.")
            return
        del self.anim["targets"][self.target_idx]
        self.target_idx = max(0, self.target_idx - 1)
        self.track_idx = None
        self._changed()

    def _on_track_select(self, _):
        sel = self.track_list.curselection()
        if sel:
            self.track_idx = sel[0]
            self._clear_key_editor()
            self._sync_track_controls()
            self._draw_timeline()

    def _add_track(self):
        t = self.target
        if t is None:
            return
        if len(t["tracks"]) >= A.MAX_TRACKS:
            messagebox.showwarning("Limit", f"At most {A.MAX_TRACKS} tracks.")
            return
        wanted = self.new_track_type.get()
        ttype = next(k for k, v in A.TRACK_NAMES.items() if v == wanted)
        if any(tr["type"] == ttype for tr in t["tracks"]):
            messagebox.showwarning("Duplicate",
                                   "That property already has a track.")
            return
        t["tracks"].append(A.new_track(ttype))
        self.track_idx = len(t["tracks"]) - 1
        self._changed()

    def _remove_track(self):
        t = self.target
        if t is None or self.track_idx is None:
            return
        del t["tracks"][self.track_idx]
        self.track_idx = None
        self._changed()

    def _set_storage(self):
        track = self.track
        if track is None:
            return
        wanted = {v: k for k, v in A.STORAGE_NAMES.items()}[self.storage_var.get()]
        if wanted == track.get("storage"):
            return

        n = self.anim["num_frames"]
        try:
            if wanted == A.STORE_BAKED:
                new = A.bake_track(track, n)
            elif wanted == A.STORE_CONST:
                new = {"type": track["type"], "storage": A.STORE_CONST,
                       "value": A.evaluate_track(track, 0)}
            else:
                # Coming back to keyframes: sample a handful so the track stays
                # editable rather than collapsing to a single value.
                step = max(1, n // 8)
                keys = [[f, A.evaluate_track(track, f << 12)]
                        for f in range(0, n, step)]
                new = {"type": track["type"], "interp": A.INTERP_LINEAR,
                       "storage": A.STORE_KEYS, "keys": keys}
        except A.AnimMatFormatError as e:
            messagebox.showerror("Cannot change storage", str(e))
            self._sync_track_controls()
            return

        self.target["tracks"][self.track_idx] = new
        self._changed()

    def _set_interp(self):
        track = self.track
        if track is None:
            return
        track["interp"] = (A.INTERP_STEP if self.interp_var.get() == "step"
                           else A.INTERP_LINEAR)
        self._changed()

    def _set_length(self):
        try:
            n = max(1, min(65535, int(self.length_var.get())))
        except ValueError:
            return
        old_n = self.anim["num_frames"]
        if n == old_n:
            return

        self.anim["num_frames"] = n

        for target in self.anim["targets"]:
            for track in target["tracks"]:
                storage = track.get("storage", A.STORE_KEYS)

                if storage == A.STORE_BAKED:
                    # A baked track holds one value per frame, so it has to be
                    # resampled across the new length. Repeating the last value
                    # instead -- which is what this did -- flat-lines the tail of
                    # every curve the moment an animation is made longer.
                    vals = track["values"]
                    if not vals:
                        continue
                    last = len(vals) - 1
                    track["values"] = [
                        vals[min(last, round(i * last / (n - 1)))] if n > 1
                        else vals[0]
                        for i in range(n)]

                elif storage == A.STORE_KEYS:
                    # Keyframes are positions in time, so they scale with the
                    # length too, and stay in order and inside the new range.
                    keys = track.get("keys", [])
                    scaled = []
                    seen = set()
                    for frame, value in keys:
                        f = (round(frame * (n - 1) / (old_n - 1))
                             if old_n > 1 else 0)
                        f = max(0, min(n - 1, f))
                        # Shortening can collapse two keys onto one frame; keep
                        # the first, since a duplicate frame is a dead key.
                        if f in seen:
                            continue
                        seen.add(f)
                        scaled.append([f, value])
                    if scaled:
                        track["keys"] = scaled

        self.frame = min(self.frame, n - 1)
        self.frame_scale.set(self.frame)
        self._changed()

    # -- timeline -----------------------------------------------------------

    def _plot_geom(self):
        w = max(self.canvas.winfo_width(), 50)
        h = max(self.canvas.winfo_height(), 50)
        return w, h, 40, 12, w - 16, h - 26   # x0, y0, x1, y1

    def _to_px(self, frame, value, lo, hi):
        w, h, x0, y0, x1, y1 = self._plot_geom()
        n = max(1, self.anim["num_frames"] - 1)
        x = x0 + (x1 - x0) * frame / n
        span = (hi - lo) or 1
        y = y1 - (y1 - y0) * (value - lo) / span
        return x, y

    def _from_px(self, px, py, lo, hi):
        w, h, x0, y0, x1, y1 = self._plot_geom()
        n = max(1, self.anim["num_frames"] - 1)
        frame = round((px - x0) / max(1, x1 - x0) * n)
        span = (hi - lo) or 1
        value = lo + (y1 - py) / max(1, y1 - y0) * span
        return max(0, min(n, frame)), value

    def _draw_timeline(self):
        c = self.canvas
        c.delete("all")
        w, h, x0, y0, x1, y1 = self._plot_geom()

        track = self.track
        if track is None:
            c.create_text(w // 2, h // 2, fill="#666",
                          text="Select a track to edit its values")
            return

        n = self.anim["num_frames"]
        storage = track.get("storage", A.STORE_KEYS)

        # A bitmask, a culling mode or a packed swap index plotted on a numeric
        # axis is a staircase between arbitrary numbers. These tracks never
        # interpolate, so what matters is which value holds over which stretch
        # of time -- draw that instead.
        if track["type"] in A.STEP_ONLY_TRACKS:
            self._draw_segments(track, n)
            return

        samples = [A.evaluate_track(track, f << 12) for f in range(n)]
        signed = [A._s32(v) if track["type"] in A.FIXED_TRACKS else v
                  for v in samples]
        lo, hi = value_range(track["type"], signed)

        # Grid and axis labels.
        for i in range(5):
            y = y0 + (y1 - y0) * i / 4
            c.create_line(x0, y, x1, y, fill="#2a2a30")
            val = hi - (hi - lo) * i / 4
            # For the fixed-point tracks the plotted values are already raw
            # 1.19.12, so they only need converting once. Running them back
            # through f32() multiplied by 4096 a second time and produced an
            # axis whose labels were neither right nor even in order.
            label = (_fmt_f32(int(val)) if track["type"] in A.FIXED_TRACKS
                     else f"{val:.0f}")
            c.create_text(x0 - 5, y, text=label, fill="#777", anchor="e",
                          font=("TkFixedFont", 7))

        for f in range(0, n, max(1, n // 10)):
            x, _ = self._to_px(f, lo, lo, hi)
            c.create_line(x, y0, x, y1, fill="#26262c")
            c.create_text(x, y1 + 9, text=str(f), fill="#777",
                          font=("TkFixedFont", 7))

        # The evaluated curve: exactly what the runtime will produce.
        pts = []
        for f in range(n):
            pts.extend(self._to_px(f, signed[f], lo, hi))
        if len(pts) >= 4:
            c.create_line(*pts, fill="#4da3ff", width=2,
                          joinstyle="round")

        if storage == A.STORE_KEYS:
            for i, (kf, kv) in enumerate(track["keys"]):
                v = A._s32(kv) if track["type"] in A.FIXED_TRACKS else kv
                x, y = self._to_px(kf, v, lo, hi)
                sel = (i == self._sel_key)
                c.create_rectangle(x - 4, y - 4, x + 4, y + 4,
                                   fill="#ffd24d" if sel else "#ff8c42",
                                   outline="#1b1b1f", tags=f"key{i}")
        elif storage == A.STORE_CONST:
            c.create_text(w // 2, y0 + 12, fill="#8f8",
                          text="constant - no per-frame cost at all",
                          font=("TkDefaultFont", 8))
        else:
            c.create_text(w // 2, y0 + 12, fill="#8f8",
                          text=f"baked - {n} values, one load per frame",
                          font=("TkDefaultFont", 8))

        # Playhead.
        x, _ = self._to_px(min(self.frame, n - 1), lo, lo, hi)
        c.create_line(x, y0, x, y1, fill="#ff4d6d", width=1)

    # Colours cycled through as a step track's value changes, so neighbouring
    # segments are always distinguishable without meaning anything by the hue.
    SEGMENT_COLORS = ["#2f5d8a", "#3d7a52", "#8a5a2f", "#6a3d7a",
                      "#2f7a7a", "#7a2f4a"]

    def _draw_segments(self, track, n):
        """Draw a step-only track as labelled bands rather than a curve."""
        c = self.canvas
        w, h, x0, y0, x1, y1 = self._plot_geom()

        label_of = SEGMENT_LABEL.get(track["type"], str)
        storage = track.get("storage", A.STORE_KEYS)

        # Walk the evaluated value and start a new band wherever it changes.
        bands = []
        prev = None
        for f in range(n):
            v = A.evaluate_track(track, f << 12)
            if v != prev:
                bands.append([f, f, v])
                prev = v
            else:
                bands[-1][1] = f

        band_top = y0 + 20
        band_bot = y1 - 24
        value_order = []

        for start, end, value in bands:
            if value not in value_order:
                value_order.append(value)
            color = self.SEGMENT_COLORS[value_order.index(value)
                                        % len(self.SEGMENT_COLORS)]

            xa, _ = self._to_px(start, 0, 0, 1)
            xb, _ = self._to_px(min(end + 1, n - 1), 0, 0, 1)
            if end >= n - 1:
                xb = x1

            c.create_rectangle(xa, band_top, max(xa + 1, xb), band_bot,
                               fill=color, outline="#1b1b1f")

            text = label_of(value)
            if xb - xa > len(text) * 6 + 8:
                c.create_text((xa + xb) / 2, (band_top + band_bot) / 2,
                              text=text, fill="#eee",
                              font=("TkDefaultFont", 8))
            elif xb - xa > 14:
                # Too narrow for the label; put it above, alternating height so
                # a run of short segments stays readable.
                c.create_text((xa + xb) / 2,
                              band_top - 8 - 10 * (len(value_order) % 2),
                              text=text, fill="#aaa",
                              font=("TkFixedFont", 7))

        # Frame ruler, matching the curve view.
        for f in range(0, n, max(1, n // 10)):
            x, _ = self._to_px(f, 0, 0, 1)
            c.create_line(x, band_bot, x, band_bot + 4, fill="#555")
            c.create_text(x, y1 + 9, text=str(f), fill="#777",
                          font=("TkFixedFont", 7))

        # Keyframe handles sit on the boundaries, draggable along time only --
        # there is no meaningful vertical axis to drag along here.
        if storage == A.STORE_KEYS:
            for i, (kf, kv) in enumerate(track["keys"]):
                x, _ = self._to_px(kf, 0, 0, 1)
                sel = (i == self._sel_key)
                c.create_polygon(x, band_top - 6, x - 5, band_top - 14,
                                 x + 5, band_top - 14,
                                 fill="#ffd24d" if sel else "#ff8c42",
                                 outline="#1b1b1f")
        elif storage == A.STORE_CONST:
            c.create_text(w // 2, y0 + 10, fill="#8f8",
                          text="constant - no per-frame cost at all",
                          font=("TkDefaultFont", 8))
        else:
            c.create_text(w // 2, y0 + 10, fill="#8f8",
                          text=f"baked - {n} values, one load per frame",
                          font=("TkDefaultFont", 8))

        x, _ = self._to_px(min(self.frame, n - 1), 0, 0, 1)
        c.create_line(x, band_top - 16, x, band_bot + 4, fill="#ff4d6d")

    def _track_range(self, track):
        """Vertical span of a track's plot, from its evaluated values."""
        samples = [A.evaluate_track(track, f << 12)
                   for f in range(self.anim["num_frames"])]
        signed = [A._s32(v) if track["type"] in A.FIXED_TRACKS else v
                  for v in samples]
        return value_range(track["type"], signed)

    def _raw_from_value(self, track, value):
        """Turn a plotted y value back into the track's stored form."""
        if track["type"] in A.FIXED_TRACKS:
            return A.f32(value)

        _, rng, _, _ = TRACK_UI[track["type"]]
        v = int(round(value))
        if rng:
            v = max(rng[0], min(rng[1], v))
        return v & 0xFFFFFFFF

    def _hit_key(self, px, py):
        track = self.track
        if track is None or track.get("storage") != A.STORE_KEYS:
            return None

        # On a segment strip the handles sit in a row above the bands, so only
        # the horizontal distance matters.
        if track["type"] in A.STEP_ONLY_TRACKS:
            _, _, _, y0, _, y1 = self._plot_geom()
            band_top = y0 + 20
            if not (band_top - 18 <= py <= band_top + 2):
                return None
            for i, (kf, _kv) in enumerate(track["keys"]):
                x, _ = self._to_px(kf, 0, 0, 1)
                if abs(px - x) <= 7:
                    return i
            return None

        lo, hi = self._track_range(track)
        for i, (kf, kv) in enumerate(track["keys"]):
            v = A._s32(kv) if track["type"] in A.FIXED_TRACKS else kv
            x, y = self._to_px(kf, v, lo, hi)
            if abs(px - x) <= 6 and abs(py - y) <= 6:
                return i
        return None

    def _on_canvas_click(self, e):
        i = self._hit_key(e.x, e.y)
        self._sel_key = i
        self._drag_key = i
        if i is None or self.track is None:
            self._drag_range = None
        elif self.track["type"] in A.STEP_ONLY_TRACKS:
            self._drag_range = (0, 1)   # unused; only time moves
        else:
            self._drag_range = self._track_range(self.track)
        if i is not None:
            kf, kv = self.track["keys"][i]
            self.key_frame_var.set(str(kf))
            self.key_value_var.set(TRACK_UI[self.track["type"]][2](kv))
        else:
            self.key_frame_var.set("")
            self.key_value_var.set("")

        self._sync_value_editor()
        self._draw_timeline()

    def _on_canvas_drag(self, e):
        if self._drag_key is None or self._drag_range is None:
            return

        track = self.track
        if track is None:
            return

        # The range is fixed for the whole gesture. Recomputing it per motion
        # event meant evaluating every frame of the track on every mouse move,
        # and made the plot rescale under the key being dragged.
        if track["type"] in A.STEP_ONLY_TRACKS:
            # A segment strip has no vertical axis, so a drag moves the
            # boundary in time and leaves the value alone.
            frame, _ = self._from_px(e.x, e.y, 0, 1)
            raw = track["keys"][self._drag_key][1]
        else:
            lo, hi = self._drag_range
            frame, value = self._from_px(e.x, e.y, lo, hi)
            raw = self._raw_from_value(track, value)

        keys = track["keys"]

        # Two keys on one frame leaves a zero-length span, which the evaluator
        # skips -- so one of them silently stops doing anything. Refuse the move
        # rather than produce that, and let the drag continue vertically.
        if any(i != self._drag_key and k[0] == frame for i, k in enumerate(keys)):
            frame = keys[self._drag_key][0]

        keys[self._drag_key] = [frame, raw]

        # Keep the list sorted and follow the dragged key by identity, not by
        # value: two keys can hold the same [frame, value] pair and index() was
        # picking whichever came first.
        dragged = keys[self._drag_key]
        keys.sort(key=lambda k: k[0])
        self._drag_key = next(i for i, k in enumerate(keys) if k is dragged)
        self._sel_key = self._drag_key

        self.key_frame_var.set(str(frame))
        self.key_value_var.set(TRACK_UI[track["type"]][2](raw))
        self._draw_timeline()
        self._draw_preview()

    def _end_drag(self):
        if self._drag_key is not None:
            self._drag_key = None
            self._drag_range = None
            self._mark_dirty()
            self._refresh_tracks()
            self._refresh_sizes()

    def _on_canvas_double(self, e):
        track = self.track
        if track is None or track.get("storage") != A.STORE_KEYS:
            return
        if len(track["keys"]) >= A.MAX_KEYFRAMES:
            messagebox.showwarning("Limit", f"At most {A.MAX_KEYFRAMES} keys.")
            return
        lo, hi = self._track_range(track)
        frame, value = self._from_px(e.x, e.y, lo, hi)
        raw = self._raw_from_value(track, value)

        # Adding on top of an existing key replaces it rather than making a
        # second one on the same frame.
        track["keys"] = [k for k in track["keys"] if k[0] != frame]
        track["keys"].append([frame, raw])
        track["keys"].sort(key=lambda k: k[0])
        self._sel_key = next(i for i, k in enumerate(track["keys"])
                             if k[0] == frame)
        self.key_frame_var.set(str(frame))
        self.key_value_var.set(TRACK_UI[track["type"]][2](raw))
        self._changed()

    def _on_canvas_right(self, e):
        i = self._hit_key(e.x, e.y)
        track = self.track
        if i is None or track is None or len(track["keys"]) <= 1:
            return
        del track["keys"][i]
        self._sel_key = None
        self._changed()

    def _apply_key_edit(self):
        track = self.track
        i = getattr(self, "_sel_key", None)
        if track is None or i is None:
            return
        try:
            frame = max(0, min(self.anim["num_frames"] - 1,
                               int(self.key_frame_var.get())))
            raw = TRACK_UI[track["type"]][3](self.key_value_var.get()) & 0xFFFFFFFF
        except (ValueError, KeyError):
            return

        if any(j != i and k[0] == frame for j, k in enumerate(track["keys"])):
            messagebox.showwarning(
                "Frame already has a key",
                f"There is already a key on frame {frame}.\n\n"
                "Two keys on one frame leave a zero-length span, and the "
                "runtime skips it, so one of them would do nothing.")
            return

        edited = [frame, raw]
        track["keys"][i] = edited
        track["keys"].sort(key=lambda k: k[0])
        self._sel_key = next(j for j, k in enumerate(track["keys"])
                             if k is edited)
        self._changed()

    def _pick_color(self):
        """Pick a colour for the selected key.

        The two packed tracks hold a pair of RGB15 values in one word -- diffuse
        with ambient, specular with emission -- so they get a swatch each. Not
        being able to author those at all is why the v2 example had to write its
        emission values by hand in Python.
        """
        track = self.track
        sel = self._selected_key()
        if track is None or sel is None:
            return

        _, _, value = sel
        ttype = track["type"]

        if ttype not in COLOR_TRACKS:
            return

        if ttype not in PACKED_COLOR_TRACKS:
            rgb, _ = colorchooser.askcolor(rgb15_to_hex(value & 0x7FFF),
                                           title="Vertex color")
            if rgb is None:
                return
            self._set_selected_value(hex_to_rgb15(rgb))
            return

        low_name, high_name = PACKED_HALVES[ttype]

        dlg = tk.Toplevel(self)
        dlg.title(f"{low_name} / {high_name}")
        dlg.transient(self)

        ttk.Label(dlg, padding=8, justify="left",
                  text=f"This track holds two colours in one value.\n"
                       f"Bit 15 of each half is a hardware flag and is kept.").pack(
            anchor="w")

        # Both halves keep their flag bit; the hardware uses it to mean "take
        # the vertex colour from the diffuse" and "use the shininess table".
        state = {"low": value & 0x7FFF, "high": (value >> 16) & 0x7FFF,
                 "low_flag": value & 0x8000, "high_flag": value & 0x80000000}

        swatches = {}

        def repaint():
            swatches["low"].configure(bg=rgb15_to_hex(state["low"]))
            swatches["high"].configure(bg=rgb15_to_hex(state["high"]))

        def choose(half, title):
            rgb, _ = colorchooser.askcolor(rgb15_to_hex(state[half]),
                                           title=title, parent=dlg)
            if rgb is not None:
                state[half] = hex_to_rgb15(rgb)
                repaint()

        for half, label in (("low", low_name), ("high", high_name)):
            row = ttk.Frame(dlg, padding=(8, 2))
            row.pack(fill="x")
            ttk.Label(row, text=f"{label:<10}", width=10).pack(side="left")
            sw = tk.Label(row, width=8, relief="sunken",
                          bg=rgb15_to_hex(state[half]))
            sw.pack(side="left", padx=4)
            swatches[half] = sw
            ttk.Button(row, text="Choose...",
                       command=lambda h=half, t=label: choose(h, t)).pack(
                side="left")

        def ok():
            packed = (state["low"] | state["low_flag"]
                      | ((state["high"] | (state["high_flag"] >> 16)) << 16))
            dlg.destroy()
            self._set_selected_value(packed)

        btns = ttk.Frame(dlg, padding=8)
        btns.pack(fill="x")
        ttk.Button(btns, text="OK", command=ok).pack(side="right")
        ttk.Button(btns, text="Cancel", command=dlg.destroy).pack(side="right",
                                                                  padx=4)

    # -- preview ------------------------------------------------------------

    def _toggle_all_targets(self):
        self._all_targets = bool(self.all_targets_var.get())
        self._save_sidecar()
        self._draw_preview()

    def _toggle_tint(self):
        self._tint_emission = self.tint_var.get() == "emission"
        self._save_sidecar()
        self._draw_preview()

    def _toggle_play(self):
        self.playing = not self.playing
        self.play_btn.config(text="Pause" if self.playing else "Play")

    def _on_scrub(self, v):
        self.frame = int(float(v))
        self.frame_label.config(text=str(self.frame))
        self._draw_timeline()
        self._draw_preview()

    def _tick(self):
        if self.playing:
            self.frame = (self.frame + 1) % max(1, self.anim["num_frames"])
            self.frame_scale.set(self.frame)
        self.after(33, self._tick)

    # -- preview ------------------------------------------------------------

    PREVIEW_BG = (0x10, 0x10, 0x14)

    def _quad_image(self, target, vals, size):
        """Render one target's quad as an RGBA image, or None if it has no art.

        The UV transform is the inverse of what the runtime applies: the runtime
        moves the texture coordinates, so the sampled image moves the other way.
        """
        src = self._target_photo(target, vals)
        if src is None:
            return None

        sx = A.from_f32(A._s32(vals.get(A.TEX_SCALE_X, A.f32(1.0)))
                        & 0xFFFFFFFF) or 1.0
        sy = A.from_f32(A._s32(vals.get(A.TEX_SCALE_Y, A.f32(1.0)))
                        & 0xFFFFFFFF) or 1.0
        # A texture matrix translation is not in texels. GBATEK's mode 1 formula
        # multiplies the translation row by a 1/16 constant, so a translate of R
        # moves the texture by R/16 texels -- confirmed on hardware, where a
        # translate of 1024 on a 64 texel texture is pixel-identical to zero.
        ox = A.from_f32(A._s32(vals.get(A.TEX_SCROLL_X, 0)) & 0xFFFFFFFF) / 16.0
        oy = A.from_f32(A._s32(vals.get(A.TEX_SCROLL_Y, 0)) & 0xFFFFFFFF) / 16.0
        ang = vals.get(A.TEX_ROTATE, 0) * math.pi / 256.0

        tw, th = src.size

        # Texture coordinates run over the texture in texels, so a scroll of one
        # texture width is one full wrap. Work in texels and let the tile below
        # take care of the wrap.
        ca, sa = math.cos(ang), math.sin(ang)
        us = tw / size * sx
        vs = th / size * sy

        # Scrolling is periodic, so fold the offset back into one texture
        # before using it. Without this the sample window walks off the end of
        # the tiled image and the quad appears to shrink away rather than slide.
        ox = math.fmod(ox, tw)
        oy = math.fmod(oy, th)
        if ox < 0:
            ox += tw
        if oy < 0:
            oy += th

        # Destination (x, y) -> source texel, as the affine PIL wants.
        a = ca * us
        b = -sa * us
        d = sa * vs
        e = ca * vs
        cc = ox
        ff = oy

        # Tile first so scrolling and scaling reveal repeats instead of edge
        # clamp, which is what NEA_TEXTURE_WRAP_S/T do on the hardware.
        tiled = Image.new("RGBA", (tw * 3, th * 3))
        for ty in range(3):
            for tx in range(3):
                tiled.paste(src, (tx * tw, ty * th))

        quad = tiled.transform(
            (size, size), Image.AFFINE,
            (a, b, cc + tw, d, e, ff + th),
            resample=Image.BILINEAR)

        # Tint. A vertex-colour track is ignored by the hardware on a mesh with
        # normals, because any NORMAL command re-runs the lighting equation, so
        # which of the two the preview uses is the user's call.
        if self._tint_emission and A.SPECULAR_EMISSION in vals:
            tint = (vals[A.SPECULAR_EMISSION] >> 16) & 0x7FFF
        elif not self._tint_emission and A.COLOR in vals:
            tint = vals[A.COLOR]
        else:
            tint = 0x7FFF

        if tint != 0x7FFF:
            tr = (tint & 0x1F) * 255 // 31
            tg = ((tint >> 5) & 0x1F) * 255 // 31
            tb = ((tint >> 10) & 0x1F) * 255 // 31
            # Modulation is a per-channel multiply, so that is what this is.
            r, g, b_, al = quad.split()
            quad = Image.merge("RGBA", (_scale(r, tr), _scale(g, tg),
                                        _scale(b_, tb), al))

        alpha = vals.get(A.ALPHA, 31)
        if alpha < 31:
            al = quad.getchannel("A").point(lambda v: v * alpha // 31)
            quad.putalpha(al)

        return quad

    def _draw_preview(self):
        c = self.preview
        c.delete("all")

        if not self.anim["targets"]:
            return

        frame = min(self.frame, self.anim["num_frames"] - 1)
        targets = (self.anim["targets"] if self._all_targets
                   else [self.target])
        targets = [t for t in targets if t is not None]
        if not targets:
            return

        # winfo_* returns 1 until the widget is mapped; fall back to what the
        # canvas was configured with rather than laying out against that.
        w = c.winfo_width()
        h = c.winfo_height()
        if w <= 1:
            w = 240
        if h <= 1:
            h = int(c.cget("height"))

        canvas_img = Image.new("RGB", (w, h), self.PREVIEW_BG)

        cols = len(targets)
        cell = max(24, min(h - 24, (w - 8 * (cols + 1)) // cols))

        for i, target in enumerate(targets):
            vals = A.evaluate_target(target, frame)
            x = 8 + i * (cell + 8)
            y = (h - cell) // 2

            quad = self._quad_image(target, vals, cell)

            if quad is not None:
                canvas_img.paste(quad, (x, y), quad)
            else:
                self._paste_untextured(canvas_img, target, vals, x, y, cell)

        self._preview_photo = ImageTk.PhotoImage(canvas_img)
        c.create_image(0, 0, anchor="nw", image=self._preview_photo)

        # Labels go on the Tk canvas, above the bitmap, so they stay crisp.
        if self._all_targets:
            for i, target in enumerate(targets):
                x = 8 + i * (cell + 8)
                y = (h - cell) // 2
                label = target["name"] or "(unnamed)"
                c.create_text(x + cell / 2, y + cell + 8, text=label,
                              fill="#999", font=("TkFixedFont", 7))

        if not self._images:
            c.create_text(6, 8, anchor="nw", fill="#777",
                          font=("TkFixedFont", 8),
                          text="no image - File > Import image")

        self._refresh_values(targets[0] if not self._all_targets
                             else self.target, frame)

    def _paste_untextured(self, canvas_img, target, vals, x, y, cell):
        """The fallback quad: a tinted square with the UV transform as a grid.

        Used when a target has no image, which is the state every animation
        starts in.
        """
        alpha = vals.get(A.ALPHA, 31)
        color = vals.get(A.COLOR, 0x7FFF)
        if self._tint_emission and A.SPECULAR_EMISSION in vals:
            color = (vals[A.SPECULAR_EMISSION] >> 16) & 0x7FFF

        r = (color & 0x1F) * 255 // 31
        g = ((color >> 5) & 0x1F) * 255 // 31
        b = ((color >> 10) & 0x1F) * 255 // 31

        f = alpha / 31.0
        rr = int(r * f + self.PREVIEW_BG[0] * (1 - f))
        gg = int(g * f + self.PREVIEW_BG[1] * (1 - f))
        bb = int(b * f + self.PREVIEW_BG[2] * (1 - f))

        d = ImageDraw.Draw(canvas_img)
        d.rectangle([x, y, x + cell, y + cell], fill=(rr, gg, bb))

        sx = A.from_f32(A._s32(vals.get(A.TEX_SCALE_X, A.f32(1.0)))
                        & 0xFFFFFFFF) or 1.0
        sy = A.from_f32(A._s32(vals.get(A.TEX_SCALE_Y, A.f32(1.0)))
                        & 0xFFFFFFFF) or 1.0
        ox = A.from_f32(A._s32(vals.get(A.TEX_SCROLL_X, 0)) & 0xFFFFFFFF)
        oy = A.from_f32(A._s32(vals.get(A.TEX_SCROLL_Y, 0)) & 0xFFFFFFFF)
        ang = vals.get(A.TEX_ROTATE, 0) * math.pi / 256.0

        cx, cy, half = x + cell / 2, y + cell / 2, cell / 2
        grid = (rr * 4 // 10, gg * 4 // 10, bb * 4 // 10)

        def uv(u, v):
            # Same 1/16 as the textured path, then over a nominal 64 texel
            # texture so the grid drifts at a comparable rate.
            u = (u + ox / 16.0 / 64.0) * sx
            v = (v + oy / 16.0 / 64.0) * sy
            ca, sa = math.cos(ang), math.sin(ang)
            return (cx + (u * ca - v * sa) * half,
                    cy + (u * sa + v * ca) * half)

        for i in range(-2, 3):
            t = i / 2.0
            d.line([uv(t, -1), uv(t, 1)], fill=grid)
            d.line([uv(-1, t), uv(1, t)], fill=grid)

    def _refresh_values(self, target, frame):
        """The numeric read-out, which is the part that has to match the DS."""
        self.value_text.config(state="normal")
        self.value_text.delete("1.0", "end")

        if target is not None:
            vals = A.evaluate_target(target, frame)
            for track in target["tracks"]:
                t = track["type"]
                name = A.TRACK_NAMES.get(t, str(t))
                raw = vals.get(t, 0)
                if t in SEGMENT_LABEL:
                    shown = SEGMENT_LABEL[t](raw)
                elif t in TRACK_UI:
                    shown = TRACK_UI[t][2](raw)
                else:
                    shown = str(raw)
                self.value_text.insert("end", f"{name:<19}{shown}\n")

        self.value_text.config(state="disabled")


def _scale(band, factor):
    """Multiply an 8 bit band by a 0-255 factor, as modulation does."""
    return band.point(lambda v: (v * factor) // 255)


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else None
    Editor(path).mainloop()


if __name__ == "__main__":
    main()
