#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
#
# Copyright (c) 2026 Warioware64
#
# Editor for .neacell cell banks and their animations.
#
# The preview runs cell_format.evaluate_sequence, which is the same evaluator
# NEACell.c implements and tests/cell_eval pins them together -- so what this
# shows is what the DS will show. The OAM preview additionally drops everything
# the hardware could not draw, because "will this still work as OBJ sprites?"
# is a question best answered while authoring, not at build time.
#
#     python3 tools/cell_editor/cell_editor.py hero.neacell
#
# The bank names its artwork and does not contain it, so the image is attached
# separately through File -> Import image and remembered in a
# <file>.editor.json sidecar. Losing the sidecar costs the preview and nothing
# else.

import argparse
import os
import sys
import tkinter as tk
from tkinter import filedialog, messagebox, ttk

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import cell_format as C          # noqa: E402
import editor_sidecar            # noqa: E402

try:
    from PIL import Image, ImageTk
except ImportError:
    print("cell_editor needs Pillow: pip install Pillow", file=sys.stderr)
    raise


BG = "#1e1e22"
PANEL = "#26262c"
GRID = "#33333c"
ACCENT = "#5aa9e6"
WARN = "#e6705a"
TEXT = "#d8d8de"

CELL_ZOOM = 3
ATLAS_ZOOM = 2
PREVIEW_ZOOM = 2


def clamp(v, lo, hi):
    return lo if v < lo else (hi if v > hi else v)


class Editor(tk.Tk):
    def __init__(self, path=None):
        super().__init__()
        self.title("NEACell editor")
        self.geometry("1320x860")
        self.configure(bg=BG)

        self.path = None
        self.bank = C.new_bank()
        self.dirty = False

        self.atlas_image = None      # PIL image of the artwork
        self.atlas_photo = None
        self.piece_cache = {}

        self.sel_cell = 0
        self.sel_part = -1
        self.sel_seq = 0
        self.sel_frame = 0
        self.sel_key = None

        self.playing = True
        self.tick = 0
        self.backend = tk.StringVar(value="quads")

        self._drag = None

        self._build_menu()
        self._build_layout()

        if path:
            self.open_path(path)
        else:
            self.new_bank()

        self.protocol("WM_DELETE_WINDOW", self._on_close)
        self.after(33, self._tick)

    # -- chrome ------------------------------------------------------------

    def _build_menu(self):
        menu = tk.Menu(self)

        f = tk.Menu(menu, tearoff=0)
        f.add_command(label="New", accelerator="Ctrl+N", command=self.new_bank)
        f.add_command(label="Open...", accelerator="Ctrl+O",
                      command=self.open_dialog)
        f.add_command(label="Save", accelerator="Ctrl+S", command=self.save)
        f.add_command(label="Save As...", command=self.save_as)
        f.add_separator()
        f.add_command(label="Import image...", command=self.import_image)
        f.add_command(label="Import retail NCER/NANR...",
                      command=self.import_retail)
        f.add_separator()
        f.add_command(label="Quit", command=self._on_close)
        menu.add_cascade(label="File", menu=f)

        e = tk.Menu(menu, tearoff=0)
        e.add_command(label="Optimize track storage", command=self.optimize)
        e.add_command(label="Flatten hierarchy", command=self.flatten)
        e.add_command(label="Recompute bounds and budget",
                      command=self.recompute)
        menu.add_cascade(label="Edit", menu=e)

        v = tk.Menu(menu, tearoff=0)
        v.add_command(label="Validate for hardware OBJ...",
                      command=self.validate)
        menu.add_cascade(label="Validate", menu=v)

        self.config(menu=menu)
        self.bind("<Control-n>", lambda _e: self.new_bank())
        self.bind("<Control-o>", lambda _e: self.open_dialog())
        self.bind("<Control-s>", lambda _e: self.save())

    def _build_layout(self):
        root = tk.Frame(self, bg=BG)
        root.pack(fill="both", expand=True)

        self.left = tk.Frame(root, bg=PANEL, width=210)
        self.left.pack(side="left", fill="y")
        self.left.pack_propagate(False)

        self.right = tk.Frame(root, bg=PANEL, width=320)
        self.right.pack(side="right", fill="y")
        self.right.pack_propagate(False)

        self.centre = tk.Frame(root, bg=BG)
        self.centre.pack(side="left", fill="both", expand=True)

        self._build_left()
        self._build_centre()
        self._build_right()

        self.status = tk.Label(self, bg=PANEL, fg=TEXT, anchor="w",
                               font=("TkFixedFont", 9))
        self.status.pack(fill="x", side="bottom")

    def _listbox(self, parent, label, height, on_select):
        tk.Label(parent, text=label, bg=PANEL, fg=TEXT, anchor="w",
                 font=("TkDefaultFont", 9, "bold")).pack(fill="x", padx=6,
                                                         pady=(8, 2))
        lb = tk.Listbox(parent, height=height, bg=BG, fg=TEXT,
                        selectbackground=ACCENT, highlightthickness=0,
                        borderwidth=0, exportselection=False,
                        font=("TkFixedFont", 9))
        lb.pack(fill="x", padx=6)
        lb.bind("<<ListboxSelect>>", on_select)
        return lb

    def _build_left(self):
        self.cell_list = self._listbox(self.left, "Cells", 9,
                                       self._on_pick_cell)
        row = tk.Frame(self.left, bg=PANEL)
        row.pack(fill="x", padx=6, pady=3)
        for text, cmd in (("+", self.add_cell), ("dup", self.dup_cell),
                          ("-", self.del_cell)):
            tk.Button(row, text=text, command=cmd, width=4).pack(side="left")

        self.part_list = self._listbox(self.left, "Parts (last is on top)", 9,
                                       self._on_pick_part)
        row = tk.Frame(self.left, bg=PANEL)
        row.pack(fill="x", padx=6, pady=3)
        for text, cmd in (("+", self.add_part), ("-", self.del_part),
                          ("up", lambda: self.move_part(-1)),
                          ("dn", lambda: self.move_part(1))):
            tk.Button(row, text=text, command=cmd, width=4).pack(side="left")

        self.seq_list = self._listbox(self.left, "Sequences", 8,
                                      self._on_pick_seq)
        row = tk.Frame(self.left, bg=PANEL)
        row.pack(fill="x", padx=6, pady=3)
        for text, cmd in (("+", self.add_seq), ("-", self.del_seq)):
            tk.Button(row, text=text, command=cmd, width=4).pack(side="left")

        self.size_label = tk.Label(self.left, bg=PANEL, fg="#8a8a94",
                                   justify="left", anchor="w",
                                   font=("TkFixedFont", 8))
        self.size_label.pack(fill="x", padx=6, pady=8, side="bottom")

    def _build_centre(self):
        top = tk.Frame(self.centre, bg=BG)
        top.pack(fill="both", expand=True)

        self.compose = tk.Canvas(top, bg=BG, highlightthickness=0)
        self.compose.pack(side="left", fill="both", expand=True)
        self.compose.bind("<Configure>", lambda _e: self._refresh_compose())
        self.compose.bind("<Button-1>", self._on_compose_press)
        self.compose.bind("<B1-Motion>", self._on_compose_drag)
        self.compose.bind("<ButtonRelease-1>", self._on_compose_release)

        self.atlas_canvas = tk.Canvas(top, bg=BG, width=280,
                                      highlightthickness=0)
        self.atlas_canvas.pack(side="right", fill="y")
        self.atlas_canvas.bind("<Button-1>", self._on_atlas_click)

        bottom = tk.Frame(self.centre, bg=PANEL, height=210)
        bottom.pack(fill="x")
        bottom.pack_propagate(False)

        bar = tk.Frame(bottom, bg=PANEL)
        bar.pack(fill="x")
        tk.Label(bar, text="Timeline", bg=PANEL, fg=TEXT,
                 font=("TkDefaultFont", 9, "bold")).pack(side="left", padx=6)
        tk.Button(bar, text="play/pause", command=self.toggle_play).pack(
            side="left", padx=4, pady=3)
        tk.Button(bar, text="+frame", command=self.add_frame).pack(side="left")
        tk.Button(bar, text="-frame", command=self.del_frame).pack(side="left")
        tk.Button(bar, text="+track", command=self.add_track).pack(
            side="left", padx=(12, 0))
        tk.Button(bar, text="-track", command=self.del_track).pack(side="left")

        self.timeline = tk.Canvas(bottom, bg=BG, highlightthickness=0)
        self.timeline.pack(fill="both", expand=True, padx=6, pady=4)
        self.timeline.bind("<Configure>", lambda _e: self._refresh_timeline())
        self.timeline.bind("<Button-1>", self._on_timeline_press)
        self.timeline.bind("<B1-Motion>", self._on_timeline_drag)
        self.timeline.bind("<ButtonRelease-1>", self._on_timeline_release)
        self.timeline.bind("<Double-Button-1>", self._on_timeline_double)
        self.timeline.bind("<Button-3>", self._on_timeline_right)

    def _build_right(self):
        tk.Label(self.right, text="Hierarchy", bg=PANEL, fg=TEXT, anchor="w",
                 font=("TkDefaultFont", 9, "bold")).pack(fill="x", padx=6,
                                                         pady=(8, 2))
        self.tree = ttk.Treeview(self.right, height=7, show="tree")
        self.tree.pack(fill="x", padx=6)
        self.tree.bind("<<TreeviewSelect>>", self._on_tree_select)
        row = tk.Frame(self.right, bg=PANEL)
        row.pack(fill="x", padx=6, pady=3)
        tk.Button(row, text="parent to selected cell part",
                  command=self.set_parent).pack(side="left")
        tk.Button(row, text="unparent", command=self.clear_parent).pack(
            side="left", padx=4)

        tk.Label(self.right, text="Inspector", bg=PANEL, fg=TEXT, anchor="w",
                 font=("TkDefaultFont", 9, "bold")).pack(fill="x", padx=6,
                                                         pady=(10, 2))
        self.fields = {}
        grid = tk.Frame(self.right, bg=PANEL)
        grid.pack(fill="x", padx=6)
        for i, (key, label) in enumerate((
                ("off_x", "offset x"), ("off_y", "offset y"),
                ("pivot_x", "pivot x"), ("pivot_y", "pivot y"),
                ("priority", "priority"), ("alpha", "alpha 0-31"),
                ("pal_slot", "palette slot"))):
            tk.Label(grid, text=label, bg=PANEL, fg=TEXT, anchor="w",
                     font=("TkFixedFont", 8)).grid(row=i, column=0,
                                                   sticky="w")
            var = tk.StringVar()
            entry = tk.Entry(grid, textvariable=var, width=8, bg=BG, fg=TEXT,
                             insertbackground=TEXT, highlightthickness=0)
            entry.grid(row=i, column=1, sticky="w", pady=1)
            entry.bind("<Return>", lambda _e, k=key: self._apply_field(k))
            entry.bind("<FocusOut>", lambda _e, k=key: self._apply_field(k))
            self.fields[key] = var

        flags = tk.Frame(self.right, bg=PANEL)
        flags.pack(fill="x", padx=6, pady=4)
        self.flag_vars = {}
        for name, bit in (("h flip", C.P_HFLIP), ("v flip", C.P_VFLIP),
                          ("hidden", C.P_HIDDEN),
                          ("double size", C.P_DOUBLE_SIZE)):
            var = tk.IntVar()
            tk.Checkbutton(flags, text=name, variable=var, bg=PANEL, fg=TEXT,
                           selectcolor=BG, activebackground=PANEL,
                           highlightthickness=0,
                           command=lambda b=bit, v=var: self._apply_flag(b, v)
                           ).pack(anchor="w")
            self.flag_vars[bit] = var

        tk.Label(self.right, text="Preview", bg=PANEL, fg=TEXT, anchor="w",
                 font=("TkDefaultFont", 9, "bold")).pack(fill="x", padx=6,
                                                         pady=(10, 2))
        row = tk.Frame(self.right, bg=PANEL)
        row.pack(fill="x", padx=6)
        for text, value in (("3D quads", "quads"), ("hardware OBJ", "oam"),
                            ("billboard", "billboard")):
            tk.Radiobutton(row, text=text, value=value,
                           variable=self.backend, bg=PANEL, fg=TEXT,
                           selectcolor=BG, activebackground=PANEL,
                           highlightthickness=0,
                           command=self._refresh_preview).pack(anchor="w")

        self.preview = tk.Canvas(self.right, bg=BG, height=200,
                                 highlightthickness=0)
        self.preview.pack(fill="x", padx=6, pady=4)
        self.preview.bind("<Configure>", lambda _e: self._refresh_preview())

        self.yaw = tk.Scale(self.right, from_=-80, to=80, orient="horizontal",
                            bg=PANEL, fg=TEXT, highlightthickness=0,
                            troughcolor=BG, label="billboard yaw",
                            command=lambda _v: self._refresh_preview())
        self.yaw.pack(fill="x", padx=6)

        self.notes = tk.Label(self.right, bg=PANEL, fg=WARN, justify="left",
                              anchor="w", wraplength=300,
                              font=("TkFixedFont", 8))
        self.notes.pack(fill="x", padx=6, pady=6)

    # -- document ----------------------------------------------------------

    def mark_dirty(self, value=True):
        self.dirty = value
        name = os.path.basename(self.path) if self.path else "untitled"
        self.title("NEACell editor - %s%s" % (name, "*" if value else ""))

    def new_bank(self):
        if not self._confirm_discard():
            return
        self.path = None
        self.bank = C.new_bank()
        self.bank["atlases"].append(C.new_atlas("atlas", 128, 128))
        self.bank["cells"].append(C.new_cell([C.new_part()]))
        seq = C.new_sequence("seq0")
        seq["frames"].append(C.new_frame(0, 6))
        self.bank["sequences"].append(seq)
        self.sel_cell = self.sel_seq = 0
        self.sel_part = 0
        self.mark_dirty(False)
        self.refresh_all()

    def open_dialog(self):
        path = filedialog.askopenfilename(
            filetypes=[("NEACell bank", "*.neacell *.bin"), ("All", "*.*")])
        if path:
            self.open_path(path)

    def open_path(self, path):
        if not self._confirm_discard():
            return
        try:
            self.bank = C.load(path)
        except Exception as exc:
            messagebox.showerror("Open failed", str(exc))
            return
        self.path = path
        self.sel_cell = self.sel_seq = 0
        self.sel_part = 0 if self.bank["cells"] \
            and self.bank["cells"][0]["parts"] else -1
        self._load_sidecar()
        self.mark_dirty(False)
        self.refresh_all()

    def save(self):
        if self.path is None:
            return self.save_as()
        try:
            C.dump(self.bank, self.path)
        except Exception as exc:
            messagebox.showerror("Save failed", str(exc))
            return
        self._save_sidecar()
        self.mark_dirty(False)
        self._set_status("saved %s" % os.path.basename(self.path))

    def save_as(self):
        path = filedialog.asksaveasfilename(defaultextension=".neacell",
                                            filetypes=[("NEACell bank",
                                                        "*.neacell")])
        if not path:
            return
        self.path = path
        self.save()

    def _confirm_discard(self):
        if not self.dirty:
            return True
        answer = messagebox.askyesnocancel("Unsaved changes",
                                           "Save before continuing?")
        if answer is None:
            return False
        if answer:
            self.save()
        return True

    def _on_close(self):
        if self._confirm_discard():
            self.destroy()

    # -- sidecar -----------------------------------------------------------

    def _load_sidecar(self):
        data = editor_sidecar.load(self.path) or {}
        stored = data.get("image")
        if not stored:
            return
        resolved = editor_sidecar.resolve_path(self.path, stored)
        if resolved and os.path.exists(resolved):
            self._set_image(resolved)

    def _save_sidecar(self):
        if self.path is None or not getattr(self, "image_path", None):
            return
        editor_sidecar.save(self.path, {
            "image": editor_sidecar.store_path(self.path, self.image_path),
            "backend": self.backend.get(),
        })

    def import_image(self):
        path = filedialog.askopenfilename(
            filetypes=[("Image", "*.png *.bmp *.gif"), ("All", "*.*")])
        if path:
            self._set_image(path)
            self._save_sidecar()

    def _set_image(self, path):
        try:
            self.atlas_image = Image.open(path).convert("RGBA")
        except Exception as exc:
            messagebox.showerror("Image failed", str(exc))
            return
        self.image_path = path
        self.piece_cache.clear()
        # A mismatch means the wrong image was attached -- most likely the
        # source spritesheet rather than the packed atlas the part rectangles
        # index into. Say so and leave the bank alone: the image is a preview
        # convenience, and the atlas size is something the runtime depends on.
        if self.bank["atlases"]:
            a = self.bank["atlases"][0]
            if (a["width"], a["height"]) != self.atlas_image.size:
                self._set_status(
                    "the image is %dx%d but the bank's atlas is %dx%d, so "
                    "parts will preview from the wrong pixels"
                    % (self.atlas_image.size + (a["width"], a["height"])))
        self.refresh_all()

    def import_retail(self):
        ncer = filedialog.askopenfilename(title="NCER (cells)")
        if not ncer:
            return
        ncgr = filedialog.askopenfilename(title="NCGR (graphics)")
        nclr = filedialog.askopenfilename(title="NCLR (palette)")
        nanr = filedialog.askopenfilename(title="NANR (animation, optional)")
        if not (ncgr and nclr):
            return
        try:
            import cell_import as I
            bank, _plane, _pal, _gfx, _size = I.build_bank(
                I.read_ncer(I.read_file(ncer)),
                I.read_nanr(I.read_file(nanr)) if nanr else None,
                I.read_ncgr(I.read_file(ncgr)),
                I.read_nclr(I.read_file(nclr)),
                "atlas")
        except Exception as exc:
            messagebox.showerror("Import failed", str(exc))
            return
        self.bank = bank
        self.path = None
        self.sel_cell = self.sel_seq = 0
        self.sel_part = 0
        self.mark_dirty()
        self.refresh_all()
        self._set_status("imported %d cells and %d sequences; use "
                         "cell_import.py to also write the artwork"
                         % (len(bank["cells"]), len(bank["sequences"])))

    # -- helpers -----------------------------------------------------------

    def cell(self):
        cells = self.bank["cells"]
        if not cells:
            return None
        self.sel_cell = clamp(self.sel_cell, 0, len(cells) - 1)
        return cells[self.sel_cell]

    def part(self):
        cell = self.cell()
        if cell is None or not cell["parts"]:
            return None
        if not (0 <= self.sel_part < len(cell["parts"])):
            return None
        return cell["parts"][self.sel_part]

    def seq(self):
        seqs = self.bank["sequences"]
        if not seqs:
            return None
        self.sel_seq = clamp(self.sel_seq, 0, len(seqs) - 1)
        return seqs[self.sel_seq]

    def _set_status(self, text):
        self.status.config(text=" " + text)

    def piece(self, part, zoom=1):
        """The part's pixels, cut from the attached image."""
        if self.atlas_image is None:
            return None
        key = (part["src_x"], part["src_y"], part["src_w"], part["src_h"],
               zoom)
        if key in self.piece_cache:
            return self.piece_cache[key]
        box = (part["src_x"], part["src_y"],
               part["src_x"] + part["src_w"], part["src_y"] + part["src_h"])
        img = self.atlas_image.crop(box)
        if zoom != 1:
            img = img.resize((max(1, img.width * zoom),
                              max(1, img.height * zoom)), Image.NEAREST)
        self.piece_cache[key] = img
        return img

    # -- refresh -----------------------------------------------------------

    def refresh_all(self):
        self._refresh_lists()
        self._refresh_tree()
        self._refresh_inspector()
        self._refresh_compose()
        self._refresh_atlas()
        self._refresh_timeline()
        self._refresh_preview()
        self._refresh_sizes()

    def _refresh_lists(self):
        self.cell_list.delete(0, "end")
        for i, cell in enumerate(self.bank["cells"]):
            self.cell_list.insert("end", "%2d  %d parts" % (i,
                                                            len(cell["parts"])))
        if self.bank["cells"]:
            self.cell_list.selection_clear(0, "end")
            self.cell_list.selection_set(self.sel_cell)

        self.part_list.delete(0, "end")
        cell = self.cell()
        if cell:
            for i, p in enumerate(cell["parts"]):
                bad = "!" if p["obj_size"] == C.OBJ_SIZE_NONE else " "
                self.part_list.insert("end", "%2d %s %dx%d @%d,%d"
                                      % (i, bad, p["src_w"], p["src_h"],
                                         p["off_x"], p["off_y"]))
            if 0 <= self.sel_part < len(cell["parts"]):
                self.part_list.selection_clear(0, "end")
                self.part_list.selection_set(self.sel_part)

        self.seq_list.delete(0, "end")
        kinds = {C.KIND_CELL: "cell", C.KIND_RIG: "rig",
                 C.KIND_MULTI: "multi"}
        for i, s in enumerate(self.bank["sequences"]):
            self.seq_list.insert("end", "%2d %-8s %s %dt"
                                 % (i, s["name"][:8],
                                    kinds.get(s["kind"], "?"),
                                    C.sequence_ticks(s)))
        if self.bank["sequences"]:
            self.seq_list.selection_clear(0, "end")
            self.seq_list.selection_set(self.sel_seq)

    def _refresh_tree(self):
        self.tree.delete(*self.tree.get_children())
        cell = self.cell()
        if cell is None:
            return
        nodes = {}
        for i, p in enumerate(cell["parts"]):
            label = "%d  %dx%d" % (i, p["src_w"], p["src_h"])
            parent = p["parent"]
            at = nodes.get(parent, "") if 0 <= parent < i else ""
            nodes[i] = self.tree.insert(at, "end", iid=str(i), text=label,
                                        open=True)

    def _refresh_inspector(self):
        part = self.part()
        for key, var in self.fields.items():
            var.set("" if part is None else str(part[key]))
        for bit, var in self.flag_vars.items():
            var.set(0 if part is None else (1 if part["flags"] & bit else 0))

    def _refresh_sizes(self):
        raw = C.dumps(self.bank)
        budget = self.bank["budget"] or {}
        self.size_label.config(
            text="%d bytes\n%d cells  %d parts\n%d seqs\npeak %d OBJs, "
                 "%d affine" % (
                     len(raw), len(self.bank["cells"]),
                     sum(len(c["parts"]) for c in self.bank["cells"]),
                     len(self.bank["sequences"]),
                     sum(budget.get("max_objs", [0])),
                     budget.get("max_affine", 0)))

    # -- cell composer -----------------------------------------------------

    def _compose_origin(self):
        return (self.compose.winfo_width() // 2,
                self.compose.winfo_height() // 2)

    def _refresh_compose(self):
        cv = self.compose
        cv.delete("all")
        ox, oy = self._compose_origin()

        # A grid on eight-pixel steps, because eight-alignment is what the
        # hardware OBJ path needs and an author should be able to see it.
        step = 8 * CELL_ZOOM
        w, h = cv.winfo_width(), cv.winfo_height()
        for x in range(ox % step, w, step):
            cv.create_line(x, 0, x, h, fill=GRID)
        for y in range(oy % step, h, step):
            cv.create_line(0, y, w, y, fill=GRID)
        cv.create_line(ox, 0, ox, h, fill="#4a4a55")
        cv.create_line(0, oy, w, oy, fill="#4a4a55")

        cell = self.cell()
        if cell is None:
            return

        self._compose_photos = []
        for i, part in enumerate(cell["parts"]):
            x0 = ox + part["off_x"] * CELL_ZOOM
            y0 = oy + part["off_y"] * CELL_ZOOM
            x1 = x0 + part["src_w"] * CELL_ZOOM
            y1 = y0 + part["src_h"] * CELL_ZOOM

            img = self.piece(part, CELL_ZOOM)
            if img is not None:
                if part["flags"] & C.P_HFLIP:
                    img = img.transpose(Image.FLIP_LEFT_RIGHT)
                if part["flags"] & C.P_VFLIP:
                    img = img.transpose(Image.FLIP_TOP_BOTTOM)
                photo = ImageTk.PhotoImage(img)
                self._compose_photos.append(photo)
                cv.create_image(x0, y0, image=photo, anchor="nw")

            bad = part["obj_size"] == C.OBJ_SIZE_NONE
            outline = WARN if bad else (ACCENT if i == self.sel_part
                                        else "#55555f")
            cv.create_rectangle(x0, y0, x1, y1, outline=outline,
                                dash=(3, 2) if bad else None)

            if i == self.sel_part:
                px = ox + part["pivot_x"] * CELL_ZOOM
                py = oy + part["pivot_y"] * CELL_ZOOM
                cv.create_oval(px - 4, py - 4, px + 4, py + 4,
                               outline=ACCENT, width=2)

            label = "%d" % i
            if bad:
                label += " not an OBJ size"
            cv.create_text(x0 + 3, y0 + 3, text=label, anchor="nw",
                           fill=outline, font=("TkFixedFont", 8))

    def _on_compose_press(self, event):
        cell = self.cell()
        if cell is None:
            return
        ox, oy = self._compose_origin()
        cx = (event.x - ox) / CELL_ZOOM
        cy = (event.y - oy) / CELL_ZOOM

        # Topmost first, which is the last part, so a click picks what the eye
        # sees rather than what happens to be first in the list.
        for i in range(len(cell["parts"]) - 1, -1, -1):
            p = cell["parts"][i]
            if p["off_x"] <= cx <= p["off_x"] + p["src_w"] \
                    and p["off_y"] <= cy <= p["off_y"] + p["src_h"]:
                self.sel_part = i
                self._drag = ("part", cx - p["off_x"], cy - p["off_y"])
                break
        self.refresh_all()

    def _on_compose_drag(self, event):
        if self._drag is None or self._drag[0] != "part":
            return
        part = self.part()
        if part is None:
            return
        ox, oy = self._compose_origin()
        cx = (event.x - ox) / CELL_ZOOM
        cy = (event.y - oy) / CELL_ZOOM

        nx = int(round(cx - self._drag[1]))
        ny = int(round(cy - self._drag[2]))
        # Snap to eight unless shift is held: OBJ positions are free, but
        # keeping parts on the grid keeps their source rects aligned too.
        if not (event.state & 0x0001):
            nx = (nx + 4) // 8 * 8
            ny = (ny + 4) // 8 * 8

        dx, dy = nx - part["off_x"], ny - part["off_y"]
        part["off_x"], part["off_y"] = nx, ny
        part["pivot_x"] += dx
        part["pivot_y"] += dy
        self.mark_dirty()
        self._refresh_compose()
        self._refresh_inspector()

    def _on_compose_release(self, _event):
        if self._drag and self._drag[0] == "part":
            self.recompute()
        self._drag = None

    # -- atlas picker ------------------------------------------------------

    def _refresh_atlas(self):
        cv = self.atlas_canvas
        cv.delete("all")
        if self.atlas_image is None:
            cv.create_text(140, 40, text="File -> Import image\nto see the "
                                         "artwork", fill="#77777f",
                           justify="center")
            return

        img = self.atlas_image
        zoom = ATLAS_ZOOM
        while img.width * zoom > 270 and zoom > 1:
            zoom -= 1
        self._atlas_zoom = zoom
        shown = img.resize((img.width * zoom, img.height * zoom),
                           Image.NEAREST)
        self._atlas_photo = ImageTk.PhotoImage(shown)
        cv.create_image(4, 4, image=self._atlas_photo, anchor="nw")

        part = self.part()
        if part is not None:
            cv.create_rectangle(4 + part["src_x"] * zoom,
                                4 + part["src_y"] * zoom,
                                4 + (part["src_x"] + part["src_w"]) * zoom,
                                4 + (part["src_y"] + part["src_h"]) * zoom,
                                outline=ACCENT, width=2)

    def _on_atlas_click(self, event):
        """Move the selected part's source rect, snapped to the tile grid."""
        part = self.part()
        if part is None or self.atlas_image is None:
            return
        zoom = getattr(self, "_atlas_zoom", ATLAS_ZOOM)
        x = int((event.x - 4) / zoom) // 8 * 8
        y = int((event.y - 4) / zoom) // 8 * 8
        part["src_x"] = clamp(x, 0, max(0, self.atlas_image.width
                                        - part["src_w"]))
        part["src_y"] = clamp(y, 0, max(0, self.atlas_image.height
                                        - part["src_h"]))
        self.piece_cache.clear()
        self.mark_dirty()
        self.refresh_all()

    # -- timeline ----------------------------------------------------------
    #
    # A CELL sequence gets a strip of frames whose widths are their durations,
    # because that is the thing being edited. A RIG sequence gets one lane per
    # track with draggable keyframes. The split is the same one animmat_editor
    # makes: a value that steps is not readable plotted on a numeric axis, and
    # a duration is not readable as a dot.

    TL_LEFT = 90
    TL_ROW = 22

    def _tl_geom(self):
        seq = self.seq()
        ticks = C.sequence_ticks(seq) if seq else 1
        width = max(1, self.timeline.winfo_width() - self.TL_LEFT - 10)
        return ticks, width

    def _tl_x(self, tick):
        ticks, width = self._tl_geom()
        return self.TL_LEFT + int(tick * width / max(1, ticks))

    def _tl_tick(self, x):
        ticks, width = self._tl_geom()
        return clamp(int(round((x - self.TL_LEFT) * ticks / max(1, width))),
                     0, ticks)

    def _refresh_timeline(self):
        cv = self.timeline
        cv.delete("all")
        seq = self.seq()
        if seq is None:
            return

        h = cv.winfo_height()
        ticks, _w = self._tl_geom()

        for t in range(0, ticks + 1, max(1, ticks // 16)):
            x = self._tl_x(t)
            cv.create_line(x, 0, x, h, fill=GRID)
            cv.create_text(x + 2, h - 10, text=str(t), anchor="w",
                           fill="#77777f", font=("TkFixedFont", 7))

        if seq["kind"] == C.KIND_RIG:
            self._refresh_tracks(seq)
        else:
            self._refresh_frames(seq)

        x = self._tl_x(self.tick)
        cv.create_line(x, 0, x, h, fill=ACCENT, width=2)

    def _refresh_frames(self, seq):
        cv = self.timeline
        at = 0
        for i, f in enumerate(seq["frames"]):
            x0 = self._tl_x(at)
            at += max(1, f["duration"])
            x1 = self._tl_x(at)
            fill = "#3a4a5a" if i == self.sel_frame else "#2c3038"
            cv.create_rectangle(x0, 6, x1 - 1, 6 + self.TL_ROW, fill=fill,
                                outline=ACCENT if i == self.sel_frame
                                else "#44444e", tags=("frame", str(i)))
            cv.create_text(x0 + 4, 8, anchor="nw",
                           text="c%d  %dt" % (f["target"], f["duration"]),
                           fill=TEXT, font=("TkFixedFont", 8))
        cv.create_text(4, 8, anchor="nw", text="frames", fill="#77777f",
                       font=("TkFixedFont", 8))

    def _refresh_tracks(self, seq):
        cv = self.timeline
        for row, track in enumerate(seq["tracks"]):
            y = 6 + row * self.TL_ROW
            name = "p%d %s" % (track["part"],
                               C.CHANNEL_NAMES.get(track["channel"], "?"))
            cv.create_text(4, y + 4, anchor="nw", text=name[:14], fill=TEXT,
                           font=("TkFixedFont", 8))
            cv.create_line(self.TL_LEFT, y + self.TL_ROW // 2,
                           cv.winfo_width() - 8, y + self.TL_ROW // 2,
                           fill="#3a3a44")

            storage = track.get("storage", C.STORE_KEYS)
            if storage == C.STORE_CONST:
                cv.create_text(self.TL_LEFT + 6, y + 4, anchor="nw",
                               text="const %d" % track["value"],
                               fill="#77777f", font=("TkFixedFont", 8))
                continue
            if storage == C.STORE_BAKED:
                cv.create_text(self.TL_LEFT + 6, y + 4, anchor="nw",
                               text="baked, %d ticks"
                                    % len(track.get("values", [])),
                               fill="#77777f", font=("TkFixedFont", 8))
                continue

            for k, key in enumerate(track.get("keys", [])):
                x = self._tl_x(key[0])
                cy = y + self.TL_ROW // 2
                sel = self.sel_key == (row, k)
                cv.create_oval(x - 4, cy - 4, x + 4, cy + 4,
                               fill=ACCENT if sel else "#8899aa",
                               outline="", tags=("key", "%d:%d" % (row, k)))

    def _on_timeline_press(self, event):
        seq = self.seq()
        if seq is None:
            return
        if seq["kind"] == C.KIND_RIG:
            hit = self._hit_key(event.x, event.y)
            if hit:
                self.sel_key = hit
                self._drag = ("key", hit)
                self._refresh_timeline()
                return
        else:
            at = 0
            for i, f in enumerate(seq["frames"]):
                x0, x1 = self._tl_x(at), self._tl_x(at + max(1, f["duration"]))
                if x0 <= event.x <= x1 and 6 <= event.y <= 6 + self.TL_ROW:
                    self.sel_frame = i
                    self._drag = ("frame", i, event.x, f["duration"])
                    self._refresh_timeline()
                    return
                at += max(1, f["duration"])

        self.tick = self._tl_tick(event.x)
        self.playing = False
        self._refresh_timeline()
        self._refresh_preview()

    def _hit_key(self, x, y):
        seq = self.seq()
        for row, track in enumerate(seq["tracks"]):
            if track.get("storage", C.STORE_KEYS) != C.STORE_KEYS:
                continue
            cy = 6 + row * self.TL_ROW + self.TL_ROW // 2
            if abs(y - cy) > 7:
                continue
            for k, key in enumerate(track["keys"]):
                if abs(x - self._tl_x(key[0])) <= 6:
                    return (row, k)
        return None

    def _on_timeline_drag(self, event):
        if self._drag is None:
            return
        seq = self.seq()
        if self._drag[0] == "key":
            row, k = self._drag[1]
            track = seq["tracks"][row]
            track["keys"][k][0] = self._tl_tick(event.x)
            track["keys"].sort(key=lambda kk: kk[0])
            self.mark_dirty()
            self._refresh_timeline()
            self._refresh_preview()
        elif self._drag[0] == "frame":
            _kind, i, x0, base = self._drag
            ticks, width = self._tl_geom()
            delta = int(round((event.x - x0) * ticks / max(1, width)))
            seq["frames"][i]["duration"] = clamp(base + delta, 1, 600)
            self.mark_dirty()
            self._refresh_timeline()

    def _on_timeline_release(self, _event):
        self._drag = None
        self._refresh_lists()

    def _on_timeline_double(self, event):
        """Add a keyframe on a rig track, or retarget a frame's cell."""
        seq = self.seq()
        if seq is None:
            return
        if seq["kind"] == C.KIND_RIG:
            row = (event.y - 6) // self.TL_ROW
            if not (0 <= row < len(seq["tracks"])):
                return
            track = seq["tracks"][row]
            if track.get("storage", C.STORE_KEYS) != C.STORE_KEYS:
                return
            tick = self._tl_tick(event.x)
            value = C.evaluate_track(track, tick << 12)
            track["keys"].append([tick, value])
            track["keys"].sort(key=lambda kk: kk[0])
        else:
            if self.sel_frame < len(seq["frames"]):
                f = seq["frames"][self.sel_frame]
                f["target"] = (f["target"] + 1) % max(1,
                                                      len(self.bank["cells"]))
        self.mark_dirty()
        self.refresh_all()

    def _on_timeline_right(self, event):
        """Remove the keyframe under the cursor."""
        seq = self.seq()
        if seq is None or seq["kind"] != C.KIND_RIG:
            return
        hit = self._hit_key(event.x, event.y)
        if hit is None:
            return
        row, k = hit
        keys = seq["tracks"][row]["keys"]
        if len(keys) > 1:
            keys.pop(k)
            self.sel_key = None
            self.mark_dirty()
            self.refresh_all()

    # -- preview -----------------------------------------------------------

    def _refresh_preview(self):
        cv = self.preview
        cv.delete("all")
        seq = self.seq()
        if seq is None:
            return

        w = max(1, cv.winfo_width())
        h = max(1, cv.winfo_height())
        ox, oy = w // 2, h // 2 + 20

        backend = self.backend.get()
        yaw = self.yaw.get() if backend == "billboard" else 0
        # A yaw slider stands in for a camera: the billboard plane turns edge
        # on, so its horizontal extent is foreshortened. It is not a renderer,
        # it is the one thing about the billboard backend an author can get
        # wrong without noticing.
        squash = max(0.05, abs(__import__("math").cos(
            yaw * 3.14159265 / 180.0)))

        cv.create_line(0, oy, w, oy, fill=GRID)
        cv.create_line(ox, 0, ox, h, fill=GRID)

        poses = C.evaluate_sequence(self.bank, self.sel_seq, self.tick << 12)
        self._preview_photos = []
        dropped = []

        for entry in poses:
            if not entry["visible"]:
                continue
            cell = self.bank["cells"][entry["cell"]]
            part = cell["parts"][entry["src"]]

            if backend == "oam":
                if part["obj_size"] == C.OBJ_SIZE_NONE \
                        or part["flags"] & C.P_NO_OAM:
                    dropped.append("part %d has no OBJ size" % entry["part"])
                    continue

            m = entry["m"]
            sx = C.from_f32(m[0])
            sy = C.from_f32(m[3])
            shear = m[1] != 0 or m[2] != 0
            tx = C.from_f32(entry["tx"])
            ty = C.from_f32(entry["ty"])

            img = self.piece(part, PREVIEW_ZOOM)
            if img is None:
                continue
            tx *= PREVIEW_ZOOM
            ty *= PREVIEW_ZOOM
            if part["flags"] & C.P_HFLIP:
                img = img.transpose(Image.FLIP_LEFT_RIGHT)
            if part["flags"] & C.P_VFLIP:
                img = img.transpose(Image.FLIP_TOP_BOTTOM)

            if shear or abs(sx - 1) > 0.01 or abs(sy - 1) > 0.01:
                # Pillow's affine takes the inverse map, same as the hardware.
                det = m[0] * m[3] - m[1] * m[2]
                if det == 0:
                    continue
                a = C.from_f32(C.divf32(m[3], det))
                b = C.from_f32(C.divf32(-m[1], det))
                c = C.from_f32(C.divf32(-m[2], det))
                d = C.from_f32(C.divf32(m[0], det))
                size = (max(1, int(img.width * abs(sx) * 2)),
                        max(1, int(img.height * abs(sy) * 2)))
                img = img.transform(size, Image.AFFINE,
                                    (a, b, -size[0] / 4 * a - size[1] / 4 * b,
                                     c, d, -size[0] / 4 * c - size[1] / 4 * d),
                                    resample=Image.NEAREST)
                tx -= img.width / 4
                ty -= img.height / 4

            if backend == "billboard" and squash < 0.999:
                nw = max(1, int(img.width * squash))
                img = img.resize((nw, img.height), Image.NEAREST)
                tx *= squash

            if entry["alpha"] < 31 and backend != "oam":
                faded = img.copy()
                alpha = faded.getchannel("A").point(
                    lambda v, a=entry["alpha"]: v * a // 31)
                faded.putalpha(alpha)
                img = faded

            photo = ImageTk.PhotoImage(img)
            self._preview_photos.append(photo)
            cv.create_image(ox + tx, oy + ty, image=photo, anchor="nw")

        if backend == "oam" and dropped:
            self.notes.config(
                text="hardware OBJ would drop:\n  " + "\n  ".join(dropped[:4]))
        elif backend == "oam":
            self.notes.config(text="")

    def toggle_play(self):
        self.playing = not self.playing

    def _tick(self):
        if self.playing:
            seq = self.seq()
            if seq is not None:
                total = C.sequence_ticks(seq)
                mode = seq["mode"]
                limit = total * 2 if mode in (C.MODE_PINGPONG,
                                              C.MODE_PINGPONG_LOOP) else total
                self.tick = (self.tick + 1) % max(1, limit)
                self._refresh_timeline()
                self._refresh_preview()
        self.after(33, self._tick)

    # -- edit commands -----------------------------------------------------

    def _on_pick_cell(self, _event):
        sel = self.cell_list.curselection()
        if sel:
            self.sel_cell = sel[0]
            self.sel_part = 0 if self.bank["cells"][sel[0]]["parts"] else -1
            self.refresh_all()

    def _on_pick_part(self, _event):
        sel = self.part_list.curselection()
        if sel:
            self.sel_part = sel[0]
            self.refresh_all()

    def _on_pick_seq(self, _event):
        sel = self.seq_list.curselection()
        if sel:
            self.sel_seq = sel[0]
            self.sel_frame = 0
            self.sel_key = None
            self.tick = 0
            self.refresh_all()

    def _on_tree_select(self, _event):
        sel = self.tree.selection()
        if sel:
            self.sel_part = int(sel[0])
            self._refresh_inspector()
            self._refresh_compose()
            self._refresh_atlas()

    def add_cell(self):
        self.bank["cells"].append(C.new_cell([C.new_part()]))
        self.sel_cell = len(self.bank["cells"]) - 1
        self.sel_part = 0
        self.mark_dirty()
        self.recompute()

    def dup_cell(self):
        cell = self.cell()
        if cell is None:
            return
        import copy
        self.bank["cells"].insert(self.sel_cell + 1, copy.deepcopy(cell))
        self.sel_cell += 1
        self.mark_dirty()
        self.recompute()

    def del_cell(self):
        if len(self.bank["cells"]) <= 1:
            return
        # Frames pointing past the end would index nothing, so pull them back
        # rather than leaving the bank invalid.
        removed = self.sel_cell
        self.bank["cells"].pop(removed)
        for seq in self.bank["sequences"]:
            for f in seq["frames"]:
                if f["target"] > removed:
                    f["target"] -= 1
                elif f["target"] == removed:
                    f["target"] = 0
            if seq["cell"] > removed:
                seq["cell"] -= 1
        self.sel_cell = max(0, removed - 1)
        self.sel_part = 0
        self.mark_dirty()
        self.recompute()

    def add_part(self):
        cell = self.cell()
        if cell is None:
            return
        cell["parts"].append(C.new_part())
        self.sel_part = len(cell["parts"]) - 1
        self.mark_dirty()
        self.recompute()

    def del_part(self):
        cell = self.cell()
        part = self.part()
        if cell is None or part is None or len(cell["parts"]) <= 1:
            return
        removed = self.sel_part
        cell["parts"].pop(removed)
        for p in cell["parts"]:
            if p["parent"] == removed:
                p["parent"] = -1
            elif p["parent"] > removed:
                p["parent"] -= 1
        self.sel_part = max(0, removed - 1)
        self.mark_dirty()
        self.recompute()

    def move_part(self, delta):
        """Reorder a part, which is what changes what is on top.

        Parents have to stay before their children, so a move that would break
        that is refused rather than silently reparenting.
        """
        cell = self.cell()
        if cell is None or self.sel_part < 0:
            return
        i = self.sel_part
        j = i + delta
        if not (0 <= j < len(cell["parts"])):
            return

        parts = cell["parts"]
        parts[i], parts[j] = parts[j], parts[i]
        for k, p in enumerate(parts):
            if p["parent"] == i:
                p["parent"] = j
            elif p["parent"] == j:
                p["parent"] = i
        if any(p["parent"] >= k for k, p in enumerate(parts)):
            parts[i], parts[j] = parts[j], parts[i]
            self._set_status("that move would put a part before its parent")
            return

        self.sel_part = j
        self.mark_dirty()
        self.refresh_all()

    def set_parent(self):
        cell = self.cell()
        sel = self.tree.selection()
        if cell is None or self.sel_part < 0 or not sel:
            return
        child = int(sel[0])
        parent = self.sel_part
        if parent >= child:
            self._set_status("a parent has to come before its child; "
                             "reorder first")
            return
        cell["parts"][child]["parent"] = parent
        self.mark_dirty()
        self.refresh_all()

    def clear_parent(self):
        part = self.part()
        if part is None:
            return
        part["parent"] = -1
        self.mark_dirty()
        self.refresh_all()

    def add_seq(self):
        seq = C.new_sequence("seq%d" % len(self.bank["sequences"]))
        seq["frames"].append(C.new_frame(self.sel_cell, 6))
        self.bank["sequences"].append(seq)
        self.sel_seq = len(self.bank["sequences"]) - 1
        self.mark_dirty()
        self.recompute()

    def del_seq(self):
        if len(self.bank["sequences"]) <= 1:
            return
        self.bank["sequences"].pop(self.sel_seq)
        self.sel_seq = max(0, self.sel_seq - 1)
        self.mark_dirty()
        self.recompute()

    def add_frame(self):
        seq = self.seq()
        if seq is None or seq["kind"] == C.KIND_RIG:
            return
        seq["frames"].append(C.new_frame(self.sel_cell, 6))
        self.mark_dirty()
        self.recompute()

    def del_frame(self):
        seq = self.seq()
        if seq is None or len(seq["frames"]) <= 1:
            return
        seq["frames"].pop(min(self.sel_frame, len(seq["frames"]) - 1))
        self.sel_frame = 0
        self.mark_dirty()
        self.recompute()

    def add_track(self):
        seq = self.seq()
        if seq is None:
            return
        if seq["kind"] != C.KIND_RIG:
            seq["kind"] = C.KIND_RIG
            seq["cell"] = self.sel_cell
            seq["num_frames"] = max(8, C.sequence_ticks(seq))
            self._set_status("sequence turned into a rig on cell %d"
                             % self.sel_cell)
        part = max(0, self.sel_part)
        length = C.sequence_ticks(seq)
        seq["tracks"].append(C.new_track(
            part, C.CH_ROT, keys=[[0, 0], [length - 1, 0]]))
        self.mark_dirty()
        self.recompute()

    def del_track(self):
        seq = self.seq()
        if seq is None or not seq["tracks"]:
            return
        row = self.sel_key[0] if self.sel_key else len(seq["tracks"]) - 1
        seq["tracks"].pop(clamp(row, 0, len(seq["tracks"]) - 1))
        self.sel_key = None
        self.mark_dirty()
        self.recompute()

    def _apply_field(self, key):
        part = self.part()
        if part is None:
            return
        try:
            value = int(self.fields[key].get())
        except ValueError:
            self._refresh_inspector()
            return
        if part[key] == value:
            return
        part[key] = value
        self.mark_dirty()
        self.recompute()

    def _apply_flag(self, bit, var):
        part = self.part()
        if part is None:
            return
        if var.get():
            part["flags"] |= bit
        else:
            part["flags"] &= ~bit
        self.mark_dirty()
        self.refresh_all()

    def optimize(self):
        C.optimize(self.bank)
        self.mark_dirty()
        self.recompute()
        self._set_status("track storage re-picked")

    def flatten(self):
        """Drop every parent link, which is what a retail-compatible bank is."""
        for cell in self.bank["cells"]:
            for p in cell["parts"]:
                p["parent"] = -1
        self.mark_dirty()
        self.refresh_all()
        self._set_status("hierarchy flattened; the bank is retail-shaped again")

    def recompute(self):
        C.compute_bounds(self.bank)
        # Part sizes may have changed, so the OAM size class has to follow.
        for cell in self.bank["cells"]:
            for p in cell["parts"]:
                dims = (p["src_w"], p["src_h"])
                p["obj_size"] = C.OBJ_SIZE_BY_DIMS.get(dims, C.OBJ_SIZE_NONE)
                if p["obj_size"] == C.OBJ_SIZE_NONE:
                    p["flags"] |= C.P_NO_OAM
                else:
                    p["flags"] &= ~C.P_NO_OAM
        C.compute_budget(self.bank)
        self.refresh_all()

    def validate(self):
        problems = C.validate_oam(self.bank)
        if not problems:
            messagebox.showinfo("Validate", "Nothing here would stop this "
                                            "bank playing as hardware OBJ "
                                            "sprites.")
            return
        messagebox.showwarning("Validate", "\n".join(problems[:20]))


def main():
    ap = argparse.ArgumentParser(description="Edit a .neacell cell bank")
    ap.add_argument("file", nargs="?")
    args = ap.parse_args()
    Editor(args.file).mainloop()


if __name__ == "__main__":
    main()
