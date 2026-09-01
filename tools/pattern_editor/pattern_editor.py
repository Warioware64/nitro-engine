#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
#
# Copyright (c) 2026 Warioware64
#
# Editor for .neaptn stylus pattern banks.
#
# The test pane runs pattern_format.recognize(), which is the same evaluator
# NEAPattern.c implements, and tests/pattern_eval pins the two together -- so
# the scores shown here are the scores the DS will produce. That is the whole
# point of the pane: "will this shape be told apart from that one?" is a
# question worth answering while authoring, not after a build.
#
#     python3 tools/pattern_editor/pattern_editor.py hero.neaptn
#
# Unlike the other editors this one needs no artwork and so has no sidecar: a
# bank contains its own prototypes, which are the only thing it draws.

import argparse
import os
import sys
import tkinter as tk
from tkinter import filedialog, messagebox, simpledialog, ttk

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import pattern_format as P    # noqa: E402
import pattern_import as I    # noqa: E402


BG = "#1e1e22"
PANEL = "#26262c"
GRID = "#33333c"
ACCENT = "#5aa9e6"
WARN = "#e6705a"
TEXT = "#d8d8de"

INK = "#f0f0f4"
REDUCED = "#7ce38b"
GHOST = "#4a4a58"

DRAW_ZOOM = 6

# A gesture drawn with a mouse arrives far denser than a stylus sampled at
# 60 Hz, so points closer together than this are dropped as they are drawn.
# Without it the resampler is handed a different kind of input than the DS
# ever sees, and the preview stops being representative.
MOUSE_MIN_STEP = 2


def clamp(v, lo, hi):
    return lo if v < lo else (hi if v > hi else v)


class StrokeCanvas(tk.Canvas):
    """A square drawing surface in a bank's coordinate space.

    Left button draws a stroke, and lifting it ends that stroke, so a
    multi-stroke shape is drawn the way it is on the DS. Right button erases
    the last stroke.
    """

    def __init__(self, parent, size, on_change, editable=True):
        self.zoom = DRAW_ZOOM
        self.size = size
        super().__init__(parent, width=size * self.zoom,
                         height=size * self.zoom, bg=PANEL,
                         highlightthickness=1, highlightbackground=GRID)
        self.strokes = []
        self.reduced = []
        self.ghost = []
        self.on_change = on_change
        self.editable = editable
        self._drawing = False

        if editable:
            self.bind("<Button-1>", self._on_down)
            self.bind("<B1-Motion>", self._on_move)
            self.bind("<ButtonRelease-1>", self._on_up)
            self.bind("<Button-3>", self._on_erase)

    def set_size(self, size):
        self.size = size
        self.configure(width=size * self.zoom, height=size * self.zoom)
        self.redraw()

    def set_strokes(self, strokes):
        self.strokes = [list(s) for s in strokes]
        self.redraw()

    def set_reduced(self, strokes):
        self.reduced = [list(s) for s in strokes]
        self.redraw()

    def set_ghost(self, strokes):
        """A second shape drawn faintly underneath, to compare against."""
        self.ghost = [list(s) for s in strokes]
        self.redraw()

    def clear(self):
        self.strokes = []
        self.reduced = []
        self.redraw()
        if self.on_change:
            self.on_change()

    def _to_model(self, event):
        x = clamp(int(self.canvasx(event.x)) // self.zoom, 0, self.size - 1)
        y = clamp(int(self.canvasy(event.y)) // self.zoom, 0, self.size - 1)
        return x, y

    def _on_down(self, event):
        self.strokes.append([self._to_model(event)])
        self._drawing = True
        self.redraw()

    def _on_move(self, event):
        if not self._drawing:
            return
        p = self._to_model(event)
        last = self.strokes[-1][-1]
        if abs(p[0] - last[0]) + abs(p[1] - last[1]) < MOUSE_MIN_STEP:
            return
        self.strokes[-1].append(p)
        self.redraw()

    def _on_up(self, event):
        self._drawing = False
        # A click without a drag is not a stroke; nothing can be scored from
        # a single point, and leaving it would break the stroke count.
        if self.strokes and len(self.strokes[-1]) < 2:
            self.strokes.pop()
        self.redraw()
        if self.on_change:
            self.on_change()

    def _on_erase(self, event):
        if self.strokes:
            self.strokes.pop()
        self.redraw()
        if self.on_change:
            self.on_change()

    def redraw(self):
        self.delete("all")
        z = self.zoom

        step = max(1, self.size // 8)
        for i in range(0, self.size + 1, step):
            self.create_line(i * z, 0, i * z, self.size * z, fill=GRID)
            self.create_line(0, i * z, self.size * z, i * z, fill=GRID)

        for strokes, colour, width in ((self.ghost, GHOST, 2),
                                       (self.strokes, INK, 2)):
            for stroke in strokes:
                if len(stroke) < 2:
                    continue
                pts = []
                for x, y in stroke:
                    pts.extend((x * z + z // 2, y * z + z // 2))
                self.create_line(*pts, fill=colour, width=width,
                                 capstyle=tk.ROUND, joinstyle=tk.ROUND)

        for stroke in self.reduced:
            for x, y in stroke:
                cx = x * z + z // 2
                cy = y * z + z // 2
                r = 3
                self.create_oval(cx - r, cy - r, cx + r, cy + r,
                                 outline=REDUCED, width=2)


class Editor(tk.Tk):
    def __init__(self, path=None):
        super().__init__()
        self.title("NEAPattern editor")
        self.geometry("1180x760")
        self.configure(bg=BG)

        self.path = None
        self.bank = None
        self.dirty = False
        self.selected = None

        self._build_menu()
        self._build_layout()

        if path:
            self.open_path(path)
        else:
            self.new_bank()

        self.protocol("WM_DELETE_WINDOW", self._on_close)

    # -- chrome ------------------------------------------------------------

    def _build_menu(self):
        menu = tk.Menu(self)

        m = tk.Menu(menu, tearoff=0)
        m.add_command(label="New", command=self.new_bank)
        m.add_command(label="Open...", command=self.open)
        m.add_command(label="Save", command=self.save)
        m.add_command(label="Save as...", command=self.save_as)
        m.add_separator()
        m.add_command(label="Import text...", command=self.import_text)
        m.add_command(label="Export text...", command=self.export_text)
        m.add_separator()
        m.add_command(label="Quit", command=self._on_close)
        menu.add_cascade(label="File", menu=m)

        m = tk.Menu(menu, tearoff=0)
        m.add_command(label="Add prototype from drawing",
                      command=self.add_prototype)
        m.add_command(label="Replace selected with drawing",
                      command=self.replace_prototype)
        m.add_command(label="Delete selected", command=self.delete_prototype)
        m.add_separator()
        m.add_command(label="Rename code...", command=self.rename_code)
        m.add_command(label="Set normalize size...",
                      command=self.set_normalize_size)
        m.add_separator()
        m.add_command(label="Validate", command=self.validate)
        m.add_command(label="Confusion report", command=self.confusion_report)
        menu.add_cascade(label="Bank", menu=m)

        self.config(menu=menu)

    def _label(self, parent, text):
        return tk.Label(parent, text=text, bg=PANEL, fg=TEXT,
                        font=("TkFixedFont", 9), anchor="w")

    def _build_layout(self):
        root = tk.Frame(self, bg=BG)
        root.pack(fill="both", expand=True, padx=6, pady=6)

        self._build_left(root)
        self._build_centre(root)
        self._build_right(root)

        self.status = tk.Label(self, text="", bg=BG, fg=TEXT, anchor="w",
                               font=("TkFixedFont", 9))
        self.status.pack(fill="x", padx=8, pady=(0, 4))

    def _build_left(self, root):
        panel = tk.Frame(root, bg=PANEL, width=280)
        panel.pack(side="left", fill="y", padx=(0, 6))
        panel.pack_propagate(False)

        self._label(panel, "Prototypes").pack(fill="x", padx=6, pady=(6, 2))

        self.tree = ttk.Treeview(panel, columns=("kind", "n"), height=22)
        self.tree.heading("#0", text="name")
        self.tree.heading("kind", text="kind")
        self.tree.heading("n", text="strokes")
        self.tree.column("#0", width=120)
        self.tree.column("kind", width=54, anchor="center")
        self.tree.column("n", width=60, anchor="center")
        self.tree.pack(fill="both", expand=True, padx=6, pady=2)
        self.tree.bind("<<TreeviewSelect>>", self._on_select)

        form = tk.Frame(panel, bg=PANEL)
        form.pack(fill="x", padx=6, pady=6)

        self.var_enabled = tk.BooleanVar(value=True)
        tk.Checkbutton(form, text="enabled", variable=self.var_enabled,
                       command=self._apply_fields, bg=PANEL, fg=TEXT,
                       selectcolor=GRID, activebackground=PANEL,
                       activeforeground=TEXT,
                       font=("TkFixedFont", 9)).grid(row=0, column=0,
                                                     columnspan=2, sticky="w")

        self.var_kind = tk.StringVar(value="1")
        self.var_correction = tk.StringVar(value="0")

        self._label(form, "kind").grid(row=1, column=0, sticky="w")
        tk.Entry(form, textvariable=self.var_kind, width=10, bg=BG, fg=TEXT,
                 insertbackground=TEXT).grid(row=1, column=1, sticky="w")
        self._label(form, "correction").grid(row=2, column=0, sticky="w")
        tk.Entry(form, textvariable=self.var_correction, width=10, bg=BG,
                 fg=TEXT, insertbackground=TEXT).grid(row=2, column=1,
                                                      sticky="w")
        tk.Button(form, text="apply", command=self._apply_fields, bg=GRID,
                  fg=TEXT, font=("TkFixedFont", 9)).grid(row=3, column=0,
                                                         columnspan=2,
                                                         sticky="we", pady=4)

    def _build_centre(self, root):
        panel = tk.Frame(root, bg=PANEL)
        panel.pack(side="left", fill="both", padx=(0, 6))

        self._label(panel, "Draw  (left draws a stroke, right undoes one)"
                    ).pack(fill="x", padx=6, pady=(6, 2))

        self.canvas = StrokeCanvas(panel, P.DEFAULT_NORMALIZE_SIZE,
                                   self._on_drawing_changed)
        self.canvas.pack(padx=6, pady=4)

        row = tk.Frame(panel, bg=PANEL)
        row.pack(fill="x", padx=6, pady=4)
        for text, cmd in (("clear", lambda: self.canvas.clear()),
                          ("add", self.add_prototype),
                          ("replace", self.replace_prototype),
                          ("recognise", self.run_test)):
            tk.Button(row, text=text, command=cmd, bg=GRID, fg=TEXT,
                      font=("TkFixedFont", 9)).pack(side="left", padx=2)

        self._label(panel, "Settings").pack(fill="x", padx=6, pady=(8, 2))

        form = tk.Frame(panel, bg=PANEL)
        form.pack(fill="x", padx=6, pady=2)

        self.var_algo = tk.StringVar(value="standard")
        self.var_method = tk.StringVar(value="recursive")
        self.var_threshold = tk.StringVar(value="2")
        self.var_mask = tk.StringVar(value="0xFFFFFFFF")

        self._label(form, "algorithm").grid(row=0, column=0, sticky="w")
        ttk.Combobox(form, textvariable=self.var_algo, width=12,
                     state="readonly",
                     values=["light", "standard", "fine"]).grid(row=0,
                                                                column=1)
        self._label(form, "resample").grid(row=1, column=0, sticky="w")
        ttk.Combobox(form, textvariable=self.var_method, width=12,
                     state="readonly",
                     values=["none", "distance", "angle",
                             "recursive"]).grid(row=1, column=1)
        self._label(form, "threshold").grid(row=2, column=0, sticky="w")
        tk.Entry(form, textvariable=self.var_threshold, width=14, bg=BG,
                 fg=TEXT, insertbackground=TEXT).grid(row=2, column=1)
        self._label(form, "kind mask").grid(row=3, column=0, sticky="w")
        tk.Entry(form, textvariable=self.var_mask, width=14, bg=BG, fg=TEXT,
                 insertbackground=TEXT).grid(row=3, column=1)

        for var in (self.var_algo, self.var_method, self.var_threshold,
                    self.var_mask):
            var.trace_add("write", lambda *a: self.run_test())

    def _build_right(self, root):
        panel = tk.Frame(root, bg=PANEL, width=320)
        panel.pack(side="left", fill="both", expand=True)
        panel.pack_propagate(False)

        self._label(panel, "Recognition  (what the DS would say)"
                    ).pack(fill="x", padx=6, pady=(6, 2))

        self.results = tk.Text(panel, height=14, bg=BG, fg=TEXT,
                               font=("TkFixedFont", 10),
                               insertbackground=TEXT, wrap="none")
        self.results.pack(fill="x", padx=6, pady=2)

        self._label(panel, "Selected prototype").pack(fill="x", padx=6,
                                                      pady=(8, 2))
        self.preview = StrokeCanvas(panel, P.DEFAULT_NORMALIZE_SIZE, None,
                                    editable=False)
        self.preview.pack(padx=6, pady=4)

    # -- bank state --------------------------------------------------------

    def _mark_dirty(self, dirty=True):
        self.dirty = dirty
        name = os.path.basename(self.path) if self.path else "untitled"
        self.title("NEAPattern editor -- %s%s" % (name, " *" if dirty else ""))

    def _set_status(self, text):
        self.status.config(text=text)

    def new_bank(self):
        if not self._confirm_discard():
            return
        self.bank = P.new_bank(P.DEFAULT_NORMALIZE_SIZE)
        self.path = None
        self.selected = None
        self.canvas.set_size(self.bank["normalize_size"])
        self.preview.set_size(self.bank["normalize_size"])
        self.canvas.clear()
        self._refresh_tree()
        self._mark_dirty(False)
        self._set_status("new bank")

    def open(self):
        if not self._confirm_discard():
            return
        path = filedialog.askopenfilename(
            filetypes=[("Pattern bank", "*.neaptn *.bin"), ("All", "*")])
        if path:
            self.open_path(path)

    def open_path(self, path):
        try:
            self.bank = P.read_bank(path)
        except (P.PatternError, OSError) as e:
            messagebox.showerror("Open failed", str(e))
            return
        self.path = path
        self.selected = None
        self.canvas.set_size(self.bank["normalize_size"])
        self.preview.set_size(self.bank["normalize_size"])
        self.canvas.clear()
        self._refresh_tree()
        self._mark_dirty(False)
        self._set_status("%d prototypes, %d names, space %d"
                         % (len(self.bank["entries"]), len(self.bank["names"]),
                            self.bank["normalize_size"]))

    def save(self):
        if self.path is None:
            return self.save_as()
        try:
            P.write_bank(self.bank, self.path)
        except (P.PatternError, OSError) as e:
            messagebox.showerror("Save failed", str(e))
            return
        self._mark_dirty(False)
        self._set_status("saved %s" % self.path)

    def save_as(self):
        path = filedialog.asksaveasfilename(
            defaultextension=".neaptn",
            filetypes=[("Pattern bank", "*.neaptn"), ("Binary", "*.bin")])
        if not path:
            return
        self.path = path
        self.save()

    def import_text(self):
        if not self._confirm_discard():
            return
        path = filedialog.askopenfilename(
            filetypes=[("Dictionary text", "*.txt"), ("All", "*")])
        if not path:
            return
        try:
            self.bank = I.load_text(path, self.bank["normalize_size"]
                                    if self.bank
                                    else P.DEFAULT_NORMALIZE_SIZE)
        except (P.PatternError, OSError) as e:
            messagebox.showerror("Import failed", str(e))
            return
        self.path = None
        self.selected = None
        self.canvas.set_size(self.bank["normalize_size"])
        self.preview.set_size(self.bank["normalize_size"])
        self._refresh_tree()
        self._mark_dirty()
        self._set_status("imported %d prototypes from %s"
                         % (len(self.bank["entries"]), os.path.basename(path)))

    def export_text(self):
        path = filedialog.asksaveasfilename(
            defaultextension=".txt", filetypes=[("Dictionary text", "*.txt")])
        if not path:
            return
        try:
            with open(path, "w", encoding="utf-8") as f:
                I.dump_text(self.bank, f)
        except OSError as e:
            messagebox.showerror("Export failed", str(e))
            return
        self._set_status("exported %s" % path)

    # -- prototypes --------------------------------------------------------

    def _refresh_tree(self):
        self.tree.delete(*self.tree.get_children())
        for i, entry in enumerate(self.bank["entries"]):
            name = P.code_name(self.bank, entry["code"])
            if name is None:
                name = "code %d" % entry["code"]
            if not entry["enabled"]:
                name += "  (off)"
            self.tree.insert("", "end", iid=str(i), text=name,
                             values=(entry["kind"], len(entry["strokes"])))
        if self.selected is not None and \
                str(self.selected) in self.tree.get_children():
            self.tree.selection_set(str(self.selected))

    def _on_select(self, _event=None):
        sel = self.tree.selection()
        if not sel:
            self.selected = None
            return
        self.selected = int(sel[0])
        entry = self.bank["entries"][self.selected]
        self.var_enabled.set(entry["enabled"])
        self.var_kind.set(str(entry["kind"]))
        self.var_correction.set(str(entry["correction"]))
        self.preview.set_strokes(entry["strokes"])
        # The selected prototype under the drawing, so a new one can be traced
        # over an existing one to see how far apart they are.
        self.canvas.set_ghost(entry["strokes"])

    def _apply_fields(self):
        if self.selected is None:
            return
        entry = self.bank["entries"][self.selected]
        try:
            kind = int(self.var_kind.get(), 0)
            correction = int(self.var_correction.get(), 0)
        except ValueError:
            messagebox.showwarning("Bad value",
                                   "kind and correction must be integers")
            return
        if not (-P.FX32_ONE < correction < P.FX32_ONE):
            messagebox.showwarning(
                "Bad correction",
                "correction is f32: it must be between %d and %d"
                % (-P.FX32_ONE + 1, P.FX32_ONE - 1))
            return
        entry["kind"] = kind
        entry["correction"] = correction
        entry["enabled"] = self.var_enabled.get()
        self._refresh_tree()
        self._mark_dirty()
        self.run_test()

    def _ask_name(self, title):
        return simpledialog.askstring(title, "Name (prototypes sharing a "
                                             "name share a code):", parent=self)

    def add_prototype(self):
        strokes = [s for s in self.canvas.strokes if len(s) >= 2]
        if not strokes:
            messagebox.showwarning("Nothing to add", "Draw a shape first.")
            return
        name = self._ask_name("Add prototype")
        if not name:
            return
        code = P.code_for_name(self.bank, name)
        try:
            index = P.add_entry(self.bank, strokes, code,
                                int(self.var_kind.get(), 0))
        except (P.PatternError, ValueError) as e:
            messagebox.showerror("Add failed", str(e))
            return
        self.selected = index
        self._refresh_tree()
        self._mark_dirty()
        self._set_status("added %s as entry %d" % (name, index))
        self.run_test()

    def replace_prototype(self):
        if self.selected is None:
            messagebox.showwarning("No selection", "Select a prototype first.")
            return
        strokes = [s for s in self.canvas.strokes if len(s) >= 2]
        if not strokes:
            messagebox.showwarning("Nothing to use", "Draw a shape first.")
            return
        entry = self.bank["entries"][self.selected]
        normalised = P.normalize(strokes, self.bank["normalize_size"])
        if not normalised:
            messagebox.showerror("Replace failed",
                                 "the drawing has nothing that can be scored")
            return
        entry["strokes"] = normalised
        entry["_pattern"] = None
        self._refresh_tree()
        self._on_select()
        self._mark_dirty()
        self.run_test()

    def delete_prototype(self):
        if self.selected is None:
            return
        P.remove_entry(self.bank, self.selected)
        self.selected = None
        self._refresh_tree()
        self._mark_dirty()
        self.run_test()

    def rename_code(self):
        if self.selected is None:
            messagebox.showwarning("No selection", "Select a prototype first.")
            return
        entry = self.bank["entries"][self.selected]
        name = self._ask_name("Rename code")
        if not name:
            return
        # Renaming the code renames it for every prototype sharing it, which
        # is the point of a code: it is the meaning, not the drawing.
        code = entry["code"]
        names = self.bank["names"]
        while len(names) <= code:
            names.append("code %d" % len(names))
        names[code] = name
        self._refresh_tree()
        self._mark_dirty()

    def set_normalize_size(self):
        size = simpledialog.askinteger(
            "Normalize size",
            "Coordinate space every prototype is stored in.\n"
            "Existing prototypes are rescaled.",
            parent=self, initialvalue=self.bank["normalize_size"],
            minvalue=8, maxvalue=512)
        if not size or size == self.bank["normalize_size"]:
            return

        old = self.bank["normalize_size"]
        for entry in self.bank["entries"]:
            scaled = I.rescale(entry["strokes"], old, size)
            entry["strokes"] = [s for s in scaled if len(s) >= 2]
            entry["_pattern"] = None
        self.bank["normalize_size"] = size

        self.canvas.set_size(size)
        self.preview.set_size(size)
        self._refresh_tree()
        self._on_select()
        self._mark_dirty()
        self.run_test()

    # -- checks ------------------------------------------------------------

    def validate(self):
        problems = P.validate(self.bank)
        if problems:
            messagebox.showwarning("Validation",
                                   "\n".join(problems[:20]))
        else:
            messagebox.showinfo("Validation", "The bank is good.")

    def confusion_report(self):
        """Score every prototype against every other one.

        A pair that scores highly against each other is a pair the DS will
        confuse, and it is far easier to see that here than to discover it by
        drawing on hardware.
        """
        algo, method, threshold, mask = self._settings()
        if algo is None:
            return

        P.ensure_features(self.bank)
        entries = self.bank["entries"]
        rows = []

        for i, entry in enumerate(entries):
            if entry["_pattern"] is None:
                rows.append((P.FX32_ONE + 1, i, None, "cannot be scored"))
                continue
            if not entry["enabled"] or not (entry["kind"] & mask):
                continue

            res = P.recognize_pattern(self.bank, entry["_pattern"], algo, mask,
                                      max_results=2)
            if not res:
                rows.append((P.FX32_ONE + 1, i, None, "matches nothing"))
            elif res[0]["entry"] != i:
                # A prototype that does not recognise as itself is a bank bug,
                # not a close call, so it sorts above every margin.
                rows.append((P.FX32_ONE + 1, i, res[0], "loses to"))
            elif len(res) > 1:
                # The runner up is the margin this prototype actually has.
                rows.append((res[1]["score"], i, res[1], "vs"))

        rows.sort(key=lambda r: -r[0])

        lines = ["algorithm %s, resample %s %d\n"
                 % (P.ALGO_NAMES[algo], P.RESAMPLE_NAMES[method], threshold)]
        if not rows:
            lines.append("nothing to compare")
        for score, i, other, note in rows[:20]:
            name = P.code_name(self.bank, entries[i]["code"]) or "?"
            if other is None:
                lines.append("%-10s %s" % (name, note))
            else:
                lines.append("%-10s %s %-10s %.3f"
                             % (name, note, other["name"] or "?",
                                other["score"] / 4096.0))
        messagebox.showinfo("Confusion report", "\n".join(lines))

    def _settings(self):
        algo = {v: k for k, v in P.ALGO_NAMES.items()}.get(self.var_algo.get())
        method = {v: k for k, v in
                  P.RESAMPLE_NAMES.items()}.get(self.var_method.get())
        try:
            threshold = int(self.var_threshold.get(), 0)
            mask = int(self.var_mask.get(), 0) & 0xFFFFFFFF
        except ValueError:
            return None, None, None, None
        return algo, method, threshold, mask

    def _on_drawing_changed(self):
        self.run_test()

    def run_test(self):
        if self.bank is None:
            return

        self.results.delete("1.0", "end")

        strokes = [s for s in self.canvas.strokes if len(s) >= 2]
        if not strokes:
            self.canvas.set_reduced([])
            self.results.insert("end", "draw a shape to test it\n")
            return

        algo, method, threshold, mask = self._settings()
        if algo is None or method is None:
            self.results.insert("end", "threshold and mask must be integers\n")
            return

        inp = P.make_input(strokes, self.bank["normalize_size"], method,
                           threshold)
        if inp is None:
            self.canvas.set_reduced([])
            self.results.insert("end",
                                "nothing here can be scored: every stroke\n"
                                "collapsed to fewer than two points\n")
            return

        reduced = [s["pts"] for s in inp["strokes"]]
        self.canvas.set_reduced(reduced)

        results = P.recognize_pattern(self.bank, inp, algo, mask,
                                      max_results=8)

        npts = sum(len(s) for s in reduced)
        self.results.insert("end", "%d strokes, %d points after resampling\n\n"
                                   % (len(reduced), npts))
        if not results:
            self.results.insert("end", "no match\n\n")
            self.results.insert(
                "end",
                "Every prototype with a different stroke\n"
                "count is skipped outright, so this is\n"
                "usually a stroke miscounted rather than\n"
                "a shape unrecognised.\n")
            return

        for i, r in enumerate(results):
            self.results.insert("end", "%d. %-10s %.3f   entry %d\n"
                                % (i + 1, r["name"] or "?", r["score"] / 4096.0,
                                   r["entry"]))

    # -- closing -----------------------------------------------------------

    def _confirm_discard(self):
        if not self.dirty:
            return True
        return messagebox.askokcancel("Unsaved changes",
                                      "Discard the unsaved changes?")

    def _on_close(self):
        if self._confirm_discard():
            self.destroy()


def main():
    ap = argparse.ArgumentParser(description="Edit a .neaptn pattern bank")
    ap.add_argument("file", nargs="?")
    args = ap.parse_args()
    Editor(args.file).mainloop()


if __name__ == "__main__":
    main()
