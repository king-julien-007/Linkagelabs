"""
linkage_studio_plus.py
Enhanced Linkage Studio with:
 - motor anchor selection (choose neighbor or world origin)
 - rotational motor constraint with anchor
 - FABRIK-based inverse kinematics for selected chain
 - export trace CSV and canvas PostScript
 - improved UI controls
 - 4-Bar preset generator with sliders and Update button
 - Scrollable left control panel (keeps Play visible)

Run:
    python linkage_studio_plus.py
"""

import tkinter as tk
from tkinter import ttk, filedialog, messagebox
import math, time, json, csv, os

# -----------------------------
# Data structures
# -----------------------------
class Point:
    def __init__(self, x, y):
        self.x = float(x); self.y = float(y)
        self.px = float(x); self.py = float(y)
        self.locked = False
        self.radius = 7
        self.selected = False
        self.is_motor = False
        self.motor_angle = 0.0  # radians
        self.motor_rps = 0.0    # rotations per second
        self.motor_anchor = None  # index of anchor point or "world"
        self.canvas_id = None

    def pos(self): return (self.x, self.y)

class Link:
    def __init__(self, a_idx, b_idx, rest_length=None):
        self.a = int(a_idx); self.b = int(b_idx)
        self.rest = rest_length
        self.canvas_id = None

# -----------------------------
# FABRIK helper for IK
# -----------------------------
def fabrik_chain_positions(points, links, indices, target, tolerance=1e-2, max_iters=50):
    """
    indices: list of point indices forming the chain in order [start,...,end]
    target: (x,y) target for end effector (indices[-1])
    Modifies points positions in-place to try to reach target while preserving distances.
    Returns True if reached within tolerance.
    """
    if len(indices) < 2:
        return False
    # build positions list
    pos = [(points[i].x, points[i].y) for i in indices]
    # distances
    dists = []
    total_len = 0.0
    for i in range(len(pos)-1):
        dx = pos[i+1][0] - pos[i][0]; dy = pos[i+1][1] - pos[i][1]
        di = math.hypot(dx, dy)
        dists.append(di)
        total_len += di
    target_dist = math.hypot(target[0] - pos[0][0], target[1] - pos[0][1])
    if target_dist > total_len:
        # target unreachable: stretch toward target
        for i in range(len(pos)-1):
            r = math.hypot(target[0] - pos[i][0], target[1] - pos[i][1])
            if r == 0: continue
            lam = dists[i] / r
            pos[i+1] = ( (1-lam)*pos[i][0] + lam*target[0], (1-lam)*pos[i][1] + lam*target[1] )
    else:
        b = pos[0]
        dif = math.hypot(pos[-1][0] - target[0], pos[-1][1] - target[1])
        it = 0
        while dif > tolerance and it < max_iters:
            # forward reaching
            pos[-1] = target
            for i in range(len(pos)-2, -1, -1):
                r = math.hypot(pos[i+1][0] - pos[i][0], pos[i+1][1] - pos[i][1])
                if r == 0: continue
                lam = dists[i] / r
                pos[i] = ( (1-lam)*pos[i+1][0] + lam*pos[i][0], (1-lam)*pos[i+1][1] + lam*pos[i][1] )
            # backward reaching
            pos[0] = b
            for i in range(len(pos)-1):
                r = math.hypot(pos[i+1][0] - pos[i][0], pos[i+1][1] - pos[i][1])
                if r == 0: continue
                lam = dists[i] / r
                pos[i+1] = ( (1-lam)*pos[i][0] + lam*pos[i+1][0], (1-lam)*pos[i][1] + lam*pos[i+1][1] )
            dif = math.hypot(pos[-1][0] - target[0], pos[-1][1] - target[1])
            it += 1
    # write back positions to points (skip locked points)
    for idx, (x,y) in zip(indices, pos):
        if not points[idx].locked:
            points[idx].x = x; points[idx].y = y
    return True

# -----------------------------
# Main App
# -----------------------------
class LinkageStudioPlus:
    def __init__(self, root):
        self.root = root
        root.title("Linkage Studio Plus")

        # layout: left scrollable controls + right canvas
        self.main_frame = ttk.Frame(root)
        self.main_frame.pack(fill='both', expand=True)

        # left: scrollable controls area
        self.left_container = ttk.Frame(self.main_frame)
        self.left_container.pack(side='left', fill='y')

        # canvas that will host the scrollable frame
        self.left_canvas = tk.Canvas(self.left_container, width=320, height=720, highlightthickness=0)
        self.left_scroll = ttk.Scrollbar(self.left_container, orient='vertical', command=self.left_canvas.yview)
        self.left_canvas.configure(yscrollcommand=self.left_scroll.set)

        self.left_scroll.pack(side='right', fill='y')
        self.left_canvas.pack(side='left', fill='y')

        # inner frame where controls live
        self.left_frame = ttk.Frame(self.left_canvas)
        self.left_frame_id = self.left_canvas.create_window((0,0), window=self.left_frame, anchor='nw')

        # bind resizing/scroll events
        self.left_frame.bind("<Configure>", self._on_left_frame_configure)
        self.left_canvas.bind("<Configure>", self._on_left_canvas_configure)
        # allow scrolling with mousewheel when cursor over left_container
        self.left_container.bind_all("<MouseWheel>", self._on_mousewheel)  # Windows
        self.left_container.bind_all("<Button-4>", self._on_mousewheel)    # Linux scroll up
        self.left_container.bind_all("<Button-5>", self._on_mousewheel)    # Linux scroll down

        # right: big drawing canvas
        self.canvas = tk.Canvas(self.main_frame, width=1100, height=720, bg='#08121a')
        self.canvas.pack(side='right', fill='both', expand=True)

        # model
        self.points = []
        self.links = []
        self.trace_points = []  # list of (x,y)

        # UI state
        self.mode = tk.StringVar(value="add")  # add, connect, select, motor, ik_target
        self.snap_grid = tk.BooleanVar(value=True)
        self.snap_points = tk.BooleanVar(value=True)
        self.grid_size = tk.IntVar(value=16)
        self.solver_iters = tk.IntVar(value=16)
        self.near_snap_radius = tk.IntVar(value=14)
        self.trace_enabled = tk.BooleanVar(value=True)
        self.coupler_link_index = tk.IntVar(value=0)
        self.coupler_fraction = tk.DoubleVar(value=0.5)

        # 4-bar preset defaults (initialized in build_controls as well)
        self.fb_scale_unit = 80.0

        # animation
        self.playing = False
        self.last_time = None

        # IK
        self.ik_active = False
        self.ik_chain = []   # list of indices
        self.ik_target = None

        # selection & drag
        self.connect_sel = []
        self.drag_idx = None
        self.hover_idx = None

        # build ui inside left_frame
        self.build_controls()

        # bottom fixed controls (always visible)
        self.bottom_bar = ttk.Frame(root, padding=4)
        self.bottom_bar.pack(side='bottom', fill='x')
        self._build_bottom_bar()

        # binds
        self.canvas.bind("<Button-1>", self.on_left_click)
        self.canvas.bind("<Button-3>", self.on_right_press)
        self.canvas.bind("<B3-Motion>", self.on_right_drag)
        self.canvas.bind("<ButtonRelease-3>", self.on_right_release)
        self.root.bind("<space>", lambda e: self.toggle_play())
        self.root.bind("<BackSpace>", lambda e: self.clear_all())
        self.root.bind("g", lambda e: self.lock_selected_as_ground())

        # tick
        self.redraw()
        self.root.after(16, self.tick)

    # -------------------------
    # scroll helpers
    # -------------------------
    def _on_left_frame_configure(self, event):
        # update scrollregion
        self.left_canvas.configure(scrollregion=self.left_canvas.bbox("all"))

    def _on_left_canvas_configure(self, event):
        # keep inner frame width in sync with canvas width
        canvas_width = event.width
        try:
            self.left_canvas.itemconfig(self.left_frame_id, width=canvas_width)
        except Exception:
            pass

    def _on_mousewheel(self, event):
        # determine if mouse is over left_container
        x, y = self.root.winfo_pointerx(), self.root.winfo_pointery()
        widget = self.root.winfo_containing(x, y)
        if widget is None:
            return
        # if mouse within left_container or left_canvas or left_frame, scroll
        if widget == self.left_container or widget == self.left_canvas or str(widget).startswith(str(self.left_frame)):
            if event.num == 4: delta = -1
            elif event.num == 5: delta = 1
            else:
                # Windows: event.delta positive when scroll up
                delta = -1 * int(event.delta/120)
            self.left_canvas.yview_scroll(delta, "units")

    # -------------------------
    # bottom bar (fixed)
    # -------------------------
    def _build_bottom_bar(self):
        btn_frame = ttk.Frame(self.bottom_bar)
        btn_frame.pack(side='left', padx=8)
        self.play_btn = ttk.Button(btn_frame, text="Play", command=self.toggle_play, width=10)
        self.play_btn.pack(side='left', padx=2)
        ttk.Button(btn_frame, text="Step", command=self.step_once, width=10).pack(side='left', padx=2)
        ttk.Button(btn_frame, text="Clear All", command=self.clear_all, width=10).pack(side='left', padx=8)

        io_frame = ttk.Frame(self.bottom_bar)
        io_frame.pack(side='right', padx=8)
        ttk.Button(io_frame, text="Save JSON", command=self.save_json, width=12).pack(side='left', padx=2)
        ttk.Button(io_frame, text="Load JSON", command=self.load_json, width=12).pack(side='left', padx=2)

    # -------------------------
    # UI building (controls go into self.left_frame)
    # -------------------------
    def build_controls(self):
        # put controls into self.left_frame (scrollable area)
        ttk.Label(self.left_frame, text="Mode").pack(anchor='w', pady=(6,0))
        for t,v in [('Add Point','add'),('Connect','connect'),('Select/Drag','select'),('Motor','motor'),('IK Target','ik_target')]:
            ttk.Radiobutton(self.left_frame, text=t, value=v, variable=self.mode).pack(anchor='w')

        ttk.Separator(self.left_frame).pack(fill='x', pady=6)
        ttk.Checkbutton(self.left_frame, text="Snap to grid", variable=self.snap_grid).pack(anchor='w')
        ttk.Checkbutton(self.left_frame, text="Snap to nearby points", variable=self.snap_points).pack(anchor='w')
        ttk.Label(self.left_frame, text="Grid size").pack(anchor='w')
        ttk.Spinbox(self.left_frame, from_=4, to=64, textvariable=self.grid_size, width=6).pack(anchor='w')
        ttk.Label(self.left_frame, text="Snap radius").pack(anchor='w')
        ttk.Spinbox(self.left_frame, from_=6, to=64, textvariable=self.near_snap_radius, width=6).pack(anchor='w')
        ttk.Label(self.left_frame, text="Solver iterations").pack(anchor='w')
        ttk.Spinbox(self.left_frame, from_=1, to=80, textvariable=self.solver_iters, width=6).pack(anchor='w')

        ttk.Separator(self.left_frame).pack(fill='x', pady=6)
        ttk.Label(self.left_frame, text="Motor Controls").pack(anchor='w')
        self.motor_speed = tk.DoubleVar(value=90.0)
        ttk.Label(self.left_frame, text="RPM (deg/sec)").pack(anchor='w')
        ttk.Spinbox(self.left_frame, from_=-360, to=360, increment=5, textvariable=self.motor_speed, width=8).pack(anchor='w')
        ttk.Button(self.left_frame, text="Toggle Motor on Selected", command=self.toggle_motor_on_selected).pack(fill='x', pady=4)
        # motor anchor dropdown
        ttk.Label(self.left_frame, text="Motor anchor (select motor point first)").pack(anchor='w')
        self.anchor_var = tk.StringVar(value="world")
        self.anchor_menu = ttk.Combobox(self.left_frame, textvariable=self.anchor_var, state="readonly")
        self.anchor_menu.pack(fill='x')

        ttk.Button(self.left_frame, text="Assign Anchor to Selected Motor", command=self.assign_anchor_to_selected).pack(fill='x', pady=4)

        ttk.Separator(self.left_frame).pack(fill='x', pady=6)
        # IK controls
        ttk.Label(self.left_frame, text="Inverse Kinematics (FABRIK)").pack(anchor='w')
        ttk.Button(self.left_frame, text="Set IK Chain (select two endpoints in Select mode)", command=self.prepare_ik_chain).pack(fill='x', pady=2)
        ttk.Button(self.left_frame, text="Place IK Target (IK Target mode)", command=lambda: None).pack(fill='x', pady=2)
        self.start_ik_btn = ttk.Button(self.left_frame, text="Start IK", command=self.start_ik)
        self.start_ik_btn.pack(fill='x', pady=2)
        ttk.Button(self.left_frame, text="Stop IK", command=self.stop_ik).pack(fill='x', pady=2)

        # ----------------- 4-BAR PRESET PANEL -----------------
        ttk.Separator(self.left_frame).pack(fill='x', pady=6)
        ttk.Label(self.left_frame, text="4-Bar Preset").pack(anchor='w')

        self.fb_a = tk.DoubleVar(value=1.2)
        self.fb_b = tk.DoubleVar(value=1.0)
        self.fb_c = tk.DoubleVar(value=1.3)
        self.fb_d = tk.DoubleVar(value=1.5)
        self.fb_scale_unit = 80.0  # pixels per unit length for visual sizing

        # sliders (convert units -> pixels inside generate)
        ttk.Label(self.left_frame, text="Link a (crank)").pack(anchor='w')
        ttk.Scale(self.left_frame, from_=0.3, to=3.0, variable=self.fb_a, orient='horizontal').pack(fill='x')
        ttk.Label(self.left_frame, text="Link b (coupler)").pack(anchor='w')
        ttk.Scale(self.left_frame, from_=0.3, to=3.0, variable=self.fb_b, orient='horizontal').pack(fill='x')
        ttk.Label(self.left_frame, text="Link c (rocker)").pack(anchor='w')
        ttk.Scale(self.left_frame, from_=0.3, to=3.0, variable=self.fb_c, orient='horizontal').pack(fill='x')
        ttk.Label(self.left_frame, text="Link d (ground)").pack(anchor='w')
        ttk.Scale(self.left_frame, from_=0.5, to=4.0, variable=self.fb_d, orient='horizontal').pack(fill='x')

        self.fb_grashof = tk.BooleanVar(value=True)
        ttk.Checkbutton(self.left_frame, text="Enforce Grashof (s + l <= p + q)", variable=self.fb_grashof).pack(anchor='w')

        ttk.Label(self.left_frame, text="Crank RPM").pack(anchor='w')
        self.fb_rpm = tk.DoubleVar(value=90.0)
        ttk.Spinbox(self.left_frame, from_=-360, to=360, textvariable=self.fb_rpm, width=8).pack(anchor='w')

        fb_btn_frame = ttk.Frame(self.left_frame)
        fb_btn_frame.pack(fill='x', pady=4)
        ttk.Button(fb_btn_frame, text="Generate 4-Bar", command=self.generate_4bar_from_sliders).pack(side='left', expand=True, fill='x')
        ttk.Button(fb_btn_frame, text="Update 4-Bar", command=self.update_4bar_from_sliders).pack(side='left', expand=True, fill='x')

        ttk.Separator(self.left_frame).pack(fill='x', pady=6)
        # trace & export
        ttk.Checkbutton(self.left_frame, text="Enable coupler trace", variable=self.trace_enabled).pack(anchor='w')
        ttk.Label(self.left_frame, text="Coupler link index").pack(anchor='w')
        self.link_index_spin = ttk.Spinbox(self.left_frame, from_=0, to=0, textvariable=self.coupler_link_index, width=6)
        self.link_index_spin.pack(anchor='w')
        ttk.Label(self.left_frame, text="Fraction along link (0..1)").pack(anchor='w')
        ttk.Spinbox(self.left_frame, from_=0.0, to=1.0, increment=0.05, textvariable=self.coupler_fraction, width=6).pack(anchor='w')

        ttk.Button(self.left_frame, text="Export Trace CSV", command=self.export_trace_csv).pack(fill='x', pady=2)
        ttk.Button(self.left_frame, text="Export Canvas (PostScript .ps)", command=self.export_canvas_ps).pack(fill='x', pady=2)

        ttk.Separator(self.left_frame).pack(fill='x', pady=6)
        ttk.Label(self.left_frame, text="Tip: Left click add/select, Right drag to move. 'g' toggles ground on selected.").pack(anchor='w', pady=(8,8))

    # -------------------------
    # helper snapping
    # -------------------------
    def screen_to_world(self, x,y): return float(x), float(y)
    def snap_to_grid(self,x,y):
        if not self.snap_grid.get(): return x,y
        g = float(self.grid_size.get())
        return round(x/g)*g, round(y/g)*g
    def snap_to_near_point(self,x,y):
        if not self.snap_points.get(): return x,y
        r = self.near_snap_radius.get()
        for i,p in enumerate(self.points):
            dx = p.x - x; dy = p.y - y
            if dx*dx + dy*dy <= r*r: return p.x, p.y
        return x,y
    def snap_all(self,x,y):
        x,y = self.snap_to_grid(x,y)
        x,y = self.snap_to_near_point(x,y)
        return x,y

    # -------------------------
    # events
    # -------------------------
    def on_left_click(self, event):
        x,y = self.screen_to_world(event.x, event.y)
        x,y = self.snap_all(x,y)
        mode = self.mode.get()
        if mode == "add":
            self.add_point(x,y)
        elif mode == "connect":
            idx = self.find_point_near(x,y)
            if idx is not None:
                if idx not in self.connect_sel: self.connect_sel.append(idx)
                if len(self.connect_sel) == 2:
                    self.create_link(self.connect_sel[0], self.connect_sel[1])
                    self.connect_sel = []
        elif mode == "select":
            idx = self.find_point_near(x,y)
            self.select_point(idx)
        elif mode == "motor":
            idx = self.find_point_near(x,y)
            if idx is not None:
                p = self.points[idx]
                p.is_motor = not p.is_motor
                if p.is_motor:
                    p.motor_rps = self.motor_speed.get() / 360.0
                    p.motor_anchor = None
                else:
                    p.motor_rps = 0.0
            self.refresh_anchor_menu()
            self.redraw()
        elif mode == "ik_target":
            # set IK target at the clicked location
            self.ik_target = (x,y)
            if self.ik_chain:
                fabrik_chain_positions(self.points, self.links, self.ik_chain, self.ik_target, tolerance=0.5, max_iters=30)
            self.redraw()

    def on_right_press(self, event):
        idx = self.find_point_near(event.x, event.y)
        if idx is not None:
            self.drag_idx = idx
            self.select_point(idx)

    def on_right_drag(self, event):
        if self.drag_idx is None: return
        x,y = self.screen_to_world(event.x, event.y)
        x,y = self.snap_all(x,y)
        p = self.points[self.drag_idx]
        if p.locked: return
        p.x, p.y = x, y
        # quick constraint solve while dragging
        self.solve_constraints(iterations=max(2, min(8, self.solver_iters.get()//2)))
        self.redraw()

    def on_right_release(self, event):
        self.drag_idx = None

    # -------------------------
    # model ops
    # -------------------------
    def add_point(self,x,y):
        p = Point(x,y)
        self.points.append(p)
        # auto connect to previous if add repeatedly
        if len(self.points) >= 2 and self.mode.get() == "add":
            self.create_link(len(self.points)-2, len(self.points)-1)
        self.redraw()

    def create_link(self,a_idx,b_idx):
        if a_idx == b_idx: return
        for l in self.links:
            if (l.a==a_idx and l.b==b_idx) or (l.a==b_idx and l.b==a_idx): return
        rest = math.hypot(self.points[a_idx].x - self.points[b_idx].x, self.points[a_idx].y - self.points[b_idx].y)
        l = Link(a_idx,b_idx,rest_length=rest)
        self.links.append(l)
        # update coupler link spin upper bound
        try:
            self.link_index_spin.config(to=max(0, len(self.links)-1))
        except Exception:
            pass
        self.redraw()

    def select_point(self, idx):
        for p in self.points: p.selected = False
        if idx is not None: self.points[idx].selected = True
        self.redraw()

    def find_point_near(self,x,y,radius=12):
        best=None; bestd=radius*radius
        for i,p in enumerate(self.points):
            dx=p.x-x; dy=p.y-y; d2=dx*dx+dy*dy
            if d2 <= bestd:
                bestd=d2; best=i
        return best

    def clear_all(self):
        self.points.clear(); self.links.clear(); self.trace_points.clear(); self.ik_chain=[]; self.ik_target=None
        self.playing=False;
        try:
            self.play_btn.config(text="Play")
        except Exception:
            pass
        self.redraw()

    # -------------------------
    # constraints solver (PBD)
    # -------------------------
    def solve_constraints(self, iterations=None):
        if iterations is None: iterations = self.solver_iters.get()
        if not self.links: return
        for it in range(iterations):
            for link in self.links:
                a = self.points[link.a]; b = self.points[link.b]
                dx = b.x - a.x; dy = b.y - a.y
                d = math.hypot(dx,dy)
                if d == 0: continue
                diff = (d - link.rest) / d
                if a.locked and b.locked: continue
                elif a.locked:
                    b.x -= dx * diff
                    b.y -= dy * diff
                elif b.locked:
                    a.x += dx * diff
                    a.y += dy * diff
                else:
                    a.x += 0.5 * dx * diff
                    a.y += 0.5 * dy * diff
                    b.x -= 0.5 * dx * diff
                    b.y -= 0.5 * dy * diff

    # -------------------------
    # motors
    # -------------------------
    def refresh_anchor_menu(self):
        # Build list of anchors for the selected motor point
        choices = ["world"]
        selected_idx = None
        for i,p in enumerate(self.points):
            if p.selected and p.is_motor:
                # collect neighbors
                for li,l in enumerate(self.links):
                    if l.a == i: choices.append(str(l.b))
                    if l.b == i: choices.append(str(l.a))
                selected_idx = i
                break
        # unique
        uniq = []
        for c in choices:
            if c not in uniq: uniq.append(c)
        self.anchor_menu['values'] = uniq
        if selected_idx is not None:
            p = self.points[selected_idx]
            if p.motor_anchor is None:
                self.anchor_var.set("world")
            else:
                self.anchor_var.set(str(p.motor_anchor))
        else:
            self.anchor_var.set("world")

    def assign_anchor_to_selected(self):
        val = self.anchor_var.get()
        for i,p in enumerate(self.points):
            if p.selected and p.is_motor:
                if val == "world":
                    p.motor_anchor = None
                else:
                    try:
                        p.motor_anchor = int(val)
                    except:
                        p.motor_anchor = None
        self.redraw()

    def toggle_motor_on_selected(self):
        for p in self.points:
            if p.selected:
                p.is_motor = not p.is_motor
                if p.is_motor:
                    p.motor_rps = self.motor_speed.get() / 360.0
                    p.motor_anchor = None
                else:
                    p.motor_rps = 0.0
        self.refresh_anchor_menu()
        self.redraw()

    def apply_motors(self, dt):
        for i,p in enumerate(self.points):
            if not p.is_motor: continue
            p.motor_rps = self.motor_speed.get() / 360.0
            # find anchor position
            if p.motor_anchor is None:
                ax,ay = 0.0,0.0
            else:
                if 0 <= p.motor_anchor < len(self.points):
                    anchor = self.points[p.motor_anchor]
                    ax,ay = anchor.x, anchor.y
                else:
                    ax,ay = 0.0,0.0
            # increment angle
            p.motor_angle += p.motor_rps * 2.0 * math.pi * dt
            # maintain radius
            dx = p.x - ax; dy = p.y - ay
            r = math.hypot(dx,dy)
            if r == 0: r = 1.0
            p.x = ax + r * math.cos(p.motor_angle)
            p.y = ay + r * math.sin(p.motor_angle)

    # -------------------------
    # IK
    # -------------------------
    def prepare_ik_chain(self):
        # user selects two endpoints in Select mode (selection stored as selected flags)
        sel = [i for i,p in enumerate(self.points) if p.selected]
        if len(sel) < 2:
            messagebox.showinfo("IK", "Select two points (endpoints) in Select mode first.")
            return
        # choose order: first selected -> start, second -> end
        self.ik_chain = sel[:2]
        # if more points between, try to find chain by walking graph shortest path
        start, end = self.ik_chain[0], self.ik_chain[1]
        path = self.find_path(start, end)
        if path is None:
            messagebox.showerror("IK", "No chain/path exists between selected endpoints.")
            self.ik_chain = []
            return
        self.ik_chain = path
        messagebox.showinfo("IK", f"IK chain set with {len(self.ik_chain)} joints. Now switch to 'IK Target' mode and click to set target.")
        self.redraw()

    def find_path(self, a, b):
        # BFS shortest path through link graph
        from collections import deque
        q = deque([a]); prev = {a: None}
        while q:
            cur = q.popleft()
            if cur == b:
                # build path
                path = []
                while cur is not None:
                    path.append(cur); cur = prev[cur]
                path.reverse()
                return path
            for l in self.links:
                nbr = None
                if l.a == cur: nbr = l.b
                elif l.b == cur: nbr = l.a
                if nbr is not None and nbr not in prev:
                    prev[nbr] = cur; q.append(nbr)
        return None

    def start_ik(self):
        if not self.ik_chain:
            messagebox.showinfo("IK", "Set an IK chain first (Select two endpoints then press 'Set IK Chain').")
            return
        self.ik_active = True
        self.playing = False
        try:
            self.play_btn.config(text="Play")
        except Exception:
            pass
        messagebox.showinfo("IK", "Now switch to IK Target mode and click on the canvas to set the target, or place it and the solver will run.")

    def stop_ik(self):
        self.ik_active = False

    # -------------------------
    # 4-bar helper methods
    # -------------------------
    def _grashof_ok(self, a,b,c,d):
        arr = sorted([a,b,c,d])
        s, l, p, q = arr[0], arr[3], arr[1], arr[2]
        return (s + l) <= (p + q) + 1e-9

    def generate_4bar_from_sliders(self):
        # clear existing
        self.points.clear()
        self.links.clear()
        self.trace_points.clear()
        # read slider lengths (units -> pixels)
        a = float(self.fb_a.get()) * self.fb_scale_unit
        b = float(self.fb_b.get()) * self.fb_scale_unit
        c = float(self.fb_c.get()) * self.fb_scale_unit
        d = float(self.fb_d.get()) * self.fb_scale_unit

        # optionally enforce grashof by small nudging
        if self.fb_grashof.get():
            attempt = 0
            while not self._grashof_ok(a,b,c,d) and attempt < 20:
                a *= 0.97; d *= 1.03
                attempt += 1

        # place ground A at (180, 360), ground D at (180 + d, 360)
        Ax, Ay = 180.0, 360.0
        Dx, Dy = Ax + d, Ay

        # compute input crank B at angle 0 initially (to the right)
        Bx = Ax + a
        By = Ay

        # find C by intersection of circle centered B radius b and circle centered D radius c
        dx = Dx - Bx; dy = Dy - By
        R = math.hypot(dx, dy)
        if R < 1e-6:
            R = 1e-6
        x = (b*b - c*c + R*R) / (2*R)
        y_sq = b*b - x*x
        if y_sq < 0:
            y = 0.0
        else:
            y = math.sqrt(y_sq)
        ex = (dx / R, dy / R)
        P = (Bx + x*ex[0], By + x*ex[1])
        perp = (-ex[1], ex[0])
        # elbow-up solution
        Cx = P[0] + y*perp[0]
        Cy = P[1] + y*perp[1]

        # create points A,B,C,D
        A_idx = len(self.points); self.points.append(Point(Ax, Ay))
        B_idx = len(self.points); self.points.append(Point(Bx, By))
        C_idx = len(self.points); self.points.append(Point(Cx, Cy))
        D_idx = len(self.points); self.points.append(Point(Dx, Dy))

        # ground A and D
        self.points[A_idx].locked = True
        self.points[D_idx].locked = True

        # create links A-B, B-C, C-D, D-A
        self.create_link(A_idx, B_idx)
        self.create_link(B_idx, C_idx)
        self.create_link(C_idx, D_idx)
        self.create_link(D_idx, A_idx)

        # attach motor to B with anchor A
        self.points[B_idx].is_motor = True
        self.points[B_idx].motor_rps = float(self.fb_rpm.get()) / 360.0
        self.points[B_idx].motor_anchor = A_idx
        # initialize motor angle relative to anchor
        dx = self.points[B_idx].x - self.points[A_idx].x; dy = self.points[B_idx].y - self.points[A_idx].y
        self.points[B_idx].motor_angle = math.atan2(dy, dx)

        self.redraw()

    def update_4bar_from_sliders(self):
        # Only updates geometry if first 4 points look like a 4-bar (A,B,C,D)
        if len(self.points) < 4 or len(self.links) < 4:
            return
        # read lengths
        a = float(self.fb_a.get()) * self.fb_scale_unit
        b = float(self.fb_b.get()) * self.fb_scale_unit
        c = float(self.fb_c.get()) * self.fb_scale_unit
        d = float(self.fb_d.get()) * self.fb_scale_unit

        # anchor A and D positions remain; reposition B and C by recomputing with same method
        A = self.points[0]; D = self.points[3]
        Ax,Ay = A.x, A.y
        Dx,Dy = Ax + d, Ay
        Bx = Ax + a; By = Ay
        dx = Dx - Bx; dy = Dy - By
        R = math.hypot(dx, dy)
        if R < 1e-6: R = 1e-6
        x = (b*b - c*c + R*R) / (2*R)
        y_sq = b*b - x*x
        y = math.sqrt(max(0.0, y_sq))
        ex = (dx / R, dy / R)
        P = (Bx + x*ex[0], By + x*ex[1])
        perp = (-ex[1], ex[0])
        Cx = P[0] + y*perp[0]; Cy = P[1] + y*perp[1]

        # set positions
        self.points[1].x, self.points[1].y = Bx, By
        self.points[2].x, self.points[2].y = Cx, Cy
        # re-calc link rest lengths to preserve specified values
        def set_rest_between(u,v,val):
            for link in self.links:
                if (link.a == u and link.b == v) or (link.a == v and link.b == u):
                    link.rest = val
                    return
        set_rest_between(0,1,a); set_rest_between(1,2,b); set_rest_between(2,3,c); set_rest_between(3,0,d)

        # update motor rpm if present
        for p in self.points:
            if p.is_motor:
                p.motor_rps = float(self.fb_rpm.get()) / 360.0

        # small solver to settle
        self.solve_constraints(iterations=8)
        self.redraw()

    # -------------------------
    # tick / play
    # -------------------------
    def tick(self):
        now = time.time()
        if self.playing:
            dt = 0.016 if self.last_time is None else min(0.05, now - self.last_time)
            self.tick_once(dt)
            self.last_time = now
        self.root.after(16, self.tick)

    def toggle_play(self):
        self.playing = not self.playing
        try:
            self.play_btn.config(text="Pause" if self.playing else "Play")
        except Exception:
            pass
        if self.playing:
            self.last_time = time.time()

    def step_once(self):
        self.tick_once(0.033)

    def tick_once(self, dt):
        # motors
        self.apply_motors(dt)
        # constraints
        self.solve_constraints()
        # IK
        if self.ik_active and self.ik_chain and self.ik_target:
            fabrik_chain_positions(self.points, self.links, self.ik_chain, self.ik_target, tolerance=0.5, max_iters=10)
            # after IK, enforce constraints globally
            self.solve_constraints(iterations=8)
        # trace
        if self.trace_enabled.get():
            cp = self.world_coupler_pos()
            if cp: self.trace_points.append(cp)
            if len(self.trace_points) > 3000: self.trace_points.pop(0)
        self.redraw()

    def world_coupler_pos(self):
        idx = self.coupler_link_index.get()
        if idx < 0 or idx >= len(self.links): return None
        link = self.links[idx]
        a = self.points[link.a]; b = self.points[link.b]
        t = max(0.0, min(1.0, self.coupler_fraction.get()))
        return (a.x + (b.x - a.x)*t, a.y + (b.y - a.y)*t)

    # -------------------------
    # save/load/export
    # -------------------------
    def save_json(self):
        data = {
            "points":[{"x":p.x,"y":p.y,"locked":p.locked,"is_motor":p.is_motor,"motor_rps":p.motor_rps,"motor_anchor":p.motor_anchor} for p in self.points],
            "links":[{"a":l.a,"b":l.b,"rest":l.rest} for l in self.links]
        }
        path = filedialog.asksaveasfilename(defaultextension=".json", filetypes=[("JSON","*.json")])
        if not path: return
        with open(path,"w") as f: json.dump(data,f,indent=2)
        messagebox.showinfo("Saved", f"Saved to {path}")

    def load_json(self):
        path = filedialog.askopenfilename(filetypes=[("JSON","*.json")])
        if not path: return
        with open(path,"r") as f: data = json.load(f)
        self.points=[]; self.links=[]
        for pd in data.get("points",[]):
            p=Point(pd["x"],pd["y"]); p.locked=pd.get("locked",False); p.is_motor=pd.get("is_motor",False)
            p.motor_rps=pd.get("motor_rps",0.0); p.motor_anchor=pd.get("motor_anchor",None)
            self.points.append(p)
        for ld in data.get("links",[]):
            l=Link(ld["a"],ld["b"],rest_length=ld.get("rest",None)); self.links.append(l)
        self.trace_points.clear()
        self.redraw()

    def export_trace_csv(self):
        if not self.trace_points:
            messagebox.showinfo("Export", "No trace points to export.")
            return
        path = filedialog.asksaveasfilename(defaultextension=".csv", filetypes=[("CSV","*.csv")])
        if not path: return
        with open(path,"w",newline="") as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(["x","y"])
            for x,y in self.trace_points: writer.writerow([x,y])
        messagebox.showinfo("Export", f"Trace exported to {path}")

    def export_canvas_ps(self):
        path = filedialog.asksaveasfilename(defaultextension=".ps", filetypes=[("PostScript","*.ps")])
        if not path: return
        # save canvas as postscript
        self.canvas.postscript(file=path)
        messagebox.showinfo("Export", f"Canvas exported as PostScript: {path}\nConvert to PNG with ImageMagick or open in Preview.")

    # -------------------------
    # drawing
    # -------------------------
    def redraw(self):
        self.canvas.delete("all")
        w = int(self.canvas.winfo_width()); h = int(self.canvas.winfo_height())
        # grid
        if self.snap_grid.get():
            g = self.grid_size.get()
            for gx in range(0, w, g): self.canvas.create_line(gx,0,gx,h, fill="#0f2430")
            for gy in range(0, h, g): self.canvas.create_line(0,gy,w,gy, fill="#0f2430")
        # links
        for i,l in enumerate(self.links):
            a=self.points[l.a]; b=self.points[l.b]
            self.canvas.create_line(a.x,a.y,b.x,b.y, fill="#33aaff", width=4)
            if i == self.coupler_link_index.get():
                mx,my = (a.x+b.x)/2,(a.y+b.y)/2
                self.canvas.create_oval(mx-6,my-6,mx+6,my+6, outline="#ffcc66")
        # trace
        if self.trace_enabled.get() and len(self.trace_points) > 1:
            coords = []
            for x,y in self.trace_points: coords.extend([x,y])
            self.canvas.create_line(coords, fill="#ff66cc", width=2, smooth=True)
        # points
        for i,p in enumerate(self.points):
            color = "#00ff99" if not p.selected else "#ffff66"
            if p.locked: color = "#ffaa00"
            self.canvas.create_oval(p.x-p.radius, p.y-p.radius, p.x+p.radius, p.y+p.radius, fill=color, outline="")
            if p.is_motor:
                self.canvas.create_text(p.x, p.y - p.radius - 10, text="M", fill="#ff6666", font=("Helvetica",10,"bold"))
            if p.locked:
                self.canvas.create_text(p.x, p.y + p.radius + 10, text="G", fill="#ffd27f", font=("Helvetica",9,"bold"))
        # IK target
        if self.ik_target:
            tx,ty = self.ik_target
            self.canvas.create_oval(tx-6,ty-6,tx+6,ty+6, outline="#ff3333", width=2)
        # HUD
        self.canvas.create_text(8,8, anchor='nw', text=f"Points:{len(self.points)} Links:{len(self.links)} Mode:{self.mode.get()}", fill="#cbd5e1")
        self.refresh_anchor_menu()

    # -------------------------
    # helpers
    # -------------------------
    def lock_selected_as_ground(self):
        for p in self.points:
            if p.selected: p.locked = not p.locked
        self.redraw()

# -----------------------------
# run
# -----------------------------
if __name__ == "__main__":
    root = tk.Tk()
    try:
        ttk.Style().theme_use('clam')
    except:
        pass
    app = LinkageStudioPlus(root)
    root.mainloop()
