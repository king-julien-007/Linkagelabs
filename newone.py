import tkinter as tk
from tkinter import ttk
import math, time

class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.locked = False
        self.selected = False
        self.is_motor = False
        self.motor_angle = 0
        self.motor_rps = 0
        self.motor_anchor = None  # anchor point index
        self.radius = 7

class Link:
    def __init__(self, a, b, rest):
        self.a = a
        self.b = b
        self.rest = rest

class LinkageApp:
    def __init__(self, root):
        self.root = root
        root.title("Linkage Studio - Fixed Build")

        self.left = ttk.Frame(root, padding=10)
        self.left.pack(side="left", fill="y")

        self.canvas = tk.Canvas(root, width=900, height=600, bg="#0f1720")
        self.canvas.pack(side="right", fill="both", expand=True)

        self.points = []
        self.links = []
        self.trace = []

        self.mode = tk.StringVar(value="add")
        self.motor_rpm = tk.DoubleVar(value=90)
        self.solver_iters = tk.IntVar(value=20)

        self.playing = False
        self.last_t = None

        self.build_ui()

        # events
        self.canvas.bind("<Button-1>", self.left_click)
        self.canvas.bind("<Button-3>", self.right_press)
        self.canvas.bind("<B3-Motion>", self.right_drag)
        self.canvas.bind("<ButtonRelease-3>", self.right_release)
        root.bind("<space>", lambda e: self.toggle_play())
        root.bind("g", lambda e: self.toggle_ground())

        self.tick()
        self.redraw()

    # ---------------- UI ----------------
    def build_ui(self):
        ttk.Label(self.left, text="Mode").pack(anchor="w")
        for t,v in [("Add", "add"), ("Connect","connect"), ("Select","select"), ("Motor","motor")]:
            ttk.Radiobutton(self.left, text=t, value=v, variable=self.mode).pack(anchor="w")

        ttk.Label(self.left, text="Motor RPM").pack(anchor="w")
        ttk.Spinbox(self.left, from_=-360, to=360, textvariable=self.motor_rpm).pack(anchor="w")

        ttk.Button(self.left, text="Play / Pause", command=self.toggle_play).pack(fill="x", pady=6)
        ttk.Button(self.left, text="Clear", command=self.clear).pack(fill="x")

    # ---------------- Events ----------------
    def left_click(self, e):
        x, y = e.x, e.y
        mode = self.mode.get()

        if mode == "add":
            self.add_point(x, y)

        elif mode == "select":
            idx = self.find_point(x, y)
            self.select(idx)

        elif mode == "connect":
            idx = self.find_point(x, y)
            if idx is not None:
                if not hasattr(self, "temp_link"):
                    self.temp_link = []
                self.temp_link.append(idx)
                if len(self.temp_link) == 2:
                    self.add_link(self.temp_link[0], self.temp_link[1])
                    self.temp_link = []

        elif mode == "motor":
            idx = self.find_point(x, y)
            if idx is not None:
                p = self.points[idx]
                p.is_motor = not p.is_motor
                if p.is_motor:
                    p.motor_rps = self.motor_rpm.get() / 360
                    p.motor_anchor = 0  # default anchor
                else:
                    p.motor_rps = 0

        self.redraw()

    def right_press(self, e):
        idx = self.find_point(e.x, e.y)
        if idx is not None:
            self.drag_idx = idx
            self.select(idx)

    def right_drag(self, e):
        if self.drag_idx is None: return
        p = self.points[self.drag_idx]
        if not p.locked:
            p.x, p.y = e.x, e.y
            self.solve_constraints()
            self.redraw()

    def right_release(self, e):
        self.drag_idx = None

    # ---------------- Core ops ----------------
    def add_point(self, x, y):
        self.points.append(Point(x, y))
        if len(self.points) > 1:
            self.add_link(len(self.points)-2, len(self.points)-1)
        self.redraw()

    def add_link(self, a, b):
        pa = self.points[a]; pb = self.points[b]
        d = math.hypot(pb.x-pa.x, pb.y-pa.y)
        self.links.append(Link(a, b, d))

    def select(self, idx):
        for p in self.points: p.selected = False
        if idx is not None:
            self.points[idx].selected = True

    def find_point(self, x, y):
        for i,p in enumerate(self.points):
            if (p.x-x)**2 + (p.y-y)**2 <= 12**2:
                return i
        return None

    def toggle_ground(self):
        for p in self.points:
            if p.selected:
                p.locked = not p.locked
        self.redraw()

    def clear(self):
        self.points.clear()
        self.links.clear()
        self.trace.clear()
        self.redraw()

    # ---------------- Constraints ----------------
    def solve_constraints(self):
        for _ in range(self.solver_iters.get()):
            for l in self.links:
                a = self.points[l.a]; b = self.points[l.b]
                dx = b.x - a.x; dy = b.y - a.y
                d = math.hypot(dx, dy)
                if d == 0: continue
                diff = (d - l.rest) / d
                if not a.locked:
                    a.x += 0.5 * dx * diff
                    a.y += 0.5 * dy * diff
                if not b.locked:
                    b.x -= 0.5 * dx * diff
                    b.y -= 0.5 * dy * diff

    # ---------------- Motors ----------------
    def apply_motors(self, dt):
        for i,p in enumerate(self.points):
            if not p.is_motor: continue
            anchor = self.points[p.motor_anchor]
            dx = p.x - anchor.x
            dy = p.y - anchor.y
            r = math.hypot(dx, dy)
            p.motor_angle += p.motor_rps * 2 * math.pi * dt
            p.x = anchor.x + r * math.cos(p.motor_angle)
            p.y = anchor.y + r * math.sin(p.motor_angle)

    # ---------------- Tick ----------------
    def toggle_play(self):
        self.playing = not self.playing

    def tick(self):
        now = time.time()
        if self.playing:
            dt = 0.016 if self.last_t is None else min(0.05, now-self.last_t)
            self.apply_motors(dt)
            self.solve_constraints()
            self.trace.append(self.get_mid_link())
        self.last_t = now
        self.redraw()
        self.root.after(16, self.tick)

    def get_mid_link(self):
        if not self.links: return None
        l = self.links[0]
        a = self.points[l.a]; b = self.points[l.b]
        return ((a.x+b.x)/2, (a.y+b.y)/2)

    # ---------------- Drawing ----------------
    def redraw(self):
        self.canvas.delete("all")

        # trace
        if len(self.trace) > 2:
            pts = []
            for x,y in self.trace:
                if x is None: continue
                pts.extend([x,y])
            if pts:
                self.canvas.create_line(pts, fill="#ff66cc", width=2)

        # links
        for l in self.links:
            a=self.points[l.a]; b=self.points[l.b]
            self.canvas.create_line(a.x,a.y,b.x,b.y,fill="#33aaff",width=4)

        # points
        for p in self.points:
            col = "#ffff66" if p.selected else "#00ff99"
            if p.locked: col = "#ffaa00"
            self.canvas.create_oval(p.x-p.radius,p.y-p.radius,p.x+p.radius,p.y+p.radius,fill=col,outline="")
            if p.is_motor:
                self.canvas.create_text(p.x, p.y-14, text="M", fill="#ff4444")

        self.canvas.create_text(10,10,anchor="nw",text=f"Mode:{self.mode.get()}",fill="#ffffff")
