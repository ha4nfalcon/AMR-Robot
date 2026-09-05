"""Edge-AI ops-console dashboard (Tkinter — stdlib, Pi-friendly).

Dark/light realtime UI: header + KPI cards, live map, fleet cards with
battery bars and status pills, task board, visual legend, color-coded
event feed. No browser / cloud needed: `python main.py --policy smart`.

Flicker-free by design: the static map layer is drawn ONCE, dynamic canvas
items carry the "dyn" tag (only those are redrawn), and fleet/task widgets
are created once and updated in place — never destroyed per tick. The
LIGHT/DARK toggle rebuilds the chrome once (event log lines are kept).
"""
import tkinter as tk

THEMES = {
    "dark": {
        "BG": "#0b0e14", "CARD": "#141a26", "CARD2": "#1a2233",
        "TXT": "#e8edf4", "MUT": "#8b98ab", "ACC": "#00e5ff",
        "GOOD": "#2ecc71", "WARN": "#f5a623", "ALERT": "#ff4d5e",
        "GRID_FREE": "#1c2333", "GRID_LINE": "#2a3348", "GRID_WALL": "#05070c",
        "LOG_BG": "#05070c", "ROBOT_EDGE": "white", "CHOKE": "#8a6d1c",
        "PILL_BG": {"green": "#0e3a24", "amber": "#4a3205", "cyan": "#073b44",
                    "red": "#4a1116", "gray": "#2a3140"},
        "PILL_FG": {"green": "#2ecc71", "amber": "#f5a623", "cyan": "#00e5ff",
                    "red": "#ff4d5e", "gray": "#8b98ab"},
    },
    "light": {
        "BG": "#edf0f5", "CARD": "#ffffff", "CARD2": "#dfe6ef",
        "TXT": "#16202f", "MUT": "#5b6b7f", "ACC": "#0087a8",
        "GOOD": "#1e9e52", "WARN": "#b97a0c", "ALERT": "#d93645",
        "GRID_FREE": "#ffffff", "GRID_LINE": "#c3cddb", "GRID_WALL": "#33415a",
        "LOG_BG": "#ffffff", "ROBOT_EDGE": "#16202f", "CHOKE": "#d9a821",
        "PILL_BG": {"green": "#d7f0e0", "amber": "#fbeecd", "cyan": "#d2f0f7",
                    "red": "#fbdfe2", "gray": "#e2e8f0"},
        "PILL_FG": {"green": "#14703c", "amber": "#925e07", "cyan": "#006d88",
                    "red": "#b01e2c", "gray": "#5b6b7f"},
    },
}

# Active palette (rebound by set_theme; methods read these at call time).
BG = CARD = CARD2 = TXT = MUT = ACC = None
GOOD = WARN = ALERT = None
GRID_FREE = GRID_LINE = GRID_WALL = LOG_BG = ROBOT_EDGE = CHOKE = None
PILL_BG = PILL_FG = None


def set_theme(name):
    """Rebind the active palette globals. Must be called before UI build."""
    global BG, CARD, CARD2, TXT, MUT, ACC, GOOD, WARN, ALERT
    global GRID_FREE, GRID_LINE, GRID_WALL, LOG_BG, ROBOT_EDGE, CHOKE
    global PILL_BG, PILL_FG
    d = THEMES[name]
    BG, CARD, CARD2 = d["BG"], d["CARD"], d["CARD2"]
    TXT, MUT, ACC = d["TXT"], d["MUT"], d["ACC"]
    GOOD, WARN, ALERT = d["GOOD"], d["WARN"], d["ALERT"]
    GRID_FREE, GRID_LINE, GRID_WALL = d["GRID_FREE"], d["GRID_LINE"], d["GRID_WALL"]
    LOG_BG, ROBOT_EDGE, CHOKE = d["LOG_BG"], d["ROBOT_EDGE"], d["CHOKE"]
    PILL_BG, PILL_FG = d["PILL_BG"], d["PILL_FG"]


set_theme("dark")

ROBOT_COLORS = ["#e74c3c", "#3498db", "#2ecc71", "#f39c12", "#9b59b6"]

TITLE_FONT = ("Segoe UI", 18, "bold")
SUB_FONT = ("Segoe UI", 10)
KPI_LABEL_FONT = ("Segoe UI", 9)
KPI_VALUE_FONT = ("Consolas", 26, "bold")
PANEL_TITLE_FONT = ("Segoe UI", 10, "bold")
BODY_FONT = ("Segoe UI", 9)
MONO_FONT = ("Consolas", 9)


def robot_status(r, docks):
    """Status pill for a robot -> (label, color). Pure: unit-testable."""
    if getattr(r, "dead", False):
        return ("DEAD", "red")
    if r.charging:
        return ("CHARGING", "cyan") if r.pos in docks else ("TO DOCK", "cyan")
    if getattr(r, "no_route", False) and r.task_id is not None:
        return ("NO ROUTE", "red")
    if getattr(r, "queued_charge", False):
        return ("DOCK QUEUE", "amber")
    if getattr(r, "topup_after_pickup", False):
        return ("TOP-UP", "amber")
    if r.task_id is None:
        return ("IDLE", "gray")
    if r.phase == "pickup":
        return ("FETCHING", "amber")
    return ("CARRYING", "green")


def log_tag(line):
    """Event-feed color tag for a log line. Pure: unit-testable."""
    u = line.upper()
    if "DEADLOCK" in u or "EMERGENCY" in u or "FAILED" in u or "DIED" in u:
        return "alert"
    if "NO ROUTE" in u or "BLACKOUT" in u:
        return "warn"
    if "PICKED UP" in u or "CHARGED" in u or "DELIVER" in u or "COMPLETE" in u:
        return "ok"
    if "OBSTACLE" in u or "BLOCKED" in u or "RE-ROUT" in u or "REROUTE" in u \
            or "DECONFLICT" in u or "SPAWN" in u:
        return "info"
    return "dim"


class Dashboard:
    def __init__(self, sim, cell=30):
        self.sim = sim
        self.policy = sim.robots[0].policy if sim.robots else "smart"
        self.cell = cell
        self.theme = "dark"
        self._running = False
        self._speed_ms = 150
        self._fleet_refs = []   # per-robot widget refs, updated in place
        self._task_refs = []    # per-task widget refs, updated in place

        self.root = tk.Tk()
        self.root.title("Edge-AI Distributed AMR Fleet Coordination")
        self.root.minsize(1240, 860)
        self._build_ui()

    def _build_ui(self):
        """Build (or rebuild, on theme toggle) all widgets from the palette."""
        self.root.configure(bg=BG)

        # ---- header ----
        tk.Frame(self.root, bg=ACC, height=3).pack(fill=tk.X)  # accent strip
        head = tk.Frame(self.root, bg=BG)
        head.pack(fill=tk.X, padx=16, pady=(10, 4))
        tk.Label(head, text="EDGE-AI DISTRIBUTED AMR FLEET COORDINATION",
                 font=TITLE_FONT, fg=TXT, bg=BG).pack(side=tk.LEFT)
        self.live = tk.Label(head, text="LIVE", font=("Consolas", 11, "bold"),
                             fg=GOOD, bg=BG)
        self.live.pack(side=tk.RIGHT, padx=(10, 0))
        btn = dict(relief=tk.FLAT, cursor="hand2",
                   activebackground=CARD, activeforeground=TXT)
        tk.Button(head, text="LIGHT" if self.theme == "dark" else "DARK",
                  width=7, command=self.toggle_theme,
                  bg=CARD2, fg=TXT, font=BODY_FONT, **btn).pack(
                      side=tk.RIGHT, padx=2)
        tk.Button(head, text="RESET", width=8, command=self.do_reset,
                  bg=CARD2, fg=TXT, font=BODY_FONT, **btn).pack(side=tk.RIGHT, padx=2)
        tk.Button(head, text="STEP", width=8, command=self.do_step,
                  bg=CARD2, fg=TXT, font=BODY_FONT, **btn).pack(side=tk.RIGHT, padx=2)
        self.run_btn = tk.Button(head, text="RUN", width=8, command=self.do_run,
                                 bg=ACC, fg="#04121a", font=("Segoe UI", 10, "bold"),
                                 relief=tk.FLAT, cursor="hand2",
                                 activebackground=CARD, activeforeground=TXT)
        self.run_btn.pack(side=tk.RIGHT, padx=2)
        tk.Label(self.root,
                 text="Decentralized multi-agent coordination for autonomous mobile "
                      "robots in smart warehouses  ·  peer-to-peer  ·  no cloud",
                 font=SUB_FONT, fg=MUT, bg=BG).pack(anchor=tk.W, padx=18)

        # ---- KPI cards ----
        kpis = tk.Frame(self.root, bg=BG)
        kpis.pack(fill=tk.X, padx=16, pady=10)
        self.kpi_vals = {}
        for shown, key in (("M O D E", "MODE"), ("S T E P S", "STEPS"),
                           ("T A S K S", "TASKS"),
                           ("C O L L I S I O N S", "COLLISIONS"),
                           ("R E P L A N S / D E A D L O C K S",
                            "REPLANS/DEADLOCKS")):
            card = tk.Frame(kpis, bg=CARD, padx=14, pady=8)
            card.pack(side=tk.LEFT, expand=True, fill=tk.X, padx=4)
            tk.Label(card, text=shown, font=KPI_LABEL_FONT, fg=MUT, bg=CARD).pack(anchor=tk.W)
            val = tk.Label(card, text="-", font=KPI_VALUE_FONT, fg=TXT, bg=CARD)
            val.pack(anchor=tk.W)
            self.kpi_vals[key] = val

        # ---- body ----
        body = tk.Frame(self.root, bg=BG)
        body.pack(fill=tk.BOTH, expand=True, padx=16, pady=4)
        leftcol = tk.Frame(body, bg=BG)
        leftcol.pack(side=tk.LEFT)
        self._legend_bar(leftcol)
        W, H = self.sim.wmap.width, self.sim.wmap.height
        map_bg = "#05070c" if self.theme == "dark" else GRID_LINE
        mapframe = tk.Frame(leftcol, bg=CARD, padx=1, pady=1)  # hairline border
        mapframe.pack()
        self.canvas = tk.Canvas(mapframe, width=W * self.cell, height=H * self.cell,
                                bg=map_bg, highlightthickness=0)
        self.canvas.pack()

        side = tk.Frame(body, bg=BG, width=400)
        side.pack(side=tk.RIGHT, fill=tk.Y, padx=(12, 0))
        side.pack_propagate(False)

        self.fleet_box = self._panel(side, "FLEET")
        self.task_box = self._panel(side, "TASKS")
        log_panel = self._panel(side, "EVENT FEED")
        logscroll = tk.Scrollbar(log_panel, orient=tk.VERTICAL,
                                 bg=CARD2, troughcolor=CARD,
                                 activebackground=ACC, width=12,
                                 relief=tk.FLAT, borderwidth=0,
                                 highlightthickness=0)
        logscroll.pack(side=tk.RIGHT, fill=tk.Y, padx=(0, 8), pady=(0, 8))
        self.logbox = tk.Text(log_panel, width=52, height=18, font=MONO_FONT,
                              bg=LOG_BG, fg=TXT, insertbackground=TXT,
                              highlightthickness=0, relief=tk.FLAT,
                              wrap="word",
                              yscrollcommand=logscroll.set)
        self.logbox.pack(side=tk.LEFT, fill=tk.BOTH, expand=True,
                         padx=(8, 0), pady=(0, 8))
        logscroll.config(command=self.logbox.yview)
        for tag, fg in (("alert", ALERT), ("warn", WARN), ("ok", GOOD),
                        ("info", ACC), ("dim", MUT)):
            self.logbox.tag_config(tag, foreground=fg)

        # ---- controls ----
        ctrl = tk.Frame(self.root, bg=BG)
        ctrl.pack(fill=tk.X, padx=16, pady=10)
        tk.Label(ctrl, text="speed (ms)", font=BODY_FONT, fg=MUT, bg=BG).pack(side=tk.LEFT, padx=(4, 4))
        self.speed = tk.Scale(ctrl, from_=50, to=600, resolution=25, orient=tk.HORIZONTAL,
                              length=160, bg=BG, fg=MUT, troughcolor=CARD2,
                              highlightthickness=0, command=self._set_speed)
        self.speed.set(self._speed_ms)
        self.speed.pack(side=tk.LEFT)
        self.mode_badge = tk.Label(ctrl, text="", font=("Consolas", 10, "bold"),
                                   fg=ACC, bg=BG)
        self.mode_badge.pack(side=tk.RIGHT)

        # static layer once + widget rows once; per-tick updates mutate in place
        self._draw_static()
        self._build_fleet()
        self._build_tasks()

    def toggle_theme(self):
        """Flip palette and rebuild chrome once; event log lines are kept."""
        lines = self.logbox.get("1.0", tk.END).splitlines()
        self._running = False
        self.theme = "light" if self.theme == "dark" else "dark"
        set_theme(self.theme)
        for w in self.root.winfo_children():
            w.destroy()
        self._build_ui()
        for line in lines:
            if line.strip():
                self._feed(line, log_tag(line))
        self.draw()

    # ---- small builders ----
    def _panel(self, parent, title):
        box = tk.Frame(parent, bg=CARD)
        box.pack(fill=tk.X, pady=4)
        tk.Label(box, text=" ".join(title), font=PANEL_TITLE_FONT, fg=MUT,
                 bg=CARD).pack(anchor=tk.W, padx=8, pady=(6, 2))
        return box

    def _legend_bar(self, parent):
        """Single horizontal legend strip that sits on top of the map."""
        bar = tk.Frame(parent, bg=CARD)
        bar.pack(fill=tk.X, pady=(0, 6))
        tk.Label(bar, text="L E G E N D", font=PANEL_TITLE_FONT, fg=MUT,
                 bg=CARD).pack(side=tk.LEFT, padx=(10, 6), pady=7)
        items = [
            ("#e74c3c", "P", "pickup"),
            (GOOD, "D", "drop-off"),
            ("#f1c40f", "T", "carrying"),
            (ACC, "CHG", "dock"),
            (CHOKE, "=", "choke"),
            (GRID_WALL, "#", "shelf"),
            ("#ff2fb3", "!", "spill"),
        ]
        for color, glyph, text in items:
            chip = tk.Label(bar, text=glyph, font=("Consolas", 9, "bold"),
                            bg=color, width=4,
                            fg="black" if color in ("#f1c40f", ACC) else "white")
            chip.pack(side=tk.LEFT, padx=(8, 2))
            tk.Label(bar, text=text, font=BODY_FONT, fg=TXT,
                     bg=CARD).pack(side=tk.LEFT, padx=(0, 2))

    def _set_speed(self, v):
        self._speed_ms = int(float(v))

    # ---- map layers ----
    def _draw_static(self):
        """Shelves, aisles, choke, dock — drawn ONCE (tagged 'static')."""
        c, m = self.cell, self.sim.wmap
        cv = self.canvas
        cv.delete("static")
        for y in range(m.height):
            for x in range(m.width):
                fill = GRID_FREE
                if m.grid[y][x] == 1:
                    fill = GRID_WALL
                if (x, y) in (m.choke_cells or []):
                    fill = CHOKE
                cv.create_rectangle(x * c, y * c, (x + 1) * c, (y + 1) * c,
                                    fill=fill, outline=GRID_LINE, tags=("static",))
        for dx0, dy0 in m.docks:
            cv.create_rectangle(dx0 * c + 2, dy0 * c + 2, (dx0 + 1) * c - 2, (dy0 + 1) * c - 2,
                                fill=ACC, outline=ROBOT_EDGE, width=1, tags=("static",))
            cv.create_text(dx0 * c + c / 2, dy0 * c + c / 2, text="CHG",
                           fill="#04121a" if self.theme == "dark" else "white",
                           font=("Consolas", 8, "bold"), tags=("static",))

    def _draw_dynamic(self):
        """Only ~20 moving items are redrawn per tick (tagged 'dyn')."""
        c, m, s = self.cell, self.sim.wmap, self.sim
        cv = self.canvas
        cv.delete("dyn")
        blocked = set()
        for r in s.robots:
            blocked |= r.blocked_known
        for (x, y) in blocked:
            cv.create_rectangle(x * c + 3, y * c + 3, (x + 1) * c - 3, (y + 1) * c - 3,
                                fill="#ff2fb3", outline="", tags=("dyn",))
        for t in s.tasks:
            tid = t["id"]
            if t.get("done"):
                # delivered: dimmed hollow marker with check stays on the map
                gx, gy = t["drop"]
                cv.create_oval(gx * c + 8, gy * c + 8, (gx + 1) * c - 8, (gy + 1) * c - 8,
                               fill="", outline="#1e5c38", width=2, tags=("dyn",))
                cv.create_text(gx * c + c / 2, gy * c + c / 2, text=f"✓D{tid}",
                               fill="#1e5c38", font=("Consolas", 8, "bold"), tags=("dyn",))
                continue
            if not t.get("picked"):
                px, py = t["pickup"]
                cv.create_rectangle(px * c + 8, py * c + 8, (px + 1) * c - 8, (py + 1) * c - 8,
                                    fill="#e74c3c", outline="#7b1a12", tags=("dyn",))
                cv.create_text(px * c + c / 2, py * c + c / 2, text=f"P{tid}",
                               fill="white", font=("Consolas", 8, "bold"), tags=("dyn",))
            gx, gy = t["drop"]
            cv.create_oval(gx * c + 8, gy * c + 8, (gx + 1) * c - 8, (gy + 1) * c - 8,
                           fill=GOOD, outline="#0e5a30", tags=("dyn",))
            cv.create_text(gx * c + c / 2, gy * c + c / 2, text=f"D{tid}",
                           fill="white", font=("Consolas", 8, "bold"), tags=("dyn",))
        for r in s.robots:
            if r.path:
                # current planned path: dotted line in the robot's own color
                col = ROBOT_COLORS[r.rid % len(ROBOT_COLORS)]
                xy = []
                for (x, y) in [r.pos] + list(r.path):
                    xy += [x * c + c / 2, y * c + c / 2]
                cv.create_line(*xy, fill=col, width=2, dash=(5, 3), tags=("dyn",))
        for r in s.robots:
            x, y = r.pos
            col = ROBOT_COLORS[r.rid % len(ROBOT_COLORS)]
            cv.create_oval(x * c + 3, y * c + 3, (x + 1) * c - 3, (y + 1) * c - 3,
                           fill=col, outline=ROBOT_EDGE, width=1, tags=("dyn",))
            cv.create_text(x * c + c / 2, y * c + c / 2, text=str(r.rid),
                           fill="white", font=("Consolas", 10, "bold"), tags=("dyn",))
            if r.task_id is not None and r.phase == "drop":
                cv.create_rectangle(x * c + 1, y * c - 12, x * c + 33, y * c - 2,
                                    fill="#f1c40f", outline="black", tags=("dyn",))
                cv.create_text(x * c + 17, y * c - 7, text=f"T{r.task_id}",
                               fill="black", font=("Consolas", 7, "bold"), tags=("dyn",))
            bw = (c - 6) * max(0.0, min(1.0, r.battery / 100.0))
            cv.create_rectangle(x * c + 3, (y + 1) * c - 6, x * c + 3 + bw, (y + 1) * c - 3,
                                fill=GOOD if r.battery > 30 else ALERT, outline="",
                                tags=("dyn",))

    # ---- fleet / task widgets: built once, mutated in place ----
    def _build_fleet(self):
        for w in self.fleet_box.winfo_children()[1:]:
            w.destroy()
        self._fleet_refs = []
        for r in self.sim.robots:
            row = tk.Frame(self.fleet_box, bg=CARD2)
            row.pack(fill=tk.X, padx=8, pady=2)
            top = tk.Frame(row, bg=CARD2)
            top.pack(fill=tk.X, padx=2, pady=(2, 0))
            dot = tk.Label(top, text="●", font=("Segoe UI", 12),
                           fg=ROBOT_COLORS[r.rid % len(ROBOT_COLORS)], bg=CARD2)
            dot.pack(side=tk.LEFT, padx=(4, 2))
            tk.Label(top, text=f"R{r.rid}", font=("Consolas", 11, "bold"),
                     fg=TXT, bg=CARD2).pack(side=tk.LEFT)
            pill = tk.Label(top, text="", font=("Consolas", 8, "bold"), padx=6,
                            width=10)
            pill.pack(side=tk.RIGHT, padx=4, pady=2)
            bottom = tk.Frame(row, bg=CARD2)
            bottom.pack(fill=tk.X, padx=2, pady=(0, 3))
            bar = tk.Canvas(bottom, width=110, height=10, bg=LOG_BG,
                            highlightthickness=0)
            bar.pack(side=tk.LEFT, padx=(4, 6))
            rect = bar.create_rectangle(0, 0, 0, 10, fill=GOOD, outline="")
            job = tk.Label(bottom, text="", font=MONO_FONT, fg=MUT, bg=CARD2,
                           anchor="w")
            job.pack(side=tk.LEFT, fill=tk.X, expand=True)
            self._fleet_refs.append({"pill": pill, "bar": bar, "rect": rect, "job": job})

    def _update_fleet(self):
        m = self.sim.wmap
        for r, refs in zip(self.sim.robots, self._fleet_refs):
            label, key = robot_status(r, m.docks)
            refs["pill"].config(text=label, fg=PILL_FG[key], bg=PILL_BG[key])
            w = 110 * max(0.0, min(1.0, r.battery / 100.0))
            refs["bar"].coords(refs["rect"], 0, 0, w, 10)
            refs["bar"].itemconfig(refs["rect"],
                                   fill=GOOD if r.battery > 30 else ALERT)
            job = "" if r.task_id is None else (
                f"T{r.task_id} {'pickup' if r.phase == 'pickup' else 'drop'} "
                f"{r.pos} → {r.goal}")
            refs["job"].config(text=f"{r.battery:.0f}%  {job}")

    def _build_tasks(self):
        for w in self.task_box.winfo_children()[1:]:
            w.destroy()
        self._task_refs = []
        for t in self.sim.tasks:
            row = tk.Frame(self.task_box, bg=CARD)
            row.pack(fill=tk.X, padx=8, pady=1)
            tk.Label(row, text=f"T{t['id']}", font=("Consolas", 9, "bold"),
                     fg=TXT, bg=CARD, width=3).pack(side=tk.LEFT)
            tk.Label(row, text="P", font=("Consolas", 8, "bold"),
                     bg="#e74c3c", fg="white", width=2).pack(side=tk.LEFT, padx=1)
            tk.Label(row, text=str(t["pickup"]), font=MONO_FONT,
                     fg=MUT, bg=CARD).pack(side=tk.LEFT)
            tk.Label(row, text="→", font=BODY_FONT, fg=MUT, bg=CARD).pack(side=tk.LEFT)
            tk.Label(row, text="D", font=("Consolas", 8, "bold"),
                     bg=GOOD, fg="white", width=2).pack(side=tk.LEFT, padx=1)
            tk.Label(row, text=str(t["drop"]), font=MONO_FONT,
                     fg=MUT, bg=CARD).pack(side=tk.LEFT)
            status = tk.Label(row, text="", font=("Consolas", 8, "bold"), bg=CARD,
                              width=13, anchor="e")
            status.pack(side=tk.RIGHT)
            self._task_refs.append(status)

    def _update_tasks(self):
        for t, status in zip(self.sim.tasks, self._task_refs):
            if t.get("done"):
                st, fg = "✓ DONE", GOOD
            elif t.get("picked"):
                st, fg = f"R{t.get('carrier')} carrying", WARN
            elif t.get("assigned") is not None:
                st, fg = f"R{t['assigned']} fetching", ACC
            else:
                st, fg = "queued", MUT
            status.config(text=st, fg=fg)

    # ---- refresh ----
    def _kpis(self):
        s = self.sim
        mode = "DISTRIBUTED" if self.policy == "smart" else "STOP-WAIT"
        done = sum(1 for t in s.tasks if t.get("done"))
        self.kpi_vals["MODE"].config(text=mode)
        self.kpi_vals["STEPS"].config(text=str(s.tick))
        self.kpi_vals["TASKS"].config(text=f"{done} / {len(s.tasks)}")
        self.kpi_vals["COLLISIONS"].config(
            text=str(s.collisions),
            fg=ALERT if s.collisions else TXT)
        self.kpi_vals["REPLANS/DEADLOCKS"].config(
            text=f"{getattr(s, 'reroutes', 0)} / {getattr(s, 'deadlocks', 0)}")
        self.mode_badge.config(text=f"policy={self.policy}")
        if s.all_done():
            self.live.config(text="COMPLETE", fg=ACC)
        else:
            self.live.config(text="LIVE", fg=GOOD)

    def _feed(self, msg, kind="dim"):
        # follow the tail only if the user hasn't scrolled up to read history
        at_bottom = self.logbox.yview()[1] >= 0.999
        self.logbox.insert(tk.END, msg + "\n", kind)
        if at_bottom:
            self.logbox.see(tk.END)

    def _flush(self):
        for r in self.sim.robots:
            while r.log:
                line = r.log.pop(0)
                self._feed(line, log_tag(line))
        for tick, text, kind in getattr(self.sim, "events", []):
            self._feed(f"[t={tick}] {text}", kind)
        self.sim.events.clear()

    def draw(self):
        self._kpis()
        self._draw_dynamic()
        self._update_fleet()
        self._update_tasks()
        self.root.update_idletasks()

    # ---- controls ----
    def do_step(self):
        if not self.sim.all_done():
            self.sim.step()
            self._flush()
            self.draw()

    def do_run(self):
        self._running = not self._running
        self.run_btn.config(text="PAUSE" if self._running else "RUN")

        def loop():
            if self._running and not self.sim.all_done():
                self.sim.step()
                self._flush()
                self.draw()
                self.root.after(self._speed_ms, loop)
            else:
                self._running = False
                self.run_btn.config(text="RUN")
                self.draw()
        loop()

    def do_reset(self):
        from warehouse_sim.simulator import make_scenario
        self._running = False
        self.run_btn.config(text="RUN")
        self.sim = make_scenario(policy=self.policy)
        self._draw_static()
        self._build_fleet()
        self._build_tasks()
        self.logbox.delete("1.0", tk.END)
        self._feed("--- reset: new random map ---", "dim")
        self.draw()

    def show(self):
        self.draw()
        self.root.mainloop()
