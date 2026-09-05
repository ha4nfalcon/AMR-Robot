"""Modern ops-console dashboard (customtkinter + telemetry strip).

Same sim backend as dashboard.py (stdlib Tk), reskinned: rounded cards,
pill buttons, progress-bar batteries, live telemetry strip (deliveries +
fleet battery over time). Run: `python main.py --policy smart --ui ctk`
(requires `pip install customtkinter`).
"""
import tkinter as tk

import customtkinter as ctk

import warehouse_sim.dashboard as base
from warehouse_sim.dashboard import robot_status, log_tag

ROBOT_COLORS = ["#e74c3c", "#3498db", "#2ecc71", "#f39c12", "#9b59b6"]


class CtkDashboard:
    def __init__(self, sim, cell=28):
        self.sim = sim
        self.policy = sim.robots[0].policy if sim.robots else "smart"
        self.cell = cell
        self.theme = "dark"
        self._running = False
        self._speed_ms = 150
        self._fleet_refs = []   # per-robot widget refs, updated in place
        self._task_refs = []    # per-task widget refs, updated in place
        self._task_rows = []    # task row widgets (for rebuilds)

        ctk.set_appearance_mode("dark")
        base.set_theme("dark")
        self.root = ctk.CTk()
        self.root.title("Edge-AI Distributed AMR Fleet Coordination")
        self.root.minsize(1200, 880)
        self._build_ui()

    # ---------- construction ----------
    def _build_ui(self):
        head = ctk.CTkFrame(self.root, fg_color="transparent")
        head.pack(fill=tk.X, padx=12, pady=(8, 2))
        ctk.CTkLabel(head, text="EDGE-AI DISTRIBUTED AMR FLEET COORDINATION",
                     font=("Segoe UI", 20, "bold")).pack(side=tk.LEFT)
        self.live = ctk.CTkLabel(head, text="● LIVE", font=("Consolas", 11, "bold"),
                                 fg_color=base.GOOD, text_color="#04121a",
                                 corner_radius=8, width=80)
        self.live.pack(side=tk.RIGHT, padx=(10, 0))
        ctk.CTkButton(head, text="LIGHT" if self.theme == "dark" else "DARK",
                      width=70, command=self.toggle_theme).pack(side=tk.RIGHT, padx=3)
        ctk.CTkButton(head, text="RESET", width=70,
                      command=self.do_reset, fg_color="gray30").pack(side=tk.RIGHT, padx=3)
        ctk.CTkButton(head, text="STEP", width=70,
                      command=self.do_step, fg_color="gray30").pack(side=tk.RIGHT, padx=3)
        self.run_btn = ctk.CTkButton(head, text="RUN", width=80,
                                     command=self.do_run,
                                     fg_color=base.ACC, text_color="#04121a",
                                     font=("Segoe UI", 12, "bold"))
        self.run_btn.pack(side=tk.RIGHT, padx=3)
        ctk.CTkLabel(self.root,
                     text="Decentralized multi-agent coordination for autonomous mobile "
                          "robots in smart warehouses  ·  peer-to-peer  ·  no cloud",
                     font=("Segoe UI", 11),
                     text_color="gray60").pack(anchor=tk.W, padx=18)

        kpis = ctk.CTkFrame(self.root, fg_color="transparent")
        kpis.pack(fill=tk.X, padx=12, pady=6)
        self.kpi_vals = {}
        strips = [base.ACC, "#8b98ab", base.GOOD, base.ALERT, base.WARN]
        for (shown, key), strip in zip(
                (("M O D E", "MODE"), ("S T E P S", "STEPS"),
                 ("T A S K S", "TASKS"),
                 ("C O L L I S I O N S", "COLLISIONS"),
                 ("T H R O U G H P U T", "THROUGHPUT")), strips):
            card = ctk.CTkFrame(kpis, corner_radius=12)
            card.pack(side=tk.LEFT, expand=True, fill=tk.X, padx=5)
            ctk.CTkFrame(card, height=4, fg_color=strip,
                         corner_radius=0).pack(fill=tk.X)
            ctk.CTkLabel(card, text=shown, font=("Segoe UI", 9),
                         text_color="gray60").pack(anchor=tk.W, padx=14,
                                                   pady=(6, 0))
            val = ctk.CTkLabel(card, text="-", font=("Consolas", 22, "bold"))
            val.pack(anchor=tk.W, padx=14, pady=(0, 6))
            self.kpi_vals[key] = val

        body = ctk.CTkFrame(self.root, fg_color="transparent")
        body.pack(fill=tk.BOTH, expand=True, padx=12, pady=2)
        leftcol = ctk.CTkFrame(body, fg_color="transparent")
        leftcol.pack(side=tk.LEFT)
        self._legend_bar(leftcol)
        W, H = self.sim.wmap.width, self.sim.wmap.height
        map_bg = "#05070c" if self.theme == "dark" else base.GRID_LINE
        self.canvas = tk.Canvas(leftcol, width=W * self.cell,
                                height=H * self.cell, bg=map_bg,
                                highlightthickness=0, bd=0)
        self.canvas.pack(pady=(0, 6))
        self._telemetry_panel(leftcol, W * self.cell)

        side = ctk.CTkScrollableFrame(body, width=380, fg_color="transparent")
        side.pack(side=tk.RIGHT, fill=tk.Y, padx=(8, 0))
        self.fleet_box = self._panel(side, "F L E E T")
        self.task_box = self._panel(side, "T A S K S")
        # fixed-height scroller: 10+ spawned tasks no longer shove the
        # event feed and telemetry off-screen
        self.task_rows = ctk.CTkScrollableFrame(self.task_box, height=190,
                                                fg_color="transparent")
        self.task_rows.pack(fill=tk.X, padx=4, pady=(0, 6))
        self._task_row_widgets = []
        log_panel = self._panel(side, "E V E N T  F E E D")
        self.logbox = tk.Text(log_panel, width=54, height=12,
                              font=("Consolas", 9),
                              bg=base.LOG_BG, fg=base.TXT,
                              insertbackground=base.TXT,
                              highlightthickness=0, relief=tk.FLAT,
                              wrap="word")
        self.logbox.pack(fill=tk.X, padx=8, pady=(0, 8))
        for tag, fg in (("alert", base.ALERT), ("warn", base.WARN),
                        ("ok", base.GOOD), ("info", base.ACC),
                        ("dim", base.MUT if self.theme == "dark" else "#5b6b7f")):
            self.logbox.tag_config(tag, foreground=fg)

        ctrl = ctk.CTkFrame(self.root, fg_color="transparent")
        ctrl.pack(fill=tk.X, padx=12, pady=6)
        ctk.CTkLabel(ctrl, text="speed (ms)").pack(side=tk.LEFT, padx=4)
        self.speed = ctk.CTkSlider(ctrl, from_=50, to=600,
                                   number_of_steps=22,
                                   width=170, command=self._set_speed)
        self.speed.set(self._speed_ms)
        self.speed.pack(side=tk.LEFT)
        self.mode_badge = ctk.CTkLabel(ctrl, text="",
                                       font=("Consolas", 10, "bold"))
        self.mode_badge.pack(side=tk.RIGHT)

        foot = ctk.CTkFrame(self.root, fg_color="transparent")
        foot.pack(fill=tk.X, padx=18, pady=(0, 10))
        self.footer = ctk.CTkLabel(foot, text="", font=("Consolas", 9),
                                   text_color="gray60")
        self.footer.pack(side=tk.LEFT)

        self._draw_static()
        self._build_fleet()
        self._build_tasks()

    def _panel(self, parent, title):
        box = ctk.CTkFrame(parent, corner_radius=10)
        box.pack(fill=tk.X, pady=3)
        ctk.CTkLabel(box, text=title, font=("Segoe UI", 10, "bold"),
                     text_color="gray60").pack(anchor=tk.W, padx=10, pady=(5, 1))
        return box

    def _legend_bar(self, parent):
        bar = ctk.CTkFrame(parent, corner_radius=10)
        bar.pack(fill=tk.X, pady=(0, 4))
        ctk.CTkLabel(bar, text="L E G E N D", font=("Segoe UI", 10, "bold"),
                     text_color="gray60").pack(side=tk.LEFT, padx=(12, 6), pady=5)
        items = [
            ("#e74c3c", "P", "pickup"), (base.GOOD, "D", "drop-off"),
            ("#f1c40f", "T", "carrying"), (base.ACC, "CHG", "dock"),
            (base.CHOKE, "=", "choke"), (base.GRID_WALL, "#", "shelf"),
            ("#ff2fb3", "!", "spill"), ("#8b98ab", "Z", "dead zone"),
        ]
        for color, glyph, text in items:
            ctk.CTkLabel(bar, text=glyph, font=("Consolas", 9, "bold"),
                         fg_color=color, corner_radius=6, width=34,
                         text_color="black" if color in ("#f1c40f", base.ACC)
                         else "white").pack(side=tk.LEFT, padx=(8, 2))
            ctk.CTkLabel(bar, text=text, font=("Segoe UI", 9)).pack(
                side=tk.LEFT, padx=(0, 2))

    def _telemetry_panel(self, parent, width):
        box = ctk.CTkFrame(parent, corner_radius=10)
        box.pack(fill=tk.X, pady=(0, 2))
        top = ctk.CTkFrame(box, fg_color="transparent")
        top.pack(fill=tk.X, padx=10, pady=(6, 0))
        ctk.CTkLabel(top, text="T E L E M E T R Y", font=("Segoe UI", 10, "bold"),
                     text_color="gray60").pack(side=tk.LEFT)
        ctk.CTkLabel(top, text="— deliveries", font=("Consolas", 9),
                     text_color=base.GOOD).pack(side=tk.LEFT, padx=(12, 0))
        ctk.CTkLabel(top, text="— fleet battery %", font=("Consolas", 9),
                     text_color=base.ACC).pack(side=tk.LEFT, padx=(8, 0))
        self.tele = tk.Canvas(box, width=width, height=100,
                              bg=base.LOG_BG, highlightthickness=0, bd=0)
        self.tele.pack(padx=8, pady=6)

    def _set_speed(self, v):
        self._speed_ms = int(float(v))

    # ---------- map ----------
    def _draw_static(self):
        c, m = self.cell, self.sim.wmap
        cv = self.canvas
        cv.delete("static")
        for y in range(m.height):
            for x in range(m.width):
                fill = base.GRID_FREE
                if m.grid[y][x] == 1:
                    fill = base.GRID_WALL
                if (x, y) in (m.choke_cells or []):
                    fill = base.CHOKE
                cv.create_rectangle(x * c, y * c, (x + 1) * c, (y + 1) * c,
                                    fill=fill, outline=base.GRID_LINE,
                                    tags=("static",))
        for dx0, dy0 in m.docks:
            cv.create_rectangle(dx0 * c + 2, dy0 * c + 2,
                                (dx0 + 1) * c - 2, (dy0 + 1) * c - 2,
                                fill=base.ACC, outline="white", width=1,
                                tags=("static",))
            cv.create_text(dx0 * c + c / 2, dy0 * c + c / 2, text="CHG",
                           fill="#04121a" if self.theme == "dark" else "white",
                           font=("Consolas", 8, "bold"), tags=("static",))
        for (x0, y0, x1, y1) in getattr(self.sim.bus, "dead_zones", []):
            cv.create_rectangle(x0 * c, y0 * c, (x1 + 1) * c, (y1 + 1) * c,
                                fill="#8b98ab", stipple="gray25",
                                outline="#8b98ab", dash=(4, 3),
                                tags=("static",))
            cv.create_text((x0 + x1 + 1) / 2 * c, y0 * c + 9,
                           text="DEAD ZONE", fill="#8b98ab",
                           font=("Consolas", 7, "bold"), tags=("static",))

    def _draw_dynamic(self):
        c, m, s = self.cell, self.sim.wmap, self.sim
        cv = self.canvas
        cv.delete("dyn")
        blocked = set()
        for r in s.robots:
            blocked |= r.blocked_known
        for (x, y) in blocked:
            cv.create_rectangle(x * c + 3, y * c + 3, (x + 1) * c - 3,
                                (y + 1) * c - 3, fill="#ff2fb3", outline="",
                                tags=("dyn",))
        for t in s.tasks:
            tid = t["id"]
            if t.get("done"):
                gx, gy = t["drop"]
                cv.create_oval(gx * c + 8, gy * c + 8, (gx + 1) * c - 8,
                               (gy + 1) * c - 8, fill="",
                               outline="#1e5c38", width=2, tags=("dyn",))
                cv.create_text(gx * c + c / 2, gy * c + c / 2, text=f"✓D{tid}",
                               fill="#1e5c38", font=("Consolas", 8, "bold"),
                               tags=("dyn",))
                continue
            if not t.get("picked"):
                px, py = t["pickup"]
                cv.create_rectangle(px * c + 8, py * c + 8, (px + 1) * c - 8,
                                    (py + 1) * c - 8, fill="#e74c3c",
                                    outline="#7b1a12", tags=("dyn",))
                cv.create_text(px * c + c / 2, py * c + c / 2, text=f"P{tid}",
                               fill="white", font=("Consolas", 8, "bold"),
                               tags=("dyn",))
            gx, gy = t["drop"]
            cv.create_oval(gx * c + 8, gy * c + 8, (gx + 1) * c - 8,
                           (gy + 1) * c - 8, fill=base.GOOD, outline="#0e5a30",
                           tags=("dyn",))
            cv.create_text(gx * c + c / 2, gy * c + c / 2, text=f"D{tid}",
                           fill="white", font=("Consolas", 8, "bold"),
                           tags=("dyn",))
        for r in s.robots:
            if getattr(r, "dead", False):
                x, y = r.pos
                cv.create_rectangle(x * c + 6, y * c + 6, (x + 1) * c - 6,
                                    (y + 1) * c - 6, fill="#555555",
                                    outline=base.ALERT, width=2, tags=("dyn",))
                cv.create_text(x * c + c / 2, y * c - 8, text="DEAD",
                               fill=base.ALERT, font=("Consolas", 7, "bold"),
                               tags=("dyn",))
                continue
            if r.path:
                col = ROBOT_COLORS[r.rid % len(ROBOT_COLORS)]
                xy = []
                for (x, y) in [r.pos] + list(r.path):
                    xy += [x * c + c / 2, y * c + c / 2]
                under = "#05070c" if self.theme == "dark" else "#ffffff"
                cv.create_line(*xy, fill=under, width=4, tags=("dyn",))
                cv.create_line(*xy, fill=col, width=2, dash=(5, 3),
                               tags=("dyn",))
        for r in s.robots:
            if getattr(r, "dead", False):
                continue
            x, y = r.pos
            col = ROBOT_COLORS[r.rid % len(ROBOT_COLORS)]
            cv.create_oval(x * c - 1, y * c - 1, (x + 1) * c + 1,
                           (y + 1) * c + 1, fill=col, outline="",
                           stipple="gray25", tags=("dyn",))
            cv.create_oval(x * c + 3, y * c + 3, (x + 1) * c - 3,
                           (y + 1) * c - 3, fill=col, outline="white",
                           width=2, tags=("dyn",))
            cv.create_text(x * c + c / 2, y * c + c / 2, text=str(r.rid),
                           fill="white", font=("Consolas", 10, "bold"),
                           tags=("dyn",))
            if r.task_id is not None and r.phase == "drop":
                cv.create_rectangle(x * c + 1, y * c - 12, x * c + 33,
                                    y * c - 2, fill="#f1c40f", outline="black",
                                    tags=("dyn",))
                cv.create_text(x * c + 17, y * c - 7, text=f"T{r.task_id}",
                               fill="black", font=("Consolas", 7, "bold"),
                               tags=("dyn",))
            bw = (c - 6) * max(0.0, min(1.0, r.battery / 100.0))
            cv.create_rectangle(x * c + 3, (y + 1) * c - 6, x * c + 3 + bw,
                                (y + 1) * c - 3,
                                fill=base.GOOD if r.battery > 30 else base.ALERT,
                                outline="", tags=("dyn",))

    # ---------- side panels (built once, mutated in place) ----------
    def _build_fleet(self):
        for w in self.fleet_box.winfo_children()[1:]:
            w.destroy()
        self._fleet_refs = []
        for r in self.sim.robots:
            row = ctk.CTkFrame(self.fleet_box, corner_radius=8)
            row.pack(fill=tk.X, padx=8, pady=3)
            accent = tk.Frame(row, bg=ROBOT_COLORS[r.rid % len(ROBOT_COLORS)],
                              width=4)
            accent.pack(side=tk.LEFT, fill=tk.Y, padx=(0, 2), pady=4)
            inner = ctk.CTkFrame(row, fg_color="transparent")
            inner.pack(side=tk.LEFT, fill=tk.X, expand=True)
            top = ctk.CTkFrame(inner, fg_color="transparent")
            top.pack(fill=tk.X, padx=6, pady=(4, 0))
            ctk.CTkLabel(top, text="●", font=("Segoe UI", 13),
                         text_color=ROBOT_COLORS[r.rid % len(ROBOT_COLORS)],
                         width=20).pack(side=tk.LEFT)
            ctk.CTkLabel(top, text=f"R{r.rid}",
                         font=("Consolas", 12, "bold")).pack(side=tk.LEFT)
            pill = ctk.CTkLabel(top, text="", font=("Consolas", 9, "bold"),
                                corner_radius=8, width=95, padx=6)
            pill.pack(side=tk.RIGHT, padx=4)
            bottom = ctk.CTkFrame(inner, fg_color="transparent")
            bottom.pack(fill=tk.X, padx=6, pady=(0, 5))
            bar = ctk.CTkProgressBar(bottom, width=120, height=10)
            bar.pack(side=tk.LEFT, padx=(4, 8))
            bar.set(1.0)
            job = ctk.CTkLabel(bottom, text="", font=("Consolas", 9),
                               text_color="gray60")
            job.pack(side=tk.LEFT, fill=tk.X, expand=True)
            self._fleet_refs.append({"pill": pill, "bar": bar, "job": job})

    def _update_fleet(self):
        m = self.sim.wmap
        for r, refs in zip(self.sim.robots, self._fleet_refs):
            label, key = robot_status(r, m.docks)
            refs["pill"].configure(
                text=label, fg_color=base.PILL_BG[key], text_color=base.PILL_FG[key])
            frac = max(0.0, min(1.0, r.battery / 100.0))
            refs["bar"].set(frac)
            refs["bar"].configure(
                progress_color=base.GOOD if r.battery > 30 else base.ALERT)
            job = "" if r.task_id is None else (
                f"T{r.task_id} {'pickup' if r.phase == 'pickup' else 'drop'} "
                f"{r.pos} → {r.goal}")
            refs["job"].configure(text=f"{r.battery:.0f}%  {job}")

    def _build_tasks(self):
        for w in self._task_row_widgets:
            w.destroy()
        self._task_row_widgets = []
        self._task_refs = []
        for t in self.sim.tasks:
            row = ctk.CTkFrame(self.task_rows, fg_color="transparent")
            row.pack(fill=tk.X, padx=8, pady=1)
            ctk.CTkLabel(row, text=f"T{t['id']}",
                         font=("Consolas", 10, "bold"),
                         width=32).pack(side=tk.LEFT)
            ctk.CTkLabel(row, text="P", font=("Consolas", 8, "bold"),
                         fg_color="#e74c3c", text_color="white",
                         corner_radius=5, width=20).pack(side=tk.LEFT, padx=1)
            ctk.CTkLabel(row, text=str(t["pickup"]),
                         font=("Consolas", 9),
                         text_color="gray60").pack(side=tk.LEFT)
            ctk.CTkLabel(row, text="→", text_color="gray60").pack(side=tk.LEFT)
            ctk.CTkLabel(row, text="D", font=("Consolas", 8, "bold"),
                         fg_color=base.GOOD, text_color="white",
                         corner_radius=5, width=20).pack(side=tk.LEFT, padx=1)
            ctk.CTkLabel(row, text=str(t["drop"]),
                         font=("Consolas", 9),
                         text_color="gray60").pack(side=tk.LEFT)
            status = ctk.CTkLabel(row, text="",
                                  font=("Consolas", 9, "bold"))
            status.pack(side=tk.RIGHT)
            self._task_row_widgets.append(row)
            self._task_refs.append(status)

    def _update_tasks(self):
        # task list can grow (spawner): rebuild rows when count changes
        if len(self._task_refs) != len(self.sim.tasks):
            self._build_tasks()
        for t, status in zip(self.sim.tasks, self._task_refs):
            if t.get("done"):
                st, fg = "✓ DONE", base.GOOD
            elif t.get("picked"):
                st, fg = f"◐ R{t.get('carrier')} carrying", base.WARN
            elif t.get("assigned") is not None:
                st, fg = f"◐ R{t['assigned']} fetching", base.ACC
            else:
                st, fg = "○ queued", "gray60"
            status.configure(text=st, text_color=fg)

    # ---------- telemetry strip ----------
    def _draw_telemetry(self):
        cv = self.tele
        cv.delete("all")
        W, H = int(cv.cget("width")), int(cv.cget("height"))
        tel = self.sim.telemetry
        ticks = tel["tick"]
        if len(ticks) < 2:
            cv.create_text(W // 2, H // 2, text="telemetry warming up…",
                           fill="gray60", font=("Consolas", 9))
            return
        done = tel["done"]
        batt = tel["avg_batt"]
        vmax = max(max(done), 1)
        grid = base.GRID_LINE
        for frac in (0.25, 0.5, 0.75):
            y = 6 + frac * (H - 16)
            cv.create_line(4, y, W - 4, y, fill=grid, dash=(2, 4))
        cv.create_text(6, 8, anchor=tk.NW, text=f"max {vmax}",
                       fill="gray60", font=("Consolas", 8))
        cv.create_text(W - 4, H - 2, anchor=tk.SE, text="tick →",
                       fill="gray60", font=("Consolas", 8))
        xs = [i / max(len(ticks) - 1, 1) * (W - 8) + 4 for i in range(len(ticks))]
        pts_d = [y for y in
                 [H - 6 - v / vmax * (H - 16) for v in done]]
        cv.create_line(*[c for p in zip(xs, pts_d) for c in p],
                       fill=base.GOOD, width=2)
        pts_b = [H - 6 - v / 100 * (H - 16) for v in batt]
        cv.create_line(*[c for p in zip(xs, pts_b) for c in p],
                       fill=base.ACC, width=2, dash=(4, 3))
        cv.create_text(W - 4, 10, anchor=tk.NE,
                       text=f"{done[-1]} delivered · {batt[-1]:.0f}% batt",
                       fill="gray60", font=("Consolas", 9))

    # ---------- refresh ----------
    def _kpis(self):
        s = self.sim
        mode = "DISTRIBUTED" if self.policy == "smart" else "STOP-WAIT"
        done = sum(1 for t in s.tasks if t.get("done"))
        self.kpi_vals["MODE"].configure(text=mode)
        self.kpi_vals["STEPS"].configure(text=str(s.tick))
        self.kpi_vals["TASKS"].configure(text=f"{done} / {len(s.tasks)}")
        self.kpi_vals["COLLISIONS"].configure(
            text=str(s.collisions),
            text_color=base.ALERT if s.collisions else base.TXT)
        thr = done / max(s.tick, 1)
        self.kpi_vals["THROUGHPUT"].configure(text=f"{thr:.3f}/tick")
        self.mode_badge.configure(text=f"policy={self.policy}")
        self.footer.configure(
            text=f"dropped {s.bus.dropped} msgs  ·  "
                 f"deaths {getattr(s, 'deaths', 0)}  ·  "
                 f"recoveries {getattr(s, 'recoveries', 0)}  ·  "
                 f"deadlocks {getattr(s, 'deadlocks', 0)}")
        if s.all_done():
            self.live.configure(text="■ COMPLETE", fg_color=base.ACC)
        else:
            self.live.configure(text="● LIVE", fg_color=base.GOOD)

    def _feed(self, msg, kind="dim"):
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
        self._draw_telemetry()
        self.root.update_idletasks()

    # ---------- controls ----------
    def do_step(self):
        if not self.sim.all_done():
            self.sim.step()
            self._flush()
            self.draw()

    def do_run(self):
        self._running = not self._running
        self.run_btn.configure(text="PAUSE" if self._running else "RUN")

        def loop():
            if self._running and not self.sim.all_done():
                self.sim.step()
                self._flush()
                self.draw()
                self.root.after(self._speed_ms, loop)
            else:
                self._running = False
                self.run_btn.configure(text="RUN")
                self.draw()
        loop()

    def do_reset(self):
        from warehouse_sim.simulator import make_scenario
        self._running = False
        self.run_btn.configure(text="RUN")
        self.sim = make_scenario(policy=self.policy)
        self._draw_static()
        self._build_fleet()
        self._build_tasks()
        self.logbox.delete("1.0", tk.END)
        self._feed("--- reset: new random map ---", "dim")
        self.draw()

    def toggle_theme(self):
        lines = self.logbox.get("1.0", tk.END).splitlines()
        self._running = False
        self.theme = "light" if self.theme == "dark" else "dark"
        ctk.set_appearance_mode(self.theme)
        base.set_theme(self.theme)
        for w in self.root.winfo_children():
            w.destroy()
        self._build_ui()
        for line in lines:
            if line.strip():
                self._feed(line, log_tag(line))
        self.draw()

    def show(self):
        self.draw()
        self.root.mainloop()
