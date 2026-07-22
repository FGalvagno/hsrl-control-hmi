import tkinter as tk
from tkinter import ttk, scrolledtext
import serial
import serial.tools.list_ports
import threading
import queue
import re
import os
import logging
from collections import deque
from dataclasses import dataclass
from datetime import datetime

import matplotlib
matplotlib.use("TkAgg")
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure

# soporte svg para logos (opcional)
try:
    import tksvg
    _SVG_OK = True
except ImportError:
    _SVG_OK = False


# -- logging a archivo --

_BASE_DIR = os.path.dirname(os.path.abspath(__file__))
_LOGS_DIR = os.path.join(_BASE_DIR, "logs")
os.makedirs(_LOGS_DIR, exist_ok=True)

_ts = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
_log_file = os.path.join(_LOGS_DIR, f"hsrl-{_ts}.log")

flog = logging.getLogger("hsrl")
flog.setLevel(logging.DEBUG)
_fh = logging.FileHandler(_log_file, encoding="utf-8")
_fh.setFormatter(logging.Formatter(
    "[%(asctime)s] %(message)s", datefmt="%Y-%m-%d %H:%M:%S"))
flog.addHandler(_fh)


# -- paleta ISA-101 --

BG         = "#D9D9D9"
PANEL_BG   = "#EBEBEB"
VALUE_BG   = "#FFFFFF"
TITLE_BG   = "#C8C8C8"
TEXT_CLR   = "#2B2B2B"
LABEL_CLR  = "#606060"
BORDER_CLR = "#BFBFBF"

GREEN  = "#00B050"
YELLOW = "#FFC000"
RED    = "#FF3030"
BLUE   = "#0070C0"
GREY   = "#808080"

MODE_STYLE = {
    "s": (GREY,     "#FFFFFF", "PARADO"),
    "f": (BLUE,     "#FFFFFF", "AVANZANDO BARRIDO"),
    "b": (BLUE,     "#FFFFFF", "RETROCEDIENDO BARRIDO"),
    "g": (GREEN,    "#FFFFFF", "SINTONIZADO"),
    "e": ("#606060", "#FFFFFF", "FINALIZADO"),
}

FONT_LBL   = ("Segoe UI", 9)
FONT_VAL   = ("Consolas", 11)
FONT_TITLE = ("Segoe UI", 9, "bold")
FONT_MODE  = ("Segoe UI", 14, "bold")
FONT_SMALL = ("Segoe UI", 8)
FONT_HDR   = ("Segoe UI", 12, "bold")


# -- datos del proceso --

@dataclass
class ProcData:
    mode: str = "s"
    heater_sp: float = 0.0
    heater_pv: float = 0.0
    d_heater: float = 0.0
    piezo_sp: float = 0.0
    piezo_pv: float = 0.0
    pp: float = 0.0
    pm: float = 0.0
    ratio: float = 0.0
    lock_on: bool = False
    lock_ref: float = 0.0
    lock_lo: float = 0.0
    lock_hi: float = 0.0
    cycle: int = 0


# -- parser de telemetria del rpico --

_RE_STATE = re.compile(
    r"modo=(\w)\s*\|\s*heater_sp=([0-9.]+)\s+heater_real=([0-9.]+)"
    r"\s*\|\s*piezo=([0-9.]+)\s+piezo_real=([0-9.]+)")
_RE_DETAIL = re.compile(
    r"pp=([0-9.]+)\s+pm=([0-9.]+)\s+prt=([0-9.]+)"
    r"\s*\|\s*d_heater=([+-]?[0-9.]+)")
_RE_LOCK = re.compile(
    r"lock:\s*ref=([0-9.]+)\s+banda=\[([0-9.]+),\s*([0-9.]+)\]")
_RE_CYCLE = re.compile(r"^\[(\d+)\]")
_RE_KV = re.compile(r"\[estado\]\s+(\S+)\s+=\s+(.+)")


def parse_line(line, data):
    hit = False

    m = _RE_STATE.search(line)
    if m:
        data.mode = m.group(1)
        data.heater_sp = float(m.group(2))
        data.heater_pv = float(m.group(3))
        data.piezo_sp = float(m.group(4))
        data.piezo_pv = float(m.group(5))
        if data.mode != "g":
            data.lock_on = False
        hit = True

    m = _RE_DETAIL.search(line)
    if m and "medicion" not in line:
        data.pp = float(m.group(1))
        data.pm = float(m.group(2))
        data.ratio = float(m.group(3))
        data.d_heater = float(m.group(4))
        hit = True

    m = _RE_LOCK.search(line)
    if m:
        data.lock_on = True
        data.lock_ref = float(m.group(1))
        data.lock_lo = float(m.group(2))
        data.lock_hi = float(m.group(3))
        hit = True

    m = _RE_CYCLE.match(line)
    if m:
        data.cycle = int(m.group(1))

    m = _RE_KV.match(line)
    if m:
        k, v = m.group(1), m.group(2).strip()
        try:
            if k == "modo":
                data.mode = v
            elif k == "piezo_v":
                data.piezo_sp = float(v)
            elif k == "heater_sp":
                data.heater_sp = float(v)
            elif k == "lock_activo":
                data.lock_on = (v == "si")
            elif k == "prt_ref":
                data.lock_ref = float(v)
        except ValueError:
            pass
        hit = True

    return hit


def _load_svg(path, height=40):
    if not _SVG_OK or not os.path.isfile(path):
        return None
    try:
        return tksvg.SvgImage(file=path, scaletoheight=height)
    except Exception:
        return None


# -- aplicacion --

class HMI:
    BAUDS = ["9600", "19200", "38400", "57600", "115200", "230400"]

    def __init__(self, root):
        self.root = root
        self.root.title("HSRL Control - Longitud de Onda")
        self.root.geometry("1100x750")
        self.root.minsize(900, 600)
        self.root.configure(bg=BG)
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

        self.ser = None
        self.running = False
        self.rx_q = queue.Queue()
        self.data = ProcData()

        # buffers para el grafico amplitud vs temperatura
        self.plot_t = deque(maxlen=500)
        self.plot_pp = deque(maxlen=500)
        self.plot_pm = deque(maxlen=500)

        self.heater_min = 61.3
        self.heater_max = 63.3

        assets = os.path.join(_BASE_DIR, "assets")
        self._logo_l = _load_svg(os.path.join(assets, "logo_fcefyn.svg"), 44)
        self._logo_r = _load_svg(os.path.join(assets, "logo_smn.svg"), 44)

        self._build_ui()
        self._refresh_ports()
        self._poll()
        self._tick_plot()

        flog.info("HMI iniciada")

    # -- construccion de la interfaz --

    def _build_ui(self):
        self._build_header()
        self._build_conn_bar()

        body = tk.Frame(self.root, bg=BG)
        body.pack(fill=tk.BOTH, expand=True, padx=6, pady=(0, 6))
        body.columnconfigure(1, weight=1)
        body.rowconfigure(0, weight=1)

        self._build_panel(body)
        self._build_right(body)

    def _build_header(self):
        hdr = tk.Frame(self.root, bg=PANEL_BG,
                       highlightbackground=BORDER_CLR, highlightthickness=1)
        hdr.pack(fill=tk.X, padx=6, pady=(6, 0))
        inner = tk.Frame(hdr, bg=PANEL_BG)
        inner.pack(fill=tk.X, padx=8, pady=4)

        if self._logo_l:
            tk.Label(inner, image=self._logo_l, bg=PANEL_BG
                     ).pack(side=tk.LEFT, padx=(0, 12))

        tk.Label(inner, text="CONTROL DE LONGITUD DE ONDA HSRL",
                 bg=PANEL_BG, fg=TEXT_CLR, font=FONT_HDR
                 ).pack(side=tk.LEFT, expand=True)

        if self._logo_r:
            tk.Label(inner, image=self._logo_r, bg=PANEL_BG
                     ).pack(side=tk.RIGHT, padx=(12, 0))

    def _build_conn_bar(self):
        bar = tk.Frame(self.root, bg=PANEL_BG,
                       highlightbackground=BORDER_CLR, highlightthickness=1)
        bar.pack(fill=tk.X, padx=6, pady=(4, 6))
        inner = tk.Frame(bar, bg=PANEL_BG)
        inner.pack(fill=tk.X, padx=8, pady=6)

        tk.Label(inner, text="PUERTO", bg=PANEL_BG, fg=LABEL_CLR,
                 font=FONT_LBL).pack(side=tk.LEFT, padx=(0, 4))
        self.port_var = tk.StringVar()
        self.port_cb = ttk.Combobox(inner, textvariable=self.port_var,
                                    width=16, state="readonly")
        self.port_cb.pack(side=tk.LEFT, padx=(0, 2))

        ttk.Button(inner, text="⟳", width=3,
                   command=self._refresh_ports
                   ).pack(side=tk.LEFT, padx=(0, 12))

        tk.Label(inner, text="BAUDIOS", bg=PANEL_BG, fg=LABEL_CLR,
                 font=FONT_LBL).pack(side=tk.LEFT, padx=(0, 4))
        self.baud_var = tk.StringVar(value="115200")
        self.baud_cb = ttk.Combobox(inner, textvariable=self.baud_var,
                                    values=self.BAUDS, width=8, state="readonly")
        self.baud_cb.pack(side=tk.LEFT, padx=(0, 12))

        self.conn_btn = ttk.Button(inner, text="Conectar",
                                   command=self._toggle_conn)
        self.conn_btn.pack(side=tk.LEFT, padx=(0, 8))

        self.ind_cv = tk.Canvas(inner, width=14, height=14,
                                bg=PANEL_BG, highlightthickness=0)
        self.ind_cv.pack(side=tk.LEFT, padx=(0, 4))
        self._ind = self.ind_cv.create_oval(1, 1, 13, 13, fill=GREY)

        self.status_lbl = tk.Label(inner, text="DESCONECTADO", bg=PANEL_BG,
                                   fg=GREY, font=FONT_TITLE)
        self.status_lbl.pack(side=tk.LEFT)

    def _build_panel(self, parent):
        """panel izquierdo con los valores del proceso"""
        panel = tk.Frame(parent, bg=BG, width=260)
        panel.grid(row=0, column=0, sticky="ns", padx=(0, 6))
        panel.grid_propagate(False)
        panel.configure(width=260)

        f = self._group(panel, "ESTADO DE CONTROL")
        self.mode_lbl = tk.Label(f, text="PARADO", bg=GREY, fg="#FFF",
                                 font=FONT_MODE, width=14, height=1)
        self.mode_lbl.pack(padx=6, pady=6)

        f = self._group(panel, "TEMPERATURA HEATER")
        self.v_hsp = self._vrow(f, "SP", "°C", fg=BLUE)
        self.v_hpv = self._vrow(f, "PV", "°C")
        self.v_hdt = self._vrow(f, "ΔT", "°C")

        f = self._group(panel, "VOLTAJE PIEZOELÉCTRICO")
        self.v_psp = self._vrow(f, "SP", "V", fg=BLUE)
        self.v_ppv = self._vrow(f, "PV", "V")

        f = self._group(panel, "DETECTOR")
        self.v_pp = self._vrow(f, "P+", "V")
        self.v_pm = self._vrow(f, "P-", "V")
        self.v_rat = self._vrow(f, "RATIO", "")

        f = self._group(panel, "SINTONIZACIÓN")
        self.v_lstat = self._vrow(f, "ESTADO", "")
        self.v_lref = self._vrow(f, "REF", "")
        self.v_lband = self._vrow(f, "BANDA", "")

        f = self._group(panel, "CICLO")
        self.v_cyc = self._vrow(f, "#", "")

    def _build_right(self, parent):
        right = tk.Frame(parent, bg=BG)
        right.grid(row=0, column=1, sticky="nsew")
        right.rowconfigure(0, weight=3)
        right.rowconfigure(2, weight=2)
        right.columnconfigure(0, weight=1)

        self._build_plot(right)
        self._build_ctrl(right)
        self._build_log(right)

    def _build_plot(self, parent):
        frame = tk.Frame(parent, bg=PANEL_BG,
                         highlightbackground=BORDER_CLR, highlightthickness=1)
        frame.grid(row=0, column=0, sticky="nsew", pady=(0, 6))

        tb = tk.Frame(frame, bg=TITLE_BG)
        tb.pack(fill=tk.X)
        tk.Label(tb, text="AMPLITUD vs TEMPERATURA", bg=TITLE_BG,
                 fg=TEXT_CLR, font=FONT_TITLE).pack(side=tk.LEFT, padx=8, pady=3)
        ttk.Button(tb, text="Limpiar", width=7,
                   command=self._clear_plot).pack(side=tk.RIGHT, padx=4, pady=2)

        self.fig = Figure(figsize=(6, 3), dpi=96, facecolor=PANEL_BG)
        self.ax = self.fig.add_subplot(111)
        self.ax.set_facecolor("#FFFFFF")
        self.ax.set_xlabel("Temperatura (°C)", fontsize=9, color=TEXT_CLR)
        self.ax.set_ylabel("Amplitud (V)", fontsize=9, color=TEXT_CLR)
        self.ax.tick_params(colors=TEXT_CLR, labelsize=8)
        self.ax.grid(True, which="major", color="#D0D0D0", linewidth=0.6)
        self.ax.grid(True, which="minor", color="#ECECEC", linewidth=0.3)
        self.ax.minorticks_on()
        self.fig.subplots_adjust(left=0.10, right=0.96, top=0.95, bottom=0.18)

        self.ln_pp, = self.ax.plot([], [], color="#0070C0", lw=1.2,
                                   marker=".", ms=3, label="P+")
        self.ln_pm, = self.ax.plot([], [], color="#C00000", lw=1.2,
                                   marker=".", ms=3, label="P−")
        self.ax.legend(loc="upper right", fontsize=8, framealpha=0.8)

        self.canvas = FigureCanvasTkAgg(self.fig, master=frame)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True, padx=2, pady=2)
        self.canvas.draw()

    def _build_ctrl(self, parent):
        frame = tk.Frame(parent, bg=PANEL_BG,
                         highlightbackground=BORDER_CLR, highlightthickness=1)
        frame.grid(row=1, column=0, sticky="ew", pady=(0, 6))

        tb = tk.Frame(frame, bg=TITLE_BG)
        tb.pack(fill=tk.X)
        tk.Label(tb, text="CONTROLES", bg=TITLE_BG,
                 fg=TEXT_CLR, font=FONT_TITLE).pack(side=tk.LEFT, padx=8, pady=3)

        inner = tk.Frame(frame, bg=PANEL_BG)
        inner.pack(fill=tk.X, padx=8, pady=6)

        bf = tk.Frame(inner, bg=PANEL_BG)
        bf.pack(fill=tk.X, pady=(0, 6))
        for txt, cmd, clr in [
            ("DETENER", "s", RED), ("AVANZAR ▲", "f", BLUE),
            ("RETROCEDER ▼", "b", BLUE), ("SINTONIZAR", "g", GREEN),
        ]:
            tk.Button(bf, text=txt, bg=clr, fg="white",
                      font=("Segoe UI", 9, "bold"), relief=tk.FLAT,
                      activebackground=clr, activeforeground="white",
                      padx=12, pady=4,
                      command=lambda c=cmd: self._send(c)
                      ).pack(side=tk.LEFT, padx=(0, 4))

        ttk.Separator(inner, orient=tk.HORIZONTAL).pack(fill=tk.X, pady=4)

        pf = tk.Frame(inner, bg=PANEL_BG)
        pf.pack(fill=tk.X)

        tk.Label(pf, text="Temp. SP", bg=PANEL_BG, fg=LABEL_CLR,
                 font=FONT_LBL).pack(side=tk.LEFT, padx=(0, 4))
        self.ent_t = ttk.Entry(pf, width=10)
        self.ent_t.pack(side=tk.LEFT, padx=(0, 2))
        self.ent_t.bind("<Return>", lambda _: self._set_param("T", self.ent_t))
        ttk.Button(pf, text="Fijar", width=5,
                   command=lambda: self._set_param("T", self.ent_t)
                   ).pack(side=tk.LEFT, padx=(0, 12))

        tk.Label(pf, text="Voltaje P.", bg=PANEL_BG, fg=LABEL_CLR,
                 font=FONT_LBL).pack(side=tk.LEFT, padx=(0, 4))
        self.ent_p = ttk.Entry(pf, width=10)
        self.ent_p.pack(side=tk.LEFT, padx=(0, 2))
        self.ent_p.bind("<Return>", lambda _: self._set_param("P", self.ent_p))
        ttk.Button(pf, text="Fijar", width=5,
                   command=lambda: self._set_param("P", self.ent_p)
                   ).pack(side=tk.LEFT)

    def _build_log(self, parent):
        frame = tk.Frame(parent, bg=PANEL_BG,
                         highlightbackground=BORDER_CLR, highlightthickness=1)
        frame.grid(row=2, column=0, sticky="nsew")

        tb = tk.Frame(frame, bg=TITLE_BG)
        tb.pack(fill=tk.X)
        tk.Label(tb, text="MONITOR SERIE", bg=TITLE_BG,
                 fg=TEXT_CLR, font=FONT_TITLE).pack(side=tk.LEFT, padx=8, pady=3)
        ttk.Button(tb, text="Limpiar", width=7,
                   command=self._clear_log
                   ).pack(side=tk.RIGHT, padx=4, pady=2)
        ttk.Button(tb, text="Estado (?)", width=10,
                   command=lambda: self._send("?")
                   ).pack(side=tk.RIGHT, padx=4, pady=2)

        self.logbox = scrolledtext.ScrolledText(
            frame, wrap=tk.WORD, state=tk.DISABLED,
            font=("Consolas", 9), bg="#1E1E1E", fg="#D4D4D4",
            insertbackground="#D4D4D4", relief=tk.FLAT, height=8)
        self.logbox.pack(fill=tk.BOTH, expand=True, padx=2, pady=(2, 0))
        self.logbox.tag_configure("tx", foreground="#569CD6")
        self.logbox.tag_configure("rx", foreground="#D4D4D4")
        self.logbox.tag_configure("info", foreground="#6A9955")
        self.logbox.tag_configure("error", foreground="#F44747")

        cf = tk.Frame(frame, bg="#1E1E1E")
        cf.pack(fill=tk.X, padx=2, pady=2)
        tk.Label(cf, text="›", bg="#1E1E1E", fg="#569CD6",
                 font=("Consolas", 11)).pack(side=tk.LEFT, padx=(4, 2))
        self.cmd_var = tk.StringVar()
        self.cmd_ent = tk.Entry(cf, textvariable=self.cmd_var,
                                font=("Consolas", 10), bg="#2D2D2D",
                                fg="#D4D4D4", insertbackground="#D4D4D4",
                                relief=tk.FLAT)
        self.cmd_ent.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=2)
        self.cmd_ent.bind("<Return>", self._on_send)
        ttk.Button(cf, text="Enviar", width=7,
                   command=self._on_send).pack(side=tk.LEFT, padx=(2, 4))

    # -- widgets auxiliares --

    def _group(self, parent, title):
        f = tk.Frame(parent, bg=PANEL_BG,
                     highlightbackground=BORDER_CLR, highlightthickness=1)
        f.pack(fill=tk.X, padx=4, pady=(4, 0))
        tb = tk.Frame(f, bg=TITLE_BG)
        tb.pack(fill=tk.X)
        tk.Label(tb, text=title, bg=TITLE_BG, fg=TEXT_CLR,
                 font=FONT_TITLE).pack(anchor="w", padx=6, pady=2)
        return f

    def _vrow(self, parent, label, unit, fg=TEXT_CLR):
        row = tk.Frame(parent, bg=PANEL_BG)
        row.pack(fill=tk.X, padx=6, pady=2)
        tk.Label(row, text=label, bg=PANEL_BG, fg=LABEL_CLR,
                 font=FONT_LBL, width=6, anchor="w").pack(side=tk.LEFT)
        val = tk.Label(row, text="---", bg=VALUE_BG, fg=fg,
                       font=FONT_VAL, width=12, anchor="e",
                       relief=tk.SUNKEN, bd=1)
        val.pack(side=tk.LEFT, padx=2)
        if unit:
            tk.Label(row, text=unit, bg=PANEL_BG, fg=LABEL_CLR,
                     font=FONT_SMALL, anchor="w").pack(side=tk.LEFT, padx=2)
        return val

    # -- puertos --

    def _refresh_ports(self):
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.port_cb["values"] = ports
        if ports and not self.port_var.get():
            self.port_var.set(ports[0])

    # -- conexion serie --

    def _toggle_conn(self):
        if self.ser and self.ser.is_open:
            self._disconnect()
        else:
            self._connect()

    def _connect(self):
        port = self.port_var.get()
        baud = self.baud_var.get()
        if not port:
            self._log("Puerto no seleccionado\n", "error")
            return
        try:
            self.ser = serial.Serial(port, int(baud), timeout=0.1)
            self.running = True
            threading.Thread(target=self._rx_loop, daemon=True).start()
            self._set_conn(True, port, baud)
            self._log(f"Conectado a {port} @ {baud}\n", "info")
        except serial.SerialException as e:
            self._log(f"Error de conexión: {e}\n", "error")
            self.ind_cv.itemconfig(self._ind, fill=RED)

    def _disconnect(self):
        self.running = False
        if self.ser and self.ser.is_open:
            name = self.ser.port
            self.ser.close()
            self._log(f"Desconectado de {name}\n", "info")
        self.ser = None
        self._set_conn(False)

    def _set_conn(self, on, port="", baud=""):
        if on:
            self.ind_cv.itemconfig(self._ind, fill=GREEN)
            self.status_lbl.config(text=f"CONECTADO  {port} @ {baud}", fg=GREEN)
            self.conn_btn.config(text="Desconectar")
            self.port_cb.config(state="disabled")
            self.baud_cb.config(state="disabled")
        else:
            self.ind_cv.itemconfig(self._ind, fill=GREY)
            self.status_lbl.config(text="DESCONECTADO", fg=GREY)
            self.conn_btn.config(text="Conectar")
            self.port_cb.config(state="readonly")
            self.baud_cb.config(state="readonly")

    # -- lectura serie en hilo secundario --

    def _rx_loop(self):
        buf = ""
        while self.running:
            try:
                if self.ser and self.ser.is_open:
                    n = self.ser.in_waiting
                    if n:
                        buf += self.ser.read(n).decode("utf-8", errors="replace")
                        while "\n" in buf:
                            line, buf = buf.split("\n", 1)
                            self.rx_q.put(line.rstrip("\r"))
            except serial.SerialException:
                self.rx_q.put("\x00DISC")
                break

    # -- poll desde el hilo principal (cada 100ms) --

    def _poll(self):
        updated = False
        for _ in range(50):
            if self.rx_q.empty():
                break
            line = self.rx_q.get_nowait()
            if line == "\x00DISC":
                self._disconnect()
                self._log("Conexión serie perdida\n", "error")
                break
            self._log(line + "\n", "rx")
            if parse_line(line, self.data):
                updated = True

        if updated:
            self._refresh_display()
            if self.data.heater_pv > 0 and (self.data.pp > 0 or self.data.pm > 0):
                self.plot_t.append(self.data.heater_pv)
                self.plot_pp.append(self.data.pp)
                self.plot_pm.append(self.data.pm)

        self.root.after(100, self._poll)

    # -- actualizar indicadores del panel --

    def _refresh_display(self):
        d = self.data

        bg, fg, txt = MODE_STYLE.get(d.mode, (GREY, "#FFF", d.mode.upper()))
        self.mode_lbl.config(text=txt, bg=bg, fg=fg)

        self.v_hsp.config(text=f"{d.heater_sp:.4f}")
        self.v_hpv.config(text=f"{d.heater_pv:.4f}")
        self.v_hdt.config(text=f"{d.d_heater:+.6f}")

        # alarma visual si el pv se acerca a los limites
        pv_fg, pv_bg = TEXT_CLR, VALUE_BG
        if d.heater_pv > 0:
            margin = (self.heater_max - self.heater_min) * 0.05
            if d.heater_pv >= self.heater_max or d.heater_pv <= self.heater_min:
                pv_fg, pv_bg = RED, "#FFE0E0"
            elif (d.heater_pv >= self.heater_max - margin
                  or d.heater_pv <= self.heater_min + margin):
                pv_fg, pv_bg = "#B08C00", "#FFF8E0"
        self.v_hpv.config(fg=pv_fg, bg=pv_bg)

        self.v_psp.config(text=f"{d.piezo_sp:.4f}")
        self.v_ppv.config(text=f"{d.piezo_pv:.4f}")

        self.v_pp.config(text=f"{d.pp:.6f}")
        self.v_pm.config(text=f"{d.pm:.6f}")
        self.v_rat.config(text=f"{d.ratio:.6f}")

        if d.lock_on:
            self.v_lstat.config(text="ACTIVO", fg=GREEN)
            self.v_lref.config(text=f"{d.lock_ref:.6f}")
            self.v_lband.config(text=f"[{d.lock_lo:.4f}, {d.lock_hi:.4f}]")
        else:
            self.v_lstat.config(text="INACTIVO", fg=GREY)
            self.v_lref.config(text="---")
            self.v_lband.config(text="---")
            if d.mode != "g":
                d.lock_on = False

        self.v_cyc.config(text=str(d.cycle))

    # -- grafico --

    def _tick_plot(self):
        if self.plot_t:
            self.ln_pp.set_data(list(self.plot_t), list(self.plot_pp))
            self.ln_pm.set_data(list(self.plot_t), list(self.plot_pm))
            self.ax.relim()
            self.ax.autoscale_view()
            self.canvas.draw_idle()
        self.root.after(1000, self._tick_plot)

    def _clear_plot(self):
        self.plot_t.clear()
        self.plot_pp.clear()
        self.plot_pm.clear()
        self.ln_pp.set_data([], [])
        self.ln_pm.set_data([], [])
        self.ax.relim()
        self.ax.autoscale_view()
        self.canvas.draw_idle()

    # -- envio de comandos --

    def _set_param(self, prefix, entry):
        v = entry.get().strip()
        if v:
            self._send(f"{prefix}{v}")
            entry.delete(0, tk.END)

    def _send(self, cmd):
        if not self.ser or not self.ser.is_open:
            self._log("No conectado\n", "error")
            return
        try:
            self.ser.write((cmd + "\n").encode("utf-8"))
            self._log(f"→ {cmd}\n", "tx")
        except serial.SerialException as e:
            self._log(f"Error de envío: {e}\n", "error")

    def _on_send(self, _evt=None):
        cmd = self.cmd_var.get().strip()
        if cmd:
            self._send(cmd)
            self.cmd_var.set("")

    # -- log (gui + archivo) --

    def _log(self, text, tag=None):
        clean = text.rstrip("\n")
        if clean:
            prefix = {"tx": "TX", "rx": "RX", "info": "INFO",
                      "error": "ERR"}.get(tag, "--")
            flog.info(f"[{prefix}] {clean}")

        self.logbox.config(state=tk.NORMAL)
        self.logbox.insert(tk.END, text, tag)
        self.logbox.see(tk.END)
        self.logbox.config(state=tk.DISABLED)

    def _clear_log(self):
        self.logbox.config(state=tk.NORMAL)
        self.logbox.delete("1.0", tk.END)
        self.logbox.config(state=tk.DISABLED)

    def _on_close(self):
        self.running = False
        if self.ser and self.ser.is_open:
            self.ser.close()
        flog.info("HMI cerrada")
        self.root.destroy()


def main():
    root = tk.Tk()
    HMI(root)
    root.mainloop()


if __name__ == "__main__":
    main()
