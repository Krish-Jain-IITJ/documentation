"""
Unicycle Curve-Following Simulation
BLF (Barrier Lyapunov Function) Control Law
Real-time PyQt5 + Matplotlib UI

Changes applied:
  - CHANGE 1b: Perpetual orbit — no time-based truncation in standalone mode
  - CHANGE 1b: Lap counter via (-r_d, 0) crossing detection; displayed as "Laps: N"
  - CHANGE 1b: (-r_d, 0) marked on trajectory plot with white star
  - CHANGE 1c: Rolling trajectory window — last 3 laps; current lap cyan, prev dim
  - CHANGE 4: Obstacle markersize fixed at 4 (no scaling)
  - BUG-A: Single random obstacle per lap in standalone mode (no fake static obstacles)
"""

import sys
import numpy as np
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QHBoxLayout, QVBoxLayout,
    QPushButton, QLabel, QSlider, QGroupBox, QGridLayout, QSplitter,
    QFrame, QSizePolicy
)
from PyQt5.QtCore import QTimer, Qt, pyqtSignal
from PyQt5.QtGui import QFont, QPalette, QColor

import matplotlib
matplotlib.use('Qt5Agg')
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.backends.backend_qt5agg import NavigationToolbar2QT as NavigationToolbar
from matplotlib.figure import Figure
import matplotlib.patches as patches
import matplotlib.patheffects as pe


# ─── Simulation Parameters ───────────────────────────────────────────────────
class SimParams:
    x0     = -0.5
    y0     = -1.0
    theta0 = -135 * np.pi / 180
    v      = 1.5   # speed (can be changed dynamically)
    r_d    = 0.7        # desired radius (circle)
    a      = 1.5        # ellipse semi-major
    b      = 1.25       # ellipse semi-minor
    kn     = 1.0        # normal gain
    kd     = 1.0        # heading gain
    dt     = 0.01       # time step
    w_max  = 2.0        # angular velocity saturation


# ─── Control Law (BLF — DO NOT MODIFY) ───────────────────────────────────────
def step(state, p: SimParams):
    x, y, theta = state
    v = p.v
    r   = np.array([x, y])
    m   = np.array([np.cos(theta), np.sin(theta)])
    E   = np.array([[0, 1], [-1, 0]])

    xdot = v * np.cos(theta)
    ydot = v * np.sin(theta)

    psi    = np.arctan2(y, x)
    norm_r = np.linalg.norm(r)
    psidot = (x * ydot - y * xdot) / (norm_r ** 2) if norm_r > 1e-9 else 0.0

    dx, dy = 2 * x, 2 * y
    dxx, dyy, dxy = 2.0, 2.0, 0.0
    H  = np.array([[dxx, dxy], [dxy, dyy]])
    n  = np.array([dx, dy])
    tau = E @ n

    M   = (p.b * np.cos(psi)) ** 2 + (p.a * np.sin(psi)) ** 2
    R   = p.a * p.b / np.sqrt(M)
    e   = norm_r - p.r_d
    del_  = R - p.r_d

    Rdot  = (-0.5 * p.a * p.b * np.sin(2 * psi) * (p.a ** 2 - p.b ** 2)
             / (M ** 1.5))
    Rddot = ((-p.a * p.b * (p.a ** 2 - p.b ** 2) * np.cos(2 * psi) / (M ** 1.5))
             + (3 * p.a * p.b * (p.a ** 2 - p.b ** 2) ** 2
                * np.sin(2 * psi) ** 2 / (4 * M ** 2.5)))

    edot  = v * (r @ m) / norm_r if norm_r > 1e-9 else 0.0

    if abs(del_) < 1e-9:
        del_ = 1e-9

    a_blf     = -p.kn * e / del_
    a_blfdot  = -p.kn * (edot * del_ - e * Rdot * psidot) / (del_ ** 2)

    a_comp = Rdot * e / (norm_r * del_) if norm_r > 1e-9 else 0.0
    if norm_r > 1e-9:
        num   = norm_r * del_ * (Rddot * psidot * e + Rdot * edot)
        denom_part = Rdot * e * (Rdot * psidot * norm_r
                                  + del_ * (r @ (v * m)) / norm_r)
        a_compdot = (num - denom_part) / (norm_r ** 2 * del_ ** 2)
    else:
        a_compdot = 0.0

    A    = a_blf + a_comp
    Adot = a_blfdot + a_compdot

    I       = np.eye(2)
    eta     = tau + A * n
    etadot  = E @ H @ (v * m) + Adot * n + v * A * H @ m
    norm_eta = np.linalg.norm(eta)
    if norm_eta < 1e-9:
        norm_eta = 1e-9
    md     = eta / norm_eta
    mddot  = (I / norm_eta - np.outer(eta, eta) / norm_eta ** 3) @ etadot

    w_d = -mddot @ (E @ md)

    m3  = np.array([m[0], m[1], 0.0])
    md3 = np.array([md[0], md[1], 0.0])
    Z   = np.cross(m3, md3)
    cross_mag = np.linalg.norm(np.cross(m3, md3))
    dot_val   = np.dot(m3, md3)
    gamma     = np.arctan2((Z[2] / (np.linalg.norm(Z) + 1e-12)) * cross_mag,
                            dot_val)

    w = w_d + p.kd * gamma
    if abs(w) >= p.w_max:
        w = np.sign(w) * p.w_max

    xn     = x + xdot * p.dt
    yn     = y + ydot * p.dt
    thetan = theta + w * p.dt

    extras = dict(
        e=e, gamma=gamma, w=w,
        tau=tau / (np.linalg.norm(tau) + 1e-12),
        n_unit=n / (np.linalg.norm(n) + 1e-12),
        m=m, del_=del_, R=R
    )
    return (xn, yn, thetan), extras


# ─── Matplotlib Canvas ────────────────────────────────────────────────────────
DARK_BG  = '#090a0f'
PANEL_BG = '#12151e'
ACCENT   = '#00f0ff'
ACCENT2  = '#ff003c'
GREEN    = '#00ff66'
YELLOW   = '#ffde00'
GREY     = '#5a6b8c'

def style_ax(ax, title='', xlabel='', ylabel=''):
    ax.set_facecolor(PANEL_BG)
    ax.tick_params(colors=GREY, labelsize=8)
    for spine in ax.spines.values():
        spine.set_edgecolor('#2a3040')
    ax.title.set_color(ACCENT)
    ax.title.set_fontsize(9)
    ax.title.set_fontweight('bold')
    ax.xaxis.label.set_color(GREY)
    ax.yaxis.label.set_color(GREY)
    ax.xaxis.label.set_fontsize(8)
    ax.yaxis.label.set_fontsize(8)
    if title:   ax.set_title(title)
    if xlabel:  ax.set_xlabel(xlabel)
    if ylabel:  ax.set_ylabel(ylabel)
    ax.grid(True, color='#1e2535', linewidth=0.5, linestyle=':')


class SimCanvas(FigureCanvas):
    def __init__(self):
        self.fig = Figure(facecolor=DARK_BG, tight_layout=True)
        super().__init__(self.fig)
        self.setMinimumSize(400, 300)
        self._build_axes()

    def _build_axes(self):
        gs = self.fig.add_gridspec(2, 2, width_ratios=[2.5, 1], hspace=0.45, wspace=0.25,
                                    left=0.05, right=0.97, top=0.92, bottom=0.1)
        self.ax_traj = self.fig.add_subplot(gs[:, 0])
        self.ax_e    = self.fig.add_subplot(gs[0, 1])
        self.ax_w    = self.fig.add_subplot(gs[1, 1])

        style_ax(self.ax_traj, 'TRAJECTORY', 'x (m)', 'y (m)')
        style_ax(self.ax_e,    'TRACKING ERROR  e(t)', 'Time (s)', 'e (m)')
        style_ax(self.ax_w,    'ANGULAR VELOCITY  ω(t)', 'Time (s)', 'ω (rad/s)')

        p = SimParams()
        rho = np.linspace(0, 2 * np.pi, 500)
        self.ax_traj.plot(p.r_d * np.cos(rho), p.r_d * np.sin(rho),
                          color=ACCENT, lw=4, alpha=0.15)
        self.ax_traj.plot(p.r_d * np.cos(rho), p.r_d * np.sin(rho),
                          color=ACCENT, lw=1.5, linestyle='--', label='Desired')
        self.ax_traj.plot(p.a * np.cos(rho), p.b * np.sin(rho),
                          color=ACCENT2, lw=1, linestyle=':', alpha=0.5, label='Ellipse bound')

        self.ax_traj.legend(fontsize=7, facecolor=DARK_BG, edgecolor=GREY,
                             labelcolor='white', loc='upper right')
        self.ax_traj.set_aspect('equal')
        self.ax_traj.set_xlim(-2.2, 2.2)
        self.ax_traj.set_ylim(-2.2, 2.2)

        # Two trajectory segments: previous laps (dull) and current lap (bright)
        self.traj_prev, = self.ax_traj.plot([], [], color='#1a5f7a', lw=0.8, alpha=0.4)
        self.traj_curr, = self.ax_traj.plot([], [], color='#00e5ff', lw=1.5, alpha=1.0)

        # Quadcopter graphics
        self.drone_arms, = self.ax_traj.plot([], [], color='#888888', lw=2.5, zorder=9)
        self.drone_body, = self.ax_traj.plot([], [], 'o', color='#111111', 
                                             markersize=10, markeredgecolor=ACCENT, 
                                             markeredgewidth=1.5, zorder=12)
        self.drone_rotors = []
        self.drone_blades = []
        for _ in range(4):
            r, = self.ax_traj.plot([], [], 'o', color=PANEL_BG, markeredgecolor='#444444', 
                                   markeredgewidth=1.5, markersize=14, alpha=0.9, zorder=8)
            b, = self.ax_traj.plot([], [], color=GREEN, lw=2, zorder=11)
            self.drone_rotors.append(r)
            self.drone_blades.append(b)

        self.action_text = self.ax_traj.text(0.02, 0.98, '', transform=self.ax_traj.transAxes,
                                            fontsize=10, verticalalignment='top',
                                            bbox=dict(boxstyle='round,pad=0.3', facecolor=PANEL_BG, edgecolor=ACCENT, alpha=0.8),
                                            color=ACCENT, weight='bold')

        self.q_heading = self.ax_traj.quiver([], [], [], [],
            color=GREEN, scale=5, width=0.006, headwidth=4, label='Heading m')
        self.q_normal  = self.ax_traj.quiver([], [], [], [],
            color=YELLOW, scale=5, width=0.005, headwidth=4, label='Normal n')
        self.q_tangent = self.ax_traj.quiver([], [], [], [],
            color=ACCENT2, scale=5, width=0.005, headwidth=4, label='Tangent τ')

        vec_legend = self.ax_traj.legend(
            handles=[self.q_heading, self.q_normal, self.q_tangent],
            fontsize=7, facecolor=DARK_BG, edgecolor=GREY, labelcolor='white',
            loc='lower right'
        )
        self.ax_traj.add_artist(vec_legend)

        self.e_line, = self.ax_e.plot([], [], color=ACCENT, lw=1.2)
        self.ax_e.axhline(0, color=GREY, lw=0.5, linestyle='--')
        self.w_line, = self.ax_w.plot([], [], color=ACCENT2, lw=1.2)
        self.ax_w.axhline(SimParams.w_max, color=GREY, lw=0.5, linestyle=':', alpha=0.5)
        self.ax_w.axhline(-SimParams.w_max, color=GREY, lw=0.5, linestyle=':', alpha=0.5)

        self.draw()

    def update_plots(self, xs, ys, ts, es, ws, extras, obstacles=None,
                     lap_start_idx=None):
        """
        Update all plots.
        lap_start_idx: index into xs/ys where the current lap started.
        """
        p = SimParams()
        
        # Previous laps (dull blue)
        if lap_start_idx is not None and lap_start_idx > 0:
            self.traj_prev.set_data(xs[:lap_start_idx], ys[:lap_start_idx])
            self.traj_curr.set_data(xs[lap_start_idx:], ys[lap_start_idx:])
        else:
            self.traj_prev.set_data([], [])
            self.traj_curr.set_data(xs, ys)

        x, y = xs[-1], ys[-1]
        theta = ts[-1]
        
        # Quadcopter updates
        L = 0.22
        arm_angles = [np.pi/4, 3*np.pi/4, 5*np.pi/4, 7*np.pi/4]
        arm_xs, arm_ys = [], []
        rotor_centers = []
        for a in arm_angles:
            rx = x + L * np.cos(theta + a)
            ry = y + L * np.sin(theta + a)
            arm_xs.extend([x, rx, None])
            arm_ys.extend([y, ry, None])
            rotor_centers.append((rx, ry))
            
        self.drone_arms.set_data(arm_xs, arm_ys)
        self.drone_body.set_data([x], [y])
        
        t_elapsed = len(xs) * p.dt
        spin_rate = 250.0
        blade_L = 0.1
        for i, (rx, ry) in enumerate(rotor_centers):
            self.drone_rotors[i].set_data([rx], [ry])
            blade_angle = t_elapsed * spin_rate * (1 if i % 2 == 0 else -1)
            bx1 = rx + blade_L * np.cos(blade_angle)
            by1 = ry + blade_L * np.sin(blade_angle)
            bx2 = rx - blade_L * np.cos(blade_angle)
            by2 = ry - blade_L * np.sin(blade_angle)
            self.drone_blades[i].set_data([bx1, bx2], [by1, by2])

        # Vectors
        sc = 0.35
        m   = extras['m']
        n_u = extras['n_unit']
        tau = extras['tau']
        self.q_heading.set_offsets([[x, y]])
        self.q_heading.set_UVC([m[0] * sc], [m[1] * sc])
        self.q_normal.set_offsets([[x, y]])
        self.q_normal.set_UVC([n_u[0] * sc], [n_u[1] * sc])
        self.q_tangent.set_offsets([[x, y]])
        self.q_tangent.set_UVC([tau[0] * sc], [tau[1] * sc])

        # Auto-scroll time series (last 30 s)
        win = 30.0
        dt  = SimParams.dt
        t_arr = np.arange(len(es)) * dt
        mask  = t_arr >= (t_arr[-1] - win) if len(t_arr) > 1 else slice(None)

        self.e_line.set_data(t_arr[mask], np.array(es)[mask])
        self.w_line.set_data(t_arr[mask], np.array(ws)[mask])

        for ax in (self.ax_e, self.ax_w):
            ax.relim()
            ax.autoscale_view()

        w = extras.get('w', 0.0)
        self.action_text.set_text(
            f'ω: {w:.3f} rad/s'
        )

        self.draw_idle()


# ─── Main Window ─────────────────────────────────────────────────────────────
class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle('Unicycle BLF Simulation')
        self.setMinimumSize(1100, 700)
        self._apply_dark_theme()

        self.p = SimParams()
        self._reset_state()

        self._build_ui()
        self.timer = QTimer()
        self.timer.timeout.connect(self._tick)

    def _reset_state(self):
        p = self.p
        self.state = (p.x0, p.y0, p.theta0)
        self.xs, self.ys, self.ts = [p.x0], [p.y0], [p.theta0]
        self.es, self.ws = [], []
        self.extras_last = dict(
            m=np.array([np.cos(p.theta0), np.sin(p.theta0)]),
            n_unit=np.array([1.0, 0.0]),
            tau=np.array([0.0, 1.0]),
        )
        self.t_elapsed  = 0.0
        self.running    = False

        # Lap tracking
        self.lap_count       = 0
        self.lap_start_idx   = 0   # index into xs/ys where current lap began
        self._prev_angle_sim = None

    def _build_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        root = QHBoxLayout(central)
        root.setContentsMargins(8, 8, 8, 8)
        root.setSpacing(8)

        left = QWidget()
        lv   = QVBoxLayout(left)
        lv.setContentsMargins(0, 0, 0, 0)

        self.canvas = SimCanvas()
        
        navbar_frame = QFrame()
        navbar_frame.setStyleSheet(f'background-color:{PANEL_BG}; border-radius:8px; border: 1px solid #1e2535;')
        navbar_frame.setFixedHeight(45)
        nav_layout = QHBoxLayout(navbar_frame)
        nav_layout.setContentsMargins(10, 0, 10, 0)
        
        toolbar = NavigationToolbar(self.canvas, self)
        toolbar.setStyleSheet(f'''
            QToolBar {{ background: transparent; border: none; spacing: 8px; }}
            QToolButton {{ background: transparent; border-radius: 4px; padding: 4px; }}
            QToolButton:hover {{ background: #2a3040; }}
        ''')
        nav_layout.addWidget(toolbar)
        nav_layout.addStretch()
        
        lv.addWidget(navbar_frame)
        lv.addWidget(self.canvas)
        root.addWidget(left, stretch=5)

        self.panel = self._build_panel()
        root.addWidget(self.panel, stretch=1)

    def _build_panel(self):
        panel = QFrame()
        panel.setFixedWidth(230)
        panel.setStyleSheet(f'background:{PANEL_BG}; border-radius:8px;')
        vl = QVBoxLayout(panel)
        vl.setContentsMargins(12, 14, 12, 14)
        vl.setSpacing(10)

        title = QLabel('<span style="color:#ffffff;">PLUTO</span> <span style="color:#00f0ff;">SIM</span>')
        title.setAlignment(Qt.AlignCenter)
        title.setFont(QFont('Segoe UI Black', 16, QFont.Bold))
        title.setTextFormat(Qt.RichText)
        vl.addWidget(title)
        vl.addWidget(self._hline())

        self.lbl_time = self._stat_label('Time', '0.00 s')
        self.lbl_e    = self._stat_label('Error e', '—')
        self.lbl_w    = self._stat_label('ω', '—')
        self.lbl_r    = self._stat_label('R(ψ)', '—')
        # CHANGE 1b: Laps label
        self.lbl_laps = self._stat_label('Laps', '0')
        for w in (self.lbl_time, self.lbl_e, self.lbl_w, self.lbl_r, self.lbl_laps):
            vl.addWidget(w)

        vl.addWidget(self._hline())

        vl.addWidget(self._section('PARAMETERS'))
        self.sl_kn = self._slider('kn', 0.1, 5.0, SimParams.kn)
        self.sl_kd = self._slider('kd', 0.1, 5.0, SimParams.kd)
        self.sl_v  = self._slider('v  (m/s)', 0.1, 5.0, SimParams.v)
        for grp in (self.sl_kn, self.sl_kd, self.sl_v):
            vl.addWidget(grp)

        vl.addWidget(self._hline())

        self.btn_start = self._button('▶  START', ACCENT, '#001f26')
        self.btn_stop  = self._button('■  STOP', ACCENT2, '#26001a')
        self.btn_reset = self._button('↺  RESET', YELLOW, '#1a1500')
        self.btn_stop.setEnabled(False)

        self.btn_start.clicked.connect(self._start)
        self.btn_stop.clicked.connect(self._stop)
        self.btn_reset.clicked.connect(self._reset)

        for b in (self.btn_start, self.btn_stop, self.btn_reset):
            vl.addWidget(b)

        vl.addStretch()

        vl.addWidget(self._section('VECTOR LEGEND'))
        for color, label in [(GREEN, 'Heading  m'),
                              (YELLOW, 'Normal   n'),
                              (ACCENT2, 'Tangent  τ'),
                              (ACCENT, '— Desired circle'),
                              ('#4fc3f7', '— Current lap'),
                              ('#1a5f7a', '— Previous laps')]:
            vl.addWidget(self._legend_item(color, label))

        return panel

    def _hline(self):
        line = QFrame()
        line.setFrameShape(QFrame.HLine)
        line.setStyleSheet(f'color: #2a3040;')
        return line

    def _section(self, text):
        lbl = QLabel(text)
        lbl.setFont(QFont('Courier New', 7, QFont.Bold))
        lbl.setStyleSheet(f'color:{GREY}; letter-spacing:2px;')
        return lbl

    def _stat_label(self, name, val):
        w = QLabel(f'{name}:  {val}')
        w.setFont(QFont('Courier New', 9))
        w.setStyleSheet(f'color:{ACCENT};')
        return w

    def _slider(self, name, lo, hi, init):
        grp = QGroupBox(name)
        grp.setStyleSheet(f'''
            QGroupBox {{ color:{GREY}; font-size:8px; font-family:Courier New;
                         border:1px solid #2a3040; border-radius:4px; margin-top:6px;}}
            QGroupBox::title {{ subcontrol-origin:margin; left:6px;}}
        ''')
        vl = QVBoxLayout(grp)
        vl.setContentsMargins(4, 8, 4, 4)
        sl = QSlider(Qt.Horizontal)
        sl.setMinimum(0)
        sl.setMaximum(100)
        sl.setValue(int((init - lo) / (hi - lo) * 100))
        sl.setStyleSheet(f'''
            QSlider::groove:horizontal {{height:4px; background:#2a3040; border-radius:2px;}}
            QSlider::handle:horizontal {{width:12px; height:12px; margin:-4px 0;
                background:{ACCENT}; border-radius:6px;}}
            QSlider::sub-page:horizontal {{background:{ACCENT}; border-radius:2px;}}
        ''')
        lbl = QLabel(f'{init:.2f}')
        lbl.setFont(QFont('Courier New', 8))
        lbl.setStyleSheet(f'color:{ACCENT};')
        lbl.setAlignment(Qt.AlignRight)

        def _update(v):
            val = lo + (v / 100) * (hi - lo)
            lbl.setText(f'{val:.2f}')

        sl.valueChanged.connect(_update)
        vl.addWidget(sl)
        vl.addWidget(lbl)
        grp._slider = sl
        grp._lo, grp._hi = lo, hi
        grp._lbl = lbl
        return grp

    def _get_slider_val(self, grp):
        v = grp._slider.value()
        return grp._lo + (v / 100) * (grp._hi - grp._lo)

    def _button(self, text, fg, bg):
        b = QPushButton(text)
        b.setFixedHeight(36)
        b.setFont(QFont('Courier New', 9, QFont.Bold))
        b.setStyleSheet(f'''
            QPushButton {{
                background:{bg}; color:{fg}; border:1.5px solid {fg};
                border-radius:6px; letter-spacing:1px;
            }}
            QPushButton:hover {{ background:{fg}; color:#000; }}
            QPushButton:disabled {{ border-color:#2a3040; color:#2a3040; background:{DARK_BG}; }}
        ''')
        return b

    def _legend_item(self, color, label):
        w = QLabel(f'<span style="color:{color}; font-size:14px;">■</span>'
                   f' <span style="color:{GREY}; font-size:8px;">{label}</span>')
        w.setTextFormat(Qt.RichText)
        return w

    def _start(self):
        self.p.kn = self._get_slider_val(self.sl_kn)
        self.p.kd = self._get_slider_val(self.sl_kd)
        self.p.v  = self._get_slider_val(self.sl_v)
        self.running = True
        self.btn_start.setEnabled(False)
        self.btn_stop.setEnabled(True)
        self.timer.start(int(self.p.dt * 1000))

    def _stop(self):
        self.timer.stop()
        self.running = False
        self.btn_start.setEnabled(True)
        self.btn_stop.setEnabled(False)

    def _reset(self):
        self._stop()
        self._reset_state()
        self.canvas._build_axes()
        self.lbl_time.setText('Time:  0.00 s')
        self.lbl_e.setText('Error e:  —')
        self.lbl_w.setText('ω:  —')
        self.lbl_r.setText('R(ψ):  —')
        self.lbl_laps.setText('Laps:  0')

    def _tick(self):
        """CHANGE 1b: perpetual orbit — no time-based truncation."""
        # Allow dynamic parameter updates
        self.p.kn = self._get_slider_val(self.sl_kn)
        self.p.kd = self._get_slider_val(self.sl_kd)
        self.p.v  = self._get_slider_val(self.sl_v)

        state, extras = step(self.state, self.p)
        self.state = state
        x, y, theta = state

        self.xs.append(x)
        self.ys.append(y)
        self.ts.append(theta)
        self.es.append(extras['e'])
        self.ws.append(extras['w'])
        self.extras_last = extras
        self.t_elapsed += self.p.dt

        # CHANGE 1b: lap crossing detection at (-r_d, 0)
        angle_new = np.arctan2(y, x)
        on_circle = abs(np.hypot(x, y) - self.p.r_d) < 0.15
        if self._prev_angle_sim is not None:
            crossed = (
                (self._prev_angle_sim > np.pi * 0.85 and angle_new < -np.pi * 0.85) or
                (abs(angle_new - np.pi) < 0.08 and on_circle)
            )
            if crossed and on_circle:
                self.lap_count += 1
                self.lap_start_idx = len(self.xs) - 1
                self.lbl_laps.setText(f'Laps:  {self.lap_count}')
        self._prev_angle_sim = angle_new

        # Update stats every 10 ticks
        if len(self.es) % 10 == 0:
            self.lbl_time.setText(f'Time:  {self.t_elapsed:.2f} s')
            self.lbl_e.setText(f'Error e:  {extras["e"]:.4f} m')
            self.lbl_w.setText(f'ω:  {extras["w"]:.4f} r/s')
            self.lbl_r.setText(f'R(ψ):  {extras["R"]:.4f} m')

        self.canvas.update_plots(
            self.xs, self.ys, self.ts,
            self.es, self.ws, extras,
            obstacles=None,
            lap_start_idx=self.lap_start_idx,
        )

    def _apply_dark_theme(self):
        self.setStyleSheet(f'''
            QMainWindow, QWidget {{ background: {DARK_BG}; color: white; }}
            QScrollBar {{ background: {PANEL_BG}; }}
        ''')


# ─── Entry ────────────────────────────────────────────────────────────────────
if __name__ == '__main__':
    app = QApplication(sys.argv)
    app.setStyle('Fusion')
    pal = QPalette()
    pal.setColor(QPalette.Window, QColor(DARK_BG))
    pal.setColor(QPalette.WindowText, Qt.white)
    pal.setColor(QPalette.Base, QColor(PANEL_BG))
    pal.setColor(QPalette.AlternateBase, QColor('#1a2035'))
    pal.setColor(QPalette.ToolTipBase, Qt.white)
    pal.setColor(QPalette.ToolTipText, Qt.white)
    pal.setColor(QPalette.Text, Qt.white)
    pal.setColor(QPalette.Button, QColor(PANEL_BG))
    pal.setColor(QPalette.ButtonText, Qt.white)
    pal.setColor(QPalette.Highlight, QColor(ACCENT))
    pal.setColor(QPalette.HighlightedText, Qt.black)
    app.setPalette(pal)

    win = MainWindow()
    win.show()
    sys.exit(app.exec_())
