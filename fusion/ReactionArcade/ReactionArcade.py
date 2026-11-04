#Author: Reaction Arcade enclosure generator
#Description: Builds the complete printable enclosure for the "Reaction Arcade" kit.
"""
REACTION ARCADE - complete enclosure, Fusion 360 Python script
==============================================================

Run from Utilities > Scripts and Add-Ins (green "+" > pick this folder).
The script always opens a NEW design document and builds everything there.

Parts (one component each, all occurrences at the origin, assembled in place)
  BASE            dark grey  - floor + 4 walls cut at the 15 deg slope, 4 corner
                               pillars (axis perpendicular to the panel) with
                               M3 heat-set inserts, ESP32 cradle, buzzer cradle +
                               grille, USB cutout, cable-tie loops, foot recesses.
  TOP_PANEL       off-white  - flat sloped plate with a 1.5 mm alignment lip,
                               display window/pocket/clips, LED-ring seat/holder,
                               4 button holes, 4 counterbored M3 holes, title.
  DIFFUSER        translucent- 1.2 mm press-fit disc over the LED ring.
  FIT_TEST        off-white  - quick coupon to test the critical fits.
  Reference parts light grey - REF_* placeholder electronics (do not print).

Print orientation (also repeated at each builder):
  BASE      : floor (Z=0 face) on the bed. No supports. 0.5 mm bed chamfer.
  TOP_PANEL : visible top face DOWN on the bed (smooth face). No supports.
  DIFFUSER  : visible top face down on the bed.
  FIT_TEST  : plate bottom on the bed (seat/button hole on the bed side like the
              panel; pillar stub and USB wall printed upright like the base).

Parameters
  Every key dimension is a Fusion User Parameter (Modify > Change Parameters).
  Feature depths/thicknesses/fillets/chamfers reference the parameters by name
  and update live. Layout positions are computed from the same parameters when
  the script runs: to apply layout changes, edit the parameters and run the
  script again with that design active - it rebuilds a new design from the
  current parameter values (or from the PARAMS table below for a fresh start).

Coordinates
  Model: mm, Z up, floor bottom at Z=0, X = width (+X = player's right),
  Y = depth (-Y = front/player side, +Y = back wall).
  Panel: (u, v, w) with origin at the centre of the panel's top face,
  u = X, v = up the slope toward the back, w = along the panel normal (w<0 is
  below the visible face). Panel layout values (disp_y, ring_y, title_y) are v.
"""

import adsk.core
import adsk.fusion
import math
import traceback

# =============================================================================
# 1. PARAMETERS  (name, default expression, units, comment)
# =============================================================================
PARAMS = [
    # --- overall shape ------------------------------------------------------
    ('box_w', '170 mm', 'mm', 'Footprint width (X)'),
    ('box_d', '130 mm', 'mm', 'Footprint depth (Y)'),
    ('h_back', '70 mm', 'mm', 'Outer height at the back wall (top of panel)'),
    ('h_front', '35 mm', 'mm', 'Outer height at the front wall (top of panel)'),
    ('r_corner', '10 mm', 'mm', 'Vertical corner radius'),
    ('top_fillet', '3 mm', 'mm', 'Top perimeter round (bed-safe on TOP_PANEL)'),
    ('panel_t', '3 mm', 'mm', 'TOP_PANEL plate thickness'),
    ('lip_h', '1.5 mm', 'mm', 'Alignment lip drop below the panel underside'),
    ('lip_w', '2 mm', 'mm', 'Alignment lip wall thickness'),
    ('floor_t', '2.5 mm', 'mm', 'BASE floor thickness'),
    ('wall_t', '2.4 mm', 'mm', 'BASE wall thickness'),
    # --- fits / print rules ---------------------------------------------------
    ('fit', '0.3 mm', 'mm', 'Slip-fit clearance (added to pocket size; per side for the lip)'),
    ('press', '0.1 mm', 'mm', 'Press-fit clearance (added to seat diameter)'),
    ('bed_chamfer', '0.5 mm', 'mm', 'Chamfer on bed edges'),
    # --- fasteners --------------------------------------------------------------
    ('m3_clear', '3.2 mm', 'mm', 'M3 screw clearance hole'),
    ('insert_d', '4.0 mm', 'mm', 'M3 heat-set insert hole diameter'),
    ('insert_depth', '5.5 mm', 'mm', 'M3 heat-set insert hole depth'),
    ('pillar_d', '8.5 mm', 'mm', 'Corner pillar diameter'),
    ('pad_d', '10 mm', 'mm', 'Screw boss pad diameter under the panel'),
    ('pad_t', '2 mm', 'mm', 'Screw boss pad height under the panel'),
    ('cb_d', '6.4 mm', 'mm', 'Counterbore diameter (M3 button head 5.7)'),
    ('cb_depth', '1.9 mm', 'mm', 'Counterbore depth (button head 1.65)'),
    # --- display (0.56in 4-digit 7-seg + HT16K33 backpack) -----------------------
    ('disp_win_w', '46 mm', 'mm', 'Display window width'),
    ('disp_win_h', '17 mm', 'mm', 'Display window height'),
    ('disp_y', '38 mm', 'mm', 'Display centre, panel v coordinate'),
    ('disp_w', '50.5 mm', 'mm', 'Display body width'),
    ('disp_h', '19.2 mm', 'mm', 'Display body height'),
    ('disp_body_t', '8 mm', 'mm', 'Display body depth (face to backpack PCB)'),
    ('disp_pcb_t', '1.6 mm', 'mm', 'Backpack PCB thickness'),
    ('disp_frame', '1.2 mm', 'mm', 'Window frame thickness in front of the display'),
    ('clip_t', '1.2 mm', 'mm', 'Display clip thickness'),
    ('clip_w', '8 mm', 'mm', 'Display clip width'),
    ('clip_hook', '0.6 mm', 'mm', 'Display clip hook overlap'),
    # --- LED ring + diffuser --------------------------------------------------
    ('ring_od', '32 mm', 'mm', 'WS2812 ring outer diameter'),
    ('ring_id', '18 mm', 'mm', 'WS2812 ring inner diameter'),
    ('ring_t', '1.6 mm', 'mm', 'WS2812 ring PCB thickness'),
    ('ring_y', '-5 mm', 'mm', 'Ring centre, panel v coordinate'),
    ('ring_gap', '4 mm', 'mm', 'Gap diffuser underside -> ring PCB (light mixing)'),
    ('diff_t', '1.2 mm', 'mm', 'Diffuser thickness'),
    ('diff_d', 'ring_od + 6 mm', 'mm', 'Diffuser diameter'),
    ('tube_t', '1.6 mm', 'mm', 'Ring holder tube wall'),
    ('rclip_t', '1 mm', 'mm', 'Ring clip thickness'),
    ('rclip_w', '5 mm', 'mm', 'Ring clip width'),
    ('rclip_hook', '0.35 mm', 'mm', 'Ring clip hook overlap'),
    # --- buttons ------------------------------------------------------------
    ('btn_hole', '16.2 mm', 'mm', 'Button panel hole'),
    ('btn_r', '45 mm', 'mm', 'Button radius from the ring centre'),
    ('btn_ang', '45 deg', 'deg', 'Button diagonal angle from the u axis'),
    ('btn_depth', '26 mm', 'mm', 'Button body length below the panel'),
    ('btn_body_d', '16 mm', 'mm', 'Button body diameter (REF)'),
    ('btn_nut_d', '20 mm', 'mm', 'Button nut diameter (REF + keep-out)'),
    ('btn_bezel_d', '19 mm', 'mm', 'Button bezel diameter on top (REF)'),
    # --- title ----------------------------------------------------------------
    ('title_h', '8 mm', 'mm', 'Title text height'),
    ('title_depth', '0.6 mm', 'mm', 'Title engraving depth'),
    ('title_y', '-56 mm', 'mm', 'Title centre, panel v coordinate'),
    # --- ESP32 DevKit cradle --------------------------------------------------
    ('esp_l', '51.5 mm', 'mm', 'ESP32 board length'),
    ('esp_w', '28.5 mm', 'mm', 'ESP32 board width'),
    ('esp_t', '1.6 mm', 'mm', 'ESP32 board thickness'),
    ('esp_clear', '4 mm', 'mm', 'Clearance under the ESP32 board'),
    ('esp_x', '0 mm', 'mm', 'ESP32 board centre X'),
    ('esp_gap', '1 mm', 'mm', 'Gap board USB edge -> back wall inner face'),
    ('rail_t', '2 mm', 'mm', 'ESP32 guide rail thickness'),
    ('esp_snap', '0.6 mm', 'mm', 'ESP32 snap lip overlap'),
    # --- USB ------------------------------------------------------------------
    ('usb_w', '12 mm', 'mm', 'USB cutout width'),
    ('usb_h', '8 mm', 'mm', 'USB cutout height'),
    ('usb_chamfer', '1 mm', 'mm', 'USB cutout chamfered frame'),
    ('usb_conn_h', '3 mm', 'mm', 'USB receptacle height on the board (sets cutout Z)'),
    # --- buzzer + grille --------------------------------------------------------
    ('buz_d', '12.2 mm', 'mm', 'Buzzer pocket diameter (12 mm buzzer)'),
    ('buz_len', '9.5 mm', 'mm', 'Buzzer body length'),
    ('buz_x', '45 mm', 'mm', 'Buzzer centre X (against the back wall)'),
    ('slot_w', '2 mm', 'mm', 'Grille slot width'),
    ('slot_l', '14 mm', 'mm', 'Grille slot length'),
    ('slot_n', '5', '', 'Number of grille slots'),
    ('slot_pitch', '4 mm', 'mm', 'Grille slot pitch'),
    # --- floor details ------------------------------------------------------
    ('tie_w', '4.5 mm', 'mm', 'Cable-tie slot width'),
    ('tie_h', '2.2 mm', 'mm', 'Cable-tie slot height'),
    ('foot_d', '10 mm', 'mm', 'Rubber foot recess diameter'),
    ('foot_depth', '1 mm', 'mm', 'Rubber foot recess depth'),
    ('foot_inset', '15 mm', 'mm', 'Foot centre inset from the outer edges'),
    # --- fit test coupon --------------------------------------------------------
    ('ft_w', '60 mm', 'mm', 'FIT_TEST coupon width (grown if features need it)'),
    ('ft_d', '40 mm', 'mm', 'FIT_TEST coupon depth (grown if features need it)'),
    ('ft_t', '3 mm', 'mm', 'FIT_TEST coupon thickness'),
    # --- derived (read only, for reference) --------------------------------------
    ('slope_ang', 'atan((h_back - h_front) / box_d)', 'deg', 'Panel slope (derived)'),
]

MM = 0.1          # Fusion internal length unit is cm
EPS = 1e-6
TITLE_TEXT = 'REACTION'

C_BASE = (0x2B, 0x2D, 0x31)
C_PANEL = (0xED, 0xED, 0xE8)
C_DIFF = (0xF4, 0xF4, 0xF2)
C_REF = (0xC8, 0xC8, 0xC8)


# =============================================================================
# 2. REPORT / LOGGING
# =============================================================================
class Report(object):
    def __init__(self):
        self.lines = []
        self.warnings = []
        self.parts = []          # (name, ok, note)
        self.validation = []

    def info(self, msg):
        self.lines.append(msg)

    def warn(self, msg):
        self.warnings.append(msg)
        self.lines.append('WARNING: ' + msg)

    def part(self, name, ok, note=''):
        self.parts.append((name, ok, note))

    def val(self, msg):
        self.validation.append(msg)

    def text(self, full=True):
        out = ['REACTION ARCADE build summary', '=' * 40]
        for name, ok, note in self.parts:
            out.append('{:<16} {}{}'.format(name, 'BUILT ' if ok else 'FAILED',
                                            ('  ' + note) if note else ''))
        if self.validation:
            out.append('')
            out.append('VALIDATION')
            out.extend('  ' + v for v in self.validation)
        if self.warnings:
            out.append('')
            out.append('WARNINGS ({})'.format(len(self.warnings)))
            shown = self.warnings if full else self.warnings[:12]
            out.extend('  - ' + w for w in shown)
            if len(shown) < len(self.warnings):
                out.append('  ... {} more in the Text Commands window'.format(
                    len(self.warnings) - len(shown)))
        return '\n'.join(out)


def log_out(app, msg):
    """Write to the Text Commands window (app.log) and stdout."""
    try:
        app.log(msg)
    except Exception:
        pass
    try:
        print(msg)
    except Exception:
        pass


def short_err(e):
    s = str(e).strip().splitlines()
    return s[-1][:160] if s else e.__class__.__name__


def step(ctx, label, fn, *args, **kw):
    """Run an optional modelling step; on failure log a warning and go on."""
    try:
        return fn(*args, **kw)
    except Exception as e:
        ctx.rep.warn('{}: skipped ({})'.format(label, short_err(e)))
        return None


# =============================================================================
# 3. PURE-PYTHON GEOMETRY (no Fusion calls)  - all values in mm / radians
# =============================================================================
class Vals(object):
    """Parameter values in mm (lengths), degrees (angles), plain numbers."""
    pass


def dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0])


def neg(a):
    return (-a[0], -a[1], -a[2])


class Frame(object):
    """Panel frame. P(u, v, w) -> model (x, y, z) in mm."""

    def __init__(self, p):
        self.th = math.atan2(p.h_back - p.h_front, p.box_d)
        self.sn = math.sin(self.th)
        self.cs = math.cos(self.th)
        self.zc = (p.h_back + p.h_front) / 2.0   # panel centre height (top face)
        self.n = (0.0, -self.sn, self.cs)         # panel normal (out of top face)
        self.vd = (0.0, self.cs, self.sn)         # up-slope direction
        self.ud = (1.0, 0.0, 0.0)
        self.d_top = self.zc * self.cs            # origin -> top plane along n
        self.half_len = (p.box_d / 2.0) / self.cs  # top face half length along v

    def P(self, u, v, w=0.0):
        return (u, v * self.cs - w * self.sn, self.zc + v * self.sn + w * self.cs)

    def uvw(self, x, y, z):
        dz = z - self.zc
        return (x, y * self.cs + dz * self.sn, -y * self.sn + dz * self.cs)

    def v_at_y(self, y, w):
        """Panel v of the point on plane w whose model Y is y."""
        return (y + w * self.sn) / self.cs


class Layout(object):
    """Every derived position, computed once from the parameters."""

    def __init__(self, p, fr):
        self.p = p
        self.fr = fr
        # parting plane = panel underside; pillar tops sit under the pads
        self.w_part = -p.panel_t
        self.w_pillar = -(p.panel_t + p.pad_t)
        self.w_lip = -(p.panel_t + p.lip_h)
        # inner wall faces
        self.xi = p.box_w / 2.0 - p.wall_t
        self.yi = p.box_d / 2.0 - p.wall_t
        self.ri = max(p.r_corner - p.wall_t, 0.0)
        # corner pillars: centred on the inner corner arc centres at pillar top
        cx = p.box_w / 2.0 - p.r_corner
        cy = p.box_d / 2.0 - p.r_corner
        self.pillar_xy = [(sx * cx, sy * cy) for sy in (1, -1) for sx in (-1, 1)]
        self.pillars = [(x, fr.v_at_y(y, self.w_pillar)) for (x, y) in self.pillar_xy]
        # buttons on the diagonals around the ring
        a = math.radians(p.btn_ang)
        names = ['BTN_BACK_R', 'BTN_BACK_L', 'BTN_FRONT_L', 'BTN_FRONT_R']
        angs = [a, math.pi - a, math.pi + a, 2 * math.pi - a]
        self.buttons = [(nm, p.btn_r * math.cos(t), p.ring_y + p.btn_r * math.sin(t))
                        for nm, t in zip(names, angs)]
        # LED ring stack (w values)
        self.w_diff_bot = -p.diff_t
        self.w_ring_top = -(p.diff_t + p.ring_gap)
        self.w_ring_bot = self.w_ring_top - p.ring_t
        self.r_tube_in = (p.ring_od + p.fit) / 2.0
        self.r_tube_out = self.r_tube_in + p.tube_t
        # display stack
        self.w_disp_face = -p.disp_frame
        self.w_disp_back = self.w_disp_face - p.disp_body_t
        self.w_disp_pcb = self.w_disp_back - p.disp_pcb_t
        # ESP32 cradle (board long axis along Y, USB end toward +Y)
        self.esp_cy = self.yi - p.esp_gap - p.esp_l / 2.0
        self.z_pcb_bot = p.floor_t + p.esp_clear
        self.z_pcb_top = self.z_pcb_bot + p.esp_t
        self.usb_z = self.z_pcb_top + p.usb_conn_h / 2.0
        self.esp_clip_y = [self.esp_cy - 12.0, self.esp_cy + 12.0]
        # buzzer (axis along Y, face against the back wall)
        self.buz_z = p.floor_t + 1.6 + p.buz_d / 2.0
        # cable-tie loops and feet
        self.ties = [(-30.0, -10.0), (30.0, -10.0)]
        fx = p.box_w / 2.0 - p.foot_inset
        fy = p.box_d / 2.0 - p.foot_inset
        self.feet = [(sx * fx, sy * fy) for sx in (-1, 1) for sy in (-1, 1)]

    def pillar_top_z(self, i):
        u, v = self.pillars[i]
        return self.fr.P(u, v, self.w_pillar)[2]


# ---- 2D helpers for the layout checks (panel plane) --------------------------
def dist_pt_rect(px, py, x0, y0, x1, y1):
    dx = max(x0 - px, 0.0, px - x1)
    dy = max(y0 - py, 0.0, py - y1)
    return math.hypot(dx, dy)


def rect_c(cx, cy, w, h):
    return (cx - w / 2.0, cy - h / 2.0, cx + w / 2.0, cy + h / 2.0)


def circle_rect_gap(cx, cy, r, rc):
    """>0: clear gap, <0: overlap depth (approx)."""
    return dist_pt_rect(cx, cy, rc[0], rc[1], rc[2], rc[3]) - r


def button_floor_clearance(p, fr, u, v):
    """Lowest point of the button body above the floor top (mm)."""
    w_bot = -(p.panel_t + p.btn_depth)
    zc = fr.P(u, v, w_bot)[2]
    return zc - (p.btn_body_d / 2.0) * fr.sn - p.floor_t


def layout_checks(p, fr, lay, disp_y=None, btn_ang=None):
    """Returns a list of (severity, text) for the panel layout.

    severity: 'ERR' = geometry conflict, 'WARN' = below the 2 mm rule.
    disp_y / btn_ang overrides are used to search for a fix.
    """
    dy = p.disp_y if disp_y is None else disp_y
    a = math.radians(p.btn_ang if btn_ang is None else btn_ang)
    angs = [a, math.pi - a, math.pi + a, 2 * math.pi - a]
    btns = [(p.btn_r * math.cos(t), p.ring_y + p.btn_r * math.sin(t)) for t in angs]
    out = []
    rh = p.btn_hole / 2.0
    rn = p.btn_nut_d / 2.0
    rb = p.btn_bezel_d / 2.0
    win = rect_c(0.0, dy, p.disp_win_w, p.disp_win_h)
    pocket = rect_c(0.0, dy, p.disp_w + p.fit, p.disp_h + p.fit)
    body = rect_c(0.0, dy, p.disp_w, p.disp_h)
    xin = (p.disp_w + p.fit) / 2.0
    clips = [(-xin - p.clip_t, dy - p.clip_w / 2.0, -xin, dy + p.clip_w / 2.0),
             (xin, dy - p.clip_w / 2.0, xin + p.clip_t, dy + p.clip_w / 2.0)]
    ring_r = lay.r_tube_out + 0.5
    seat_r = (p.diff_d + p.press) / 2.0
    tw = 0.75 * p.title_h * len(TITLE_TEXT)
    title = rect_c(0.0, p.title_y, tw, p.title_h)
    edge_u = p.box_w / 2.0 - p.wall_t - p.fit - p.lip_w
    edge_v_hi = fr.v_at_y(lay.yi - p.fit - p.lip_w, lay.w_part)
    edge_v_lo = fr.v_at_y(-(lay.yi - p.fit - p.lip_w), lay.w_part)
    for i, (u, v) in enumerate(btns):
        nm = lay.buttons[i][0]
        g = circle_rect_gap(u, v, rh, win)
        if g < 2.0:
            out.append(('ERR' if g < 0 else 'WARN',
                        '{} hole -> display window web {:.1f} mm'.format(nm, g)))
        g = circle_rect_gap(u, v, rh, pocket)
        if g < 0:
            out.append(('ERR', '{} hole cuts into the display pocket by {:.1f} mm'.format(nm, -g)))
        g = circle_rect_gap(u, v, rb, win)
        if g < 0:
            out.append(('ERR', '{} bezel overlaps the display window by {:.1f} mm'.format(nm, -g)))
        g = min(circle_rect_gap(u, v, rn, body), min(circle_rect_gap(u, v, rn, c) for c in clips))
        if g < 0.5:
            out.append(('ERR', '{} nut/body collides with the display module ({:.1f} mm)'.format(nm, g)))
        d = math.hypot(u, v - p.ring_y)
        if d - rn - ring_r < 0.5:
            out.append(('ERR', '{} nut collides with the LED ring holder'.format(nm)))
        if d - rh - seat_r < 2.0:
            out.append(('WARN', '{} hole -> diffuser seat web {:.1f} mm'.format(nm, d - rh - seat_r)))
        g = circle_rect_gap(u, v, rb, title)
        if g < 0:
            out.append(('ERR', '{} bezel overlaps the title'.format(nm)))
        if abs(u) + rn > edge_u or v + rn > edge_v_hi or v - rn < edge_v_lo:
            out.append(('ERR', '{} nut hits the panel lip / walls'.format(nm)))
        for (pu, pv) in lay.pillars:
            if math.hypot(u - pu, v - pv) < rn + p.pad_d / 2.0 + 0.5:
                out.append(('ERR', '{} nut collides with a screw pad'.format(nm)))
        c = button_floor_clearance(p, fr, u, v)
        if c < 5.0:
            out.append(('ERR', '{} body only {:.1f} mm above the floor (< 5 mm)'.format(nm, c)))
    for i in range(len(btns)):
        for j in range(i + 1, len(btns)):
            d = math.hypot(btns[i][0] - btns[j][0], btns[i][1] - btns[j][1])
            if d < 2 * rn + 1.0:
                out.append(('ERR', '{} and {} nuts collide'.format(lay.buttons[i][0], lay.buttons[j][0])))
    if pocket[3] > edge_v_hi or pocket[2] > edge_u:
        out.append(('ERR', 'display pocket runs into the panel lip'))
    if abs(p.ring_y - dy) < (p.disp_h + p.fit) / 2.0 + ring_r:
        out.append(('ERR', 'display pocket overlaps the LED ring holder'))
    return out


def suggest_layout_fix(p, fr, lay):
    """Smallest change of disp_y (up) or btn_ang that clears all ERR checks."""
    def clean(**kw):
        return not [c for c in layout_checks(p, fr, lay, **kw) if c[0] == 'ERR']

    tips = []
    for k in range(1, 61):
        dy = p.disp_y + 0.5 * k
        if clean(disp_y=dy):
            tips.append('disp_y = {:.1f} mm (display moved toward the back)'.format(dy))
            break
    found = None
    for k in range(1, 81):
        for sgn in (-1, 1):
            a = p.btn_ang + sgn * 0.5 * k
            if 5.0 < a < 85.0 and clean(btn_ang=a):
                found = a
                break
        if found is not None:
            tips.append('btn_ang = {:.1f} deg (buttons rotated on their circle)'.format(found))
            break
    return tips


# =============================================================================
# 4. FUSION HELPERS
# =============================================================================
def P3(x, y, z):
    return adsk.core.Point3D.create(x * MM, y * MM, z * MM)


def P3t(t):
    return P3(t[0], t[1], t[2])


def V3(t):
    return adsk.core.Vector3D.create(t[0], t[1], t[2])


def VI(x):
    """ValueInput: strings are Fusion expressions, numbers are mm."""
    if isinstance(x, str):
        return adsk.core.ValueInput.createByString(x)
    return adsk.core.ValueInput.createByString('{:.5f} mm'.format(x))


def VA(deg):
    return adsk.core.ValueInput.createByString('{:.6f} deg'.format(deg))


def OC(items):
    oc = adsk.core.ObjectCollection.create()
    for it in items:
        oc.add(it)
    return oc


def tup(v):
    return (v.x, v.y, v.z)


def unit(t):
    ln = math.sqrt(dot(t, t))
    return (t[0] / ln, t[1] / ln, t[2] / ln) if ln > EPS else t


def feature_ops():
    fo = adsk.fusion.FeatureOperations
    return (fo.NewBodyFeatureOperation, fo.JoinFeatureOperation,
            fo.CutFeatureOperation, fo.IntersectFeatureOperation)


def plane_dist_mm(cplane, pt):
    """Distance (mm) from model point pt (mm) to a construction plane."""
    g = cplane.geometry
    o = g.origin
    n = unit(tup(g.normal))
    d = (pt[0] * MM - o.x) * n[0] + (pt[1] * MM - o.y) * n[1] + (pt[2] * MM - o.z) * n[2]
    return abs(d) / MM


class Planes(object):
    """Self-checking construction planes inside one component.

    Every plane is verified against a point that must lie on it; if Fusion's
    sign convention for an offset/angle turns out the other way round, the
    plane is deleted and rebuilt with the opposite sign.
    """

    def __init__(self, comp, fr, tag):
        self.c = comp
        self.fr = fr
        self.tag = tag
        self.cache = {}
        self._tilt = None
        self._sec = None

    def _angle_plane(self, ref, want, name):
        cps = self.c.constructionPlanes
        for ang in (self.fr.th, -self.fr.th):
            inp = cps.createInput()
            inp.setByAngle(self.c.xConstructionAxis,
                           adsk.core.ValueInput.createByReal(ang), ref)
            pl = cps.add(inp)
            nrm = unit(tup(pl.geometry.normal))
            if abs(abs(dot(nrm, want)) - 1.0) < 1e-5:
                pl.name = name
                return pl
            pl.deleteMe()
        raise RuntimeError('could not create the tilted plane ' + name)

    def _offset(self, base, dist, check, name):
        if abs(dist) < 1e-7 and plane_dist_mm(base, check) < 1e-3:
            return base
        cps = self.c.constructionPlanes
        for sgn in (1.0, -1.0):
            inp = cps.createInput()
            inp.setByOffset(base, VI(sgn * dist))
            pl = cps.add(inp)
            if plane_dist_mm(pl, check) < 1e-3:
                pl.name = name
                return pl
            pl.deleteMe()
        raise RuntimeError('offset plane failed: ' + name)

    def frame(self, w):
        """Plane parallel to the panel, w mm along the panel normal."""
        key = ('f', round(w, 4))
        if key not in self.cache:
            if self._tilt is None:
                self._tilt = self._angle_plane(self.c.xYConstructionPlane,
                                               self.fr.n, self.tag + ' tilt')
            self.cache[key] = self._offset(
                self._tilt, self.fr.d_top + w, self.fr.P(0.0, 0.0, w),
                '{} panel w={:+.2f}'.format(self.tag, w))
        return self.cache[key]

    def section(self, v):
        """Plane perpendicular to the panel, normal = up-slope, at panel v."""
        key = ('s', round(v, 4))
        if key not in self.cache:
            if self._sec is None:
                self._sec = self._angle_plane(self.c.xZConstructionPlane,
                                              self.fr.vd, self.tag + ' tilt section')
            self.cache[key] = self._offset(
                self._sec, self.fr.zc * self.fr.sn + v, self.fr.P(0.0, v, 0.0),
                '{} section v={:+.2f}'.format(self.tag, v))
        return self.cache[key]

    def xy(self, z):
        key = ('xy', round(z, 4))
        if key not in self.cache:
            self.cache[key] = self._offset(self.c.xYConstructionPlane, z, (0.0, 0.0, z),
                                           '{} Z={:.2f}'.format(self.tag, z))
        return self.cache[key]

    def xz(self, y):
        key = ('xz', round(y, 4))
        if key not in self.cache:
            self.cache[key] = self._offset(self.c.xZConstructionPlane, y, (0.0, y, 0.0),
                                           '{} Y={:.2f}'.format(self.tag, y))
        return self.cache[key]

    def yz(self, x):
        key = ('yz', round(x, 4))
        if key not in self.cache:
            self.cache[key] = self._offset(self.c.yZConstructionPlane, x, (x, 0.0, 0.0),
                                           '{} X={:.2f}'.format(self.tag, x))
        return self.cache[key]


class Sk(object):
    """Sketch wrapper: draws in a local 2D frame given by mapper(a, b)->model mm.

    All points go through modelToSketchSpace, so the internal orientation of
    the sketch plane never matters.
    """

    def __init__(self, comp, plane, mapper, name):
        self.comp = comp
        self.sk = comp.sketches.add(plane)
        self.m = mapper
        try:
            self.sk.name = name
        except Exception:
            pass

    def p(self, a, b):
        x, y, z = self.m(a, b)
        q = self.sk.modelToSketchSpace(P3(x, y, z))
        return adsk.core.Point3D.create(q.x, q.y, 0.0)

    def normal(self):
        n = self.sk.xDirection.crossProduct(self.sk.yDirection)
        return unit(tup(n))

    def circle(self, a, b, d):
        return self.sk.sketchCurves.sketchCircles.addByCenterRadius(self.p(a, b), d / 2.0 * MM)

    def poly(self, pts):
        lines = self.sk.sketchCurves.sketchLines
        first = None
        prev = None
        n = len(pts)
        for i in range(n):
            a1, b1 = pts[(i + 1) % n]
            start = prev.endSketchPoint if prev is not None else self.p(pts[0][0], pts[0][1])
            if i == n - 1 and first is not None:
                end = first.startSketchPoint
            else:
                end = self.p(a1, b1)
            ln = lines.addByTwoPoints(start, end)
            if first is None:
                first = ln
            prev = ln
        return first

    def rect(self, a0, b0, a1, b1):
        return self.poly([(a0, b0), (a1, b0), (a1, b1), (a0, b1)])

    def _end_near(self, arc, ab):
        t = self.p(ab[0], ab[1])
        sp = arc.startSketchPoint
        ep = arc.endSketchPoint
        return sp if sp.geometry.distanceTo(t) <= ep.geometry.distanceTo(t) else ep

    def rrect(self, ca, cb, w, h, r):
        r = min(r, w / 2.0 - 0.01, h / 2.0 - 0.01)
        a0, a1, b0, b1 = ca - w / 2.0, ca + w / 2.0, cb - h / 2.0, cb + h / 2.0
        if r < 0.05:
            return self.rect(a0, b0, a1, b1)
        k = r * (1.0 - math.sqrt(0.5))
        corners = [((a1 - r, b0), (a1 - k, b0 + k), (a1, b0 + r)),   # bottom right
                   ((a1, b1 - r), (a1 - k, b1 - k), (a1 - r, b1)),   # top right
                   ((a0 + r, b1), (a0 + k, b1 - k), (a0, b1 - r)),   # top left
                   ((a0, b0 + r), (a0 + k, b0 + k), (a0 + r, b0))]   # bottom left
        arcs = self.sk.sketchCurves.sketchArcs
        lines = self.sk.sketchCurves.sketchLines
        objs = []
        for (s, m, e) in corners:
            arc = arcs.addByThreePoints(self.p(s[0], s[1]), self.p(m[0], m[1]), self.p(e[0], e[1]))
            objs.append((arc, s, e))
        for i in range(4):
            arc, _s, e = objs[i]
            arc2, s2, _e2 = objs[(i + 1) % 4]
            lines.addByTwoPoints(self._end_near(arc, e), self._end_near(arc2, s2))

    def slot(self, ca, cb, w, ln):
        """Stadium slot, long axis along b."""
        r = w / 2.0
        hl = max(ln / 2.0 - r, 0.01)
        top = ((ca + r, cb + hl), (ca, cb + hl + r), (ca - r, cb + hl))
        bot = ((ca - r, cb - hl), (ca, cb - hl - r), (ca + r, cb - hl))
        arcs = self.sk.sketchCurves.sketchArcs
        lines = self.sk.sketchCurves.sketchLines
        A = arcs.addByThreePoints(self.p(*top[0]), self.p(*top[1]), self.p(*top[2]))
        B = arcs.addByThreePoints(self.p(*bot[0]), self.p(*bot[1]), self.p(*bot[2]))
        lines.addByTwoPoints(self._end_near(A, top[2]), self._end_near(B, bot[0]))
        lines.addByTwoPoints(self._end_near(B, bot[2]), self._end_near(A, top[0]))

    def profiles(self, loops=None):
        items = []
        prs = self.sk.profiles
        for i in range(prs.count):
            pr = prs.item(i)
            if loops is None or pr.profileLoops.count == loops:
                items.append(pr)
        if not items:
            raise RuntimeError('no closed profile in sketch "{}"'.format(self.sk.name))
        return OC(items)


def _dir_for(sk, dirv):
    ed = adsk.fusion.ExtentDirections
    if dirv is None or dot(sk.normal(), dirv) >= 0:
        return ed.PositiveExtentDirection
    return ed.NegativeExtentDirection


def _name(feat, name):
    if name:
        try:
            feat.name = name
        except Exception:
            pass
    return feat


def extrude(comp, sk, prof, op, dist, dirv=None, sym=False, bodies=None, name=None):
    """dist: mm number or Fusion expression. dirv: desired model direction."""
    if not isinstance(dist, str) and dist <= 0:
        raise RuntimeError('non-positive extrude distance for ' + str(name))
    ef = comp.features.extrudeFeatures
    inp = ef.createInput(prof, op)
    if sym:
        inp.setSymmetricExtent(VI(dist), True)
    else:
        inp.setOneSideExtent(adsk.fusion.DistanceExtentDefinition.create(VI(dist)),
                             _dir_for(sk, dirv))
    if bodies:
        inp.participantBodies = list(bodies)
    return _name(ef.add(inp), name)


def extrude_to(comp, sk, prof, op, entity, dirv, bodies=None, name=None, start=None):
    """Extrude up to a plane/face, optionally starting from another one."""
    ef = comp.features.extrudeFeatures
    inp = ef.createInput(prof, op)
    if start is not None:
        inp.startExtent = adsk.fusion.FromEntityStartDefinition.create(start, VI(0.0))
    inp.setOneSideExtent(adsk.fusion.ToEntityExtentDefinition.create(entity, False, VI(0.0)),
                         _dir_for(sk, dirv))
    if bodies:
        inp.participantBodies = list(bodies)
    return _name(ef.add(inp), name)


def fillet(comp, edges, radius, tangent=True, name=None):
    ff = comp.features.filletFeatures
    inp = ff.createInput()
    oc = OC(edges)
    try:
        inp.edgeSetInputs.addConstantRadiusEdgeSet(oc, VI(radius), tangent)
    except Exception:
        inp.addConstantRadiusEdgeSet(oc, VI(radius), tangent)
    try:
        inp.isRollingBallCorner = True
    except Exception:
        pass
    return _name(ff.add(inp), name)


def chamfer(comp, edges, dist, tangent=True, name=None):
    cf = comp.features.chamferFeatures
    oc = OC(edges)
    try:
        inp = cf.createInput2()
        inp.chamferEdgeSets.addEqualDistanceChamferEdgeSet(oc, VI(dist), tangent)
    except Exception:
        inp = cf.createInput(oc, tangent)
        inp.setToEqualDistance(VI(dist))
    return _name(cf.add(inp), name)


def chamfer_da(comp, edges, dist, ang_deg, flipped, tangent=True, name=None):
    cf = comp.features.chamferFeatures
    inp = cf.createInput2()
    inp.chamferEdgeSets.addDistanceAndAngleChamferEdgeSet(OC(edges), VI(dist), VA(ang_deg),
                                                          flipped, tangent)
    return _name(cf.add(inp), name)


def get_body(comp, name):
    b = comp.bRepBodies.itemByName(name)
    if b is None:
        raise RuntimeError('body "{}" not found'.format(name))
    return b


def is_planar(face):
    return face.geometry.surfaceType == adsk.core.SurfaceTypes.PlaneSurfaceType


def face_normal(face):
    ok, n = face.evaluator.getNormalAtPoint(face.pointOnFace)
    return unit(tup(n))


def find_planar_face(body, normal, pt, tol=0.02):
    """Largest planar face with outward normal ~normal lying through pt (mm)."""
    best = None
    best_area = -1.0
    for f in body.faces:
        if not is_planar(f):
            continue
        n = face_normal(f)
        if dot(n, normal) < 0.9999:
            continue
        o = adsk.core.Plane.cast(f.geometry).origin
        d = abs((pt[0] * MM - o.x) * n[0] + (pt[1] * MM - o.y) * n[1] +
                (pt[2] * MM - o.z) * n[2]) / MM
        if d > tol:
            continue
        if f.area > best_area:
            best, best_area = f, f.area
    if best is None:
        raise RuntimeError('planar face not found')
    return best


def outer_loop_edges(face):
    for lp in face.loops:
        if lp.isOuter:
            return [e for e in lp.edges]
    return []


def loop_bbox(lp):
    """(xmin, ymin, zmin, xmax, ymax, zmax) in mm of a B-Rep loop."""
    lo = [1e9, 1e9, 1e9]
    hi = [-1e9, -1e9, -1e9]
    for e in lp.edges:
        bb = e.boundingBox
        for i, (a, b) in enumerate(((bb.minPoint.x, bb.maxPoint.x), (bb.minPoint.y, bb.maxPoint.y),
                                    (bb.minPoint.z, bb.maxPoint.z))):
            lo[i] = min(lo[i], a / MM)
            hi[i] = max(hi[i], b / MM)
    return (lo[0], lo[1], lo[2], hi[0], hi[1], hi[2])


def inner_loops_near(face, test):
    """Inner loops of a face whose bbox satisfies test(bbox)."""
    out = []
    for lp in face.loops:
        if lp.isOuter:
            continue
        if test(loop_bbox(lp)):
            out.append(lp)
    return out


def body_bbox(body):
    bb = body.boundingBox
    return (bb.minPoint.x / MM, bb.minPoint.y / MM, bb.minPoint.z / MM,
            bb.maxPoint.x / MM, bb.maxPoint.y / MM, bb.maxPoint.z / MM)


def new_comp(ctx, name):
    occ = ctx.root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
    comp = occ.component
    comp.name = name
    return occ, comp


# ---- appearances ---------------------------------------------------------------
def _find_lib_appearance(app, prefs, keywords):
    libs = []
    try:
        for i in range(app.materialLibraries.count):
            lib = app.materialLibraries.item(i)
            try:
                if lib.appearances.count > 0:
                    libs.append(lib)
            except Exception:
                pass
    except Exception:
        return None
    for lib in libs:
        for nm in prefs:
            try:
                a = lib.appearances.itemByName(nm)
                if a:
                    return a
            except Exception:
                pass
    for lib in libs:
        apps = lib.appearances
        for j in range(apps.count):
            a = apps.item(j)
            low = a.name.lower()
            if all(k in low for k in keywords):
                return a
    return None


def _set_color(appearance, rgb):
    col = adsk.core.Color.create(rgb[0], rgb[1], rgb[2], 255)
    props = appearance.appearanceProperties
    for pid in ('opaque_albedo', 'transparent_color', 'metal_f0', 'surface_albedo'):
        try:
            cp = adsk.core.ColorProperty.cast(props.itemById(pid))
            if cp:
                cp.value = col
                return True
        except Exception:
            pass
    for i in range(props.count):
        try:
            cp = adsk.core.ColorProperty.cast(props.item(i))
            if cp:
                cp.value = col
                return True
        except Exception:
            pass
    return False


def make_appearance(ctx, name, rgb, translucent=False):
    des = ctx.design
    try:
        a = des.appearances.itemByName(name)
        if a:
            return a
    except Exception:
        pass
    src = None
    if translucent:
        src = _find_lib_appearance(ctx.app, ['Plastic - Translucent Matte (White)',
                                             'Plastic - Translucent Glossy (White)',
                                             'Plastic - Translucent Matte (Gray)',
                                             'Glass - Frosted'], ['translucent'])
    if src is None:
        src = _find_lib_appearance(ctx.app, ['Plastic - Matte (Yellow)', 'Plastic - Glossy (Yellow)',
                                             'Plastic - Matte (Gray)', 'Paint - Enamel Glossy (Yellow)'],
                                   ['plastic'])
    if src is None:
        ctx.rep.warn('no library appearance found for ' + name)
        return None
    a = des.appearances.addByCopy(src, name)
    if not _set_color(a, rgb):
        ctx.rep.warn('could not set the colour of appearance ' + name)
    return a


def paint(ctx, body, name, rgb, translucent=False, opacity=None):
    try:
        a = ctx.appearances.get(name)
        if a is None:
            a = make_appearance(ctx, name, rgb, translucent)
            ctx.appearances[name] = a
        if a is not None:
            body.appearance = a
    except Exception as e:
        ctx.rep.warn('appearance {} on {}: {}'.format(name, body.name, short_err(e)))
    if opacity is not None:
        try:
            body.opacity = opacity
        except Exception:
            pass


# ---- temporary B-Rep (reference parts, interference) ----------------------------
def tbm():
    return adsk.fusion.TemporaryBRepManager.get()


def t_box(center, ldir, wdir, L, W, H):
    obb = adsk.core.OrientedBoundingBox3D.create(P3t(center), V3(unit(ldir)), V3(unit(wdir)),
                                                 L * MM, W * MM, H * MM)
    return tbm().createBox(obb)


def t_cyl(a, b, d):
    return tbm().createCylinderOrCone(P3t(a), d / 2.0 * MM, P3t(b), d / 2.0 * MM)


def t_bool(target, tools, kind):
    bt = adsk.fusion.BooleanTypes
    k = {'union': bt.UnionBooleanType, 'sub': bt.DifferenceBooleanType,
         'int': bt.IntersectionBooleanType}[kind]
    for t in tools:
        tbm().booleanOperation(target, t, k)
    return target


def add_temp_bodies(comp, items, feat_name):
    """items: [(body_name, temporary BRepBody)] -> real bodies in a base feature."""
    bf = comp.features.baseFeatures.add()
    bf.startEdit()
    try:
        for nm, tb in items:
            b = comp.bRepBodies.add(tb, bf)
            b.name = nm
    finally:
        bf.finishEdit()
    _name(bf, feat_name)
    return [comp.bRepBodies.itemByName(nm) for nm, _tb in items]


# =============================================================================
# 5. SHARED SHAPE: wedge + top perimeter round
# =============================================================================
ROUND_STYLES = ('bed-safe round', 'fillet', 'chamfer')


def cut_halfspace(ctx, comp, planes, body, w, side, name):
    """Cut everything above (side=+1) or below (side=-1) panel plane w."""
    p, fr = ctx.p, ctx.fr
    s = Sk(comp, planes.frame(w), lambda a, b: fr.P(a, b, w), name)
    U = p.box_w / 2.0 + 40.0
    V = fr.half_len + 60.0
    s.rect(-U, -V, U, V)
    dirv = fr.n if side > 0 else neg(fr.n)
    return extrude(comp, s, s.profiles(), ctx.CUT, 2.0 * p.h_back + 60.0,
                   dirv=dirv, bodies=[body], name=name)


def build_wedge(ctx, comp, planes, name):
    """Footprint (rounded rectangle) extruded up to the sloped top plane."""
    p = ctx.p
    s = Sk(comp, comp.xYConstructionPlane, lambda a, b: (a, b, 0.0), name + ' footprint')
    s.rrect(0.0, 0.0, p.box_w, p.box_d, p.r_corner)
    top = planes.frame(0.0)
    try:
        f = extrude_to(comp, s, s.profiles(), ctx.NEW, top, (0.0, 0.0, 1.0),
                       name=name + ' wedge')
        body = f.bodies.item(0)
    except Exception as e:
        ctx.rep.warn('{} wedge: extrude-to-plane failed ({}); used extrude + cut'.format(
            name, short_err(e)))
        f = extrude(comp, s, s.profiles(), ctx.NEW, p.h_back + 5.0, dirv=(0.0, 0.0, 1.0),
                    name=name + ' wedge')
        body = f.bodies.item(0)
        cut_halfspace(ctx, comp, planes, body, 0.0, +1, name + ' slope cut')
    body.name = name
    return body, s


def _top_face(ctx, comp, bname):
    return find_planar_face(get_body(comp, bname), ctx.fr.n, ctx.fr.P(0.0, 0.0, 0.0))


def _chamfer_is_45_to_top(chf, n):
    for f in chf.faces:
        if not is_planar(f):
            continue
        c = max(-1.0, min(1.0, abs(dot(face_normal(f), n))))
        if abs(math.degrees(math.acos(c)) - 45.0) > 1.0:
            return False
    return True


def _lower_edges(chf, top_id):
    ids = set(f.tempId for f in chf.faces)
    out = {}
    for f in chf.faces:
        for e in f.edges:
            fids = [ff.tempId for ff in e.faces]
            if top_id in fids or all(i in ids for i in fids):
                continue
            out[e.tempId] = e
    return list(out.values())


def _round_bedsafe(ctx, comp, bname):
    """45 deg chamfer (measured from the top face) blended into a top_fillet
    radius: identical to a true fillet from 45 to 90 deg, but the part that
    would overhang more than 45 deg when the panel prints face-down is flat."""
    k = 2.0 - math.sqrt(2.0)
    d_expr = '{:.6f} * top_fillet'.format(k)
    chf = None
    for flipped in (False, True):
        edges = outer_loop_edges(_top_face(ctx, comp, bname))
        try:
            chf = chamfer_da(comp, edges, d_expr, 45.0, flipped, True, bname + ' top chamfer 45')
        except Exception:
            chf = None
            continue
        if _chamfer_is_45_to_top(chf, ctx.fr.n):
            break
        chf.deleteMe()
        chf = None
    if chf is None:
        raise RuntimeError('distance/angle chamfer failed')
    try:
        lower = _lower_edges(chf, _top_face(ctx, comp, bname).tempId)
        if not lower:
            raise RuntimeError('chamfer boundary edges not found')
        fillet(comp, lower, 'top_fillet', True, bname + ' top round')
    except Exception:
        try:
            chf.deleteMe()
        except Exception:
            pass
        raise


def top_round(ctx, comp, bname):
    order = list(ROUND_STYLES)
    if ctx.round_style:
        order.remove(ctx.round_style)
        order.insert(0, ctx.round_style)
    for st in order:
        try:
            if st == 'bed-safe round':
                _round_bedsafe(ctx, comp, bname)
            elif st == 'fillet':
                fillet(comp, outer_loop_edges(_top_face(ctx, comp, bname)), 'top_fillet',
                       True, bname + ' top fillet')
            else:
                chamfer(comp, outer_loop_edges(_top_face(ctx, comp, bname)), 'bed_chamfer',
                        True, bname + ' top chamfer')
        except Exception as e:
            ctx.rep.warn('{} top perimeter "{}" failed: {}'.format(bname, st, short_err(e)))
            continue
        if ctx.round_style is None:
            ctx.round_style = st
        elif st != ctx.round_style:
            ctx.rep.warn('{} top perimeter uses "{}" but the other part uses "{}"'.format(
                bname, st, ctx.round_style))
        if st != ROUND_STYLES[0]:
            ctx.rep.warn('{} top perimeter built as "{}" (bed-safe round failed)'.format(bname, st))
        return st
    raise RuntimeError('no top perimeter treatment possible')


# =============================================================================
# 6. BASE
#    Print orientation: the floor (Z=0 face) goes on the bed. No supports.
#    Bed edge: bed_chamfer. Pillars lean 15 deg (< 45). USB cutout = 12 mm
#    bridge, grille slots are vertical (2 mm bridges), tie-loop bridges 4.5 mm,
#    snap lips have 45/45 deg hooks.
# =============================================================================
def build_base(ctx):
    p, fr, lay = ctx.p, ctx.fr, ctx.lay
    occ, comp = new_comp(ctx, 'BASE')
    ctx.comps['BASE'] = comp
    pl = Planes(comp, fr, 'BASE')
    body, fsk = build_wedge(ctx, comp, pl, 'BASE')
    step(ctx, 'BASE top perimeter', top_round, ctx, comp, 'BASE')
    body = get_body(comp, 'BASE')
    cut_halfspace(ctx, comp, pl, body, lay.w_part, +1, 'BASE parting cut')
    zf = p.floor_t
    s = Sk(comp, pl.xy(zf), lambda a, b: (a, b, zf), 'BASE cavity')
    s.rrect(0.0, 0.0, 2.0 * lay.xi, 2.0 * lay.yi, lay.ri)
    extrude(comp, s, s.profiles(), ctx.CUT, p.h_back + 10.0, dirv=(0.0, 0.0, 1.0),
            bodies=[body], name='Cavity')
    step(ctx, 'BASE pillars + inserts', _base_pillars, ctx, comp, pl, fsk)
    for label, fn in (('BASE ESP32 cradle', _base_esp),
                      ('BASE buzzer cradle', _base_buzzer),
                      ('BASE cable-tie loops', _base_ties),
                      ('BASE USB cutout', _base_usb),
                      ('BASE buzzer grille', _base_grille),
                      ('BASE foot recesses', _base_feet),
                      ('BASE bed chamfer', _base_bed_chamfer)):
        step(ctx, label, fn, ctx, comp, pl)
    body = get_body(comp, 'BASE')
    paint(ctx, body, 'RA Base Dark Grey', C_BASE)
    ctx.bodies['BASE'] = body


def _base_pillars(ctx, comp, pl, fsk):
    p, fr, lay = ctx.p, ctx.fr, ctx.lay
    body = get_body(comp, 'BASE')
    w = lay.w_pillar
    mp = lambda a, b: fr.P(a, b, w)
    s = Sk(comp, pl.frame(w), mp, 'BASE pillars')
    for (u, v) in lay.pillars:
        s.circle(u, v, p.pillar_d)
    length = max(lay.pillar_top_z(i) for i in range(4)) / fr.cs + p.pillar_d
    extrude(comp, s, s.profiles(), ctx.JOIN, length, dirv=neg(fr.n), bodies=[body],
            name='Pillars (axis normal to panel)')
    # The tilted pillars run through the floor and (at the back) through the
    # back wall: keep only what lies inside the footprint and above Z=0.
    try:
        extrude(comp, fsk, fsk.profiles(), ctx.INT, p.h_back + 20.0, dirv=(0.0, 0.0, 1.0),
                bodies=[body], name='Pillar trim')
    except Exception as e:
        ctx.rep.warn('pillar trim (intersect) failed ({}); using two cuts'.format(short_err(e)))
        s2 = Sk(comp, comp.xYConstructionPlane, lambda a, b: (a, b, 0.0), 'BASE trim outside')
        s2.rect(-p.box_w, -p.box_d, p.box_w, p.box_d)
        s2.rrect(0.0, 0.0, p.box_w, p.box_d, p.r_corner)
        extrude(comp, s2, s2.profiles(loops=2), ctx.CUT, p.h_back + 20.0, dirv=(0.0, 0.0, 1.0),
                bodies=[body], name='Pillar trim outside')
        s3 = Sk(comp, comp.xYConstructionPlane, lambda a, b: (a, b, 0.0), 'BASE trim below')
        s3.rect(-p.box_w, -p.box_d, p.box_w, p.box_d)
        extrude(comp, s3, s3.profiles(), ctx.CUT, 60.0, dirv=(0.0, 0.0, -1.0),
                bodies=[body], name='Pillar trim below')
    s = Sk(comp, pl.frame(w), mp, 'BASE insert holes')
    for (u, v) in lay.pillars:
        s.circle(u, v, p.insert_d)
    extrude(comp, s, s.profiles(), ctx.CUT, 'insert_depth', dirv=neg(fr.n), bodies=[body],
            name='Heat-set insert holes')
    w2 = w - p.insert_depth
    s = Sk(comp, pl.frame(w2), lambda a, b: fr.P(a, b, w2), 'BASE insert relief')
    for (u, v) in lay.pillars:
        s.circle(u, v, min(p.m3_clear, p.insert_d - 0.4))
    extrude(comp, s, s.profiles(), ctx.CUT, 3.0, dirv=neg(fr.n), bodies=[body],
            name='Screw tip relief')


def _base_esp(ctx, comp, pl):
    """Cradle: 4 corner pads (4 mm under the board, clear of the pin rows),
    2 edge rails, each with 2 flexible snap lips (45/45 deg hook)."""
    p, lay = ctx.p, ctx.lay
    body = get_body(comp, 'BASE')
    zb = p.floor_t
    x0, cy = p.esp_x, lay.esp_cy
    hw, hl = p.esp_w / 2.0, p.esp_l / 2.0
    x_in = hw + p.fit / 2.0
    xy = lambda a, b: (a, b, zb)
    s = Sk(comp, pl.xy(zb), xy, 'ESP pads')
    for sx in (-1, 1):
        for sy in (-1, 1):
            xa, xb = x0 + sx * (hw - 3.0), x0 + sx * x_in
            ya, yb = cy + sy * (hl - 4.0), cy + sy * hl
            s.rect(min(xa, xb), min(ya, yb), max(xa, xb), max(ya, yb))
    extrude(comp, s, s.profiles(), ctx.JOIN, 'esp_clear', dirv=(0.0, 0.0, 1.0),
            bodies=[body], name='ESP pads')
    zr = lay.z_pcb_top + 0.5
    s = Sk(comp, pl.xz(cy), lambda a, b: (a, cy, b), 'ESP rails')
    for sx in (-1, 1):
        xa, xb = x0 + sx * x_in, x0 + sx * (x_in + p.rail_t)
        s.rect(min(xa, xb), zb, max(xa, xb), zr)
    extrude(comp, s, s.profiles(), ctx.JOIN, 'esp_l', sym=True, bodies=[body], name='ESP rails')
    clip_l = 6.0
    s = Sk(comp, pl.xy(zb), xy, 'ESP rail gaps')
    for sx in (-1, 1):
        xa, xb = x0 + sx * (x_in - 0.05), x0 + sx * (x_in + p.rail_t + 0.05)
        for yc in lay.esp_clip_y:
            s.rect(min(xa, xb), yc - clip_l / 2.0 - 0.8, max(xa, xb), yc + clip_l / 2.0 + 0.8)
    extrude(comp, s, s.profiles(), ctx.CUT, zr - zb + 1.0, dirv=(0.0, 0.0, 1.0),
            bodies=[body], name='ESP rail gaps')
    ct, h = 1.2, p.esp_snap
    zc = lay.z_pcb_top
    zt = zc + 2.0 * h + 0.6
    lip = [(x_in, zb), (x_in + ct, zb), (x_in + ct, zt), (x_in, zt),
           (x_in, zc + 2.0 * h), (x_in - h, zc + h), (x_in, zc)]
    for yc in lay.esp_clip_y:
        s = Sk(comp, pl.xz(yc), lambda a, b, yc=yc: (a, yc, b), 'ESP snap lips')
        for sx in (-1, 1):
            s.poly([(x0 + sx * a, b) for (a, b) in lip])
        extrude(comp, s, s.profiles(), ctx.JOIN, clip_l, sym=True, bodies=[body],
                name='ESP snap lips')
    s = Sk(comp, pl.xy(zb), xy, 'ESP end stop')
    yf = cy - hl - p.fit / 2.0
    s.rect(x0 - 4.0, yf - 2.0, x0 + 4.0, yf)
    extrude(comp, s, s.profiles(), ctx.JOIN, p.esp_clear + p.esp_t + 1.0,
            dirv=(0.0, 0.0, 1.0), bodies=[body], name='ESP end stop')


def _base_buzzer(ctx, comp, pl):
    """U-cradle against the back wall; buzzer axis along Y, snaps in from above."""
    p, lay = ctx.p, ctx.lay
    body = get_body(comp, 'BASE')
    zb = p.floor_t
    bx, bz, yi = p.buz_x, lay.buz_z, lay.yi
    half = p.buz_d / 2.0 + 2.4
    length = max(p.buz_len - 1.5, 4.0)
    s = Sk(comp, pl.xy(zb), lambda a, b: (a, b, zb), 'Buzzer cradle')
    s.rect(bx - half, yi - length, bx + half, yi + 0.5)
    extrude(comp, s, s.profiles(), ctx.JOIN, bz + 2.0 - zb, dirv=(0.0, 0.0, 1.0),
            bodies=[body], name='Buzzer cradle')
    s = Sk(comp, pl.xz(yi), lambda a, b: (a, yi, b), 'Buzzer bore')
    s.circle(bx, bz, p.buz_d)
    extrude(comp, s, s.profiles(), ctx.CUT, length + 1.0, dirv=(0.0, -1.0, 0.0),
            bodies=[body], name='Buzzer bore')


def _base_ties(ctx, comp, pl):
    """Inverted-U arches on the floor; a tie passes under the 4.5 mm bridge."""
    p, lay = ctx.p, ctx.lay
    body = get_body(comp, 'BASE')
    zb = p.floor_t
    a, b = p.tie_w / 2.0, p.tie_w / 2.0 + 2.0
    h1, h2 = p.tie_h, p.tie_h + 2.0
    arch = [(-b, 0.0), (-a, 0.0), (-a, h1), (a, h1), (a, 0.0), (b, 0.0), (b, h2), (-b, h2)]
    for (tx, ty) in lay.ties:
        s = Sk(comp, pl.yz(tx), lambda c, d, tx=tx: (tx, c, d), 'Cable-tie loop')
        s.poly([(ty + c, zb + d) for (c, d) in arch])
        extrude(comp, s, s.profiles(), ctx.JOIN, 4.0, sym=True, bodies=[body],
                name='Cable-tie loop')


def _base_usb(ctx, comp, pl):
    p, lay = ctx.p, ctx.lay
    body = get_body(comp, 'BASE')
    yi = lay.yi
    s = Sk(comp, pl.xz(yi), lambda a, b: (a, yi, b), 'USB cutout')
    s.rrect(p.esp_x, lay.usb_z, p.usb_w, p.usb_h, 1.5)
    extrude(comp, s, s.profiles(), ctx.CUT, p.wall_t + 1.0, dirv=(0.0, 1.0, 0.0),
            bodies=[body], name='USB cutout')
    body = get_body(comp, 'BASE')
    face = find_planar_face(body, (0.0, 1.0, 0.0), (0.0, p.box_d / 2.0, lay.usb_z))

    def near(bb):
        return (abs((bb[0] + bb[3]) / 2.0 - p.esp_x) < 1.0 and
                abs((bb[2] + bb[5]) / 2.0 - lay.usb_z) < 1.0)
    lps = inner_loops_near(face, near)
    if not lps:
        raise RuntimeError('USB cutout rim not found on the back face')
    chamfer(comp, [e for e in lps[0].edges], 'usb_chamfer', True, 'USB chamfered frame')


def _base_grille(ctx, comp, pl):
    p, lay = ctx.p, ctx.lay
    body = get_body(comp, 'BASE')
    yi = lay.yi
    n = max(int(round(p.slot_n)), 1)
    s = Sk(comp, pl.xz(yi), lambda a, b: (a, yi, b), 'Buzzer grille')
    for i in range(n):
        s.slot(p.buz_x + (i - (n - 1) / 2.0) * p.slot_pitch, lay.buz_z, p.slot_w, p.slot_l)
    extrude(comp, s, s.profiles(), ctx.CUT, p.wall_t + 1.0, dirv=(0.0, 1.0, 0.0),
            bodies=[body], name='Buzzer grille')


def _base_feet(ctx, comp, pl):
    p, lay = ctx.p, ctx.lay
    body = get_body(comp, 'BASE')
    s = Sk(comp, comp.xYConstructionPlane, lambda a, b: (a, b, 0.0), 'Foot recesses')
    for (x, y) in lay.feet:
        s.circle(x, y, p.foot_d)
    extrude(comp, s, s.profiles(), ctx.CUT, 'foot_depth', dirv=(0.0, 0.0, 1.0),
            bodies=[body], name='Rubber foot recesses')


def _base_bed_chamfer(ctx, comp, pl):
    body = get_body(comp, 'BASE')
    face = find_planar_face(body, (0.0, 0.0, -1.0), (0.0, 0.0, 0.0))
    chamfer(comp, outer_loop_edges(face), 'bed_chamfer', True, 'Bed chamfer')


# =============================================================================
# 7. TOP_PANEL
#    Print orientation: visible top face DOWN on the bed (smooth bed side).
#    No supports: the top perimeter starts with a 45 deg chamfer that blends
#    into the round, hole rims on the bed get bed_chamfer, and everything under
#    the panel (lip, pads, clips, ring tube, tabs) grows upward from the plate.
#    Clip hooks and ring stop tabs use 45 deg faces.
# =============================================================================
def build_top_panel(ctx):
    fr, lay = ctx.fr, ctx.lay
    occ, comp = new_comp(ctx, 'TOP_PANEL')
    ctx.comps['TOP_PANEL'] = comp
    pl = Planes(comp, fr, 'PANEL')
    build_wedge(ctx, comp, pl, 'TOP_PANEL')
    step(ctx, 'TOP_PANEL top perimeter', top_round, ctx, comp, 'TOP_PANEL')
    body = get_body(comp, 'TOP_PANEL')
    cut_halfspace(ctx, comp, pl, body, lay.w_part, -1, 'PANEL parting cut')
    for label, fn in (('TOP_PANEL alignment lip', _panel_lip),
                      ('TOP_PANEL screw pads', _panel_pads),
                      ('TOP_PANEL display window/pocket', _panel_display),
                      ('TOP_PANEL display clips', _panel_display_clips),
                      ('TOP_PANEL diffuser seat', _panel_seat),
                      ('TOP_PANEL LED ring holder', _panel_ring_holder),
                      ('TOP_PANEL button holes', _panel_buttons),
                      ('TOP_PANEL screw holes', _panel_screws),
                      ('TOP_PANEL hole rim chamfers', _panel_rim_chamfers),
                      ('TOP_PANEL title', _panel_title)):
        step(ctx, label, fn, ctx, comp, pl)
    body = get_body(comp, 'TOP_PANEL')
    paint(ctx, body, 'RA Panel Off-White', C_PANEL)
    ctx.bodies['TOP_PANEL'] = body


def _panel_lip(ctx, comp, pl):
    """Vertical-walled ring just inside the base walls (constant gap = fit)."""
    p, lay = ctx.p, ctx.lay
    body = get_body(comp, 'TOP_PANEL')
    a = p.wall_t + p.fit
    b = a + p.lip_w
    s = Sk(comp, comp.xYConstructionPlane, lambda c, d: (c, d, 0.0), 'PANEL lip')
    s.rrect(0.0, 0.0, p.box_w - 2.0 * a, p.box_d - 2.0 * a, p.r_corner - a)
    s.rrect(0.0, 0.0, p.box_w - 2.0 * b, p.box_d - 2.0 * b, max(p.r_corner - b, 0.5))
    p_lip, p_part = pl.frame(lay.w_lip), pl.frame(lay.w_part)
    try:
        extrude_to(comp, s, s.profiles(loops=2), ctx.JOIN, p_part, (0.0, 0.0, 1.0),
                   bodies=[body], name='Alignment lip', start=p_lip)
        return
    except Exception as e:
        ctx.rep.warn('lip from/to extrude failed ({}); using cut + combine'.format(short_err(e)))
    f = extrude(comp, s, s.profiles(loops=2), ctx.NEW, ctx.p.h_back + 5.0,
                dirv=(0.0, 0.0, 1.0), name='Lip blank')
    blank = f.bodies.item(0)
    cut_halfspace(ctx, comp, pl, blank, lay.w_part, +1, 'Lip top cut')
    cut_halfspace(ctx, comp, pl, blank, lay.w_lip, -1, 'Lip bottom cut')
    cf = comp.features.combineFeatures
    ci = cf.createInput(get_body(comp, 'TOP_PANEL'), OC([blank]))
    ci.operation = ctx.JOIN
    cf.add(ci)


def _panel_pads(ctx, comp, pl):
    p, fr, lay = ctx.p, ctx.fr, ctx.lay
    body = get_body(comp, 'TOP_PANEL')
    w = lay.w_part
    s = Sk(comp, pl.frame(w), lambda a, b: fr.P(a, b, w), 'PANEL screw pads')
    for (u, v) in lay.pillars:
        s.circle(u, v, p.pad_d)
    extrude(comp, s, s.profiles(), ctx.JOIN, 'pad_t', dirv=neg(fr.n), bodies=[body],
            name='Screw boss pads')


def _panel_display(ctx, comp, pl):
    p, fr, lay = ctx.p, ctx.fr, ctx.lay
    body = get_body(comp, 'TOP_PANEL')
    w = lay.w_part
    s = Sk(comp, pl.frame(w), lambda a, b: fr.P(a, b, w), 'PANEL display pocket')
    s.rrect(0.0, p.disp_y, p.disp_w + p.fit, p.disp_h + p.fit, 0.5)
    extrude(comp, s, s.profiles(), ctx.CUT, 'panel_t - disp_frame', dirv=fr.n, bodies=[body],
            name='Display pocket')
    s = Sk(comp, pl.frame(0.0), lambda a, b: fr.P(a, b, 0.0), 'PANEL display window')
    s.rrect(0.0, p.disp_y, p.disp_win_w, p.disp_win_h, 1.0)
    extrude(comp, s, s.profiles(), ctx.CUT, p.panel_t + 0.5, dirv=neg(fr.n), bodies=[body],
            name='Display window')


def _clip_profile(x_in, t, hook, w_top, w_catch):
    """Cantilever with a 45/45 deg hook. (u, w) points for the +u side.
    w_catch: w where the catch face meets the clip's inner face."""
    w_tip = w_catch - 2.0 * hook
    return [(x_in, w_top), (x_in + t, w_top), (x_in + t, w_tip), (x_in, w_tip),
            (x_in - hook, w_catch - hook), (x_in, w_catch)]


def _panel_display_clips(ctx, comp, pl):
    """2 snap clips at the display ends; hooks catch the backpack PCB."""
    p, fr, lay = ctx.p, ctx.fr, ctx.lay
    body = get_body(comp, 'TOP_PANEL')
    v0 = p.disp_y
    x_in = (p.disp_w + p.fit) / 2.0
    w_catch = lay.w_disp_pcb - 0.05 + (x_in - p.disp_w / 2.0)
    prof = _clip_profile(x_in, p.clip_t, p.clip_hook, lay.w_part + 0.4, w_catch)
    s = Sk(comp, pl.section(v0), lambda a, b: fr.P(a, v0, b), 'PANEL display clips')
    for sx in (-1, 1):
        s.poly([(sx * a, b) for (a, b) in prof])
    extrude(comp, s, s.profiles(), ctx.JOIN, 'clip_w', sym=True, bodies=[body],
            name='Display snap clips')


def _panel_seat(ctx, comp, pl):
    """Stepped hole: press-fit seat for the diffuser (flush) + light hole."""
    p, fr, lay = ctx.p, ctx.fr, ctx.lay
    body = get_body(comp, 'TOP_PANEL')
    mp = lambda a, b: fr.P(a, b, 0.0)
    s = Sk(comp, pl.frame(0.0), mp, 'PANEL diffuser seat')
    s.circle(0.0, p.ring_y, p.diff_d + p.press)
    extrude(comp, s, s.profiles(), ctx.CUT, 'diff_t', dirv=neg(fr.n), bodies=[body],
            name='Diffuser seat (press fit)')
    s = Sk(comp, pl.frame(0.0), mp, 'PANEL light hole')
    s.circle(0.0, p.ring_y, 2.0 * lay.r_tube_in)
    extrude(comp, s, s.profiles(), ctx.CUT, p.panel_t + 0.5, dirv=neg(fr.n), bodies=[body],
            name='Light hole')


def _panel_ring_holder(ctx, comp, pl):
    """Tube that holds the ring PCB ring_gap behind the diffuser: 2 stop tabs
    (+-v) fix the depth, 2 snap clips (+-u) hold the PCB's back face."""
    p, fr, lay = ctx.p, ctx.fr, ctx.lay
    body = get_body(comp, 'TOP_PANEL')
    vr = p.ring_y
    w = lay.w_part
    r_in, r_out = lay.r_tube_in, lay.r_tube_out
    tube_len = w - lay.w_ring_bot
    if tube_len < 0.5:
        raise RuntimeError('LED ring stack does not reach below the panel')
    mp = lambda a, b: fr.P(a, b, w)
    s = Sk(comp, pl.frame(w), mp, 'PANEL ring tube')
    s.circle(0.0, vr, 2.0 * r_in)
    s.circle(0.0, vr, 2.0 * r_out)
    extrude(comp, s, s.profiles(loops=2), ctx.JOIN, tube_len, dirv=neg(fr.n), bodies=[body],
            name='LED ring tube')
    gap_v = p.rclip_w / 2.0 + 0.8
    s = Sk(comp, pl.frame(w), mp, 'PANEL ring clip gaps')
    for sx in (-1, 1):
        a0, a1 = sx * (r_in - 0.6), sx * (r_out + 0.6)
        s.rect(min(a0, a1), vr - gap_v, max(a0, a1), vr + gap_v)
    extrude(comp, s, s.profiles(), ctx.CUT, tube_len + 0.5, dirv=neg(fr.n), bodies=[body],
            name='Ring clip gaps')
    r_s = p.ring_od / 2.0 - 1.0
    r_o2 = r_in + 0.75
    wt = lay.w_ring_top
    d45 = min(r_in - r_s, 0.9 * (w - wt))
    tab = [(r_o2, w), (r_in, w), (r_in - d45, w - d45)]
    if r_in - d45 - r_s > 0.01:
        tab.append((r_s, w - d45))
    tab += [(r_s, wt), (r_o2, wt)]
    s = Sk(comp, pl.yz(0.0), lambda a, b: fr.P(0.0, a, b), 'PANEL ring stop tabs')
    for sv in (-1, 1):
        s.poly([(vr + sv * a, b) for (a, b) in tab])
    extrude(comp, s, s.profiles(), ctx.JOIN, 2.5, sym=True, bodies=[body], name='Ring stop tabs')
    w_catch = lay.w_ring_bot - 0.05 + (r_in - p.ring_od / 2.0)
    prof = _clip_profile(r_in, p.rclip_t, p.rclip_hook, w + 0.4, w_catch)
    s = Sk(comp, pl.section(vr), lambda a, b: fr.P(a, vr, b), 'PANEL ring clips')
    for sx in (-1, 1):
        s.poly([(sx * a, b) for (a, b) in prof])
    extrude(comp, s, s.profiles(), ctx.JOIN, 'rclip_w', sym=True, bodies=[body],
            name='Ring snap clips')


def _panel_buttons(ctx, comp, pl):
    """Plain through holes: the panel stays panel_t thick around every hole
    (no pads/ribs inside the nut keep-out, see validation) so the nut grips."""
    p, fr, lay = ctx.p, ctx.fr, ctx.lay
    body = get_body(comp, 'TOP_PANEL')
    s = Sk(comp, pl.frame(0.0), lambda a, b: fr.P(a, b, 0.0), 'PANEL button holes')
    for (_nm, u, v) in lay.buttons:
        s.circle(u, v, p.btn_hole)
    extrude(comp, s, s.profiles(), ctx.CUT, p.panel_t + 0.5, dirv=neg(fr.n), bodies=[body],
            name='Button holes')


def _panel_screws(ctx, comp, pl):
    p, fr, lay = ctx.p, ctx.fr, ctx.lay
    body = get_body(comp, 'TOP_PANEL')
    mp = lambda a, b: fr.P(a, b, 0.0)
    s = Sk(comp, pl.frame(0.0), mp, 'PANEL screw holes')
    for (u, v) in lay.pillars:
        s.circle(u, v, p.m3_clear)
    extrude(comp, s, s.profiles(), ctx.CUT, p.panel_t + p.pad_t + 0.5, dirv=neg(fr.n),
            bodies=[body], name='M3 clearance holes')
    s = Sk(comp, pl.frame(0.0), mp, 'PANEL counterbores')
    for (u, v) in lay.pillars:
        s.circle(u, v, p.cb_d)
    extrude(comp, s, s.profiles(), ctx.CUT, 'cb_depth', dirv=neg(fr.n), bodies=[body],
            name='Counterbores (M3 button head)')


def _panel_rim_chamfers(ctx, comp, pl):
    """bed_chamfer on hole rims of the visible face (= bed face)."""
    p, fr, lay = ctx.p, ctx.fr, ctx.lay
    groups = [('buttons', [(u, v) for (_nm, u, v) in lay.buttons]),
              ('counterbores', list(lay.pillars)),
              ('diffuser seat', [(0.0, p.ring_y)]),
              ('display window', [(0.0, p.disp_y)])]
    for label, centres in groups:
        def run_group(centres=centres, label=label):
            face = _top_face(ctx, comp, 'TOP_PANEL')
            edges = []
            for (u, v) in centres:
                c = fr.P(u, v, 0.0)

                def test(bb, c=c):
                    return (abs((bb[0] + bb[3]) / 2.0 - c[0]) < 0.6 and
                            abs((bb[1] + bb[4]) / 2.0 - c[1]) < 0.6 and
                            abs((bb[2] + bb[5]) / 2.0 - c[2]) < 0.6)
                for lp in inner_loops_near(face, test):
                    edges.extend([e for e in lp.edges])
            if not edges:
                raise RuntimeError('rims not found')
            chamfer(comp, edges, 'bed_chamfer', True, 'Rim chamfer ' + label)
        step(ctx, 'TOP_PANEL rim chamfer ' + label, run_group)


def _panel_title(ctx, comp, pl):
    """'REACTION' engraved title_depth into the visible face."""
    p, fr = ctx.p, ctx.fr
    body = get_body(comp, 'TOP_PANEL')
    face = _top_face(ctx, comp, 'TOP_PANEL')
    sk = comp.sketches.add(face)
    sk.name = 'PANEL title'
    xs, ys = unit(tup(sk.xDirection)), unit(tup(sk.yDirection))
    ns = unit(cross(xs, ys))
    h = p.title_h * MM
    q = sk.modelToSketchSpace(P3t(fr.P(0.0, p.title_y, 0.0)))
    cx, cy = q.x, q.y
    texts = sk.sketchTexts
    hflip = vflip = False
    cu = dot(xs, fr.ud)
    txt = None
    if abs(abs(cu) - 1.0) < 1e-3:
        if dot(ns, fr.n) > 0:
            hflip = vflip = cu < 0
        elif cu < 0:
            hflip = True
        else:
            vflip = True
        try:
            ti = texts.createInput2(TITLE_TEXT, h)
            hw = 0.85 * h * len(TITLE_TEXT) / 2.0 + h
            hh = 0.9 * h
            ti.setAsMultiLine(adsk.core.Point3D.create(cx - hw, cy - hh, 0.0),
                              adsk.core.Point3D.create(cx + hw, cy + hh, 0.0),
                              adsk.core.HorizontalAlignments.CenterHorizontalAlignment,
                              adsk.core.VerticalAlignments.MiddleVerticalAlignment, 0.0)
            _text_style(ti, hflip, vflip)
            txt = texts.add(ti)
        except Exception as e:
            ctx.rep.warn('multi-line text failed ({}), trying legacy text'.format(short_err(e)))
            txt = None
    if txt is None:
        ang = math.atan2(dot(fr.ud, ys), dot(fr.ud, xs))
        width = 0.72 * h * len(TITLE_TEXT)
        ox = -width / 2.0
        oy = -h / 2.0
        pos = adsk.core.Point3D.create(cx + ox * math.cos(ang) - oy * math.sin(ang),
                                       cy + ox * math.sin(ang) + oy * math.cos(ang), 0.0)
        ti = texts.createInput(TITLE_TEXT, h, pos)
        ti.angle = ang
        _text_style(ti, dot(ns, fr.n) < 0, False)
        txt = texts.add(ti)
    ed = adsk.fusion.ExtentDirections
    dr = ed.PositiveExtentDirection if dot(ns, neg(fr.n)) >= 0 else ed.NegativeExtentDirection
    ef = comp.features.extrudeFeatures
    inp = ef.createInput(txt, ctx.CUT)
    inp.setOneSideExtent(adsk.fusion.DistanceExtentDefinition.create(VI('title_depth')), dr)
    inp.participantBodies = [body]
    _name(ef.add(inp), 'Title engraving')


def _text_style(ti, hflip, vflip):
    try:
        ti.fontName = 'Arial'
    except Exception:
        pass
    try:
        ti.textStyle = adsk.fusion.TextStyles.TextStyleBold
    except Exception:
        pass
    if hflip:
        ti.isHorizontalFlip = True
    if vflip:
        ti.isVerticalFlip = True


# =============================================================================
# 8. DIFFUSER   Print orientation: visible (top) face down on the bed.
# =============================================================================
def build_diffuser(ctx):
    p, fr = ctx.p, ctx.fr
    occ, comp = new_comp(ctx, 'DIFFUSER')
    ctx.comps['DIFFUSER'] = comp
    pl = Planes(comp, fr, 'DIFFUSER')
    s = Sk(comp, pl.frame(0.0), lambda a, b: fr.P(a, b, 0.0), 'Diffuser disc')
    s.circle(0.0, p.ring_y, p.diff_d)
    f = extrude(comp, s, s.profiles(), ctx.NEW, 'diff_t', dirv=neg(fr.n), name='Diffuser disc')
    f.bodies.item(0).name = 'DIFFUSER'

    def bed_chamfer():
        face = find_planar_face(get_body(comp, 'DIFFUSER'), fr.n, fr.P(0.0, 0.0, 0.0))
        chamfer(comp, outer_loop_edges(face), 'bed_chamfer', True, 'Diffuser bed chamfer')
    step(ctx, 'DIFFUSER bed chamfer', bed_chamfer)
    body = get_body(comp, 'DIFFUSER')
    paint(ctx, body, 'RA Diffuser Translucent White', C_DIFF, translucent=True, opacity=0.55)
    ctx.bodies['DIFFUSER'] = body


# =============================================================================
# 9. REFERENCE PARTS (light grey placeholders, NOT for printing)
# =============================================================================
def build_reference(ctx):
    p, fr, lay = ctx.p, ctx.fr, ctx.lay
    occ, comp = new_comp(ctx, 'Reference parts')
    ctx.comps['Reference parts'] = comp
    X, Y = (1.0, 0.0, 0.0), (0.0, 1.0, 0.0)
    g = 0.02          # tiny gap so that resting faces do not count as interference
    items = []

    def add(name, fn):
        try:
            items.append((name, fn()))
        except Exception as e:
            ctx.rep.warn('{}: {}'.format(name, short_err(e)))

    def esp32():
        x0, cy, zt = p.esp_x, lay.esp_cy, lay.z_pcb_top
        b = t_box((x0, cy, lay.z_pcb_bot + p.esp_t / 2.0), X, Y,
                  p.esp_w - 2 * g, p.esp_l - 2 * g, p.esp_t - 2 * g)
        # components overlap the board by 0.1 mm so the union is one solid
        mod = t_box((x0, cy - p.esp_l / 2.0 + 12.75, zt + 1.55 - 0.1), X, Y, 18.0, 25.5, 3.1)
        usb = t_box((x0, cy + p.esp_l / 2.0 - 2.2, zt + p.usb_conn_h / 2.0 - 0.1), X, Y,
                    8.0, 5.6, p.usb_conn_h)
        return t_bool(b, [mod, usb], 'union')

    def usb_plug():
        y0 = lay.yi + 0.1
        return t_box((p.esp_x, y0 + 8.0, lay.usb_z), X, Y, 10.6, 16.0, 7.0)

    def buzzer():
        y1 = lay.yi - g
        return t_cyl((p.buz_x, y1 - p.buz_len, lay.buz_z), (p.buz_x, y1, lay.buz_z), 12.0)

    def display():
        c1 = fr.P(0.0, p.disp_y, lay.w_disp_face - g - p.disp_body_t / 2.0)
        c2 = fr.P(0.0, p.disp_y, lay.w_disp_back - p.disp_pcb_t / 2.0 + g)
        a = t_box(c1, X, fr.vd, p.disp_w, p.disp_h, p.disp_body_t - g)
        b = t_box(c2, X, fr.vd, p.disp_w, p.disp_h, p.disp_pcb_t)   # overlaps body by g
        return t_bool(a, [b], 'union')

    def led_ring():
        top, bot = lay.w_ring_top - g, lay.w_ring_bot + g
        a = t_cyl(fr.P(0.0, p.ring_y, top), fr.P(0.0, p.ring_y, bot), p.ring_od)
        hole = t_cyl(fr.P(0.0, p.ring_y, top + 1.0), fr.P(0.0, p.ring_y, bot - 1.0), p.ring_id)
        t_bool(a, [hole], 'sub')
        leds = []
        rl = (p.ring_od + p.ring_id) / 4.0
        for k in range(8):
            t = math.radians(22.5 + 45.0 * k)        # LEDs sit between the stop tabs
            ru = (math.cos(t), 0.0, 0.0)
            radial = (ru[0] + math.sin(t) * fr.vd[0], math.sin(t) * fr.vd[1], math.sin(t) * fr.vd[2])
            tang = (-math.sin(t) + math.cos(t) * fr.vd[0], math.cos(t) * fr.vd[1],
                    math.cos(t) * fr.vd[2])
            c = fr.P(rl * math.cos(t), p.ring_y + rl * math.sin(t), top + 0.78)
            leds.append(t_box(c, radial, tang, 5.0, 5.0, 1.6))
        return t_bool(a, leds, 'union')

    def button(u, v):
        bez = t_cyl(fr.P(u, v, g), fr.P(u, v, g + 2.0), p.btn_bezel_d)
        bod = t_cyl(fr.P(u, v, 1.0), fr.P(u, v, -(p.panel_t + p.btn_depth)), p.btn_body_d)
        nut = t_cyl(fr.P(u, v, lay.w_part - g), fr.P(u, v, lay.w_part - 3.0), p.btn_nut_d)
        return t_bool(bez, [bod, nut], 'union')

    add('REF_ESP32', esp32)
    add('REF_USB_PLUG', usb_plug)
    add('REF_BUZZER', buzzer)
    add('REF_DISPLAY', display)
    add('REF_LED_RING', led_ring)
    for i, (nm, u, v) in enumerate(lay.buttons):
        add('REF_BUTTON_{}'.format(i + 1), lambda u=u, v=v: button(u, v))
    if not items:
        raise RuntimeError('no reference bodies could be created')
    bodies = add_temp_bodies(comp, items, 'REF placeholders')
    for (nm, _tb), b in zip(items, bodies):
        if b is None:
            ctx.rep.warn('{} not added'.format(nm))
            continue
        paint(ctx, b, 'RA Reference Light Grey', C_REF)
        ctx.ref_bodies[nm] = b
    ctx.rep.info('Reference parts: ' + ', '.join(sorted(ctx.ref_bodies)))


# =============================================================================
# 10. FIT_TEST coupon
#     Print orientation: plate bottom (Z=0) on the bed. The diffuser seat and
#     the button hole open on the bed side exactly like TOP_PANEL (face-down);
#     the insert pillar stub and the USB wall print upright like the BASE.
#     Placed to the right of the enclosure so it does not overlap the assembly.
# =============================================================================
def build_fit_test(ctx):
    p, lay = ctx.p, ctx.lay
    occ, comp = new_comp(ctx, 'FIT_TEST')
    ctx.comps['FIT_TEST'] = comp
    pl = Planes(comp, ctx.fr, 'FIT')
    m, g = 3.0, 3.0
    seat_d = p.diff_d + p.press
    col = max(p.btn_hole, p.pillar_d)
    need_w = m + seat_d + g + col + g + p.wall_t
    need_d = max(seat_d + 2 * m, p.btn_hole + p.pillar_d + g + 2 * m, p.usb_w + 2 * p.usb_chamfer + 8.0)
    W, D = max(p.ft_w, need_w), max(p.ft_d, need_d)
    if W > p.ft_w + 1e-6 or D > p.ft_d + 1e-6:
        ctx.rep.info('FIT_TEST grown to {:.1f} x {:.1f} mm so the {:.1f} mm diffuser seat keeps '
                     '>= {:.0f} mm walls'.format(W, D, seat_d, m))
    X0 = p.box_w / 2.0 + 30.0 + W / 2.0
    xs = X0 - W / 2.0 + m + seat_d / 2.0
    xc = xs + seat_d / 2.0 + g + col / 2.0
    yb = -D / 2.0 + m + p.btn_hole / 2.0
    yp = D / 2.0 - m - p.pillar_d / 2.0
    t = p.ft_t
    xy0 = lambda a, b: (a, b, 0.0)
    s = Sk(comp, comp.xYConstructionPlane, xy0, 'FIT plate')
    s.rrect(X0, 0.0, W, D, 3.0)
    f = extrude(comp, s, s.profiles(), ctx.NEW, 'ft_t', dirv=(0.0, 0.0, 1.0), name='FIT plate')
    body = f.bodies.item(0)
    body.name = 'FIT_TEST'

    def seat_and_holes():
        s = Sk(comp, comp.xYConstructionPlane, xy0, 'FIT diffuser seat')
        s.circle(xs, 0.0, seat_d)
        extrude(comp, s, s.profiles(), ctx.CUT, 'diff_t', dirv=(0.0, 0.0, 1.0), bodies=[body],
                name='FIT diffuser seat')
        s = Sk(comp, comp.xYConstructionPlane, xy0, 'FIT through holes')
        s.circle(xs, 0.0, 2.0 * lay.r_tube_in)
        s.circle(xc, yb, p.btn_hole)
        extrude(comp, s, s.profiles(), ctx.CUT, t + 1.0, dirv=(0.0, 0.0, 1.0), bodies=[body],
                name='FIT light + button holes')

    def pillar_stub():
        hs = p.insert_depth + 3.0
        s = Sk(comp, pl.xy(t), lambda a, b: (a, b, t), 'FIT pillar stub')
        s.circle(xc, yp, p.pillar_d)
        extrude(comp, s, s.profiles(), ctx.JOIN, hs, dirv=(0.0, 0.0, 1.0), bodies=[body],
                name='FIT pillar stub')
        zt = t + hs
        s = Sk(comp, pl.xy(zt), lambda a, b: (a, b, zt), 'FIT insert hole')
        s.circle(xc, yp, p.insert_d)
        extrude(comp, s, s.profiles(), ctx.CUT, 'insert_depth', dirv=(0.0, 0.0, -1.0),
                bodies=[body], name='FIT insert hole')
        zr = zt - p.insert_depth
        s = Sk(comp, pl.xy(zr), lambda a, b: (a, b, zr), 'FIT insert relief')
        s.circle(xc, yp, min(p.m3_clear, p.insert_d - 0.4))
        extrude(comp, s, s.profiles(), ctx.CUT, 2.0, dirv=(0.0, 0.0, -1.0), bodies=[body],
                name='FIT insert relief')

    def usb_wall():
        zc = t + (lay.usb_z - p.floor_t)
        ztop = zc + p.usb_h / 2.0 + 3.0
        half = p.usb_w / 2.0 + p.usb_chamfer + 3.0
        xo = X0 + W / 2.0
        xi = xo - p.wall_t
        s = Sk(comp, pl.xy(t), lambda a, b: (a, b, t), 'FIT USB wall')
        s.rect(xi, -half, xo, half)
        extrude(comp, s, s.profiles(), ctx.JOIN, ztop - t, dirv=(0.0, 0.0, 1.0), bodies=[body],
                name='FIT USB wall')
        s = Sk(comp, pl.yz(xi), lambda a, b: (xi, a, b), 'FIT USB cutout')
        s.rrect(0.0, zc, p.usb_w, p.usb_h, 1.5)
        extrude(comp, s, s.profiles(), ctx.CUT, p.wall_t + 1.0, dirv=(1.0, 0.0, 0.0),
                bodies=[body], name='FIT USB cutout')
        face = find_planar_face(get_body(comp, 'FIT_TEST'), (1.0, 0.0, 0.0), (xo, 0.0, zc))
        lps = inner_loops_near(face, lambda bb: abs((bb[2] + bb[5]) / 2.0 - zc) < 1.0)
        if not lps:
            raise RuntimeError('USB rim not found')
        chamfer(comp, [e for e in lps[0].edges], 'usb_chamfer', True, 'FIT USB chamfered frame')

    def bed_chamfers():
        face = find_planar_face(get_body(comp, 'FIT_TEST'), (0.0, 0.0, -1.0), (X0, 0.0, 0.0))
        edges = outer_loop_edges(face)
        for lp in face.loops:
            if not lp.isOuter:
                edges.extend([e for e in lp.edges])
        chamfer(comp, edges, 'bed_chamfer', True, 'FIT bed chamfers')

    step(ctx, 'FIT_TEST seat + holes', seat_and_holes)
    step(ctx, 'FIT_TEST insert pillar stub', pillar_stub)
    step(ctx, 'FIT_TEST USB wall', usb_wall)
    step(ctx, 'FIT_TEST bed chamfers', bed_chamfers)
    body = get_body(comp, 'FIT_TEST')
    paint(ctx, body, 'RA Panel Off-White', C_PANEL)
    ctx.bodies['FIT_TEST'] = body
    ctx.rep.info('FIT_TEST: {:.1f} x {:.1f} x {:.1f} mm at X = {:.0f} mm'.format(W, D, t, X0))


# =============================================================================
# 11. VALIDATION
# =============================================================================
def _intersect_mm3(a, b):
    """Volume (mm^3) of a ^ b, or None if the boolean failed."""
    try:
        if not a.boundingBox.intersects(b.boundingBox):
            return 0.0
        ta = tbm().copy(a)
        tb = tbm().copy(b)
        tbm().booleanOperation(ta, tb, adsk.fusion.BooleanTypes.IntersectionBooleanType)
        return max(ta.volume, 0.0) * 1000.0
    except Exception:
        return None


def validate(ctx):
    p, fr, lay, rep = ctx.p, ctx.fr, ctx.lay, ctx.rep
    enc = [ctx.bodies[k] for k in ('BASE', 'TOP_PANEL', 'DIFFUSER') if k in ctx.bodies]
    if enc:
        bbs = [body_bbox(b) for b in enc]
        lo = [min(bb[i] for bb in bbs) for i in range(3)]
        hi = [max(bb[i + 3] for bb in bbs) for i in range(3)]
        rep.val('Overall size: {:.1f} x {:.1f} x {:.1f} mm (W x D x H), nominal {:.0f} x {:.0f}, '
                'heights {:.0f} back / {:.0f} front'.format(hi[0] - lo[0], hi[1] - lo[1],
                                                            hi[2] - lo[2], p.box_w, p.box_d,
                                                            p.h_back, p.h_front))
    slope = math.degrees(fr.th)
    meas = ''
    if 'TOP_PANEL' in ctx.bodies:
        try:
            f = find_planar_face(ctx.bodies['TOP_PANEL'], fr.n, fr.P(0.0, 0.0, 0.0))
            nz = max(-1.0, min(1.0, face_normal(f)[2]))
            meas = ', measured on TOP_PANEL face {:.2f} deg'.format(math.degrees(math.acos(nz)))
        except Exception:
            meas = ''
    rep.val('Panel slope: {:.2f} deg (atan((h_back - h_front) / box_d)){}'.format(slope, meas))
    rep.val('Top perimeter: {}'.format(ctx.round_style or 'not built'))
    worst = 1e9
    for (nm, u, v) in lay.buttons:
        c = button_floor_clearance(p, fr, u, v)
        worst = min(worst, c)
        rep.val('{:<12} body bottom {:5.1f} mm above the floor {}'.format(
            nm, c, 'OK (>= 5)' if c >= 5.0 else '<-- LESS THAN 5 mm'))
        if c < 5.0:
            rep.warn('{} has only {:.1f} mm clearance above the BASE floor (need 5 mm)'.format(nm, c))
    rep.val('Button-to-floor clearance (minimum): {:.1f} mm'.format(worst))
    issues = layout_checks(p, fr, lay)
    errs = [m for (sev, m) in issues if sev == 'ERR']
    for sev, msg in issues:
        rep.val(('LAYOUT CONFLICT: ' if sev == 'ERR' else 'LAYOUT NOTE: ') + msg)
    if errs:
        rep.warn('{} panel layout conflict(s) with the current parameters'.format(len(errs)))
        tips = suggest_layout_fix(p, fr, lay)
        if tips:
            rep.val('Suggested fix (any one): ' + ' | '.join(tips))
    elif not issues:
        rep.val('Panel layout: no conflicts (>= 2 mm webs, nut keep-outs clear)')
    printed = [(k, ctx.bodies[k]) for k in ('BASE', 'TOP_PANEL', 'DIFFUSER') if k in ctx.bodies]
    refs = sorted(ctx.ref_bodies.items())
    pairs = [(a, b, 'REF/printed') for a in refs for b in printed]
    pairs += [(refs[i], refs[j], 'REF/REF') for i in range(len(refs)) for j in range(i + 1, len(refs))]
    pairs += [(printed[i], printed[j], 'printed/printed') for i in range(len(printed))
              for j in range(i + 1, len(printed))]
    hits = 0
    for (na, ba), (nb, bb), kind in pairs:
        vol = _intersect_mm3(ba, bb)
        if vol is None:
            rep.warn('interference check failed for {} / {}'.format(na, nb))
        elif vol > 0.5:
            hits += 1
            rep.val('INTERFERENCE ({}) {} <-> {}: {:.1f} mm^3'.format(kind, na, nb, vol))
    if not refs:
        rep.val('Interference: no reference parts to check')
    elif hits == 0:
        rep.val('Interference: none ({} pairs checked; resting/contact faces ignored)'.format(len(pairs)))
    else:
        rep.warn('{} interfering pair(s) - see VALIDATION'.format(hits))


# =============================================================================
# 12. PARAMETERS IN FUSION, CONTEXT, RUN
# =============================================================================
def _read_existing_expressions(app):
    """If the active design is a previous Reaction Arcade build, reuse its
    parameter expressions (edit in Change Parameters -> run again)."""
    out = {}
    try:
        des = adsk.fusion.Design.cast(app.activeProduct)
        if des is None or des.userParameters.itemByName('box_w') is None:
            return out
        for name, _e, _u, _c in PARAMS:
            prm = des.userParameters.itemByName(name)
            if prm is not None:
                out[name] = prm.expression
    except Exception:
        pass
    return out


def create_params(design, rep, old):
    ups = design.userParameters
    vals = Vals()
    for name, expr, units, comment in PARAMS:
        prm = ups.itemByName(name)
        e = old.get(name, expr)
        if prm is None:
            try:
                prm = ups.add(name, adsk.core.ValueInput.createByString(e), units, comment)
            except Exception:
                rep.warn('parameter {}: expression "{}" rejected, default used'.format(name, e))
                prm = ups.add(name, adsk.core.ValueInput.createByString(expr), units, comment)
        if units == 'mm':
            val = prm.value / MM
        elif units == 'deg':
            val = math.degrees(prm.value)
        else:
            val = prm.value
        setattr(vals, name, val)
    expect = math.degrees(math.atan2(vals.h_back - vals.h_front, vals.box_d))
    if abs(vals.slope_ang - expect) > 0.01:
        try:
            ups.itemByName('slope_ang').expression = '{:.4f} deg'.format(expect)
        except Exception:
            pass
        vals.slope_ang = expect
    if old:
        rep.info('Parameters taken from the active Reaction Arcade design')
    return vals


class Ctx(object):
    def __init__(self, app, ui, design, p, rep):
        self.app = app
        self.ui = ui
        self.design = design
        self.root = design.rootComponent
        self.p = p
        self.rep = rep
        self.fr = Frame(p)
        self.lay = Layout(p, self.fr)
        self.NEW, self.JOIN, self.CUT, self.INT = feature_ops()
        self.comps = {}
        self.bodies = {}
        self.ref_bodies = {}
        self.appearances = {}
        self.round_style = None


def tidy_view(ctx):
    for comp in list(ctx.comps.values()) + [ctx.root]:
        for attr in ('isConstructionFolderLightBulbOn', 'isSketchFolderLightBulbOn'):
            try:
                setattr(comp, attr, False)
            except Exception:
                pass
    try:
        vp = ctx.app.activeViewport
        cam = vp.camera
        cam.isSmoothTransition = False
        cam.target = P3(30.0, 0.0, 30.0)
        cam.eye = P3(330.0, -460.0, 360.0)
        cam.upVector = adsk.core.Vector3D.create(0.0, 0.0, 1.0)
        vp.camera = cam
        vp.fit()
    except Exception:
        pass


BUILDERS = (('BASE', build_base),
            ('TOP_PANEL', build_top_panel),
            ('DIFFUSER', build_diffuser),
            ('Reference parts', build_reference),
            ('FIT_TEST', build_fit_test))


def run(context):
    app = adsk.core.Application.get()
    ui = app.userInterface
    rep = Report()
    try:
        old = _read_existing_expressions(app)
        app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
        design = adsk.fusion.Design.cast(app.activeProduct)
        design.designType = adsk.fusion.DesignTypes.ParametricDesignType
        try:
            design.fusionUnitsManager.distanceDisplayUnits = \
                adsk.fusion.DistanceUnits.MillimeterDistanceUnits
        except Exception:
            pass
        p = create_params(design, rep, old)
        ctx = Ctx(app, ui, design, p, rep)
        for name, fn in BUILDERS:
            n0 = len(rep.warnings)
            try:
                fn(ctx)
                k = len(rep.warnings) - n0
                rep.part(name, True, '({} step warning{})'.format(k, '' if k == 1 else 's') if k else '')
            except Exception as e:
                rep.part(name, False, short_err(e))
                rep.warn('{} failed:\n{}'.format(name, traceback.format_exc()))
            try:
                adsk.doEvents()
            except Exception:
                pass
        try:
            validate(ctx)
        except Exception:
            rep.warn('validation failed:\n' + traceback.format_exc())
        tidy_view(ctx)
    except Exception:
        rep.warn('fatal error:\n' + traceback.format_exc())
    log_out(app, rep.text(full=True))
    try:
        ui.messageBox(rep.text(full=False)[:3800], 'Reaction Arcade')
    except Exception:
        pass


def stop(context):
    pass
