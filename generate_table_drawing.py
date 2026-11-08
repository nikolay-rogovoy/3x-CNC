#!/usr/bin/env python3
"""Сборочный чертёж базы со столом и чертежи её деталей (DXF + PDF, 2 листа А1).

Геометрия сборки берётся из generate_drawing.base_parts(), поэтому сборка
совпадает с компоновочным чертежом. DXF в мм бумаги (виды в разных масштабах),
размеры показывают натуральные величины (DIMLFAC). Все надписи — латиницей.

Запуск: python3 generate_table_drawing.py
"""
import math
from copy import copy
from pathlib import Path

import ezdxf
from ezdxf.addons.drawing import Frontend, RenderContext
from ezdxf.addons.drawing.config import BackgroundPolicy, ColorPolicy, Configuration
from ezdxf.addons.drawing.matplotlib import MatplotlibBackend
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages

import generate_drawing as g
from generate_drawing import lat

OUT = Path(__file__).resolve().parent / "чертёж-стола"
W, H = 841.0, 594.0              # лист А1, мм
SHEET2_DX = 900.0                # второй лист правее первого
TH = 3.5
TS = 2.5                         # мелкий шрифт: таблицы, выноски
DIM_TXT = 3.0                    # текст размеров
K_ASM = 2.5                      # масштаб видов сборки 1:2,5

# Профиль 40x80 паз 8: оси сердечников D6,8 (под M8) в 20 мм от граней
CORE = (20, 60)


# ---------------------- примитивы ----------------------
def text(msp, s, x, y, h=TH, align="LEFT", layer="Tekst"):
    return g.text(msp, s, x, y, h, layer=layer, align=align)


def line(msp, p0, p1, layer="Rama"):
    msp.add_line(p0, p1, dxfattribs={"layer": layer})


def natural(length, k):
    """Натуральный размер текстом: виды уменьшены, а LibreCAD и др. игнорируют DIMLFAC
    и пересчитывают размер по геометрии, поэтому число пишется в размер явно."""
    s = f"{abs(length) * k:.2f}".rstrip("0").rstrip(".")
    return s.replace(".", ",")


def dimh(msp, x0, x1, y, base_y, k=1.0, label=None, y1=None):
    d = msp.add_linear_dim(base=(x0, base_y), p1=(x0, y), p2=(x1, y if y1 is None else y1), dimstyle="T1",
                           text=lat(label) if label else natural(x1 - x0, k), override={"dimlfac": k},
                           dxfattribs={"layer": "Razmery"})
    d.render()


def dimv(msp, y0, y1, x, base_x, k=1.0, label=None, x1=None):
    d = msp.add_linear_dim(base=(base_x, y0), p1=(x, y0), p2=(x if x1 is None else x1, y1), angle=90,
                           dimstyle="T1", text=lat(label) if label else natural(y1 - y0, k), override={"dimlfac": k},
                           dxfattribs={"layer": "Razmery"})
    d.render()


def leader(msp, x, y, dx, dy, s, h=TS):
    """Выноска с полкой: от точки (x, y) к полке, надпись над полкой."""
    ex, ey = x + dx, y + dy
    line(msp, (x, y), (ex, ey), "Tekst")
    msp.add_circle((x, y), 0.6, dxfattribs={"layer": "Tekst"})
    w = len(lat(s)) * h * 0.62 + 2
    sx = ex if dx >= 0 else ex - w
    line(msp, (sx, ey), (sx + w, ey), "Tekst")
    text(msp, s, sx + 1, ey + 0.8, h)


def balloon(msp, n, tx, ty, bx, by, r=5.0):
    d = math.hypot(tx - bx, ty - by)
    line(msp, (bx + (tx - bx) * r / d, by + (ty - by) * r / d), (tx, ty), "Tekst")
    msp.add_circle((tx, ty), 0.6, dxfattribs={"layer": "Tekst"})
    msp.add_circle((bx, by), r, dxfattribs={"layer": "Tekst"})
    text(msp, str(n), bx, by, TH * 0.85 if len(str(n)) < 3 else TS, align="MIDDLE_CENTER")


def hatch_poly(msp, pts, holes=(), angle=45, scale=0.5):
    h = msp.add_hatch(dxfattribs={"layer": "Shtrihovka"})
    h.set_pattern_fill("ANSI31", scale=scale, angle=angle - 45)
    h.paths.add_polyline_path(pts, is_closed=True, flags=ezdxf.const.BOUNDARY_PATH_EXTERNAL)
    for c, r in holes:
        n = 24
        h.paths.add_polyline_path([(c[0] + r * math.cos(2 * math.pi * i / n), c[1] + r * math.sin(2 * math.pi * i / n))
                                   for i in range(n)], is_closed=True, flags=ezdxf.const.BOUNDARY_PATH_OUTERMOST)


class Det:
    """Вид детали: u, v — мм детали, (ox, oy) — начало на листе, k — знаменатель масштаба."""

    def __init__(self, msp, ox, oy, k):
        self.msp, self.ox, self.oy, self.k = msp, ox, oy, k

    def X(self, u):
        return self.ox + u / self.k

    def Y(self, v):
        return self.oy + v / self.k

    def P(self, u, v):
        return self.X(u), self.Y(v)

    def line(self, u0, v0, u1, v1, layer="Rama"):
        line(self.msp, self.P(u0, v0), self.P(u1, v1), layer)

    def rect(self, u0, v0, u1, v1, layer="Rama"):
        for a, b in (((u0, v0), (u1, v0)), ((u1, v0), (u1, v1)), ((u1, v1), (u0, v1)), ((u0, v1), (u0, v0))):
            self.line(*a, *b, layer)

    def hole(self, u, v, d, layer="Rama", thread=False, mark=True):
        """Отверстие; thread — резьба: внутренний D сверла сплошной, наружный — тонкой 3/4 дуги."""
        if thread:
            self.msp.add_circle(self.P(u, v), d * 0.85 / 2 / self.k, dxfattribs={"layer": layer})
            self.msp.add_arc(self.P(u, v), d / 2 / self.k, 0, 270, dxfattribs={"layer": "Tonkie"})
        else:
            self.msp.add_circle(self.P(u, v), d / 2 / self.k, dxfattribs={"layer": layer})
        if mark:
            e = d / 2 + 2.5 * self.k
            self.line(u - e, v, u + e, v, "Osevye")
            self.line(u, v - e, u, v + e, "Osevye")

    def hidden_bore(self, u0, u1, v, d):
        """Скрытое отверстие вдоль u (вид сбоку)."""
        for vv in (v - d / 2, v + d / 2):
            self.line(u0, vv, u1, vv, "Nevidimye")
        self.line(u0, v, u1, v, "Osevye")

    def hdim(self, u0, u1, v, off, label=None, v1=None):
        dimh(self.msp, self.X(u0), self.X(u1), self.Y(v), self.Y(v) + off, self.k, label,
             None if v1 is None else self.Y(v1))

    def vdim(self, v0, v1, u, off, label=None, u1=None):
        dimv(self.msp, self.Y(v0), self.Y(v1), self.X(u), self.X(u) + off, self.k, label,
             None if u1 is None else self.X(u1))

    def leader(self, u, v, dx, dy, s):
        leader(self.msp, self.X(u), self.Y(v), dx, dy, s)

    def title(self, s, du=0, dv=8):
        text(self.msp, s, self.ox + du, self.oy + dv, TH)


def profile_outline(w, h):
    """Контур сечения профиля w x h (мм) с пазами 8: открытие 8, полость 16."""
    pts = []
    corners = [(0, 0), (w, 0), (w, h), (0, h)]
    for i in range(4):
        (x0, y0), (x1, y1) = corners[i], corners[(i + 1) % 4]
        ln = math.hypot(x1 - x0, y1 - y0)
        tx, ty = (x1 - x0) / ln, (y1 - y0) / ln
        nx, ny = -ty, tx                      # внутрь (обход против часовой)
        pts.append((x0, y0))
        for t in ([20] if ln == 40 else [20, 60]):
            for a, d in ((-4, 0), (-4, 2.5), (-8, 2.5), (-5, 9), (5, 9), (8, 2.5), (4, 2.5), (4, 0)):
                pts.append((x0 + tx * (t + a) + nx * d, y0 + ty * (t + a) + ny * d))
    return pts


def draw_profile_section(msp, x0, y0, w, h, k, hatch=True, layer="Rama"):
    """Сечение профиля, левый нижний угол (x0, y0) на листе."""
    pts = [(x0 + u / k, y0 + v / k) for u, v in profile_outline(w, h)]
    for a, b in zip(pts, pts[1:] + pts[:1]):
        line(msp, a, b, layer)
    cores = [((x0 + cu / k, y0 + cv / k), 3.4 / k) for cu in ([20] if w == 40 else CORE) for cv in ([20] if h == 40 else CORE)]
    for c, r in cores:
        msp.add_circle(c, r, dxfattribs={"layer": layer})
    if hatch:
        hatch_poly(msp, pts, cores)


# ---------------------- лист 1: сборка ----------------------
def asm_views(fx, fy, sx, sy, tx, ty, ax, ay, k=K_ASM):
    """Проекции: фронт (x,z), сверху (x,y), слева (-y,z), разрез A-A (x,z)."""
    return {
        "front": (lambda p: (p.x, p.z), lambda p: p.y[0], lambda u, v: (fx + u / k, fy + v / k), "y"),
        "top": (lambda p: (p.x, p.y), lambda p: -p.z[1], lambda u, v: (tx + u / k, ty + v / k), "z"),
        "side": (lambda p: (tuple(sorted((-p.y[0], -p.y[1]))), p.z), lambda p: p.x[0],
                 lambda u, v: (sx + u / k, sy + v / k), "x"),
        "section": (lambda p: (p.x, p.z), lambda p: p.y[0], lambda u, v: (ax + u / k, ay + v / k), "y"),
    }


SECTION_Y = 165       # плоскость разреза A-A: через стол, BF12 и полосы


def section_parts(parts, yc):
    """Отбросить материал перед плоскостью y = yc; вернуть (детали, рассечённые)."""
    out, cut = [], []
    for p in parts:
        if p.y[1] <= yc or p.layer == "Krepezh":
            continue
        q = copy(p)
        q.hidden = False
        if p.y[0] < yc:
            q.y = (yc, p.y[1])
            cut.append(q)
        out.append(q)
    return out, cut


def draw_assembly(msp):
    k = K_ASM
    FX, FY = 175.0, 450.0          # вид спереди: x=0, z=0
    TX, TY = 175.0, 105.0          # вид сверху: x=0, y=0
    SX, SY = 590.0, 450.0          # вид слева: y=0, z=0
    AX, AY = 470.0, 320.0          # разрез A-A: x=0, z=0
    views = asm_views(FX, FY, SX, SY, TX, TY, AX, AY)
    parts = g.base_parts(fasteners=True)
    for v in ("front", "top", "side"):
        g.draw_view(msp, parts, v, views)

    # --- разрез A-A ---
    sp, cut = section_parts(parts, SECTION_Y)
    for p in cut:
        if p.pos in (1, 3):
            p.skip = True           # профиль рисуется сечением ниже
    g.draw_view(msp, sp, "section", views)
    for p in cut:
        x0, z0 = AX + p.x[0] / k, AY + p.z[0] / k
        if p.pos in (1, 3):
            draw_profile_section(msp, x0, z0, 40, 80, k)
        elif p.axis == "y":
            c = (AX + sum(p.x) / 2 / k, AY + sum(p.z) / 2 / k)
            r = (p.x[1] - p.x[0]) / 2 / k
            hatch_poly(msp, [(c[0] + r * math.cos(a / 12 * math.pi), c[1] + r * math.sin(a / 12 * math.pi)) for a in range(24)])
        else:
            x1, z1 = AX + p.x[1] / k, AY + p.z[1] / k
            ang = 135 if p.pos in (4, 32) else 45
            hatch_poly(msp, [(x0, z0), (x1, z0), (x1, z1), (x0, z1)], angle=ang, scale=0.35 if p.pos == 4 else 0.5)
    text(msp, "A-A", AX, AY + 75, TH * 1.2, align="CENTER")

    # --- узел Б 1:1 (вид сверху без стола) ---
    BX0, BX1, BY0, BY1 = 95, 252, -18, 96
    NX, NY = 350.0, 120.0
    nodes = {"top": (views["top"][0], views["top"][1],
                     lambda u, v: (NX + (u - BX0), NY + (v - BY0)), "z")}
    g.draw_view(msp, [p for p in parts if p.pos != 4], "top", nodes, clip=(BX0, BX1, BY0, BY1))
    text(msp, "B (1:1), stol ne pokazan", NX + (BX1 - BX0) / 2, NY + (BY1 - BY0) + 8, TH * 1.2, align="CENTER")
    msp.add_circle((TX + 190 / k, TY + 30 / k), 70 / k, dxfattribs={"layer": "Tonkie"})
    text(msp, "B", TX + 190 / k + 26, TY + 30 / k + 20, TH * 1.2)
    N = lambda x, y: (NX + (x - BX0), NY + (y - BY0))
    dimh(msp, N(110, 0)[0], N(240, 0)[0], N(0, -PLATE)[1], N(0, -PLATE)[1] - 10, 1.0)
    dimh(msp, N(104, 40)[0], N(144, 40)[0], N(0, 40)[1], N(0, 96)[1] + 0, 1.0)
    dimh(msp, N(144, 40)[0], N(200, 40)[0], N(0, 40)[1], N(0, 96)[1] + 0, 1.0)
    dimv(msp, N(0, 0)[1], N(0, 40)[1], N(110, 0)[0], N(110, 0)[0] - 8, 1.0)
    dimv(msp, N(0, -PLATE)[1], N(0, 0)[1], N(240, 0)[0], N(240, 0)[0] + 18, 1.0)
    for n, x, y, bx, by in [(1, 225, 80, 262, 86), (2.1, 170, 20, 262, 40), (3, 124, 75, 100, 86),
                            (30, 130, -3, 108, -30), ("K1", 244, 20, 268, 4), ("K2", 124, 10, 85, 10),
                            ("K3", 220, -12, 262, -12), ("K4", 172, -10, 160, -30)]:
        balloon(msp, n, *N(x, y), *N(bx, by), r=5 if len(str(n)) < 3 else 5.5)

    # --- оси и обозначения ---
    line(msp, (FX, FY - 8), (FX, FY + 150 / k), "Osevye")
    line(msp, (TX, TY - 18), (TX, TY + 660 / k), "Osevye")
    line(msp, (AX, AY - 8), (AX, AY + 150 / k), "Osevye")
    for sgn in (-1, 1):          # оси винтов Y на виде сверху
        line(msp, (TX + sgn * g.Y_AXIS_X / k, TY + 120 / k), (TX + sgn * g.Y_AXIS_X / k, TY + 660 / k), "Osevye")
    # след разреза A-A на виде сверху
    ys = TY + SECTION_Y / k
    for sgn in (-1, 1):
        x0 = TX + sgn * 262 / k
        line(msp, (x0, ys), (x0 + sgn * 10, ys), "Rama")
        line(msp, (x0 + sgn * 5, ys), (x0 + sgn * 5, ys + 7), "Rama")
        msp.add_solid([(x0 + sgn * 5 - 1.2, ys + 7), (x0 + sgn * 5 + 1.2, ys + 7), (x0 + sgn * 5, ys + 10)],
                      dxfattribs={"layer": "Rama"})
        text(msp, "A", x0 + sgn * 9, ys + 4, TH)
    text(msp, "VID SPEREDI", FX, FY + 75, TH * 1.2, align="CENTER")
    text(msp, "VID SVERHU", TX, TY + 300, TH * 1.2, align="CENTER")
    text(msp, "VID SLEVA", SX - 300 / k, SY + 75, TH * 1.2, align="CENTER")

    # --- размеры: вид сверху ---
    T = lambda x, y: (TX + x / k, TY + y / k)
    dimh(msp, T(-240, 0)[0], T(240, 0)[0], T(0, -PLATE)[1], TY - 26, k)
    dimh(msp, T(-220, 0)[0], T(220, 0)[0], T(0, 130)[1], TY - 18, k, "440 (osi relsov)", T(0, 130)[1])
    dimh(msp, T(-150, 0)[0], T(150, 0)[0], T(0, 20)[1], TY - 10, k)
    dimh(msp, T(-g.Y_AXIS_X, 0)[0], T(g.Y_AXIS_X, 0)[0], T(0, 600)[1], TY + 690 / k, k, "338 (osi vintov Y)")
    dimh(msp, T(-g.BEAM_X1, 0)[0], T(g.BEAM_X1, 0)[0], T(0, 525)[1], TY + 662 / k, k)
    dimh(msp, T(-g.BEAM_X0, 0)[0], T(g.BEAM_X0, 0)[0], T(0, 525)[1], TY + 640 / k, k)
    dimv(msp, T(0, -PLATE)[1], T(0, g.MOTOR_Y0 + g.MOTOR_L)[1], T(240, 0)[0], T(240, 0)[0] + 34, k)
    dimv(msp, T(0, 0)[1], T(0, g.BASE_L)[1], T(240, 0)[0], T(240, 0)[0] + 24, k)
    dimv(msp, T(0, 130)[1], T(0, 530)[1], T(240, 0)[0], T(240, 0)[0] + 14, k)
    dimv(msp, T(0, 0)[1], T(0, 130)[1], T(240, 0)[0], T(240, 0)[0] + 14, k)
    dimv(msp, T(0, 0)[1], T(0, 20)[1], T(-150, 0)[0], T(-240, 0)[0] - 12, k)
    dimv(msp, T(0, 20)[1], T(0, 220)[1], T(-150, 0)[0], T(-240, 0)[0] - 12, k)
    dimv(msp, T(0, 0)[1], T(0, g.BF_YC - 46.5)[1], T(-200, 0)[0], T(-240, 0)[0] - 24, k)
    dimv(msp, T(0, g.BK_YC - 46.5)[1], T(0, 525)[1], T(-200, 0)[0], T(-240, 0)[0] - 12, k)

    # --- размеры: вид спереди ---
    F = lambda x, z: (FX + x / k, FY + z / k)
    dimv(msp, F(0, 0)[1], F(0, g.BASE_TOP)[1], F(-240, 0)[0], F(-240, 0)[0] - 10, k)
    dimv(msp, F(0, 0)[1], F(0, g.TABLE_TOP)[1], F(-240, 0)[0], F(-240, 0)[0] - 20, k)
    dimv(msp, F(0, g.BASE_TOP)[1], F(0, g.BASE_TOP + g.STRIP + g.RAIL_H)[1], F(240, 0)[0], F(240, 0)[0] + 10, k)
    dimv(msp, F(0, 0)[1], F(0, g.FOOT_H)[1], F(240, 0)[0], F(240, 0)[0] + 10, k)

    # --- размеры: разрез A-A ---
    A = lambda x, z: (AX + x / k, AY + z / k)
    dimh(msp, A(0, 0)[0], A(g.Y_AXIS_X, 0)[0], A(0, g.Y_AXIS_Z)[1], AY - 10, k)
    dimv(msp, A(0, 0)[1], A(0, g.Y_AXIS_Z)[1], A(-g.Y_AXIS_X, 0)[0], A(-240, 0)[0] - 10, k)
    dimv(msp, A(0, g.BASE_TOP)[1], A(0, g.TABLE_TOP)[1], A(-150, 0)[0], A(-240, 0)[0] - 10, k)
    leader(msp, *A(-60, g.BASE_TOP + 20), -20, 22, "Al 10 + MDF 19 + MDF 8")

    # --- позиции ---
    for n, x, y, bx, by in [(1, -220, 300, -290, 330), (2.2, -100, 545, -290, 545), (3, -124, 400, -290, 400),
                            (4.1, 100, 120, 290, 200), (5, 207, 480, 290, 470), (6, 220, 420, 290, 420),
                            (10, 172, 505, 290, 520), (11, 172, 160, 290, 150), (13, 169, 610, 290, 610),
                            (31, -175, 568, -290, 600), (32, -197, 490, -290, 475), (2.1, -50, 10, -50, -35)]:
        balloon(msp, n, *T(x, y), *T(bx, by))
    for n, x, z, bx, bz in [(4.3, 120, 128, 120, 180), (4.2, 80, 115, 70, 180), (24, 220, 7, 280, 20),
                            (30, -175, 60, -175, 160)]:
        balloon(msp, n, *F(x, z), *F(bx, bz))
    for n, x, z, bx, bz in [(9, 169, 47, 270, 100), (32, 197, 70, 270, 160), (11, -175, 20, -270, 165)]:
        balloon(msp, n, *A(x, z), *A(bx, bz))


PLATE = g.PLATE_T

ASM_BOM = [
    ("1", "Lonzheron, profil 40x80 paz 8", "L=565", 2, "list 2"),
    ("2.1", "Poperechina perednyaya, profil 40x80", "L=400", 1, "list 2"),
    ("2.2", "Poperechina zadnyaya, profil 40x80", "L=400", 1, "list 2"),
    ("3", "Balka stola, profil 40x80", "L=485", 2, "list 2"),
    ("4.1", "Plita stola, Al", "300x200x10", 1, "list 2"),
    ("4.2", "Osnovanie stola, MDF", "300x200x19", 1, "list 2"),
    ("4.3", "Zhertvennyj sloj, MDF", "300x200x8", 1, "list 2"),
    ("5", "Polosa posadochnaya, stal", "400x30x6", 2, "list 2"),
    ("6", "Rels HGR15", "L=400", 2, "pokup., E=20"),
    ("9", "SHVP DFU1605 C7 (spravochno)", "L~400", 2, "pokup."),
    ("10", "Opora BK12", "", 2, "pokup."),
    ("11", "Opora BF12", "", 2, "pokup."),
    ("12", "Mufta 6,35x10 (spravochno)", "D25x30", 2, "pokup."),
    ("13", "Motor NEMA 23, 76 mm", "", 2, "pokup."),
    ("24", "Opora reguliruemaya M8", "", 4, "pokup."),
    ("30", "Plita uglovaya perednyaya, stal", "130x80x6", 2, "list 2"),
    ("31", "Plita uglovaya zadnyaya (motornaya)", "130x80x6", 2, "list 2"),
    ("32", "Plita pod BK12/BF12, stal", "64x80x6", 4, "2 + 2 zerk."),
    ("K1", "Vint M8x55 DIN 912, 8.8", "", 8, "lonzh. - poper."),
    ("K2", "Vint M8x70 DIN 912, 8.8", "", 8, "poper. - balka"),
    ("K3", "Vint M8x20 DIN 912, 8.8", "", 8, "plita - lonzh."),
    ("K4", "Vint M6x12 DIN 912 + T-gajka M6 paz 8", "", 4, "plita 30"),
    ("K5", "Vint M6x12 DIN 7991 + T-gajka M6", "", 16, "plita 32"),
    ("K6", "Vint M5x12 DIN 7991 + T-gajka M5", "", 16, "polosy"),
    ("K7", "Vint M4x16 DIN 912", "", 14, "relsy"),
    ("K8", "Vint M6x16 DIN 7991 + T-gajka M6", "", 4, "plita stola"),
    ("K9", "Vint M6x35 DIN 7991", "", 4, "MDF"),
    ("K10", "Vint M5 DIN 912 pod BK/BF, dlina po mestu", "", 8, "sm. p. 9"),
    ("K11", "Vint M5x10 DIN 912", "", 8, "motory"),
    ("K12", "T-gajka M8 paz 8", "", 4, "opory 24"),
]

ASM_NOTES = [
    "TEHNICHESKIE TREBOVANIYA",
    "1. Razmery v mm. D - diametr. Kolichestva - na odnu bazu.",
    "2. Poryadok sborki:",
    "  a) ramu (1, 2.1, 2.2) sobrat na poverochnoj plite,",
    "     vinty K1 naziv; balki 3 - vintami K2;",
    "  b) raznost diagonalej ramy <= 0,3; verh profilej",
    "     v odnoj ploskosti <= 0,05; zatyanut M8 12 N*m;",
    "  c) plity 30, 31 - vinty K3, K4 (K2 obschie);",
    "  d) opory 24 na T-gajki K12, vystavit po urovnyu;",
    "  e) polosy 5 (K6): obschaya ploskost dvuh polos",
    "     <= 0,02 po linejke, inache shabrit/folga;",
    "  f) bazovyj rels 6 - po linejke, pryamolinejnost",
    "     <= 0,01/100; vtoroj - parallelno bazovomu",
    "     <= 0,03 po indikatoru s karetki; M4 - 4 N*m,",
    "     zatyagivat ot serediny k kontsam;",
    "  g) plity 32 (K5) s BK12/BF12; vint 9 vyverit:",
    "     parallelnost relsu <= 0,05 v dvuh ploskostyah",
    "     po hodu karetki; motory 13 na plity 31 (K11);",
    "  h) plita stola 4.1 na balki (K8); MDF 4.2, 4.3",
    "     (K9); posle sborki stanka frezerovat 4.3",
    "     \"na sebya\", 0,3-0,5 mm.",
    "3. Rezba M8 v Al: fiksator srednej prochnosti.",
    "4. T-gajki - vstavnye (\"molotok\"): stavyatsya",
    "   v sobrannuyu ramu.",
    "5. Gabarit bazy 480 x 653 (s motorami) x 132.",
    "6. Razmery BK12/BF12, motorov, mufty i vintov",
    "   K10 sverit s pokupnymi do sverlovki plit.",
    "7. Dlina K10: konets ne vyhodit iz plity 32",
    "   (<= 6 mm rezby), inache upretsya v profil.",
]


def table(msp, x0, ytop, cols, heads, rows, rh=6.0, h=TS):
    n = len(rows) + 1
    for i in range(n + 1):
        line(msp, (x0, ytop - i * rh), (x0 + cols[-1], ytop - i * rh), "Ramka")
    for c in cols:
        line(msp, (x0 + c, ytop), (x0 + c, ytop - n * rh), "Ramka")
    for c, s in zip(cols, heads):
        text(msp, s, x0 + c + 1.2, ytop - rh * 0.72, h)
    for i, r in enumerate(rows, start=1):
        for c, s in zip(cols, r):
            text(msp, str(s), x0 + c + 1.2, ytop - i * rh - rh * 0.72, h)
    return ytop - n * rh


def notes(msp, x0, y, lines, h=TS, step=4.6):
    for s in lines:
        text(msp, s, x0, y, h)
        y -= step
    return y


def frame(msp, ox, title, sub, scale, sheet):
    x0, y0, x1, y1 = ox + 20, 5, ox + W - 5, H - 5
    for a, b in (((x0, y0), (x1, y0)), ((x1, y0), (x1, y1)), ((x1, y1), (x0, y1)), ((x0, y1), (x0, y0))):
        line(msp, a, b, "Ramka")
    w, h = 185, 40
    bx = x1 - w
    for a, b in [((bx, y0), (bx, y0 + h)), ((bx, y0 + h), (x1, y0 + h)), ((bx, y0 + h / 2), (x1, y0 + h / 2)),
                 ((x1 - 60, y0), (x1 - 60, y0 + h / 2)), ((x1 - 30, y0), (x1 - 30, y0 + h / 2))]:
        line(msp, a, b, "Ramka")
    text(msp, title, bx + 5, y0 + h * 0.66, TH * 1.15)
    text(msp, sub, bx + 5, y0 + h * 0.2, TH * 0.85)
    text(msp, "Masshtab", x1 - 57, y0 + h * 0.3, TS)
    text(msp, scale, x1 - 57, y0 + h * 0.08, TH * 0.9)
    text(msp, "List", x1 - 27, y0 + h * 0.3, TS)
    text(msp, sheet, x1 - 27, y0 + h * 0.08, TH * 0.9)


# ---------------------- лист 2: детали ----------------------
def details(msp, ox):
    # ---- поз. 1: лонжерон, 1:2 ----
    d = Det(msp, ox + 45, 485, 2)
    d.title("Poz. 1  Lonzheron, profil 40x80 paz 8, 6060/6063-T6  2 sht.   M 1:2", dv=66)
    d.rect(0, 0, 565, 80)
    for v in (16, 24, 56, 64):
        d.line(0, v, 565, v, "Tonkie")
    for u in (20, 545):
        for v in CORE:
            d.hole(u, v, 9)
    for u0, u1 in ((0, 15), (550, 565)):
        for v in CORE:
            d.hidden_bore(u0, u1, v, 6.8)
    d.hdim(0, 565, 80, 16)
    d.hdim(0, 20, 80, 8)
    d.hdim(545, 565, 80, 8)
    d.vdim(0, 20, 0, -8)
    d.vdim(20, 60, 0, -8)
    d.vdim(60, 80, 0, -8)
    d.leader(545, 20, -40, -24, "4 otv. D9 skvoz (po osi X)")
    d.leader(10, 20, 30, -26, "M8 gl. 15 v serdechnike, 2 otv. s kazhdogo tortsa")
    e = Det(msp, ox + 345, 485, 2)
    draw_profile_section(msp, e.X(0), e.Y(0), 40, 80, 2, hatch=False)
    e.hdim(0, 40, 80, 8)
    e.vdim(0, 80, 40, 8)
    text(msp, "tores", e.X(20), e.Y(-10), TS, align="CENTER")

    # ---- поз. 3: балка стола, 1:2 ----
    d = Det(msp, ox + 470, 485, 2)
    d.title("Poz. 3  Balka stola, profil 40x80   2 sht.   M 1:2", dv=66)
    d.rect(0, 0, 485, 80)
    for v in (16, 24, 56, 64):
        d.line(0, v, 485, v, "Tonkie")
    for u0, u1 in ((0, 25), (460, 485)):
        for v in CORE:
            d.hidden_bore(u0, u1, v, 6.8)
    d.hdim(0, 485, 80, 10)
    d.vdim(0, 20, 485, 8)
    d.vdim(20, 60, 485, 8)
    d.leader(11, 20, 25, -24, "M8 gl. 25 v serdechnike, 2 otv. s kazhdogo tortsa")

    # ---- поз. 2.1, 2.2: поперечины, 1:2 ----
    for n, (x, y) in enumerate(((45, 380), (310, 380))):
        rear = n == 1
        d = Det(msp, ox + x, y, 2)
        d.title("Poz. 2.%d  Poperechina %s, 40x80   1 sht.   M 1:2" % (n + 1, "zadnyaya" if rear else "perednyaya"), dv=66)
        d.rect(0, 0, 400, 80)
        for v in (16, 24, 56, 64):
            d.line(0, v, 400, v, "Tonkie")
        for u in (76, 324):
            for v in CORE:
                d.hole(u, v, 9)
        for u0, u1 in ((0, 15), (385, 400)):
            for v in CORE:
                d.hidden_bore(u0, u1, v, 6.8)
        d.hdim(0, 400, 80, 18)
        d.hdim(0, 76, 80, 10)
        d.hdim(324, 400, 80, 10)
        d.vdim(0, 20, 400, 8)
        d.vdim(20, 60, 400, 8)
        d.leader(324, 20, 12, -18, "4 otv. D9 skvoz (po osi Y)")
        d.leader(8, 20, 22, -34, "M8 gl. 15, 2 otv. s kazhdogo tortsa")
        if rear:
            for u in (31, 369):
                d.hole(u, 32, 32)
            d.hdim(0, 31, 32, -26)
            d.hdim(369, 400, 32, -26)
            d.vdim(0, 32, 0, -8)
            d.leader(369, 32, -24, -38, "2 otv. D32 skvoz")
            # вид сверху: доступ к винтам муфты
            t = Det(msp, ox + 570, 405, 2)
            t.rect(0, 0, 400, 40)
            t.line(0, 16, 400, 16, "Tonkie")
            t.line(0, 24, 400, 24, "Tonkie")
            for u in (31, 369):
                for v in (12, 28):
                    t.hole(u, v, 8)
            t.vdim(0, 12, 400, 8)
            t.vdim(12, 28, 400, 8)
            t.leader(31, 28, 10, 12, "4 otv. D8 do D32 (klyuch k mufte), po mufte")
            text(msp, "Poz. 2.2 - vid sverhu (gran k stolu - vnizu)", t.X(0), t.Y(-9), TS)

    # ---- поз. 5: полоса, 1:2 ----
    d = Det(msp, ox + 45, 255, 2)
    d.title("Poz. 5  Polosa posadochnaya 30x6, stal 45   2 sht.   M 1:2", dv=48)
    d.rect(0, 0, 400, 30)
    rail = [20 + 60 * i for i in range(7)]
    strip = [10, 50, 110, 170, 230, 290, 350, 390]
    for u in rail:
        d.hole(u, 15, 4, thread=True, mark=False)
    for u in strip:
        d.hole(u, 15, 5.5, mark=False)
        msp.add_circle(d.P(u, 15), 10.4 / 2 / 2, dxfattribs={"layer": "Rama"})
    d.line(-4, 15, 404, 15, "Osevye")
    d.hdim(0, 400, 30, 20)
    d.hdim(0, 20, 30, 10)
    d.hdim(20, 380, 30, 10, "6x60=360")
    d.hdim(0, 10, 0, -10)
    d.hdim(10, 50, 0, -10)
    d.hdim(50, 350, 0, -10, "5x60=300")
    d.hdim(350, 390, 0, -10)
    d.vdim(0, 30, 400, 8)
    d.vdim(0, 15, 0, -8)
    d.leader(380, 15, 12, 26, "7 otv. M4 skvoz (pod rels HGR15)")
    d.leader(390, 15, 22, -26, "8 otv. D5,5, zenk. D10,4x90 (K6)")
    s = Det(msp, ox + 45, 222, 2)
    s.rect(0, 0, 400, 6)
    s.vdim(0, 6, 400, 8, "6 -0,02")
    text(msp, "Shlifovat: ploskostnost 0,01, raznotolschinnost 0,01", s.X(0), s.Y(-12), TS)

    # ---- поз. 30: плита угловая передняя, 1:1 ----
    d = Det(msp, ox + 300, 205, 1)
    d.title("Poz. 30  Plita uglovaya perednyaya, stal   2 sht.   M 1:1", dv=110)
    d.rect(0, 0, 130, 80)
    for u in (14, 110):
        for v in CORE:
            d.hole(u, v, 9)
    for v in CORE:
        d.hole(62, v, 6.6)
    d.hdim(0, 130, 80, 22)
    d.hdim(0, 14, 80, 8)
    d.hdim(0, 62, 80, 15)
    d.hdim(110, 130, 80, 8)
    d.vdim(0, 20, 0, -8)
    d.vdim(20, 60, 0, -8)
    d.vdim(0, 80, 130, 10)
    d.leader(110, 20, 12, -28, "4 otv. D9")
    d.leader(62, 20, -4, -28, "2 otv. D6,6")
    text(msp, "t = 6 h14; levaya = pravaya, perevernut", d.X(0), d.Y(-34), TS)

    # ---- поз. 31: плита моторная, 1:1 ----
    d = Det(msp, ox + 475, 205, 1)
    d.title("Poz. 31  Plita uglovaya zadnyaya (motornaya), stal   2 sht.   M 1:1", dv=110)
    d.rect(0, 0, 130, 80)
    for u in (14, 110):
        for v in CORE:
            d.hole(u, v, 9)
    d.hole(59, 32, 39)
    m = 47.14 / 2
    for du in (-m, m):
        for dv in (-m, m):
            d.hole(59 + du, 32 + dv, 5, thread=True, mark=False)
    d.hdim(0, 130, 80, 22)
    d.hdim(0, 14, 80, 8)
    d.hdim(0, 59, 80, 15)
    d.hdim(110, 130, 80, 8)
    d.hdim(59 - m, 59 + m, 0, -10, "47,14")
    d.vdim(0, 20, 0, -8)
    d.vdim(20, 60, 0, -8)
    d.vdim(0, 32, 130, 10)
    d.vdim(32 - m, 32 + m, 130, 20, "47,14")
    d.leader(110, 60, 12, 14, "4 otv. D9")
    d.leader(59 + m, 32 - m, 18, -16, "4 otv. M5 skvoz")
    text(msp, "D39 H11 (poyasok motora D38,1)", d.X(40), d.Y(-24), TS)
    text(msp, "t = 6 h14; levaya = pravaya, perevernut", d.X(0), d.Y(-34), TS)

    # ---- поз. 32: плита под BK12/BF12, 1:1 ----
    d = Det(msp, ox + 690, 205, 1)
    d.title("Poz. 32  Plita pod BK12/BF12, stal   2 + 2 zerk.   M 1:1", dv=110)
    d.rect(0, 0, 64, 80)
    for u in (10, 27):
        for v in CORE:
            d.hole(u, v, 6.6)
            msp.add_circle(d.P(u, v), 13 / 2, dxfattribs={"layer": "Rama"})
    for v in (9, 55):
        d.hole(46.5, v, 5, thread=True)
    d.hdim(0, 64, 80, 22)
    d.hdim(0, 10, 80, 8)
    d.hdim(10, 27, 80, 8)
    d.hdim(0, 46.5, 80, 15)
    d.vdim(0, 9, 64, 8)
    d.vdim(9, 55, 64, 8, "46")
    d.vdim(0, 20, 0, -8)
    d.vdim(20, 60, 0, -8)
    d.leader(27, 20, 14, -24, "4 otv. D6,6, zenk. D13x90")
    d.leader(46.5, 55, 34, 8, "2 otv. M5 skvoz (BK/BF)")
    text(msp, "t = 6 (5,8...6,0), shlifovat;", d.X(0), d.Y(-34), TS)
    text(msp, "raznotolshchinnost <= 0,02; plity", d.X(0), d.Y(-39), TS)
    text(msp, "odnogo vinta - v pare, raznica <= 0,02", d.X(0), d.Y(-44), TS)
    text(msp, "Vid so storony BK/BF, pravaya;", d.X(0), d.Y(-49), TS)
    text(msp, "levaya - zerkalno; zenk. - so st. BK/BF", d.X(0), d.Y(-54), TS)

    # ---- поз. 4.1: плита стола, 1:2.5 ----
    d = Det(msp, ox + 45, 52, 2.5)
    d.title("Poz. 4.1  Plita stola, Al 5083 lit. / 6082-T6   1 sht.   M 1:2,5", dv=106)
    d.rect(0, 0, 300, 200)
    for i in range(6):
        for j in range(4):
            d.hole(25 + 50 * i, 25 + 50 * j, 6, thread=True, mark=False)
    for u in (26, 274):
        for v in (50, 150):
            d.hole(u, v, 6.6, mark=False)
            msp.add_circle(d.P(u, v), 13 / 2 / 2.5, dxfattribs={"layer": "Rama"})
    d.hdim(0, 300, 200, 18)
    d.hdim(0, 25, 200, 9)
    d.hdim(25, 275, 200, 9, "5x50=250")
    d.vdim(0, 25, 300, 8)
    d.vdim(25, 175, 300, 8, "3x50=150")
    d.vdim(0, 200, 300, 18)
    d.hdim(0, 26, 0, -8)
    d.vdim(0, 50, 0, -8)
    d.vdim(50, 150, 0, -8)
    d.leader(275, 175, 8, 14, "24 otv. M6 skvoz")
    d.leader(274, 50, 30, -12, "4 otv. D6,6, zenk. D13x90 (K8)")
    text(msp, "t = 10 +-0,05, obe storony frezerovat", d.X(0), d.Y(-38), TS)

    # ---- поз. 4.2, 4.3: МДФ, 1:2.5 ----
    d = Det(msp, ox + 255, 52, 2.5)
    d.title("Poz. 4.2 (t=19), 4.3 (t=8)  MDF   po 1 sht.   M 1:2,5", dv=106)
    d.rect(0, 0, 300, 200)
    corners = {(25, 25), (275, 25), (25, 175), (275, 175)}
    for i in range(6):
        for j in range(4):
            u, v = 25 + 50 * i, 25 + 50 * j
            d.hole(u, v, 7, mark=False)
            if (u, v) in corners:
                msp.add_circle(d.P(u, v), 14 / 2 / 2.5, dxfattribs={"layer": "Rama"})
    d.hdim(0, 300, 200, 9)
    d.vdim(0, 200, 300, 8)
    d.leader(275, 175, 8, 14, "24 otv. D7 skvoz, raspolozhenie kak 4.1")
    d.leader(25, 25, 12, -20, "4.3: 4 ugl. otv. zenk. D14x90, gl. +2")
    text(msp, "Propitat epoksidkoj / gruntom", d.X(0), d.Y(-38), TS)

    # ---- примечания ----
    notes(msp, ox + 455, 135, [
        "TEHNICHESKIE TREBOVANIYA K DETALYAM",
        "1. Razmery v mm, D - diametr. Neukazannye",
        "   predelnye otkloneniya +-0,2; koordinaty",
        "   otverstij +-0,1.",
        "2. Profil 40x80 paz 8, 6060-T6 / 6063-T6,",
        "   serdechnik D6,8 s 4 prorezyami do D10,5:",
        "   rezba M8 metchikom bez rassverlivaniya.",
        "   Tortsy frezerovat: dlina +-0,1,",
        "   neperpendikulyarnost <= 0,05.",
        "3. Plity 30, 31, 32: St3 / S235, list 6.",
        "   30, 31: t = 6 h14, holodnokatanyj -",
        "   bez shlifovki. 32: t = 5,8...6,0, shlif.",
        "   parno. Neploskostnost <= 0,05;",
        "   kromki pritupit.",
        "4. Plita 4.1: litaya Al 5083 (ne korobitsya)",
        "   ili 6082-T6; rezba M6 - skvoznaya.",
        "5. Pazy profilej - uslovno.",
    ], step=4.4)


# ---------------------- вывод ----------------------
def main():
    doc = g.setup_doc()
    ds = doc.dimstyles.new("T1")
    for k, v in dict(dimtxsty="CHERT", dimdec=2, dimzin=8, dimdsep=ord(","), dimlfac=1).items():
        ds.dxf.set(k, v)
    g.apply_dim_defaults(doc, dimtxt=DIM_TXT, dimasz=2.5, dimexo=1, dimexe=1.5, dimgap=0.8)
    doc.header["$LTSCALE"] = 0.4
    msp = doc.modelspace()

    draw_assembly(msp)
    y = table(msp, 625, 585, [0, 12, 120, 152, 162, 211], ["Poz.", "Naimenovanie", "Razmer", "Kol.", "Primech."],
              ASM_BOM, rh=5.6)
    notes(msp, 625, y - 7, ASM_NOTES, step=4.3)
    frame(msp, 0, "Baza so stolom. Sborochnyj chertezh", "Spec. v2.2. Detali - list 2", "1:2,5; 1:1", "1 / 2")

    details(msp, SHEET2_DX)
    frame(msp, SHEET2_DX, "Baza so stolom. Detali", "Spec. v2.2. Sborka - list 1", "1:2; 1:1", "2 / 2")

    auditor = doc.audit()
    if auditor.has_errors:
        raise SystemExit(f"Ошибки DXF: {auditor.errors}")
    doc.saveas(OUT.with_suffix(".dxf"), encoding="utf-8")

    cfg = Configuration(background_policy=BackgroundPolicy.WHITE, color_policy=ColorPolicy.COLOR,
                        min_lineweight=0.15, lineweight_scaling=1.0)
    with PdfPages(OUT.with_suffix(".pdf")) as pdf:
        for ox in (0, SHEET2_DX):
            fig = plt.figure(figsize=(W / 25.4, H / 25.4))
            ax = fig.add_axes([0, 0, 1, 1])
            Frontend(RenderContext(doc), MatplotlibBackend(ax, adjust_figure=False), config=cfg).draw_layout(msp)
            ax.set_xlim(ox, ox + W)
            ax.set_ylim(0, H)
            pdf.savefig(fig)
            plt.close(fig)
    print("OK:", OUT.with_suffix(".dxf").name, OUT.with_suffix(".pdf").name)


if __name__ == "__main__":
    main()
