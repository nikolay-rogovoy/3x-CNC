#!/usr/bin/env python3
"""Сборочный чертёж портала с осями X и Z и чертежи его деталей (DXF + PDF, 3 листа А1).

Геометрия сборки берётся из generate_drawing.build(fasteners=True) — та же модель,
что и у компоновочного чертежа, со всеми проверками столкновений. Оформление и
примитивы — общие с generate_table_drawing. Все надписи — латиницей.

Запуск: python3 generate_portal_drawing.py
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
import generate_table_drawing as t
from generate_table_drawing import (Det, TH, TS, W, H, text, line, dimh, dimv, leader, balloon,
                                    hatch_poly, table, notes, frame, CORE)

OUT = Path(__file__).resolve().parent / "чертёж-портала"
SHEETS = (0.0, 900.0, 1800.0)
K_ASM = 2.5
FC = g.TABLE_Y0 + g.WORK_Y / 2 + g.TOOL_OFFSET       # центр опорной плиты 8 по Y (инструмент в центре поля)
SF = -g.BF - g.SADDLE_FRONT                           # лицо суппорта, y относительно FC
CS = 47.14 / 2                                       # полуквадрат крепления NEMA 23


# ---------------------- общие примитивы ----------------------
def cbore(d, u, v, dh, dc, layer="Rama", mark=True):
    """Отверстие dh с цековкой/зенковкой dc (вид со стороны цековки)."""
    d.hole(u, v, dh, layer=layer, mark=mark)
    d.msp.add_circle(d.P(u, v), dc / 2 / d.k, dxfattribs={"layer": layer})


def poly(d, pts, layer="Rama"):
    for a, b in zip(pts, pts[1:] + pts[:1]):
        d.line(*a, *b, layer)


def vbore(d, u, v0, v1, dia):
    """Скрытое отверстие вдоль v."""
    for uu in (u - dia / 2, u + dia / 2):
        d.line(uu, v0, uu, v1, "Nevidimye")
    d.line(u, v0 - 2, u, v1 + 2, "Osevye")


def hbore(d, u0, u1, v, dia):
    d.hidden_bore(u0, u1, v, dia)


def hole_table(msp, x0, ytop, rows, title=None):
    if title:
        text(msp, title, x0, ytop + 2, TS)
    return table(msp, x0, ytop, [0, 12, 64, 72, 100, 142], ["Obozn.", "Otverstie", "Kol.", "X", "Y"], rows, rh=5.0)


# ---------------------- лист 1: сборка ----------------------
def shifted(parts):
    out = []
    for p in parts:
        q = copy(p)
        q.y = (p.y[0] - FC, p.y[1] - FC)
        out.append(q)
    return out


def views(fx, fy, sx, sy, tx, ty, bx, by, k=K_ASM, kb=2.0):
    return {
        "front": (lambda p: (p.x, p.z), lambda p: p.y[0], lambda u, v: (fx + u / k, fy + v / k), "y"),
        "top": (lambda p: (p.x, p.y), lambda p: -p.z[1], lambda u, v: (tx + u / k, ty + v / k), "z"),
        "side": (lambda p: (tuple(sorted((-p.y[0], -p.y[1]))), p.z), lambda p: p.x[0],
                 lambda u, v: (sx + u / k, sy + v / k), "x"),
        "secx": (lambda p: (tuple(sorted((-p.y[0], -p.y[1]))), p.z), lambda p: p.x[0],
                 lambda u, v: (bx + u / kb, by + v / kb), "x"),
    }


def draw_assembly(msp):
    k = K_ASM
    FX, FY = 150.0, 312.0          # вид спереди: x=0, z=0
    SX, SY = 345.0, 312.0          # вид слева: y'=0, z=0 (u = -y')
    TX, TY = 150.0, 208.0          # вид сверху: x=0, y'=0
    BX, BY = 372.0, -2.0           # разрез B-B 1:2
    vw = views(FX, FY, SX, SY, TX, TY, BX, BY)
    allp = shifted(g.build(fasteners=True))
    parts = [p for p in allp if p.group != "base"]
    rails_y = [p for p in allp if p.name in ("рельс Y", "полоса") and p.y[1] > -90 and p.y[0] < 90]
    for v in ("front", "top", "side"):
        g.draw_view(msp, parts + rails_y, v, vw)

    # --- разрез B-B по оси инструмента (x = 0), смотреть слева ---
    sec, cut = [], []
    for p in parts:
        if p.x[1] <= 0 or p.x[0] >= 140 or p.layer == "Krepezh":
            continue
        q = copy(p)
        q.hidden = False
        if p.x[0] < 0:
            q.x = (0, p.x[1])
            cut.append(q)
        sec.append(q)
    g.draw_view(msp, sec, "secx", vw)
    S = lambda y, z: (BX - y / 2, BY + z / 2)            # y' -> лист (u = -y')
    for p in cut:
        y0, y1, z0, z1 = p.y[0], p.y[1], p.z[0], p.z[1]
        rect = lambda a, b, c, d_: [S(a, c), S(b, c), S(b, d_), S(a, d_)]
        if p.pos == 17:                                  # труба: стенки 5
            inner = rect(y0 + 5, y1 - 5, z0 + 5, z1 - 5)
            for a, b in zip(inner, inner[1:] + inner[:1]):
                line(msp, a, b)
            hatch_poly(msp, rect(y0, y1, z0, z1), angle=45, scale=0.4)
            msp.query("HATCH").last.paths.add_polyline_path(inner, is_closed=True,
                                                             flags=ezdxf.const.BOUNDARY_PATH_OUTERMOST)
        elif p.pos == 28:                                # хомут: стенки до и после шпинделя
            ya = (y0 + y1) / 2 + (y1 - y0) / 2 - g.CLAMP_WALL - g.SPINDLE_D / 2
            for a, b in ((y0, ya - g.SPINDLE_D / 2), (ya + g.SPINDLE_D / 2, y1)):
                hatch_poly(msp, rect(a, b, z0, z1), angle=135, scale=0.4)
        elif p.pos in (22, 26, 19.4):
            hatch_poly(msp, rect(y0, y1, z0, z1), angle=135 if p.pos == 26 else 45,
                       scale=0.4 if p.pos != 19.4 else 0.3)
        elif p.pos == 16 and p.name == "кронштейн гайки X":
            zc, yc = g.BEAM_Z0 + g.BEAM_H + 10 + g.BKF_CH, -g.BF + g.BEAM_D / 2
            c = S(yc, zc)
            hatch_poly(msp, rect(y0, y1, z0, z1), [(c, g.NUT_R / 2)], angle=135, scale=0.4)
        elif p.pos == 16 and p.name == "кронштейн гайки Z":
            yn = SF - g.BKF_CH + g.NUT_R
            hatch_poly(msp, rect(yn, y1, z0, z1), angle=135, scale=0.4)
        elif p.pos == 5:
            hatch_poly(msp, rect(y0, y1, z0, z1), angle=45, scale=0.25)
    text(msp, "B-B (1:2)", BX - 30, BY + 290, TH * 1.2, align="CENTER")
    text(msp, "stojki, plity 8 ne pokazany", BX - 30, BY + 284, TS, align="CENTER")

    # --- оси и обозначения ---
    line(msp, (FX, FY + 100 / k), (FX, FY + 625 / k), "Osevye")
    line(msp, (TX, TY - 290 / k), (TX, TY + 95 / k), "Osevye")
    ya = g.BF + g.SADDLE_FRONT + g.TOOL_FROM_SADDLE          # ось шпинделя: y' = -ya
    line(msp, (SX + ya / k, SY + 120 / k), (SX + ya / k, SY + 625 / k), "Osevye")
    line(msp, (BX + ya / 2, BY + 120 / 2), (BX + ya / 2, BY + 625 / 2), "Osevye")
    # след разреза B-B на виде сверху
    for yy, sgn in ((-300, -1), (100, 1)):
        y0 = TY + yy / k
        line(msp, (TX, y0), (TX, y0 + sgn * 8), "Rama")
        line(msp, (TX, y0 + sgn * 4), (TX - 7, y0 + sgn * 4), "Rama")
        msp.add_solid([(TX - 7, y0 + sgn * 4 - 1.2), (TX - 7, y0 + sgn * 4 + 1.2), (TX - 10, y0 + sgn * 4)],
                      dxfattribs={"layer": "Rama"})
        text(msp, "B", TX - 14, y0 + sgn * 4 - 1.5, TH)
    text(msp, "VID SPEREDI", FX + 30, FY + 640 / k, TH * 1.2, align="CENTER")
    text(msp, "VID SVERHU", TX - 60, TY + 80 / k + 18, TH * 1.2, align="CENTER")
    text(msp, "VID SLEVA", SX + 90 / k, SY + 640 / k, TH * 1.2, align="CENTER")

    F = lambda x, z: (FX + x / k, FY + z / k)
    T = lambda x, y: (TX + x / k, TY + y / k)
    L = lambda y, z: (SX - y / k, SY + z / k)
    # --- размеры: вид спереди ---
    dimh(msp, F(-246, 0)[0], F(286 + g.MOTOR_L, 0)[0], F(0, 129)[1], FY - 14, k)
    dimh(msp, F(-220, 0)[0], F(220, 0)[0], F(0, 116)[1], FY - 7, k, "440 (osi relsov Y)")
    dimh(msp, F(-240, 0)[0], F(240, 0)[0], F(0, 370)[1], FY + 640 / k - 12, k)
    dimv(msp, F(0, 0)[1], F(0, 129)[1], F(-246, 0)[0], F(-246, 0)[0] - 8, k)
    dimv(msp, F(0, 0)[1], F(0, 270)[1], F(-246, 0)[0], F(-246, 0)[0] - 16, k)
    dimv(msp, F(0, 0)[1], F(0, 370)[1], F(-246, 0)[0], F(-246, 0)[0] - 24, k)
    zx = g.BEAM_Z0 + g.BEAM_H + 10 + g.BKF_CH
    dimv(msp, F(0, 370)[1], F(0, zx)[1], F(286 + g.MOTOR_L, 0)[0], F(286 + g.MOTOR_L, 0)[0] + 8, k)
    dimv(msp, F(0, 0)[1], F(0, g.SADDLE_Z1 + 6 + g.MOTOR_L)[1], F(-246, 0)[0], F(-246, 0)[0] - 32, k)
    dimh(msp, F(-g.Z_RAIL_DX, 0)[0], F(g.Z_RAIL_DX, 0)[0], F(0, 200)[1], FY + 105 / k - 2, k, "150 (osi relsov Z)")
    # --- размеры: вид сверху ---
    dimh(msp, T(-200, 0)[0], T(200, 0)[0], T(0, 80)[1], TY + 80 / k + 8, k, "400 (mezhdu stojkami)")
    dimv(msp, T(0, -80)[1], T(0, 80)[1], T(246, 0)[0], T(246, 0)[0] + 10, k)
    dimv(msp, T(0, -10)[1], T(0, 70)[1], T(246, 0)[0], T(246, 0)[0] + 18, k)
    dimv(msp, T(0, -ya)[1], T(0, 0)[1], T(-246, 0)[0], T(-246, 0)[0] - 10, k, "%d (vylet)" % ya)
    # --- размеры: вид слева ---
    dimh(msp, L(-60, 0)[0], L(-10, 0)[0], L(0, 370)[1], SY + 640 / k - 12, k)
    dimh(msp, L(-10, 0)[0], L(70, 0)[0], L(0, 370)[1], SY + 640 / k - 12, k)
    dimh(msp, L(-ya, 0)[0], L(0, 0)[0], L(0, 129)[1], SY + 105 / k - 10, k)
    dimh(msp, L(0, 0)[0], L(80, 0)[0], L(0, 129)[1], SY + 105 / k - 10, k)
    # --- размеры: разрез B-B (пакет кареток) ---
    bf = -g.BF
    B = lambda y, z: S(y, z)
    dimh(msp, B(bf, 0)[0], B(bf - 34, 0)[0], B(0, 300)[1], BY + 250 / 2 - 8, 2.0)
    dimh(msp, B(bf - 34, 0)[0], B(bf - 54, 0)[0], B(0, 240)[1], BY + 250 / 2 - 8, 2.0)
    dimh(msp, B(SF, 0)[0], B(SF - 28, 0)[0], B(0, 240)[1], BY + 250 / 2 - 8, 2.0)
    dimh(msp, B(SF - 28, 0)[0], B(SF - 48, 0)[0], B(0, 175)[1], BY + 250 / 2 - 8, 2.0)
    dimh(msp, B(SF - 48, 0)[0], B(-ya, 0)[0], B(0, 175)[1], BY + 250 / 2 - 8, 2.0)
    dimh(msp, B(bf, 0)[0], B(-ya, 0)[0], B(0, 175)[1], BY + 250 / 2 - 16, 2.0)
    dimv(msp, B(0, g.SADDLE_Z0)[1], B(0, g.SADDLE_Z1)[1], B(SF, 0)[0], B(bf + 10, 0)[0] + 12, 2.0)
    dimv(msp, B(0, g.TABLE_TOP)[1], B(0, g.TABLE_TOP + g.STICKOUT)[1], B(-ya, 0)[0], B(-ya, 0)[0] - 30, 2.0)
    text(msp, "kontsy instrumenta - na urovne stola (Z niz)", B(-ya, 0)[0] - 2, B(0, g.TABLE_TOP)[1] - 6, TS,
         align="CENTER")

    # --- позиции ---
    pos_front = [(14, -220, 300, -290, 330), (15, 243, 200, 300, 230), (8, 200, 136, 300, 150),
                 (17, -120, 360, -150, 410), ("5.2", -180, 273, -300, 268), (18, -170, 288, -300, 295),
                 (22, 70, 500, 150, 560), ("19.1", 230, 375, 260, 470), ("19.3", 281, 430, 330, 470),
                 ("19.2", -220, 375, -240, 440), (10, 228, 395, 245, 520), (11, -230, 395, -275, 470),
                 (20, -150, 405, -200, 450), (13, 330, 405, 340, 520), (29, 0, 330, 120, 345),
                 (28, -50, 210, -150, 230), (7, -220, 115, -290, 100), (21, -60, 285, -150, 250),
                 (12, 265, 405, 310, 335), (12, 0, 516, -110, 560)]
    for n, x, z, bx, bz in pos_front:
        balloon(msp, n, *F(x, z), *F(bx, bz))
    pos_side = [("16.2", -40, 410, 40, 470), (26, SF - 38, 200, SF - 100, 160), (27, SF - 20, 300, SF - 90, 320),
                (25, SF - 12, 360, SF - 90, 380), ("16.3", SF - 15, 390, SF - 100, 430),
                ("19.4", SF - 30, 535, SF - 100, 560), (23, SF - 25, 480, SF - 100, 500),
                ("16.1", -30, 60, 40, 30), (9, 0, 47, 60, 70), ("K1", 75, 295, 120, 330),
                ("K2", 30, 189, 110, 220)]
    for n, y, z, by, bz in pos_side:
        balloon(msp, n, *L(y, z), *L(by, bz), r=5 if len(str(n)) < 3 else 5.5)
    balloon(msp, 33, *T(220, -20), *T(270, -70))


ASM_BOM = [
    ("5.2", "Polosa X, stal 45", "480x30x6", 2, "list 2"),
    ("7", "Karetka HGH15CA (os Y)", "", 4, "pokup."),
    ("8", "Plita opornaya, stal (svar. s 15)", "91x160x15", 2, "1 + 1 zerk."),
    ("9", "Gajka SHVP Y DFU1605 (vint - chert. bazy)", "", 2, "pokup."),
    ("10", "Opora BK12 (X, Z)", "", 2, "pokup."),
    ("11", "Opora BF12 (X)", "", 1, "pokup."),
    ("12", "Mufta 6,35x10 (X, Z)", "D25x30", 2, "pokup."),
    ("13", "Motor NEMA 23, 76 mm (X, Z)", "", 2, "pokup."),
    ("14", "Stojka, profil 40x80 paz 8", "L=226", 2, "list 2"),
    ("15", "Scheka, stal", "160x100x6", 4, "list 3"),
    ("16.1", "Kronshtejn gajki Y, svarnoj, stal", "40x60x106", 2, "1 + 1 zerk."),
    ("16.2", "Kronshtejn gajki X, Al (kvadrat 40x40)", "40x83x40", 1, "list 3"),
    ("16.3", "Kronshtejn gajki Z, Al", "100x26x30", 1, "list 3"),
    ("17", "Balka X, truba Al 100x50x5", "L=480", 1, "list 2"),
    ("18", "Rels HGR15 (os X)", "L=480", 2, "pokup., E=30"),
    ("19.1", "Plita BK12 X, Al", "81x60x10", 1, "list 3"),
    ("19.2", "Plita BF12 X, Al", "40x60x10", 1, "list 3"),
    ("19.3", "Plita motora X, Al", "60x85x10", 1, "list 3"),
    ("19.4", "Plita motora Z, stal", "70x80x6", 1, "list 3"),
    ("20", "SHVP DFU1605 C7 (os X)", "L~520", 1, "pokup."),
    ("21", "Karetka HGH15CA (os X)", "", 4, "pokup."),
    ("22", "Support X, Al", "200x292x20", 1, "list 2"),
    ("23", "SHVP DFU1605 C7 (os Z)", "L~205", 1, "pokup."),
    ("25", "Karetka HGH15CA (os Z)", "", 4, "pokup."),
    ("26", "Plita Z, Al", "180x240x20", 1, "list 2"),
    ("27", "Rels HGR15 (os Z)", "L=220", 2, "pokup., E=20"),
    ("28", "Homut shpindelya D80, Al", "120x110x60", 2, "list 3"),
    ("29", "Shpindel 1,5 kVt ER11, D80", "", 1, "pokup."),
    ("33", "Planka zakladnaya, stal", "80x30x10", 2, "list 3"),
    ("K1", "Vint M8x100 DIN 912, 8.8", "", 4, "stojka - balka"),
    ("K2", "Vint M8x65 DIN 912 + gajka DIN 985", "", 8, "scheki - stojka"),
    ("K3", "Vint M8x30 DIN 912", "", 2, "plita 8 - stojka"),
    ("K4", "Vint M4x12 DIN 912", "", 16, "karetki Y"),
    ("K5", "Vint M4x16 DIN 912", "", 32, "karetki X, Z"),
    ("K6", "Vint M4x16 DIN 912", "", 24, "relsy X, Z"),
    ("K7", "Vint M5x12 DIN 7991", "", 24, "polosy, plity 19"),
    ("K8", "Vint M5x16 DIN 7991", "", 5, "plity 19.3, 19.4"),
    ("K9", "Vint M5 DIN 912, dlina po mestu", "", 6, "BK12/BF12"),
    ("K10", "Vint M5x16 / M5x12 DIN 912", "", "4/4", "motory X / Z"),
    ("K11", "Vint M5x16 DIN 912", "", 15, "flantsy gaek"),
    ("K12", "Vint M6x25 DIN 912", "", 16, "kronshtejny 16"),
    ("K13", "Vint M6x35 DIN 912", "", 8, "homuty 28"),
    ("K14", "Vint M8x40 DIN 912", "", 2, "styazhka homutov"),
]

ASM_NOTES = [
    "TEHNICHESKIE TREBOVANIYA",
    "1. Razmery v mm. Polozhenie: instrument v tsentre polya,",
    "   Z vnizu. Spec. v2.2. Baza so stolom - otd. chertezh.",
    "2. Poryadok sborki:",
    "  a) scheki 15 privarit k plite 8: snaruzhi sploshnoj shov,",
    "     katet 6, kromka s razdelkoj 30x4; vnutri paza - tolko",
    "     pered stojkoj (0...65 ot pereda), pod stojkoj NE varit;",
    "     zatem frezerovat niz plity (ploskostnost 0,02) i paz",
    "     40 +0,1...+0,3 mezhdu schekami;",
    "  b) plity 8 na karetki Y (K4) - DO ustanovki stoek:",
    "     4 vinta iz 8 okazhutsya pod stojkoj;",
    "  c) stojki 14 v paz, K3 snizu; otv. D9 v stojke sverlit",
    "     po schekam, K2 - 25 N*m;",
    "  d) planki 33 zavesti v trubu s tortsov, balka 17 na",
    "     stojki (K1): perpendikulyarnost relsam Y <= 0,05",
    "     na 300 (diagonali portala), K1 - 25 N*m;",
    "  e) polosy 5.2 (K7): obschaya ploskost <= 0,02; bazovyj",
    "     rels 18 - po linejke <= 0,01/100, vtoroj parallelno",
    "     <= 0,02 po indikatoru s karetki; M4 - 4 N*m;",
    "  f) support 22: SNACHALA karetki Z (K5 s tyla),",
    "     POTOM karetki X (K5 speredi) - inache ne dostat;",
    "  g) plita Z 26: relsy 27, homuty 28, kronshtejn 16.3;",
    "     rels Z zavesti v karetki Z sverhu;",
    "  h) BK/BF na plity 19, vint X vyverit: parallelnost",
    "     relsam <= 0,05 v dvuh ploskostyah (folga pod plity);",
    "     kronshtejny gaek 16 zatyagivat POSLEDNIMI, progonyaya",
    "     karetku iz kraya v kraj; to zhe dlya Y i Z;",
    "  i) shpindel: perpendikulyarnost stolu <= 0,05/100",
    "     v dvuh ploskostyah (folga pod homuty).",
    "3. Rezba v Al: fiksator srednej prochnosti.",
    "4. Razmery kupnyh (BK/BF, flanets DFU1605, motor,",
    "   shpindel) sverit do sverlovki.",
]


# ---------------------- лист 2: крупные детали ----------------------
STRIP_X = [10, 60, 120, 180, 240, 300, 360, 420, 470]
RAIL_X = [30 + 60 * i for i in range(8)]


def sheet2(msp, ox):
    # ---- поз. 17: балка X ----
    d = Det(msp, ox + 45, 470, 2)
    d.title("Poz. 17  Balka X, truba Al 100x50x5 (6060/6063-T6)   1 sht.   M 1:2", dv=88)
    d.rect(0, 0, 480, 100)
    for v in (18, 82):
        for u in STRIP_X:
            d.hole(u, v, 5, thread=True, mark=False)
        for u in RAIL_X:
            d.hole(u, v, 5, mark=False)
        d.line(-4, v, 484, v, "Osevye")
    for u in (20, 460):
        for v in (25, 75):
            d.hole(u, v, 9, layer="Nevidimye", mark=False)
    d.hdim(0, 480, 100, 28)
    d.hdim(0, 10, 100, 9)
    d.hdim(10, 60, 100, 9)
    d.hdim(60, 420, 100, 9, "6x60=360")
    d.hdim(420, 470, 100, 9)
    d.hdim(0, 30, 0, -9)
    d.hdim(30, 450, 0, -9, "7x60=420")
    d.hdim(0, 20, 0, -17)
    d.hdim(460, 480, 0, -17)
    d.vdim(0, 18, 0, -8)
    d.vdim(18, 82, 0, -8)
    d.vdim(0, 25, 480, 8)
    d.vdim(25, 75, 480, 8)
    d.vdim(0, 100, 480, 16)
    d.leader(470, 82, 6, 22, "18 otv. M5 skvoz (polosy, K7)")
    d.leader(390, 18, 8, -30, "16 otv. D5 skvoz (pod kontsy vintov relsa)")
    d.leader(460, 25, -150, -42, "4 otv. D9 v ZADNEJ stenke (skryto)")
    text(msp, "vid speredi", d.X(0), d.Y(-30), TS)
    # сечение трубы
    s = Det(msp, ox + 300, 470, 2)
    s.rect(0, 0, 50, 100)
    s.rect(5, 5, 45, 95)
    hatch_poly(msp, [s.P(0, 0), s.P(50, 0), s.P(50, 100), s.P(0, 100)], scale=0.35)
    h = msp.query("HATCH").last
    h.paths.add_polyline_path([s.P(5, 5), s.P(45, 5), s.P(45, 95), s.P(5, 95)], is_closed=True,
                              flags=ezdxf.const.BOUNDARY_PATH_OUTERMOST)
    s.hdim(0, 50, 100, 8)
    s.vdim(0, 100, 50, 8)
    s.hdim(0, 5, 0, -7)
    text(msp, "sechenie", s.X(25), s.Y(-14), TS, align="CENTER")
    text(msp, "pered - sleva", s.X(0), s.Y(-24), TS)
    # вид сверху
    d = Det(msp, ox + 45, 388, 2)
    d.rect(0, 0, 480, 50)
    d.line(0, 5, 480, 5, "Nevidimye")
    d.line(0, 45, 480, 45, "Nevidimye")
    top = [(10, 25), (30, 10), (30, 40), (445, 10), (445, 40), (467.5, 25)]
    for u, v in top:
        d.hole(u, v, 5, thread=True, mark=False)
    d.hdim(0, 10, 50, 8)
    d.hdim(0, 30, 50, 15)
    d.hdim(445, 480, 50, 15)
    d.hdim(467.5, 480, 50, 8)
    d.vdim(10, 40, 0, -8)
    d.vdim(0, 25, 480, 8)
    d.leader(445, 40, -20, 16, "6 otv. M5 v verhnej stenke (plity 19.1, 19.2)")
    text(msp, "vid sverhu (pered - vnizu)", d.X(0), d.Y(-10), TS)

    # ---- поз. 5.2: полоса X ----
    d = Det(msp, ox + 45, 312, 2)
    d.title("Poz. 5.2  Polosa X 30x6, stal 45   2 sht.   M 1:2", dv=48)
    d.rect(0, 0, 480, 30)
    for u in RAIL_X:
        d.hole(u, 15, 4, thread=True, mark=False)
    for u in STRIP_X:
        cbore(d, u, 15, 5.5, 10.4, mark=False)
    d.line(-4, 15, 484, 15, "Osevye")
    d.hdim(0, 480, 30, 22)
    d.hdim(0, 30, 30, 10)
    d.hdim(30, 450, 30, 10, "7x60=420")
    d.hdim(0, 10, 0, -10)
    d.hdim(10, 60, 0, -10)
    d.hdim(60, 420, 0, -10, "6x60=360")
    d.hdim(420, 470, 0, -10)
    d.vdim(0, 30, 480, 8)
    d.leader(450, 15, 14, 24, "8 otv. M4 skvoz (rels HGR15)")
    d.leader(470, 15, 10, -24, "9 otv. D5,5, zenk. D10,4x90 (K7)")
    e = Det(msp, ox + 45, 282, 2)
    e.rect(0, 0, 480, 6)
    e.vdim(0, 6, 480, 8, "6 -0,02")
    text(msp, "Shlifovat: ploskostnost 0,01, raznotolschinnost 0,01; obe polosy - v pare", e.X(0), e.Y(-12), TS)

    # ---- поз. 14: стойка ----
    d = Det(msp, ox + 360, 470, 2)
    d.title("Poz. 14  Stojka, profil 40x80 paz 8   2 sht.   M 1:2", dv=72)
    d.rect(0, 0, 226, 80)
    for v in (16, 24, 56, 64):
        d.line(0, v, 226, v, "Tonkie")
    for u in (30, 75):
        for v in CORE:
            d.hole(u, v, 9)
    for u in (151, 201):
        vbore(d, u, 0, 80, 9)
    hbore(d, 0, 15, 20, 6.8)
    d.hdim(0, 226, 80, 24)
    d.hdim(0, 30, 80, 10)
    d.hdim(30, 75, 80, 10)
    d.hdim(0, 151, 80, 17)
    d.hdim(151, 201, 80, 17)
    d.vdim(0, 20, 0, -8)
    d.vdim(20, 60, 0, -8)
    d.vdim(0, 80, 226, 8)
    d.leader(75, 20, 20, -18, "4 otv. D9 skvoz po X - sverlit po schekam 15")
    text(msp, "vid sboku (gran k scheke); niz - sleva, pered - vnizu", d.X(0), d.Y(-44), TS)
    e = Det(msp, ox + 360, 400, 2)
    e.rect(0, 0, 226, 40)
    e.line(0, 16, 226, 16, "Tonkie")
    e.line(0, 24, 226, 24, "Tonkie")
    for u in (151, 201):
        e.hole(u, 20, 9)
    for u in (30, 75):
        vbore(e, u, 0, 40, 9)
    e.vdim(0, 20, 226, 8)
    e.vdim(0, 40, 226, 16)
    e.leader(201, 20, 14, -24, "2 otv. D9 skvoz po Y (K1)")
    e.leader(4, 20, 18, 24, "M8 gl. 15 v perednem serdechnike (K3)")
    text(msp, "vid speredi (gran k balke)", e.X(0), e.Y(-28), TS)

    # ---- поз. 22: суппорт ----
    d = Det(msp, ox + 400, 32, 2)
    d.title("Poz. 22  Support X, Al 6082-T6 / 5083   1 sht.   M 1:2", dv=168)
    d.rect(0, 0, 200, 292)
    A = [(100 + sx * dx, v) for sx in (-1, 1) for dx in (27, 53) for v in (35, 61, 99, 125)]
    B_ = [(100 + sx * dx, v) for sx in (-1, 1) for dx in (62, 88) for v in (27, 53, 107, 133)]
    C = [(100 + sx * 12, v) for sx in (-1, 1) for v in (153, 177)]
    D = [(100 + sx * 23, 244.5) for sx in (-1, 1)]
    for u, v in A:
        cbore(d, u, v, 4.5, 8, mark=False)
    for u, v in B_:
        d.hole(u, v, 4.5, mark=False)
        d.msp.add_circle(d.P(u, v), 4 / 2, dxfattribs={"layer": "Nevidimye"})
    for u, v in C:
        cbore(d, u, v, 6.6, 11, mark=False)
    for u, v in D:
        d.hole(u, v, 5, thread=True, mark=False)
    d.line(100, -5, 100, 297, "Osevye")
    for n, (u, v) in (("A", A[3]), ("B", B_[7]), ("C", C[3]), ("D", D[1])):
        text(msp, n, d.X(u) + 2.5, d.Y(v) + 1.5, TS)
    d.hdim(0, 200, 292, 8)
    d.vdim(0, 292, 200, 8)
    for u in (75, 100, 125):
        d.line(u, 292, u, 280, "Nevidimye")
    text(msp, "vid speredi (so storony plity Z)", d.X(0), d.Y(-8), TS)
    hole_table(msp, ox + 515, 160, [
        ("A", "D4,5, tsek. D8 gl. 8 speredi", 16, "+-27; +-53", "35; 61; 99; 125"),
        ("B", "D4,5, tsek. D8 gl. 8 SZADI", 16, "+-62; +-88", "27; 53; 107; 133"),
        ("C", "D6,6, tsek. D11 gl. 6,5 speredi", 4, "+-12", "153; 177"),
        ("D", "M5 gl. 12 speredi (BK12 Z)", 2, "+-23", "244,5"),
        ("E", "M5 gl. 12 v verhnem tortse", 3, "0; +-25", "po t. 10"),
    ], "Poz. 22: otverstiya. X - ot osi, Y - ot niza")
    text(msp, "A - karetki X (K5), B - karetki Z (K5), C - kronshtejn 16.2 (K12)", ox + 515, 124, TS)

    # ---- поз. 26: плита Z ----
    d = Det(msp, ox + 630, 280, 2)
    d.title("Poz. 26  Plita Z, Al 6082-T6 / 5083   1 sht.   M 1:2", dv=146)
    poly(d, [(0, 0), (180, 0), (180, 240), (116, 240), (116, 75), (64, 75), (64, 240), (0, 240)])
    E = [(90 + sx * 75, v) for sx in (-1, 1) for v in (30, 90, 150, 210)]
    Fh = [(90 + sx * 50, v) for sx in (-1, 1) for v in (22, 58, 132, 168)]
    G = [(90 + sx * 40, v) for sx in (-1, 1) for v in (207, 223)]
    for u, v in E:
        d.hole(u, v, 4, thread=True, mark=False)
    for u, v in Fh:
        d.hole(u, v, 6, thread=True, mark=False)
    for u, v in G:
        cbore(d, u, v, 6.6, 11, mark=False)
    d.line(90, -5, 90, 245, "Osevye")
    for n, (u, v) in (("E", E[3]), ("F", Fh[7]), ("G", G[3])):
        text(msp, n, d.X(u) + 2.5, d.Y(v) + 1.5, TS)
    d.hdim(0, 180, 240, 8)
    d.hdim(64, 116, 240, 15)
    d.vdim(0, 75, 180, 8)
    d.vdim(0, 240, 180, 16)
    text(msp, "vid speredi (so storony homutov)", d.X(0), d.Y(-8), TS)
    text(msp, "okno: R3 v uglah", d.X(0), d.Y(-13), TS)
    hole_table(msp, ox + 630, 255, [
        ("E", "M4 skvoz (relsy Z, K6)", 8, "+-75", "30; 90; 150; 210"),
        ("F", "M6 skvoz (homuty, K13)", 8, "+-50", "22; 58; 132; 168"),
        ("G", "D6,6, tsek. D11 gl. 6,5 speredi", 4, "+-40", "207; 223"),
    ], "Poz. 26: otverstiya. X - ot osi, Y - ot niza")
    text(msp, "G - kronshtejn 16.3 (K12), s tylnoj storony", ox + 630, 231, TS)

    notes(msp, ox + 45, 232, [
        "TEHNICHESKIE TREBOVANIYA K DETALYAM (LISTY 2, 3)",
        "1. Razmery v mm, D - diametr. Neukazannye predelnye",
        "   otkloneniya +-0,2; koordinaty otverstij +-0,1.",
        "2. Plity 22, 26: litaya 5083 ili 6082-T6, obe storony",
        "   frezerovat, ploskostnost 0,02, t = 20 +-0,05.",
        "3. Truba 17: tortsy podrezat, perpendikulyarnost 0,1;",
        "   otv. v perednej stenke sverlit po polose 5.2 kak",
        "   po konduktoru.",
        "4. Profil 14 - kak list 2 bazy: 6060/6063-T6, rezba M8",
        "   metchikom v serdechnike bez rassverlivaniya.",
        "5. Plita 8 + scheki 15, kronshtejn 16.1 - St3 / S235;",
        "   svarnye uzly obrabatyvat posle svarki (list 1, 2a).",
        "6. Kromki pritupit. Pazy profilya - uslovno.",
    ], step=4.4)


# ---------------------- лист 3: мелкие детали 1:1 ----------------------
def sheet3(msp, ox):
    # ---- поз. 8: опорная плита ----
    d = Det(msp, ox + 50, 360, 1)
    d.title("Poz. 8  Plita opornaya, stal 15   1 + 1 zerk.   M 1:1", dv=198)
    d.rect(0, 0, 91, 160)
    for u, w in ((39, 45), (85, 91)):
        d.rect(u, 0, w, 160, "Tonkie")
    d.rect(45, 70, 85, 150, "Nevidimye")                 # стойка 14
    for u in (47, 83):                                   # внутренние швы только перед стойкой
        d.line(u, 0, u, 65, "Tonkie")
    d.leader(47, 40, -10, -75, "vnutri paza shov - tolko 0...65")
    text(msp, "stojka", d.X(65), d.Y(110), TS, align="CENTER")
    for u in (52, 78):
        for v in (17, 43, 117, 143):
            cbore(d, u, v, 4.5, 8, mark=False)
    d.hole(65, 90, 9)
    d.msp.add_circle(d.P(65, 90), 7, dxfattribs={"layer": "Nevidimye"})
    for u in (10, 28):
        for v in (28, 72):
            d.hole(u, v, 6.6, mark=False)
    d.hdim(0, 91, 160, 30)
    d.hdim(0, 52, 160, 9)
    d.hdim(52, 78, 160, 9)
    d.hdim(0, 65, 160, 18)
    d.hdim(0, 10, 0, -9)
    d.hdim(10, 28, 0, -9)
    d.hdim(45, 85, 0, -9, "40 +0,1/+0,3")
    d.vdim(0, 17, 0, -8)
    d.vdim(17, 43, 0, -8)
    d.vdim(43, 117, 0, -8)
    d.vdim(117, 143, 0, -8)
    d.vdim(0, 90, 0, -17)
    d.vdim(0, 28, 91, 8)
    d.vdim(28, 72, 91, 8)
    d.vdim(0, 160, 91, 17)
    d.leader(78, 143, 22, 12, "8 otv. D4,5, tsek. D8 gl. 7 sverhu (K4)")
    d.leader(65, 90, 40, 6, "D9, tsek. D14 gl. 9 SNIZU (K3)")
    d.leader(28, 72, 30, -10, "4 otv. D6,6 (16.1, K12)")
    text(msp, "vid sverhu, pravaya; pered - vnizu,", d.X(0), d.Y(-18), TS)
    text(msp, "vnutr. kromka - sleva; scheki 15 - tonko", d.X(0), d.Y(-23), TS)
    e = Det(msp, ox + 175, 360, 1)
    e.rect(0, 0, 15, 160)
    e.hdim(0, 15, 160, 9)
    text(msp, "t=15", e.X(0), e.Y(-8), TS)

    # ---- поз. 15: щека ----
    d = Det(msp, ox + 225, 430, 1)
    d.title("Poz. 15  Scheka, stal 6   4 sht.   M 1:1", dv=128)
    d.rect(0, 0, 160, 100)
    for u in (90, 130):
        for v in (30, 75):
            d.hole(u, v, 9)
    d.hdim(0, 160, 100, 20)
    d.hdim(0, 90, 100, 10)
    d.hdim(90, 130, 100, 10)
    d.vdim(0, 30, 160, 8)
    d.vdim(30, 75, 160, 8)
    d.vdim(0, 100, 160, 16)
    d.leader(130, 30, 12, -40, "4 otv. D9")
    text(msp, "niz - k plite 8: razdelka snaruzhi 30x4, shov katet 6;", d.X(0), d.Y(-10), TS)
    text(msp, "pered - sleva; t = 6", d.X(0), d.Y(-15), TS)

    # ---- поз. 16.1: кронштейн гайки Y, сварной ----
    d = Det(msp, ox + 440, 420, 1)
    d.title("Poz. 16.1  Kronshtejn gajki Y, svarnoj, stal   1 + 1 zerk.   M 1:1", dv=128)
    poly(d, [(0, 0), (40, 0), (40, 106), (6, 106), (6, 72), (0, 72)])
    d.line(6, 96, 40, 96, "Tonkie")
    d.hole(20, 24, 29)
    d.hdim(0, 40, 106, 9)
    d.hdim(0, 6, 106, 16)
    d.hdim(0, 20, 0, -9)
    d.vdim(0, 24, 0, -8)
    d.vdim(0, 72, 0, -16)
    d.vdim(0, 106, 40, 8)
    d.vdim(96, 106, 40, 16)
    d.leader(20, 24, 30, -36, "D29 skvoz - posle svarki")
    text(msp, "vid speredi", d.X(0), d.Y(-46), TS)
    e = Det(msp, ox + 520, 420, 1)
    poly(e, [(20, 0), (40, 0), (40, 96), (60, 96), (60, 106), (0, 106), (0, 96), (20, 96)])
    for u, sg in ((20, -1), (40, 1)):                    # угловые швы катет 5 под полкой
        tri = [e.P(u, 96), e.P(u + sg * 5, 96), e.P(u, 91)]
        for a, b in zip(tri, tri[1:] + tri[:1]):
            line(msp, a, b)
        h = msp.add_hatch(color=7, dxfattribs={"layer": "Shtrihovka"})
        h.paths.add_polyline_path(tri, is_closed=True)
    for v in (8, 52):
        vbore(e, v, 96, 106, 6)
    hbore(e, 20, 40, 24, 29)
    e.hdim(0, 60, 106, 9)
    e.hdim(20, 40, 0, -9)
    e.hdim(0, 8, 106, 16)
    e.hdim(8, 52, 106, 16)
    e.leader(40, 93, 16, 10, "shov 5 s dvuh storon, po 34")
    e.leader(40, 50, 14, -18, "4 otv. M5 po flantsu gajki (K11)")
    text(msp, "vid sboku (pered - sleva)", e.X(0), e.Y(-46), TS)
    notes(msp, e.X(-80), e.Y(-53), [
        "4 otv. M6 skvoz v polke (K12): X 16; 34, Y 8; 52.",
        "Zagotovki St3: stojka - polosa 40x20, L=96 (stupenka",
        "6x24 vypilit do svarki); polka - polosa 60x12, L=34.",
        "Posle svarki frezerovat: verh polki v t=10, ploskostnost",
        "0,05; zadnyuyu gran (pod flanets) - perpendikulyarno",
        "verhu 0,05/40, t >= 18; D29 - os perpendikulyarna",
        "zadnej grani; zatem rezby. Grunt/kraska, krome",
        "privalochnyh poverhnostej.",
    ], step=4.4)

    # ---- поз. 16.2: кронштейн гайки X ----
    d = Det(msp, ox + 640, 440, 1)
    d.title("Poz. 16.2  Kronshtejn gajki X, Al (kvadrat 40x40)   1 sht.   M 1:1", dv=62)
    d.rect(0, 0, 83, 40)
    d.hole(59, 20, 28.5)
    for v in (8, 32):
        hbore(d, 0, 12, v, 5)
    d.hdim(0, 83, 40, 16)
    d.hdim(0, 59, 40, 8)
    d.vdim(0, 20, 83, 8)
    d.vdim(0, 40, 83, 16)
    d.vdim(0, 8, 0, -8)
    d.vdim(8, 32, 0, -8)
    d.leader(59, 20, 6, -34, "D28,5 H8 skvoz; flanets - 4 otv. M5 (K11)")
    text(msp, "vid sboku; sleva - k supportu 22", d.X(0), d.Y(-12), TS)
    e = Det(msp, ox + 650, 365, 1)
    e.rect(0, 0, 40, 40)
    for u in (8, 32):
        for v in (8, 32):
            e.hole(u, v, 6, thread=True)
    e.hdim(0, 40, 40, 8)
    e.hdim(8, 32, 0, -8)
    e.vdim(20 - 14.25, 20 + 14.25, 40, 8, "D28,5")
    for v in (20 - 14.25, 20 + 14.25):
        e.line(0, v, 40, v, "Nevidimye")
    e.leader(32, 8, 14, -14, "4 otv. M6 gl. 12 (K12)")
    text(msp, "vid na gran k supportu", e.X(0), e.Y(-22), TS)
    for k_, t_ in enumerate((
            "Zagotovka: kvadrat Al 40x40 (AD31T1 / D16T), L 86;",
            "dopuskaetsya paket 4 plit Al 10 (83x40), styanut",
            "vintami flantsa M5 naskvoz, D28,5 - v sbore.",
            "Tortsy frezerovat v razmer 83; gran k supportu i",
            "gran pod flanets - perpendikulyarno osi D28,5 <= 0,05.",
            "Flanets gajki - lyskami vverh/vniz; M5 - po flantsu.")):
        text(msp, t_, e.X(60), e.Y(34 - k_ * 5), TS)

    # ---- поз. 16.3: кронштейн гайки Z ----
    d = Det(msp, ox + 50, 150, 1)
    d.title("Poz. 16.3  Kronshtejn gajki Z, Al   1 sht.   M 1:1", dv=50)
    a0 = math.degrees(math.asin(3 / 14.25))
    w = math.sqrt(14.25 ** 2 - 9)
    d.line(0, 0, 50 - w, 0)
    d.line(50 + w, 0, 100, 0)
    d.line(100, 0, 100, 26)
    d.line(100, 26, 0, 26)
    d.line(0, 26, 0, 0)
    msp.add_arc(d.P(50, 3), 14.25, -a0, 180 + a0, dxfattribs={"layer": "Rama"})
    d.line(50, -3, 50, 20, "Osevye")
    for u in (10, 90):
        vbore(d, u, 0, 12, 5)
    d.hdim(0, 100, 26, 16)
    d.hdim(0, 50, 26, 8)
    d.hdim(10, 90, 0, -9)
    d.vdim(0, 3, 100, 8)
    d.vdim(0, 26, 100, 16)
    d.leader(50, 8, 30, -26, "D28,5 H8, os v 3 ot perednej grani")
    text(msp, "vid sverhu; pered (k plite Z) - vnizu", d.X(0), d.Y(-24), TS)
    e = Det(msp, ox + 50, 60, 1)
    e.rect(0, 0, 100, 30)
    for u in (10, 90):
        for v in (7, 23):
            e.hole(u, v, 6, thread=True)
    for u in (50 - w, 50 + w):
        e.line(u, 0, u, 30)
    e.hdim(50 - w, 50 + w, 30, 8, "27,9")
    e.vdim(0, 7, 100, 8)
    e.vdim(7, 23, 100, 8)
    e.vdim(0, 30, 100, 16)
    e.leader(90, 23, 16, 14, "4 otv. M6 gl. 12 (K12)")
    text(msp, "vid speredi; 3 otv. M5 sverhu - po flantsu (K11)", e.X(0), e.Y(-10), TS)

    # ---- поз. 19.1 ----
    d = Det(msp, ox + 215, 205, 1)
    d.title("Poz. 19.1  Plita BK12 X, Al 10   1 sht.   M 1:1", dv=88)
    d.rect(0, 0, 81, 60)
    for v in (7, 53):
        d.hole(32.5, v, 5, thread=True)
    for u, v in ((10, 15), (10, 45), (32.5, 30)):
        cbore(d, u, v, 5.5, 10.4, mark=False)
    for v in (22, 38):
        hbore(d, 69, 81, v, 4.2)
    d.hdim(0, 81, 60, 18)
    d.hdim(0, 10, 60, 9)
    d.hdim(0, 32.5, 0, -9)
    d.vdim(0, 7, 0, -8)
    d.vdim(7, 53, 0, -8)
    d.vdim(15, 45, 81, 8)
    d.vdim(22, 38, 81, 16)
    d.vdim(0, 60, 81, 24)
    d.leader(32.5, 53, 30, 18, "2 otv. M5 skvoz (BK12)")
    d.leader(10, 15, -6, -22, "3 otv. D5,5 zenk. D10,4 (K7)")
    d.leader(75, 22, 10, -26, "2 otv. M5 gl. 12 v tortse (19.3)")
    text(msp, "BK12 os - po u = 32,5; levyj kraj - k tsentru", d.X(0), d.Y(-32), TS)

    # ---- поз. 19.2 ----
    d = Det(msp, ox + 360, 205, 1)
    d.title("Poz. 19.2  Plita BF12 X, Al 10   1 sht.", dv=88)
    d.rect(0, 0, 40, 60)
    for v in (7, 53):
        d.hole(10, v, 5, thread=True)
    for u, v in ((30, 15), (30, 45), (10, 30)):
        cbore(d, u, v, 5.5, 10.4, mark=False)
    d.hdim(0, 40, 60, 18)
    d.hdim(0, 10, 60, 9)
    d.hdim(10, 30, 60, 9)
    d.vdim(0, 7, 0, -8)
    d.vdim(7, 53, 0, -8)
    d.vdim(15, 45, 40, 8)
    d.vdim(0, 60, 40, 16)
    d.leader(10, 53, 22, 18, "2 otv. M5 (BF12)")
    d.leader(30, 15, 8, -22, "3 otv. D5,5 zenk. D10,4")
    text(msp, "levyj kraj - torets balki", d.X(0), d.Y(-32), TS)

    # ---- поз. 19.3 ----
    d = Det(msp, ox + 470, 175, 1)
    d.title("Poz. 19.3  Plita motora X, Al 10   1 sht.  (D39 H11 - poyasok motora D38,1)", dv=112)
    d.rect(0, 0, 60, 85)
    d.hole(30, 50, 39)
    for du in (-CS, CS):
        for dv in (-CS, CS):
            d.hole(30 + du, 50 + dv, 5, thread=True, mark=False)
    for u in (22, 38):
        cbore(d, u, 20, 5.5, 10.4, mark=False)
    d.hdim(0, 60, 85, 18)
    d.hdim(30 - CS, 30 + CS, 85, 9, "47,14")
    d.hdim(22, 38, 0, -9)
    d.vdim(0, 20, 60, 8)
    d.vdim(0, 50, 60, 16)
    d.vdim(50 - CS, 50 + CS, 0, -8, "47,14")
    d.vdim(0, 85, 60, 24)
    d.leader(30 + CS, 50 - CS, 30, -20, "4 otv. M5 skvoz (motor)")
    d.leader(38, 20, 30, -26, "2 otv. D5,5, zenk. so st. motora (K8)")

    # ---- поз. 19.4 ----
    d = Det(msp, ox + 610, 175, 1)
    d.title("Poz. 19.4  Plita motora Z, stal 6   1 sht.", dv=107)
    d.rect(0, 0, 70, 80)
    d.hole(35, 35, 39)
    for du in (-CS, CS):
        for dv in (-CS, CS):
            d.hole(35 + du, 35 + dv, 5, thread=True, mark=False)
    for u in (10, 35, 60):
        cbore(d, u, 70, 5.5, 10.4, mark=False)
    d.line(0, 60, 70, 60, "Tonkie")
    d.hdim(0, 70, 80, 18)
    d.hdim(10, 60, 80, 9)
    d.hdim(35 - CS, 35 + CS, 0, -9, "47,14")
    d.vdim(0, 35, 70, 8)
    d.vdim(60, 80, 70, 16)
    d.vdim(0, 80, 70, 24)
    d.leader(60, 70, 18, -48, "3 otv. D5,5 zenk. (K8)")
    text(msp, "D39 H11; 4 otv. M5 (motor)", d.X(0), d.Y(-22), TS)
    text(msp, "tonkaya - gran supporta 22", d.X(0), d.Y(-27), TS)

    # ---- поз. 33: планка ----
    d = Det(msp, ox + 215, 40, 1)
    d.title("Poz. 33  Planka zakladnaya, stal 10   2 sht.", dv=96)
    d.rect(0, 0, 30, 80)
    for v in (15, 65):
        d.hole(15, v, 8, thread=True)
    d.hdim(0, 30, 80, 8)
    d.vdim(0, 15, 30, 8)
    d.vdim(15, 65, 30, 8)
    d.vdim(0, 80, 30, 16)
    d.leader(15, 65, 24, 8, "2 otv. M8 skvoz (K1)")

    # ---- поз. 28: хомут ----
    d = Det(msp, ox + 330, 30, 1)
    d.title("Poz. 28  Homut shpindelya, Al 6082-T6   2 sht.   M 1:1 (vid sverhu)", dv=132)
    d.rect(0, 0, 120, 110)
    d.hole(60, 55, 80)
    d.line(59, 95, 59, 110)
    d.line(61, 95, 61, 110)
    for u in (10, 110):
        vbore(d, u, 20, 110, 11)
        vbore(d, u, 0, 20, 6.6)
    hbore(d, 0, 50, 102.5, 13)
    hbore(d, 61, 85, 102.5, 8)
    d.hdim(0, 120, 110, 16)
    d.hdim(0, 10, 110, 8)
    d.hdim(110, 120, 110, 8)
    d.vdim(0, 55, 120, 8)
    d.vdim(0, 20, 120, 16)
    d.vdim(0, 110, 120, 24)
    d.vdim(102.5, 110, 0, -8)
    d.leader(60, 15, 46, -20, "D80 H7, razrez 2 do otv.")
    text(msp, "zad (k plite Z) - vnizu; vysota 60", d.X(0), d.Y(-8), TS)
    text(msp, "4 otv. D6,6, tsek. D11 do 20 ot zadnej grani, Z 12; 48 (K13)", d.X(0), d.Y(-13), TS)
    text(msp, "styazhka: D9, tsek. D13 sleva / M8 sprava, Z 30 (K14)", d.X(0), d.Y(-18), TS)


# ---------------------- вывод ----------------------
def main():
    g.check_collisions()
    doc = g.setup_doc()
    ds = doc.dimstyles.new("T1")
    for k, v in dict(dimtxsty="CHERT", dimdec=2, dimzin=8, dimdsep=ord(","), dimlfac=1).items():
        ds.dxf.set(k, v)
    g.apply_dim_defaults(doc, dimtxt=t.DIM_TXT, dimasz=2.5, dimexo=1, dimexe=1.5, dimgap=0.8)
    doc.header["$LTSCALE"] = 0.4
    msp = doc.modelspace()

    draw_assembly(msp)
    y = table(msp, 625, 585, [0, 12, 120, 152, 162, 211], ["Poz.", "Naimenovanie", "Razmer", "Kol.", "Primech."],
              ASM_BOM, rh=5.0)
    notes(msp, 625, y - 6, ASM_NOTES, step=4.1)
    frame(msp, SHEETS[0], "Portal s osyami X i Z. Sborka", "Spec. v2.2. Detali - listy 2, 3", "1:2,5; 1:2", "1 / 3")
    sheet2(msp, SHEETS[1])
    frame(msp, SHEETS[1], "Portal. Detali", "Spec. v2.2. Sborka - list 1", "1:2", "2 / 3")
    sheet3(msp, SHEETS[2])
    frame(msp, SHEETS[2], "Portal. Detali", "Spec. v2.2. Sborka - list 1", "1:1", "3 / 3")

    auditor = doc.audit()
    if auditor.has_errors:
        raise SystemExit(f"Ошибки DXF: {auditor.errors}")
    doc.saveas(OUT.with_suffix(".dxf"), encoding="utf-8")

    cfg = Configuration(background_policy=BackgroundPolicy.WHITE, color_policy=ColorPolicy.COLOR,
                        min_lineweight=0.15, lineweight_scaling=1.0)
    with PdfPages(OUT.with_suffix(".pdf")) as pdf:
        for ox in SHEETS:
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
