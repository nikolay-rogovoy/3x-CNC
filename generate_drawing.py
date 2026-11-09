#!/usr/bin/env python3
"""Генератор компоновочного чертежа 3-осевого CNC (DXF + PDF).

Все виды строятся из одной модели (параллелепипеды и цилиндры, мм), поэтому
виды не могут разойтись между собой. Перед выводом проверяются столкновения
узлов во всех крайних положениях осей; при столкновении чертёж не создаётся.

Координаты: X — вправо, Y — от оператора назад, Z — вверх, Z=0 — поверхность верстака.

Запуск: pip install ezdxf matplotlib && python3 generate_drawing.py
"""
import itertools
import math
from dataclasses import dataclass
from pathlib import Path

import ezdxf
from ezdxf.addons.drawing import Frontend, RenderContext
from ezdxf.addons.drawing.config import BackgroundPolicy, ColorPolicy, Configuration
from ezdxf.addons.drawing.matplotlib import MatplotlibBackend
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

OUT = Path(__file__).resolve().parent / "чертёж-станка"

# --- Покупные узлы ---
RAIL_W, RAIL_H, CAR_W, CAR_L, CAR_H = 15, 15, 34, 61.4, 28   # HGR15 / HGH15CA
SCREW_R, NUT_R, NUT_L = 8, 14, 100        # DFU1605; длина гайки ~100 — сверить у поставщика
FLANGE_R, FLANGE_FLAT, FLANGE_T = 24, 20, 10
BK_L, BF_L, BKF_W, BKF_H, BKF_CH = 25, 20, 60, 43, 25  # сверить с чертежом поставщика
MOTOR, MOTOR_L = 57, 76                   # NEMA 23, корпус 76 мм, 1,9 Н·м
SPINDLE_D, SPINDLE_L = 80, 240            # 1,5 кВт, ER11; длина с запасом — сверить с покупным
STICKOUT = 35                             # вылет инструмента от торца шпинделя

# --- Главные параметры ---
WORK_X, WORK_Y, WORK_Z = 300, 200, 60
TABLE_Y0 = 20
FOOT_H = 15                 # регулируемые опоры
BASE_H = 80
BASE_TOP = FOOT_H + BASE_H
STRIP = 5.5                 # посадочная полоса под рельс: полоса 6, шлифовать в 5,5
RAIL_X = 220                # рельсы Y: X = ±220
Y_RAIL_L, Y_RAIL_Y0 = 400, 130
BASE_L = 565
CROSS_D = 40
BEAM_LEN, BEAM_H, BEAM_D = 480, 100, 50
BEAM_WALL = 4               # балка X: стальная труба 100x50x4
BEAM_Z0 = 270
GANTRY_PLATE_T = 9         # опорная плита портала (поз. 8): лист 10, фрезеровать в 9
NUT_BR_H, NUT_BR_T, NUT_BR_SHELF = 106, 19, 9  # кронштейн 16.1: высота, стойка, полка (после фрезеровки)
X_RAIL_L = 480
X_RAIL_DZ = 32              # рельсы X: ±32 от оси балки
SADDLE_T, SADDLE_W = 20, 200     # ширина 200: каретки Z вне зоны винтов кареток X
SADDLE_Z0, SADDLE_Z1 = 240, 532
ZPLATE_T, ZPLATE_W, ZPLATE_L = 20, 180, 240
Z_RAIL_L, Z_RAIL_DX = 220, 75   # рельсы Z ±75: винты кареток X не закрыты каретками Z
CLAMP_SPACER, CLAMP_WALL = 0, 15
WINDOW_Z0 = 75              # низ окна плиты Z: хвост винта при верхнем положении

TABLE_TOP = BASE_TOP + 10 + 19 + 8
UPRIGHT_D = 80
UPRIGHT_SHIFT = 30          # стойка смещена назад: балка ближе к центру кареток Y
Z_CAR_C = 320               # центр пакета кареток Z
BK_Z0 = 472                 # низ BK12 оси Z, над верхним положением плиты
Y_AXIS_X = 169              # ось винта Y: BK/BF стоят на плите 6 мм на внутренней грани лонжерона
Y_AXIS_Z = 47
BEAM_X0, BEAM_X1 = 104, 144 # балки стола (ось ±124)
PLATE_T = 6                 # стальные плиты: угловые, моторные, под BK/BF
CORNER_X0 = 110             # угловые плиты от X=±110 до ±240
BF_YC, BK_YC = 160, 507.5   # центры BF12 и BK12 по Y
MOTOR_Y0 = BASE_L + PLATE_T # фланец мотора Y на задней плите
SCREW_Y = (145, 547)        # винт Y с обработанными концами
COUPLING_Y = (532, 562)
# Вылет оси шпинделя вперёд от центра опорной плиты gantry
BF = UPRIGHT_D / 2 + BEAM_D - UPRIGHT_SHIFT      # центр плиты -> передняя грань балки
SADDLE_REAR = STRIP + CAR_H                      # передняя грань балки -> тыл суппорта
SADDLE_FRONT = SADDLE_REAR + SADDLE_T            # передняя грань балки -> лицо суппорта
TOOL_FROM_SADDLE = CAR_H + ZPLATE_T + CLAMP_SPACER + CLAMP_WALL + SPINDLE_D / 2
TOOL_OFFSET = BF + SADDLE_FRONT + TOOL_FROM_SADDLE
Z_LOW = TABLE_TOP           # кончик инструмента на столе


@dataclass
class P:
    pos: int
    name: str
    layer: str
    group: str
    x: tuple
    y: tuple
    z: tuple
    axis: str = ""          # "" — параллелепипед; "x"/"y"/"z" — цилиндр
    hidden: bool = False    # показывать невидимые участки штрихом


def cyl(pos, name, layer, group, axis, c1, c2, r, a0, a1, hidden=False):
    """Цилиндр: c1, c2 — центр в двух остальных осях (порядок x, y, z)."""
    lo, hi = sorted((a0, a1))
    if axis == "x":
        return P(pos, name, layer, group, (lo, hi), (c1 - r, c1 + r), (c2 - r, c2 + r), "x", hidden)
    if axis == "y":
        return P(pos, name, layer, group, (c1 - r, c1 + r), (lo, hi), (c2 - r, c2 + r), "y", hidden)
    return P(pos, name, layer, group, (c1 - r, c1 + r), (c2 - r, c2 + r), (lo, hi), "z", hidden)


def base_parts(fasteners=False):
    """Неподвижная база со столом. fasteners — показать болты M8/M6 (для сборочного чертежа)."""
    parts = []
    a = parts.append
    zc = (FOOT_H + 20, FOOT_H + 60)          # оси сердечников профиля 40x80 по высоте
    for s in (-1, 1):
        a(P(1, "лонжерон", "Rama", "base", sorted((s * 200, s * 240)), (0, BASE_L), (FOOT_H, BASE_TOP)))
        a(P(3, "балка стола", "Rama", "base", sorted((s * BEAM_X0, s * BEAM_X1)), (CROSS_D, BASE_L - CROSS_D), (FOOT_H, BASE_TOP)))
        a(P(5, "полоса", "Relsy", "base", sorted((s * (RAIL_X - 15), s * (RAIL_X + 15))), (Y_RAIL_Y0, Y_RAIL_Y0 + Y_RAIL_L), (BASE_TOP, BASE_TOP + STRIP)))
        a(P(6, "рельс Y", "Relsy", "base", sorted((s * (RAIL_X - RAIL_W / 2), s * (RAIL_X + RAIL_W / 2))), (Y_RAIL_Y0, Y_RAIL_Y0 + Y_RAIL_L), (BASE_TOP + STRIP, BASE_TOP + STRIP + RAIL_H)))
        ax = s * Y_AXIS_X
        a(cyl(9, "винт Y", "SHVP", "base", "y", ax, Y_AXIS_Z, SCREW_R, *SCREW_Y, True))
        x_in = 200 - PLATE_T
        for yc, n, name, ln in ((BF_YC, 11, "BF12 Y", BF_L), (BK_YC, 10, "BK12 Y", BK_L)):
            a(P(32, "плита опоры Y", "Uzly", "base", sorted((s * x_in, s * 200)), (yc - 46.5, yc + 17.5), (FOOT_H, BASE_TOP), "", True))
            a(P(n, name, "Uzly", "base", sorted((s * x_in, s * (x_in - BKF_H))), (yc - ln / 2, yc + ln / 2), (Y_AXIS_Z - BKF_W / 2, Y_AXIS_Z + BKF_W / 2), "", True))
        a(cyl(12, "муфта Y", "Privod", "base", "y", ax, Y_AXIS_Z, 12.5, *COUPLING_Y, True))
        a(P(13, "мотор Y", "Privod", "base", (ax - MOTOR / 2, ax + MOTOR / 2), (MOTOR_Y0, MOTOR_Y0 + MOTOR_L), (Y_AXIS_Z - MOTOR / 2, Y_AXIS_Z + MOTOR / 2)))
        a(P(30, "плита угловая", "Uzly", "base", sorted((s * CORNER_X0, s * 240)), (-PLATE_T, 0), (FOOT_H, BASE_TOP)))
        a(P(31, "плита моторная", "Uzly", "base", sorted((s * CORNER_X0, s * 240)), (BASE_L, BASE_L + PLATE_T), (FOOT_H, BASE_TOP)))
        for yy in (30, BASE_L - 30):
            a(cyl(24, "опора", "Rama", "base", "z", s * 220, yy, 15, 0, FOOT_H))
        if fasteners:
            for z in zc:
                for yy in (CROSS_D / 2, BASE_L - CROSS_D / 2):        # K1: лонжерон -> поперечина
                    a(cyl(41, "винт M8x55", "Krepezh", "base", "x", yy, z, 4, s * 185, s * 240, True))
                    a(cyl(41, "головка M8", "Krepezh", "base", "x", yy, z, 6.5, s * 240, s * 248))
                bx = s * (BEAM_X0 + BEAM_X1) / 2                     # K2: плита + поперечина -> балка
                a(cyl(42, "винт M8x70", "Krepezh", "base", "y", bx, z, 4, -PLATE_T, CROSS_D + 24, True))
                a(cyl(42, "винт M8x70", "Krepezh", "base", "y", bx, z, 4, BASE_L - CROSS_D - 24, BASE_L + PLATE_T, True))
                for x in (bx, s * 220):                              # K3: плита -> торец лонжерона
                    a(cyl(43, "головка M8", "Krepezh", "base", "y", x, z, 6.5, -PLATE_T - 8, -PLATE_T))
                    a(cyl(43, "головка M8", "Krepezh", "base", "y", x, z, 6.5, BASE_L + PLATE_T, BASE_L + PLATE_T + 8))
                a(cyl(43, "винт M8x20", "Krepezh", "base", "y", s * 220, z, 4, -PLATE_T, 14, True))
                a(cyl(43, "винт M8x20", "Krepezh", "base", "y", s * 220, z, 4, BASE_L - 14, BASE_L + PLATE_T, True))
                a(cyl(44, "головка M6", "Krepezh", "base", "y", s * 172, z, 5, -PLATE_T - 6, -PLATE_T))
    a(P(2, "поперечина перед", "Rama", "base", (-200, 200), (0, CROSS_D), (FOOT_H, BASE_TOP)))
    a(P(2, "поперечина зад", "Rama", "base", (-200, 200), (BASE_L - CROSS_D, BASE_L), (FOOT_H, BASE_TOP)))
    # ---------- Стол ----------
    tx0, tx1, ty0, ty1 = -WORK_X / 2, WORK_X / 2, TABLE_Y0, TABLE_Y0 + WORK_Y
    a(P(4, "плита", "Stol", "base", (tx0, tx1), (ty0, ty1), (BASE_TOP, BASE_TOP + 10)))
    a(P(4, "МДФ", "Stol", "base", (tx0, tx1), (ty0, ty1), (BASE_TOP + 10, BASE_TOP + 29)))
    a(P(4, "жертвенный", "Stol", "base", (tx0, tx1), (ty0, ty1), (BASE_TOP + 29, TABLE_TOP)))
    return parts


def build(tx=0.0, ty=None, tz=0.0, fasteners=False):
    """tx, ty, tz — положение инструмента относительно центра рабочего поля и низа Z.
    fasteners — крепёж портала (для сборочного чертежа портала)."""
    ty = TABLE_Y0 + WORK_Y / 2 if ty is None else ty
    tool_y = ty
    z = Z_LOW + tz
    parts = base_parts()
    a = parts.append
    # ---------- Gantry ----------
    fc = tool_y + TOOL_OFFSET
    car_z0 = BASE_TOP + STRIP + 4
    foot_z = BASE_TOP + STRIP + CAR_H
    bf = fc - BF                       # передняя грань балки
    for s in (-1, 1):
        for dy in (-50, 50):
            a(P(7, "каретка Y", "Uzly", "gantry", sorted((s * (RAIL_X - CAR_W / 2), s * (RAIL_X + CAR_W / 2))), (fc + dy - CAR_L / 2, fc + dy + CAR_L / 2), (car_z0, foot_z)))
        a(P(8, "опорная плита", "Uzly", "gantry", sorted((s * 155, s * 246)), (fc - 80, fc + 80), (foot_z, foot_z + GANTRY_PLATE_T)))
        a(P(14, "стойка", "Rama", "gantry", sorted((s * 200, s * 240)), (fc + UPRIGHT_SHIFT - UPRIGHT_D / 2, fc + UPRIGHT_SHIFT + UPRIGHT_D / 2), (foot_z + GANTRY_PLATE_T, BEAM_Z0 + BEAM_H)))
        for xx in (s * 194, s * 240):
            a(P(15, "щека", "Uzly", "gantry", sorted((xx, xx + s * 6)), (fc - 80, fc + 80), (foot_z + GANTRY_PLATE_T, foot_z + GANTRY_PLATE_T + 100)))
        ax = s * Y_AXIS_X
        # кронштейн 16.1 сварной, всё из листа 10: стойка 2 пластины, задняя грань фрезеруется в 19;
        # полка фрезеруется в 9; выше верха профиля сужен (край стола)
        by = (fc - 20 - NUT_BR_T, fc - 20)
        sh = foot_z - NUT_BR_SHELF
        a(P(16, "кронштейн гайки Y", "Uzly", "gantry", sorted((ax - s * 20, ax + s * 20)), by, (foot_z - NUT_BR_H, BASE_TOP), "", True))
        a(P(16, "кронштейн гайки Y", "Uzly", "gantry", sorted((ax - s * 14, ax + s * 20)), by, (BASE_TOP, sh), "", True))
        a(P(16, "кронштейн гайки Y", "Uzly", "gantry", sorted((ax - s * 14, ax + s * 20)), (fc - 60, fc), (sh, foot_z), "", True))
        a(cyl(9, "гайка Y", "SHVP", "gantry", "y", ax, Y_AXIS_Z, NUT_R, fc - NUT_L / 2, fc + NUT_L / 2, True))
        # фланец гайки D48 с лысками 40 (лыски по X)
        a(P(9, "фланец гайки Y", "SHVP", "gantry", (ax - FLANGE_FLAT, ax + FLANGE_FLAT), (fc - 20, fc - 20 + FLANGE_T), (Y_AXIS_Z - FLANGE_R, Y_AXIS_Z + FLANGE_R), "", True))
    a(P(17, "балка X", "Rama", "gantry", (-BEAM_LEN / 2, BEAM_LEN / 2), (bf, bf + BEAM_D), (BEAM_Z0, BEAM_Z0 + BEAM_H)))
    bc = BEAM_Z0 + BEAM_H / 2
    for s in (-1, 1):
        rz = bc + s * X_RAIL_DZ
        a(P(5, "полоса X", "Relsy", "gantry", (-X_RAIL_L / 2, X_RAIL_L / 2), (bf - STRIP, bf), (rz - 15, rz + 15)))
        a(P(18, "рельс X", "Relsy", "gantry", (-X_RAIL_L / 2, X_RAIL_L / 2), (bf - STRIP - RAIL_H, bf - STRIP), (rz - RAIL_W / 2, rz + RAIL_W / 2)))
    x_axis_y = bf + BEAM_D / 2
    top = BEAM_Z0 + BEAM_H
    x_axis_z = top + 10 + BKF_CH
    a(P(19.2, "плита BF X", "Uzly", "gantry", (-240, -200), (x_axis_y - 30, x_axis_y + 30), (top, top + 10)))
    a(P(19.1, "плита BK X", "Uzly", "gantry", (195, 276), (x_axis_y - 30, x_axis_y + 30), (top, top + 10)))
    a(P(11, "BF12 X", "Uzly", "gantry", (-240, -220), (x_axis_y - 30, x_axis_y + 30), (top + 10, top + 10 + BKF_H)))
    a(P(10, "BK12 X", "Uzly", "gantry", (215, 240), (x_axis_y - 30, x_axis_y + 30), (top + 10, top + 10 + BKF_H)))
    a(cyl(20, "винт X", "SHVP", "gantry", "x", x_axis_y, x_axis_z, SCREW_R, -250, 270))
    a(cyl(12, "муфта X", "Privod", "gantry", "x", x_axis_y, x_axis_z, 12.5, 250, 280))
    a(P(19.3, "плита мотора X", "Uzly", "gantry", (276, 286), (x_axis_y - 30, x_axis_y + 30), (top - 15, top + 70)))
    a(P(13, "мотор X", "Privod", "gantry", (286, 286 + MOTOR_L), (x_axis_y - MOTOR / 2, x_axis_y + MOTOR / 2), (x_axis_z - MOTOR / 2, x_axis_z + MOTOR / 2)))
    # ---------- Суппорт X ----------
    sr = bf - SADDLE_REAR               # тыл суппорта
    sf = sr - SADDLE_T                  # лицо суппорта
    for s, dx in itertools.product((-1, 1), (-40, 40)):
        rz = bc + s * X_RAIL_DZ
        a(P(21, "каретка X", "Uzly", "saddle", (tx + dx - CAR_L / 2, tx + dx + CAR_L / 2), (sr, sr + CAR_H - 4), (rz - CAR_W / 2, rz + CAR_W / 2)))
    a(P(22, "суппорт", "Golova", "saddle", (tx - SADDLE_W / 2, tx + SADDLE_W / 2), (sf, sr), (SADDLE_Z0, SADDLE_Z1)))
    a(P(16, "кронштейн гайки X", "Uzly", "saddle", (tx - 20, tx + 20), (sr, x_axis_y + 24), (x_axis_z - 20, x_axis_z + 20)))
    a(cyl(20, "гайка X", "SHVP", "saddle", "x", x_axis_y, x_axis_z, NUT_R, tx - NUT_L / 2, tx + NUT_L / 2))
    zaxis_y = sf - BKF_CH
    a(P(10, "BK12 Z", "Uzly", "saddle", (tx - 30, tx + 30), (sf - BKF_H, sf), (BK_Z0, BK_Z0 + BK_L)))
    a(cyl(23, "винт Z", "SHVP", "saddle", "z", tx, zaxis_y, SCREW_R, Z_LOW + 38 + ZPLATE_L - NUT_L, BK_Z0 + BK_L + 15))
    a(cyl(12, "муфта Z", "Privod", "saddle", "z", tx, zaxis_y, 12.5, BK_Z0 + BK_L + 4, BK_Z0 + BK_L + 34))
    a(P(19.4, "плита мотора Z", "Uzly", "saddle", (tx - 35, tx + 35), (sf - 60, sf + 20), (SADDLE_Z1, SADDLE_Z1 + 6)))
    a(P(13, "мотор Z", "Privod", "saddle", (tx - MOTOR / 2, tx + MOTOR / 2), (zaxis_y - MOTOR / 2, zaxis_y + MOTOR / 2), (SADDLE_Z1 + 6, SADDLE_Z1 + 6 + MOTOR_L)))
    zc = Z_CAR_C
    for sx, dz in itertools.product((-1, 1), (-40, 40)):
        a(P(25, "каретка Z", "Uzly", "saddle", (tx + sx * Z_RAIL_DX - CAR_W / 2, tx + sx * Z_RAIL_DX + CAR_W / 2), (sf - CAR_H + 4, sf), (zc + dz - CAR_L / 2, zc + dz + CAR_L / 2)))
    # ---------- Голова Z ----------
    nose = z + STICKOUT
    pz0 = z + 38                        # низ плиты Z
    zr = sf - CAR_H                     # тыл плиты Z
    zf = zr - ZPLATE_T                  # лицо плиты Z
    a(P(26, "плита Z низ", "Golova", "head", (tx - ZPLATE_W / 2, tx + ZPLATE_W / 2), (zf, zr), (pz0, pz0 + WINDOW_Z0)))
    for s in (-1, 1):
        a(P(26, "плита Z щека", "Golova", "head", sorted((tx + s * 26, tx + s * ZPLATE_W / 2)), (zf, zr), (pz0 + WINDOW_Z0, pz0 + ZPLATE_L)))
        a(P(27, "рельс Z", "Relsy", "head", (tx + s * Z_RAIL_DX - RAIL_W / 2, tx + s * Z_RAIL_DX + RAIL_W / 2), (zr, zr + RAIL_H), (pz0 + 10, pz0 + 10 + Z_RAIL_L)))
    nut_top = pz0 + ZPLATE_L
    a(cyl(23, "гайка Z", "SHVP", "head", "z", tx, zaxis_y, NUT_R, nut_top - NUT_L, nut_top))
    a(P(16, "кронштейн гайки Z", "Uzly", "head", (tx - 50, tx + 50), (zr, sf - 2), (nut_top - 40, nut_top - 10)))
    a(cyl(23, "фланец гайки Z", "SHVP", "head", "z", tx, zaxis_y, FLANGE_R, nut_top - FLANGE_T, nut_top))
    clamp_y = zf - CLAMP_SPACER
    axis_y = clamp_y - CLAMP_WALL - SPINDLE_D / 2
    for cz in (pz0 + 10, pz0 + 120):              # 2 хомута по 3 кольца Al 20
        for k in range(3):
            a(P(28, "хомут", "Golova", "head", (tx - 60, tx + 60), (axis_y - SPINDLE_D / 2 - CLAMP_WALL, clamp_y), (cz + 20 * k, cz + 20 * k + 20)))
    a(cyl(29, "шпиндель", "Golova", "head", "z", tx, axis_y, SPINDLE_D / 2, nose, nose + SPINDLE_L))
    a(cyl(29, "гайка цанги", "Golova", "head", "z", tx, axis_y, 12, nose - 13, nose))
    a(cyl(29, "инструмент", "Golova", "head", "z", tx, axis_y, 2, z, nose - 13))
    if fasteners:
        bc_y = bf + BEAM_D                       # тыл балки = перед стойки
        for s in (-1, 1):
            x0 = s * 220
            for zz in (BEAM_Z0 + 25, BEAM_Z0 + 75):                       # K1: стойка -> балка
                a(cyl(41, "винт M8x100", "Krepezh", "gantry", "y", x0, zz, 4, bc_y - 20, bc_y + UPRIGHT_D, True))
                a(cyl(41, "головка M8", "Krepezh", "gantry", "y", x0, zz, 6.5, bc_y + UPRIGHT_D, bc_y + UPRIGHT_D + 8))
                a(cyl(41, "шайба DIN 9021", "Krepezh", "gantry", "y", x0, zz, 12, bc_y - BEAM_WALL - 2, bc_y - BEAM_WALL, True))
                a(cyl(41, "гайка DIN 985", "Krepezh", "gantry", "y", x0, zz, 6.5, bc_y - BEAM_WALL - 10, bc_y - BEAM_WALL - 2, True))
            for yy in (bc_y + 20, bc_y + 60):                              # K2: щёки -> стойка
                for zz in (foot_z + GANTRY_PLATE_T + 30, foot_z + GANTRY_PLATE_T + 75):
                    a(cyl(42, "винт M8x65", "Krepezh", "gantry", "x", yy, zz, 4, s * 188, s * 252, True))
                    a(cyl(42, "головка M8", "Krepezh", "gantry", "x", yy, zz, 6.5, s * 186, s * 194))
            a(cyl(43, "винт M8x20 DIN 7984", "Krepezh", "gantry", "z", x0, bc_y + 20, 4, foot_z + 4.5, foot_z + 24.5, True))
    return parts


# Намеренные контакты между группами (винт в гайке, каретка на рельсе, опора на винте и т. п.)
ALLOWED = [
    ("винт Y", "гайка Y"), ("винт Y", "кронштейн гайки Y"), ("винт Y", "фланец гайки Y"),
    ("рельс Y", "каретка Y"), ("рельс X", "каретка X"), ("рельс Z", "каретка Z"),
    ("винт X", "гайка X"), ("винт X", "кронштейн гайки X"),
    ("винт Z", "гайка Z"), ("винт Z", "кронштейн гайки Z"), ("винт Z", "фланец гайки Z"),
]


def overlap(a, b, eps=1e-6):
    return all(min(getattr(a, k)[1], getattr(b, k)[1]) - max(getattr(a, k)[0], getattr(b, k)[0]) > eps for k in "xyz")


def check_collisions():
    """Перебор всех комбинаций крайних положений X/Y/Z (рабочий ход ±запас)."""
    problems = set()
    positions = itertools.product(
        (-WORK_X / 2 - 12, WORK_X / 2 + 12),
        (TABLE_Y0 - 5, TABLE_Y0 + WORK_Y + 5),
        (0, WORK_Z),
    )
    for tx, ty, tz in positions:
        parts = build(tx, ty, tz)
        for a, b in itertools.combinations(parts, 2):
            if a.group == b.group or not overlap(a, b):
                continue
            pair = (a.name, b.name)
            if pair in ALLOWED or pair[::-1] in ALLOWED:
                continue
            problems.add((a.name, b.name, tx, ty, tz))
    if problems:
        for p in sorted(problems, key=str):
            print("СТОЛКНОВЕНИЕ:", p)
        raise SystemExit(f"Найдено столкновений: {len(problems)} — чертёж не создан")
    # Рельс должен оставаться под каретками
    rail_z = (Z_LOW + 38 + 10, Z_LOW + 38 + 10 + Z_RAIL_L)
    assert rail_z[0] + WORK_Z <= Z_CAR_C - 40 - CAR_L / 2 and rail_z[1] >= Z_CAR_C + 40 + CAR_L / 2, "Каретка Z сходит с рельса"
    assert WORK_X / 2 + 12 + 40 + CAR_L / 2 <= X_RAIL_L / 2, "Каретка X сходит с рельса"
    x_stroke = X_RAIL_L - (80 + CAR_L)
    y_stroke = Y_RAIL_L - (100 + CAR_L)
    for ty in (TABLE_Y0 - 5, TABLE_Y0 + WORK_Y + 5):
        fc = ty + TOOL_OFFSET
        assert fc - 50 - CAR_L / 2 >= Y_RAIL_Y0 and fc + 50 + CAR_L / 2 <= Y_RAIL_Y0 + Y_RAIL_L, "Каретка Y сходит с рельса"
    return x_stroke, y_stroke, Z_RAIL_L - (80 + CAR_L)


# ---------------------- Построение видов ----------------------
SCALE = 2.5                         # 1:2,5 на листе А1
SHEET_W, SHEET_H = 841 * SCALE, 594 * SCALE
TH = 3.5 * SCALE                    # основной шрифт 3,5 мм на бумаге

FRONT = (420.0, 775.0)
TOP = (420.0, 75.0)
SIDE = (1540.0, 775.0)              # X листа = SIDE[0] - y

VIEWS = {
    "front": (lambda p: (p.x, p.z), lambda p: p.y[0], lambda u, v: (FRONT[0] + u, FRONT[1] + v), "y"),
    "top": (lambda p: (p.x, p.y), lambda p: -p.z[1], lambda u, v: (TOP[0] + u, TOP[1] + v), "z"),
    "side": (lambda p: (tuple(sorted((-p.y[0], -p.y[1]))), p.z), lambda p: p.x[0], lambda u, v: (SIDE[0] + u, SIDE[1] + v), "x"),
}


def sub_intervals(iv, cuts):
    res = [iv]
    for c0, c1 in cuts:
        nxt = []
        for a, b in res:
            if c1 <= a or c0 >= b:
                nxt.append((a, b))
            else:
                if c0 > a:
                    nxt.append((a, c0))
                if c1 < b:
                    nxt.append((c1, b))
        res = nxt
    return [(a, b) for a, b in res if b - a > 0.05]


class Occl:
    def __init__(self, u, v, depth, circle):
        self.u, self.v, self.depth, self.circle = u, v, depth, circle

    def chord_h(self, v):
        """Интервал по u, перекрытый на горизонтали v."""
        if self.circle:
            cu, cv = sum(self.u) / 2, sum(self.v) / 2
            r = (self.u[1] - self.u[0]) / 2
            d = abs(v - cv)
            if d >= r:
                return None
            w = math.sqrt(r * r - d * d)
            return cu - w, cu + w
        return self.u if self.v[0] < v < self.v[1] else None

    def chord_v(self, u):
        if self.circle:
            cu, cv = sum(self.u) / 2, sum(self.v) / 2
            r = (self.u[1] - self.u[0]) / 2
            d = abs(u - cu)
            if d >= r:
                return None
            w = math.sqrt(r * r - d * d)
            return cv - w, cv + w
        return self.v if self.u[0] < u < self.u[1] else None

    def contains(self, u, v):
        c = self.chord_h(v)
        return c is not None and c[0] < u < c[1]


def clip_segment(p0, p1, box):
    """Отсечение отрезка прямоугольником (u0, u1, v0, v1), алгоритм Лианга-Барски."""
    (x0, y0), (x1, y1) = p0, p1
    dx, dy = x1 - x0, y1 - y0
    t0, t1 = 0.0, 1.0
    for p, q in ((-dx, x0 - box[0]), (dx, box[1] - x0), (-dy, y0 - box[2]), (dy, box[3] - y0)):
        if abs(p) < 1e-12:
            if q < 0:
                return None
            continue
        t = q / p
        if p < 0:
            t0 = max(t0, t)
        else:
            t1 = min(t1, t)
        if t0 > t1:
            return None
    return (x0 + t0 * dx, y0 + t0 * dy), (x0 + t1 * dx, y0 + t1 * dy)


def draw_view(msp, parts, view, views=None, clip=None):
    """clip — окно (u0, u1, v0, v1) в координатах модели; детали с атрибутом skip только заслоняют."""
    uv, depth_of, to_sheet, axial = (views or VIEWS)[view]
    occ = []
    for p in parts:
        u, v = uv(p)
        occ.append(Occl(u, v, depth_of(p), p.axis == axial))

    def emit(p0, p1, layer):
        if clip:
            seg = clip_segment(p0, p1, clip)
            if seg is None or math.dist(*seg) < 1e-3:
                return
            p0, p1 = seg
        msp.add_line(to_sheet(*p0), to_sheet(*p1), dxfattribs={"layer": layer})

    for p, me in zip(parts, occ):
        if getattr(p, "skip", False):
            continue
        front_of = [o for o in occ if o.depth < me.depth - 1e-6]
        u0, u1, v0, v1 = me.u[0], me.u[1], me.v[0], me.v[1]
        if me.circle:
            cu, cv, r = (u0 + u1) / 2, (v0 + v1) / 2, (u1 - u0) / 2
            n = 72
            pts = [(cu + r * math.cos(2 * math.pi * i / n), cv + r * math.sin(2 * math.pi * i / n)) for i in range(n + 1)]
            for a, b in zip(pts, pts[1:]):
                mu, mv = (a[0] + b[0]) / 2, (a[1] + b[1]) / 2
                hid = any(o.contains(mu, mv) for o in front_of)
                if not hid:
                    emit(a, b, p.layer)
                elif p.hidden:
                    emit(a, b, "Nevidimye")
            continue
        for v in (v0, v1):
            cuts = [c for c in (o.chord_h(v) for o in front_of) if c]
            vis = sub_intervals((u0, u1), cuts)
            for a, b in vis:
                emit((a, v), (b, v), p.layer)
            if p.hidden:
                pos = u0
                for a, b in vis + [(u1, u1)]:
                    if a - pos > 0.05:
                        emit((pos, v), (a, v), "Nevidimye")
                    pos = b
        for u in (u0, u1):
            cuts = [c for c in (o.chord_v(u) for o in front_of) if c]
            vis = sub_intervals((v0, v1), cuts)
            for a, b in vis:
                emit((u, a), (u, b), p.layer)
            if p.hidden:
                pos = v0
                for a, b in vis + [(v1, v1)]:
                    if a - pos > 0.05:
                        emit((u, pos), (u, a), "Nevidimye")
                    pos = b


def setup_doc():
    doc = ezdxf.new("R2013", setup=True)
    doc.header["$INSUNITS"] = 4
    doc.header["$LTSCALE"] = 2.5
    doc.styles.add("CHERT", font="arial.ttf")
    for name, color, lt in [
        ("Rama", 7, "CONTINUOUS"), ("Stol", 34, "CONTINUOUS"), ("Relsy", 5, "CONTINUOUS"),
        ("SHVP", 1, "CONTINUOUS"), ("Golova", 212, "CONTINUOUS"), ("Uzly", 94, "CONTINUOUS"),
        ("Privod", 134, "CONTINUOUS"), ("Nevidimye", 8, "DASHED"), ("Osevye", 8, "CENTER"),
        ("Hoda", 40, "PHANTOM"), ("Razmery", 8, "CONTINUOUS"), ("Tekst", 7, "CONTINUOUS"),
        ("Ramka", 7, "CONTINUOUS"), ("Krepezh", 6, "CONTINUOUS"), ("Tonkie", 8, "CONTINUOUS"),
        ("Shtrihovka", 8, "CONTINUOUS"),
    ]:
        doc.layers.add(name, color=color, linetype=lt)
    ds = doc.dimstyles.new("CHERT")
    ds.dxf.dimtxt = TH * 0.9
    ds.dxf.dimasz = TH * 0.8
    ds.dxf.dimexo = 2
    ds.dxf.dimexe = 4
    ds.dxf.dimgap = 2.5
    ds.dxf.dimtad = 1
    ds.dxf.dimtxsty = "CHERT"
    ds.dxf.dimdec = 0
    ds.dxf.dimlfac = 1
    apply_dim_defaults(doc, dimtxt=TH * 0.9, dimasz=TH * 0.8, dimexo=2, dimexe=4, dimgap=2.5)
    return doc


def apply_dim_defaults(doc, **v):
    """LibreCAD строит размеры по переменным заголовка ($DIMTXT и т. п.), а не по стилю.
    Пишем одинаковые значения в заголовок, в стиль Standard и во все свои стили."""
    v.setdefault("dimtad", 1)
    for k, val in v.items():
        doc.header["$" + k.upper()] = val
        for ds in doc.dimstyles:
            ds.dxf.set(k, val)


TRANSLIT = dict(zip("абвгдезиклмнопрстуфхцыэ", "abvgdeziklmnoprstufhcye"))
TRANSLIT.update({"ё": "e", "ж": "zh", "й": "j", "ч": "ch", "ш": "sh", "щ": "sch", "ъ": "", "ь": "", "ю": "yu", "я": "ya"})
SYMBOLS = {"×": "x", "≈": "~", "≤": "<=", "≥": ">=", "±": "+-", "·": "*", "–": "-", "—": "-"}


def lat(s):
    """Все надписи чертежа выводятся латиницей и только в ASCII."""
    out = []
    for i, ch in enumerate(s):
        low = ch.lower()
        if low in TRANSLIT:
            t = TRANSLIT[low]
            if ch != low:
                next_lower = i + 1 < len(s) and s[i + 1].islower()
                t = t.capitalize() if next_lower else t.upper()
            out.append(t)
        else:
            out.append(SYMBOLS.get(ch, ch))
    return "".join(out)


def text(msp, s, x, y, h=TH, layer="Tekst", align="LEFT", color=None):
    attrs = {"layer": layer, "height": h, "style": "CHERT"}
    if color is not None:
        attrs["color"] = color
    t = msp.add_text(lat(s), dxfattribs=attrs)
    t.set_placement((x, y), align=ezdxf.enums.TextEntityAlignment[align])
    return t


def dim_h(msp, x0, x1, y, base_y, label=None):
    d = msp.add_linear_dim(base=(x0, base_y), p1=(x0, y), p2=(x1, y), dimstyle="CHERT",
                           text=lat(label) if label else "<>", dxfattribs={"layer": "Razmery"})
    d.render()


def dim_v(msp, y0, y1, x, base_x, label=None):
    d = msp.add_linear_dim(base=(base_x, y0), p1=(x, y0), p2=(x, y1), angle=90, dimstyle="CHERT",
                           text=lat(label) if label else "<>", dxfattribs={"layer": "Razmery"})
    d.render()


def balloon(msp, n, tx, ty, bx, by):
    """Выноска с номером позиции."""
    r = 13
    d = math.hypot(tx - bx, ty - by)
    ex, ey = bx + (tx - bx) * r / d, by + (ty - by) * r / d
    msp.add_line((ex, ey), (tx, ty), dxfattribs={"layer": "Tekst"})
    msp.add_circle((tx, ty), 1.5, dxfattribs={"layer": "Tekst"})
    msp.add_circle((bx, by), r, dxfattribs={"layer": "Tekst"})
    text(msp, str(n), bx, by, TH, align="MIDDLE_CENTER")


BOM = [
    (1, "Лонжерон базы, проф. 40×80 паз 8", f"L={BASE_L}", 2, "6060-T6"),
    (2, "Поперечина базы, проф. 40×80", "L=400", 2, "задняя с отв."),
    (3, "Балка стола, проф. 40×80", f"L={BASE_L - 2 * CROSS_D}", 2, ""),
    (4, "Стол: Al 10 + МДФ 19 + жертв. МДФ 8", "300×200", 1, "M6 шаг 50"),
    (5, "Полоса посадочная под рельс", f"30×{STRIP:g}", 4, "полоса 6, шлифовать"),
    (6, "Рельс HGR15, ось Y", f"L={Y_RAIL_L}", 2, ""),
    (7, "Каретка HGH15CA, ось Y", "", 4, "шаг 100"),
    (8, "Плита опорная gantry", f"91×160×{GANTRY_PLATE_T}", 2, "сталь"),
    (9, "ШВП DFU1605 C7, ось Y", "L≈395", 2, "преднатяг"),
    (10, "Опора BK12 (X, Y×2, Z)", "", 4, ""),
    (11, "Опора BF12 (X, Y×2)", "", 3, ""),
    (12, "Муфта лучевая 6,35×10", "по валу", 4, ""),
    (13, "Мотор NEMA 23, 76 мм (X, Y×2, Z)", "1,9 Н·м", 4, "3 А"),
    (14, "Стойка, проф. 40×80", f"L={BEAM_Z0 + BEAM_H - (BASE_TOP + STRIP + CAR_H + GANTRY_PLATE_T):g}", 2, ""),
    (15, "Щека-косынка", "t=6", 4, "сталь"),
    (16, "Кронштейн гайки (X, Y×2, Z)", "", 4, "16.1 сталь сварн.; 16.2–16.3 Al"),
    (17, "Балка X, труба 100×50×4", f"L={BEAM_LEN}", 1, "сталь Ст3"),
    (18, "Рельс HGR15, ось X", f"L={X_RAIL_L}", 2, ""),
    (19, "Плиты BK/BF X и моторов X, Z", "", 4, "19.1-19.4"),
    (20, "ШВП DFU1605 C7, ось X", "L≈520", 1, "преднатяг"),
    (21, "Каретка HGH15CA, ось X", "", 4, "шаг 80"),
    (22, "Суппорт X", f"{SADDLE_W}×{SADDLE_Z1 - SADDLE_Z0}×{SADDLE_T}", 1, "Al"),
    (23, "ШВП DFU1605 C7, ось Z", "L≈205", 1, "без BF12"),
    (24, "Опора регулируемая", "диам. 30", 4, ""),
    (25, "Каретка HGH15CA, ось Z", "", 4, "шаг 80"),
    (26, "Плита Z с окном под гайку", f"{ZPLATE_W}×{ZPLATE_L}×{ZPLATE_T}", 1, "Al"),
    (27, "Рельс HGR15, ось Z", f"L={Z_RAIL_L}", 2, ""),
    (28, "Кольцо хомута шпинделя диам. 80", "120×110×20", 6, "Al 20"),
    (29, "Шпиндель 1,5 кВт, ER11, воздух", "диам. 80", 1, "s VFD 220 V"),
    (30, "Плита угловая передняя", "130×80×6", 2, "сталь"),
    (31, "Плита угловая задняя, моторная", "130×80×6", 2, "сталь"),
    (32, "Плита под BK12/BF12 оси Y", "64×80×6", 4, "сталь, 2+2 зерк."),
]

NOTES = [
    "ТЕХНИЧЕСКИЕ УКАЗАНИЯ",
    "1. Компоновка, спец. v2.2. Базу со столом и портал - см. отд. чертежи.",
    "2. Положение на чертеже: инструмент в центре поля, Z внизу.",
    "3. Рабочее поле 300×200, ход Z 60 (механич. предел 78).",
    "4. Параллельность рельсов ≤0,03 мм на длину: только по",
    "    посадочным полосам, на необработанный профиль не ставить.",
    "5. Перпендикулярность шпинделя столу ≤0,05 мм/100 мм.",
    "6. Y — два винта и два мотора с выверкой по двум датчикам.",
    "7. Длины рельсов и винтов — под заказ, в размер.",
    "8. Размеры покупных узлов сверить до изготовления кронштейнов:",
    "    шпиндель, гайка DFU1605, каретка HGH15CA.",
]


def draw_bom_notes(msp):
    x0, ytop = 1595.0, 1455.0
    cols = [0, 32, 268, 350, 380, 480]
    rh = 19.0
    heads = ["Поз.", "Наименование", "Размер", "Кол.", "Примеч."]
    rows = len(BOM) + 1
    for i in range(rows + 1):
        msp.add_line((x0, ytop - i * rh), (x0 + cols[-1], ytop - i * rh), dxfattribs={"layer": "Ramka"})
    for c in cols:
        msp.add_line((x0 + c, ytop), (x0 + c, ytop - rows * rh), dxfattribs={"layer": "Ramka"})
    for c, h in zip(cols, heads):
        text(msp, h, x0 + c + 3, ytop - rh * 0.72, TH * 0.72)
    for i, (n, name, size, qty, rem) in enumerate(BOM, start=1):
        y = ytop - i * rh - rh * 0.72
        for c, s in zip(cols, (str(n), name, size, str(qty), rem)):
            text(msp, s, x0 + c + 3, y, TH * 0.72)
    y = ytop - rows * rh - 26
    for line in NOTES:
        if line:
            text(msp, line, x0, y, TH * 0.72)
        y -= 17


def draw_frame(msp):
    x0, y0, x1, y1 = 20 * SCALE, 5 * SCALE, SHEET_W - 5 * SCALE, SHEET_H - 5 * SCALE
    for a, b in ((x0, y0), (x1, y0)), ((x1, y0), (x1, y1)), ((x1, y1), (x0, y1)), ((x0, y1), (x0, y0)):
        msp.add_line(a, b, dxfattribs={"layer": "Ramka"})
    # Основная надпись (упрощённая) в правом нижнем углу
    w, h = 185 * SCALE, 40 * SCALE
    bx, by = x1 - w, y0
    for a, b in [((bx, by), (bx, by + h)), ((bx, by + h), (x1, by + h)),
                 ((bx, by + h / 2), (x1, by + h / 2)), ((x1 - 60 * SCALE, by), (x1 - 60 * SCALE, by + h / 2)),
                 ((x1 - 30 * SCALE, by), (x1 - 30 * SCALE, by + h / 2))]:
        msp.add_line(a, b, dxfattribs={"layer": "Ramka"})
    text(msp, "Станок фрезерный настольный, 3 оси", bx + 12, by + h * 0.68, TH * 1.25)
    text(msp, "Компоновка. Поле 300×200, Z 60. Спец. v2.2", bx + 12, by + h * 0.22, TH)
    text(msp, "Масштаб", x1 - 56 * SCALE, by + h * 0.3, TH * 0.75)
    text(msp, "1:2,5", x1 - 56 * SCALE, by + h * 0.12, TH)
    text(msp, "Лист А1", x1 - 26 * SCALE, by + h * 0.3, TH * 0.75)
    text(msp, "v2", x1 - 26 * SCALE, by + h * 0.12, TH)
    text(msp, "Размеры в мм", x1 - 56 * SCALE, by + h * 0.7, TH * 0.8)


def main():
    strokes = check_collisions()
    parts = build()
    doc = setup_doc()
    msp = doc.modelspace()
    for v in VIEWS:
        draw_view(msp, parts, v)

    fx, fz = FRONT
    tx, ty = TOP
    sx, sz = SIDE

    # Заголовки видов
    text(msp, "ВИД СПЕРЕДИ", fx + 50, fz + 675, TH * 1.25, align="CENTER")
    text(msp, "ВИД СВЕРХУ", tx - 320, ty + 672, TH * 1.25, align="CENTER")
    text(msp, "ВИД СЛЕВА", sx - 300, sz + 675, TH * 1.25, align="CENTER")

    # Осевые: инструмент и середина базы
    tool_y = TABLE_Y0 + WORK_Y / 2
    msp.add_line((fx, fz - 15), (fx, fz + 640), dxfattribs={"layer": "Osevye"})
    msp.add_line((tx, ty - 15), (tx, ty + BASE_L + 75), dxfattribs={"layer": "Osevye"})
    msp.add_line((tx - 260, ty + tool_y), (tx + 260, ty + tool_y), dxfattribs={"layer": "Osevye"})
    msp.add_line((sx - tool_y, sz + 110), (sx - tool_y, sz + 640), dxfattribs={"layer": "Osevye"})

    # Рабочее поле и ход инструмента
    x0, x1 = tx - WORK_X / 2, tx + WORK_X / 2
    y0, y1 = ty + TABLE_Y0, ty + TABLE_Y0 + WORK_Y
    for a, b in ((x0, y0), (x1, y0)), ((x1, y0), (x1, y1)), ((x1, y1), (x0, y1)), ((x0, y1), (x0, y0)):
        msp.add_line(a, b, dxfattribs={"layer": "Hoda"})
    # Сетка M6 (видимая часть стола)
    for i in range(6):
        for j in range(4):
            hx, hy = -125 + 50 * i, TABLE_Y0 + 25 + 50 * j
            if abs(hx) > 100 or hy < tool_y - 55:  # остальные скрыты головой
                msp.add_circle((tx + hx, ty + hy), 3.2, dxfattribs={"layer": "Stol"})

    # --- Размеры: вид сверху ---
    overall_x1 = 286 + MOTOR_L
    overall_y1 = MOTOR_Y0 + MOTOR_L
    dim_h(msp, tx - 240, tx + 240, ty - PLATE_T, ty - 28)
    dim_h(msp, tx - WORK_X / 2, tx + WORK_X / 2, ty + TABLE_Y0, ty - 56, "300 (поле X)")
    dim_v(msp, ty, ty + BASE_L, tx + 240, tx + 385)
    dim_v(msp, ty - PLATE_T, ty + overall_y1, tx + 240, tx + 415)
    dim_v(msp, ty + TABLE_Y0, ty + TABLE_Y0 + WORK_Y, tx - 150, tx - 272, "200 (поле Y)")
    dim_v(msp, ty + Y_RAIL_Y0, ty + Y_RAIL_Y0 + Y_RAIL_L, tx - 240, tx - 305)

    # --- Размеры: вид спереди ---
    dim_h(msp, fx - BEAM_LEN / 2, fx + BEAM_LEN / 2, fz + BEAM_Z0 + BEAM_H, fz + 645, f"балка {BEAM_LEN}, рельс {X_RAIL_L}")
    dim_h(msp, fx - 246, fx + overall_x1, fz, fz - 40)
    dim_h(msp, fx - RAIL_X, fx + RAIL_X, fz, fz - 16)
    dim_v(msp, fz, fz + SADDLE_Z1 + 6 + MOTOR_L, fx - 246, fx - 300)
    dim_v(msp, fz, fz + TABLE_TOP, fx - 246, fx - 268)
    dim_v(msp, fz + BEAM_Z0, fz + BEAM_Z0 + BEAM_H, fx + 240, fx + 425)
    # Ход Z
    dim_v(msp, fz + TABLE_TOP, fz + TABLE_TOP + WORK_Z, fx, fx + 150, "ход Z 60")

    # --- Размеры: вид слева ---
    dim_h(msp, sx - BASE_L, sx, sz, sz - 34)
    dim_h(msp, sx - overall_y1, sx + PLATE_T, sz, sz - 66)
    fc = tool_y + TOOL_OFFSET
    dim_h(msp, sx - fc, sx - tool_y, sz + 320, sz + 645, f"вылет {TOOL_OFFSET:.0f}")
    dim_v(msp, sz + BASE_TOP, sz + TABLE_TOP, sx, sx + 30)

    # --- Позиции ---
    for n, ux, uz, bx, bz in [
        (17, 215, 320, 330, 235), (18, 215, 352, 370, 325), (22, -70, 470, -190, 540),
        (29, 0, 265, -125, 275), (26, -68, 220, -190, 215), (4, 90, 120, 90, 62),
        (13, 330, 405, 420, 500), (10, 228, 410, 300, 560), (20, 130, 405, 190, 490),
        (2, -60, 50, -60, -62), (24, -220, 7, -290, -55), (30, -175, 30, -175, -62),
    ]:
        balloon(msp, n, fx + ux, fz + uz, fx + bx, fz + bz)
    for n, ux, uy, bx, by in [
        (1, -220, 40, -345, 40), (6, 220, 520, 330, 520), (3, 130, 420, 300, 450),
        (8, -175, 395, -345, 395), (13, -175, 600, -290, 560), (14, 220, 340, 330, 240),
    ]:
        balloon(msp, n, tx + ux, ty + uy, tx + bx, ty + by)
    for n, uy, uz, bu, bz in [
        (15, 400, 190, -480, 260), (9, 470, 47, -500, 175), (28, 118, 300, -30, 410),
        (23, 198, 460, -60, 560), (11, 160, 47, 40, 60), (12, 540, 47, -620, 130),
        (16, 300, 80, -215, 215),
    ]:
        balloon(msp, n, sx - uy, sz + uz, sx + bu, sz + bz)

    draw_bom_notes(msp)
    draw_frame(msp)

    auditor = doc.audit()
    if auditor.has_errors:
        raise SystemExit(f"Ошибки DXF: {auditor.errors}")
    doc.saveas(OUT.with_suffix(".dxf"), encoding="utf-8")

    # PDF: лист А1 в масштабе 1:2,5
    fig = plt.figure(figsize=(841 / 25.4, 594 / 25.4))
    ax = fig.add_axes([0, 0, 1, 1])
    ax.set_xlim(0, SHEET_W)
    ax.set_ylim(0, SHEET_H)
    cfg = Configuration(background_policy=BackgroundPolicy.WHITE, color_policy=ColorPolicy.COLOR,
                        min_lineweight=0.18, lineweight_scaling=1.0)
    Frontend(RenderContext(doc), MatplotlibBackend(ax, adjust_figure=False), config=cfg).draw_layout(msp)
    ax.set_xlim(0, SHEET_W)
    ax.set_ylim(0, SHEET_H)
    fig.savefig(OUT.with_suffix(".pdf"))
    print(f"Столкновений нет. Запас хода рельсов: X {strokes[0]:.1f}, Y {strokes[1]:.1f}, Z {strokes[2]:.1f} мм")
    print(f"Вылет инструмента от центра плиты gantry: {TOOL_OFFSET:.1f} мм")
    print(f"Габарит: X {246 + overall_x1}, Y {overall_y1 + PLATE_T}, Z {SADDLE_Z1 + 6 + MOTOR_L}")


if __name__ == "__main__":
    main()
