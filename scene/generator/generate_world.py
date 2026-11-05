#!/usr/bin/env python3
"""Генератор полигона «Виртуальный курьер».

Читает YAML-сценарий, строит лабиринт склада и пишет:
  * <out_dir>/<name>.wbt  — мир Webots;
  * <out_dir>/<name>.json — метаданные для судьи (стены, зоны, станции).

Использование:
    generate_world.py scenario.yaml --out scene/webots/worlds
"""

import argparse
import json
import math
import os
import random

import yaml

WALL_T = 0.05      # толщина стены, м
WALL_H = 0.30      # высота стены, м
SIDES = {'N': (0, 1), 'S': (0, -1), 'E': (1, 0), 'W': (-1, 0)}
STATION_COLORS = {'A': (0.95, 0.75, 0.1), 'B': (0.1, 0.7, 0.9), 'C': (0.85, 0.3, 0.85)}
STATION_MARKERS = {'A': 1, 'B': 2, 'C': 3}
FINISH_MARKER = 0


# ---------------------------------------------------------------- геометрия

def quat_mul(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw)


def quat_axis(axis, angle):
    s = math.sin(angle / 2)
    return (math.cos(angle / 2), axis[0] * s, axis[1] * s, axis[2] * s)


def quat_to_axis_angle(q):
    w, x, y, z = q
    w = max(-1.0, min(1.0, w))
    angle = 2 * math.acos(w)
    s = math.sqrt(max(1e-12, 1 - w * w))
    if s < 1e-6:
        return (0, 0, 1, 0)
    return (x / s, y / s, z / s, angle)


def fmt(*v):
    return ' '.join(f'{x:.4f}' for x in v)


# ---------------------------------------------------------------- лабиринт

def build_maze(cols, rows, rng, loops):
    """Возвращает множество внутренних стен.

    Стена задаётся как ('v', i, j) — вертикальная между ячейками (i-1, j) и (i, j),
    или ('h', i, j) — горизонтальная между (i, j-1) и (i, j).
    """
    walls = set()
    for i in range(1, cols):
        for j in range(rows):
            walls.add(('v', i, j))
    for i in range(cols):
        for j in range(1, rows):
            walls.add(('h', i, j))

    # Рекурсивный backtracker (итеративно) — идеальный лабиринт.
    visited = {(0, 0)}
    stack = [(0, 0)]
    while stack:
        ci, cj = stack[-1]
        nbrs = []
        for di, dj in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            ni, nj = ci + di, cj + dj
            if 0 <= ni < cols and 0 <= nj < rows and (ni, nj) not in visited:
                nbrs.append((ni, nj, di, dj))
        if not nbrs:
            stack.pop()
            continue
        ni, nj, di, dj = rng.choice(nbrs)
        walls.discard(wall_between(ci, cj, ni, nj))
        visited.add((ni, nj))
        stack.append((ni, nj))

    # Дополнительные проходы — несколько маршрутов, а не одно дерево.
    inner = sorted(walls)
    rng.shuffle(inner)
    for w in inner[:int(len(inner) * loops)]:
        walls.discard(w)
    return walls


def wall_between(ai, aj, bi, bj):
    if ai != bi:
        return ('v', max(ai, bi), aj)
    return ('h', ai, max(aj, bj))


def merge_segments(walls, cell):
    """Склеивает соседние стены в длинные отрезки (меньше узлов в мире)."""
    segs = []
    vert = sorted((i, j) for t, i, j in walls if t == 'v')
    hor = sorted((j, i) for t, i, j in walls if t == 'h')
    for kind, items in (('v', vert), ('h', hor)):
        run = None
        for a, b in items:
            if run and run[0] == a and run[2] == b:
                run[2] = b + 1
            else:
                if run:
                    segs.append((kind, *run))
                run = [a, b, b + 1]
        if run:
            segs.append((kind, *run))
    out = []
    for kind, line, start, end in segs:
        if kind == 'v':   # x = line*cell, y от start до end
            out.append(((line * cell, start * cell), (line * cell, end * cell)))
        else:              # y = line*cell, x от start до end
            out.append(((start * cell, line * cell), (end * cell, line * cell)))
    return out


# ---------------------------------------------------------------- узлы Webots

def wall_node(name, p0, p1, color=(0.75, 0.72, 0.66)):
    (x0, y0), (x1, y1) = p0, p1
    length = math.hypot(x1 - x0, y1 - y0) + WALL_T
    cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
    yaw = math.atan2(y1 - y0, x1 - x0)
    return f'''Solid {{
  translation {fmt(cx, cy, WALL_H / 2)}
  rotation 0 0 1 {yaw:.4f}
  children [ Shape {{ appearance PBRAppearance {{ baseColor {fmt(*color)} roughness 0.8 metalness 0 }} geometry Box {{ size {fmt(length, WALL_T, WALL_H)} }} }} ]
  name "{name}"
  boundingObject Box {{ size {fmt(length, WALL_T, WALL_H)} }}
}}
'''


def box_node(name, cx, cy, sx, sy, sz, yaw, color):
    return f'''Solid {{
  translation {fmt(cx, cy, sz / 2)}
  rotation 0 0 1 {yaw:.4f}
  children [ Shape {{ appearance PBRAppearance {{ baseColor {fmt(*color)} roughness 0.7 metalness 0 }} geometry Box {{ size {fmt(sx, sy, sz)} }} }} ]
  name "{name}"
  boundingObject Box {{ size {fmt(sx, sy, sz)} }}
}}
'''


def floor_mark(cx, cy, sx, sy, color, z=0.0005):
    return f'''Pose {{
  translation {fmt(cx, cy, z)}
  children [ Shape {{ appearance PBRAppearance {{ baseColor {fmt(*color)} roughness 1 metalness 0 }} geometry Plane {{ size {fmt(sx, sy)} }} }} ]
}}
'''


def wall_sign(cx, cy, side, marker_id, z=0.2, size=0.24):
    """ArUco-маркер на стене, повёрнутый лицом внутрь полигона."""
    dx, dy = SIDES[side]
    # Plane лежит в XY с нормалью +z. Циклическая перестановка осей (x→y, y→z, z→x)
    # ставит его вертикально нормалью вдоль +x, затем поворачиваем нормалью внутрь полигона.
    normal_yaw = math.atan2(-dy, -dx)
    s3 = 1 / math.sqrt(3)
    q = quat_mul(quat_axis((0, 0, 1), normal_yaw), quat_axis((s3, s3, s3), 2 * math.pi / 3))
    ax, ay, az, ang = quat_to_axis_angle(q)
    off = WALL_T / 2 + 0.002
    return f'''Pose {{
  translation {fmt(cx - dx * off, cy - dy * off, z)}
  rotation {fmt(ax, ay, az, ang)}
  children [ Shape {{ appearance PBRAppearance {{ baseColorMap ImageTexture {{ url [ "textures/aruco_{marker_id}.png" ] filtering 0 }} roughness 1 metalness 0 }} geometry Plane {{ size {fmt(size, size)} }} }} ]
}}
'''


# ---------------------------------------------------------------- сборка мира

def cell_center(cell, size):
    return ((cell[0] + 0.5) * size, (cell[1] + 0.5) * size)


def side_point(cell, side, size):
    cx, cy = cell_center(cell, size)
    dx, dy = SIDES[side]
    return cx + dx * size / 2, cy + dy * size / 2


def check_boundary(cell, side, cols, rows, what):
    i, j = cell
    ok = {'W': i == 0, 'E': i == cols - 1, 'S': j == 0, 'N': j == rows - 1}[side]
    if not ok:
        raise ValueError(f'{what}: сторона {side} ячейки {cell} не на границе полигона')


def generate(sc, extern=False):
    rng = random.Random(sc.get('seed', 0))
    size = float(sc.get('cell_size', 2.0))
    cols, rows = sc.get('grid', [8, 8])
    W, H = cols * size, rows * size
    start = sc['start']
    stations = sc['stations']

    check_boundary(start['cell'], start['side'], cols, rows, 'start')
    if float(start.get('yaw_deg', 0)) % 90 != 0:
        raise ValueError('start.yaw_deg должен быть кратен 90° (робот смотрит вдоль оси сетки)')
    for st in stations:
        check_boundary(st['cell'], st['side'], cols, rows, f"station {st['id']}")

    walls = build_maze(cols, rows, rng, float(sc.get('loops', 0.15)))
    segments = merge_segments(walls, size)
    boundary = [((0, 0), (W, 0)), ((W, 0), (W, H)), ((W, H), (0, H)), ((0, H), (0, 0))]

    # Препятствия-паллеты внутри ячеек (не в старте и не на станциях).
    reserved = {tuple(start['cell'])} | {tuple(s['cell']) for s in stations}
    free = [(i, j) for i in range(cols) for j in range(rows) if (i, j) not in reserved]
    rng.shuffle(free)
    obstacles = []
    for (i, j) in free[:int(sc.get('obstacles', 10))]:
        cx, cy = cell_center((i, j), size)
        sx, sy = rng.uniform(0.3, 0.5), rng.uniform(0.3, 0.5)
        lim = size / 2 - 0.75   # гарантированный проход ≥ ~0.45 м с каждой стороны
        cx += rng.uniform(-lim, lim)
        cy += rng.uniform(-lim, lim)
        obstacles.append({'center': [cx, cy], 'size': [sx, sy], 'yaw': 0.0})

    parts = []
    parts.append(f'''#VRML_SIM R2025a utf8
# Сгенерировано generate_world.py из сценария "{sc['name']}". Не редактировать вручную.
EXTERNPROTO "../protos/Courier.proto"

WorldInfo {{
  info [ "Virtual Courier — {sc['name']}" ]
  title "Virtual Courier"
  basicTimeStep 20
  contactProperties [
    ContactProperties {{ material2 "caster" coulombFriction [ 0.01 ] }}
    ContactProperties {{ material1 "cargo" material2 "tray" coulombFriction [ 0.6 ] }}
    ContactProperties {{ material1 "cargo" coulombFriction [ 0.6 ] }}
    ContactProperties {{ material2 "wheel" coulombFriction [ 2.0 ] }}
  ]
}}
''')
    # Обзорная камера: сверху-сбоку на центр полигона.
    eye = (W / 2, -H * 0.25, max(W, H) * 1.25)
    pitch = math.atan2(eye[2], H / 2 - eye[1])
    look = quat_mul(quat_axis((0, 0, 1), math.pi / 2), quat_axis((0, 1, 0), pitch))
    ax, ay, az, ang = quat_to_axis_angle(look)
    parts.append(f'''Viewpoint {{
  orientation {fmt(ax, ay, az, ang)}
  position {fmt(*eye)}
}}
Background {{ skyColor [ 0.78 0.82 0.88 ] }}
DirectionalLight {{ direction -0.3 0.5 -1 intensity 1.6 castShadows FALSE }}
DirectionalLight {{ direction 0.4 -0.3 -1 intensity 0.6 castShadows FALSE }}
Solid {{
  translation {fmt(W / 2, H / 2, -0.01)}
  children [ Shape {{ appearance PBRAppearance {{ baseColor 0.55 0.57 0.6 roughness 1 metalness 0 }} geometry Box {{ size {fmt(W + 2, H + 2, 0.02)} }} }} ]
  name "floor"
  boundingObject Box {{ size {fmt(W + 2, H + 2, 0.02)} }}
}}
''')
    # Сетка на полу — помогает зрителям и отладке.
    for k in range(1, cols):
        parts.append(floor_mark(k * size, H / 2, 0.02, H, (0.48, 0.5, 0.53), 0.001))
    for k in range(1, rows):
        parts.append(floor_mark(W / 2, k * size, W, 0.02, (0.48, 0.5, 0.53), 0.001))

    for n, (p0, p1) in enumerate(boundary):
        parts.append(wall_node(f'boundary_{n}', p0, p1, (0.6, 0.6, 0.62)))
    for n, (p0, p1) in enumerate(segments):
        parts.append(wall_node(f'wall_{n}', p0, p1))
    for n, ob in enumerate(obstacles):
        parts.append(box_node(f'pallet_{n}', *ob['center'], *ob['size'], 0.25, ob['yaw'], (0.62, 0.45, 0.25)))

    # ---- Старт-Финиш и зона выгрузки
    scx, scy = cell_center(start['cell'], size)
    parts.append(floor_mark(scx, scy, size - 0.1, size - 0.1, (0.35, 0.62, 0.38), 0.002))
    dx, dy = SIDES[start['side']]
    uz_depth, uz_width = 0.6, 1.0
    wx, wy = side_point(start['cell'], start['side'], size)
    ucx = wx - dx * (WALL_T / 2 + uz_depth / 2)
    ucy = wy - dy * (WALL_T / 2 + uz_depth / 2)
    usx, usy = (uz_depth, uz_width) if dx else (uz_width, uz_depth)
    parts.append(floor_mark(ucx, ucy, usx, usy, (0.95, 0.95, 0.95), 0.003))
    parts.append(floor_mark(ucx, ucy, usx - 0.06, usy - 0.06, (0.2, 0.75, 0.3), 0.004))
    parts.append(wall_sign(wx, wy, start['side'], FINISH_MARKER))

    # ---- Станции загрузки
    station_meta = []
    for st in stations:
        sid = st['id']
        color = STATION_COLORS.get(sid, (0.9, 0.9, 0.3))
        cx, cy = cell_center(st['cell'], size)
        zone = 1.4
        parts.append(floor_mark(cx, cy, zone, zone, color, 0.002))
        parts.append(floor_mark(cx, cy, zone - 0.1, zone - 0.1, tuple(c * 0.7 for c in color), 0.003))
        wx, wy = side_point(st['cell'], st['side'], size)
        parts.append(wall_sign(wx, wy, st['side'], STATION_MARKERS.get(sid, 9)))
        dx, dy = SIDES[st['side']]
        # Сигнальная лампа над маркером (цвет меняет судья).
        lx, ly = wx - dx * 0.08, wy - dy * 0.08
        parts.append(f'''Pose {{
  translation {fmt(lx, ly, 0.45)}
  children [ Shape {{ appearance DEF LAMP_{sid} PBRAppearance {{ baseColor 0.2 0.2 0.2 emissiveColor 0 0 0 roughness 0.3 metalness 0 }} geometry Sphere {{ radius 0.07 }} }} ]
}}
Pose {{
  translation {fmt(wx - dx * 0.04, wy - dy * 0.04, 0.39)}
  children [ Shape {{ appearance PBRAppearance {{ baseColor 0.3 0.3 0.3 }} geometry Box {{ size {fmt(0.08 if dx else 0.04, 0.04 if dx else 0.08, 0.04)} }} }} ]
}}
''')
        # Бункер над центром станции (только визуал, без физики, выше плоскости лидара).
        parts.append(f'''Pose {{
  translation {fmt(cx, cy, 0.75)}
  children [
    Shape {{ appearance PBRAppearance {{ baseColor {fmt(*color)} roughness 0.4 metalness 0.3 }} geometry Cone {{ bottomRadius 0.22 height 0.3 }} }}
    Pose {{ translation 0 0 0.3 children [ Shape {{ appearance PBRAppearance {{ baseColor {fmt(*color)} roughness 0.4 metalness 0.3 }} geometry Cylinder {{ radius 0.22 height 0.3 }} }} ] }}
  ]
}}
''')
        station_meta.append({
            'id': sid, 'cell': st['cell'], 'side': st['side'], 'active': bool(st.get('active', True)),
            'marker_id': STATION_MARKERS.get(sid, 9), 'center': [cx, cy], 'zone_size': zone,
            'color': color,
        })

    # ---- Робот и судья
    yaw = math.radians(float(start.get('yaw_deg', 0)))
    courier_ctrl = '<extern>' if extern else 'courier_driver'
    referee_ctrl = '<extern>' if extern else 'referee'
    parts.append(f'''DEF COURIER Courier {{
  translation {fmt(scx, scy, 0.0)}
  rotation 0 0 1 {yaw:.4f}
  controller "{courier_ctrl}"
}}
Robot {{
  name "referee"
  controller "{referee_ctrl}"
  supervisor TRUE
}}
''')

    meta = {
        'name': sc['name'],
        'cell_size': size,
        'grid': [cols, rows],
        'size': [W, H],
        'time_limit': float(sc.get('time_limit', 600)),
        'start_timeout': float(sc.get('start_timeout', 120)),
        'start': {'cell': start['cell'], 'side': start['side'], 'yaw_deg': float(start.get('yaw_deg', 0)),
                  'pose': [scx, scy, yaw]},
        'unload_zone': {'center': [ucx, ucy], 'size': [usx, usy]},
        'stations': station_meta,
        'walls': [[list(p0), list(p1)] for p0, p1 in boundary + segments],
        'wall_thickness': WALL_T,
        'obstacles': obstacles,
    }
    return ''.join(parts), meta


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('scenario')
    ap.add_argument('--out', default=os.path.join(os.path.dirname(__file__), '..', 'webots', 'worlds'))
    ap.add_argument('--name', help='имя выходных файлов (по умолчанию — из сценария)')
    ap.add_argument('--extern', action='store_true', help='использовать <extern> для внешнего запуска контроллеров')
    args = ap.parse_args()

    with open(args.scenario, encoding='utf-8') as f:
        sc = yaml.safe_load(f)
    world, meta = generate(sc, extern=args.extern)
    name = args.name or sc['name']
    os.makedirs(args.out, exist_ok=True)
    with open(os.path.join(args.out, name + '.wbt'), 'w', encoding='utf-8') as f:
        f.write(world)
    with open(os.path.join(args.out, name + '.json'), 'w', encoding='utf-8') as f:
        json.dump(meta, f, ensure_ascii=False, indent=1)
    print(f'world: {os.path.join(args.out, name + ".wbt")}')


if __name__ == '__main__':
    main()
