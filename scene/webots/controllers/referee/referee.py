"""Судья «Виртуального курьера» (Supervisor).

* зажигает лампы активных станций и выдаёт груз (сбрасывает кубик на лоток),
* эмулирует весовой датчик лотка (/courier/cargo_count),
* считает баллы и штрафы, ведёт таймер, рисует табло в окне Webots,
* по окончании заезда пишет results.json (и закрывает Webots в режиме EVAL).

Служебные топики (решению использовать ЗАПРЕЩЕНО):
    /referee/state  std_msgs/String  — JSON с полным состоянием (2 Гц)
    /referee/score  std_msgs/Int32
    /referee/events std_msgs/String  — по сообщению на каждое событие
"""

import json
import math
import os
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32, UInt8, Float64
from geometry_msgs.msg import Twist

from controller import Supervisor

# ---- баллы (должны совпадать с разделом 12 регламента)
PTS_LEAVE_START = 5
PTS_STATION_REACHED = 10
PTS_CARGO_LOADED = 15
PTS_CARGO_DELIVERED = 25
PTS_ALL_DELIVERED = 20
PTS_TIME_BONUS_MAX = 20
PEN_COLLISION = -2
PEN_COLLISION_CAP = -20
PEN_CARGO_LOST = -10

ROBOT_RADIUS = 0.175
DWELL_TIME = 2.0          # с неподвижно в зоне станции → выдача груза
STILL_LIN = 0.03          # м/с
STILL_ANG = 0.15          # рад/с
COLLISION_DEBOUNCE = 1.0  # с
CARGO_SIZE = 0.05
CARGO_MASS = 0.3
FLOOR_Z = 0.045           # центр кубика ниже — лежит на полу
FINISH_DELAY = 3.0        # с после окончания до закрытия (EVAL)

# Объём лотка в системе координат робота: (xmin, xmax, ymin, ymax, zmin, zmax)
TRAY_BOX = (-0.20, 0.05, -0.14, 0.14, 0.07, 0.25)
# Места на лотке, куда падает груз из бункера (в СК робота): берётся первое свободное,
# чтобы кубики не ложились друг на друга.
TRAY_DROP_SLOTS = [(-0.07, 0.0, 0.16), (-0.07, 0.065, 0.16), (-0.07, -0.065, 0.16)]


def seg_dist(px, py, ax, ay, bx, by):
    vx, vy = bx - ax, by - ay
    wx, wy = px - ax, py - ay
    L = vx * vx + vy * vy
    t = 0.0 if L == 0 else max(0.0, min(1.0, (wx * vx + wy * vy) / L))
    return math.hypot(px - (ax + t * vx), py - (ay + t * vy))


def rect_dist(px, py, cx, cy, sx, sy):
    dx = max(abs(px - cx) - sx / 2, 0.0)
    dy = max(abs(py - cy) - sy / 2, 0.0)
    return math.hypot(dx, dy)


class Referee(Node):
    def __init__(self, sup):
        super().__init__('referee')
        self.sup = sup
        self.dt = int(sup.getBasicTimeStep())

        meta_path = os.environ.get('VC_WORLD_META')
        if not meta_path:
            world = sup.getWorldPath()
            meta_path = os.path.splitext(world)[0] + '.json'
        if not os.path.exists(meta_path):
            fallback = os.path.join('/scene/webots/worlds', os.path.basename(meta_path))
            if os.path.exists(fallback):
                meta_path = fallback
        with open(meta_path, encoding='utf-8') as f:
            self.meta = json.load(f)

        self.eval_mode = os.environ.get('VC_EVAL', 'false').lower() == 'true'
        self.results_dir = os.environ.get('VC_RESULTS_DIR', '/results')
        self.team = os.environ.get('VC_TEAM', 'local')
        self.attempt = os.environ.get('VC_ATTEMPT', '1')
        self.snapshot = os.environ.get('VC_SNAPSHOT')   # путь PNG: снимок обзорной камеры и выход

        self.robot = sup.getFromDef('COURIER')
        self.root_children = sup.getRoot().getField('children')

        self.state_pub = self.create_publisher(String, '/referee/state', 10)
        self.score_pub = self.create_publisher(Int32, '/referee/score', 10)
        self.events_pub = self.create_publisher(String, '/referee/events', 50)
        self.cargo_pub = self.create_publisher(UInt8, '/courier/cargo_count', 10)
        self.create_subscription(Twist, '/cmd_vel', self.on_command, 10)
        self.create_subscription(Float64, '/courier/tray_cmd', self.on_command, 10)

        self.started = False
        self.start_time = None
        self.finished = False
        self.finish_time = None
        self.finish_reason = ''
        self.closed_at = None

        self.score_items = []          # (sim_time, points, text)
        self.collision_pen = 0
        self.collisions = 0
        self.in_contact = False
        self.last_collision_t = -1e9
        self.left_start = False

        self.stations = {}
        for st in self.meta['stations']:
            self.stations[st['id']] = {
                **st, 'reached': False, 'dwell': 0.0, 'issued': False,
            }
        self.cargos = {}               # id станции → состояние груза
        self.set_lamps()

    # ---------------------------------------------------------------- утилиты

    def now(self):
        return self.sup.getTime()

    def elapsed(self):
        if not self.started:
            return 0.0
        end = self.finish_time if self.finished else self.now()
        return end - self.start_time

    def score(self):
        return sum(p for _, p, _ in self.score_items)

    def add(self, points, text):
        self.score_items.append((round(self.elapsed(), 2), points, text))
        msg = f'[{self.elapsed():6.1f} с] {points:+d}  {text}'
        self.get_logger().info(msg)
        self.events_pub.publish(String(data=json.dumps(
            {'t': round(self.elapsed(), 2), 'points': points, 'text': text}, ensure_ascii=False)))

    def event(self, text):
        self.get_logger().info(f'[{self.elapsed():6.1f} с] {text}')
        self.events_pub.publish(String(data=json.dumps(
            {'t': round(self.elapsed(), 2), 'points': 0, 'text': text}, ensure_ascii=False)))

    def set_lamp(self, sid, rgb):
        node = self.sup.getFromDef(f'LAMP_{sid}')
        if node:
            node.getField('emissiveColor').setSFColor(list(rgb))
            node.getField('baseColor').setSFColor([max(0.2, c) for c in rgb])

    def set_lamps(self):
        for sid, st in self.stations.items():
            if not st['active']:
                self.set_lamp(sid, (0.0, 0.0, 0.0))
            elif st['issued']:
                self.set_lamp(sid, (0.9, 0.6, 0.0))   # груз уже выдан
            else:
                self.set_lamp(sid, (0.0, 1.0, 0.1))   # готова к выдаче

    def robot_pose(self):
        p = self.robot.getPosition()
        R = self.robot.getOrientation()
        yaw = math.atan2(R[3], R[0])
        return p, R, yaw

    def to_robot_frame(self, p, R, q):
        d = (q[0] - p[0], q[1] - p[1], q[2] - p[2])
        # R — 3x3 построчно, локальные = R^T * d
        return (R[0] * d[0] + R[3] * d[1] + R[6] * d[2],
                R[1] * d[0] + R[4] * d[1] + R[7] * d[2],
                R[2] * d[0] + R[5] * d[1] + R[8] * d[2])

    def to_world(self, p, R, l):
        return [p[0] + R[0] * l[0] + R[1] * l[1] + R[2] * l[2],
                p[1] + R[3] * l[0] + R[4] * l[1] + R[5] * l[2],
                p[2] + R[6] * l[0] + R[7] * l[1] + R[8] * l[2]]

    def in_cell(self, x, y, cell, inset=0.0):
        s = self.meta['cell_size']
        return (cell[0] * s + inset <= x <= (cell[0] + 1) * s - inset and
                cell[1] * s + inset <= y <= (cell[1] + 1) * s - inset)

    # ---------------------------------------------------------------- ROS

    def on_command(self, msg):
        if self.started:
            return
        active = True
        if isinstance(msg, Twist):
            active = abs(msg.linear.x) > 1e-3 or abs(msg.angular.z) > 1e-3
        if active:
            self.start('первая команда решения')

    def start(self, why):
        self.started = True
        self.start_time = self.now()
        self.event(f'СТАРТ ({why})')

    # ---------------------------------------------------------------- логика

    def step(self):
        t = self.now()
        if not self.started and t >= self.meta['start_timeout']:
            self.start('таймаут ожидания решения')

        p, R, yaw = self.robot_pose()
        v = self.robot.getVelocity()
        lin = math.hypot(v[0], v[1])
        ang = abs(v[5])

        if self.started and not self.finished:
            self.check_start_zone(p)
            self.check_collisions(p)
            self.check_stations(p, lin, ang)
        self.check_cargos(p, R)

        if self.started and not self.finished:
            if self.cargos and self.all_delivered():
                self.finish('все грузы доставлены')
            elif self.elapsed() >= self.meta['time_limit']:
                self.finish('истекло время')

        if self.finished and self.eval_mode and self.closed_at is None and \
                t - self.finish_time >= FINISH_DELAY:
            self.closed_at = t
            self.sup.simulationQuit(0)

        ms = int(round(t * 1000))
        if self.snapshot and t >= 1.0:
            self.draw_hud()
            self.sup.exportImage(self.snapshot, 90)
            self.get_logger().info(f'снимок сохранён: {self.snapshot}')
            self.snapshot = None
            self.sup.simulationQuit(0)

        if ms % 100 == 0:
            self.cargo_pub.publish(UInt8(data=self.cargo_on_tray(p, R)))
        if ms % 500 == 0:
            self.publish_state(p, yaw)
        if ms % 200 == 0:
            self.draw_hud()

    def check_start_zone(self, p):
        if not self.left_start and not self.in_cell(p[0], p[1], self.meta['start']['cell']):
            self.left_start = True
            self.add(PTS_LEAVE_START, 'выезд из зоны Старт')

    def check_collisions(self, p):
        r = ROBOT_RADIUS + self.meta['wall_thickness'] / 2 + 0.01
        touching = any(seg_dist(p[0], p[1], a[0], a[1], b[0], b[1]) < r for a, b in self.meta['walls'])
        if not touching:
            touching = any(rect_dist(p[0], p[1], o['center'][0], o['center'][1], o['size'][0], o['size'][1])
                           < ROBOT_RADIUS + 0.01 for o in self.meta['obstacles'])
        t = self.now()
        if touching and not self.in_contact and t - self.last_collision_t > COLLISION_DEBOUNCE:
            self.collisions += 1
            self.last_collision_t = t
            if self.collision_pen > PEN_COLLISION_CAP:
                pen = max(PEN_COLLISION, PEN_COLLISION_CAP - self.collision_pen)
                self.collision_pen += pen
                self.add(pen, f'касание препятствия #{self.collisions}')
            else:
                self.event(f'касание препятствия #{self.collisions} (лимит штрафа исчерпан)')
        self.in_contact = touching

    def check_stations(self, p, lin, ang):
        dt = self.dt / 1000.0
        for sid, st in self.stations.items():
            half = st['zone_size'] / 2
            inside = abs(p[0] - st['center'][0]) <= half and abs(p[1] - st['center'][1]) <= half
            if not inside:
                st['dwell'] = 0.0
                continue
            if st['active'] and not st['reached']:
                st['reached'] = True
                self.add(PTS_STATION_REACHED, f'станция {sid} достигнута')
            if not st['active'] or st['issued']:
                continue
            if lin < STILL_LIN and ang < STILL_ANG:
                st['dwell'] += dt
            else:
                st['dwell'] = 0.0
            if st['dwell'] >= DWELL_TIME:
                self.issue_cargo(sid)

    def issue_cargo(self, sid):
        st = self.stations[sid]
        st['issued'] = True
        p, R, _ = self.robot_pose()
        taken = [self.to_robot_frame(p, R, c['node'].getPosition()) for c in self.cargos.values()
                 if c['node'] is not None and c['state'] == 'on_robot']
        slot = next((sl for sl in TRAY_DROP_SLOTS
                     if all(math.hypot(sl[0] - q[0], sl[1] - q[1]) > 0.045 for q in taken)), TRAY_DROP_SLOTS[0])
        pos = self.to_world(p, R, slot)
        color = ' '.join(f'{c:.3f}' for c in st['color'])
        node = f'''DEF CARGO_{sid} Solid {{
  translation {pos[0]:.4f} {pos[1]:.4f} {pos[2]:.4f}
  rotation 0 0 1 {math.atan2(R[3], R[0]):.4f}
  children [ Shape {{ appearance PBRAppearance {{ baseColor {color} roughness 0.6 metalness 0 }} geometry Box {{ size {CARGO_SIZE} {CARGO_SIZE} {CARGO_SIZE} }} }} ]
  name "cargo_{sid}"
  contactMaterial "cargo"
  boundingObject Box {{ size {CARGO_SIZE} {CARGO_SIZE} {CARGO_SIZE} }}
  physics Physics {{ density -1 mass {CARGO_MASS} }}
}}'''
        self.root_children.importMFNodeFromString(-1, node)
        self.cargos[sid] = {'node': self.sup.getFromDef(f'CARGO_{sid}'), 'state': 'issued',
                            'loaded': False, 'delivered': False, 'lost': False}
        self.event(f'станция {sid}: груз выдан')
        self.set_lamps()

    def cargo_on_tray(self, p, R):
        n = 0
        for c in self.cargos.values():
            if c['node'] is not None and c['state'] == 'on_robot':
                n += 1
        return n

    def check_cargos(self, p, R):
        uz = self.meta['unload_zone']
        for sid, c in self.cargos.items():
            node = c['node']
            if node is None:
                continue
            q = node.getPosition()
            local = self.to_robot_frame(p, R, q)
            on_robot = (TRAY_BOX[0] <= local[0] <= TRAY_BOX[1] and TRAY_BOX[2] <= local[1] <= TRAY_BOX[3]
                        and TRAY_BOX[4] <= local[2] <= TRAY_BOX[5])
            on_floor = q[2] < FLOOR_Z
            vq = node.getVelocity()
            resting = math.hypot(vq[0], vq[1], vq[2]) < 0.02
            if on_robot:
                c['state'] = 'on_robot'
            elif on_floor:
                c['state'] = 'floor'
            else:
                c['state'] = 'moving'

            if self.finished:
                continue
            st = self.stations[sid]
            in_station = abs(p[0] - st['center'][0]) <= st['zone_size'] / 2 and \
                abs(p[1] - st['center'][1]) <= st['zone_size'] / 2
            if not c['loaded'] and on_robot and not in_station:
                c['loaded'] = True
                self.add(PTS_CARGO_LOADED, f'груз {sid} погружен и вывезен со станции')
            if on_floor and resting and not c['delivered'] and not c['lost']:
                in_zone = abs(q[0] - uz['center'][0]) <= uz['size'][0] / 2 and \
                    abs(q[1] - uz['center'][1]) <= uz['size'][1] / 2
                if in_zone:
                    c['delivered'] = True
                    self.add(PTS_CARGO_DELIVERED, f'груз {sid} доставлен в зону выгрузки')
                else:
                    c['lost'] = True
                    self.add(PEN_CARGO_LOST, f'груз {sid} потерян (упал вне зоны выгрузки)')
                    node.remove()
                    c['node'] = None
                    c['state'] = 'removed'

    def all_delivered(self):
        active = [sid for sid, st in self.stations.items() if st['active']]
        return all(self.cargos.get(sid, {}).get('delivered') for sid in active)

    def finish(self, reason):
        self.finished = True
        self.finish_time = self.now()
        self.finish_reason = reason
        if reason == 'все грузы доставлены':
            self.add(PTS_ALL_DELIVERED, 'бонус: все грузы доставлены')
            T, Tmax = self.elapsed(), self.meta['time_limit']
            bonus = int(round(PTS_TIME_BONUS_MAX * max(0.0, Tmax - T) / Tmax))
            if bonus:
                self.add(bonus, f'бонус за время ({T:.1f} с)')
        self.event(f'ФИНИШ: {reason}. Итог: {self.score()} баллов')
        self.write_results()
        self.draw_hud()

    # ---------------------------------------------------------------- вывод

    def summary(self):
        return {
            'team': self.team,
            'attempt': self.attempt,
            'scenario': self.meta['name'],
            'score': self.score(),
            'finished': self.finished,
            'finish_reason': self.finish_reason,
            'elapsed': round(self.elapsed(), 2),
            'time_limit': self.meta['time_limit'],
            'collisions': self.collisions,
            'stations': {sid: {'active': st['active'], 'reached': st['reached'], 'issued': st['issued']}
                         for sid, st in self.stations.items()},
            'cargos': {sid: {k: c[k] for k in ('state', 'loaded', 'delivered', 'lost')}
                       for sid, c in self.cargos.items()},
            'log': [{'t': t, 'points': pts, 'text': txt} for t, pts, txt in self.score_items],
        }

    def publish_state(self, p, yaw):
        s = self.summary()
        s['started'] = self.started
        s['robot'] = {'x': round(p[0], 3), 'y': round(p[1], 3), 'yaw': round(yaw, 3)}
        self.state_pub.publish(String(data=json.dumps(s, ensure_ascii=False)))
        self.score_pub.publish(Int32(data=self.score()))

    def write_results(self):
        try:
            os.makedirs(self.results_dir, exist_ok=True)
            name = f'{self.team}_{self.meta["name"]}_{self.attempt}_{time.strftime("%Y%m%d-%H%M%S")}.json'
            path = os.path.join(self.results_dir, name)
            with open(path, 'w', encoding='utf-8') as f:
                json.dump(self.summary(), f, ensure_ascii=False, indent=2)
            self.get_logger().info(f'результат записан: {path}')
        except OSError as e:
            self.get_logger().error(f'не удалось записать результат: {e}')

    def draw_hud(self):
        el = self.elapsed()
        left = max(0.0, self.meta['time_limit'] - el)
        status = 'ОЖИДАНИЕ СТАРТА' if not self.started else ('ФИНИШ: ' + self.finish_reason if self.finished else 'ЗАЕЗД')
        self.sup.setLabel(0, f'Виртуальный курьер | {self.team} | {self.meta["name"]}', 0.01, 0.01, 0.07, 0xFFFFFF, 0.2, 'Arial')
        self.sup.setLabel(1, f'{status}   Время: {int(el // 60):02d}:{el % 60:04.1f}   Осталось: {int(left // 60):02d}:{int(left % 60):02d}',
                          0.01, 0.05, 0.07, 0xFFFF66, 0.2, 'Arial')
        delivered = sum(1 for c in self.cargos.values() if c['delivered'])
        need = sum(1 for st in self.stations.values() if st['active'])
        self.sup.setLabel(2, f'Баллы: {self.score()}   Грузы: {delivered}/{need}   Касания: {self.collisions}',
                          0.01, 0.09, 0.07, 0x66FF66, 0.2, 'Arial')
        last = self.score_items[-3:]
        self.sup.setLabel(3, '\n'.join(f'{pts:+d} {txt}' for _, pts, txt in last), 0.01, 0.13, 0.055, 0xDDDDDD, 0.3, 'Arial')


def main():
    sup = Supervisor()
    rclpy.init()
    node = Referee(sup)
    dt = int(sup.getBasicTimeStep())
    while sup.step(dt) != -1:
        rclpy.spin_once(node, timeout_sec=0)
        node.step()
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
