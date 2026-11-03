#!/usr/bin/env python3

import math

import rclpy
from geometry_msgs.msg import Point, Twist
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray


class Explorer(Node):
    def __init__(self):
        super().__init__('explorer')
        self.scan = None
        self.scan_time = None
        self.heading = None
        self.turning = False
        self.junction_latched = False
        self.decision_position = None
        self.bypass = []
        self.last_control = None
        self.linear_speed = 0.0
        self.visits = {}
        self.current_cell = None
        self.grid_origin = (0.0, 0.0)
        self.grid_yaw = 0.0
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.targets_pub = self.create_publisher(MarkerArray, '/exploration_targets', qos)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_subscription = self.create_subscription(
            LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.timer = self.create_timer(0.05, self.control_loop)
        self.get_logger().info('Прямо по коридору; выбор прохода на развилке; скорость до 0.35 м/с')

    def on_scan(self, msg):
        self.scan = msg
        self.scan_time = self.get_clock().now()

    def control_loop(self):
        now = self.get_clock().now()
        if self.scan is None or (now - self.scan_time).nanoseconds > 500_000_000:
            self.drive(0.0, 0.0)
            return
        try:
            transform = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            grid = self.tf_buffer.lookup_transform('map', 'maze_grid', rclpy.time.Time()).transform
            odom = self.tf_buffer.lookup_transform('odom', 'base_link', rclpy.time.Time()).transform
        except TransformException:
            self.drive(0.0, 0.0)
            return
        p = transform.transform.translation
        q = transform.transform.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.grid_origin = (grid.translation.x, grid.translation.y)
        gq = grid.rotation
        self.grid_yaw = math.atan2(2*(gq.w*gq.z+gq.x*gq.y),1-2*(gq.y*gq.y+gq.z*gq.z))
        self.record_visit(*self.to_grid(p.x, p.y))
        oq = odom.rotation
        odom_yaw = math.atan2(2*(oq.w*oq.z+oq.x*oq.y), 1-2*(oq.y*oq.y+oq.z*oq.z))
        if self.heading is None:
            self.heading = odom_yaw
        error = math.atan2(math.sin(self.heading-odom_yaw), math.cos(self.heading-odom_yaw))
        if self.turning and abs(error) < math.radians(8):
            self.turning = False

        front = self.clearance(0.0)
        right = self.passage_open(-math.pi/2)
        left = self.passage_open(math.pi/2)
        if self.decision_position is not None:
            travelled = math.hypot(odom.translation.x-self.decision_position[0],
                                   odom.translation.y-self.decision_position[1])
            if not self.turning and travelled > 1.0 and not right and not left:
                self.junction_latched = False
        if not self.turning and not self.bypass and abs(error) < math.radians(15):
            if front < 0.65 or ((right or left) and not self.junction_latched):
                if front < 0.65:
                    self.bypass = self.find_bypass(odom.translation.x, odom.translation.y, odom_yaw)
                if self.bypass:
                    self.get_logger().info('Объезд препятствия; сохраняем направление коридора')
                else:
                    self.choose_direction(p.x, p.y, yaw, odom_yaw, right, left, front)
                self.decision_position = (odom.translation.x, odom.translation.y)
                self.junction_latched = True
                error = math.atan2(math.sin(self.heading-odom_yaw), math.cos(self.heading-odom_yaw))

        while self.bypass and math.hypot(self.bypass[0][0]-odom.translation.x,
                                        self.bypass[0][1]-odom.translation.y) < 0.2:
            self.bypass.pop(0)
            if not self.bypass:
                self.get_logger().info('Объезд завершён; продолжаем по коридору')
        if self.bypass:
            target = math.atan2(self.bypass[0][1]-odom.translation.y,
                                self.bypass[0][0]-odom.translation.x)
            error = math.atan2(math.sin(target-odom_yaw), math.cos(target-odom_yaw))

        desired = 0.35 * max(0.0, min(1.0, (front - 0.1) / 0.6)) * max(0.0, math.cos(error))**2
        dt = min(0.1, (now - self.last_control).nanoseconds / 1e9) if self.last_control else 0.05
        self.last_control = now
        speed = max(self.linear_speed - 0.8 * dt, min(self.linear_speed + 0.4 * dt, desired))
        if front < 0.1 or abs(error) > math.radians(30):
            speed = 0.0
        correction = 0.0 if self.turning or self.bypass else self.wall_correction()
        self.drive(speed, max(-1.2, min(1.2, 1.8 * error + correction)))
        self.publish_memory(p.x, p.y, yaw + error)

    def find_bypass(self, x, y, yaw):
        points = []
        for i, distance in enumerate(self.scan.ranges):
            if not math.isfinite(distance) or distance < self.scan.range_min:
                continue
            angle = self.scan.angle_min+i*self.scan.angle_increment
            points.append((distance*math.cos(angle), distance*math.sin(angle)))
        for side in (-1, 1):
            path = [(0.0, 0.0), (0.6, side*0.6), (1.6, side*0.6)]
            clear = True
            for (ax, ay), (bx, by) in zip(path, path[1:]):
                dx, dy = bx-ax, by-ay
                for px, py in points:
                    fraction = max(0.0, min(1.0, ((px-ax)*dx+(py-ay)*dy)/(dx*dx+dy*dy)))
                    if math.hypot(px-ax-fraction*dx, py-ay-fraction*dy) < 0.30:
                        clear = False
                        break
                if not clear:
                    break
            if clear:
                return [(x+px*math.cos(yaw)-py*math.sin(yaw),
                         y+px*math.sin(yaw)+py*math.cos(yaw)) for px, py in path[1:]]
        return []

    def choose_direction(self, x, y, yaw, odom_yaw, right, left, front):
        options = []
        for offset, available, name in [(-math.pi/2, right, 'направо'),
                                       (0.0, front >= 0.65, 'прямо'),
                                       (math.pi/2, left, 'налево')]:
            if available:
                angle = yaw + self.heading - odom_yaw + offset
                cell = self.cell_at(*self.to_grid(x+2*math.cos(angle), y+2*math.sin(angle)))
                options.append((self.visits.get(cell, 0), offset, name))
        _, offset, name = min(options, key=lambda item: item[0]) if options else (0, math.pi, 'разворот')
        self.heading += offset
        self.turning = offset != 0.0
        self.get_logger().info(f'Проход: {name}; посещено клеток: {len(self.visits)}')

    def passage_open(self, angle):
        return min(self.clearance(angle+offset) for offset in (-0.15, 0.0, 0.15)) > 1.35

    def wall_correction(self):
        left, right = self.clearance(math.pi/2), self.clearance(-math.pi/2)
        if left < 1.1 and right < 1.1:
            correction = 0.4*(left-right)
        elif right < 1.1:
            correction = 0.6*(0.65-right)
        elif left < 1.1:
            correction = 0.6*(left-0.65)
        else:
            correction = 0.0
        return max(-0.3, min(0.3, correction))

    def to_grid(self, x, y):
        dx, dy = x-self.grid_origin[0], y-self.grid_origin[1]
        return (math.cos(self.grid_yaw)*dx+math.sin(self.grid_yaw)*dy,
                -math.sin(self.grid_yaw)*dx+math.cos(self.grid_yaw)*dy)

    def cell_at(self, x, y):
        return math.floor(x/2+0.5), math.floor(y/2+0.5)

    def record_visit(self, x, y):
        cell = self.cell_at(x,y)
        if max(abs(x-2*cell[0]),abs(y-2*cell[1])) > 0.8 or cell == self.current_cell:
            return
        self.current_cell = cell
        self.visits[cell] = self.visits.get(cell,0)+1
        self.get_logger().info(f'Клетка {cell}: посещений {self.visits[cell]}; уникальных {len(self.visits)}')

    def publish_memory(self, x, y, heading):
        markers = MarkerArray()
        direction = Marker()
        direction.header.frame_id = 'map'
        direction.header.stamp = self.get_clock().now().to_msg()
        direction.ns = 'exploration_targets'
        direction.id = 0
        direction.type = Marker.SPHERE
        direction.pose.orientation.w = 1.0
        direction.pose.position = Point(x=x+math.cos(heading),y=y+math.sin(heading),z=0.1)
        direction.scale.x = direction.scale.y = direction.scale.z = 0.15
        direction.color.g = direction.color.a = 1.0
        direction.lifetime.nanosec = 300_000_000
        markers.markers.append(direction)
        visited = Marker()
        visited.header.frame_id = 'maze_grid'
        visited.header.stamp = direction.header.stamp
        visited.ns = 'visited_cells'
        visited.id = 0
        visited.type = Marker.CUBE_LIST
        visited.action = Marker.ADD if self.visits else Marker.DELETE
        visited.pose.orientation.w = 1.0
        visited.scale.x = visited.scale.y = 1.9
        visited.scale.z = 0.01
        visited.color.b = 1.0
        visited.color.g = 0.5
        visited.color.a = 0.2
        visited.lifetime.nanosec = 300_000_000
        visited.points = [Point(x=float(2*i),y=float(2*j),z=0.02) for i,j in sorted(self.visits)]
        markers.markers.append(visited)
        self.targets_pub.publish(markers)

    def clearance(self, heading):
        best = self.scan.range_max
        valid = False
        for i, distance in enumerate(self.scan.ranges):
            if math.isnan(distance) or distance < self.scan.range_min:
                continue
            valid = True
            distance = min(distance, self.scan.range_max)
            angle = self.scan.angle_min + i * self.scan.angle_increment - heading
            forward = distance * math.cos(angle)
            side = abs(distance * math.sin(angle))
            # Проверяем полосу движения с запасом относительно радиуса робота 0.175 м.
            if forward > 0 and side < 0.27:
                best = min(best, max(0.0, forward - math.sqrt(0.27 ** 2 - side ** 2)))
        return best if valid else 0.0

    def drive(self, speed, turn):
        msg = Twist()
        msg.linear.x = float(speed)
        msg.angular.z = float(turn)
        self.cmd_pub.publish(msg)
        self.linear_speed = speed


def main():
    rclpy.init()
    node = Explorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.drive(0.0, 0.0)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
