import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import math

class BugExplorer(Node):
    def __init__(self):
        super().__init__('bug_explorer')
        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        qos.durability = DurabilityPolicy.VOLATILE

        # Params (simple y confiable)
        self.declare_parameter('use_scan_topic', '/scan')
        self.declare_parameter('forward_speed', 0.12)
        self.declare_parameter('turn_speed', 0.7)
        self.declare_parameter('safe_distance', 0.45)     # cuando frenar y pasar a TURN
        self.declare_parameter('clear_distance', 0.60)    # cuando considerar “libre” para volver a avanzar
        self.declare_parameter('loop_rate', 10.0)

        self.scan_topic = self.get_parameter('use_scan_topic').value
        self.forward_speed = float(self.get_parameter('forward_speed').value)
        self.turn_speed = float(self.get_parameter('turn_speed').value)
        self.safe_distance = float(self.get_parameter('safe_distance').value)
        self.clear_distance = float(self.get_parameter('clear_distance').value)
        rate = float(self.get_parameter('loop_rate').value)

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_sub = self.create_subscription(LaserScan, self.scan_topic, self.scan_cb, qos)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_cb, 10)

        # Estado mínimo
        self.state = 'MOVE_FORWARD'   # MOVE_FORWARD | TURN
        self.latest_scan = None
        self.latest_odom = None
        self.turn_direction = 1.0     # +1 izquierda, -1 derecha

        self.timer = self.create_timer(1.0/float(rate), self.control_loop)
        self.get_logger().info(f'BugExplorer(simple) started. Using scan: {self.scan_topic}')

    def odom_cb(self, msg: Odometry):
        self.latest_odom = msg

    def scan_cb(self, msg: LaserScan):
        self.latest_scan = msg

    def get_range_sector(self, msg: LaserScan, angle_center_deg: float, width_deg: float):
        # Devuelve distancia mínima en sector centrado en angle_center_deg (robot frame, degrees)
        if msg is None:
            return float('inf')
        angle_center = math.radians(angle_center_deg)
        half = math.radians(width_deg/2.0)
        angles = []
        n = len(msg.ranges)
        angle = msg.angle_min
        best = float('inf')
        for r in msg.ranges:
            # normalizamos angulo al rango [-pi, pi]
            if abs(self._angle_diff(angle, angle_center)) <= half:
                if not math.isinf(r) and not math.isnan(r):
                    if r < best:
                        best = r
            angle += msg.angle_increment
        return best

    def _angle_diff(self, a, b):
        d = a - b
        while d > math.pi:
            d -= 2*math.pi
        while d < -math.pi:
            d += 2*math.pi
        return d

    def control_loop(self):
        if self.latest_scan is None:
            return

        # Detectores robustos: frente ancho + diagonales para esquinas
        front = self.get_range_sector(self.latest_scan, 0.0, 90.0)          # ±45°
        front_left = self.get_range_sector(self.latest_scan, 30.0, 40.0)
        front_right = self.get_range_sector(self.latest_scan, -30.0, 40.0)
        left = self.get_range_sector(self.latest_scan, 90.0, 60.0)
        right = self.get_range_sector(self.latest_scan, -90.0, 60.0)

        # Si cualquiera de estos está cerca, consideramos “bloqueado”
        blocked = (front < self.safe_distance) or (front_left < self.safe_distance) or (front_right < self.safe_distance)
        clear = (front > self.clear_distance) and (front_left > self.clear_distance) and (front_right > self.clear_distance)

        twist = Twist()

        if self.state == 'MOVE_FORWARD':
            if blocked:
                # Elegir dirección con más espacio lateral (si alguno es inf, perfecto)
                self.turn_direction = 1.0 if left > right else -1.0
                self.state = 'TURN'
                twist.linear.x = 0.0
                twist.angular.z = self.turn_direction * self.turn_speed
            else:
                twist.linear.x = self.forward_speed
                twist.angular.z = 0.0

        elif self.state == 'TURN':
            twist.linear.x = 0.0
            twist.angular.z = self.turn_direction * self.turn_speed
            if clear:
                self.state = 'MOVE_FORWARD'
                twist.angular.z = 0.0
                twist.linear.x = self.forward_speed

        self.cmd_pub.publish(twist)

    def destroy_node(self):
        # stop robot before exit
        stop = Twist()
        self.cmd_pub.publish(stop)
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = BugExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()
