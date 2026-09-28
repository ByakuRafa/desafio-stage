import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan


#states que o robo vai usar
class State:
    WAIT_FOR_GOAL = 0
    GO_TO_GOAL = 1
    WALL_FOLLOW = 2
    FINISHED = 3

def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))

class Pursuer(Node):
    def __init__(self):
        super().__init__('pursuer')

        # Estado iniciaç
        self.state = State.WAIT_FOR_GOAL
        self.start_x = 0.0
        self.start_y = 0.0
        self.target_x = 0.0
        self.target_y = 0.0
        self.hit_distance = 0.0

        # Posicao global
        self.curr_x = 0.0
        self.curr_y = 0.0
        self.curr_yaw = 0.0
        self.has_pose = False

        # config de sensores
        self.front_dist = 10.0
        self.front_right_dist = 10.0
        self.right_dist = 10.0
        self.has_scan = False

    # cria publishers e subscirbers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_sub = self.create_subscription(LaserScan, '/base_scan', self.scan_callback, 10)
        self.goal_sub = self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)
        
        # ODOM recebe o ground trurth
        self.gt_sub = self.create_subscription(Odometry, '/ground_truth', self.pose_callback, 10)

        self.timer = self.create_timer(0.05, self.control_loop)

    def goal_callback(self, msg: PoseStamped):
       
        self.target_x = msg.pose.position.x
        self.target_y = msg.pose.position.y

        self.start_x = self.curr_x
        self.start_y = self.curr_y

        self.state = State.GO_TO_GOAL
        
        self.get_logger().info(
            f'define objetivo posicao inicial: ({self.start_x:.2f}, {self.start_y:.2f}) '
            f' objetivo: ({self.target_x:.2f}, {self.target_y:.2f})'
        )   

    # recebe a pose do robo e atualiza a posição atual e inclinacao
    def pose_callback(self, msg: Odometry):
        self.has_pose = True
        self.curr_x = msg.pose.pose.position.x
        self.curr_y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.curr_yaw = math.atan2(siny_cosp, cosy_cosp)

    # recebe varredura de scan e atualiza distancias frente e lateral
    def scan_callback(self, msg: LaserScan):
        self.has_scan = True
        
        ranges = [r if not math.isnan(r) and not math.isinf(r) else msg.range_max for r in msg.ranges]
        if not ranges:
            return

        angle_inc = msg.angle_increment
        center = len(ranges) // 2

        idx_front = int(0.2 / angle_inc) if angle_inc > 0 else 5
        idx_right = int(1.57 / angle_inc) if angle_inc > 0 else 20
        idx_f_right = int(0.78 / angle_inc) if angle_inc > 0 else 10

        self.front_dist = min(ranges[max(0, center - idx_front) : min(len(ranges), center + idx_front)])
        self.front_right_dist = min(ranges[center - idx_right : center - idx_front])
        self.right_dist = ranges[center - idx_right]

    # calcula distancia de robo ate linha entre ponto incial e objetivo
    def dist_to_m_line(self):
        num = abs((self.target_x - self.start_x) * (self.start_y - self.curr_y) - 
                  (self.start_x - self.curr_x) * (self.target_y - self.start_y))
        den = math.hypot(self.target_x - self.start_x, self.target_y - self.start_y)
        return num / den if den > 0 else 0.0

    # laco de control princpal
    def control_loop(self):
        if not self.has_pose or not self.has_scan or self.state == State.WAIT_FOR_GOAL:
            return

        twist = Twist()
        dist_to_goal = math.hypot(self.target_x - self.curr_x, self.target_y - self.curr_y)

        if dist_to_goal < 0.25:
            if self.state != State.FINISHED:
                self.get_logger().info(f'cehou no objetivo (X={self.curr_x:.2f}, Y={self.curr_y:.2f}).')
                self.state = State.FINISHED
            self.cmd_pub.publish(Twist())
            return

        if self.state == State.GO_TO_GOAL:
            desired_yaw = math.atan2(self.target_y - self.curr_y, self.target_x - self.curr_x)
            yaw_error = normalize_angle(desired_yaw - self.curr_yaw)

            if self.front_dist < 0.60:
                self.hit_distance = dist_to_goal
                self.get_logger().warn(f'parede detectada a {self.front_dist:.2f}, troca estado')
                self.state = State.WALL_FOLLOW
                return

            if abs(yaw_error) > 0.1:
                twist.linear.x = 0.0
                twist.angular.z = math.copysign(0.8, yaw_error)
            else:
                twist.linear.x = min(0.4, dist_to_goal)
                twist.angular.z = 1.2 * yaw_error

        elif self.state == State.WALL_FOLLOW:
            line_dist = self.dist_to_m_line()
            
            if line_dist < 0.15 and dist_to_goal < (self.hit_distance - 0.25):
                self.get_logger().info(f'M-Line encontrada a {dist_to_goal:.2f}m do alvo. Retomando rota.')
                self.state = State.GO_TO_GOAL
                return

            desired_wall_dist = 0.45
            
            if self.front_dist < 0.55:
                twist.linear.x = 0.0
                twist.angular.z = 0.8
            elif self.front_right_dist < 0.55:
                twist.linear.x = 0.15
                twist.angular.z = 0.5
            elif self.right_dist > 1.2:
                twist.linear.x = 0.20
                twist.angular.z = -0.6
            else:
                error = desired_wall_dist - self.right_dist
                twist.linear.x = 0.25
                twist.angular.z = max(-0.8, min(0.8, 1.8 * error))

        self.cmd_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = Pursuer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.cmd_pub.publish(Twist())
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()