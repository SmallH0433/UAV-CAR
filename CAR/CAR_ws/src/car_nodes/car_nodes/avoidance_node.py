"""避障节点（趋向目标 + 扇区避障，类 VFH 简化版）

订阅：
  /perception/obstacles (car_interfaces/ObstacleArray)
  /odom                 (nav_msgs/Odometry)
  /goal_pose            (geometry_msgs/PoseStamped)
发布：
  /cmd_vel (geometry_msgs/Twist)
服务：
  /avoidance/set_goal (car_interfaces/SetGoal)
参数：
  safety_distance    (float, 0.5)  安全距离，小于该值停车/原地转向
  slow_down_distance (float, 1.2)  开始减速的距离
  max_linear         (float, 0.6)  最大线速度 m/s
  max_angular        (float, 1.0)  最大角速度 rad/s
  cruise_speed       (float, 0.3)  巡航速度 m/s
  enable_cruise      (bool,  False) 无目标时是否巡航
  num_sectors        (int,   36)    前方 180° 划分的扇区数
"""

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry

from car_interfaces.msg import ObstacleArray
from car_interfaces.srv import SetGoal


def yaw_from_quaternion(q):
    """四元数提取偏航角"""
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


class AvoidanceNode(Node):
    def __init__(self):
        super().__init__('avoidance_node')
        # 声明参数
        self.declare_parameter('safety_distance', 0.5)
        self.declare_parameter('slow_down_distance', 1.2)
        self.declare_parameter('max_linear', 0.6)
        self.declare_parameter('max_angular', 1.0)
        self.declare_parameter('cruise_speed', 0.3)
        self.declare_parameter('enable_cruise', False)
        self.declare_parameter('num_sectors', 36)

        self.safety_distance = self.get_parameter('safety_distance').value
        self.slow_down_distance = self.get_parameter('slow_down_distance').value
        self.max_linear = self.get_parameter('max_linear').value
        self.max_angular = self.get_parameter('max_angular').value
        self.cruise_speed = self.get_parameter('cruise_speed').value
        self.enable_cruise = self.get_parameter('enable_cruise').value
        self.num_sectors = self.get_parameter('num_sectors').value

        self.obstacles = []
        self.pose = None          # odom 系下 (x, y, yaw)
        self.goal = None          # odom 系下 PoseStamped

        self.create_subscription(
            ObstacleArray, '/perception/obstacles', self.obstacles_cb, 10)
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.create_subscription(PoseStamped, '/goal_pose', self.goal_cb, 10)
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_service(SetGoal, '/avoidance/set_goal', self.set_goal_cb)

        # 10Hz 控制循环
        self.timer = self.create_timer(0.1, self.control_loop)

    # ---------- 回调 ----------
    def obstacles_cb(self, msg):
        self.obstacles = list(msg.obstacles)

    def odom_cb(self, msg):
        p = msg.pose.pose.position
        yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        self.pose = (p.x, p.y, yaw)

    def goal_cb(self, msg):
        self.goal = msg
        self.get_logger().info(
            f'收到新目标：x={msg.pose.position.x:.2f} y={msg.pose.position.y:.2f}')

    def set_goal_cb(self, request, response):
        self.goal = request.goal
        response.accepted = True
        response.message = '已接受目标点'
        self.get_logger().info('通过服务设置目标点')
        return response

    # ---------- 控制逻辑 ----------
    def control_loop(self):
        cmd = Twist()
        min_dist = self._min_obstacle_distance()

        # 期望方向：有目标朝目标，否则巡航（前方）或待命
        desired_angle = None
        if self.goal is not None and self.pose is not None:
            dx = self.goal.pose.position.x - self.pose[0]
            dy = self.goal.pose.position.y - self.pose[1]
            dist_goal = math.hypot(dx, dy)
            if dist_goal < 0.2:
                # 到达目标
                self.goal = None
                self.pub_cmd.publish(cmd)
                self.get_logger().info('已到达目标点')
                return
            goal_bearing = math.atan2(dy, dx)
            desired_angle = self._normalize_angle(goal_bearing - self.pose[2])
        elif self.enable_cruise:
            desired_angle = 0.0
        else:
            # 原地待命
            self.pub_cmd.publish(cmd)
            return

        # 目标方向在正后方等超出可通行扇区范围时，先原地转向
        if abs(desired_angle) > math.pi / 2.0:
            cmd.angular.z = self._clamp(
                1.5 * desired_angle, -self.max_angular, self.max_angular)
            self.pub_cmd.publish(cmd)
            return

        # 扇区避障：前方 180° 分扇区，选代价最低的可通过扇区
        best_angle = self._select_sector(desired_angle)
        if best_angle is None:
            # 全部扇区不可通行：原地转向（朝障碍较少的一侧）
            cmd.angular.z = self.max_angular if desired_angle >= 0 else -self.max_angular
            self.pub_cmd.publish(cmd)
            return

        # 线速度：随最近障碍距离减速
        if min_dist < self.safety_distance:
            linear = 0.0
        elif min_dist < self.slow_down_distance:
            ratio = (min_dist - self.safety_distance) / \
                (self.slow_down_distance - self.safety_distance)
            linear = self.max_linear * ratio
        else:
            linear = self.max_linear
        # 无目标巡航时用巡航速度封顶
        if self.goal is None:
            linear = min(linear, self.cruise_speed)

        # 角速度：朝选中扇区方向比例控制
        angular = self._clamp(1.5 * best_angle, -self.max_angular, self.max_angular)
        # 转角大时进一步减速
        linear *= max(0.0, 1.0 - abs(best_angle) / (math.pi / 2.0))

        cmd.linear.x = linear
        cmd.angular.z = angular
        self.pub_cmd.publish(cmd)

    def _min_obstacle_distance(self):
        if not self.obstacles:
            return float('inf')
        return min(o.distance for o in self.obstacles)

    def _select_sector(self, desired_angle):
        """前方 180° 扇区代价评估，返回最佳扇区中心角；全堵返回 None"""
        best_angle = None
        best_cost = float('inf')
        for k in range(self.num_sectors):
            # 扇区中心角：-90° ~ +90°
            sector_angle = -math.pi / 2.0 + \
                (k + 0.5) * math.pi / self.num_sectors
            # 扇区内最近障碍（考虑障碍角宽度）
            nearest = float('inf')
            for o in self.obstacles:
                half_width = math.atan2(o.radius, max(o.distance, 0.05))
                if abs(self._normalize_angle(o.angle - sector_angle)) <= half_width:
                    nearest = min(nearest, o.distance)
            if nearest < self.safety_distance:
                continue  # 不可通行
            # 代价 = 与目标方向偏差 + 障碍接近惩罚
            cost = abs(self._normalize_angle(sector_angle - desired_angle))
            if nearest < self.slow_down_distance:
                cost += (self.slow_down_distance - nearest)
            if cost < best_cost:
                best_cost = cost
                best_angle = sector_angle
        return best_angle

    @staticmethod
    def _normalize_angle(a):
        while a > math.pi:
            a -= 2.0 * math.pi
        while a < -math.pi:
            a += 2.0 * math.pi
        return a

    @staticmethod
    def _clamp(v, lo, hi):
        return max(lo, min(hi, v))


def main(args=None):
    rclpy.init(args=args)
    node = AvoidanceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
