"""避障节点（趋向目标 + 扇区避障 + 阿克曼绕行恢复，类 VFH 简化版）

订阅：
  /perception/obstacles (car_interfaces/ObstacleArray)
  /odom                 (nav_msgs/Odometry)
  /goal_pose            (geometry_msgs/PoseStamped)
发布：
  /cmd_vel (geometry_msgs/Twist)
服务：
  /avoidance/set_goal (car_interfaces/SetGoal)
参数：
  safety_distance    (float, 0.5)  扇区可通行阈值 m；小于该值降为蠕动速度
  hard_stop_distance (float, 0.30) 贴脸阈值 m，小于该值触发倒车脱困
  slow_down_distance (float, 1.2)  开始减速的距离
  max_linear         (float, 0.6)  最大线速度 m/s
  max_angular        (float, 1.0)  最大角速度 rad/s
  cruise_speed       (float, 0.3)  巡航速度 m/s
  turn_speed         (float, 0.25) 掉头/脱困时的前进速度 m/s（带速转向）
  creep_speed        (float, 0.15) 贴障蠕动速度 m/s（不停死，边挪边绕）
  reverse_speed      (float, 0.20) 倒车脱困速度 m/s
  recover_min_time   (float, 1.5)  倒车脱困最短持续时间 s（防抖动）
  detour_timeout     (float, 3.0)  绕行方向承诺保持时间 s（防左右摆动）
  enable_cruise      (bool,  False) 无目标时是否巡航
  num_sectors        (int,   36)    前方 180° 划分的扇区数

绕行策略（阿克曼不能原地自旋，全程保持 |v| > 0 或停车）：
  1. 正常：前方 180° 扇区选代价最低的可通行方向，速度随净空缩放；
  2. 贴障（< safety_distance）：不降为零，以 creep_speed 边挪边绕；
  3. 贴脸（< hard_stop_distance）或前方全堵：倒车-转向脱困（倒弧线把
     车头甩向较空一侧），前方净空且持续 recover_min_time 后恢复正常；
  4. 方向承诺：选定绕行侧后 detour_timeout 内换侧加代价，防止在障碍
     前来回摆动。前后都堵死时才真正停车等待。
"""

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import SetParametersResult

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
        self.declare_parameter('hard_stop_distance', 0.30)
        self.declare_parameter('slow_down_distance', 1.2)
        self.declare_parameter('max_linear', 0.6)
        self.declare_parameter('max_angular', 1.0)
        self.declare_parameter('cruise_speed', 0.3)
        self.declare_parameter('turn_speed', 0.25)
        self.declare_parameter('creep_speed', 0.15)
        self.declare_parameter('reverse_speed', 0.20)
        self.declare_parameter('recover_min_time', 1.5)
        self.declare_parameter('detour_timeout', 3.0)
        self.declare_parameter('enable_cruise', False)
        self.declare_parameter('num_sectors', 36)

        self.safety_distance = self.get_parameter('safety_distance').value
        self.hard_stop_distance = self.get_parameter('hard_stop_distance').value
        self.slow_down_distance = self.get_parameter('slow_down_distance').value
        self.max_linear = self.get_parameter('max_linear').value
        self.max_angular = self.get_parameter('max_angular').value
        self.cruise_speed = self.get_parameter('cruise_speed').value
        self.turn_speed = self.get_parameter('turn_speed').value
        self.creep_speed = self.get_parameter('creep_speed').value
        self.reverse_speed = self.get_parameter('reverse_speed').value
        self.recover_min_time = self.get_parameter('recover_min_time').value
        self.detour_timeout = self.get_parameter('detour_timeout').value
        self.enable_cruise = self.get_parameter('enable_cruise').value
        self.num_sectors = self.get_parameter('num_sectors').value

        self.obstacles = []
        self.pose = None          # odom 系下 (x, y, yaw)
        self.goal = None          # odom 系下 PoseStamped
        # 绕行状态
        self.recovering = False   # 倒车脱困中
        self.recover_start = 0.0
        self.detour_side = 0      # 绕行方向承诺：+1 左 -1 右 0 无
        self.detour_time = 0.0

        self.create_subscription(
            ObstacleArray, '/perception/obstacles', self.obstacles_cb, 10)
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.create_subscription(PoseStamped, '/goal_pose', self.goal_cb, 10)
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_service(SetGoal, '/avoidance/set_goal', self.set_goal_cb)
        # enable_cruise 支持运行时动态修改（网页巡航开关 / ros2 param set）
        self.add_on_set_parameters_callback(self.on_set_params)

        # 10Hz 控制循环
        self.timer = self.create_timer(0.1, self.control_loop)

    def on_set_params(self, params):
        for p in params:
            if p.name == 'enable_cruise' and p.type_ == p.Type.BOOL:
                self.enable_cruise = bool(p.value)
                self.get_logger().info(
                    f'定速巡航：{"开启" if self.enable_cruise else "关闭"}')
            elif p.name == 'cruise_speed' and p.type_ == p.Type.DOUBLE:
                self.cruise_speed = float(p.value)
        return SetParametersResult(successful=True)

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
        # 前向锥（±60°）内最近障碍：贴脸/蠕动/脱困判定只看正前方，
        # 绕行时侧面近距离掠过障碍属正常，不该触发倒车
        fwd_dist = self._forward_obstacle_distance()
        now = self.get_clock().now().nanoseconds * 1e-9

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

        # 目标方向在正后方等超出可通行扇区范围时，带速弧线掉头
        # （阿克曼底盘不能原地自旋，必须保持前进速度）
        if abs(desired_angle) > math.pi / 2.0:
            cmd.linear.x = self.turn_speed
            cmd.angular.z = self._clamp(
                1.5 * desired_angle, -self.max_angular, self.max_angular)
            self.pub_cmd.publish(cmd)
            return

        # 倒车脱困状态：前方净空且持续足够时间后退出
        if self.recovering:
            if fwd_dist > self.safety_distance and \
                    now - self.recover_start >= self.recover_min_time:
                self.recovering = False
                self.get_logger().info('脱困完成，恢复绕行')
            else:
                self._publish_recovery()
                return

        # 扇区避障：前方 180° 分扇区，选代价最低的可通过扇区
        best_angle = self._select_sector(desired_angle)

        # 触发倒车脱困：正前方障碍贴脸，或前方扇区全部不可通行
        if fwd_dist < self.hard_stop_distance or best_angle is None:
            if self._rear_clearance() > self.hard_stop_distance:
                self.recovering = True
                self.recover_start = now
                self.detour_side = self._freer_side()
                self.detour_time = now
                self.get_logger().info(
                    f'前方受阻，倒车脱困（绕行侧：{"左" if self.detour_side > 0 else "右"}）')
                self._publish_recovery()
            else:
                # 前后都堵死，只能停车等待
                self.pub_cmd.publish(cmd)
                self.get_logger().warn(
                    '前后均被堵死，停车等待', throttle_duration_sec=2.0)
            return

        # 绕行方向承诺：偏离目标方向时记录绕行侧；目标方向仍被挡时
        # 承诺不过期（防止绕到一半被目标方向拉回、在障碍前振荡）
        desired_blocked = \
            self._direction_distance(desired_angle) < self.safety_distance
        if self.detour_side and not desired_blocked and \
                now - self.detour_time > self.detour_timeout:
            self.detour_side = 0
        if abs(best_angle) > 0.15:
            side = 1 if best_angle > 0 else -1
            if self.detour_side == 0 or self.detour_side == side:
                self.detour_side = side
                self.detour_time = now
        elif abs(self._normalize_angle(best_angle - desired_angle)) < 0.1:
            self.detour_side = 0

        # 线速度：正前方贴障不停死，保持蠕动速度边挪边绕
        if fwd_dist < self.safety_distance:
            linear = self.creep_speed
        elif fwd_dist < self.slow_down_distance:
            ratio = (fwd_dist - self.safety_distance) / \
                (self.slow_down_distance - self.safety_distance)
            linear = max(self.creep_speed, self.max_linear * ratio)
        else:
            linear = self.max_linear
        # 无目标巡航时用巡航速度封顶
        if self.goal is None:
            linear = min(linear, self.cruise_speed)

        # 角速度：朝选中扇区方向比例控制
        angular = self._clamp(1.5 * best_angle, -self.max_angular, self.max_angular)
        # 转角大时减速（保留下限，阿克曼不能在转向时停死）
        linear *= max(0.3, 1.0 - abs(best_angle) / (math.pi / 2.0))

        cmd.linear.x = linear
        cmd.angular.z = angular
        self.pub_cmd.publish(cmd)

    def _publish_recovery(self):
        """倒车-转向脱困：倒弧线把车头甩向较空一侧。

        倒车时 v<0，底盘按 δ=atan(ωL/v) 自动处理转向符号，这里只需
        按期望的车头旋转方向给 ω。后方也堵时停车等待。
        """
        cmd = Twist()
        if self._rear_clearance() < self.hard_stop_distance:
            self.pub_cmd.publish(cmd)
            return
        cmd.linear.x = -self.reverse_speed
        cmd.angular.z = self.detour_side * self.max_angular
        self.pub_cmd.publish(cmd)

    def _forward_obstacle_distance(self, fov=math.pi / 3.0):
        """前向锥（±fov）内最近障碍距离"""
        fwd = float('inf')
        for o in self.obstacles:
            if abs(self._normalize_angle(o.angle)) <= fov:
                fwd = min(fwd, o.distance)
        return fwd

    def _direction_distance(self, direction):
        """某方向上被障碍角宽度覆盖的最近距离（用于判断该方向是否被挡）"""
        nearest = float('inf')
        for o in self.obstacles:
            half_width = math.atan2(o.radius, max(o.distance, 0.05))
            if abs(self._normalize_angle(o.angle - direction)) <= half_width:
                nearest = min(nearest, o.distance)
        return nearest

    def _rear_clearance(self):
        """后半平面（|angle| > 90°）最近障碍距离"""
        rear = float('inf')
        for o in self.obstacles:
            if abs(self._normalize_angle(o.angle)) > math.pi / 2.0:
                rear = min(rear, o.distance)
        return rear

    def _freer_side(self):
        """前半平面左/右两侧哪边更空：+1 左 -1 右（按各侧最近障碍比较）"""
        left = right = float('inf')
        for o in self.obstacles:
            a = self._normalize_angle(o.angle)
            if 0.0 <= a <= math.pi / 2.0:
                left = min(left, o.distance)
            elif -math.pi / 2.0 <= a < 0.0:
                right = min(right, o.distance)
        return 1 if left >= right else -1

    def _select_sector(self, desired_angle):
        """前方 180° 扇区代价评估，返回最佳扇区中心角；全堵返回 None"""
        # 期望方向本身可通行时直接采用：否则扇区中心量化（±π/72）会在
        # 无障碍时产生恒定小角速度，导致车辆持续向一侧缓慢偏转
        nearest_desired = float('inf')
        for o in self.obstacles:
            half_width = math.atan2(o.radius, max(o.distance, 0.05))
            if abs(self._normalize_angle(o.angle - desired_angle)) <= half_width:
                nearest_desired = min(nearest_desired, o.distance)
        if nearest_desired >= self.safety_distance:
            return desired_angle
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
            # 绕行方向承诺：换侧加代价，防止在障碍前左右摆动
            if self.detour_side and sector_angle * self.detour_side < 0:
                cost += 0.6
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
