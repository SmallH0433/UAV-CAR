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
  recover_exit_distance(float, 0.45) 脱困退出净空 m（独立于 safety_distance）
  recover_max_time   (float, 4.0)  脱困最长 s，超时强制退出防死循环
  detour_timeout     (float, 3.0)  绕行方向承诺保持时间 s（防左右摆动）
  enable_cruise      (bool,  False) 无目标时是否巡航
  num_sectors        (int,   36)    前方 180° 划分的扇区数
  vehicle_half_width (float, 0.20)  车体半宽 m（含余量）：障碍角宽度按
      障碍半径 + 车体半宽膨胀——否则前侧方障碍的角宽度盖不住正前方 0° 方向，
      中心线判定"可通行"，但车身前角会撞上（前侧方避障失效的根因）
  path_history_length (float, 5.0)  轨迹历史保留时长 s（用于死胡同记忆）
  blocked_direction_memory_s (float, 8.0)
      被阻方向记忆时长 s：触发脱困前 1s 的平均行进方向会被标记为"已碰壁"，
      后续选路时优先避开，防止在狭长死胡同里前后反复震荡
  blocked_direction_penalty  (float, 1.0) 被阻方向扇区附加代价
  blocked_direction_tolerance(float, 0.50) 被阻方向惩罚容差 rad（约 28°）
  lateral_response_enable (bool, True)  是否启用侧前方障碍提前响应
  lateral_bias_max        (float, 0.35) 侧前方障碍导致的期望方向最大偏移 rad
  lateral_clearance_margin(float, 0.30) 左右侧净空差超过该值才触发提前转向
  side_obstacle_penalty (float, 0.40) 侧前方障碍对同侧扇区的附加代价系数
  sector_lock_enable  (bool, True)  绕行状态下是否锁定扇区评估范围到当前
      绕行侧；锁定后只关注转向路线上的障碍，避免另一侧障碍干扰
  loop_detect_tolerance (float, 0.12) 两次脱困雷达签名平均净空差阈值 m，
      小于该值判定为原地往复
  escape_stop_time    (float, 1.0)  触发脱困路径前的停车时长 s
  escape_path_step    (float, 0.40) 脱困路径步长 m
  escape_path_points  (int,   6)    脱困路径点数上限
  escape_path_max_range (float, 2.0) 脱困路径总长的硬封顶 m
  escape_waypoint_tolerance (float, 0.15) 路径点到达判定半径 m
  escape_early_exit_distance (float, 0.80) 脱困路径行驶中前方净空
      超过该值立即退出脱困模式，恢复常规导航
  escape_path_length  (float, 1.50) 脱困路径目标长度 m（直线距离）
  escape_retry_reverse_s (float, 1.0) 脱困路径无法规划时，先沿原方向
      后退的时长 s，之后重新尝试规划

绕行策略（阿克曼不能原地自旋，全程保持 |v| > 0 或停车）：
  1. 正常：前方 180° 扇区选代价最低的可通行方向，速度随净空缩放；
  2. 贴障（< safety_distance）：不降为零，以 creep_speed 边挪边绕；
  3. 贴脸（< hard_stop_distance）或前方全堵：后方有真实净空
     （> recover_exit_distance）才倒车-转向脱困；后方也贴障（如斜挤
     进墙角）则进入锁存的死角蠕动脱困状态，朝最空扇区缓慢拱，
     避免倒车/蠕动在阈值抖动上来回切换形成极限环；
  4. 方向承诺：选定绕行侧后 detour_timeout 内换侧加代价，防止在障碍
     前来回摆动。蠕动方向也四面贴脸（真正被围死）才停车等待。
  5. 死胡同记忆：触发脱困时把最近 1s 的平均行进方向标记为"已碰壁"，
     后续 _select_sector / _clearest_direction 对接近该方向的扇区加
     代价，使车辆倾向于探索未走过的侧向出口，而不是在死胡同里
     前后反复震荡。记忆 8s 后自动过期，避免长期影响目标可达性。
  6. 侧前方提前响应：当某一侧侧前方净空明显小于另一侧时，期望方向
     自动向空旷侧偏移（最大 lateral_bias_max），避免到正前方被堵才急转，
     提升对正侧方/斜前方障碍物的响应能力。
  7. 绕行扇区锁定：选定绕行侧后只评估该侧扇区，只关注转向路线上的
     障碍，避免另一侧障碍干扰导致"能转却不敢转"；该侧全堵时自动
     解除锁定重新全向评估。
  8. 原地往复检测：每次倒车脱困结束时记录当前雷达签名（前方各扇区净空），
     若与上一次几乎一致（说明退回原处、在原地往复），立即停车
     escape_stop_time，再用当前扫描贪心规划一条局部脱困路径
     （escape_path_step 折线；路径终点距当前位置直线距离约
     escape_path_length，且不超过 escape_path_max_range 硬封顶，
     未看到的区域不能当作空旷）。若无法规划，先沿原方向后退
     escape_retry_reverse_s 后重新尝试。沿路径行驶中若前方净空超过
     escape_early_exit_distance 则立即退出、交回常规避障。
  9. 脱困状态高优先级：recovering / escaping / escape_pathing /
     escape_retrying 任一激活时，常规巡航与目标跟随被禁用，直到脱困
     完全结束或网页遥控接管。
  10. 网页遥控最高优先：/ugv/operator/heartbeat 活跃时立即取消包括
     自主脱困在内的全部自主状态并停车，底盘输出由 mux 切到遥控链路。
"""

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
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
        self.declare_parameter('recover_exit_distance', 0.45)  # 脱困退出净空 m
        self.declare_parameter('recover_max_time', 4.0)        # 脱困最长 s
        self.declare_parameter('detour_timeout', 3.0)
        self.declare_parameter('enable_cruise', False)
        self.declare_parameter('num_sectors', 36)
        self.declare_parameter('vehicle_half_width', 0.20)  # 车宽0.32/2=0.16+余量
        self.declare_parameter('path_history_length', 5.0)
        self.declare_parameter('blocked_direction_memory_s', 8.0)
        self.declare_parameter('blocked_direction_penalty', 1.0)
        self.declare_parameter('blocked_direction_tolerance', 0.50)
        self.declare_parameter('lateral_response_enable', True)
        self.declare_parameter('lateral_bias_max', 0.20)
        self.declare_parameter('lateral_clearance_margin', 0.50)
        self.declare_parameter('side_obstacle_penalty', 0.20)
        self.declare_parameter('sector_lock_enable', True)
        self.declare_parameter('loop_detect_tolerance', 0.12)
        self.declare_parameter('escape_stop_time', 1.0)
        self.declare_parameter('escape_path_step', 0.40)
        self.declare_parameter('escape_path_points', 6)
        self.declare_parameter('escape_path_max_range', 2.0)
        self.declare_parameter('escape_waypoint_tolerance', 0.15)
        self.declare_parameter('escape_early_exit_distance', 0.80)
        self.declare_parameter('escape_path_length', 1.50)
        self.declare_parameter('escape_retry_reverse_s', 1.0)

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
        self.recover_exit_distance = \
            self.get_parameter('recover_exit_distance').value
        self.recover_max_time = self.get_parameter('recover_max_time').value
        self.detour_timeout = self.get_parameter('detour_timeout').value
        self.enable_cruise = self.get_parameter('enable_cruise').value
        self.num_sectors = self.get_parameter('num_sectors').value
        self.vehicle_half_width = \
            self.get_parameter('vehicle_half_width').value
        self.path_history_length = \
            self.get_parameter('path_history_length').value
        self.blocked_direction_memory_s = \
            self.get_parameter('blocked_direction_memory_s').value
        self.blocked_direction_penalty = \
            self.get_parameter('blocked_direction_penalty').value
        self.blocked_direction_tolerance = \
            self.get_parameter('blocked_direction_tolerance').value
        self.lateral_response_enable = \
            self.get_parameter('lateral_response_enable').value
        self.lateral_bias_max = \
            self.get_parameter('lateral_bias_max').value
        self.lateral_clearance_margin = \
            self.get_parameter('lateral_clearance_margin').value
        self.side_obstacle_penalty = \
            self.get_parameter('side_obstacle_penalty').value
        self.sector_lock_enable = \
            self.get_parameter('sector_lock_enable').value
        self.loop_detect_tolerance = \
            self.get_parameter('loop_detect_tolerance').value
        self.escape_stop_time = self.get_parameter('escape_stop_time').value
        self.escape_path_step = self.get_parameter('escape_path_step').value
        self.escape_path_points = self.get_parameter('escape_path_points').value
        self.escape_path_max_range = \
            self.get_parameter('escape_path_max_range').value
        self.escape_waypoint_tolerance = \
            self.get_parameter('escape_waypoint_tolerance').value
        self.escape_early_exit_distance = \
            self.get_parameter('escape_early_exit_distance').value
        self.escape_path_length = \
            self.get_parameter('escape_path_length').value
        self.escape_retry_reverse_s = \
            self.get_parameter('escape_retry_reverse_s').value

        self.obstacles = []
        self.pose = None          # odom 系下 (x, y, yaw)
        self.goal = None          # odom 系下 PoseStamped
        # 绕行状态
        self.recovering = False   # 倒车脱困中
        self.recover_start = 0.0
        self.detour_side = 0      # 绕行方向承诺：+1 左 -1 右 0 无
        self.detour_time = 0.0
        # 死角蠕动脱困状态（锁存，防与倒车脱困在阈值抖动上往复切换）
        self.escaping = False
        self.escape_start = 0.0
        self.escape_angle = 0.0
        # 死胡同方向记忆：轨迹历史 + 被阻方向集合
        self.position_history = []   # [(x, y, yaw, time), ...]
        self.blocked_directions = []  # [(车体坐标系方向 rad, 过期时间), ...]
        # 原地往复检测 + 脱困路径状态
        self.recover_snapshots = []  # 最近两次倒车脱困完成时的雷达签名
        self.escape_pathing = False  # 脱困路径模式中
        self.escape_path = []        # odom 系路径点 [(x, y), ...]
        self.escape_idx = 0
        self.escape_stop_until = 0.0
        # 脱困路径无法规划时的后退重试状态
        self.escape_retrying = False
        self.escape_retry_until = 0.0
        # 网页遥控接管状态（/ugv/operator/heartbeat）
        self.operator_active = False
        self.operator_time = 0.0

        self.create_subscription(
            ObstacleArray, '/perception/obstacles', self.obstacles_cb, 10)
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.create_subscription(PoseStamped, '/goal_pose', self.goal_cb, 10)
        # 网页遥控心跳：活跃时遥控优先于一切自主逻辑
        self.create_subscription(
            Bool, '/ugv/operator/heartbeat', self.operator_cb, 10)
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

    def operator_cb(self, msg):
        """网页遥控心跳（web_gateway 10Hz 持续发布）：True=操作员活跃"""
        self.operator_active = bool(msg.data)
        self.operator_time = self.get_clock().now().nanoseconds * 1e-9

    def odom_cb(self, msg):
        p = msg.pose.pose.position
        yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        now = self.get_clock().now().nanoseconds * 1e-9
        self.pose = (p.x, p.y, yaw)
        # 保留最近 path_history_length 秒的 odom 轨迹，用于死胡同方向记忆
        self.position_history.append((p.x, p.y, yaw, now))
        cutoff = now - self.path_history_length
        while self.position_history and self.position_history[0][3] < cutoff:
            self.position_history.pop(0)

    def goal_cb(self, msg):
        self.goal = msg
        self.recover_snapshots = []  # 新目标=新场景，清空往复检测记忆
        self.get_logger().info(
            f'收到新目标：x={msg.pose.position.x:.2f} y={msg.pose.position.y:.2f}')

    def set_goal_cb(self, request, response):
        self.goal = request.goal
        self.recover_snapshots = []
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

        # 网页遥控优先于一切自主逻辑：操作员活跃（心跳 0.6s 内，与 mux 的
        # operator_timeout 一致）时立即取消包括自主脱困在内的全部自主状态
        # 并停车；底盘输出由 mux 切到遥控链路
        if self.operator_active and now - self.operator_time < 0.6:
            if self.recovering or self.escaping or self.escape_pathing:
                self.get_logger().info('网页遥控接管，取消全部自主脱困动作')
            self.recovering = False
            self.escaping = False
            self.escape_pathing = False
            self.escape_path = []
            self.pub_cmd.publish(cmd)
            return

        # 脱困路径无法规划时的后退重试：沿原方向后退 escape_retry_reverse_s，
        # 之后重新尝试规划脱困路径
        if self.escape_retrying:
            if now < self.escape_retry_until:
                cmd.linear.x = -self.reverse_speed
                cmd.angular.z = 0.0
                self.pub_cmd.publish(cmd)
                return
            self.escape_retrying = False
            if self._enter_escape_path_mode():
                self.pub_cmd.publish(cmd)
                return

        # 脱困路径模式（原地往复检测触发）：先停车 escape_stop_time，
        # 再沿规划路径逐个路径点前进（仍走扇区避障），走完交回常规避障
        desired_angle = None
        if self.escape_pathing:
            if self.pose is None:
                self.escape_pathing = False
            elif now < self.escape_stop_until:
                self.pub_cmd.publish(cmd)  # 停止一切动作
                return
            elif fwd_dist > self.escape_early_exit_distance:
                # 前方已较为空旷：立即退出脱困模式，恢复常规导航
                self.escape_pathing = False
                self.escape_path = []
                self.detour_side = 0  # 解除扇区锁定
                self.get_logger().info(
                    f'前方净空 {fwd_dist:.2f}m，提前退出脱困路径，'
                    f'恢复常规导航')
            else:
                while self.escape_idx < len(self.escape_path) and \
                        math.hypot(
                            self.escape_path[self.escape_idx][0] - self.pose[0],
                            self.escape_path[self.escape_idx][1] - self.pose[1]
                        ) < self.escape_waypoint_tolerance:
                    self.escape_idx += 1
                if self.escape_idx >= len(self.escape_path):
                    self.escape_pathing = False
                    self.escape_path = []
                    self.detour_side = 0  # 解除扇区锁定
                    self.get_logger().info('脱困路径走完，恢复常规避障')
                else:
                    wx, wy = self.escape_path[self.escape_idx]
                    desired_angle = self._normalize_angle(math.atan2(
                        wy - self.pose[1], wx - self.pose[0]) - self.pose[2])
                    # 脱困路径行驶时锁定扇区：只关注路径方向上的障碍，
                    # 避免另一侧障碍干扰导致"能转却不敢转"
                    if desired_angle > 0.1:
                        self.detour_side = 1
                    elif desired_angle < -0.1:
                        self.detour_side = -1

        # 脱困状态优先于一切目标跟随逻辑：倒车/蠕动脱困中车头会甩向
        # 空旷侧，往往偏离目标方向；若先做目标跟随（尤其 |desired|>90°
        # 的弧线掉头分支），脱困指令会被前进指令覆盖——转向轮刚反向就
        # 被拉回目标侧，前后换向失效、沿原路径往复
        # 倒车脱困状态：前方有基本净空且持续足够时间后退出；
        # 退出阈值独立于 safety_distance（实机：用 1.0m 安全距做退出条件，
        # 杂物多的环境永远达不到，车会一直倒车转圈"不回正"）；
        # 超时强制退出，杜绝无限倒车死循环
        if self.recovering:
            timed_out = now - self.recover_start >= self.recover_max_time
            if (fwd_dist > self.recover_exit_distance and
                    now - self.recover_start >= self.recover_min_time) \
                    or timed_out:
                self.recovering = False
                if timed_out:
                    self.get_logger().warn('脱困超时，强制恢复正常绕行')
                else:
                    self.get_logger().info('脱困完成，恢复绕行')
                # 完成后退时记录雷达签名；若与上一次几乎一致（原地往复），
                # 进入脱困路径模式并立即停车
                if self._record_recovery_snapshot():
                    self.pub_cmd.publish(cmd)
                    return
            else:
                self._publish_recovery()
                return

        # 死角蠕动脱困（锁存状态）：朝最空扇区缓慢拱，承诺方向被堵才重选；
        # 前方恢复基本净空后退出交回正常扇区流程，超时兜底退出
        if self.escaping:
            clearance = self._direction_distance(self.escape_angle)
            if clearance < self.hard_stop_distance:
                self.escape_angle, clearance = self._clearest_direction()
            if clearance <= self.hard_stop_distance + 0.05:
                self.escaping = False
                self.pub_cmd.publish(cmd)
                self.get_logger().warn(
                    '前后均被堵死，停车等待', throttle_duration_sec=2.0)
                return
            if now - self.escape_start >= self.recover_max_time * 2:
                self.escaping = False
                self.get_logger().warn('死角脱困超时，恢复正常流程')
            elif fwd_dist > self.recover_exit_distance and \
                    now - self.escape_start >= self.recover_min_time:
                self.escaping = False
                self.get_logger().info('死角脱困完成，恢复正常绕行')
            else:
                cmd.linear.x = self.creep_speed
                cmd.angular.z = self._clamp(
                    1.5 * self.escape_angle, -self.max_angular, self.max_angular)
                self.pub_cmd.publish(cmd)
                return

        # 期望方向：有目标朝目标，否则巡航（前方）或待命
        # 脱困状态（recovering / escaping / escape_pathing / escape_retrying）
        # 激活时禁用常规巡航与目标跟随，直到脱困完全结束或人工介入
        escape_active = (self.recovering or self.escaping or
                         self.escape_pathing or self.escape_retrying)
        if desired_angle is None:
            if escape_active:
                self.pub_cmd.publish(cmd)
                return
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
                desired_angle = \
                    self._normalize_angle(goal_bearing - self.pose[2])
            elif self.enable_cruise:
                desired_angle = 0.0
            else:
                # 原地待命
                self.pub_cmd.publish(cmd)
                return

        # 侧前方障碍提前响应：期望方向自动向空旷侧偏移
        if self.lateral_response_enable:
            desired_angle = self._bias_desired_angle(desired_angle)

        # 目标方向在正后方等超出可通行扇区范围时，带速弧线掉头
        # （阿克曼底盘不能原地自旋，必须保持前进速度）；
        # 仅正常导航生效，脱困状态已在上方优先处理
        if abs(desired_angle) > math.pi / 2.0:
            cmd.linear.x = self.turn_speed
            cmd.angular.z = self._clamp(
                1.5 * desired_angle, -self.max_angular, self.max_angular)
            self.pub_cmd.publish(cmd)
            return

        # 扇区避障：前方 180° 分扇区，选代价最低的可通过扇区
        best_angle = self._select_sector(desired_angle)
        if best_angle is None and self.sector_lock_enable and \
                self.detour_side != 0:
            # 当前绕行侧全堵，解除扇区锁定重新全向评估
            self.detour_side = 0
            self.get_logger().info('绕行侧扇区全堵，解除锁定重新评估')
            best_angle = self._select_sector(desired_angle)

        # 触发脱困：正前方障碍贴脸，或前方扇区全部不可通行。
        # 后方有真实净空（> recover_exit_distance）才倒车脱困；后方也贴障
        # 时进入锁存的死角蠕动脱困状态（阈值紧贴时两分支来回切换会形成
        # 极限环：蠕动 0.4s→倒车 4s 超时→再蠕动，永远困在墙角）
        if fwd_dist < self.hard_stop_distance or best_angle is None:
            # 先记录"触发脱困前的行进方向"为已碰壁方向，后续选路时优先避开，
            # 防止在狭长死胡同里前后反复震荡
            self._record_blocked_direction()
            if self._rear_clearance() > self.recover_exit_distance:
                self.recovering = True
                self.recover_start = now
                self.detour_side = self._freer_side()
                self.detour_time = now
                self.get_logger().info(
                    f'前方受阻，倒车脱困（绕行侧：{"左" if self.detour_side > 0 else "右"}）')
                self._publish_recovery()
            else:
                angle, clearance = self._clearest_direction()
                if clearance <= self.hard_stop_distance + 0.05:
                    # 四面贴脸，真正被围死
                    self.pub_cmd.publish(cmd)
                    self.get_logger().warn(
                        '前后均被堵死，停车等待', throttle_duration_sec=2.0)
                else:
                    self.escaping = True
                    self.escape_start = now
                    self.escape_angle = angle
                    self.get_logger().warn(
                        f'前后贴障，死角蠕动脱困（最空方向 '
                        f'{math.degrees(angle):.0f}°，净空 {clearance:.2f}m）')
                    cmd.linear.x = self.creep_speed
                    cmd.angular.z = self._clamp(
                        1.5 * angle, -self.max_angular, self.max_angular)
                    self.pub_cmd.publish(cmd)
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

    # ---------- 原地往复检测 + 脱困路径 ----------
    def _recovery_signature(self):
        """前方 180° 各扇区净空签名（截断 3m）：两次脱困结束时签名几乎
        一致，说明车退回了原处，正在原地往复。"""
        sig = []
        for k in range(self.num_sectors):
            a = -math.pi / 2.0 + (k + 0.5) * math.pi / self.num_sectors
            sig.append(min(self._direction_distance(a), 3.0))
        return sig

    def _record_recovery_snapshot(self):
        """倒车脱困完成时记录雷达签名；与上一次几乎一致则进入脱困路径
        模式（停车 + 规划 + 沿路径前进）。返回是否触发了脱困路径模式。"""
        sig = self._recovery_signature()
        triggered = False
        if self.recover_snapshots:
            prev = self.recover_snapshots[-1]
            mean_diff = sum(abs(a - b) for a, b in zip(sig, prev)) / len(sig)
            if mean_diff < self.loop_detect_tolerance:
                self.get_logger().warn(
                    f'两次脱困雷达签名几乎一致（平均差 {mean_diff:.3f}m < '
                    f'{self.loop_detect_tolerance:.2f}m），判定原地往复，'
                    f'停车并规划脱困路径')
                triggered = self._enter_escape_path_mode()
        self.recover_snapshots = (self.recover_snapshots + [sig])[-2:]
        return triggered

    def _enter_escape_path_mode(self):
        """停车 escape_stop_time 并生成脱困路径；无法规划时进入后退重试状态。

        路径终点距当前位置直线距离约 escape_path_length（默认 1.5m），
        按步长折算点数。若当前环境无法规划出可行路径，先沿原方向后退
        escape_retry_reverse_s，之后重新尝试规划。
        """
        if self.pose is None:
            return False
        # 路径长度目标 escape_path_length，按步长折算点数；同时不超过
        # 当前雷达实际感知范围（未看到的区域不能视为空旷）和硬封顶
        if self.obstacles:
            seen_range = max(o.distance for o in self.obstacles)
        else:
            seen_range = self.escape_path_max_range
        target_length = min(self.escape_path_length, seen_range,
                            self.escape_path_max_range)
        points = max(2, int(math.ceil(target_length / self.escape_path_step)))
        path = self._plan_escape_path(points)
        if not path:
            # 无法规划：沿原方向后退 escape_retry_reverse_s 后重新规划
            now = self.get_clock().now().nanoseconds * 1e-9
            self.escape_retrying = True
            self.escape_retry_until = now + self.escape_retry_reverse_s
            self.get_logger().warn(
                f'脱困路径无法规划，沿原方向后退 '
                f'{self.escape_retry_reverse_s:.1f}s 后重新规划')
            return True
        now = self.get_clock().now().nanoseconds * 1e-9
        self.escape_path = path
        self.escape_idx = 0
        self.escape_pathing = True
        self.escape_stop_until = now + self.escape_stop_time
        self.recovering = False
        self.escaping = False
        self.detour_side = 0
        self.recover_snapshots = []  # 清空记忆，避免路径跟随中连锁触发
        end = path[-1]
        self.get_logger().warn(
            f'脱困路径已规划（{len(path)} 点，终点 ({end[0]:.2f}, '
            f'{end[1]:.2f})），停车 {self.escape_stop_time:.1f}s 后沿路径前进')
        return True

    def _plan_escape_path(self, num_points):
        """用当前雷达快照贪心规划局部脱困路径（odom 系折线）。

        每步在当前段朝向的前方 180° 内，沿候选方向的一步线段采样 3 点
        取最小净空（障碍按 半径+车体半宽 膨胀），净空 < 5cm 的方向不可行；
        可行方向中选净空最大者，相同偏直走。全部不可行（被围死）时退而
        选"最不差"方向。只依赖当前扫描，目标是走出困住车辆的局部区域，
        随后交回常规避障。
        """
        x, y, yaw = self.pose
        # 障碍圆盘转到 odom 系
        discs = []
        for o in self.obstacles:
            a = self._normalize_angle(o.angle)
            discs.append((x + o.distance * math.cos(yaw + a),
                          y + o.distance * math.sin(yaw + a),
                          o.radius + self.vehicle_half_width))
        margin = 0.05
        vx, vy, heading = x, y, yaw
        path = []
        for i in range(num_points):
            best_score, best_dir = -float('inf'), heading
            fb_clear, fb_dir = -float('inf'), heading  # 全不可行时的兜底
            for k in range(self.num_sectors):
                rel = -math.pi / 2.0 + \
                    (k + 0.5) * math.pi / self.num_sectors
                d = heading + rel
                min_clear = float('inf')
                for t in (0.33, 0.67, 1.0):
                    px = vx + self.escape_path_step * t * math.cos(d)
                    py = vy + self.escape_path_step * t * math.sin(d)
                    for ox, oy, r in discs:
                        min_clear = min(
                            min_clear, math.hypot(px - ox, py - oy) - r)
                if min_clear > fb_clear:
                    fb_clear, fb_dir = min_clear, d
                if min_clear < margin:
                    continue  # 会撞上膨胀障碍，不可行
                score = min(min_clear, 2.0) - 0.5 * abs(rel)  # 偏直走
                if score > best_score:
                    best_score, best_dir = score, d
            if best_score == -float('inf'):
                if i == 0:
                    return None  # 第一步就无法规划
                break  # 后续点无法规划，返回已有路径
            heading = best_dir
            vx += self.escape_path_step * math.cos(heading)
            vy += self.escape_path_step * math.sin(heading)
            path.append((vx, vy))
        return path if path else None

    def _record_blocked_direction(self):
        """把触发脱困前 1s 内的平均行进方向标记为"已碰壁"。

        方向换算到车体坐标系，保证车辆转向后记忆仍有效。
        记忆会在 blocked_direction_memory_s 秒后自动过期。
        """
        now = self.get_clock().now().nanoseconds * 1e-9
        if len(self.position_history) < 2:
            return
        recent = [pt for pt in self.position_history
                  if now - pt[3] <= 1.0]
        if len(recent) < 2:
            recent = self.position_history[-2:]
        start = recent[0]
        end = recent[-1]
        dx = end[0] - start[0]
        dy = end[1] - start[1]
        if math.hypot(dx, dy) < 0.01:
            # 位移太小，直接用车头朝向作为被阻方向
            direction_body = 0.0
        else:
            travel_yaw = math.atan2(dy, dx)
            direction_body = self._normalize_angle(travel_yaw - start[2])
        expiration = now + self.blocked_direction_memory_s
        self.blocked_directions.append((direction_body, expiration))
        self.get_logger().info(
            f'记录被阻方向 {math.degrees(direction_body):.0f}°，'
            f'记忆 {self.blocked_direction_memory_s:.1f}s')

    def _blocked_direction_cost(self, sector_angle):
        """返回 sector_angle 相对被阻方向记忆的额外代价（自动清理过期项）。"""
        now = self.get_clock().now().nanoseconds * 1e-9
        self.blocked_directions = [
            (d, exp) for d, exp in self.blocked_directions if exp > now
        ]
        cost = 0.0
        for direction, _ in self.blocked_directions:
            diff = abs(self._normalize_angle(sector_angle - direction))
            if diff < self.blocked_direction_tolerance:
                scale = 1.0 - diff / self.blocked_direction_tolerance
                cost += self.blocked_direction_penalty * scale
        return cost

    def _bias_desired_angle(self, desired_angle):
        """根据侧前方障碍分布把期望方向向空旷侧偏移。

        只在 0.8m 范围内的侧前方（±15° ~ ±90°）障碍参与计算；
        左右净空差超过 lateral_clearance_margin 才触发偏移，最大偏移
        lateral_bias_max。让小车在正前方还没被堵死时就提前向空旷侧转向，
        提升对侧前方/斜前方障碍物的响应能力。
        """
        response_range = 0.8  # 只响应 0.8m 内的侧前方障碍，避免过于灵敏
        left_clearance = right_clearance = float('inf')
        for o in self.obstacles:
            a = self._normalize_angle(o.angle)
            if 0.26 < a < math.pi / 2.0 and \
                    o.distance < response_range:
                left_clearance = min(left_clearance, o.distance)
            elif -math.pi / 2.0 < a < -0.26 and \
                    o.distance < response_range:
                right_clearance = min(right_clearance, o.distance)
        if left_clearance == float('inf') and right_clearance == float('inf'):
            return desired_angle
        if left_clearance == float('inf'):
            # 左侧无障碍而右侧有：向左侧空旷侧偏移
            return desired_angle + self.lateral_bias_max
        if right_clearance == float('inf'):
            # 右侧无障碍而左侧有：向右侧空旷侧偏移
            return desired_angle - self.lateral_bias_max
        diff = left_clearance - right_clearance
        if abs(diff) < self.lateral_clearance_margin:
            return desired_angle
        # diff > 0：左侧更空，向左偏移（desired_angle 增大）
        bias = self._clamp(
            diff * 0.15, -self.lateral_bias_max, self.lateral_bias_max)
        return self._clamp(
            desired_angle + bias, -math.pi / 2.0, math.pi / 2.0)

    def _side_obstacle_cost(self, sector_angle):
        """侧前方障碍对同侧扇区的附加代价，促使提前向对侧绕行。"""
        cost = 0.0
        for o in self.obstacles:
            a = self._normalize_angle(o.angle)
            if 0.26 < abs(a) < math.pi / 2.0 and \
                    o.distance < self.slow_down_distance:
                if sector_angle * a > 0:
                    proximity = (self.slow_down_distance - o.distance) / \
                        self.slow_down_distance
                    cost += self.side_obstacle_penalty * proximity * \
                        math.cos(abs(a))
        return cost

    def _inflated_half_width(self, o):
        """障碍角半宽（rad）：障碍半径 + 车体半宽 膨胀后的角宽度。

        把车当质点会导致前侧方障碍的角宽度盖不住 0° 方向而被判"可通行"，
        但车身前角实际会撞上；膨胀后中心线判定即代表整车走廊。
        """
        return math.atan2(
            o.radius + self.vehicle_half_width, max(o.distance, 0.05))

    def _forward_obstacle_distance(self, fov=math.pi / 3.0):
        """前向锥（±fov）内最近障碍距离；膨胀后伸进正前走廊的侧方障碍也算"""
        fwd = float('inf')
        for o in self.obstacles:
            a = abs(self._normalize_angle(o.angle))
            if a <= fov or a <= self._inflated_half_width(o):
                fwd = min(fwd, o.distance)
        return fwd

    def _direction_distance(self, direction):
        """某方向上被障碍角宽度（含车体膨胀）覆盖的最近距离"""
        nearest = float('inf')
        for o in self.obstacles:
            half_width = self._inflated_half_width(o)
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

    def _clearest_direction(self):
        """前方 180° 内净空最大的扇区 (中心角, 净空距离)，用于死角蠕动脱困。

        选择时叠加被阻方向惩罚，使死胡同内蠕动不会反复拱向已经碰壁的方向。
        """
        best_angle, best_clearance, best_score = 0.0, -1.0, -float('inf')
        for k in range(self.num_sectors):
            sector_angle = -math.pi / 2.0 + \
                (k + 0.5) * math.pi / self.num_sectors
            nearest = float('inf')
            for o in self.obstacles:
                half_width = self._inflated_half_width(o)
                if abs(self._normalize_angle(o.angle - sector_angle)) <= half_width:
                    nearest = min(nearest, o.distance)
            score = nearest - self._blocked_direction_cost(sector_angle)
            if score > best_score:
                best_score = score
                best_clearance = nearest
                best_angle = sector_angle
        return best_angle, best_clearance

    def _select_sector(self, desired_angle):
        """前方 180° 扇区代价评估，返回最佳扇区中心角；全堵返回 None"""
        # 期望方向本身可通行时直接采用：否则扇区中心量化（±π/72）会在
        # 无障碍时产生恒定小角速度，导致车辆持续向一侧缓慢偏转
        # （角宽度均按 障碍半径+车体半宽 膨胀，见 _inflated_half_width）
        nearest_desired = float('inf')
        for o in self.obstacles:
            half_width = self._inflated_half_width(o)
            if abs(self._normalize_angle(o.angle - desired_angle)) <= half_width:
                nearest_desired = min(nearest_desired, o.distance)
        if nearest_desired >= self.safety_distance:
            # 期望方向畅通且未被标记为"已碰壁"时直接采用；否则继续
            # 走扇区评估，让死胡同记忆有机会把车辆推离反复震荡的方向
            if self._blocked_direction_cost(desired_angle) < 0.05:
                return desired_angle
        best_angle = None
        best_cost = float('inf')
        # 绕行状态下只评估绕行方向一侧的扇区：倒车脱困/绕行时只关注
        # 当前转向路线上的障碍，避免另一侧障碍干扰导致"能转却不敢转"
        sector_range = range(self.num_sectors)
        if self.sector_lock_enable and self.detour_side != 0:
            half = self.num_sectors // 2
            if self.detour_side > 0:
                sector_range = range(half, self.num_sectors)
            else:
                sector_range = range(0, half)
        for k in sector_range:
            # 扇区中心角：-90° ~ +90°
            sector_angle = -math.pi / 2.0 + \
                (k + 0.5) * math.pi / self.num_sectors
            # 扇区内最近障碍（考虑障碍角宽度）
            nearest = float('inf')
            for o in self.obstacles:
                half_width = self._inflated_half_width(o)
                if abs(self._normalize_angle(o.angle - sector_angle)) <= half_width:
                    nearest = min(nearest, o.distance)
            if nearest < self.safety_distance:
                continue  # 不可通行
            # 代价 = 与目标方向偏差 + 障碍接近惩罚 + 死胡同被阻方向惩罚
            cost = abs(self._normalize_angle(sector_angle - desired_angle))
            if nearest < self.slow_down_distance:
                cost += (self.slow_down_distance - nearest)
            cost += self._blocked_direction_cost(sector_angle)
            cost += self._side_obstacle_cost(sector_angle)
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
