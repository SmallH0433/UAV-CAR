"""avoidance_node 扇区避障的车体宽度膨胀测试。

根因场景：前侧方障碍（如 -25°/0.7m 的小障碍）角宽度按障碍半径算盖不住
正前方 0° 方向，中心线判"可通行"，但车身前角会撞上——车宽 0.32m，
判定必须按 障碍半径 + 车体半宽(vehicle_half_width) 膨胀。
"""

import math

import pytest
import rclpy

from car_nodes.avoidance_node import AvoidanceNode


class FakeObstacle:
    """car_interfaces/Obstacle 的等价替身（避免在单测里依赖消息生成）。"""

    def __init__(self, angle_deg, distance, radius):
        self.angle = math.radians(angle_deg)
        self.distance = distance
        self.radius = radius
        self.velocity = 0.0
        self.source = 0


@pytest.fixture(scope="module")
def node():
    rclpy.init()
    node = AvoidanceNode()
    node.safety_distance = 1.0      # 与 launch 实机调优值一致
    node.slow_down_distance = 1.8
    yield node
    node.destroy_node()
    rclpy.shutdown()


def test_clear_path_returns_desired_direction(node):
    node.obstacles = []
    assert node._select_sector(0.0) == 0.0


def test_distant_obstacle_does_not_block(node):
    # 远处小障碍（5m）角宽度膨胀后仍盖不到 0°，不应影响直行
    node.obstacles = [FakeObstacle(-25.0, 5.0, 0.15)]
    assert node._select_sector(0.0) == 0.0


def test_front_side_obstacle_blocks_straight_and_detours_left(node):
    # 前侧方小障碍：-25°/0.7m/r=0.15——未膨胀时角半宽 12° 盖不住 0°，
    # 中心线判可通行直行撞前角；膨胀后（r+0.20）角半宽 26.6° 必须封堵
    node.obstacles = [FakeObstacle(-25.0, 0.7, 0.15)]
    assert node._direction_distance(0.0) == pytest.approx(0.7)
    best = node._select_sector(0.0)
    assert best is not None
    assert best > math.radians(2.0)  # 障碍在右侧，应向左绕行


def test_true_side_obstacle_stays_out_of_forward_cone(node):
    # 正侧方障碍（-80°/2m）：绕行时侧面掠过属正常，不应算前向受阻
    node.obstacles = [FakeObstacle(-80.0, 2.0, 0.15)]
    assert node._forward_obstacle_distance() == float('inf')


def test_inflated_span_reaching_zero_counts_as_forward(node):
    # 超出 ±60° 锥、但膨胀后伸进正前走廊的近障（-70°/0.2m 大障碍）要算前向
    node.obstacles = [FakeObstacle(-70.0, 0.2, 0.4)]
    assert node._forward_obstacle_distance() == pytest.approx(0.2)
    # 同样障碍放远到 1m 就不该算
    node.obstacles = [FakeObstacle(-70.0, 1.0, 0.4)]
    assert node._forward_obstacle_distance() == float('inf')


class CmdRecorder:
    """替换 pub_cmd 记录 control_loop 发布的指令。"""

    def __init__(self):
        self.last = None

    def publish(self, msg):
        self.last = msg


def _run_control_loop(node, obstacles):
    node.obstacles = obstacles
    node.pose = (0.0, 0.0, 0.0)
    node.goal = None
    node.enable_cruise = True
    node.recovering = False
    node.escaping = False
    node.detour_side = 0
    recorder = CmdRecorder()
    real_pub = node.pub_cmd
    node.pub_cmd = recorder
    try:
        node.control_loop()
    finally:
        node.pub_cmd = real_pub
        node.enable_cruise = False
    return recorder.last


def test_wedged_corner_creeps_toward_open_side(node):
    # 实机/仿真卡点：斜挤进墙角，车头左前（-10.5°/0.25m）与左后
    # （118.5°/0.25m）都贴墙，右侧空——旧逻辑"前后堵死停车等待"困死；
    # 修复后应朝最空的右侧蠕动脱困
    cmd = _run_control_loop(node, [
        FakeObstacle(-10.5, 0.25, 0.14),
        FakeObstacle(118.5, 0.25, 0.27),
    ])
    assert cmd is not None
    assert cmd.linear.x > 0.0    # 不是停车等待
    assert cmd.angular.z < 0.0   # 朝空旷的右侧转


def test_fully_surrounded_still_stops(node):
    # 四面八方 0.2m 全贴障：蠕动方向也不比贴脸更空，才停车等待
    cmd = _run_control_loop(node, [
        FakeObstacle(a, 0.2, 0.1) for a in range(-180, 180, 30)
    ])
    assert cmd is not None
    assert cmd.linear.x == 0.0
    assert cmd.angular.z == 0.0


# ---------- 死胡同方向记忆测试 ----------

def test_blocked_direction_cost_penalises_recently_blocked(node):
    node.blocked_directions = [(0.0, float('inf'))]  # 正前刚碰壁，不过期
    cost_front = node._blocked_direction_cost(0.0)
    cost_side = node._blocked_direction_cost(math.radians(45.0))
    assert cost_front > cost_side
    assert cost_front > 0.0


def test_blocked_direction_cost_expires(node):
    now = node.get_clock().now().nanoseconds * 1e-9
    node.blocked_directions = [(0.0, now - 0.1)]  # 已过期
    assert node._blocked_direction_cost(0.0) == 0.0


def test_select_sector_avoids_blocked_direction(node):
    # 正前障碍 1.5m（> safety_distance 1.0），不记忆时 _select_sector 会直行；
    # 记录正前为被阻方向后，应偏向侧方，不再直冲已碰壁的方向
    node.blocked_directions = [(0.0, float('inf'))]
    node.obstacles = [FakeObstacle(0.0, 1.5, 0.15)]
    best = node._select_sector(0.0)
    assert best is not None
    assert abs(best) > math.radians(5.0)


def test_clearest_direction_prefers_unblocked_side(node):
    # 左右净空对称，仅正前被标记为已碰壁：最空方向应偏离正前
    node.blocked_directions = [(0.0, float('inf'))]
    node.obstacles = [
        FakeObstacle(-45.0, 2.0, 0.15),
        FakeObstacle(45.0, 2.0, 0.15),
    ]
    angle, clearance = node._clearest_direction()
    assert abs(angle) > math.radians(5.0)


def test_dead_end_escapes_sideways_not_front_back(node):
    # 死胡同：正前、正后都贴障，左侧空旷；记录正前为被阻方向后，
    # 车辆应选择左侧出口，而不是再次尝试前后震荡
    node.blocked_directions = [(0.0, float('inf'))]
    cmd = _run_control_loop(node, [
        FakeObstacle(0.0, 0.25, 0.15),
        FakeObstacle(175.0, 0.25, 0.15),
        FakeObstacle(-60.0, 2.0, 0.15),   # 左侧空
        FakeObstacle(60.0, 0.25, 0.15),  # 右侧堵
    ])
    assert cmd is not None
    assert cmd.linear.x > 0.0
    assert cmd.angular.z < 0.0  # 朝左侧转


# ---------- 侧前方障碍物响应测试 ----------

def test_bias_desired_angle_toward_clear_side(node):
    # 左侧近距离障碍，右侧空旷：期望方向应向左偏移
    node.obstacles = [
        FakeObstacle(-45.0, 0.4, 0.15),
        FakeObstacle(45.0, 2.0, 0.15),
    ]
    biased = node._bias_desired_angle(0.0)
    assert biased > math.radians(5.0)


def test_bias_desired_angle_no_obstacles_unchanged(node):
    node.obstacles = []
    assert node._bias_desired_angle(0.0) == 0.0


def test_side_obstacle_cost_penalises_same_side(node):
    node.obstacles = [FakeObstacle(-45.0, 0.4, 0.15)]
    cost_left = node._side_obstacle_cost(math.radians(-45.0))
    cost_right = node._side_obstacle_cost(math.radians(45.0))
    assert cost_left > cost_right


def test_select_sector_avoids_side_obstacle_side(node):
    # 正前空旷但左侧近距离障碍：应提前向右侧绕行
    node.obstacles = [
        FakeObstacle(0.0, 2.0, 0.15),
        FakeObstacle(-45.0, 0.4, 0.15),
    ]
    best = node._select_sector(0.0)
    assert best is not None
    assert best > math.radians(5.0)


# ---------- 脱困优先级测试（回归：脱困不得被弧线掉头分支抢占） ----------

class FakeGoal:
    """geometry_msgs/PoseStamped 的等价替身（仅 control_loop 用到的字段）。"""

    class pose:
        class position:
            x = -1.0   # 目标在正后方 → desired_angle ≈ ±180° > 90°
            y = 0.0


def _run_recovery_loop(node, obstacles):
    """脱困状态 + 目标在正后方 下跑一次 control_loop，返回发布的指令。"""
    node.obstacles = obstacles
    node.pose = (0.0, 0.0, 0.0)
    node.goal = FakeGoal()
    node.enable_cruise = False
    now = node.get_clock().now().nanoseconds * 1e-9
    node.recover_start = now   # 刚进脱困，远未到 recover_min_time
    node.escape_start = now
    node.detour_side = 1
    recorder = CmdRecorder()
    real_pub = node.pub_cmd
    node.pub_cmd = recorder
    try:
        node.control_loop()
    finally:
        node.pub_cmd = real_pub
        node.goal = None
        node.recovering = False
        node.escaping = False
        node.detour_side = 0
    return recorder.last


def test_recovery_not_preempted_by_turnaround(node):
    # 回归：倒车脱困中车头甩向空旷侧、目标落到后方（|desired|>90°）时，
    # 弧线掉头分支不得抢占——否则倒车指令被前进掉头覆盖，转向轮刚反向
    # 就回到原位，车沿同一弧线前后往复
    node.recovering = True
    node.escaping = False
    # 正前贴脸障碍（触发倒车的那堵墙），后方空旷
    cmd = _run_recovery_loop(node, [FakeObstacle(0.0, 0.25, 0.15)])
    assert cmd is not None
    assert cmd.linear.x < 0.0   # 必须仍是倒车，而不是 turn_speed 前进掉头
    assert cmd.angular.z == pytest.approx(node.max_angular)  # detour_side=+1


def test_escaping_not_preempted_by_turnaround(node):
    # 同理：死角蠕动脱困中目标在后方时，也不得被掉头分支抢占
    node.recovering = False
    node.escaping = True
    node.escape_angle = math.radians(80.0)  # 承诺方向：右侧（无障碍）
    cmd = _run_recovery_loop(node, [FakeObstacle(0.0, 0.25, 0.15)])
    assert cmd is not None
    assert cmd.linear.x == pytest.approx(node.creep_speed)  # 蠕动而非掉头


# ---------- 原地往复检测 + 脱困路径测试 ----------

def _run(node, obstacles):
    """巡航模式（无目标）下跑一次 control_loop，返回发布的指令。"""
    node.obstacles = obstacles
    node.pose = (0.0, 0.0, 0.0)
    node.goal = None
    node.enable_cruise = True
    recorder = CmdRecorder()
    real_pub = node.pub_cmd
    node.pub_cmd = recorder
    try:
        node.control_loop()
    finally:
        node.pub_cmd = real_pub
        node.enable_cruise = False
    return recorder.last


def _reset_escape_state(node):
    node.escape_pathing = False
    node.escape_path = []
    node.escape_idx = 0
    node.escape_stop_until = 0.0
    node.recover_snapshots = []
    node.recovering = False
    node.escaping = False
    node.operator_active = False
    node.detour_side = 0
    node.escape_retrying = False


def test_repeated_identical_scan_triggers_escape_path(node):
    # 两次倒车脱困完成时雷达画面几乎一致 → 判定原地往复：
    # 进入脱困路径模式并立即停车
    _reset_escape_state(node)
    obs = [FakeObstacle(-20.0, 1.0, 0.15), FakeObstacle(30.0, 1.2, 0.2)]
    now = node.get_clock().now().nanoseconds * 1e-9
    # 第一次脱困完成：只记录签名，不触发
    node.recovering = True
    node.recover_start = now - 2.0  # 已超过 recover_min_time
    node.detour_side = 1
    _run(node, obs)
    assert node.recovering is False
    assert not node.escape_pathing
    # 第二次脱困完成，雷达签名几乎一致 → 触发
    node.recovering = True
    node.recover_start = now - 2.0
    cmd = _run(node, obs)
    assert node.escape_pathing
    assert cmd.linear.x == 0.0 and cmd.angular.z == 0.0  # 停止一切动作
    # 路径点数按 escape_path_length（默认 1.5m）与当前雷达范围取小后
    # 除以步长向上取整
    expected = max(2, int(math.ceil(min(node.escape_path_length, 1.2) /
                                    node.escape_path_step)))
    assert len(node.escape_path) == expected
    _reset_escape_state(node)


def test_different_scans_do_not_trigger(node):
    # 两次签名差异大（环境明显变化）→ 不触发
    _reset_escape_state(node)
    now = node.get_clock().now().nanoseconds * 1e-9
    node.recovering = True
    node.recover_start = now - 2.0
    node.detour_side = 1
    _run(node, [FakeObstacle(0.0, 1.0, 0.15)])
    node.recovering = True
    node.recover_start = now - 2.0
    _run(node, [FakeObstacle(80.0, 0.6, 0.15)])  # 完全不同的画面
    assert not node.escape_pathing
    _reset_escape_state(node)


def test_escape_path_stop_then_follow(node):
    # 停车阶段发零指令；停车结束后沿路径点前进并朝路径点转向
    # （前方 0.6m 有障碍，低于提前退出阈值，不会被 early-exit 抢占）
    _reset_escape_state(node)
    obs = [FakeObstacle(0.0, 0.6, 0.15)]
    node.escape_path = [(0.4, 0.3), (0.8, 0.6)]
    node.escape_pathing = True
    node.escape_stop_until = \
        node.get_clock().now().nanoseconds * 1e-9 + 10.0
    cmd = _run(node, obs)
    assert cmd.linear.x == 0.0 and cmd.angular.z == 0.0
    node.escape_stop_until = 0.0  # 停车结束
    cmd = _run(node, obs)
    assert cmd.linear.x > 0.0
    assert cmd.angular.z > 0.0   # 第一个路径点在左前方
    _reset_escape_state(node)


def test_escape_path_completion_resumes_normal(node):
    # 到达路径终点（已在到达半径内）→ 退出脱困路径模式，恢复常规巡航
    # （前方 0.6m 障碍挡住 early-exit，走真正的路径点完成分支）
    _reset_escape_state(node)
    node.escape_path = [(0.05, 0.05)]
    node.escape_pathing = True
    cmd = _run(node, [FakeObstacle(0.0, 0.6, 0.15)])
    assert node.escape_pathing is False
    assert cmd.linear.x > 0.0  # 常规巡航/蠕动前进


def test_escape_path_early_exit_when_clear(node):
    # 脱困路径行驶中前方已较为空旷 → 立即退出脱困模式，恢复常规导航
    _reset_escape_state(node)
    node.escape_path = [(0.4, 0.3), (0.8, 0.6)]
    node.escape_pathing = True
    node.escape_stop_until = 0.0  # 停车阶段已结束
    cmd = _run(node, [])  # 前方空旷（无障碍）
    assert node.escape_pathing is False
    assert node.escape_path == []
    assert cmd.linear.x > 0.0


def test_escape_path_limited_to_lidar_range(node):
    # 脱困路线不得超过当前雷达范围：最远障碍 1.0m 时路径点数按
    # ceil(1.0 / step) 折算
    _reset_escape_state(node)
    node.pose = (0.0, 0.0, 0.0)
    node.obstacles = [FakeObstacle(0.0, 1.0, 0.15)]
    assert node._enter_escape_path_mode()
    assert len(node.escape_path) == max(
        2, int(math.ceil(min(node.escape_path_length, 1.0) /
                         node.escape_path_step)))
    # 无障碍时按 escape_path_length 折算
    node.obstacles = []
    assert node._enter_escape_path_mode()
    expected = max(2, int(math.ceil(node.escape_path_length /
                                    node.escape_path_step)))
    assert len(node.escape_path) == expected
    _reset_escape_state(node)


def test_operator_override_cancels_all_autonomy(node):
    # 网页遥控最高优先：操作员活跃时立即取消脱困/倒车等一切自主状态并停车
    _reset_escape_state(node)
    node.escape_path = [(0.4, 0.3), (0.8, 0.6)]
    node.escape_pathing = True
    node.recovering = True
    node.operator_active = True
    node.operator_time = node.get_clock().now().nanoseconds * 1e-9
    cmd = _run(node, [])
    assert cmd.linear.x == 0.0 and cmd.angular.z == 0.0
    assert node.escape_pathing is False
    assert node.recovering is False
    assert node.escape_path == []
    node.operator_active = False


def test_plan_escape_path_avoids_obstacle(node):
    # 正前方障碍：规划的路径必须绕开，而不是直线穿过去
    _reset_escape_state(node)
    node.pose = (0.0, 0.0, 0.0)
    node.obstacles = [FakeObstacle(0.0, 0.8, 0.2)]
    path = node._plan_escape_path(node.escape_path_points)
    assert len(path) == node.escape_path_points
    assert any(abs(py) > 0.05 for _, py in path)  # 明显偏离直线
    # 路径点不得进入障碍膨胀圈（半径 0.2 + 车半宽 0.2）
    for px, py in path:
        assert math.hypot(px - 0.8, py - 0.0) >= 0.4 - 1e-6
    node.obstacles = []


# ---------- 绕行方向扇区锁定测试 ----------

def test_sector_lock_left_only_evaluates_left(node):
    # 绕行侧为左：即使右侧更空旷，也只评估左侧扇区
    node.detour_side = 1
    node.obstacles = [
        FakeObstacle(45.0, 2.0, 0.15),   # 左侧远处
        FakeObstacle(-45.0, 5.0, 0.15),  # 右侧更空旷（不评估）
    ]
    best = node._select_sector(0.0)
    assert best is not None
    assert best > 0.0  # 必须选左侧扇区
    node.detour_side = 0


def test_sector_lock_right_only_evaluates_right(node):
    # 绕行侧为右：即使左侧更空旷，也只评估右侧扇区
    node.detour_side = -1
    node.obstacles = [
        FakeObstacle(45.0, 5.0, 0.15),   # 左侧更空旷（不评估）
        FakeObstacle(-45.0, 2.0, 0.15),  # 右侧远处
    ]
    best = node._select_sector(0.0)
    assert best is not None
    assert best < 0.0  # 必须选右侧扇区
    node.detour_side = 0


def test_sector_lock_released_when_side_blocked(node):
    # 绕行侧全堵：_select_sector 返回 None，control_loop 会解除锁定
    node.detour_side = 1
    node.obstacles = [
        FakeObstacle(45.0, 0.2, 0.15),
        FakeObstacle(60.0, 0.2, 0.15),
        FakeObstacle(75.0, 0.2, 0.15),
    ]
    assert node._select_sector(0.0) is None
    node.detour_side = 0


# ---------- 脱困路径延长与重试测试 ----------

def test_plan_escape_path_reaches_target_length(node):
    # 无障碍环境：路径累计长度应接近 escape_path_length（默认 1.5m）
    _reset_escape_state(node)
    node.pose = (0.0, 0.0, 0.0)
    node.obstacles = []
    node.escape_path_length = 1.5
    node.escape_path_step = 0.4
    points = max(2, int(math.ceil(node.escape_path_length /
                                  node.escape_path_step)))
    path = node._plan_escape_path(points)
    assert path is not None
    total = 0.0
    px, py = 0.0, 0.0
    for x, y in path:
        total += math.hypot(x - px, y - py)
        px, py = x, y
    assert total >= 1.3  # 至少接近 1.5m


def test_plan_escape_path_returns_none_when_blocked(node):
    # 正前方被大障碍完全封堵：第一步就无法规划，应返回 None
    _reset_escape_state(node)
    node.pose = (0.0, 0.0, 0.0)
    # 正前方 0.3m 处放一个半径 0.5m 的大障碍，覆盖整个前方
    node.obstacles = [FakeObstacle(0.0, 0.3, 0.5)]
    assert node._plan_escape_path(5) is None
    node.obstacles = []


def test_escape_retrying_backs_up_and_retries(node):
    # 无法规划时进入后退重试状态，1s 后重新尝试规划
    _reset_escape_state(node)
    node.pose = (0.0, 0.0, 0.0)
    node.obstacles = [FakeObstacle(0.0, 0.3, 0.5)]
    now = node.get_clock().now().nanoseconds * 1e-9
    node.escape_retry_until = now + 1.0
    node.escape_retrying = True
    cmd = _run(node, node.obstacles)
    assert cmd is not None
    assert cmd.linear.x < 0.0   # 后退
    node.escape_retrying = False
    node.obstacles = []


def test_escape_active_disables_cruise(node):
    # 脱困状态激活时，即使 enable_cruise=True 也不应巡航
    _reset_escape_state(node)
    node.escape_pathing = True
    node.escape_path = [(0.4, 0.0)]
    node.escape_stop_until = node.get_clock().now().nanoseconds * 1e-9 + 1.0
    node.enable_cruise = True
    cmd = _run(node, [])
    assert cmd.linear.x == 0.0
    assert cmd.angular.z == 0.0
    node.escape_pathing = False
    node.enable_cruise = False


# ---------- 脱困路径扇区锁定测试 ----------

def test_escape_pathing_locks_sector_left(node):
    # 脱困路径向左：detour_side 应设为 1，只评估左侧扇区
    _reset_escape_state(node)
    node.pose = (0.0, 0.0, 0.0)
    node.escape_path = [(0.4, 0.3)]  # 向左前方
    node.escape_pathing = True
    node.escape_stop_until = 0.0  # 停车阶段已结束
    node.obstacles = [FakeObstacle(0.0, 0.5, 0.15)]  # 正前障碍，防止提前退出
    cmd = _run(node, node.obstacles)
    assert node.detour_side == 1
    node.detour_side = 0


def test_escape_pathing_locks_sector_right(node):
    # 脱困路径向右：detour_side 应设为 -1，只评估右侧扇区
    _reset_escape_state(node)
    node.pose = (0.0, 0.0, 0.0)
    node.escape_path = [(0.4, -0.3)]  # 向右前方
    node.escape_pathing = True
    node.escape_stop_until = 0.0  # 停车阶段已结束
    node.obstacles = [FakeObstacle(0.0, 0.5, 0.15)]  # 正前障碍，防止提前退出
    cmd = _run(node, node.obstacles)
    assert node.detour_side == -1
    node.detour_side = 0


def test_escape_pathing_unlocks_on_exit(node):
    # 脱困路径走完或提前退出时，escape_pathing 应退出，detour_side 由后续
    # 常规避障逻辑重新决定
    _reset_escape_state(node)
    node.pose = (0.0, 0.0, 0.0)
    node.escape_path = [(0.4, 0.3)]
    node.escape_pathing = True
    node.escape_stop_until = 0.0
    node.escape_idx = 1  # 已走完
    node.obstacles = [FakeObstacle(0.0, 0.5, 0.15)]  # 正前障碍，防止提前退出
    cmd = _run(node, node.obstacles)
    assert node.escape_pathing is False
    assert node.escape_path == []
