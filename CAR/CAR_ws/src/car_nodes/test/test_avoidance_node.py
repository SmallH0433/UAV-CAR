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
