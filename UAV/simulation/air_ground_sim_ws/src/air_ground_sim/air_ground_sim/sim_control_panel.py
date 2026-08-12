"""Small WSLg control panel for AprilTag takeover and manual UGV driving."""

import json
import tkinter as tk
from tkinter import ttk

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool


def drive_velocity(actions, linear_speed: float = 0.35, angular_speed: float = 0.70):
    """Return the UGV command for the currently held panel actions.

    The Gazebo vehicle's joint frame has the opposite yaw sign to the labels
    shown to the operator, so a requested left turn is a negative command.
    """
    linear = 0.0
    angular = 0.0
    if "forward" in actions:
        linear += linear_speed
    if "backward" in actions:
        linear -= linear_speed
    if "left" in actions:
        angular -= angular_speed
    if "right" in actions:
        angular += angular_speed
    return linear, angular


class SimControlNode(Node):
    def __init__(self) -> None:
        super().__init__("sim_control_panel")
        # Route legacy desktop teleoperation through the same fail-closed
        # authority mux as the browser UI.  No operator is allowed to publish
        # directly to the chassis-adapter output.
        self.drive_publisher = self.create_publisher(
            Twist, "/ugv/teleop/cmd_vel", 10
        )
        self.operator_heartbeat_publisher = self.create_publisher(
            Bool, "/ugv/operator/heartbeat", 10
        )
        self.tracker_client = self.create_client(SetBool, "/apriltag_tracker/enable")
        self.demo_client = self.create_client(SetBool, "/ugv_demo_motion/enable")
        self.tracker_status = {}
        self.uav_status = {}
        self.create_subscription(String, "/apriltag/status", self._tracker_callback, 10)
        self.create_subscription(String, "/uav/mavlink/status", self._uav_callback, 10)

    def _tracker_callback(self, message: String) -> None:
        try:
            self.tracker_status = json.loads(message.data)
        except json.JSONDecodeError:
            pass

    def _uav_callback(self, message: String) -> None:
        try:
            self.uav_status = json.loads(message.data)
        except json.JSONDecodeError:
            pass

    def call_switch(self, client, enabled: bool, callback=None) -> None:
        if not client.service_is_ready():
            if callback:
                callback(False, "服务尚未就绪")
            return
        request = SetBool.Request()
        request.data = enabled
        future = client.call_async(request)

        if callback:
            def done(result_future):
                try:
                    result = result_future.result()
                    callback(bool(result.success), result.message)
                except Exception as error:  # service transport error
                    callback(False, str(error))

            future.add_done_callback(done)

    def publish_drive(self, linear: float, angular: float) -> None:
        heartbeat = Bool()
        heartbeat.data = True
        self.operator_heartbeat_publisher.publish(heartbeat)
        command = Twist()
        command.linear.x = linear
        command.angular.z = angular
        self.drive_publisher.publish(command)

    def release_operator_authority(self) -> None:
        heartbeat = Bool()
        heartbeat.data = False
        self.operator_heartbeat_publisher.publish(heartbeat)


class SimControlPanel:
    LINEAR_SPEED = 0.35
    ANGULAR_SPEED = 0.70
    FONT_FAMILY = "Noto Sans SC"

    def __init__(self, node: SimControlNode) -> None:
        self.node = node
        self.root = tk.Tk()
        self.root.title("无人机托管 / 小车控制面板")
        self.root.geometry("640x515")
        self.root.resizable(False, False)
        self.root.option_add("*Font", (self.FONT_FAMILY, 10))
        style = ttk.Style(self.root)
        style.configure("TLabel", font=(self.FONT_FAMILY, 10))
        style.configure("TLabelframe.Label", font=(self.FONT_FAMILY, 10, "bold"))
        self.root.protocol("WM_DELETE_WINDOW", self.close)
        self.pressed = set()
        self.closed = False
        self.message = tk.StringVar(value="正在连接ROS 2服务……")
        self.uav_text = tk.StringVar(value="无人机：等待状态")
        self.tracker_text = tk.StringVar(value="树莓派托管：关闭")
        self.tag_text = tk.StringVar(value="AprilTag：等待画面")
        self.car_text = tk.StringVar(value="小车：停止")
        self._build()
        self._bind_keys()

        # Stop the scripted route so it cannot compete with manual commands.
        self.node.call_switch(self.node.demo_client, False)
        self.root.after(20, self.ros_tick)
        self.root.after(100, self.drive_tick)
        self.root.after(200, self.status_tick)

    def _build(self) -> None:
        ttk.Label(
            self.root,
            text="AprilTag 跟随仿真控制",
            font=(self.FONT_FAMILY, 19, "bold"),
        ).pack(pady=(10, 2))
        ttk.Label(
            self.root,
            text="软件开关等价于现实遥控器的 RC7 拨片",
            font=(self.FONT_FAMILY, 10),
        ).pack(pady=(0, 7))

        switch_frame = ttk.LabelFrame(self.root, text="树莓派代理模式", padding=8)
        switch_frame.pack(fill="x", padx=14, pady=3)
        tk.Button(
            switch_frame,
            text="进入托管（RC7高位）",
            bg="#2e9d52",
            fg="white",
            activebackground="#277f45",
            font=(self.FONT_FAMILY, 11, "bold"),
            command=lambda: self.set_tracking(True),
            height=1,
        ).pack(side="left", expand=True, fill="x", padx=(0, 6))
        tk.Button(
            switch_frame,
            text="退出托管（RC7低位）",
            bg="#c84242",
            fg="white",
            activebackground="#a93636",
            font=(self.FONT_FAMILY, 11, "bold"),
            command=lambda: self.set_tracking(False),
            height=1,
        ).pack(side="left", expand=True, fill="x", padx=(6, 0))

        drive_frame = ttk.LabelFrame(
            self.root, text="小车人工控制（按住移动，松开停车）", padding=7
        )
        drive_frame.pack(fill="x", padx=14, pady=7)
        button_specs = [
            ("前进\nW / ↑", "forward", 0, 1),
            ("左转\nA / ←", "left", 1, 0),
            ("立即停车\nSPACE", "stop", 1, 1),
            ("右转\nD / →", "right", 1, 2),
            ("后退\nS / ↓", "backward", 2, 1),
        ]
        for label, action, row, column in button_specs:
            button = tk.Button(
                drive_frame,
                text=label,
                font=(self.FONT_FAMILY, 10, "bold"),
                width=15,
                height=2,
            )
            button.grid(row=row, column=column, padx=7, pady=4)
            if action == "stop":
                button.configure(bg="#efb642", command=self.stop_car)
            else:
                button.bind("<ButtonPress-1>", lambda _event, a=action: self.press(a))
                button.bind("<ButtonRelease-1>", lambda _event, a=action: self.release(a))
        for index in range(3):
            drive_frame.columnconfigure(index, weight=1)

        status_frame = ttk.LabelFrame(self.root, text="实时状态", padding=7)
        status_frame.pack(fill="x", padx=14, pady=(0, 5))
        for variable in (self.uav_text, self.tracker_text, self.tag_text, self.car_text):
            ttk.Label(status_frame, textvariable=variable).pack(anchor="w", pady=1)
        ttk.Label(self.root, textvariable=self.message, foreground="#285a8f").pack(pady=(0, 5))

    def _bind_keys(self) -> None:
        key_actions = {
            "w": "forward", "Up": "forward",
            "s": "backward", "Down": "backward",
            "a": "left", "Left": "left",
            "d": "right", "Right": "right",
        }
        for key, action in key_actions.items():
            self.root.bind(f"<KeyPress-{key}>", lambda _event, a=action: self.press(a))
            self.root.bind(f"<KeyRelease-{key}>", lambda _event, a=action: self.release(a))
        self.root.bind("<space>", lambda _event: self.stop_car())
        self.root.focus_force()

    def set_tracking(self, enabled: bool) -> None:
        self.message.set("正在开启托管……" if enabled else "正在退出托管……")

        def completed(success: bool, message: str) -> None:
            self.message.set(("成功：" if success else "失败：") + message)

        self.node.call_switch(self.node.tracker_client, enabled, completed)

    def press(self, action: str) -> None:
        if action != "stop":
            self.pressed.add(action)

    def release(self, action: str) -> None:
        self.pressed.discard(action)

    def stop_car(self) -> None:
        self.pressed.clear()
        self.node.publish_drive(0.0, 0.0)
        self.message.set("小车已停车")

    def drive_tick(self) -> None:
        if self.closed:
            return
        linear, angular = drive_velocity(
            self.pressed, self.LINEAR_SPEED, self.ANGULAR_SPEED
        )
        self.node.publish_drive(linear, angular)
        if linear or angular:
            self.car_text.set(f"小车：线速度 {linear:+.2f} m/s，角速度 {angular:+.2f} rad/s")
        else:
            self.car_text.set("小车：停止")
        self.root.after(100, self.drive_tick)

    def ros_tick(self) -> None:
        if self.closed:
            return
        rclpy.spin_once(self.node, timeout_sec=0.0)
        self.root.after(20, self.ros_tick)

    def status_tick(self) -> None:
        if self.closed:
            return
        uav = self.node.uav_status
        tracker = self.node.tracker_status
        altitude = uav.get("relative_alt_m")
        altitude_text = "?" if altitude is None else f"{float(altitude):.2f}m"
        self.uav_text.set(
            f"无人机：{'已解锁' if uav.get('armed') else '已锁定'} / "
            f"{uav.get('mode', '?')} / 高度 {altitude_text}"
        )
        self.tracker_text.set(
            f"树莓派托管：{'运行中' if tracker.get('active') else '关闭'} / "
            f"状态 {tracker.get('reason', '?')}"
        )
        self.tag_text.set(
            f"AprilTag ID 0：{'已识别' if tracker.get('tag_visible') else '未识别'} / "
            f"画面误差 ({tracker.get('error_x', 0)}, {tracker.get('error_y', 0)})"
        )
        self.root.after(200, self.status_tick)

    def close(self) -> None:
        self.closed = True
        self.node.publish_drive(0.0, 0.0)
        self.node.release_operator_authority()
        self.node.call_switch(self.node.tracker_client, False)
        self.root.destroy()

    def run(self) -> None:
        self.root.mainloop()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SimControlNode()
    panel = SimControlPanel(node)
    try:
        panel.run()
    finally:
        node.publish_drive(0.0, 0.0)
        node.release_operator_authority()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
