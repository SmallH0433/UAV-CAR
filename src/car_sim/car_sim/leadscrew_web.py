"""HTTP facade for the ROS-owned leadscrew; never accesses GPIO."""
import json
import math
import threading
import time
from std_msgs.msg import String
from car_interfaces.srv import LeadscrewControl


class LeadscrewWeb:
    def __init__(self, node):
        self.lock = threading.Lock()
        self.latest, self.received = {}, 0.
        self.client = node.create_client(LeadscrewControl, '/leadscrew/control')
        self.subscription = node.create_subscription(String, '/leadscrew/detail', self.update, 10)

    def update(self, msg):
        try:
            state = json.loads(msg.data)
            if not isinstance(state, dict):
                return
        except ValueError:
            return
        with self.lock:
            self.latest, self.received = state, time.monotonic()

    def snapshot(self):
        with self.lock:
            state = dict(self.latest)
            fresh = time.monotonic()-self.received < 2
        state['available'] = fresh and self.client.service_is_ready()
        if not state['available']:
            state['message'] = '丝杆 ROS 节点离线，控制已禁用'
        return state

    def command(self, action, payload):
        if not isinstance(payload, dict):
            raise ValueError('请求必须为 JSON 对象')
        group = payload.get('motor', 0 if action in ('stop', 'simulation_toggle') else None)
        if isinstance(group, bool) or group not in (0,1,2,'0','1','2'):
            raise ValueError('请选择 1 号、2 号或两台电机')
        request = LeadscrewControl.Request()
        request.group, request.action = int(group), action
        request.rpm, request.distance_mm, request.ramp_seconds = 6.,1.,1.
        if action == 'move':
            direction = payload.get('direction')
            if isinstance(direction,bool) or direction not in (0,1,'0','1'):
                raise ValueError('请选择向内或向外')
            request.direction = int(direction)
            for field in ('rpm','distance_mm','ramp_seconds'):
                value = payload.get(field)
                if isinstance(value,bool):
                    raise ValueError('运动参数必须为数字')
                value = float(value)
                if not math.isfinite(value):
                    raise ValueError('运动参数必须为有限数值')
                setattr(request,field,value)
        elif action == 'simulation_toggle':
            for field in ('rpm', 'ramp_seconds'):
                value = payload.get(field)
                if isinstance(value, bool):
                    raise ValueError('运动参数必须为数字')
                value = float(value)
                if not math.isfinite(value):
                    raise ValueError('运动参数必须为有限数值')
                setattr(request, field, value)
        if not self.client.service_is_ready():
            raise RuntimeError('丝杆 ROS 节点离线')
        if action != 'stop' and not self.snapshot()['available']:
            raise RuntimeError('丝杆状态已过期，未发送指令')
        request.deadline_unix = time.time()+2.5
        future = self.client.call_async(request)
        done = threading.Event()
        future.add_done_callback(lambda _:done.set())
        if not done.wait(3.):
            raise RuntimeError('指令响应超时，结果未知；请先停止并查看状态，勿重复启动')
        response = future.result()
        if response is None or not response.accepted:
            raise ValueError(response.message if response else '驱动器未接受指令')
        return {'accepted':True,'message':response.message}

    def post(self, handler, action):
        try:
            origin = handler.headers.get('Origin')
            if origin and origin != 'http://'+handler.headers.get('Host',''):
                handler._json(403, {'error':'Origin rejected'})
                return
            length = int(handler.headers.get('Content-Length','0'))
            if not 0 < length <= 2048:
                raise ValueError('请求长度无效')
            payload = json.loads(handler.rfile.read(length))
            handler._json(200, self.command(action, payload))
        except (ValueError,TypeError,UnicodeError) as exc:
            handler._json(400, {'accepted':False,'error':str(exc)})
        except (RuntimeError,OSError) as exc:
            handler._json(503, {'accepted':False,'error':str(exc)})
