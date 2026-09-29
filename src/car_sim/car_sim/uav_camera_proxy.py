"""Same-origin HTTP bridge to the UAV's existing A8 preview/control service."""
import http.client
import json
from datetime import datetime
from urllib.parse import urlsplit


class UavCameraProxy:
    def __init__(self, base_url):
        target = urlsplit(base_url)
        if target.scheme != 'http' or not target.hostname or target.path not in ('', '/'):
            raise ValueError('uav_camera_url must be an http origin')
        self.host = target.hostname
        self.port = target.port or 80
        self.origin = f'http://{target.netloc}'

    def serve(self, handler, path):
        routes = {
            '/api/uav/stream': '/stream',
            '/api/uav/status': '/api/status',
            '/api/uav/photo/download': '/snapshot.jpg',
            '/api/uav/gimbal': '/api/gimbal',
        }
        upstream = routes[path]
        body = None
        headers = {}
        if upstream == '/api/gimbal':
            # Do not turn the proxy into a cross-origin control bypass.
            origin = handler.headers.get('Origin')
            if origin and origin != 'http://' + handler.headers.get('Host', ''):
                handler._json(403, {'ok': False, 'error': 'Origin rejected'})
                return
            try:
                size = int(handler.headers.get('Content-Length', '0'))
                if not 0 < size <= 512:
                    raise ValueError('Invalid body length')
                request = json.loads(handler.rfile.read(size))
                if not isinstance(request, dict):
                    raise ValueError('JSON object required')
                action, token = request.get('action'), request.get('press_id', '')
                speed = request.get('speed', 15)
                if action not in ('up', 'down', 'left', 'right', 'stop'):
                    raise ValueError('Invalid action')
                if not isinstance(token, str) or len(token) > 80 or (action != 'stop' and not token):
                    raise ValueError('Invalid press_id')
                if type(speed) is not int or not 5 <= speed <= 30:
                    raise ValueError('Invalid speed')
                body = json.dumps(dict(action=action, press_id=token, speed=speed)).encode()
            except (ValueError, UnicodeError, TypeError) as exc:
                handler._json(400, {'ok': False, 'error': str(exc)})
                return
            headers = {'Content-Type': 'application/json', 'Origin': self.origin}
        conn = http.client.HTTPConnection(self.host, self.port, timeout=8 if upstream == '/stream' else 3)
        started = False
        try:
            conn.request('POST' if body is not None else 'GET', upstream, body, headers)
            response = conn.getresponse()
            kind = response.getheader('Content-Type', 'application/octet-stream')
            streaming = upstream == '/stream' and response.status == 200
            data = None if streaming else response.read(4 * 1024 * 1024 + 1)
            if data is not None and len(data) > 4 * 1024 * 1024:
                raise ValueError('Upstream response too large')
            if upstream == '/snapshot.jpg' and response.status == 200:
                if not data.startswith(b'\xff\xd8') or not data.endswith(b'\xff\xd9'):
                    raise ValueError('Invalid camera frame')
            handler.connection.settimeout(10)
            handler.send_response(response.status)
            handler.send_header('Content-Type', kind)
            handler.send_header('Cache-Control', 'no-store')
            handler.send_header('X-Content-Type-Options', 'nosniff')
            if data is not None:
                handler.send_header('Content-Length', str(len(data)))
            if upstream == '/snapshot.jpg' and response.status == 200:
                stamp = datetime.now().strftime('%Y%m%d_%H%M%S_%f')
                handler.send_header('Content-Disposition', f'attachment; filename="uav_{stamp}.jpg"')
            handler.end_headers()
            started = True
            if streaming:
                while True:
                    chunk = response.read1(65536)
                    if not chunk:
                        break
                    handler.wfile.write(chunk)
                    handler.wfile.flush()
            else:
                handler.wfile.write(data)
        except (OSError, http.client.HTTPException, ValueError):
            if not started:
                handler._json(503, {'ok': False, 'error': '无人机相机服务暂不可用，请检查连接后重试'})
        finally:
            conn.close()
