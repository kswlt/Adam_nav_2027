#!/usr/bin/env python3
"""Loopback-only operator UI. ROS launches and PTYs belong to this workspace."""
import argparse
import base64
from collections import deque
import fcntl
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
import os
from pathlib import Path
import secrets
import signal
import struct
import subprocess
import sys
import termios
import threading
import time
from urllib.parse import urlsplit, parse_qs

COMPONENTS = ('driver', 'mapping', 'state', 'navigation', 'foxglove')


def identity(pid):
    """Include kernel start ticks so a recycled PID can never be stopped."""
    try:
        raw = Path(f'/proc/{pid}/stat').read_text()
        return raw[raw.rfind(')')+2:].split()[19]
    except (FileNotFoundError, ProcessLookupError):
        return None


def livox_processes():
    found = []
    for path in Path('/proc').glob('[0-9]*/cmdline'):
        try:
            args = path.read_bytes().split(b'\0')
            if args and Path(os.fsdecode(args[0])).name in ('livox_ros_driver2_node', 'livox_ros_driver_node'):
                found.append(int(path.parent.name))
        except (OSError, ValueError):
            continue
    return found


class Manager:
    def __init__(self, workspace, runtime):
        self.ws = workspace.resolve()
        self.repo = self.ws/'src/Adam_nav_2027'
        self.runtime = runtime.resolve()
        self.runtime.mkdir(parents=True, exist_ok=True)
        self.registry = self.runtime/'operator_processes.json'
        self.procs = json.loads(self.registry.read_text()) if self.registry.exists() else {}
        self.process_handles = {}
        self.lock = threading.Lock()
        self.job = {'busy': False, 'action': '', 'result': '尚未执行操作'}
        self.telemetry = {}
        self.review = None
        self.review_id = ''
        cached_review = self.runtime/'operator_review_preview.json'
        if cached_review.exists():
            data = json.loads(cached_review.read_text())
            cells = bytearray(base64.b64decode(data.pop('cells')))
            if len(cells) != data['width']*data['height'] or len(cells)>4000000:
                raise ValueError('Cached review grid is invalid')
            self.review_id = data.pop('review_id')
            self.review = {**data, '_cells':cells}
        self.mapping_output = self.procs.get('mapping', {}).get('output', '')
        if not self.mapping_output:
            for token in self.procs.get('mapping', {}).get('command', '').split():
                if token.startswith('save_directory:='):
                    self.mapping_output = token.split(':=', 1)[1]
                    self.procs['mapping']['output'] = self.mapping_output

    def persist(self):
        temporary = self.registry.with_suffix('.tmp')
        temporary.write_text(json.dumps(self.procs, indent=2))
        temporary.replace(self.registry)

    def adopt(self):
        """Adopt only explicitly named runtime pidfiles, with matching cwd/command."""
        expected = {'driver': 'livox_ros_driver2_node', 'mapping': 'z_localization_mapping.launch.py',
                    'foxglove': 'foxglove_visualization.launch.py'}
        for name, needle in expected.items():
            path = self.runtime/(name+'.pid')
            if not path.exists() or self.alive(name):
                continue
            pid = int(path.read_text().strip())
            try:
                cwd = Path(f'/proc/{pid}/cwd').resolve(strict=True)
                command = Path(f'/proc/{pid}/cmdline').read_bytes().replace(b'\0', b' ').decode()
                if cwd != self.repo or needle not in command or os.getpgid(pid) != pid:
                    raise ValueError('PID不属于此工作区独立进程组')
                self.procs[name] = {'pid': pid, 'ticks': identity(pid),
                                    'log': str(self.runtime/(name+'.log')), 'command': command}
                if name == 'mapping':
                    for token in command.split():
                        if token.startswith('save_directory:='):
                            self.mapping_output = token.split(':=', 1)[1]
                            self.procs[name]['output'] = self.mapping_output
            except (OSError, ValueError):
                continue
        self.persist()

    def alive(self, name):
        entry = self.procs.get(name, {})
        pid = entry.get('pid')
        if name in self.process_handles and self.process_handles[name].poll() is not None:
            return False
        return bool(pid and entry.get('ticks') and identity(pid) == entry['ticks'])

    def command(self, *args):
        return ['bash', '-c', 'source "$1"; shift; exec bash "$@"', 'z_operator',
                str(self.repo/'tools/z_environment.bash'), str(self.repo/'tools/z_nav.bash'), *args]

    def start(self, name, *args):
        if name not in COMPONENTS:
            raise ValueError('未知组件')
        if self.alive(name):
            return f'{name}已经运行，未重复启动'
        if name == 'driver':
            occupied = livox_processes()
            if occupied:
                raise RuntimeError('已有Livox驱动运行，拒绝竞争设备；请先由操作者完成切换。PID='+str(occupied))
        log = self.runtime/(name+'.log')
        with log.open('ab') as stream:
            stream.write(f'\n--- operator start {time.ctime()} ---\n'.encode())
            proc = subprocess.Popen(self.command(name, *args), cwd=self.repo,
                                    stdin=subprocess.DEVNULL, stdout=stream, stderr=stream,
                                    start_new_session=True)
        self.process_handles[name] = proc
        self.procs[name] = {'pid': proc.pid, 'ticks': identity(proc.pid), 'log': str(log)}
        self.persist()
        time.sleep(1)
        if proc.poll() is not None:
            raise RuntimeError(f'{name}启动失败，请查看日志')
        return f'{name}已启动；进程运行不代表传感器或定位健康通过'

    def stop(self, name):
        if name not in COMPONENTS:
            raise ValueError('未知组件')
        if not self.alive(name):
            return name+'未运行'
        pid = self.procs[name]['pid']
        if os.getpgid(pid) != pid:
            raise RuntimeError('拒绝停止非独立进程组')
        os.killpg(pid, signal.SIGINT)
        deadline = time.monotonic()+12
        while self.alive(name) and time.monotonic() < deadline:
            time.sleep(.1)
        if self.alive(name):
            raise RuntimeError(name+'尚未退出；未强制杀死，请检查日志')
        return name+'已退出'

    def maps(self):
        root = self.ws/'maps'
        native = sorted(str(p) for p in root.glob('*/keyframes.txt')) if root.exists() else []
        published = sorted(str(p) for p in root.glob('*/map_bundle.json')) if root.exists() else []
        return {'native': [str(Path(p).parent) for p in native], 'manifests': published}

    def status(self):
        return {'components': {c: {'running': self.alive(c), **self.procs.get(c, {})} for c in COMPONENTS},
                'job': dict(self.job), 'mapping_output': self.mapping_output,
                'maps': self.maps(), 'telemetry': {k: {**v, 'age_s': max(0.,time.monotonic()-v['received_monotonic'])
                    if 'received_monotonic' in v else max(0.,time.time()-v['received'])} for k,v in self.telemetry.items()},
                'server_time': time.time(),
                'review_id': self.review_id,
                'domain': os.environ.get('ROS_DOMAIN_ID', '98'), 'physical_output': False}

    def execute(self, action, data):
        if action == 'prepare_review':
            from map_review import prepare
            native = Path(data.get('native', ''))
            native.resolve(strict=True).relative_to(self.ws/'maps')
            self.review = prepare(native, float(data.get('resolution',.05)),
                                  float(data.get('min_z',-.10)), float(data.get('max_z',1.50)))
            self.review_id = secrets.token_hex(12)
            from map_review import public
            cache = self.runtime/'operator_review_preview.json'
            temp = cache.with_suffix('.tmp')
            temp.write_text(json.dumps({**public(self.review),'review_id':self.review_id}))
            temp.replace(cache)
            return '检查图已生成：灰色未知、黑色障碍。请圈选实际确认可通行的区域；当前自由格仍为0。'
        if action == 'publish_review':
            from map_review import publish
            if not self.review or data.get('review_id') != self.review_id:
                raise ValueError('检查图已变化，请重新打开并圈选')
            # Preserve the actual operator submission even if geometry validation
            # fails; never replace it with an invented region after a reload.
            attempt = self.runtime/'operator_review_submission.json'
            temporary = attempt.with_suffix('.tmp')
            temporary.write_text(json.dumps(data, ensure_ascii=False))
            temporary.replace(attempt)
            return '正式本地场地MapBundle：'+publish(self.review, data.get('polygons'),
                                                       data.get('reviewer'), data.get('confirmed'))
        if action == 'cancel_task':
            if not self.alive('navigation'):
                raise ValueError('影子导航未运行')
            command = ['bash','-c','source "$1"; shift; exec "$@"','z_operator',
                       str(self.repo/'tools/z_environment.bash'),'ros2','service','call',
                       '/nav/cancel_task','std_srvs/srv/Trigger','{}']
            result = subprocess.run(command,cwd=self.repo,text=True,stdout=subprocess.PIPE,
                                    stderr=subprocess.STDOUT,timeout=15)
            if result.returncode or 'success=False' in result.stdout or 'success: false' in result.stdout:
                raise RuntimeError(result.stdout[-8000:])
            return result.stdout+'\n取消请求已返回；动作结束请看任务状态，不代表实车停车反馈。'
        if action == 'goal':
            if not self.alive('navigation'):
                raise ValueError('先启动影子导航，并等待定位健康')
            health = self.telemetry.get('/localization/reason', {})
            if time.monotonic()-health.get('received_monotonic',-math.inf)>3 or 'status=NORMAL;' not in health.get('text',''):
                raise ValueError('尚未收到近期NORMAL定位状态')
            for topic in ('/localization/healthy', '/state/sensor_reference_healthy', '/lio/input_healthy'):
                sample = self.telemetry.get(topic, {})
                if (sample.get('value') is not True or
                        not 0 <= time.monotonic()-sample.get('received_monotonic', -math.inf) <= 1.):
                    raise ValueError('导航健康未通过或心跳过期：'+topic)
            x,y,yaw = (float(data.get(k,0)) for k in ('x','y','yaw'))
            if not all(math.isfinite(v) and abs(v)<20000 for v in (x,y,yaw)):
                raise ValueError('目标必须为有限的map坐标和弧度朝向')
            request = {'goal': {'header': {'frame_id':'map'},'pose':{'position':{'x':x,'y':y},
                        'orientation':{'z':math.sin(yaw/2),'w':math.cos(yaw/2)}}}}
            command = ['bash','-c','source "$1"; shift; exec "$@"','z_operator',
                       str(self.repo/'tools/z_environment.bash'),'ros2','service','call',
                       '/nav/submit_goal','rm_nav_interfaces/srv/SubmitGoal',json.dumps(request)]
            result = subprocess.run(command,cwd=self.repo,text=True,stdout=subprocess.PIPE,
                                    stderr=subprocess.STDOUT,timeout=30)
            if result.returncode or 'accepted=False' in result.stdout or 'accepted: false' in result.stdout:
                raise RuntimeError(result.stdout[-8000:])
            return result.stdout+'\n当前为影子导航，不启用物理串口。'
        if action == 'start_mapping':
            if any(self.alive(c) for c in ('mapping', 'state', 'navigation')):
                raise ValueError('状态/建图/导航已运行。先保存并停止旧任务，再创建新地图')
            name = data.get('name', '').strip()
            if not name or len(name) > 80 or any(c not in 'abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-' for c in name):
                raise ValueError('地图名称只能用字母、数字、下划线、连字符')
            out = self.ws/'maps'/name
            if out.exists():
                raise ValueError('该地图目录已存在，换一个名称')
            self.start('driver', str(self.runtime/'mid360_identity.json'))
            self.start('foxglove')
            result = self.start('mapping', str(out))
            self.mapping_output = str(out)
            self.procs['mapping']['output'] = str(out)
            self.persist()
            return result+'；保存目标：'+str(out)
        if action == 'save':
            if not self.alive('mapping'):
                raise ValueError('没有运行中的建图后端')
            result = subprocess.run(self.command('save'), cwd=self.repo, text=True,
                                    stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=260)
            if result.returncode:
                raise RuntimeError(result.stdout[-12000:])
            return result.stdout+'\n前端已停止；下一轮请先停止mapping。'
        if action == 'stop':
            return self.stop(data.get('component', ''))
        if action == 'navigation':
            path = Path(data.get('manifest', ''))
            if not path.is_absolute() or not path.is_file():
                raise ValueError('选择已有正式MapBundle绝对路径')
            value = json.loads(path.read_text())
            if value.get('status') != 'published' or not value.get('z_localization'):
                raise ValueError('仅接受已发布并绑定Z原生地图的MapBundle；不能直接使用draft或PCD')
            if self.alive('mapping'):
                raise ValueError('先保存地图并停止mapping，不能同时运行两个后端')
            self.start('driver', str(self.runtime/'mid360_identity.json'))
            self.start('foxglove')
            self.start('state')
            return self.start('navigation', str(path), 'local_site')+'；影子导航，无物理串口输出'
        if action == 'import':
            native = Path(data.get('native', ''))
            if not native.is_absolute() or not (native/'keyframes.txt').is_file():
                raise ValueError('选择已保存的Z原生地图目录')
            out = native.parent/(native.name+'_draft')
            args = [sys.executable, str(self.repo/'tools/import_z_map.py'), '--native-bundle', str(native),
                    '--output', str(out), '--calibration-id', 'vehicle164_yaw_axis_fit153cm_debug_v3']
            result = subprocess.run(args, cwd=self.repo, text=True, stdout=subprocess.PIPE,
                                    stderr=subprocess.STDOUT, timeout=120)
            if result.returncode:
                raise RuntimeError(result.stdout[-12000:])
            return result.stdout+'\n已导入draft；仍需检查通行区域和发布MapBundle。'
        raise ValueError('不支持的操作')

    def submit(self, action, data):
        if not self.lock.acquire(blocking=False):
            raise ValueError('上一操作尚未完成')
        self.job = {'busy': True, 'action': action, 'result': '执行中…'}
        def worker():
            try:
                result = self.execute(action, data)
                self.job = {'busy': False, 'action': action, 'ok': True, 'result': result}
            except Exception as exc:
                self.job = {'busy': False, 'action': action, 'ok': False, 'result': str(exc)}
            finally:
                self.lock.release()
        threading.Thread(target=worker, daemon=True).start()


class Terminal:
    def __init__(self, manager):
        self.master, slave = os.openpty()
        self.lock = threading.Lock()
        self.buffer = bytearray()
        self.end = 0
        rc = manager.runtime/'operator_shell.bash'
        rc.write_text(f'source "{manager.repo}/tools/z_environment.bash"\n'
                      'export PS1="[ASUS Z:${ROS_DOMAIN_ID}] \\w $ "\n')
        helper = ('import os,fcntl,termios;os.setsid();fcntl.ioctl(0,termios.TIOCSCTTY,0);'
                  'os.execv("/bin/bash",["bash","--noprofile","--rcfile",'+repr(str(rc))+',"-i"])')
        self.proc = subprocess.Popen([sys.executable, '-c', helper], stdin=slave, stdout=slave,
                                     stderr=slave, cwd=manager.repo, close_fds=True)
        os.close(slave)
        threading.Thread(target=self.reader, daemon=True).start()

    def reader(self):
        try:
            while True:
                data = os.read(self.master, 8192)
                if not data:
                    break
                with self.lock:
                    self.end += len(data)
                    self.buffer.extend(data)
                    del self.buffer[:-262144]
        except OSError:
            pass

    def read(self, offset):
        with self.lock:
            begin = self.end-len(self.buffer)
            start = max(offset, begin)
            return {'offset': self.end, 'data': base64.b64encode(self.buffer[start-begin:]).decode(),
                    'running': self.proc.poll() is None, 'truncated': offset < begin}

    def write(self, value):
        if self.proc.poll() is not None:
            raise ValueError('终端已退出，请新建终端')
        os.write(self.master, value.encode())

    def close(self):
        if self.proc.poll() is None:
            os.killpg(self.proc.pid, signal.SIGHUP)
        try:
            os.close(self.master)
        except OSError:
            pass


def ros_observer(manager):
    try:
        import rclpy
        from rclpy.qos import qos_profile_sensor_data
        from rclpy.signals import SignalHandlerOptions
        from std_msgs.msg import Bool, String
        # The HTTP process owns shutdown. ROS must not consume SIGTERM and leave
        # a dead observer with an orphaned listening HTTP socket.
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        node = rclpy.create_node('z_operator_observer')
        subs = []
        def update(topic, message):
            manager.telemetry[topic] = {'text': message.data, 'received': time.time(),
                                        'received_monotonic': time.monotonic()}
        for topic in ('/backend_node/localization_state', '/localization/reason', '/lio/input_reason', '/state/sensor_reference_reason', '/nav/task_status'):
            subs.append(node.create_subscription(String, topic, lambda m, t=topic: update(t, m), qos_profile_sensor_data))
        def update_health(topic, message):
            manager.telemetry[topic] = {'text': str(message.data).lower(), 'value': message.data,
                                       'received': time.time(), 'received_monotonic': time.monotonic()}
        for topic in ('/localization/healthy', '/state/sensor_reference_healthy', '/lio/input_healthy'):
            subs.append(node.create_subscription(Bool, topic, lambda m, t=topic: update_health(t, m), 10))
        from geometry_msgs.msg import TwistStamped
        def velocity(message):
            v = message.twist
            manager.telemetry['/nav/sensor_cmd_vel_shadow'] = {
                'text': f'frame={message.header.frame_id}; vx={v.linear.x:.3f} m/s; vy={v.linear.y:.3f} m/s; wz={v.angular.z:.3f} rad/s',
                'received': time.time(), 'received_monotonic': time.monotonic()}
        subs.append(node.create_subscription(TwistStamped, '/nav/sensor_cmd_vel_shadow', velocity, qos_profile_sensor_data))
        rclpy.spin(node)
    except Exception as exc:
        manager.telemetry['observer'] = {'text': str(exc), 'received': time.time()}


def make_handler(manager, token, terminals):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *_args):
            pass  # Never log access tokens or terminal contents.

        def send(self, status, data, mime='application/json'):
            payload = json.dumps(data, ensure_ascii=False).encode() if mime == 'application/json' else data
            self.send_response(status)
            self.send_header('Content-Type', mime)
            self.send_header('Cache-Control', 'no-store')
            self.send_header('X-Content-Type-Options', 'nosniff')
            self.send_header('Content-Length', str(len(payload)))
            self.end_headers()
            self.wfile.write(payload)

        def authorized(self):
            return secrets.compare_digest(self.headers.get('Authorization', ''), 'Bearer '+token)

        def do_GET(self):
            url = urlsplit(self.path)
            static = {'/': ('index.html', 'text/html; charset=utf-8'),
                      '/review.js': ('review.js', 'text/javascript'),
                      '/xterm.js': ('vendor/xterm.js', 'text/javascript'),
                      '/xterm.css': ('vendor/xterm.css', 'text/css')}
            if url.path in static:
                name, mime = static[url.path]
                file = Path(__file__).parent/name
                if file.exists():
                    self.send(200, file.read_bytes(), mime)
                else:
                    self.send(404, {'error': '静态文件缺失'})
                return
            if not self.authorized():
                self.send(401, {'error': '需要本机操作令牌'})
                return
            try:
                query = parse_qs(url.query)
                if url.path == '/api/status':
                    self.send(200, manager.status())
                elif url.path == '/api/review':
                    from map_review import public
                    if not manager.review:
                        raise ValueError('先生成检查图')
                    self.send(200, {**public(manager.review), 'review_id':manager.review_id})
                elif url.path == '/api/log':
                    component = query.get('component', ['mapping'])[0]
                    if component not in COMPONENTS:
                        raise ValueError('未知组件')
                    path = manager.runtime/(component+'.log')
                    text = ''
                    if path.exists():
                        with path.open('rb') as stream:
                            stream.seek(max(0, path.stat().st_size-32768))
                            text = stream.read().decode(errors='replace')
                    self.send(200, {'text': text})
                elif url.path == '/api/terminal':
                    terminal = terminals[query['id'][0]]
                    self.send(200, terminal.read(int(query.get('offset', ['0'])[0])))
                else:
                    self.send(404, {'error': '未知接口'})
            except (ValueError, KeyError) as exc:
                self.send(400, {'error': str(exc)})

        def do_POST(self):
            if not self.authorized():
                self.send(401, {'error': '需要本机操作令牌'})
                return
            try:
                size = int(self.headers.get('Content-Length', '0'))
                if size < 0 or size > 65536:
                    raise ValueError('请求过大')
                data = json.loads(self.rfile.read(size))
                if self.path == '/api/action':
                    manager.submit(data['action'], data)
                    self.send(202, {'ok': True})
                elif self.path == '/api/terminal/open':
                    if len(terminals) >= 3:
                        raise ValueError('最多3个终端，请关闭旧终端')
                    key = secrets.token_hex(16)
                    terminals[key] = Terminal(manager)
                    self.send(200, {'id': key})
                elif self.path == '/api/terminal/write':
                    terminals[data['id']].write(data['text'])
                    self.send(200, {'ok': True})
                elif self.path == '/api/terminal/resize':
                    cols, rows = int(data['cols']), int(data['rows'])
                    if not 10 <= cols <= 500 or not 3 <= rows <= 200:
                        raise ValueError('终端尺寸不合法')
                    fcntl.ioctl(terminals[data['id']].master, termios.TIOCSWINSZ, struct.pack('HHHH', rows, cols, 0, 0))
                    self.send(200, {'ok': True})
                elif self.path == '/api/terminal/close':
                    terminals.pop(data['id']).close()
                    self.send(200, {'ok': True})
                else:
                    self.send(404, {'error': '未知接口'})
            except Exception as exc:
                self.send(400, {'error': str(exc)})
    return Handler


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--workspace', type=Path, required=True)
    parser.add_argument('--runtime', type=Path, required=True)
    parser.add_argument('--port', type=int, default=8878)
    args = parser.parse_args()
    manager = Manager(args.workspace, args.runtime)
    manager.adopt()
    token_file = manager.runtime/'operator_token'
    if not token_file.exists():
        fd = os.open(token_file, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
        with os.fdopen(fd, 'w') as stream:
            stream.write(secrets.token_urlsafe(32))
    token = token_file.read_text().strip()
    terminals = {}
    server = ThreadingHTTPServer(('127.0.0.1', args.port), make_handler(manager, token, terminals))
    def terminate(_number, _frame):
        raise SystemExit(0)
    signal.signal(signal.SIGTERM, terminate)
    threading.Thread(target=ros_observer, args=(manager,), daemon=True).start()
    print(f'Z操作台已启动于127.0.0.1:{args.port}；使用SSH隧道；不提供物理输出按钮。', flush=True)
    try:
        server.serve_forever()
    finally:
        for terminal in terminals.values():
            terminal.close()
        server.server_close()


if __name__ == '__main__':
    main()
