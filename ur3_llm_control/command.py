"""Terminal entry point: plan only, or send one natural language task to ROS."""
import argparse
import select
import termios
import tty
import json
import sys
import time
import uuid
import unicodedata

from .configuration import load_student, share_path
from .llm_planner import LLMPlanner
from .task_validator import format_step


def is_orange_temp_command(command):
    """Recognize the dedicated natural-language orange staging command."""
    text = unicodedata.normalize('NFD', command.lower())
    text = ''.join(ch for ch in text if unicodedata.category(ch) != 'Mn')
    has_orange = 'cam' in text or 'orange' in text
    has_temporary = ('temporary' in text or 'tam thoi' in text or
                     'vung tam' in text or 'zone tam' in text)
    return has_orange and has_temporary


def teleop():
    """Keyboard teleop for in-session scene reset.

    ``h`` sends a reset request; ``a/b/c`` place the plus cube in the
    selected zone; ``q`` exits teleop while
    leaving Gazebo and the planner running.
    """
    import rclpy
    from std_msgs.msg import String

    if not sys.stdin.isatty():
        raise RuntimeError('--teleop cần chạy trong terminal tương tác')
    rclpy.init()
    node = rclpy.create_node('ur3_keyboard_teleop_' + uuid.uuid4().hex[:8])
    pub = node.create_publisher(String, '/llm/control', 10)
    pending = set()
    results = {}

    def on_result(msg):
        try:
            data = json.loads(msg.data)
            task_id = data.get('id')
            if task_id in pending:
                results[task_id] = data
        except (ValueError, AttributeError):
            pass

    node.create_subscription(String, '/llm/task_result', on_result, 10)
    deadline = time.monotonic() + 10.0
    while pub.get_subscription_count() == 0:
        if time.monotonic() > deadline:
            node.destroy_node()
            rclpy.shutdown()
            raise RuntimeError('Không thấy llm_planner; chạy llm_robot.launch.py trước')
        rclpy.spin_once(node, timeout_sec=0.1)

    print('TELEOP: h reset; a=cam vào A, b=cam vào B, c=cam vào C; q thoát.', flush=True)
    old_settings = termios.tcgetattr(sys.stdin.fileno())
    try:
        tty.setcbreak(sys.stdin.fileno())
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            for task_id in list(results):
                data = results.pop(task_id)
                pending.discard(task_id)
                print('\n'.join(data.get('log', [])), flush=True)
                print('RESULT: ' + str(data.get('status')), flush=True)
            readable, _, _ = select.select([sys.stdin], [], [], 0.05)
            if not readable:
                continue
            key = sys.stdin.read(1).lower()
            if key == 'q':
                break
            if key in ('h', 'a', 'b', 'c'):
                action = 'reset_scene' if key == 'h' else 'plus_' + key
                task_id = ('reset-' if key == 'h' else 'plus-' + key + '-') + uuid.uuid4().hex
                pending.add(task_id)
                pub.publish(String(data=json.dumps(
                    {'id': task_id, 'action': action})))
                if key == 'h':
                    print('Đã gửi reset; chờ robot về home...', flush=True)
                else:
                    print(f'Đã chọn PLUS: khối cam sẽ xuất hiện tại Zone {key.upper()}...', flush=True)
    finally:
        termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, old_settings)
        node.destroy_node()
        rclpy.shutdown()


def send_control(action):
    import rclpy
    from std_msgs.msg import String
    rclpy.init()
    node = rclpy.create_node('ur3_control_' + uuid.uuid4().hex[:8])
    task_id = uuid.uuid4().hex
    received = []

    def on_result(msg):
        try:
            data = json.loads(msg.data)
            if data.get('id') == task_id:
                received.append(data)
        except (ValueError, AttributeError):
            pass

    node.create_subscription(String, '/llm/task_result', on_result, 10)
    pub = node.create_publisher(String, '/llm/control', 10)
    try:
        deadline = time.monotonic() + 10.0
        while pub.get_subscription_count() == 0 or node.count_publishers('/llm/task_result') == 0:
            if time.monotonic() > deadline:
                raise RuntimeError('Không thấy llm_planner; chạy llm_robot.launch.py trước')
            rclpy.spin_once(node, timeout_sec=0.1)
        pub.publish(String(data=json.dumps({'id': task_id, 'action': action})))
        deadline = time.monotonic() + 300.0
        while not received:
            if time.monotonic() > deadline:
                raise RuntimeError('Hết thời gian chờ lệnh điều khiển')
            rclpy.spin_once(node, timeout_sec=0.1)
        result = received[0]
        print('\n'.join(result.get('log', [])))
        print('RESULT: ' + str(result.get('status')))
        raise SystemExit(0 if result.get('status') == 'SUCCESS' else 1)
    except (RuntimeError, KeyboardInterrupt) as exc:
        print(str(exc), file=sys.stderr)
        raise SystemExit(1)
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(description='Điều khiển UR3 bằng câu lệnh tự nhiên')
    parser.add_argument('command', nargs='?', help='Ví dụ: Đưa vật màu đỏ sang vùng B.')
    parser.add_argument('--plan-only', action='store_true', help='Gọi LLM và kiểm tra JSON; không chạy robot')
    parser.add_argument('--teleop', action='store_true', help='Bàn phím: h reset, a/b/c chọn zone cam, q thoát')
    parser.add_argument('--orange-temp', action='store_true', help='Đưa khối cam vào temporary zone, không gọi LLM')
    parser.add_argument('--timeout', type=float, default=600.0)
    args = parser.parse_args()
    if args.orange_temp:
        send_control('stage_orange')
    if args.teleop:
        try:
            teleop()
        except (RuntimeError, KeyboardInterrupt) as exc:
            print(str(exc), file=sys.stderr)
            raise SystemExit(1)
        return
    command = args.command or input('USER COMMAND: ').strip()
    if not args.plan_only and is_orange_temp_command(command):
        send_control('stage_orange')
    if args.plan_only:
        share = share_path()
        try:
            plan = LLMPlanner((share / 'prompts/planner.txt').read_text(),
                              load_student(share / 'config/student_config.yaml')).plan(command)
            print('USER COMMAND:\n' + command)
            print('LLM PLAN:\n' + '\n'.join(format_step(s) for s in plan))
            print(json.dumps({'plan': plan}, indent=2, ensure_ascii=False))
            print('PLAN VALID — chưa thực thi robot')
        except Exception as exc:
            print('TASK FAILED: ' + str(exc), file=sys.stderr)
            raise SystemExit(1)
        return
    import rclpy
    from std_msgs.msg import String
    rclpy.init()
    node = rclpy.create_node('llm_command_' + uuid.uuid4().hex[:8])
    task_id = uuid.uuid4().hex
    received = []

    def on_result(msg):
        try:
            data = json.loads(msg.data)
            if data.get('id') == task_id:
                received.append(data)
        except (ValueError, AttributeError):
            pass

    sub = node.create_subscription(String, '/llm/task_result', on_result, 10)
    pub = node.create_publisher(String, '/llm/command', 10)
    exit_code = 1
    try:
        deadline = time.monotonic() + 10.0
        while (pub.get_subscription_count() == 0 or node.count_publishers('/llm/task_result') == 0):
            if time.monotonic() > deadline:
                raise RuntimeError('Không thấy llm_planner; chạy llm_robot.launch.py trước')
            rclpy.spin_once(node, timeout_sec=0.1)
        pub.publish(String(data=json.dumps({'id': task_id, 'command': command}, ensure_ascii=False)))
        print('Đã gửi lệnh. Xem tiến trình tại terminal launch.', flush=True)
        deadline = time.monotonic() + args.timeout
        while not received:
            if time.monotonic() > deadline:
                raise RuntimeError('Hết thời gian chờ kết quả; lệnh có thể vẫn đang chạy. Xem terminal launch.')
            rclpy.spin_once(node, timeout_sec=0.1)
        result = received[0]
        print('\n'.join(result['log']))
        print('RESULT: ' + result['status'])
        exit_code = 0 if result['status'] == 'SUCCESS' else 1
    except (RuntimeError, KeyboardInterrupt) as exc:
        print(str(exc) or 'Ngừng chờ; Ctrl+C tại terminal launch để dừng mô phỏng.', file=sys.stderr)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(exit_code)
