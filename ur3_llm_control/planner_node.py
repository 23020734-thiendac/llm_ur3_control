import json
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .configuration import load_student, read_yaml, share_path
from .llm_planner import LLMPlanner
from .robot_skills import RobotSkills
from .skill_executor import SkillExecutor
from .task_validator import format_step


class PlannerNode(Node):
    def __init__(self):
        super().__init__('llm_planner')
        share = share_path()
        scene_path = self.declare_parameter('scene_file', str(share / 'config/scene.yaml')).value
        student_path = self.declare_parameter('student_file', str(share / 'config/student_config.yaml')).value
        student = load_student(student_path)
        self.planner = LLMPlanner((share / 'prompts/planner.txt').read_text(), student)
        if not self.planner.model:
            self.get_logger().warning('Chưa có NINE_ROUTER_MODEL: robot có thể READY nhưng chưa gọi được LLM')
        self.skills = RobotSkills(self, read_yaml(scene_path))
        self.runner = SkillExecutor(self.skills)
        self.busy = threading.Lock()
        self.results = self.create_publisher(String, '/llm/task_result', 10)
        self.subscription = self.create_subscription(String, '/llm/command', self.command, 10)
        self.control_subscription = self.create_subscription(String, '/llm/control', self.control, 10)
        self.get_logger().info('STUDENT: ' + json.dumps(student, ensure_ascii=False))
        self.worker = threading.Thread(target=self.initialize, daemon=True)
        self.worker.start()

    def initialize(self):
        try:
            self.skills.initialize()
            self.get_logger().info('READY: nhập lệnh bằng ros2 run ur3_llm_control command')
        except Exception as exc:
            self.get_logger().error('STARTUP FAILED: ' + str(exc))

    def result(self, task_id, status, lines):
        self.results.publish(String(data=json.dumps(
            {'id': task_id, 'status': status, 'log': lines}, ensure_ascii=False)))

    def control(self, message):
        """Handle teleop controls: ``h`` reset and ``a/b/c`` plus placement."""
        task_id = ''
        try:
            data = json.loads(message.data)
            if not isinstance(data, dict) or set(data) != {'id', 'action'}:
                raise ValueError('Expected {id, action}')
            task_id = data['id']
            if not isinstance(task_id, str) or len(task_id) > 80:
                raise ValueError('Invalid task id')
            if data['action'] not in ('reset_scene', 'plus_a', 'plus_b', 'plus_c', 'stage_orange'):
                raise ValueError('Unsupported control action')
        except (ValueError, TypeError) as exc:
            self.result(task_id, 'INVALID_CONTROL', [str(exc)])
            return
        if not self.skills.ready:
            self.result(task_id, 'NOT_READY', ['Mô phỏng chưa READY'])
            return
        if not self.busy.acquire(blocking=False):
            self.result(task_id, 'BUSY', ['Robot đang chạy; chờ task kết thúc rồi nhấn h lại'])
            return
        action = data['action']
        if action == 'reset_scene':
            target, args = self.run_reset, (task_id,)
        elif action == 'stage_orange':
            target, args = self.run_stage_orange, (task_id,)
        else:
            target, args = self.run_plus, (task_id, {'plus_a': 'zone_a', 'plus_b': 'zone_b', 'plus_c': 'zone_c'}[action])
        self.worker = threading.Thread(target=target, args=args, daemon=True)
        self.worker.start()

    def run_plus(self, task_id, zone):
        lines = ['CONTROL: PLUS SCENE']
        try:
            status, selected_zone = self.skills.activate_plus(zone)
            if status == 'SUCCESS':
                lines.append(f'orange_cube xuất hiện tại {selected_zone}')
                lines.append('PLUS READY: vùng tạm sẽ được dùng khi plan chạm zone đang bị chiếm')
            elif status == 'ALREADY_ACTIVE':
                lines.append(f'PLUS đã bật tại {selected_zone}')
            else:
                lines.append('PLUS FAILED: ' + status)
            self.result(task_id, 'SUCCESS' if status in ('SUCCESS', 'ALREADY_ACTIVE') else 'FAILED', lines)
        except Exception as exc:
            lines.append('PLUS FAILED: ' + str(exc))
            self.result(task_id, 'FAILED', lines)
        finally:
            self.busy.release()

    def run_stage_orange(self, task_id):
        lines = ['CONTROL: STAGE ORANGE']
        try:
            status = self.skills.stage_plus_object()
            lines.append('place(orange_cube, temporary_zone) ' + status)
            if status == 'SUCCESS':
                lines.append('ORANGE TEMP SUCCESS: khối cam đã ở vùng tạm')
            self.result(task_id, 'SUCCESS' if status in ('SUCCESS', 'ALREADY_STAGED') else 'FAILED', lines)
        except Exception as exc:
            lines.append('ORANGE TEMP FAILED: ' + str(exc))
            self.result(task_id, 'FAILED', lines)
        finally:
            self.busy.release()

    def run_reset(self, task_id):
        lines = ['CONTROL: RESET SCENE']
        try:
            status = self.skills.reset_scene()
            lines.append('reset_scene()                 ' + status)
            if status == 'SUCCESS':
                self.runner.reset()
                lines.append('RESET SUCCESS: robot và 3 vật đã về vị trí ban đầu')
            else:
                lines.append('RESET FAILED: executor vẫn khóa để bảo toàn trạng thái')
            self.result(task_id, 'SUCCESS' if status == 'SUCCESS' else 'FAILED', lines)
        except Exception as exc:
            lines.append('RESET FAILED: ' + str(exc))
            self.result(task_id, 'FAILED', lines)
        finally:
            self.busy.release()

    def command(self, message):
        task_id = ''
        try:
            data = json.loads(message.data)
            if not isinstance(data, dict) or set(data) != {'id', 'command'}:
                raise ValueError('Expected {id, command}')
            task_id = data['id']
            if not isinstance(task_id, str) or len(task_id) > 80:
                raise ValueError('Invalid task id')
            command = data['command']
            if not isinstance(command, str) or not command.strip() or len(command) > 2000:
                raise ValueError('Command requires 1–2000 characters')
        except (ValueError, TypeError) as exc:
            self.result(task_id, 'INVALID_COMMAND', [str(exc)])
            return
        if not self.skills.ready:
            self.result(task_id, 'NOT_READY', ['Mô phỏng chưa READY; xem terminal launch'])
            return
        if not self.busy.acquire(blocking=False):
            self.result(task_id, 'BUSY', ['Đang thực hiện một lệnh khác; không xếp hàng lệnh cũ'])
            return
        self.worker = threading.Thread(target=self.run_task, args=(task_id, command), daemon=True)
        self.worker.start()

    def run_task(self, task_id, command):
        lines = []

        def log(line):
            lines.append(line)
            self.get_logger().info(line)

        try:
            log('USER COMMAND:\n' + command)
            plan = self.planner.plan(command)
            log('LLM PLAN:\n' + '\n'.join(format_step(step) for step in plan))
            success = self.runner.execute(plan, log)
            self.result(task_id, 'SUCCESS' if success else 'FAILED', lines)
        except Exception as exc:
            log('TASK FAILED: ' + str(exc))
            self.result(task_id, 'FAILED', lines)
        finally:
            self.busy.release()


def main(args=None):
    rclpy.init(args=args)
    node = PlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.skills.active_goal and rclpy.ok():
            node.skills.active_goal.cancel_goal_async()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
