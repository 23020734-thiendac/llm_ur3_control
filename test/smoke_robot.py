"""Explicit integration check, only for a FRESH, isolated Gazebo simulation.

This bypasses the LLM to test actual MoveIt/controller/Gazebo skills.
Do not submit language commands concurrently. Restart the launch afterwards.
"""
import argparse
import threading

import rclpy
from rclpy.parameter import Parameter
from ur3_llm_control.configuration import read_yaml, share_path
from ur3_llm_control.robot_skills import RobotSkills
from ur3_llm_control.skill_executor import SkillExecutor


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute', action='store_true', required=True)
    args = parser.parse_args()
    rclpy.init()
    node = rclpy.create_node('skill_integration_check', parameter_overrides=[
        Parameter('use_sim_time', value=True)])
    skills = RobotSkills(node, read_yaml(share_path() / 'config/scene.yaml'))
    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()
    try:
        skills.initialize()
        runner = SkillExecutor(skills)
        for obj, zone in [('red_cube', 'zone_b'), ('blue_cube', 'zone_a'), ('yellow_cube', 'zone_c')]:
            plan = [{'skill': 'pick', 'object': obj},
                    {'skill': 'place', 'object': obj, 'zone': zone}, {'skill': 'home'}]
            print(f'SKILL INTEGRATION TEST (NO LLM): {obj} -> {zone}', flush=True)
            if not runner.execute(plan, lambda line: print(line, flush=True)):
                raise SystemExit(1)
        print('ALL THREE SKILL INTEGRATION TASKS PASSED; LLM NOT TESTED', flush=True)
    finally:
        rclpy.shutdown()
        thread.join(timeout=3)
        node.destroy_node()


if __name__ == '__main__':
    main()
