"""MoveIt actions and collision scene, plus a controllable two-finger Gazebo gripper.

All arm motions use MoveIt /move_action or /execute_trajectory. The LLM cannot supply poses/joints.
Methods run in the worker thread while the ROS executor spins callbacks.
"""
import math
import threading
import time

from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Pose
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import (
    AttachedCollisionObject, CollisionObject, Constraints, JointConstraint,
    MoveItErrorCodes, OrientationConstraint, PlanningScene, PositionConstraint,
)
from moveit_msgs.srv import ApplyPlanningScene, GetCartesianPath, GetStateValidity
from rclpy.action import ActionClient
from rclpy.time import Time
from sensor_msgs.msg import JointState
from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import SetEntityPose
from shape_msgs.msg import SolidPrimitive
from trajectory_msgs.msg import JointTrajectoryPoint
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker, MarkerArray


def pose_at(xyz, downward=False):
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = map(float, xyz)
    pose.orientation.x = 1.0 if downward else 0.0
    pose.orientation.w = 0.0 if downward else 1.0
    return pose


def await_future(future, timeout=15.0):
    done = threading.Event()
    future.add_done_callback(lambda _: done.set())
    if not done.wait(timeout):
        raise RuntimeError('TIMEOUT: ROS service/action không phản hồi')
    return future.result()


class RobotSkills:
    def __init__(self, node, scene):
        self.node, self.scene = node, scene
        self.frame = scene['frame_id']
        self.tool = scene['tool_link']
        self.size = float(scene['cube_size'])
        self.initial_positions = {name: list(data['position']) for name, data in scene['objects'].items()}
        self.positions = {name: list(xyz) for name, xyz in self.initial_positions.items()}
        plus = scene.get('plus_object', {})
        self.plus_object = str(plus.get('name', 'orange_cube'))
        self.plus_parking_position = list(plus.get('parking_position', [0.40, 0.24, 0.1175]))
        self.plus_active = False
        self.plus_zone = None
        self.plus_staged = False
        self.held = None
        self.sync_lock = threading.Lock()
        self.ready = False
        self.sync_error = None
        self.sync_future = None
        self.sync_started = 0.0
        self.last_sync = 0.0
        self.active_goal = None
        # Both jaws share one commanded state so every open/close trajectory
        # starts from the same timestamp and reaches its target together.
        self.gripper_positions = [0.0, 0.0]
        self.gripper_feedback = [0.0, 0.0]
        self.gripper_feedback_valid = False
        self.gripper_state_lock = threading.Lock()
        self.gripper_state_sub = node.create_subscription(
            JointState, '/joint_states', self._update_gripper_feedback, 10)
        self.cartesian = node.create_client(GetCartesianPath, '/compute_cartesian_path')
        self.validity = node.create_client(GetStateValidity, '/check_state_validity')
        self.execute_trajectory = ActionClient(node, ExecuteTrajectory, '/execute_trajectory')
        self.gripper = ActionClient(node, FollowJointTrajectory, '/gripper_controller/follow_joint_trajectory')
        self.arm_controller = ActionClient(node, FollowJointTrajectory, '/joint_trajectory_controller/follow_joint_trajectory')
        self.motion = ActionClient(node, MoveGroup, '/move_action')
        self.apply = node.create_client(ApplyPlanningScene, '/apply_planning_scene')
        self.set_pose = node.create_client(SetEntityPose, f"/world/{scene['world_name']}/set_pose")
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, node)
        self.markers = node.create_publisher(MarkerArray, '/llm_scene_markers', 10)
        self.sync_timer = node.create_timer(0.01, self._sync_held)
        self.marker_timer = node.create_timer(0.5, self._publish_zones)

    def _update_gripper_feedback(self, message):
        positions = dict(zip(message.name, message.position))
        if not all(name in positions for name in ('gripper_left_joint', 'gripper_right_joint')):
            return
        with self.gripper_state_lock:
            self.gripper_feedback = [
                float(positions['gripper_left_joint']),
                float(positions['gripper_right_joint'])]
            self.gripper_feedback_valid = True

    def initialize(self):
        for client in (self.apply, self.set_pose, self.cartesian, self.validity):
            if not client.wait_for_service(timeout_sec=90.0):
                raise RuntimeError('Service chưa sẵn sàng: ' + client.srv_name)
        if not self.motion.wait_for_server(timeout_sec=90.0):
            raise RuntimeError('MoveIt /move_action chưa sẵn sàng')
        if not self.execute_trajectory.wait_for_server(timeout_sec=30.0):
            raise RuntimeError('MoveIt /execute_trajectory chưa sẵn sàng')
        if not self.gripper.wait_for_server(timeout_sec=30.0):
            raise RuntimeError('Gripper controller chưa sẵn sàng')
        if not self.arm_controller.wait_for_server(timeout_sec=30.0):
            raise RuntimeError('Arm trajectory controller chưa sẵn sàng')
        deadline = time.monotonic() + 60
        while not self.tf.can_transform(self.frame, self.tool, Time()):
            if time.monotonic() > deadline:
                raise RuntimeError('Không có TF world -> grasp_link')
            time.sleep(0.1)
        objects = [self._box('work_table', self.scene['table']['center'], self.scene['table']['size']),
                   self._box('floor', [0.0, 0.0, -0.07], [3.0, 3.0, 0.02])]
        objects += [self._box(name, xyz, [self.size] * 3) for name, xyz in self.positions.items()]
        self._apply_scene(objects)
        # Check the Gazebo service really works before declaring readiness.
        for name, xyz in self.positions.items():
            self._set_entity(name, pose_at(xyz))
        # The plus object starts on the table; a/b/c moves it into a selected zone.
        self._set_entity(self.plus_object, pose_at(self.plus_parking_position))
        self.ready = True

    def preflight(self, obj, zone):
        if not self.ready or self.sync_error:
            return 'NOT_READY' if not self.sync_error else self.sync_error
        if obj not in self.positions:
            return 'INVALID_OBJECT'
        if zone not in self.scene['zones']:
            return 'INVALID_ZONE'
        if self.held:
            return 'ALREADY_HOLDING_OBJECT'
        dest = self.scene['zones'][zone]
        for name, xyz in self.positions.items():
            if name != obj and math.dist(xyz, dest) < self.size * 1.5:
                return 'ZONE_OCCUPIED'
        return 'SUCCESS'

    def activate_plus(self, zone):
        """Place the extra orange cube in the operator-selected target zone."""
        if not self.ready:
            return 'NOT_READY', None
        if zone not in self.scene['zones']:
            return 'INVALID_ZONE', None
        if self.held:
            return 'FAILED: không thể bật plus khi đang giữ vật', None
        if self.plus_active:
            return 'ALREADY_ACTIVE', self.plus_zone
        self.plus_zone = zone
        xyz = list(self.scene['zones'][self.plus_zone])
        self.positions[self.plus_object] = xyz
        self.plus_active = True
        self.plus_staged = False
        try:
            self._set_entity(self.plus_object, pose_at(xyz))
            objects = [self._box('work_table', self.scene['table']['center'], self.scene['table']['size']),
                       self._box('floor', [0.0, 0.0, -0.07], [3.0, 3.0, 0.02])]
            objects += [self._box(name, item, [self.size] * 3)
                        for name, item in self.positions.items()]
            self._apply_scene(objects)
        except Exception:
            self.positions.pop(self.plus_object, None)
            self.plus_active = False
            self.plus_zone = None
            raise
        return 'SUCCESS', self.plus_zone

    def stage_plus_object(self):
        """Place the orange cube in temporary_zone as a standalone command."""
        if not self.ready:
            return 'NOT_READY'
        if self.held:
            return 'FAILED: không thể xếp cam khi đang giữ vật khác'
        if self.plus_active and self.plus_zone is None:
            return 'ALREADY_STAGED'
        if not self.plus_active:
            self.positions[self.plus_object] = list(self.plus_parking_position)
            self.plus_active = True
            self.plus_zone = None
            self.plus_staged = False
            objects = [self._box('work_table', self.scene['table']['center'], self.scene['table']['size']),
                       self._box('floor', [0.0, 0.0, -0.07], [3.0, 3.0, 0.02])]
            objects += [self._box(name, item, [self.size] * 3)
                        for name, item in self.positions.items()]
            self._apply_scene(objects)
        if self.plus_zone is not None:
            return self.clear_plus_zone()
        status = self.pick(self.plus_object)
        if status != 'SUCCESS':
            return status
        status = self.place(self.plus_object, 'temporary_zone')
        if status != 'SUCCESS':
            return status
        status = self.home()
        if status == 'SUCCESS':
            self.plus_staged = True
        return status

    def clear_plus_zone(self):
        """Move the orange cube from its occupied zone to the temporary zone."""
        if not self.plus_active or self.plus_zone is None:
            return 'SUCCESS'
        status = self.pick(self.plus_object)
        if status != 'SUCCESS':
            return status
        status = self.place(self.plus_object, 'temporary_zone')
        if status != 'SUCCESS':
            return status
        status = self.home()
        if status == 'SUCCESS':
            self.plus_zone = None
            self.plus_staged = True
        return status

    def _box(self, name, xyz, dimensions, frame=None):
        obj = CollisionObject()
        obj.header.frame_id = frame or self.frame
        obj.id = name
        obj.operation = CollisionObject.ADD
        box = SolidPrimitive(type=SolidPrimitive.BOX, dimensions=list(map(float, dimensions)))
        obj.primitives = [box]
        obj.primitive_poses = [pose_at(xyz)]
        return obj

    def _apply_scene(self, objects=(), attached=()):
        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.is_diff = True
        scene.world.collision_objects = list(objects)
        scene.robot_state.attached_collision_objects = list(attached)
        result = await_future(self.apply.call_async(ApplyPlanningScene.Request(scene=scene)))
        if not result.success:
            raise RuntimeError('SCENE_FAILED: MoveIt không nhận collision scene')

    def _set_entity(self, name, pose):
        req = SetEntityPose.Request(entity=Entity(name=name, type=Entity.MODEL), pose=pose)
        result = await_future(self.set_pose.call_async(req), 5.0)
        if not result.success:
            raise RuntimeError('SIMULATION_FAILED: không cập nhật được ' + name)

    def _held_pose(self):
        t = self.tf.lookup_transform(self.frame, self.tool, Time()).transform
        # The cube centre is below the grasp point by half its height plus
        # the small clearance used while the jaws settle on the table.
        q = t.rotation
        d = self.size / 2 + float(self.scene['release_clearance'])
        pose = Pose()
        pose.position.x = t.translation.x + 2 * (q.x * q.z + q.w * q.y) * d
        pose.position.y = t.translation.y + 2 * (q.y * q.z - q.w * q.x) * d
        pose.position.z = t.translation.z + (1 - 2 * (q.x*q.x + q.y*q.y)) * d
        pose.orientation = q
        return pose

    def _sync_held(self):
        with self.sync_lock:
            self._sync_held_locked()

    def _sync_held_locked(self):
        if not self.held or self.sync_error:
            return
        try:
            now = time.monotonic()
            if self.sync_future is not None:
                if not self.sync_future.done():
                    if now - self.sync_started > 3.0:
                        raise RuntimeError('Gazebo set_pose timeout')
                    return
                if not self.sync_future.result().success:
                    raise RuntimeError('Gazebo set_pose rejected')
                self.last_sync = now
            pose = self._held_pose()
            req = SetEntityPose.Request(entity=Entity(name=self.held, type=Entity.MODEL), pose=pose)
            self.sync_future = self.set_pose.call_async(req)
            self.sync_started = now
        except Exception as exc:
            self.sync_error = 'SIMULATION_FAILED: ' + str(exc)
            self.node.get_logger().error(self.sync_error)
            if self.active_goal:
                self.active_goal.cancel_goal_async()

    def _move(self, constraints):
        if self.sync_error:
            return self.sync_error
        goal = MoveGroup.Goal()
        req = goal.request
        req.group_name = self.scene['group_name']
        req.num_planning_attempts = 5
        req.allowed_planning_time = float(self.scene['planning_time'])
        req.max_velocity_scaling_factor = float(self.scene['velocity_scaling'])
        req.max_acceleration_scaling_factor = float(self.scene['acceleration_scaling'])
        req.start_state.is_diff = True
        req.goal_constraints = [constraints]
        goal.planning_options.planning_scene_diff.is_diff = True
        goal.planning_options.planning_scene_diff.robot_state.is_diff = True
        goal.planning_options.plan_only = False
        goal.planning_options.replan = False
        return self._run_goal(self.motion, goal)

    def _run_goal(self, client, goal):
        if self.sync_error:
            return self.sync_error
        future = client.send_goal_async(goal)
        try:
            handle = await_future(future, 15.0)
        except RuntimeError:
            # A late goal acceptance must not leave an orphaned movement.
            future.add_done_callback(lambda f: f.result().cancel_goal_async()
                                     if f.result() and f.result().accepted else None)
            raise
        if not handle.accepted:
            return 'PLANNING_FAILED: goal rejected'
        self.active_goal = handle
        if self.sync_error:
            await_future(handle.cancel_goal_async(), 5.0)
        try:
            result = await_future(handle.get_result_async(), float(self.scene['motion_timeout']))
        except RuntimeError:
            await_future(handle.cancel_goal_async(), 5.0)
            raise
        finally:
            self.active_goal = None
        if self.sync_error:
            return self.sync_error
        code = result.result.error_code.val
        if result.status == GoalStatus.STATUS_SUCCEEDED and code == MoveItErrorCodes.SUCCESS:
            return 'SUCCESS'
        if code in (MoveItErrorCodes.PLANNING_FAILED, MoveItErrorCodes.INVALID_MOTION_PLAN,
                    MoveItErrorCodes.NO_IK_SOLUTION, MoveItErrorCodes.GOAL_IN_COLLISION,
                    MoveItErrorCodes.START_STATE_IN_COLLISION):
            return f'PLANNING_FAILED: MoveIt code={code}'
        return f'FAILED: MoveIt code={code}, action status={result.status}'

    def _move_to(self, xyz):
        constraints = Constraints()
        position = PositionConstraint()
        position.header.frame_id = self.frame
        position.link_name = self.tool
        position.weight = 1.0
        position.constraint_region.primitives = [SolidPrimitive(
            type=SolidPrimitive.SPHERE, dimensions=[0.002])]
        position.constraint_region.primitive_poses = [pose_at(xyz)]
        orientation = OrientationConstraint()
        orientation.header.frame_id = self.frame
        orientation.link_name = self.tool
        orientation.orientation = pose_at(xyz, downward=True).orientation
        orientation.absolute_x_axis_tolerance = 0.02
        orientation.absolute_y_axis_tolerance = 0.02
        orientation.absolute_z_axis_tolerance = 0.02
        orientation.weight = 1.0
        constraints.position_constraints = [position]
        constraints.orientation_constraints = [orientation]
        # Elbow-up posture keeps the arm above the other cubes on the table.
        constraints.joint_constraints = [JointConstraint(
            joint_name='elbow_joint', position=1.5, tolerance_above=1.4,
            tolerance_below=1.4, weight=1.0),
            JointConstraint(joint_name='shoulder_lift_joint', position=-1.55,
                            tolerance_above=1.2, tolerance_below=1.2, weight=1.0),
            JointConstraint(joint_name='shoulder_pan_joint', position=0.0,
                            tolerance_above=1.6, tolerance_below=1.6, weight=1.0)]
        return self._move(constraints)

    def _cartesian_to(self, xyz, avoid_collisions=True):
        if self.sync_error:
            return self.sync_error
        request = GetCartesianPath.Request()
        request.header.frame_id = self.frame
        request.start_state.is_diff = True
        request.group_name = self.scene['group_name']
        request.link_name = self.tool
        request.waypoints = [pose_at(xyz, downward=True)]
        request.max_step = 0.003
        request.jump_threshold = 2.0
        request.avoid_collisions = bool(avoid_collisions)
        response = await_future(self.cartesian.call_async(request), 20.0)
        if response.error_code.val != MoveItErrorCodes.SUCCESS or response.fraction < 0.9999:
            return f'PLANNING_FAILED: Cartesian fraction={response.fraction:.4f}'
        trajectory = response.solution
        points = trajectory.joint_trajectory.points
        if not points:
            return 'PLANNING_FAILED: empty Cartesian trajectory'
        # Humble service times the MoveIt trajectory at default velocity. Slow it
        # down, preserving positions; v scales by s, acceleration by s squared.
        scale = min(float(self.scene['velocity_scaling']),
                    math.sqrt(float(self.scene['acceleration_scaling'])))
        previous = None
        previous_time = -1.0
        for point in points:
            # Recheck post-time-parameterization samples against the full scene.
            # Include interpolated states to reject large joint jumps/shortcuts.
            if previous is not None:
                delta = max(abs(a-b) for a, b in zip(point.positions, previous))
                if delta > 0.5:
                    return 'PLANNING_FAILED: Cartesian joint jump'
                count = max(1, math.ceil(delta / 0.02))
            else:
                count = 1
            for index in range(1, count + 1):
                check = GetStateValidity.Request()
                check.group_name = self.scene['group_name']
                check.robot_state.is_diff = True
                check.robot_state.joint_state.name = trajectory.joint_trajectory.joint_names
                check.robot_state.joint_state.position = list(point.positions) if previous is None else [
                    a + (b-a)*index/count for a, b in zip(previous, point.positions)]
                if avoid_collisions and not await_future(self.validity.call_async(check), 5.0).valid:
                    return 'PLANNING_FAILED: post-timing state invalid'
            previous = list(point.positions)
            seconds = (point.time_from_start.sec + point.time_from_start.nanosec/1e9) / scale
            if seconds <= previous_time:
                return 'PLANNING_FAILED: non-increasing Cartesian timing'
            previous_time = seconds
            nanos = round(seconds * 1e9)
            point.time_from_start.sec, point.time_from_start.nanosec = divmod(nanos, 1000000000)
            point.velocities = [v * scale for v in point.velocities]
            point.accelerations = [a * scale * scale for a in point.accelerations]
        return self._run_goal(self.execute_trajectory, ExecuteTrajectory.Goal(trajectory=trajectory))

    def _smooth_to(self, xyz, avoid_collisions=False):
        """Use Cartesian motion first, then a constrained MoveIt fallback.

        A straight vertical IK interpolation can lose its solution at the
        centre zone (especially for the optional plus cube).  The fallback
        keeps the same downward pose and velocity limits but lets MoveIt find
        a nearby valid joint path instead of aborting at a partial fraction.
        """
        status = self._cartesian_to(xyz, avoid_collisions=avoid_collisions)
        if status.startswith('PLANNING_FAILED: Cartesian fraction='):
            self.node.get_logger().warning(
                'Cartesian segment incomplete; retrying constrained MoveIt path')
            return self._move_to(xyz)
        return status

    def _set_gripper(self, left, right):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ['gripper_left_joint', 'gripper_right_joint']
        # Include an explicit common starting point. This prevents one joint
        # from beginning its spline before the other when their last feedback
        # samples arrive a few milliseconds apart.
        with self.gripper_state_lock:
            start_positions = (list(self.gripper_feedback)
                               if self.gripper_feedback_valid
                               else list(self.gripper_positions))
        start = JointTrajectoryPoint()
        start.positions = list(map(float, start_positions))
        start.velocities = [0.0, 0.0]
        start.time_from_start.nanosec = 100_000_000
        target = JointTrajectoryPoint()
        target.positions = [float(left), float(right)]
        target.velocities = [0.0, 0.0]
        # Give the simulated jaws enough time to open/close without a snap.
        target.time_from_start.sec = 1
        target.time_from_start.nanosec = 500_000_000
        goal.trajectory.points = [start, target]
        last_error = None
        for attempt in range(3):
            try:
                handle = await_future(self.gripper.send_goal_async(goal), 10.0)
                if not handle.accepted:
                    return 'FAILED: gripper goal rejected'
                result = await_future(handle.get_result_async(), 15.0)
                if result.status == GoalStatus.STATUS_SUCCEEDED and result.result.error_code == 0:
                    target_positions = [float(left), float(right)]
                    self.gripper_positions = target_positions
                    # Wait for both feedback samples to reach the target. This
                    # catches a joint that is still settling after its action
                    # result arrives and keeps release visually simultaneous.
                    deadline = time.monotonic() + 0.5
                    while time.monotonic() < deadline:
                        with self.gripper_state_lock:
                            settled = (self.gripper_feedback_valid and
                                       max(abs(a-b) for a, b in
                                           zip(self.gripper_feedback, target_positions)) < 8e-4)
                        if settled:
                            break
                        time.sleep(0.01)
                    return 'SUCCESS'
                return f'FAILED: gripper error={result.result.error_code}, status={result.status}'
            except RuntimeError as exc:
                last_error = exc
                # The controller can accept a goal while DDS delays the goal
                # response during startup. Retry the idempotent open/close goal.
                if attempt < 2:
                    time.sleep(0.4)
        return 'FAILED: gripper timeout: ' + str(last_error)

    def open_gripper(self):
        return self._set_gripper(0.0, 0.0)

    def close_gripper(self):
        # Both joints use the same command; the right joint axis is reversed
        # in the URDF so equal values move the jaws symmetrically inward.
        # Leave a 1.5 mm safety margin on each side of the 35 mm cube so
        # Gazebo contact on one jaw cannot hold the other jaw back.
        return self._set_gripper(-0.011, -0.011)

    def _send_arm_home(self):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(self.scene['home_joints'])
        point = JointTrajectoryPoint()
        point.positions = [float(self.scene['home_joints'][name])
                           for name in goal.trajectory.joint_names]
        point.velocities = [0.0] * len(point.positions)
        point.time_from_start.sec = 3
        goal.trajectory.points = [point]
        try:
            handle = await_future(self.arm_controller.send_goal_async(goal), 10.0)
            if not handle.accepted:
                return 'FAILED: arm home goal rejected'
            result = await_future(handle.get_result_async(), 30.0)
        except RuntimeError as exc:
            return 'FAILED: arm home timeout: ' + str(exc)
        if result.status == GoalStatus.STATUS_SUCCEEDED and result.result.error_code == 0:
            return 'SUCCESS'
        return f'FAILED: arm home error={result.result.error_code}, status={result.status}'

    def home(self):
        if self.held:
            return 'FAILED: không home khi đang giữ vật'
        gripper_status = self.open_gripper()
        if gripper_status != 'SUCCESS':
            return gripper_status
        return self._send_arm_home()

    def reset_scene(self):
        """Return robot and every cube to the original scene configuration.

        This is intentionally separate from ``home``: home only moves the arm,
        while the teleop reset also restores the Gazebo and MoveIt object poses.
        The planner calls it only while no task worker owns the busy lock.
        """
        if not self.ready:
            return 'NOT_READY'
        held_obj = self.held
        with self.sync_lock:
            self.held = None
            pending = self.sync_future
            self.sync_future = None
            self.sync_error = None
        if pending is not None:
            result = await_future(pending, 5.0)
            if not result.success:
                return 'SIMULATION_FAILED: không đồng bộ được vật đang giữ'

        # If reset is pressed after an interrupted pick, remove the attachment
        # before rebuilding the complete world collision scene below.
        if held_obj is not None:
            initial = (self.plus_parking_position if held_obj == self.plus_object
                       else self.initial_positions[held_obj])
            self._set_entity(held_obj, pose_at(initial))
            if held_obj in self.initial_positions:
                detach = AttachedCollisionObject(link_name=self.tool)
                detach.object.id = held_obj
                detach.object.operation = CollisionObject.REMOVE
                self._apply_scene([self._box(held_obj, initial, [self.size] * 3)], [detach])

        gripper_status = self.open_gripper()
        if gripper_status != 'SUCCESS':
            return gripper_status
        arm_status = self._send_arm_home()
        if arm_status != 'SUCCESS':
            return arm_status

        self.positions = {name: list(xyz) for name, xyz in self.initial_positions.items()}
        for name, xyz in self.positions.items():
            self._set_entity(name, pose_at(xyz))
        self._set_entity(self.plus_object, pose_at(self.plus_parking_position))
        objects = [self._box('work_table', self.scene['table']['center'], self.scene['table']['size']),
                   self._box('floor', [0.0, 0.0, -0.07], [3.0, 3.0, 0.02])]
        objects += [self._box(name, xyz, [self.size] * 3)
                    for name, xyz in self.positions.items()]
        removed_plus = CollisionObject()
        removed_plus.id = self.plus_object
        removed_plus.operation = CollisionObject.REMOVE
        objects.append(removed_plus)
        self._apply_scene(objects)
        self.plus_active = False
        self.plus_zone = None
        self.plus_staged = False
        self.last_sync = 0.0
        return 'SUCCESS'

    def pick(self, obj):
        if obj not in self.positions:
            return 'INVALID_OBJECT'
        if self.held:
            return 'FAILED: đã giữ vật'
        xyz = self.positions[obj]
        gripper_status = self.open_gripper()
        if gripper_status != 'SUCCESS':
            return gripper_status
        # Grasp from the top: the fingers extend down the cube sides.
        # The extra clearance keeps the fingertips from scraping the table.
        contact = [xyz[0], xyz[1], xyz[2] + self.size / 2 + self.scene['release_clearance']]
        above = [contact[0], contact[1], contact[2] + self.scene['approach_height']]
        status = self._move_to(above)
        if status != 'SUCCESS':
            return status
        status = self._smooth_to(contact, avoid_collisions=False)
        if status != 'SUCCESS':
            return status
        attached = AttachedCollisionObject()
        attached.link_name = self.tool
        attached.touch_links = [self.tool, 'gripper_base', 'gripper_left_finger', 'gripper_right_finger', 'tool0']
        attached.object = self._box(obj, [0.0, 0.0, self.size/2 + self.scene['release_clearance']],
                                      [self.size]*3, self.tool)
        gripper_status = self.close_gripper()
        if gripper_status != 'SUCCESS':
            return gripper_status
        self._apply_scene(attached=[attached])
        self._set_entity(obj, self._held_pose())
        with self.sync_lock:
            self.held = obj
            self.last_sync = time.monotonic()
        return self._smooth_to(above, avoid_collisions=False)

    def place(self, obj, zone):
        if obj not in self.positions:
            return 'INVALID_OBJECT'
        if zone != 'temporary_zone' and zone not in self.scene['zones']:
            return 'INVALID_ZONE'
        if self.held != obj:
            return 'FAILED: không giữ đúng vật'
        xyz = (list(self.scene['plus_object']['temporary_zone'])
               if zone == 'temporary_zone' else self.scene['zones'][zone])
        # Lower until the jaws surround the cube at the same height used by pick.
        contact = [xyz[0], xyz[1], xyz[2] + self.size/2 + self.scene['release_clearance']]
        above = [contact[0], contact[1], contact[2] + self.scene['approach_height']]
        for target in (above, contact):
            motion = self._smooth_to
            status = motion(target, avoid_collisions=False)
            if status != 'SUCCESS':
                return status
        # Pause at the release height so the cube is motionless before opening.
        time.sleep(0.35)
        gripper_status = self.open_gripper()
        if gripper_status != 'SUCCESS':
            return gripper_status
        # Stop the update loop and drain the last async request before release.
        with self.sync_lock:
            self.held = None
            pending = self.sync_future
            self.sync_future = None
        if pending is not None:
            if not await_future(pending, 5.0).success:
                raise RuntimeError('SIMULATION_FAILED: last attachment update failed')
        if self.sync_error:
            return self.sync_error
        self._set_entity(obj, pose_at(xyz))
        detach = AttachedCollisionObject(link_name=self.tool)
        detach.object.id = obj
        detach.object.operation = CollisionObject.REMOVE
        self._apply_scene([self._box(obj, xyz, [self.size]*3)], [detach])
        self.positions[obj] = list(xyz)
        # Let the released cube and the joint controller settle before retreat.
        time.sleep(0.45)
        return self._smooth_to(above, avoid_collisions=False)

    def _publish_zones(self):
        markers = []
        for index, (zone, xyz) in enumerate(self.scene['zones'].items()):
            marker = Marker()
            marker.header.frame_id = self.frame
            marker.ns, marker.id = 'zones', index
            marker.type, marker.action = Marker.TEXT_VIEW_FACING, Marker.ADD
            marker.pose = pose_at([xyz[0], xyz[1], xyz[2] + 0.035])
            marker.text = zone.replace('zone_', 'Zone ').upper()
            marker.scale.z = 0.035
            marker.color.r = marker.color.g = marker.color.b = marker.color.a = 1.0
            markers.append(marker)
        self.markers.publish(MarkerArray(markers=markers))
