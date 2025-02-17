#!/usr/bin/env python3
import rospy
import sys
import moveit_commander
import geometry_msgs.msg
from geometry_msgs.msg import Pose
from std_msgs.msg import String
import numpy as np
from scipy.spatial.transform import Rotation as R
from std_msgs.msg import Bool
import time
robot_moving_pub = rospy.Publisher('/robot_moving_status', Bool, queue_size=10)

last_received_pose = None  # 마지막으로 받은 좌표 저장 (새로운 좌표를 감지하기 위해 사용)
pose_updated_during_execution = False  # 동작 중 새로운 좌표가 들어왔는지 확인하는 플래그


def set_robot_moving(state):
    robot_moving_pub.publish(Bool(data=state))

class MoveUR3e:
    def __init__(self):
        # Initialize the moveit_commander
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('move_ur3e_node', anonymous=True)

        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group = moveit_commander.MoveGroupCommander("manipulator")

        # Set planning time and limit speed
        self.group.set_planning_time(10.0)
        self.group.set_max_velocity_scaling_factor(0.1)
        self.group.set_max_acceleration_scaling_factor(0.1)

        # Set initial pose
        self.initial_joint_goal = self.group.get_current_joint_values()
        self.initial_pose = self.group.get_current_pose().pose  # 초기 포즈 저장

        self.set_initial_pose()

        # ROS Subscribers and Publishers
        self.pose_sub = rospy.Subscriber('/detected_normal_pose', Pose, self.update_pose)
        self.status_pub = rospy.Publisher('/move_ur3e_status', String, queue_size=10)
        self.arduino_pub = rospy.Publisher('/arduino_command', String, queue_size=10)

        self.current_pose = None
        self.current_orientation = None

    def send_arduino_command(self):
        """아두이노 명령 전송."""
        command = String()
        command.data = "RUN_MOTOR"
        self.arduino_pub.publish(command)
        rospy.loginfo("Sent command to Arduino to run motor")

    def send_arduino_command2(self):
        """아두이노 명령 전송."""
        command = String()
        command.data = "STOP_MOTOR"
        self.arduino_pub.publish(command)
        rospy.loginfo("Sent command to Arduino to stop motor")

    def set_initial_pose(self):
        """초기 포즈로 이동."""
        joint_goal = self.initial_joint_goal
        joint_goal[0] = 1.57
        joint_goal[2] = 1.57 / 2
        joint_goal[3] = -1.57 / 2
        joint_goal[4] = -1.57

        self.group.go(joint_goal, wait=True)
        self.group.stop()
        rospy.loginfo("Initial pose set.")

    def update_pose(self, pose_msg):
        """법선 벡터와 위치 업데이트 (동작 중에는 무시)."""
        global pose_updated_during_execution

        if pose_updated_during_execution:
            return  # 동작 중이면 좌표 업데이트 무시

        self.current_pose = pose_msg.position
        self.current_orientation = pose_msg.orientation

    def move_to_specific_pose(self):
        """z값은 고정하고 x, y 좌표로 카르테시안 경로 계획."""
        if self.current_pose is None:
            rospy.logwarn("No position data received yet. Skipping movement.")
            return

        waypoints = []
        start_pose = self.group.get_current_pose().pose

        # 현재 위치와 목표 위치 설정
        rospy.loginfo("Starting Cartesian path planning for x, y movement.")
        rospy.loginfo(f"Initial pose: {start_pose}")

        # 웨이포인트 생성
        target_pose = geometry_msgs.msg.Pose()
        target_pose.position.x = - self.current_pose.x
        target_pose.position.y = - self.current_pose.y
        target_pose.position.z = self.initial_pose.position.z -0.2# z값 고정
        target_pose.orientation = start_pose.orientation

        rospy.loginfo(
            f"Target 3D Coordinates: x={target_pose.position.x}, y={target_pose.position.y}, z={target_pose.position.z}")
        rospy.loginfo(
            f"Target orientation: x={target_pose.orientation.x}, y={target_pose.orientation.y}, z={target_pose.orientation.z}, w={target_pose.orientation.w}")

        waypoints.append(target_pose)

        # Path constraints 초기화
        self.group.clear_path_constraints()

        # Cartesian 경로 계획
        try:
            (plan, fraction) = self.group.compute_cartesian_path(
                waypoints,  # 웨이포인트 목록
                eef_step=0.01,  # End-effector의 최대 이동 거리
            )
        except Exception as e:
            rospy.logerr(f"Error during compute_cartesian_path: {e}")
            return

        rospy.loginfo(f"Cartesian path planning fraction: {fraction}")
        if fraction > 0.7:
            rospy.loginfo("Cartesian path successfully planned.")

            # 경로 속도 조정
            plan = self.scale_plan_speed(plan, 0.8)
            rospy.loginfo("Scaled plan speed.")

            try:
                self.group.execute(plan, wait=True)
                rospy.loginfo("Cartesian path executed successfully.")
            except Exception as e:
                rospy.logerr(f"Error during execution of Cartesian path: {e}")
        else:
            rospy.logwarn(f"Failed to plan Cartesian path. Fraction achieved: {fraction}")

    def move_to_specific_pose2(self, x, y):
        """z값은 고정하고 x, y 좌표로 카르테시안 경로 계획."""
        waypoints = []
        start_pose = self.group.get_current_pose().pose

        # 현재 위치와 목표 위치 설정
        rospy.loginfo("Starting Cartesian path planning for x, y movement.")
        rospy.loginfo(f"Initial pose: {start_pose}")

        # 웨이포인트 생성
        target_pose = geometry_msgs.msg.Pose()
        target_pose.position.x = x
        target_pose.position.y = y
        target_pose.position.z = self.initial_pose.position.z - 0.1
        target_pose.orientation = start_pose.orientation

        waypoints.append(target_pose)

        # Path constraints 초기화
        self.group.clear_path_constraints()

        # Cartesian 경로 계획
        try:
            (plan, fraction) = self.group.compute_cartesian_path(
                waypoints,  # 웨이포인트 목록
                eef_step=0.01,  # End-effector의 최대 이동 거리
            )
        except Exception as e:
            rospy.logerr(f"Error during compute_cartesian_path: {e}")
            return

        rospy.loginfo(f"Cartesian path planning fraction: {fraction}")
        if fraction > 0.6:
            rospy.loginfo("Cartesian path successfully planned.")

            # 경로 속도 조정
            plan = self.scale_plan_speed(plan, 0.8)
            rospy.loginfo("Scaled plan speed.")

            try:
                self.group.execute(plan, wait=True)
                rospy.loginfo("Cartesian path executed successfully.")
            except Exception as e:
                rospy.logerr(f"Error during execution of Cartesian path: {e}")
        else:
            rospy.logwarn(f"Failed to plan Cartesian path. Fraction achieved: {fraction}")

    def return_to_initial_pose_cartesian(self):
        """카르테시안 패스를 통해 초기 포즈로 돌아가기."""
        waypoints = []

        # 현재 로봇의 시작 포즈 가져오기
        current_pose = self.group.get_current_pose().pose

        # 초기 포즈를 목표로 설정
        target_pose = geometry_msgs.msg.Pose()
        target_pose.position.x = self.initial_pose.position.x
        target_pose.position.y = self.initial_pose.position.y
        target_pose.position.z = self.initial_pose.position.z
        target_pose.orientation = self.initial_pose.orientation

        waypoints.append(target_pose)

        # 카르테시안 경로 계산
        try:
            (plan, fraction) = self.group.compute_cartesian_path(
                waypoints,  # 경로에 포함될 웨이포인트
                eef_step=0.01,  # 엔드 이펙터의 최대 이동 거리
            )
        except Exception as e:
            rospy.logerr(f"Error during compute_cartesian_path: {e}")
            return

        rospy.loginfo(f"Cartesian path planning fraction: {fraction}")
        if fraction > 0.3:
            rospy.loginfo("Cartesian path successfully planned.")

            # 경로 속도 조정
            plan = self.scale_plan_speed(plan, 0.8)

            # 계획 실행
            try:
                self.group.execute(plan, wait=True)
                rospy.loginfo("Returned to initial pose via Cartesian path.")
            except Exception as e:
                rospy.logerr(f"Error during execution of Cartesian path: {e}")
        else:
            rospy.logwarn(f"Failed to plan Cartesian path. Fraction achieved: {fraction}")

    def move_to_6d_pose_cartesian(self):
        set_robot_moving(True)
        try:
            """6D 좌표를 추종하는 카르테시안 경로 생성."""
            if self.current_pose is None or self.current_orientation is None:
                rospy.logwarn("No pose or orientation received yet.")
                return

            waypoints = []

            # 현재 로봇의 시작 포즈 가져오기
            start_pose = self.group.get_current_pose().pose

            # 목표 포즈 설정
            target_pose = geometry_msgs.msg.Pose()
            target_pose.orientation.x = self.current_orientation.x
            target_pose.orientation.y = self.current_orientation.y
            target_pose.orientation.z = self.current_orientation.z
            target_pose.orientation.w = self.current_orientation.w
            target_pose.position.x = -self.current_pose.x
            target_pose.position.y = -self.current_pose.y
            target_pose.position.z = self.current_pose.z

            rospy.loginfo(
                f"Target 3D Coordinates: x={target_pose.position.x}, y={target_pose.position.y}, z={target_pose.position.z}")
            rospy.loginfo(
                f"Target orientation: x={target_pose.orientation.x}, y={target_pose.orientation.y}, z={target_pose.orientation.z}, w={target_pose.orientation.w}")

            waypoints.append(target_pose)

            # 카르테시안 경로 계산
            try:
                (plan, fraction) = self.group.compute_cartesian_path(
                    waypoints,  # 경로에 포함될 웨이포인트
                    eef_step=0.01,  # 엔드 이펙터의 최대 이동 거리
                )
            except Exception as e:
                rospy.logerr(f"Error during compute_cartesian_path: {e}")
                return

            rospy.loginfo(f"Cartesian path planning fraction: {fraction}")
            if fraction > 0.7:
                rospy.loginfo("Cartesian path successfully planned.")

                # 경로 속도 조정
                plan = self.scale_plan_speed(plan, 0.8)

                # 계획 실행
                try:
                    self.group.execute(plan, wait=True)
                    rospy.loginfo("Cartesian path executed successfully.")
                except Exception as e:
                    rospy.logerr(f"Error during execution of Cartesian path: {e}")
            else:
                rospy.logwarn(f"Failed to plan Cartesian path. Fraction achieved: {fraction}")
        finally:
            set_robot_moving(False)


    def move_down_z_axis(self, z_offset, steps):
        """z축 방향으로 단계별 이동."""
        waypoints = []
        start_pose = self.group.get_current_pose().pose

        rospy.loginfo("Starting Cartesian path planning for Z-axis movement.")
        rospy.loginfo(f"Initial pose: {start_pose}")

        # 웨이포인트 생성
        for i in range(steps):
            next_pose = geometry_msgs.msg.Pose()
            next_pose.position.x = start_pose.position.x
            next_pose.position.y = start_pose.position.y
            next_pose.position.z = start_pose.position.z + (z_offset / steps) * (i + 1)
            next_pose.orientation.x = -1
            next_pose.orientation.y = 0
            next_pose.orientation.z = 0
            next_pose.orientation.w = 0

            waypoints.append(next_pose)
            rospy.loginfo(f"Waypoint {i + 1}/{steps}: {next_pose}")

        # Path constraints 초기화
        self.group.clear_path_constraints()

        # Cartesian 경로 계획
        try:
            (plan, fraction) = self.group.compute_cartesian_path(
                waypoints,  # 웨이포인트 목록
                eef_step=0.01  # End-effector의 최대 이동 거리
            )
        except Exception as e:
            rospy.logerr(f"Error during compute_cartesian_path: {e}")
            return

        rospy.loginfo(f"Cartesian path planning fraction: {fraction}")
        if fraction > 0.5:
            rospy.loginfo("Cartesian path successfully planned.")

            # 경로 속도 조정
            plan = self.scale_plan_speed(plan, 0.8)
            rospy.loginfo("Scaled plan speed.")

            try:
                self.group.execute(plan, wait=True)
                rospy.loginfo("Cartesian path executed successfully.")
            except Exception as e:
                rospy.logerr(f"Error during execution of Cartesian path: {e}")
        else:
            rospy.logwarn(f"Failed to plan Cartesian path. Fraction achieved: {fraction}")
    def move_down_x_axis(self, x_offset, steps):
        """z축 방향으로 단계별 이동."""
        waypoints = []
        start_pose = self.group.get_current_pose().pose

        rospy.loginfo("Starting Cartesian path planning for Z-axis movement.")
        rospy.loginfo(f"Initial pose: {start_pose}")

        # 웨이포인트 생성
        for i in range(steps):


            next_pose = geometry_msgs.msg.Pose()
            next_pose.position.x = start_pose.position.x + (x_offset / steps) * (i + 1)
            next_pose.position.y = start_pose.position.y
            next_pose.position.z = start_pose.position.z
            next_pose.orientation.x = -1
            next_pose.orientation.y = 0
            next_pose.orientation.z = 0
            next_pose.orientation.w = 0

            waypoints.append(next_pose)
            rospy.loginfo(f"Waypoint {i + 1}/{steps}: {next_pose}")

        # Path constraints 초기화
        self.group.clear_path_constraints()

        # Cartesian 경로 계획
        try:
            (plan, fraction) = self.group.compute_cartesian_path(
                waypoints,  # 웨이포인트 목록
                eef_step=0.01  # End-effector의 최대 이동 거리
            )
        except Exception as e:
            rospy.logerr(f"Error during compute_cartesian_path: {e}")
            return

        rospy.loginfo(f"Cartesian path planning fraction: {fraction}")
        if fraction > 0.7:
            rospy.loginfo("Cartesian path successfully planned.")

            # 경로 속도 조정
            plan = self.scale_plan_speed(plan, 0.8)
            rospy.loginfo("Scaled plan speed.")

            try:
                self.group.execute(plan, wait=True)
                rospy.loginfo("Cartesian path executed successfully.")
            except Exception as e:
                rospy.logerr(f"Error during execution of Cartesian path: {e}")
        else:
            rospy.logwarn(f"Failed to plan Cartesian path. Fraction achieved: {fraction}")

    def move_to_6d_pose(self):
        """법선 벡터를 포함한 6D 좌표로 한 번에 이동."""

        if self.current_pose is None or self.current_orientation is None:
            rospy.logwarn("No pose or orientation received yet")
            return

        # 목표 포즈 정의
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.position.x = -self.current_pose.x
        pose_goal.position.y = -self.current_pose.y
        pose_goal.position.z = self.current_pose.z
        pose_goal.orientation.x = self.current_orientation.x
        pose_goal.orientation.y = self.current_orientation.y
        pose_goal.orientation.z = self.current_orientation.z
        pose_goal.orientation.w = self.current_orientation.w

        self.group.set_pose_target(pose_goal)
        rospy.loginfo(f"6D Pose target set: {pose_goal}")

        plan = self.group.plan()
        if plan[1]:
            self.group.execute(plan[1], wait=True)
            rospy.loginfo("Reached target 6D pose successfully")
        else:
            rospy.logwarn("Planning failed")
            self.publish_status("Move failed")

    def publish_status(self, status):
        """상태 메시지 전송."""
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)
    def scale_plan_speed(self, plan, scale_factor):
        """Scale the speed of the plan by a given factor."""
        for point in plan.joint_trajectory.points:
            point.time_from_start *= 1 / scale_factor
            point.velocities = [v * scale_factor for v in point.velocities]
            point.accelerations = [a * scale_factor for a in point.accelerations]
        return plan
def execute_movement_sequence():
    global pose_updated_during_execution, last_received_pose

    pose_updated_during_execution = True  # 동작 중 좌표 업데이트 차단

    move_ur3e.move_to_6d_pose_cartesian()
    rospy.sleep(1)
    move_ur3e.send_arduino_command()
    rospy.sleep(11)
    move_ur3e.move_down_z_axis(0.2, 1)
    rospy.sleep(1)
    move_ur3e.move_to_specific_pose2(0.3, 0.2)
    rospy.sleep(1)
    move_ur3e.move_down_z_axis(-0.15, 1)
    rospy.sleep(1)
    move_ur3e.send_arduino_command2()
    rospy.sleep(4)
    move_ur3e.move_down_z_axis(0.3, 1)
    rospy.sleep(1)
    move_ur3e.return_to_initial_pose_cartesian()
    rospy.sleep(1)

    last_received_pose = None
    move_ur3e.current_pose = None  # 이전 좌표 삭제 (새로운 좌표가 들어오기 전까지 무효)
    rospy.loginfo("Pose cleared after Pick-and-Place")
    pose_updated_during_execution = False  # 새로운 좌표 업데이트 허용

def wait_for_new_pose(timeout=5.0):
    """초기 위치에서 지정된 시간 동안 새로운 좌표가 들어오는지 확인"""
    global last_received_pose, pose_updated_during_execution
    start_time = time.time()

    while time.time() - start_time < timeout:
        if move_ur3e.current_pose is not None and move_ur3e.current_pose != last_received_pose:
            if not pose_updated_during_execution:  # 동작 중 업데이트된 좌표는 무시
                rospy.loginfo("New pose received. Moving to new position.")
                last_received_pose = move_ur3e.current_pose  # **마지막 받은 좌표 업데이트**
                return True
        rospy.sleep(0.1)  # 0.1초 대기하면서 계속 확인

    rospy.loginfo("No new pose received. Stopping execution.")
    return False  # **새 좌표가 들어오지 않으면 False 반환하여 동작 멈춤**

def arduino_callback(msg):
    global move_ur3e
    if msg.data == "run_system":
        rospy.sleep(1)
        rospy.loginfo("Arduino command 'run_system' received.")

        while True:  # **반복 루프**
            rospy.sleep(0.5)
            execute_movement_sequence()  # **로봇 동작 수행**
            rospy.sleep(0.5)

            # **초기 위치에서 새로운 좌표가 퍼블리시되는지 5초 동안 검사**
            if not wait_for_new_pose(5.0):
                rospy.loginfo("No new pose received. Stopping loop.")
                break  # **새 좌표가 없으면 루프 탈출**


if __name__ == '__main__':
    move_ur3e = MoveUR3e()

    # 추가된 ROS Subscriber
    rospy.Subscriber("/arduino_command", String, arduino_callback)

    rospy.loginfo("ROS node started. Waiting for commands from /arduino_command topic.")

    try:
        while not rospy.is_shutdown():
            key = input("Enter command: ")
            if key == 'z':

                rospy.loginfo("Exiting program.")
                break
            if key == 'b':
                while True:  # **반복 루프**
                    rospy.sleep(0.5)
                    execute_movement_sequence()  # **로봇 동작 수행**
                    rospy.sleep(0.5)


                    # **초기 위치에서 새로운 좌표가 퍼블리시되는지 5초 동안 검사**
                    if not wait_for_new_pose(5.0):
                        rospy.loginfo("No new pose received. Stopping loop.")
                        break  # **새 좌표가 없으면 루프 탈출**

            if key == 'n':
                execute_movement_sequence()
            elif key == 'w':
                move_ur3e.return_to_initial_pose_cartesian()

    except rospy.ROSInterruptException:
        pass
    finally:
        moveit_commander.roscpp_shutdown()

