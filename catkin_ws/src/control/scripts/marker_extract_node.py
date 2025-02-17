#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Pose
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import Bool

# ROS 초기화
rospy.init_node('normal_vector_marker_node', anonymous=True)

# Marker 퍼블리셔 설정
marker_pub = rospy.Publisher('visualization_marker_normal', Marker, queue_size=10)

# 글로벌 변수로 위치와 법선 벡터 저장
position = None
normal_vector = None

robot_moving = False

def robot_moving_callback(msg):
    global robot_moving
    robot_moving = msg.data

rospy.Subscriber('/robot_moving_status', Bool, robot_moving_callback)

def publish_marker():
    global position, normal_vector, robot_moving

    if robot_moving:
        rospy.loginfo("Robot is moving. Stopping marker publishing.")
        return
    # 기존 마커 생성 및 퍼블리싱 로직


def pose_callback(msg):
    global position
    # 위치 업데이트
    position = [msg.position.x, msg.position.y, msg.position.z]

def normal_vector_callback(msg):
    global normal_vector
    # 법선 벡터 업데이트
    normal_vector = [msg.position.x, msg.position.y, msg.position.z]
def publish_marker():
    global position, normal_vector, robot_moving

    if robot_moving:  # 로봇이 움직일 때 퍼블리싱 중단
        rospy.loginfo("Robot is moving. Stopping marker publishing.")
        rospy.sleep(3)
        return

    if position is not None and normal_vector is not None:
        # 마커 생성
        marker = Marker()
        marker.header.frame_id = "base"
        marker.header.stamp = rospy.Time.now()
        marker.type = Marker.ARROW
        marker.action = Marker.ADD

        start_point = Point(position[0], position[1], position[2])
        end_point = Point(
            position[0] - 0.2 * normal_vector[0],
            position[1] - 0.2 * normal_vector[1],
            position[2] + 0.2 * normal_vector[2]
        )
        marker.points = [start_point, end_point]

        # 마커 스타일 설정
        marker.scale.x = 0.005
        marker.scale.y = 0.01
        marker.scale.z = 0.01
        marker.color.r = 1.0
        marker.color.a = 1.0

        rospy.loginfo(f"Publishing marker from {start_point} to {end_point}.")
        marker_pub.publish(marker)

# 토픽 구독 설정
rospy.Subscriber('/detected_object_pose', Pose, pose_callback)
rospy.Subscriber('/detected_normal_vector', Pose, normal_vector_callback)

# 루프 실행
rate = rospy.Rate(10)  # 10Hz
while not rospy.is_shutdown():
    publish_marker()
    rate.sleep()
