#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Bool
import numpy as np

class PointCloudRepublisher:
    def __init__(self):
        rospy.init_node("roi_pointcloud_republisher", anonymous=True)

        # ROS 퍼블리셔
        self.pointcloud_pub = rospy.Publisher('/roi_pointcloud_new', PointCloud2, queue_size=10)

        # 로봇 움직임 상태 변수
        self.robot_moving = False
        self.pointcloud_list = []  # 여러 개의 객체 포인트 클라우드 저장 리스트

        # ROS 구독자
        rospy.Subscriber('/roi_pointcloud', PointCloud2, self.pointcloud_callback)
        rospy.Subscriber('/robot_moving_status', Bool, self.robot_moving_callback)

    def robot_moving_callback(self, msg):
        """로봇 움직임 상태를 업데이트합니다."""
        self.robot_moving = msg.data

    def pointcloud_callback(self, msg):
        """ROI PointCloud 토픽을 구독하여 데이터 누적."""
        cloud_points = list(pc2.read_points(msg, field_names=("x", "y", "z", "rgb"), skip_nans=True))
        if len(cloud_points) > 0:
            # 수신된 PointCloud 데이터를 누적
            self.pointcloud_list.append((msg.header, cloud_points))

    def merge_and_publish_pointcloud(self):
        rate = rospy.Rate(10)  # 10Hz 루프
        while not rospy.is_shutdown():
            if self.robot_moving:
                rospy.loginfo("Robot is moving. Stopping PointCloud publishing to /roi_pointcloud_new.")
                rospy.sleep(5)
            elif not self.robot_moving and self.pointcloud_list:
                merged_points = []
                header = None

                for h, points in self.pointcloud_list:
                    if header is None:
                        header = h  # 첫 번째 포인트 클라우드의 헤더 사용
                    merged_points.extend(points)

                fields = [
                    pc2.PointField('x', 0, pc2.PointField.FLOAT32, 1),
                    pc2.PointField('y', 4, pc2.PointField.FLOAT32, 1),
                    pc2.PointField('z', 8, pc2.PointField.FLOAT32, 1),
                    pc2.PointField('rgb', 12, pc2.PointField.UINT32, 1)
                ]
                merged_cloud = pc2.create_cloud(header, fields, merged_points)
                self.pointcloud_pub.publish(merged_cloud)
                rospy.loginfo("Published merged PointCloud with multiple objects to /roi_pointcloud_new.")
                self.pointcloud_list = []
            rate.sleep()


if __name__ == "__main__":
    try:
        republisher = PointCloudRepublisher()
        republisher.merge_and_publish_pointcloud()
    except rospy.ROSInterruptException:
        pass
