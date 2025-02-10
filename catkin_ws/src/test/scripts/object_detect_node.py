#!/usr/bin/env python3
import rospy
import tf2_ros
import cv2
import pyrealsense2 as rs
import numpy as np
import time
from geometry_msgs.msg import Pose, Vector3
from sensor_msgs.msg import Image, PointCloud2
from cv_bridge import CvBridge
from scipy.spatial.transform import Rotation as R
from ultralytics import YOLO
import sensor_msgs.point_cloud2 as pc2
import open3d as o3d
from geometry_msgs.msg import Point
from std_msgs.msg import Empty,Bool


class PublisherControl:
    def __init__(self):
        self.should_publish = True
        rospy.Subscriber("/stop_publishing", Empty, self.stop_callback)

    def stop_callback(self, msg):
        rospy.loginfo("Stop signal received. Stopping publishers.")
        self.should_publish = False
# YOLOv8 모델 로드
model_path = "./train/weights/best.pt"
model = YOLO(model_path)

# RealSense 파이프라인 설정
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
pipeline.start(config)

# Align object to align depth frames to color frames
align_to = rs.stream.color
align = rs.align(align_to)

# ROS 초기화
rospy.init_node('object_detection_node', anonymous=True)
pose_pub = rospy.Publisher('detected_object_pose', Pose, queue_size=10)
normal_pub = rospy.Publisher('detected_normal_pose', Pose, queue_size=10)
normal_vector_pub = rospy.Publisher('detected_normal_vector', Pose, queue_size=10)
roi_depth_pub = rospy.Publisher('roi_depth_image', Image, queue_size=10)
pointcloud_pub = rospy.Publisher('roi_pointcloud', PointCloud2, queue_size=10)



# 마지막 출력 시간 초기화
last_output_time = time.time()
bridge = CvBridge()
# TF 버퍼 초기화
tf_buffer = tf2_ros.Buffer()
tf_listener = tf2_ros.TransformListener(tf_buffer)

# 마지막 출력 시간 초기화
last_output_time = time.time()
bridge = CvBridge()


# TF 데이터를 가져와 카메라 → 로봇 베이스 변환 행렬 계산
def get_camera_to_base_transform():
    try:
        # TF에서 변환 가져오기 (tool0 → base)
        trans = tf_buffer.lookup_transform('base', 'tool0', rospy.Time(0), rospy.Duration(1.0))
        translation_tool0_to_base = [
            trans.transform.translation.x,
            trans.transform.translation.y,
            trans.transform.translation.z
        ]
        rotation_tool0_to_base = [
            trans.transform.rotation.x,
            trans.transform.rotation.y,
            trans.transform.rotation.z,
            trans.transform.rotation.w,
        ]

        # 런치 파일에서 정의된 카메라 → tool0 변환
        translation_camera_to_tool0 = [-0.0309928, 0.04142, 0.010657]
        quaternion_camera_to_tool0 = [0.00962063, 0.00225575, -0.00978338, 0.999903]

        # 회전 행렬 계산
        R_tool0_to_base = R.from_quat(rotation_tool0_to_base).as_matrix()
        R_camera_to_tool0 = R.from_quat(quaternion_camera_to_tool0).as_matrix()

        # 변환 행렬 구성
        T_tool0_to_base = np.eye(4)
        T_tool0_to_base[:3, :3] = R_tool0_to_base
        T_tool0_to_base[:3, 3] = translation_tool0_to_base

        T_camera_to_tool0 = np.eye(4)
        T_camera_to_tool0[:3, :3] = R_camera_to_tool0
        T_camera_to_tool0[:3, 3] = translation_camera_to_tool0

        # 카메라 → 베이스 변환
        T_camera_to_base = np.dot(T_tool0_to_base, T_camera_to_tool0)
        return T_camera_to_base, T_camera_to_tool0

    except Exception as e:
        rospy.logerr(f"Failed to get TF: {e}")
        return None, None



# ROI를 기준으로 컬러 포인트클라우드를 생성 및 퍼블리시하는 함수
def publish_roi_colored_pointcloud(depth_frame, color_frame, x1, y1, x2, y2, topic_name="roi_pointcloud", pose=None, normal_pose=None):
    # 로봇이 움직일 때 퍼블리싱 중단 (필요하다면 이 부분 유지)
    if normal_pose is not None:
        normal_vector_pub.publish(normal_pose)

    # ROI 영역의 포인트 클라우드 생성
    intrinsics = depth_frame.profile.as_video_stream_profile().intrinsics
    points = []

    # 바운딩 박스 내 픽셀 순회
    for v in range(y1, y2):
        for u in range(x1, x2):
            depth = depth_frame.get_distance(u, v)
            if depth > 0:
                # 픽셀 좌표를 3D 포인트로 변환
                point = rs.rs2_deproject_pixel_to_point(intrinsics, [u, v], depth)
                # 컬러 정보 가져오기 (BGR → RGB)
                color = color_frame[v, u]
                r, g, b = color[2], color[1], color[0]
                rgb = (int(r) << 16) | (int(g) << 8) | int(b)
                points.append([point[0], point[1], point[2], rgb])

    # ROS PointCloud2 메시지 생성
    header = rospy.Header()
    header.stamp = rospy.Time.now()
    header.frame_id = "camera_color_optical_frame"
    fields = [
        pc2.PointField('x', 0, pc2.PointField.FLOAT32, 1),
        pc2.PointField('y', 4, pc2.PointField.FLOAT32, 1),
        pc2.PointField('z', 8, pc2.PointField.FLOAT32, 1),
        pc2.PointField('rgb', 12, pc2.PointField.UINT32, 1)
    ]
    pointcloud_msg = pc2.create_cloud(header, fields, points)

    # 개별 토픽으로 퍼블리시 (새로운 퍼블리셔 생성)
    pub = rospy.Publisher(topic_name, PointCloud2, queue_size=10)
    rospy.sleep(0.1)  # 퍼블리셔 등록 대기 시간
    rospy.loginfo("Publishing ROI PointCloud on topic: " + topic_name)
    pub.publish(pointcloud_msg)


# 법선 벡터 추출 함수
def extract_normal_at_point(depth_frame, x, y, intrinsics, kernel_size=10):


    pc = rs.pointcloud()
    points = pc.calculate(depth_frame)
    verts = np.asanyarray(points.get_vertices()).view(np.float32).reshape(-1, 3)
    point_cloud = verts.reshape((intrinsics.height, intrinsics.width, 3))

    # 주변 영역 추출
    y_min, y_max = max(0, y - kernel_size), min(intrinsics.height, y + kernel_size + 1)
    x_min, x_max = max(0, x - kernel_size), min(intrinsics.width, x + kernel_size + 1)
    local_points = point_cloud[y_min:y_max, x_min:x_max].reshape(-1, 3)
    local_points = local_points[local_points[:, 2] > 0]  # 깊이 값이 유효한 포인트만

    # Open3D를 사용해 법선 벡터 계산
    if len(local_points) == 0:
        return None  # 유효한 포인트가 없으면 None 반환
    pc_o3d = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(local_points))
    pc_o3d.estimate_normals(o3d.geometry.KDTreeSearchParamKNN(20), fast_normal_computation=False)
    pc_o3d.orient_normals_to_align_with_direction(np.array([0., 0., -1.]))

    # 법선 벡터 평균 계산
    normals = np.asarray(pc_o3d.normals)
    normal_at_point = normals.mean(axis=0)  # 주변 영역의 평균 법선 벡터
    return normal_at_point


try:
    while not rospy.is_shutdown():
        # 로봇 동작 중이면 퍼블리시 중단


        T_camera_to_base, T_camera_to_tool0 = get_camera_to_base_transform()
        if T_camera_to_base is None or T_camera_to_tool0 is None:
            continue

        # 프레임 얻기
        frames = pipeline.wait_for_frames()
        aligned_frames = align.process(frames)

        depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()

        if not depth_frame or not color_frame:
            continue

        # Depth 이미지와 Color 이미지를 numpy 배열로 변환
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        # YOLOv8로 객체 탐지
        # YOLOv8로 객체 탐지
        # YOLOv8로 객체 탐지
        results = model(color_image, verbose=False)

        # 감지된 객체 리스트 초기화
        detected_objects = []

        roi_index = 1

        for result in results:
            for box in result.boxes:
                conf = box.conf[0]
                if conf >= 0.2:  # 신뢰도가 0.5 이상인 경우만
                    cls = int(box.cls[0])
                    label = model.names[cls]
                    if label in ["mug", "apple", "bottle"]:  # 관심 있는 객체만 처리
                        x1, y1, x2, y2 = map(int, box.xyxy[0])
                        cx = (x1 + x2) // 2
                        cy = (y1 + y2) // 2
                        depth = depth_frame.get_distance(cx, cy)

                        if depth > 0:
                            # 각 객체에 대해 고유 토픽 이름 생성
                            topic_name = "/roi_pointcloud_{}".format(roi_index)
                            publish_roi_colored_pointcloud(depth_frame, color_image, x1, y1, x2, y2, topic_name)
                            roi_index += 1

                            # 3D 좌표 계산 및 기타 처리 (기존 코드 유지)
                            closest_3d_coords = rs.rs2_deproject_pixel_to_point(
                                depth_frame.profile.as_video_stream_profile().intrinsics,
                                [cx, cy], depth
                            )
                            camera_coords = np.array(
                                [closest_3d_coords[0], closest_3d_coords[1], closest_3d_coords[2], 1.0])
                            robot_base_coords = np.dot(T_camera_to_base, camera_coords)
                            distance = np.linalg.norm(robot_base_coords[:3])
                            detected_objects.append({
                                "distance": distance,
                                "bounding_box": (x1, y1, x2, y2),
                                "center": (cx, cy),
                                "label": label,
                                "robot_base_coords": robot_base_coords
                            })

        # 거리순으로 정렬
        detected_objects.sort(key=lambda obj: obj["distance"])

        # 가장 가까운 객체 퍼블리시
        if detected_objects:
            closest_object = detected_objects[0]
            robot_base_coords = closest_object["robot_base_coords"]
            cx, cy = closest_object["center"]

            # 법선 벡터 계산
            normal_vector = extract_normal_at_point(
                depth_frame, cx, cy,
                depth_frame.profile.as_video_stream_profile().get_intrinsics()
            )

            if time.time() - last_output_time >= 0.01:
                last_output_time = time.time()

                # 로봇 베이스 좌표 퍼블리시
                pose = Pose()
                pose.position.x = robot_base_coords[0]
                pose.position.y = robot_base_coords[1]
                pose.position.z = robot_base_coords[2]
                pose_pub.publish(pose)

                if normal_vector is not None:

                    normal_vector[0] = -normal_vector[0]
                    normal_vector[1] = -normal_vector[1]
                    normal_vector_homogeneous = np.append(normal_vector, 0)
                    normal_vector_tool0 = np.dot(T_camera_to_tool0[:3, :3], normal_vector_homogeneous[:3])
                    normal_vector_base = np.dot(T_camera_to_base[:3, :3], normal_vector_tool0)

                    # 법선 벡터 퍼블리시
                    normal_pose = Pose()
                    normal_pose.position.x = normal_vector_base[0]
                    normal_pose.position.y = normal_vector_base[1]
                    normal_pose.position.z = normal_vector_base[2]
                    normal_vector_pub.publish(normal_pose)

                    # 쿼터니언 변환 및 퍼블리시
                    normal_vector_base = normal_vector_base / np.linalg.norm(normal_vector_base)
                    default_direction = np.array([0, 0, -1])
                    rotation_vector = np.cross(default_direction, normal_vector_base)
                    angle = np.arccos(np.clip(np.dot(default_direction, normal_vector_base), -1.0, 1.0))
                    if np.linalg.norm(rotation_vector) > 0:
                        rotation_vector = rotation_vector / np.linalg.norm(rotation_vector) * angle
                    else:
                        rotation_vector = np.zeros(3)
                    quaternion = R.from_rotvec(rotation_vector).as_quat()

                    normal_quaternion_pose = Pose()
                    normal_quaternion_pose.position.x = robot_base_coords[0]
                    normal_quaternion_pose.position.y = robot_base_coords[1]
                    normal_quaternion_pose.position.z = robot_base_coords[2]
                    normal_quaternion_pose.orientation.x = quaternion[0]
                    normal_quaternion_pose.orientation.y = quaternion[1]
                    normal_quaternion_pose.orientation.z = quaternion[2]
                    normal_quaternion_pose.orientation.w = quaternion[3]
                    normal_pub.publish(normal_quaternion_pose)
                    rospy.loginfo(
                        "\033[1;34m"  # 굵게(Bold) + 파란색
                        "\n    ========================================"
                        "\n           Detected Object Information"
                        "\n    ========================================"
                        "\n          Position (Robot Base Frame):"
                        f"\n            X: {robot_base_coords[0]:.5f} m"
                        f"\n            Y: {robot_base_coords[1]:.5f} m"
                        f"\n            Z: {robot_base_coords[2]:.5f} m"
                        "\n    ----------------------------------------"
                        "\n          Quaternion Orientation:"
                        f"\n            X: {quaternion[0]:.5f}"
                        f"\n            Y: {quaternion[1]:.5f}"
                        f"\n            Z: {quaternion[2]:.5f}"
                        f"\n            W: {quaternion[3]:.5f}"
                        "\n    ========================================"
                        "\033[0m"  # 색상 및 스타일 초기화
                    )

        # 거리 순서에 따라 바운딩 박스와 라벨 및 법선 벡터 표시
        for rank, obj in enumerate(detected_objects, start=1):
            x1, y1, x2, y2 = obj["bounding_box"]
            cx, cy = obj["center"]
            label = obj["label"]

            # 바운딩 박스 색상 설정: 1등은 붉은색, 나머지는 초록색
            box_color = (0, 0, 255) if rank == 1 else (0, 255, 0)

            # 바운딩 박스와 순위 텍스트 표시
            cv2.circle(color_image, (cx, cy), 5, (0, 0, 255), -1)
            cv2.rectangle(color_image, (x1, y1), (x2, y2), box_color, 2)
            cv2.putText(
                color_image,
                f"{rank}: {label}",
                (x1, y1 - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                box_color,
                2
            )

            # 각 객체에 대한 법선 벡터 계산 및 시각화
            normal_vector = extract_normal_at_point(
                depth_frame, cx, cy,
                depth_frame.profile.as_video_stream_profile().get_intrinsics()
            )

            if normal_vector is not None:
                start_point = (cx, cy)
                end_point = (
                    int(cx + normal_vector[0] * 50),
                    int(cy + normal_vector[1] * 50)
                )
                cv2.arrowedLine(color_image, start_point, end_point, (255, 0, 0), 2, tipLength=0.3)

        # 결과 이미지 표시
        cv2.imshow('YOLOv8 Object Detection', color_image)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break


finally:
    pipeline.stop()
    cv2.destroyAllWindows()
