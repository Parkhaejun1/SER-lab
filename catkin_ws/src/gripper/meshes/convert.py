import open3d as o3d

# STL 파일 읽기
input_file = "/home/robot/anaconda3/envs/realsense/bin/python /home/robot/PycharmProjects/realsense/catkin_ws/src/gripper/meshes/AOS.stl"  # STL 파일 경로
output_file = "/home/robot/anaconda3/envs/realsense/bin/python /home/robot/PycharmProjects/realsense/catkin_ws/src/gripper/meshes/AOS.dae"  # DAE 파일로 저장할 경로

# 메시 읽기
mesh = o3d.io.read_triangle_mesh(input_file)

# DAE 파일로 저장
if o3d.io.write_triangle_mesh(output_file, mesh):
    print(f"Successfully converted {input_file} to {output_file}")
else:
    print("Failed to convert STL to DAE")
