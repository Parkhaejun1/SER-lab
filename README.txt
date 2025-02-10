#Hardware connections

roscore
roslaunch ur_robot_driver ur3e_bringup.launch robot_ip:=192.168.1.102
roslaunch moveit_setup_assistant setup_assistant.launch
roslaunch ur3e_moveit_config moveit_planning_execution.launch
roslaunch ur3e_moveit_config moveit_rviz.launch
rosrun rosserial_python serial_node.py /dev/ttyACM0
roslaunch test pose1.launch
rosrun robot_state_publisher robot_state_publisher $(rospack find ur_description)/urdf/test_ur3e.urdf



#Script nodes

rosrun test object_detect_node.py
rosrun test robot_control_node.py
rosrun test marker_extract_node.py
rosrun test point_cloud_extract_node.py



#Motify urdf files
rosrun xacro xacro --inorder $(rospack find ur_description)/urdf/ur3e.xacro -o ur3e.urdf
check_urdf test_ur3e.urdf
