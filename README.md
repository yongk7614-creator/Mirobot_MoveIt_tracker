# Mirobot_MoveIt_tracker

캡스톤 디자인(위성 재급유 시스템 - Mirobot_tracker)

동작 과정

1. 준영님께서 카메라 기준의 아루코 마커 위치에 대한 좌표를 동체기준(0,0,0)의 좌표로 변환해주신 x,y,z,qx,qy,qz,qw를 토픽형태로(PoseStamped) 받아옵니다.
   
2. 해당 토픽형태(PoseStamped)로 받은 데이터는 5 size의 버퍼를 통해 평균값을 계산하고, 도출된 하나의 좌표를 뽑는다. 이후 아루코 마커의 중심부(확실하지는 않음)이기 때문에 주유구까지는 offset이 존재하므로 Wheel_stop_to_goal에서 해당 offset만큼을 계산하여 도출된 좌표를 Moveit2에 넘겨준다.

3. Moveit2에서는 해당 넘겨받은 좌표를 기준으로 trajectory를 뽑아낸다.

4. Wlkata_mirobot_ros2를 통해 Moveit2에서 도출된 Trajectory를 기준으로 mirobot에 전송할 G-code를 생성하여 mirobot에 전송한다.
-----------------------------------------------------------------------------------------------------------------------------------------------------------
> mirobot_moveit_tracker.launch.py
   - launch 인자 선언 및 해당 값들을 ROS parameter로 변환해서 Wheel_stop_to_goal_node와 moveit_goal_node에 넘김.

> mirobot_wlkata_system.launch.py
   - tracker launch를 포함하고 WLKATA MoveIt demo launch를 포함. Trajectory를 받아서 실제 Mirobot에 보내는 노드를 띄움.
  
> wheel_stop_to_goal_node.py
   - wheel_status가 STOPPED가 되면, aruco_pose_base에서 pose 5개를 모아 평균내고 offset을 더해서 mirobot_goal_pose로 보냄
