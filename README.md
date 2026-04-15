# Mirobot_MoveIt_tracker

캡스톤 디자인(위성 재급유 시스템 - Mirobot_tracker)

동작 과정

1. 준영님께서 카메라 기준의 아루코 마커 위치에 대한 좌표를 동체기준(0,0,0)의 좌표로 변환해주신 x,y,z,qx,qy,qz,qw를 토픽형태로() 받아옵니다.
   
2. 해당 토픽형태(PoseStamped)로 받은 데이터는 아루코 마커의 중심부(확실하지 않음)이기 때문에 해당 아루코 마커로부터 주유구까지는 offset이 존재하므로 Wheel_stop_to_goal에서 해당 offset만큼을 계산 후 
