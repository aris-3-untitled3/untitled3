# src/camera_ar_marker 디렉토리에 있는 ar_marker_subscriber.py, rotate_robot.py 두 노드만 실행하면 됩니다.  

1.ar_marker_subscriber.py: 아루코마커 인식 노드  
  노드명: aruco_marker_subscriber  
  구독: 압축 이미지('/camera/color/image_raw/compressed')  
  발행: 마커 각도, 거리, ID('/ar_marker_angle', '/ar_marker_distance', '/ar_marker_id')  
  노드 실행: ros2 launch storagy bringup.launch.py -> ros2 run my_image_publisher image_publiser -> ros2 run camera_ar_marker ar_marker_subscriber  

2.rotate_robot.py: 로봇 이동 노드  
  노드명: rotate_controller  
  구독: 마커 각도, 거리, ID('/ar_marker_angle', '/ar_marker_distance', '/ar_marker_id')  
  발행: 로봇 속도('/cmd_vel')  
  노드 실행: ros2 launch storagy bringup.launch.py -> ros2 run my_image_publisher image_publiser -> ros2 run camera_ar_marker ar_marker_subscriber -> ros2 run camera_ar_marker rotate_robot    

    결론: ros2 launch storagy bringup.launch.py -> ros2 run my_image_publisher image_publiser -> ros2 run camera_ar_marker ar_marker_subscriber -> ros2 run camera_ar_marker rotate_robot    
