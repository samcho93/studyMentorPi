# 원본 교재 ↔ 챕터 매핑

원본: Hiwonder MentorPi 교재 (영문 PDF, `D:\MentorPi`). 본 사이트는 원본을 요약·재구성·보강한 한국어 강의이며 원문을 복제하지 않는다.

| 챕터 | 원본 폴더 / 레슨 |
|---|---|
| B01 | 1. Getting Ready — L1 MentorPi Introduction, L2 Charging & Usage Guide, Assembly |
| B02 | 1. Getting Ready — L3 Get Started, L4 APP Control, L5 Wireless Handle Control |
| B03 | 8. Set Development Environment — L1 VNC, L2 Robot Version Configuration, L3 System Introduction, L4 Docker Introduction & Usage, L5 Servo Deviation Adjustment |
| B04 | 2. Linux Basic Lesson — L1~L7 |
| B05 | 3. Python Basic Lesson — L1~L7 |
| B06 | 3. Python Basic Lesson — L8~L13 |
| B07 | 6. Raspberry Pi 5 Controller — 1. Hardware & System Introduction, 2. Basic Operation and Configuration |
| B08 | 6. Raspberry Pi 5 Controller — 4. Hardware Control Course (GPIO/UART/I2C); 6. Expansion Board Control L1~L4 (RGB, 버튼, 부저), board_demo Analysis |
| B09 | 6. Expansion Board Control L5~L12 (PWM 서보, 버스 서보, DC 모터, GPIO); Appendix/Source Code/RRCLite_demo |
| B10 | 7. Docker Basic Lesson — L1~L8 |
| R01 | 5. ROS2 Basic Course — L1, L2, L3 |
| R02 | 5. ROS2 Basic Course — L4, L5, L6, L7 |
| R03 | 5. ROS2 Basic Course — L8 |
| R04 | 5. ROS2 Basic Course — L9, L10, L11 |
| R05 | 5. ROS2 Basic Course — L12, L13, L14, L15 |
| R06 | 5. ROS2 Basic Course — L16, L17 |
| R07 | 5. ROS2 Basic Course — L18, L19, L20 |
| R08 | 4. OpenCV Computer Vision Course — L1~L6 |
| R09 | 4. OpenCV — L7, L8, L9 |
| R10 | 4. OpenCV — L10, L11, L12 |
| R11 | 4. OpenCV — L13, L14, L15 |
| M01 | 9. Motion Control — L1 Mecanum Analysis, L5 Mecanum Speed Control; src/driver/controller/controller/mecanum.py |
| M02 | 9. Motion Control — L2 Ackerman Analysis, L6 Ackerman Speed Control; controller/ackermann.py |
| M03 | 9. Motion Control — L3 IMU/Linear/Angular Calibration, L4 Publish IMU and Odometer; controller/odom_publisher_node.py |
| M04 | 9. Motion Control — L5, L6 (속도 제어 프로그래밍) |
| M05 | 10. Lidar Lesson — L1, L2 |
| M06 | 10. Lidar Lesson — L3 Obstacle Avoidance, L4 Following, L5 Guarding; src/app |
| M07 | 11. Depth Camera Basic Lesson — L1~L5, Test and Configure ROS2 |
| M08 | 12. Mapping Lesson — L1 URDF Model Introduction, L2 ROS Robot URDF Model; src/simulations/mentorpi_description |
| M09 | 12. Mapping Lesson — L3 SLAM Principle, L4 slam_toolbox; Install WinSCP |
| M10 | 12. Mapping Lesson — L5 RTAB-VSLAM 3D Mapping |
| M11 | 13. Navigation Lesson — L1 Autonomous Navigation, L2 AMCL |
| M12 | 13. Navigation Lesson — L3 DWA Path Planning |
| M13 | 13. Navigation Lesson — L4 Point-to-Point/Multi-Point, L5 RTAB-VSLAM 3D Navigation |
| A01 | 14. ROS+OpenCV — Install Depth Camera, L1 Color Threshold, L2 Color Recognition |
| A02 | 14. ROS+OpenCV — L3 QR Code |
| A03 | 14. ROS+OpenCV — L4 Autonomous Line Following |
| A04 | 14. ROS+OpenCV — L5 Color Tracking, Network Configuration |
| A05 | 15. MediaPipe — L1, L2 |
| A06 | 15. MediaPipe — L3, L4, L5 |
| A07 | 16. Machine Learning — L1, L2, L3 |
| A08 | 16. Machine Learning — L4 Yolov5 Model Training; Appendix/Source Code/software (labelImg, yolov5) |
| A09 | 17. Autonomous Driving — Map Laying and Prop Installation, L1 Lane Keeping |
| A10 | 17. Autonomous Driving — L2 Road Sign Detection, L3 Traffic Light Recognition |
| A11 | 17. Autonomous Driving — L4 Turning Decision, L5 Autonomous Parking, L6 Integrated Application |
| A12 | 18. Group Control — 01 Master-Slave Configuration, 02 Group Control Startup |
| P01 | 통합 프로젝트 — 12. Mapping, 13. Navigation, 10. Lidar(Guarding), src/navigation, src/app/lidar_controller.py |
| P02 | 통합 프로젝트 — 14. ROS+OpenCV L3 QR, 13. Navigation L4, src/example/qrcode |
| P03 | 통합 프로젝트 — 17. Autonomous Driving L1~L6, src/example/self_driving, yolov5_ros2 |
| P04 | 통합 프로젝트 — 15. MediaPipe, 10. Lidar(Following), src/example/body_control, hand_track |

부록(Appendix: RRC Lite 하드웨어/STM32, 시스템 이미지 굽기)은 B03·B08·B09의 참고자료로 인용한다.
