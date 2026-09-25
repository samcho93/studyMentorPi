# ROS 2 Playground — 지원 API (mentorpi_sim shim)

`tools/playground.html`은 Pyodide(브라우저 Python 3.12)에서 학생 코드를 실행한다.
`python/mentorpi_sim/`이 **rclpy·메시지 패키지와 같은 import 경로**를 제공하고, 2D 가상 MentorPi와 월드를 시뮬레이션한다.
강의 본문의 ` ```python run ` 블록은 **이 문서에 있는 API만** 써야 한다(`tests/test_run_blocks.py`가 CPython에서 전부 실행해 검사).

## 1. 시뮬레이션 설정 (시뮬 전용)

```python
import mentorpi_sim as sim            # 실물에서는 이 줄과 sim.setup(...)을 지운다
sim.setup(chassis="mecanum",          # "mecanum" | "ackermann"
          world="room",               # "empty" | "room" | "maze" | "corridor" | "arena"
          duration=20.0,              # 시뮬레이션 시간 상한 [s] — spin()이 여기서 끝남
          start=(0.0, 0.0, 0.0),      # 시작 자세 (x, y, yaw[rad]); 생략하면 월드 기본값
          noise=True)                 # 오도메트리·라이다 잡음
```
`sim.setup()`을 부르지 않으면 기본값(mecanum, room, 20 s)이 쓰인다. 월드 크기: empty 6×6 m, room 6×4 m, maze 5×5 m, corridor 8×2 m, arena 4×4 m(상자 여러 개).
`sim.add_box(x, y, w, h)`, `sim.add_cylinder(x, y, r)`, `sim.add_mover(x, y, vx, vy)`(움직이는 원통 — 추종 실습용), `sim.add_walker([(x,y),...], speed=0.15, r=0.12, loop=True)`(경로를 따라 걷는 사람)로 장애물을 추가할 수 있다.
`sim.movers()` → `[(x, y, r), ...]` 움직이는 물체의 실제 위치(시뮬 전용 — 카메라 인식 결과를 흉내 낼 때 사용), `sim.pose()` → 실제 로봇 자세 `(x, y, yaw)`.
`sim.add_object(label, x, y, r=0.08)` 이름표가 붙은 원통 물체(라이다에도 보임), `sim.objects()` → `[(label, x, y, r), ...]` (시뮬 전용 — YOLO+깊이 인식 결과 대용).

## 2. rclpy

```python
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, qos_profile_sensor_data
from rclpy.parameter import Parameter
from rclpy.action import ActionServer, ActionClient

rclpy.init(args=None); rclpy.ok(); rclpy.shutdown()
rclpy.spin(node)                         # duration까지(또는 rclpy.shutdown()/KeyboardInterrupt까지) 실행
rclpy.spin_once(node, timeout_sec=0.1)   # 시뮬 시간을 최대 timeout_sec 진행
rclpy.spin_until_future_complete(node, future, timeout_sec=None)
```

`Node`:
- `create_publisher(msg_type, topic, qos)` → `.publish(msg)`, `.topic_name`
- `create_subscription(msg_type, topic, callback, qos)`
- `create_timer(period_sec, callback)` → `.cancel()`
- `create_service(srv_type, name, callback)` — callback(request, response) → response
- `create_client(srv_type, name)` → `.wait_for_service(timeout_sec)`, `.call_async(req)` → Future(`.done()`, `.result()`, `.add_done_callback`), `.call(req)`
- `declare_parameter(name, default)`, `get_parameter(name).value`, `set_parameters([Parameter(name, Parameter.Type.DOUBLE, v)])`, `add_on_set_parameters_callback(cb)`
- `get_logger().info/warn/error/debug(msg)`, `get_clock().now()` (`.nanoseconds`, `.to_msg()`), `get_name()`, `destroy_node()`
- 토픽 이름은 `/`로 시작하지 않으면 `/`가 붙는다(네임스페이스 없음).

액션: `ActionServer(node, ActionType, name, execute_callback)` — execute_callback(goal_handle): `goal_handle.request`, `goal_handle.publish_feedback(fb)`, `goal_handle.succeed()`, `return ActionType.Result(...)`.
`ActionClient(node, ActionType, name)` → `.wait_for_server()`, `.send_goal_async(goal, feedback_callback=cb)` → future → goal_handle → `.get_result_async()`.

## 3. 메시지·서비스·액션 타입

| 패키지 | 타입 |
|---|---|
| `std_msgs.msg` | `String, Bool, Int32, Int64, Float32, Float64, Header, ColorRGBA` |
| `geometry_msgs.msg` | `Twist, Vector3, Point, Quaternion, Pose, PoseStamped, Pose2D, PoseWithCovarianceStamped, Transform, TransformStamped` |
| `sensor_msgs.msg` | `LaserScan, Imu, Image, CompressedImage, CameraInfo, PointCloud2, PointField, RegionOfInterest` |
| `nav_msgs.msg` | `Odometry, Path, OccupancyGrid` |
| `std_srvs.srv` | `Trigger, SetBool, Empty` |
| `example_interfaces.srv` | `AddTwoInts` |
| `example_interfaces.action` | `Fibonacci` (goal.order / feedback.sequence / result.sequence) |
| `builtin_interfaces.msg` | `Time, Duration` |

필드 이름·기본값은 ROS 2 Humble과 같다(예: `Twist().linear.x`, `LaserScan().ranges`, `Odometry().pose.pose.position.x`).

## 4. 가상 MentorPi의 토픽

| 토픽 | 타입 | 방향 | 설명 |
|---|---|---|---|
| `/cmd_vel` | Twist | 구독 | 앱/사용자 속도 명령. 실물과 같이 \|linear.x\|,\|linear.y\| ≤ 0.2 m/s, \|angular.z\| ≤ 0.5 rad/s로 자름 |
| `/controller/cmd_vel` | Twist | 구독 | 제한 없는 입력(내비게이션용). 물리 한계 0.6 m/s, 3 rad/s |
| `/odom` | Odometry | 발행 50 Hz | frame `odom` → child `base_footprint`, 잡음 누적(드리프트) |
| `/scan` | LaserScan | 발행 10 Hz | 360개, angle_min −π, angle_max π(0 = 전방, 반시계 +), range 0.05~12 m, 없으면 `inf` |
| `/imu` | Imu | 발행 50 Hz | orientation(쿼터니언), angular_velocity.z, linear_acceleration |
| `/tf` | TransformStamped | 발행 | odom→base_footprint |
| `/ground_truth` | Pose2D | 발행 10 Hz | 시뮬 전용 실제 자세 (오도메트리 오차 비교용) |
| `/ascamera/camera_publisher/rgb0/image` | Image `rgb8` | 발행 5 Hz* | 깊이 카메라(HP60C)의 컬러 영상, frame `ascamera_color_0` |
| `/ascamera/camera_publisher/depth0/image_raw` | Image `16UC1`(mm) | 발행 5 Hz* | 컬러와 정렬된 깊이 영상, 0 = 무효, frame `ascamera_camera_link_0` |
| `/ascamera/camera_publisher/rgb0/camera_info`, `depth0/camera_info` | CameraInfo | 발행 5 Hz* | K = [fx 0 cx; 0 fy cy; 0 0 1] (실물 camera_info.yaml을 해상도에 맞춰 축소) |
| `/ascamera/camera_publisher/depth0/points` | PointCloud2 (x,y,z float32) | 발행 5 Hz* | 광학 좌표계(x 오른쪽, y 아래, z 앞) 포인트 클라우드(2픽셀 간격) |

\* 카메라 토픽은 **구독자가 있을 때만** 렌더링된다. 기본 160×120 (실물 640×480, 15 fps) — `sim.setup(camera=(320, 240), camera_hz=5)`로 변경. 깊이 모델: 0.15~4.0 m, 잡음 σ = 1 mm + 2.5 mm·d², 1 % 결측. 장면 높이: 벽 0.30 m, 상자 0.25 m, 원통 0.30 m, 이름 붙은 물체 2r, 걷는 사람 1.6 m.

애커만 섀시에서는 `linear.y`가 무시되고, `angular.z`는 조향각으로 바뀐다(δ = atan(L·ω/v), |δ| ≤ 29°, 정지 상태에서는 회전 불가).

## 5. 기타 모듈

- `time.sleep(t)`, `time.time()`, `time.monotonic()` — 실행 중에는 **시뮬레이션 시간** 기준으로 동작한다(sleep하면 시뮬이 진행됨).
- `tf_transformations.euler_from_quaternion([x,y,z,w])`, `quaternion_from_euler(r,p,y)`
- `tf2_ros.TransformBroadcaster(node)`, `StaticTransformBroadcaster(node)`, `Buffer()`, `TransformListener(buffer, node)`, `buffer.lookup_transform(target, source, rclpy.time.Time())`
- `numpy`, `math`, `random`, `threading`(스레드는 동기 실행으로 흉내), `cv2`(Pyodide opencv-python)
- `cv2.imshow(name, img)` → 오른쪽 이미지 패널에 표시, `cv2.waitKey()` → -1, `cv2.imread("shapes.png")`로 샘플 이미지 사용:
  `shapes.png`(도형), `colors.png`(색 블록), `track.png`(검은 선 트랙), `lane.png`(차선 원근), `qr.png`(QR "MentorPi"), `lena.png` 대체 `scene.png`(실내 장면), `coins.png`(원형 물체)
- `mentorpi_sim.plot(xs, ys, label)` — 결과 그래프 패널에 선 추가 (시뮬 전용)
- `mentorpi_sim.camera()` → `(bgr uint8, depth float32 m)` 지금 이 순간의 RGB-D 영상(ROS 없이, 시뮬 전용), `mentorpi_sim.camera_intrinsics()` → `(fx, fy, cx, cy)`
- `cv_bridge.CvBridge().imgmsg_to_cv2(msg, "bgr8"|"rgb8"|"mono8"|"passthrough"|"32FC1")`, `cv2_to_imgmsg(arr, encoding)`
- `sensor_msgs_py.point_cloud2.read_points_numpy(cloud, ("x","y","z"))`, `read_points(...)`, `create_cloud_xyz32(header, points)`

## 6. 실행 코드 작성 규칙

1. 시뮬 전용 줄에는 `# 시뮬 전용` 주석을 단다. 나머지는 실물 MentorPi 컨테이너에서도 그대로 돈다.
2. `rclpy.spin()`은 `try/except KeyboardInterrupt` + `finally:`에서 정지 Twist 발행 → `destroy_node()` → `rclpy.shutdown()`.
3. 실행 시간이 길면 `duration`을 줄여 30 s 이내로 한다.
4. 예상 출력은 ` ```text ` 블록으로 코드 뒤에 붙인다(잡음 때문에 숫자는 “대략”으로 표기).
