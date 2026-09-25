# title: 깊이 카메라 — RGB·깊이·포인트 클라우드
# group: 깊이 카메라
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge

import mentorpi_sim as sim                                      # 시뮬 전용
sim.setup(world="room", duration=1.0, start=(-2.3, -1.3, 0.35))  # 시뮬 전용
sim.add_object("red_ball", -1.35, -0.85, 0.1)                   # 시뮬 전용

NS = "/ascamera/camera_publisher"                  # MentorPi HP60C (ascamera) 토픽


class DepthViewer(Node):
    def __init__(self):
        super().__init__("depth_viewer")
        self.bridge = CvBridge()
        self.rgb = self.depth = self.K = None
        self.create_subscription(Image, NS + "/rgb0/image", self.on_rgb, 1)
        self.create_subscription(Image, NS + "/depth0/image_raw", self.on_depth, 1)
        self.create_subscription(CameraInfo, NS + "/rgb0/camera_info", self.on_info, 1)

    def on_rgb(self, m):
        self.rgb = self.bridge.imgmsg_to_cv2(m, "bgr8")

    def on_depth(self, m):
        self.depth = self.bridge.imgmsg_to_cv2(m, "passthrough").astype(np.float32) / 1000.0   # mm -> m

    def on_info(self, m):
        self.K = np.array(m.k).reshape(3, 3)


rclpy.init()
node = DepthViewer()
rclpy.spin(node)                                   # 시뮬: 1 s 동안 영상 수신

rgb, depth, K = node.rgb, node.depth, node.K
h, w = depth.shape
fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
print("해상도 %dx%d, fx=%.1f fy=%.1f cx=%.1f cy=%.1f" % (w, h, fx, fy, cx, cy))
valid = depth > 0
print("유효 픽셀 %.0f%%, 깊이 범위 %.2f~%.2f m" % (100 * valid.mean(), depth[valid].min(), depth[valid].max()))
print("영상 중심 (%d,%d) 깊이 = %.3f m" % (w // 2, h // 2, depth[h // 2, w // 2]))

# 1) 깊이 → 컬러맵 (가까울수록 빨강)
vis = cv2.applyColorMap(cv2.convertScaleAbs(depth, alpha=255 / 4.0), cv2.COLORMAP_JET)
vis[~valid] = 0
# 2) 역투영: 픽셀 (u, v, Z) → 카메라 좌표 (X, Y, Z)
v, u = np.nonzero(valid)
Z = depth[v, u]
X = (u + 0.5 - cx) / fx * Z
Y = (v + 0.5 - cy) / fy * Z
print("포인트 클라우드 %d점" % len(Z))
# 3) 바닥 제거(카메라 높이 0.121 m → 광학 y ≈ +0.121) 후 가장 가까운 장애물
obst = Y < 0.121 - 0.03
i = np.argmin(Z[obst])
print("가장 가까운 장애물: Z=%.2f m, X=%+.2f m (오른쪽 +)" % (Z[obst][i], X[obst][i]))
# 4) 위에서 본 모습 (X-Z 평면)
top = np.full((200, 200, 3), 30, np.uint8)
px = np.clip((X * 50 + 100).astype(int), 0, 199)
pz = np.clip((200 - Z * 50).astype(int), 0, 199)
top[pz, px] = (0, 255, 0)
top[pz[obst], px[obst]] = (0, 0, 255)

cv2.imshow("RGB (rgb0/image)", rgb)
cv2.imshow("depth colormap (depth0/image_raw)", vis)
cv2.imshow("top view: green=floor, red=obstacle", top)
node.destroy_node()
rclpy.shutdown()
