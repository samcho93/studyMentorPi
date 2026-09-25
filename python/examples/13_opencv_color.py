# title: OpenCV — HSV 색 마스크와 윤곽선
# group: OpenCV
import cv2
import numpy as np

img = cv2.imread("shapes.png")                 # 샘플 이미지 640×480
hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

ranges = {                                     # (H 0~179, S, V)
    "red":    [((0, 120, 70), (10, 255, 255)), ((170, 120, 70), (179, 255, 255))],
    "green":  [((40, 80, 60), (85, 255, 255))],
    "blue":   [((100, 120, 60), (130, 255, 255))],
    "yellow": [((20, 120, 120), (35, 255, 255))],
}
out = img.copy()
for name, rs in ranges.items():
    mask = np.zeros(img.shape[:2], np.uint8)
    for lo, hi in rs:
        mask |= cv2.inRange(hsv, np.array(lo), np.array(hi))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for c in contours:
        area = cv2.contourArea(c)
        if area < 500:
            continue
        m = cv2.moments(c)
        cx, cy = int(m["m10"] / m["m00"]), int(m["m01"] / m["m00"])
        x, y, w, h = cv2.boundingRect(c)
        cv2.rectangle(out, (x, y), (x + w, y + h), (0, 0, 0), 2)
        cv2.putText(out, name, (x, y - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)
        print("%-6s 중심=(%3d,%3d) 면적=%6.0f" % (name, cx, cy, area))

cv2.imshow("original", img)
cv2.imshow("detected", out)
cv2.waitKey(0)
