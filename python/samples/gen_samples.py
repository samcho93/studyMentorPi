"""Generate the Playground sample images (deterministic). Run: python python/samples/gen_samples.py

All images are 640x480 BGR PNG except qr.png (330x330). Contents (pixel coords, x right / y down):
  shapes.png  white bg; red filled circle c=(140,150) r=70; green filled rect (300,80)-(480,220);
              blue filled triangle (120,420),(260,420),(190,290); yellow filled ellipse c=(470,360) axes (110,60)
  colors.png  gray(128) bg; red block (60,160)-(200,320); green block (250,160)-(390,320); blue block (440,160)-(580,320)
  track.png   light floor (220); black line 40 px wide: polyline (320,480)->(320,360)->(360,260)->(460,160)->(560,120)->(640,110)
  lane.png    road view: sky/background dark, gray road trapezoid; white left lane line (150,480)->(290,250),
              yellow right lane line (520,480)->(360,250); dashed center none
  qr.png      QR code for text "MentorPi"
  scene.png   indoor scene: wall + floor gradients, a red ball c=(200,330) r=45, a blue box (380,250)-(520,400),
              a green cone triangle (560,400),(620,400),(590,300)
  coins.png   dark bg (30); 6 light gray filled circles, centres (110,120,r40) (300,110,r55) (500,130,r35)
              (150,340,r50) (340,330,r30) (520,350,r60)
"""
import os

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))


def save(name, img):
    cv2.imwrite(os.path.join(HERE, name), img)


def shapes():
    img = np.full((480, 640, 3), 255, np.uint8)
    cv2.circle(img, (140, 150), 70, (0, 0, 255), -1)
    cv2.rectangle(img, (300, 80), (480, 220), (0, 200, 0), -1)
    pts = np.array([[120, 420], [260, 420], [190, 290]], np.int32)
    cv2.fillPoly(img, [pts], (255, 0, 0))
    cv2.ellipse(img, (470, 360), (110, 60), 0, 0, 360, (0, 220, 255), -1)
    save("shapes.png", img)


def colors():
    img = np.full((480, 640, 3), 128, np.uint8)
    cv2.rectangle(img, (60, 160), (200, 320), (0, 0, 220), -1)
    cv2.rectangle(img, (250, 160), (390, 320), (0, 200, 0), -1)
    cv2.rectangle(img, (440, 160), (580, 320), (220, 0, 0), -1)
    save("colors.png", img)


def track():
    img = np.full((480, 640, 3), 220, np.uint8)
    pts = np.array([[320, 480], [320, 360], [360, 260], [460, 160], [560, 120], [640, 110]], np.int32)
    cv2.polylines(img, [pts], False, (20, 20, 20), 40, lineType=cv2.LINE_AA)
    save("track.png", img)


def lane():
    img = np.full((480, 640, 3), 40, np.uint8)
    road = np.array([[60, 480], [600, 480], [380, 240], [260, 240]], np.int32)
    cv2.fillPoly(img, [road], (90, 90, 90))
    cv2.line(img, (150, 480), (290, 250), (255, 255, 255), 14, cv2.LINE_AA)
    cv2.line(img, (520, 480), (360, 250), (0, 215, 255), 14, cv2.LINE_AA)
    save("lane.png", img)


def qr():
    enc = cv2.QRCodeEncoder.create()
    q = enc.encode("MentorPi")
    q = cv2.resize(q, (q.shape[1] * 10, q.shape[0] * 10), interpolation=cv2.INTER_NEAREST)
    q = cv2.copyMakeBorder(q, 40, 40, 40, 40, cv2.BORDER_CONSTANT, value=255)
    q = cv2.resize(q, (330, 330), interpolation=cv2.INTER_NEAREST)
    save("qr.png", cv2.cvtColor(q, cv2.COLOR_GRAY2BGR))


def scene():
    img = np.zeros((480, 640, 3), np.uint8)
    for y in range(480):
        if y < 260:
            img[y, :] = (200 - y // 4, 190 - y // 5, 180 - y // 6)
        else:
            v = 110 + (y - 260) // 3
            img[y, :] = (v - 30, v - 10, v)
    cv2.circle(img, (200, 330), 45, (30, 30, 210), -1)
    cv2.rectangle(img, (380, 250), (520, 400), (190, 80, 30), -1)
    cv2.fillPoly(img, [np.array([[560, 400], [620, 400], [590, 300]], np.int32)], (40, 170, 40))
    save("scene.png", img)


def coins():
    img = np.full((480, 640, 3), 30, np.uint8)
    for cx, cy, r in [(110, 120, 40), (300, 110, 55), (500, 130, 35), (150, 340, 50), (340, 330, 30), (520, 350, 60)]:
        cv2.circle(img, (cx, cy), r, (200, 200, 200), -1, cv2.LINE_AA)
    save("coins.png", img)


if __name__ == "__main__":
    for f in (shapes, colors, track, lane, qr, scene, coins):
        f()
    print("samples written to", HERE)
