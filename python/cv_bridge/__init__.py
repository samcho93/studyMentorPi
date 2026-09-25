"""cv_bridge shim: sensor_msgs/Image <-> numpy (same API as ROS 2 cv_bridge for common encodings)."""
import numpy as np

from sensor_msgs.msg import Image

_ENC = {  # encoding -> (dtype, channels)
    "rgb8": (np.uint8, 3), "bgr8": (np.uint8, 3), "rgba8": (np.uint8, 4), "bgra8": (np.uint8, 4),
    "mono8": (np.uint8, 1), "8UC1": (np.uint8, 1), "8UC3": (np.uint8, 3),
    "mono16": (np.uint16, 1), "16UC1": (np.uint16, 1), "32FC1": (np.float32, 1),
}


class CvBridgeError(TypeError):
    pass


class CvBridge:
    def imgmsg_to_cv2(self, img_msg, desired_encoding="passthrough"):
        enc = img_msg.encoding
        if enc not in _ENC:
            raise CvBridgeError("unsupported encoding %s" % enc)
        dt, ch = _ENC[enc]
        a = np.frombuffer(bytes(img_msg.data), dtype=dt)
        a = a.reshape(img_msg.height, img_msg.width, ch) if ch > 1 else a.reshape(img_msg.height, img_msg.width)
        a = a.copy()
        if desired_encoding in ("passthrough", enc):
            return a
        if {enc, desired_encoding} <= {"rgb8", "bgr8"}:
            return a[..., ::-1].copy()
        if enc in ("rgb8", "bgr8") and desired_encoding == "mono8":
            rgb = a if enc == "rgb8" else a[..., ::-1]
            return (0.299 * rgb[..., 0] + 0.587 * rgb[..., 1] + 0.114 * rgb[..., 2]).astype(np.uint8)
        if enc == "16UC1" and desired_encoding == "32FC1":
            return a.astype(np.float32) / 1000.0
        raise CvBridgeError("cannot convert %s -> %s" % (enc, desired_encoding))

    def cv2_to_imgmsg(self, cvim, encoding="passthrough", header=None):
        a = np.ascontiguousarray(cvim)
        if encoding == "passthrough":
            encoding = {np.dtype(np.uint16): "16UC1", np.dtype(np.float32): "32FC1"}.get(
                a.dtype, "mono8" if a.ndim == 2 else "bgr8")
        m = Image()
        if header is not None:
            m.header = header
        m.height, m.width = a.shape[0], a.shape[1]
        m.encoding = encoding
        m.step = a.strides[0]
        m.data = a.tobytes()
        return m
