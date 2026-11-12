"""Burn a caption bar into a camera frame and save it as JPEG (uses OpenCV, not Webots)."""
import cv2
import numpy as np

COLOURS = {"SAFE": (80, 200, 80), "CAUTION": (0, 190, 255), "DANGER": (60, 60, 255)}   # BGR


def bgra_bytes_to_bgr(buf, width, height):
    return np.frombuffer(buf, np.uint8).reshape((height, width, 4))[:, :, :3].copy()


def caption_lines(t, level, reason, gas_ppm, dist_m, tilt_deg, pan_deg, why=""):
    gas = "warming up" if gas_ppm is None else f"{gas_ppm:.0f} ppm"
    dist = "no echo" if dist_m is None or dist_m == float("inf") else f"{dist_m * 100:.0f} cm"
    return (f"t={t:6.1f}s  {level} {reason}" + (f"  [{why}]" if why and why != reason else ""),
            f"gas {gas}   dist {dist}   tilt {tilt_deg:.0f} deg   pan {pan_deg:+.0f} deg")


def save_captioned(path, bgr, t, level, reason, gas_ppm, dist_m, tilt_deg, pan_deg, why="", quality=85):
    img = bgr.copy()
    h, w = img.shape[:2]
    bar = 52
    img[h - bar:h] = (img[h - bar:h] * 0.25).astype(np.uint8)
    cv2.rectangle(img, (0, h - bar), (8, h), COLOURS.get(level, (200, 200, 200)), -1)
    l1, l2 = caption_lines(t, level, reason, gas_ppm, dist_m, tilt_deg, pan_deg, why)
    cv2.putText(img, l1, (16, h - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.55, COLOURS.get(level, (255, 255, 255)), 1, cv2.LINE_AA)
    cv2.putText(img, l2, (16, h - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (235, 235, 235), 1, cv2.LINE_AA)
    return bool(cv2.imwrite(path, img, [cv2.IMWRITE_JPEG_QUALITY, int(quality)]))
