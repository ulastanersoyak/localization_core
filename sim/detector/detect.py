#!/usr/bin/env python3
"""Runs YOLO on the pole camera and publishes the drone's pixel position.

Publishes "<frame time> <u> <v> <confidence>" on /pole/detection, one message
per detected frame. Always works on the newest frame, so a slow model drops
frames instead of falling behind.

Every processed frame is also republished with the boxes drawn on it (see
--annotated-topic), for the "YOLO detections" panel in the Gazebo window.
"""
import argparse
import threading
import time

import cv2
import numpy as np
from gz.msgs10.image_pb2 import Image
from gz.msgs10.stringmsg_pb2 import StringMsg
from gz.transport13 import Node
from ultralytics import YOLO


def draw(msg, rgb, result, best, scale):
    """The camera frame with every box drawn: the one sent to the filter in
    green, any others in orange. The drone is small at range, so the green box
    is also shown magnified in the top right corner."""
    img = rgb.copy()  # the message buffer is read only
    h, w = img.shape[:2]
    # Sized for the full frame so they stay readable after scaling down.
    thick = max(1, round(1.5 / scale))
    font = 0.6 / scale
    pad = round(6 / scale)  # keep the lines off the drone itself
    boxes = [tuple(round(c) for c in b) for b in result.boxes.xyxy.tolist()]
    confs = result.boxes.conf.tolist()

    inset = None
    if best is not None:
        # Crop before drawing, so the magnified view shows the raw pixels.
        x0, y0, x1, y1 = boxes[best]
        half = max(x1 - x0, y1 - y0, 20)
        cx, cy = (x0 + x1) // 2, (y0 + y1) // 2
        crop = img[max(cy - half, 0):cy + half, max(cx - half, 0):cx + half]
        if crop.size:
            side = round(h * 0.4)
            inset = cv2.resize(crop, (side, side),
                               interpolation=cv2.INTER_NEAREST)
            # The box, in the magnified view's pixels.
            k = side / crop.shape[1], side / crop.shape[0]
            ox, oy = max(cx - half, 0), max(cy - half, 0)
            cv2.rectangle(inset, (round((x0 - ox) * k[0]), round((y0 - oy) * k[1])),
                          (round((x1 - ox) * k[0]), round((y1 - oy) * k[1])),
                          (40, 220, 60), thick)

    for i, ((x0, y0, x1, y1), conf) in enumerate(zip(boxes, confs)):
        colour = (40, 220, 60) if i == best else (255, 150, 0)  # RGB
        cv2.rectangle(img, (x0 - pad, y0 - pad), (x1 + pad, y1 + pad), colour,
                      thick)
        cv2.putText(img, "drone %.2f" % conf, (x0 - pad, max(y0 - 2 * pad, 30)),
                    cv2.FONT_HERSHEY_SIMPLEX, font, colour, thick)
    if best is None:
        cv2.putText(img, "no detection", (20, round(40 / scale)),
                    cv2.FONT_HERSHEY_SIMPLEX, font, (255, 60, 60), thick)
    if inset is not None:
        side = inset.shape[0]
        margin = round(10 / scale)
        img[margin:margin + side, w - margin - side:w - margin] = inset
        cv2.rectangle(img, (w - margin - side, margin),
                      (w - margin, margin + side), (255, 255, 255), thick)
        cv2.putText(img, "zoom", (w - margin - side, margin + side + round(24 / scale)),
                    cv2.FONT_HERSHEY_SIMPLEX, font, (40, 220, 60), thick)

    if scale != 1:
        img = cv2.resize(img, None, fx=scale, fy=scale,
                         interpolation=cv2.INTER_AREA)
    out = Image()
    out.header.CopyFrom(msg.header)
    out.width, out.height = img.shape[1], img.shape[0]
    out.step = img.shape[1] * 3
    out.pixel_format_type = msg.pixel_format_type
    out.data = img.tobytes()
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--weights", required=True)
    ap.add_argument("--imgsz", type=int, default=1920)
    ap.add_argument("--conf", type=float, default=0.25)
    ap.add_argument("--device", default=None)
    ap.add_argument("--annotated-topic", default="/pole/camera/annotated",
                    help="frames with the boxes drawn, empty to disable")
    ap.add_argument("--annotated-scale", type=float, default=0.5,
                    help="size of the annotated frames relative to the camera")
    args = ap.parse_args()

    model = YOLO(args.weights)
    condition = threading.Condition()
    newest = {"msg": None}

    def on_image(msg):
        with condition:
            newest["msg"] = msg
            condition.notify()

    node = Node()
    publisher = node.advertise("/pole/detection", StringMsg)
    annotated = (node.advertise(args.annotated_topic, Image)
                 if args.annotated_topic else None)
    node.subscribe(Image, "/pole/camera", on_image)
    print("detector ready", flush=True)

    frames = detections = 0
    busy = 0.0
    report = time.time()
    while True:
        with condition:
            while newest["msg"] is None:
                condition.wait()
            msg, newest["msg"] = newest["msg"], None

        start = time.time()
        rgb = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, 3)
        result = model.predict(rgb[:, :, ::-1], imgsz=args.imgsz, conf=args.conf,
                               device=args.device, verbose=False)[0]
        busy += time.time() - start
        frames += 1
        best = int(result.boxes.conf.argmax()) if len(result.boxes) else None
        if annotated:
            annotated.publish(draw(msg, rgb, result, best, args.annotated_scale))
        if best is not None:
            u, v = result.boxes.xywh[best][:2].tolist()
            t = msg.header.stamp.sec + msg.header.stamp.nsec * 1e-9
            out = StringMsg()
            out.data = "%.4f %.2f %.2f %.3f" % (t, u, v, float(result.boxes.conf[best]))
            publisher.publish(out)
            detections += 1

        if time.time() - report > 10:
            report = time.time()
            print("detector: %d frames, %d detections, %.0f ms per frame"
                  % (frames, detections, 1000 * busy / frames), flush=True)


if __name__ == "__main__":
    main()
