#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
import numpy as np
import cv2
from ultralytics import YOLO
import time


class YoloDetector(Node):
    def __init__(self):
        super().__init__('laptop_yolo_detector')

        self.sub = self.create_subscription(
            CompressedImage,
            '/camera/image/compressed',
            self.image_cb,
            1
        )

        self.pub = self.create_publisher(
            String,
            '/traffic_sign',
            10
        )

        self.model = YOLO(
            "/home/phong/ros2_ws/src/camera_test_pkg/camera_test_pkg/best9.pt"
        )

        # ===== THAM SỐ LỌC =====
        self.FRAME_W = 320
        self.FRAME_H = 240
        self.MIN_BOX_AREA = 0.10 * (self.FRAME_W * self.FRAME_H)  # 10%
        self.SEND_COOLDOWN = 1.0  # giây

        self.last_sent_time = 0
        self.last_sent_label = ""
        self.AREA_THRESHOLD = 22000

        self.get_logger().info("YOLO detector with distance filter started")

    def image_cb(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if frame is None:
            return

        results = self.model.predict(
            frame,
            conf=0.8,
            verbose=False
        )

        best_box = None
        best_area = 0
        best_label = None

        for r in results:
            if r.boxes is None:
                continue

            for box in r.boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                area = (x2 - x1) * (y2 - y1)

                if area > best_area:
                    best_area = area
                    best_box = (x1, y1, x2, y2)
                    best_label = self.model.names[int(box.cls[0])]

        # ===== HIỂN THỊ CAMERA =====
        if best_box:
            x1, y1, x2, y2 = best_box
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(
                frame,
                f"{best_label} | area={int(best_area)}",
                (x1, y1 - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 0),
                1
            )

        cv2.imshow("YOLO Distance Filter", frame)
        cv2.waitKey(1)

        # ===== LỌC THEO KHOẢNG CÁCH (AREA) =====
        now = time.time()

        if (
            best_label
            and best_area >= self.AREA_THRESHOLD
            and (now - self.last_sent_time) > self.SEND_COOLDOWN
        ):

            cmd = None

            if best_label == "left":
                cmd = "LEFT"
            elif best_label == "right":
                cmd = "RIGHT"
            elif best_label == "stop":
                cmd = "STOP"

            if cmd:
                msg = String()
                msg.data = cmd
                self.pub.publish(msg)

                self.last_sent_time = now
                self.get_logger().info(f"SEND CMD: {cmd}")


            self.last_sent_time = now
            self.last_sent_label = best_label

            self.get_logger().info(
                f"SEND: {best_label} | area={int(best_area)}"
            )


def main():
    rclpy.init()
    node = YoloDetector()
    rclpy.spin(node)
    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
