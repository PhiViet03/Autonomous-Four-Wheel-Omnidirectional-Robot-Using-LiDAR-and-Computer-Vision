import cv2
import numpy as np
from flask import Flask, Response

# ================= CONFIG =================
CAM_INDEX = 2
WIDTH, HEIGHT = 640, 480

HSV_LOWER = np.array([35, 40, 40])
HSV_UPPER = np.array([85, 255, 255])
MIN_AREA = 500

ROI_Y_RATIO = 0.6  # bottom 40% of frame
# =========================================

app = Flask(__name__)

cap = cv2.VideoCapture(CAM_INDEX)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)

latest_offset = 0


def find_centroid(mask):
    M = cv2.moments(mask)
    if M["m00"] > MIN_AREA:
        return (
            int(M["m10"] / M["m00"]),
            int(M["m01"] / M["m00"])
        )
    return None


def gen():
    global latest_offset

    roi_y = int(HEIGHT * ROI_Y_RATIO)

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        frame = cv2.flip(frame, 1)
        vis = frame.copy()

        # -------- ROI PROCESSING --------
        roi = frame[roi_y:HEIGHT, :]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        mask = cv2.inRange(hsv, HSV_LOWER, HSV_UPPER)
        mask = cv2.GaussianBlur(mask, (5, 5), 0)

        line_center = find_centroid(mask)
        car_center = (WIDTH // 2, (HEIGHT - roi_y) // 2)

        # -------- DRAWING --------
        cv2.rectangle(vis, (0, roi_y), (WIDTH, HEIGHT), (0, 255, 0), 2)

        full_car_center = (car_center[0], car_center[1] + roi_y)
        cv2.circle(vis, full_car_center, 6, (255, 255, 255), -1)

        if line_center:
            full_line_center = (
                line_center[0],
                line_center[1] + roi_y
            )

            latest_offset = line_center[0] - car_center[0]

            cv2.circle(vis, full_line_center, 8, (0, 0, 255), -1)
            cv2.line(vis, full_car_center, full_line_center, (255, 255, 0), 2)

            cv2.putText(
                vis,
                f"Offset: {latest_offset}",
                (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.9,
                (0, 255, 255),
                2
            )
        else:
            latest_offset = 0
            cv2.putText(
                vis,
                "Line lost",
                (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.9,
                (0, 0, 255),
                2
            )

        # -------- STREAM --------
        _, jpeg = cv2.imencode(".jpg", vis)
        yield (
            b"--frame\r\n"
            b"Content-Type: image/jpeg\r\n\r\n" +
            jpeg.tobytes() +
            b"\r\n"
        )


@app.route("/video")
def video():
    return Response(
        gen(),
        mimetype="multipart/x-mixed-replace; boundary=frame"
    )


if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5000, threaded=True)
