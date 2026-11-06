"""Optional live ZED preview. Visualization only; no motor control."""

import json
import math
import time

import cv2
import pyzed.sl as sl
import rclpy

from std_msgs.msg import String


BONES = [
    (0, 1),
    (1, 2), (2, 3), (3, 4),
    (1, 5), (5, 6), (6, 7),
    (1, 8), (8, 9), (9, 10),
    (1, 11), (11, 12), (12, 13),
    (8, 11),
]

YELLOW = (0, 255, 255)
GREEN = (0, 255, 0)
RED = (0, 0, 255)
WHITE = (255, 255, 255)


def pixel(point, width, height):
    try:
        x, y = float(point[0]), float(point[1])
        if math.isfinite(x) and math.isfinite(y):
            if 0 < x < width and 0 < y < height:
                return (int(x), int(y))
    except (TypeError, ValueError, IndexError):
        pass
    return None


class ZedPreview:

    def __init__(self, node, camera):
        self.node = node
        self.camera = camera
        self.image = sl.Mat()

        self.locked_id = None
        self.target_valid = False
        self.target_reason = "NO_TARGET"
        self.target_updated = 0.0

        self.event_text = ""
        self.event_until = 0.0

        self.target_sub = node.create_subscription(
            String,
            "/rover/vision/target",
            self.on_target,
            10,
        )

        self.event_sub = node.create_subscription(
            String,
            "/rover/gesture/event",
            self.on_gesture,
            10,
        )

        cv2.namedWindow(
            "BATMOBILE - ZED 2i",
            cv2.WINDOW_NORMAL
        )
        cv2.resizeWindow("BATMOBILE - ZED 2i", 1100, 620)

        print("ZED PREVIEW READY - Press Q to quit", flush=True)

    def on_target(self, msg):
        try:
            data = json.loads(msg.data)
            if not isinstance(data, dict):
                return

            self.locked_id = (
                data.get("person_id")
                if data.get("locked") is True
                else None
            )

            self.target_valid = (
                data.get("valid") is True
                and data.get("tracking_state") == "OK"
                and data.get("reason") == "TRACKING_OK"
            )

            self.target_reason = str(
                data.get("reason", "UNKNOWN")
            )

            self.target_updated = time.monotonic()

        except (ValueError, TypeError):
            pass

    def on_gesture(self, msg):
        try:
            data = json.loads(msg.data)
            if not isinstance(data, dict):
                return

            if (
                data.get("gesture") == "FOLLOW"
                and data.get("confirmed") is True
            ):
                self.event_text = (
                    f"FOLLOW CONFIRMED - PERSON {data.get('person_id')}"
                )
                self.event_until = time.monotonic() + 5.0

        except (ValueError, TypeError):
            pass

    def update(self, bodies):
        # Process ROS callbacks without starting another camera thread.
        rclpy.spin_once(self.node, timeout_sec=0.0)
        rclpy.spin_once(self.node, timeout_sec=0.0)

        result = self.camera.retrieve_image(
            self.image, sl.VIEW.LEFT
        )

        if result != sl.ERROR_CODE.SUCCESS:
            return True

        # ZED VIEW.LEFT uses BGRA pixels.
        frame = cv2.cvtColor(
            self.image.get_data(),
            cv2.COLOR_BGRA2BGR
        )

        height, width = frame.shape[:2]

        target_fresh = (
            time.monotonic() - self.target_updated < 0.5
        )

        for person in bodies.body_list:
            pid = int(person.id)

            is_locked = (
                target_fresh
                and self.target_valid
                and self.locked_id == pid
                and str(person.tracking_state) == "OK"
            )

            color = GREEN if is_locked else YELLOW

            # Draw skeleton.
            keypoints = person.keypoint_2d
            points = {}

            for i, point in enumerate(keypoints):
                p = pixel(point, width, height)
                if p is not None:
                    points[i] = p
                    cv2.circle(frame, p, 4, color, -1)

            for a, b in BONES:
                if a in points and b in points:
                    cv2.line(
                        frame,
                        points[a],
                        points[b],
                        color,
                        2
                    )

            # Draw bounding box.
            box_points = [
                pixel(p, width, height)
                for p in person.bounding_box_2d
            ]
            box_points = [
                p for p in box_points if p is not None
            ]

            if box_points:
                x1 = min(p[0] for p in box_points)
                y1 = min(p[1] for p in box_points)
                x2 = max(p[0] for p in box_points)
                y2 = max(p[1] for p in box_points)

                cv2.rectangle(
                    frame,
                    (x1, y1),
                    (x2, y2),
                    color,
                    2
                )

                label_x = x1
                label_y = max(110, y1 - 10)

            elif points:
                label_x, label_y = next(iter(points.values()))
                label_y = max(110, label_y - 10)

            else:
                continue

            pos = [float(v) for v in person.position]

            distance = (
                math.hypot(pos[0], pos[2])
                if all(math.isfinite(v) for v in pos)
                else None
            )

            distance_text = (
                f"{distance:.2f}m"
                if distance is not None
                else "N/A"
            )

            label = f"ID {pid} | {distance_text}"

            if is_locked:
                label += " | LOCKED"

            cv2.putText(
                frame,
                label,
                (label_x, label_y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                color,
                2
            )

        # Dashboard header.
        cv2.rectangle(
            frame, (0, 0), (width, 95), (25, 25, 25), -1
        )

        cv2.putText(
            frame,
            f"BATMOBILE | DETECTED: {len(bodies.body_list)}",
            (20, 32),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            WHITE,
            2
        )

        if self.locked_id is None:
            status = "TARGET: NOT SELECTED"
            status_color = YELLOW
        elif (
            target_fresh
            and self.target_valid
        ):
            status = f"LOCKED ID: {self.locked_id}"
            status_color = GREEN
        else:
            status = (
                f"TARGET ID {self.locked_id}: "
                f"{self.target_reason}"
            )
            status_color = RED

        cv2.putText(
            frame,
            status,
            (20, 68),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            status_color,
            2
        )

        if time.monotonic() < self.event_until:
            cv2.putText(
                frame,
                self.event_text,
                (20, height - 35),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.9,
                GREEN,
                3
            )

        cv2.imshow("BATMOBILE - ZED 2i", frame)

        # Q closes the camera bridge and preview together.
        return (cv2.waitKey(1) & 0xFF) != ord("q")

    def close(self):
        try:
            self.image.free(sl.MEM.CPU)
            cv2.destroyAllWindows()
        except Exception:
            pass
