"""Single camera owner: fresh BODY_18 observations, CAMERA/IMAGE meters."""
import json
import math

import pyzed.sl as sl
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


def main():
    rclpy.init()
    node = Node("zed_body_bridge")
    pub = node.create_publisher(
        String, "/rover/vision/bodies", 10
    )

    cam = sl.Camera()
    cfg = sl.InitParameters()
    cfg.camera_resolution = sl.RESOLUTION.HD720
    cfg.camera_fps = 15
    cfg.depth_mode = sl.DEPTH_MODE.NEURAL_LIGHT
    cfg.coordinate_units = sl.UNIT.METER
    cfg.coordinate_system = sl.COORDINATE_SYSTEM.IMAGE

    camera_open = False
    pose_on = False
    body_on = False

    try:
        result = cam.open(cfg)
        if result != sl.ERROR_CODE.SUCCESS:
            raise RuntimeError(f"Camera: {result}")
        camera_open = True

        result = cam.enable_positional_tracking(
            sl.PositionalTrackingParameters()
        )
        if result != sl.ERROR_CODE.SUCCESS:
            raise RuntimeError(f"Tracking: {result}")
        pose_on = True

        bt = sl.BodyTrackingParameters()
        bt.detection_model = sl.BODY_TRACKING_MODEL.HUMAN_BODY_FAST
        bt.body_format = sl.BODY_FORMAT.BODY_18
        bt.enable_tracking = True
        bt.enable_body_fitting = False
        # SDK defaults to 0.2 s of predictions that may still report OK.
        # Stage 2 must only count actual tracked detections as fresh.
        bt.prediction_timeout_s = 0.0

        result = cam.enable_body_tracking(bt)
        if result != sl.ERROR_CODE.SUCCESS:
            raise RuntimeError(f"Body tracking: {result}")
        body_on = True

        runtime = sl.BodyTrackingRuntimeParameters()
        runtime.detection_confidence_threshold = 40

        grab_runtime = sl.RuntimeParameters()
        grab_runtime.measure3D_reference_frame = sl.REFERENCE_FRAME.CAMERA

        bodies = sl.Bodies()
        print("ZED BODY BRIDGE READY", flush=True)

        while rclpy.ok():
            if cam.grab(grab_runtime) != sl.ERROR_CODE.SUCCESS:
                continue

            result = cam.retrieve_bodies(bodies, runtime)
            if result != sl.ERROR_CODE.SUCCESS or not bodies.is_new:
                continue  # Never give a cached body result a fresh timestamp.
            people = []

            if result == sl.ERROR_CODE.SUCCESS:
                for p in bodies.body_list:
                    x, y, z = [float(v) for v in p.position]

                    if not all(math.isfinite(v) for v in (x, y, z)):
                        continue

                    if z <= 0:
                        continue

                    people.append({
                        "id": int(p.id),
                        "x_m": round(x, 3),
                        "y_m": round(y, 3),
                        "z_m": round(z, 3),
                        "distance_m": round(math.hypot(x, z), 3),
                        "bearing_deg": round(
                            math.degrees(math.atan2(x, z)), 2
                        ),
                        "tracking_state": str(p.tracking_state),
                        "keypoints_m": [
                            [
                                float(v) if math.isfinite(float(v)) else None
                                for v in pt
                            ]
                            for pt in p.keypoint
                        ]
                    })

            msg = String()
            msg.data = json.dumps({
                "timestamp": bodies.timestamp.get_nanoseconds() / 1e9,
                "detected": len(people) > 0,
                "people": people
            }, allow_nan=False)

            pub.publish(msg)

    except KeyboardInterrupt:
        pass
    finally:
        if body_on:
            cam.disable_body_tracking()
        if pose_on:
            cam.disable_positional_tracking()
        if camera_open:
            cam.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
