#!/usr/bin/env python3

import sys
import os
import json
import numpy as np
import cv2
import rospy
import rospkg
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# --- Import RealSense ---
realsense_dir = "/home/pi5/librealsense/build/Release"
if realsense_dir not in sys.path:
    sys.path.append(realsense_dir)

try:
    import pyrealsense2 as rs
except ImportError:
    print(f"❌ Error: Could not find pyrealsense2 in {realsense_dir}")
    sys.exit(1)

from ultralytics import YOLO


class ObjectDetector:
    def __init__(self):
        rospy.init_node("object_detector", anonymous=True)

        self.enable_vis = rospy.get_param("~enable_vis", True)
        self.bridge = CvBridge()

        # 1. Path & Model Loading using rospkg
        # rospack = rospkg.RosPack()
        # package_name = "autonomous_litter_bot_package"

        # try:
        #     package_share = rospack.get_path(package_name)
        # except rospkg.ResourceNotFound:
        #     rospy.logerr(f"❌ Error: Could not find package '{package_name}'")
        #     sys.exit(1)

        # Pointing to the OpenVINO directory
        # self.model_path = os.path.join(
        #     package_share, "models", "segmentation_small_openvino_model"
        # )

        self.model_path = "./segmentation_small_openvino_model"

        rospy.loginfo(f"Loading Model: {self.model_path}")
        self.model = YOLO(self.model_path, task="segment")

        # 2. Publishers
        self.publisher_ = rospy.Publisher("camera_data", String, queue_size=10)
        self.debug_pub_ = rospy.Publisher("debug_image", Image, queue_size=10)

        # 3. Setup RealSense
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 6)
        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 6)
        self.align = rs.align(rs.stream.color)

        profile = self.pipeline.start(self.config)
        color_stream = profile.get_stream(rs.stream.color).as_video_stream_profile()
        self.intrinsics = color_stream.get_intrinsics()

        # ROS 1 Timer (0.1s = 10Hz)
        self.timer = rospy.Timer(rospy.Duration(0.1), self.timer_callback)
        rospy.loginfo("✅ Node Initialized and Running.")

    def timer_callback(self, event=None):
        show_debug = rospy.get_param("~enable_vis", self.enable_vis)

        try:
            frames = self.pipeline.wait_for_frames(timeout_ms=5000)
        except RuntimeError:
            return

        aligned_frames = self.align.process(frames)
        color_frame = aligned_frames.get_color_frame()
        depth_frame = aligned_frames.get_depth_frame()

        if not color_frame or not depth_frame:
            return

        frame = np.asanyarray(color_frame.get_data())
        H, W = frame.shape[:2]
        debug_frame = frame.copy() if show_debug else None

        # --- HUD: Draw Guide Lines (Done ONCE per frame, before detection) ---
        mode = "search"
        mode = "pickup"
        if show_debug and debug_frame is not None:
            if mode == "pickup":
                y_threshold_percentage = 1-0.40
                y_threshold_tolerance = 0.05
                x_threshold_percentage = 1-0.53
                x_threshold_tolerance = 0.05
            elif mode == "search":
                y_threshold_percentage = 1-0.50
                y_threshold_tolerance = 0.01
                x_threshold_percentage = 1-0.53
                x_threshold_tolerance = 0.01

            y_percent = 1 - y_threshold_percentage
            x_percent = 1 - x_threshold_percentage

            # Draw two horizontal guide lines
            cv2.line(
                debug_frame,
                (0, int(y_percent * H - y_threshold_tolerance * H)),
                (W, int(y_percent * H - y_threshold_tolerance * H)),
                (255, 0, 0),
                2,
            )
            cv2.line(
                debug_frame,
                (0, int(y_percent * H + y_threshold_tolerance * H)),
                (W, int(y_percent * H + y_threshold_tolerance * H)),
                (255, 0, 0),
                2,
            )

            # Draw two vertical guide lines
            cv2.line(
                debug_frame,
                (int(x_percent * W - x_threshold_tolerance * W), 0),
                (int(x_percent * W - x_threshold_tolerance * W), H),
                (255, 0, 0),
                2,
            )
            cv2.line(
                debug_frame,
                (int(x_percent * W + x_threshold_tolerance * W), 0),
                (int(x_percent * W + x_threshold_tolerance * W), H),
                (255, 0, 0),
                2,
            )

        # Prediction step (will pause here for a minute on the very first frame)
        results = self.model.predict(frame, imgsz=640, conf=0.75, verbose=False)
        r = results[0]
        detected_list = []

        if r.masks is not None:
            for mask_points in r.masks.xy:
                c = mask_points.astype(np.int32)
                if len(c) < 5:
                    continue

                # --- 1. Geometry Calculation ---
                ellipse = cv2.fitEllipse(c)
                (cx, cy), (MA, ma), angle = ellipse

                cx_int = max(0, min(int(cx), W - 1))
                cy_int = max(0, min(int(cy), H - 1))

                # --- 2. Distance & Real Size ---
                distance_meters = depth_frame.get_distance(cx_int, cy_int)
                if distance_meters <= 0:
                    continue

                real_width_cm = (ma * distance_meters / self.intrinsics.fx) * 100
                real_length_cm = (MA * distance_meters / self.intrinsics.fy) * 100

                if angle > 90:
                    angle_major = angle - 180
                else:
                    angle_major = angle

                angle_major = -angle_major

                # --- 4. Visualization (Object Specific) ---
                if show_debug and debug_frame is not None:
                    cv2.drawContours(debug_frame, [c], -1, (0, 255, 0), 2)
                    cv2.circle(debug_frame, (cx_int, cy_int), 5, (0, 0, 255), -1)

                    labels = [
                        f"Pixels: {cx_int}, {cy_int}",
                        f"Dist: {distance_meters:.2f}m",
                        f"Size: {real_width_cm:.1f}x{real_length_cm:.1f}cm",
                        f"Ang: {angle_major:.1f}deg",
                    ]

                    text_x = min(cx_int + 15, W - 160)
                    text_y = cy_int
                    for line in labels:
                        cv2.putText(
                            debug_frame,
                            line,
                            (text_x, text_y),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.5,
                            (0, 0, 0),
                            1,
                        )
                        text_y += 18

                # --- 5. Data Collection ---
                obj_data = {
                    "x_pos": float(cx),
                    "y_pos": float(cy),
                    "dist_meters": float(distance_meters),
                    "width_cm": float(real_width_cm),
                    "length_cm": float(real_length_cm),
                    "angle": float(angle_major),
                    "height": float(MA),
                    "width": float(ma),
                }
                detected_list.append(obj_data)

        # Publish JSON string
        self.publisher_.publish(json.dumps(detected_list))

        # Publish Debug Image & Show Local Window
        if show_debug and debug_frame is not None:
            self.debug_pub_.publish(self.bridge.cv2_to_imgmsg(debug_frame, "bgr8"))

            # Show local OpenCV window
            cv2.imshow("YOLO + RealSense Output", debug_frame)
            cv2.waitKey(1)

    def cleanup(self):
        rospy.loginfo("Shutting down Object Detector node...")
        self.pipeline.stop()
        cv2.destroyAllWindows()  # Clean up the pop-up window


def main():
    try:
        node = ObjectDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        if "node" in locals():
            node.cleanup()


if __name__ == "__main__":
    main()
