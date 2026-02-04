import cv2
import numpy as np
import rospy
import pyrealsense2 as rs
from std_msgs.msg import Int32MultiArray

DEPTH_THRESHOLD = 0.3
MOVE_DISTANCE = 0.05

# ---------------------------------------------------------
# ROS Setup
# ---------------------------------------------------------
rospy.init_node("camera_publisher", anonymous=True)
pub = rospy.Publisher("camera_data", Int32MultiArray, queue_size=10)
# TODO: Want to make it so that subscriber only takes 1 msg at a time so doesnt get stuck on old stuff. maybe change queue size to 1?
rate = rospy.Rate(60)

# ---------------------------------------------------------
# Realsense Init
# ---------------------------------------------------------
pipe = rs.pipeline()
cfg = rs.config()

cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
# cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
pipe.start(cfg)


# cap = cv2.VideoCapture(0)
# if not cap.isOpened():
#     raise RuntimeError("Could not open webcam.")
# print("Webcam initialized. Press 'q' to quit.")

# ---------------------------------------------------------
# HSV Range for ORANGE
# (adjust if needed based on lighting)
# ---------------------------------------------------------
# Lower/darker orange
lower_orange = np.array([5, 120, 120])
upper_orange = np.array([25, 255, 255])

try:
    while not rospy.is_shutdown():
        # ret, frame = cap.read()
        # if not ret:
        #     continue

        frame = pipe.wait_for_frames()
        frame = frame.get_color_frame()
        frame = np.asanyarray(frame.get_data())

        # H, W = frame.shape[:2]
        H = 480
        W = 640

        # Draw two horizontal guide lines, number 0.XX is percent down the frame
        cv2.line(
            frame,
            (0, int(0.40 * H - 0.10 * H)),
            (W, int(0.40 * H - 0.10 * H)),
            (255, 0, 0),
            2,
        )
        cv2.line(
            frame,
            (0, int(0.40 * H + 0.10 * H)),
            (W, int(0.40 * H + 0.10 * H)),
            (255, 0, 0),
            2,
        )

        # -----------------------------------------------------
        # ORANGE OBJECT DETECTION (HSV Thresholding)
        # -----------------------------------------------------
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, lower_orange, upper_orange)

        # Morphological filtering for cleaner masks
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))

        # -----------------------------------------------------
        # Find contours (largest orange object)
        # -----------------------------------------------------
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if len(contours) > 0:
            c = max(contours, key=cv2.contourArea)
            if len(c) >= 5:
                ellipse = cv2.fitEllipse(c)
                (cx, cy), (MA, ma), angle = ellipse

                # Draw fitted ellipse
                cv2.ellipse(frame, ellipse, (255, 0, 0), 2)

                # ---------------------------------------------
                # Normalize major-axis angle to [-90, 90]
                # ---------------------------------------------
                angle_major = angle
                if MA < ma:
                    angle_major += 90
                angle_major = (angle_major + 90) % 180 - 90

                # Draw text
                cv2.putText(
                    frame,
                    f"{angle_major:.1f} deg | origin: ({int(cx)}, {int(cy)})",
                    (int(cx), int(cy)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (0, 0, 255),
                    2,
                )

                # -------------------------------------------------
                # Publish centroid to ROS
                # -------------------------------------------------
                msg = Int32MultiArray()
                msg.data = [int(cx), int(cy)]
                pub.publish(msg)
                rospy.loginfo(f"Published: {msg.data}")
                rate.sleep()

        # ---------------------------------------------------------
        # Display
        # ---------------------------------------------------------
        cv2.imshow("Orange Object Detection + Ellipse Fit", frame)
        # cv2.imshow("Mask", mask)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

finally:
    # cap.release()
    cv2.destroyAllWindows()
