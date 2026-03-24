import pyrealsense2 as rs
import numpy as np
import cv2
import time

def main():
    pipeline = rs.pipeline()
    config = rs.config()

    # TEST 1: Enable ONLY Color at low resolution/FPS to isolate the bus
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 15)

    print("Attempting to open COLOR ONLY stream...")
    try:
        profile = pipeline.start(config)
        print("Pipeline started. Waiting for first frame...")
        
        # Give the camera a moment to settle
        time.sleep(2)

        while True:
            # Increase timeout to 5 seconds for the first test
            frames = pipeline.wait_for_frames(timeout_ms=5000)
            color_frame = frames.get_color_frame()

            if not color_frame:
                print("Empty frame received.")
                continue

            color_image = np.asanyarray(color_frame.get_data())
            cv2.imshow('RealSense Color Test', color_image)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    except Exception as e:
        print(f"Hardware Error: {e}")
    finally:
        pipeline.stop()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()