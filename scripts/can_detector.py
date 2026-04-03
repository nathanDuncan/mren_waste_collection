#!/usr/bin/env python3

import os
import sys

# Get the absolute path to the directory this script is in
script_dir = os.path.dirname(os.path.abspath(__file__))

# Change the working directory to the script's directory
os.chdir(script_dir)

# Add the script's directory to the Python path just in case
if script_dir not in sys.path:
    sys.path.append(script_dir)

import json
import numpy as np
import cv2
import threading
import time
import socket
import struct
import zlib
from ultralytics import YOLO

# Import generated Protobuf
script_dir = os.path.dirname(os.path.abspath(__file__))
if script_dir not in sys.path:
    sys.path.append(script_dir)

try:
    import detection_pb2
    import image_stream_pb2
except ImportError as e:
    print(f"[ERROR] Failed to import protobuf modules: {e}")
    # This will tell you if it's 'No module named google' or the actual file
    sys.exit(1)

PI4_IP = "192.168.12.128"

class CanDetector:
    def __init__(self):
        # 1. Configuration
        self.network_port = 25005
        self.target_ip = PI4_IP
        self.target_port = 25006
        self.enable_vis = True
        
        # 2. Load YOLO Model
        # Assuming the same relative path as in trash_detection_node.py
        # package_path/models/segmentation_nano_openvino_model
        # But here it's likely in the same dir or models dir relative to script
        self.model_path = os.path.join(script_dir, '..', 'models', 'segmentation_nano_openvino_model')
        if not os.path.exists(self.model_path):
            # Fallback for local BEST.PT if available
            self.model_path = os.path.join(script_dir, 'best.pt')
            
        print(f"Loading Model: {self.model_path}")
        try:
            self.model = YOLO(self.model_path, task="segment")
            print("[INFO] YOLO Model loaded successfully.")
        except Exception as e:
            print(f"[ERROR] Failed to load YOLO Model: {e}")
            sys.exit(1)

        # 3. UDP Socket for Detection Results
        self.udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        
        # 4. TCP Server for Image Stream
        self.server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_sock.bind(("0.0.0.0", self.network_port))
        self.server_sock.listen(1)
        self.server_sock.settimeout(1.0)
        
        print(f"✅ CanDetector Initialized.")
        print(f"   Listening for images on TCP port {self.network_port}")
        print(f"   Sending detections to {self.target_ip}:{self.target_port}")

    def recv_all(self, conn, n):
        """Helper function to receive exactly n bytes or return None if connection closes."""
        data = bytearray()
        while len(data) < n:
            try:
                packet = conn.recv(n - len(data))
                if not packet:
                    return None
                data.extend(packet)
            except socket.timeout:
                continue
            except Exception as e:
                print(f"Receive error: {e}")
                return None
        return data

    def process_frame(self, msg):
        """Process incoming ImageFrame protobuf message."""
        try:
            # 1. Decode Color Image
            colour_np = np.frombuffer(msg.colour_data, dtype=np.uint8)
            frame = cv2.imdecode(colour_np, cv2.IMREAD_COLOR)
            if frame is None:
                return

            # 2. Decompress and Reshape Depth Image
            depth_decompressed = zlib.decompress(msg.depth_data)
            depth_img = np.frombuffer(depth_decompressed, dtype=np.uint16).reshape((msg.height, msg.width))
            
            # 3. Get Intrinsics & Scale
            # In image_stream.proto: fx, fy, ppx, ppy, depth_scale
            fx, fy = msg.fx, msg.fy
            ppx, ppy = msg.ppx, msg.ppy
            depth_scale = msg.depth_scale
            H, W = frame.shape[:2]

            # 4. YOLO Prediction
            results = self.model.predict(frame, imgsz=640, conf=0.75, verbose=False)
            r = results[0]
            detected_list = []

            debug_frame = frame.copy() if self.enable_vis else None

            if r.masks is not None:
                for mask_points in r.masks.xy:
                    c = mask_points.astype(np.int32)
                    if len(c) < 5: continue 

                    ellipse = cv2.fitEllipse(c)
                    (cx, cy), (MA, ma), angle = ellipse
                    
                    cx_int = max(0, min(int(cx), W - 1))
                    cy_int = max(0, min(int(cy), H - 1))

                    # Calculate distance in meters from depth map
                    dist_raw = depth_img[cy_int, cx_int]
                    distance_meters = float(dist_raw) * depth_scale
                    
                    if distance_meters <= 0: continue 

                    # Real-world size logic (consistent with trash_detection_node.py)
                    real_width_cm = (ma * distance_meters / fx) * 100
                    real_length_cm = (MA * distance_meters / fy) * 100
                    
                    # Angle Logic
                    if angle > 90:
                        angle_major = angle - 180
                    else:
                        angle_major = angle
                    angle_major = -angle_major

                    if self.enable_vis:
                        cv2.drawContours(debug_frame, [c], -1, (0, 255, 0), 2)
                        cv2.circle(debug_frame, (cx_int, cy_int), 5, (0, 0, 255), -1)
                        cv2.putText(debug_frame, f"{distance_meters:.2f}m", (cx_int + 10, cy_int), 
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)

                    obj_data = {
                        "x_pos": float(cx), "y_pos": float(cy),
                        "dist_meters": float(distance_meters),
                        "width_cm": float(real_width_cm),
                        "length_cm": float(real_length_cm),
                        "angle": float(angle_major),
                        "area": float(cv2.contourArea(c))
                    }
                    detected_list.append(obj_data)

            # 5. Select largest object and send via UDP
            if detected_list:
                largest_obj = max(detected_list, key=lambda x: x['area'])
                
                try:
                    frame_pb = detection_pb2.DetectionFrame()
                    obj_pb = frame_pb.objects.add()
                    obj_pb.x_pos = largest_obj['x_pos']
                    obj_pb.y_pos = largest_obj['y_pos']
                    obj_pb.dist_meters = largest_obj['dist_meters']
                    obj_pb.width_cm = largest_obj['width_cm']
                    obj_pb.length_cm = largest_obj['length_cm']
                    obj_pb.angle = largest_obj['angle']

                    serialized_data = frame_pb.SerializeToString()
                    self.udp_sock.sendto(serialized_data, (self.target_ip, self.target_port))
                    print(f"[INFO] Sent Detection: {largest_obj['dist_meters']:.2f}m at ({largest_obj['x_pos']:.0f}, {largest_obj['y_pos']:.0f})")
                except Exception as e:
                    print(f"[ERROR] Protobuf Sending Error: {e}")
            else:
                # Send "no detection" frame (all negative values)
                try:
                    frame_pb = detection_pb2.DetectionFrame()
                    obj_pb = frame_pb.objects.add()
                    obj_pb.x_pos = -1.0
                    obj_pb.y_pos = -1.0
                    obj_pb.dist_meters = -1.0
                    obj_pb.width_cm = -1.0
                    obj_pb.length_cm = -1.0
                    obj_pb.angle = -1.0

                    serialized_data = frame_pb.SerializeToString()
                    self.udp_sock.sendto(serialized_data, (self.target_ip, self.target_port))
                    print("[INFO] Sent No Detection Frame")
                except Exception as e:
                    print(f"[ERROR] Protobuf Sending Error (No Detection): {e}")

            if self.enable_vis:
                cv2.imshow("Can Detector Debug", debug_frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    return False
            
            return True

        except Exception as e:
            print(f"Error processing frame: {e}")
            return True

    def run(self):
        while True:
            print("Waiting for a connection...")
            try:
                conn, addr = self.server_sock.accept()
                print(f"Connected by {addr}")
                conn.settimeout(1.0)
            except socket.timeout:
                continue
            except KeyboardInterrupt:
                break

            with conn:
                while True:
                    try:
                        # 1. Read the 4-byte length header
                        header = self.recv_all(conn, 4)
                        if header is None:
                            print("Sender disconnected.")
                            break
                        
                        msg_len = struct.unpack('>I', header)[0]
                        
                        # 2. Read the actual Protobuf payload
                        payload = self.recv_all(conn, msg_len)
                        if payload is None:
                            print("Connection lost during payload transfer.")
                            break
                        
                        # 3. Parse Protobuf
                        msg = image_stream_pb2.ImageFrame()
                        msg.ParseFromString(payload)
                        
                        # 4. Process
                        if not self.process_frame(msg):
                            break

                    except KeyboardInterrupt:
                        return
                    except Exception as e:
                        print(f"Session error: {e}")
                        break

if __name__ == '__main__':
    detector = CanDetector()
    try:
        detector.run()
    except KeyboardInterrupt:
        print("\nShutting down detector...")
    finally:
        cv2.destroyAllWindows()
