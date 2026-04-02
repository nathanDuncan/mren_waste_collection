#!/usr/bin/env python3
import socket
import states_pb2
import sys

def send_command(action, ip="127.0.0.1", port=25002):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    state = states_pb2.States()
    state.action = action
    
    serialized_data = state.SerializeToString()
    sock.sendto(serialized_data, (ip, port))
    print(f"Sent '{action}' command to {ip}:{port}")
    sock.close()

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python3 test_idle_cmd.py <sit|stand> [ip]")
        sys.exit(1)
    
    cmd = sys.argv[1]
    target_ip = sys.argv[2] if len(sys.argv) > 2 else "127.0.0.1"
    
    send_command(cmd, target_ip)
