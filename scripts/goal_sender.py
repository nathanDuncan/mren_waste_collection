import socket
import goal_position_pb2

PI5_IP = "192.168.12.188"
PORT = 25001

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

goal = goal_position_pb2.GoalPosition()
goal.x = 1.5
goal.y = 0.0
goal.theta = 0.0

serialized_data = goal.SerializeToString()
sock.sendto(serialized_data, (PI5_IP, PORT))

print(f"Goal send to pi5 at: {PI5_IP} {PORT}")

