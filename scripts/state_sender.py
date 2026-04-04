import socket
import states_pb2

PI5_IP = "192.168.12.188"
PORT = 25002

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

state = states_pb2.States()
state.action = "can picked up"

serialized_data = state.SerializeToString()
sock.sendto(serialized_data, (PI5_IP, PORT))

print(f"Goal send to pi5 at: {PI5_IP} {PORT}")

