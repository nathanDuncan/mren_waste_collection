import socket
import message_pb2

# Configuration
PORT = 25000

# Create and bind the UDP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("0.0.0.0", PORT))

print(f"Listening for Protobuf packets on port {PORT}...")

while True:
    # 1. Receive raw binary data
    data, addr = sock.recvfrom(1024) # Buffer size 1024 bytes

    # 2. Initialize the Protobuf object
    received_msg = message_pb2.LitterPoint()

    try:
        # 3. Parse the binary back into an object
        received_msg.ParseFromString(data)

        print(f"--- New Message from {addr[0]} ---")
        print(f"x:     {received_msg.x}")
        print(f"y:   {received_msg.y}")
        print(f"depth:   {received_msg.depth}")
        print(f"angle: {received_msg.angle}\n")

    except Exception as e:
        print(f"Failed to parse message: {e}")