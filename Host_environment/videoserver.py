# This is server code to send video frames over UDP
import cv2, socket
import numpy as np
import time
import base64

# Constants
BUFF_SIZE = 65536
CHUNK_SIZE = 9216  # Set the chunk size to 9216 bytes to be compliant with MacOS

PORT = 9999

# Setting up Server connection
server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, BUFF_SIZE)
host_name = socket.gethostname()
host_ip = '192.168.56.1'  # Fill in the ip address of the multipass instance
# host_ip = '0.0.0.0'  # Fill in the ip address of the multipass instance
# host_ip = '127.0.0.1'  # Fill in the ip address of the multipass instance
print(host_ip)
port = 9999
socket_address = (host_ip, port)
server_socket.bind(socket_address)
print('Listening at:', socket_address)

# Video Capture
vid = cv2.VideoCapture(0)
fps, st, frames_to_count, cnt = (0, 0, 20, 0)
WIDTH = 400

while True:
    msg, client_addr = server_socket.recvfrom(BUFF_SIZE)
    print('GOT connection from ', client_addr)
    while vid.isOpened():
        _, frame = vid.read()
        frame = cv2.resize(frame, (0,0), fx=0.5, fy=0.5)
        encoded, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
        message = base64.b64encode(buffer)

        # Split the message into chunks of size CHUNK_SIZE
        chunks = [message[i:i + CHUNK_SIZE] for i in range(0, len(message), CHUNK_SIZE)]
        for chunk in chunks:
            server_socket.sendto(chunk, client_addr)

        # Send end-of-frame marker
        server_socket.sendto(b'EOF', client_addr)

        # Wait for acknowledgment from client
        # ack, _ = server_socket.recvfrom(BUFF_SIZE)
        # print (ack)
        # print(_)

        frame = cv2.putText(frame, 'FPS: ' + str(fps), (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        cv2.imshow('TRANSMITTING VIDEO', frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            server_socket.sendto(b'EOS', client_addr)  # Sends an end of stream signal to the client
            server_socket.close()
            break
        if cnt == frames_to_count:
            try:
                fps = round(frames_to_count / (time.time() - st))
                st = time.time()
                cnt = 0
            except:
                pass
        cnt += 1
