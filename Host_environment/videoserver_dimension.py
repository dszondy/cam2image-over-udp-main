# This is server code to send video frames over UDP
from types import NoneType
import cv2, socket
import numpy as np
import time
import base64
import json

# Constants
BUFF_SIZE = 65536
CHUNK_SIZE = 9216  # Set the chunk size to 9216 bytes to be compliant with MacOS
PORT = 9999
HOST_IP = '192.168.56.1'  # Fill in the ip address of the multipass instance
WIDTH = 640
HEIGHT = 480

# Setting up Server connection
server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, BUFF_SIZE)
host_name = socket.gethostname()
print(HOST_IP)
socket_address = (HOST_IP, PORT)
server_socket.bind(socket_address)
print('Listening at:', socket_address)

# Video Capture
vid = cv2.VideoCapture(0, cv2.CAP_DSHOW)
vid.set(cv2.CAP_PROP_FPS, 30.0)
vid.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc('m','j','p','g'))
vid.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc('M','J','P','G'))
fps, st, frames_to_count, cnt = (0, 0, 20, 0)
do_cap = True
width = WIDTH
height = HEIGHT

while do_cap:
    msg, client_addr = server_socket.recvfrom(BUFF_SIZE)
    print('GOT connection from ', client_addr)

    # Process frame size
    dec_msg = base64.b64decode(msg)
    try:
        json_object = json.loads(dec_msg)
        width = json_object["width"]
        height = json_object["height"]
    except ValueError:  # includes simplejson.decoder.JSONDecodeError
        print('Decoding JSON has failed. Using default dimensions')

    # Limit size to prevent buffer issues
    if width > WIDTH:
        width = WIDTH
    if height > HEIGHT:
        height = HEIGHT

    vid.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    vid.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    print(f"Capture dimension set to {width}x{height}")

    while vid.isOpened():
        _, frame = vid.read()
        if type(frame) == NoneType:
            continue
        else:
            encoded, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
        message = base64.b64encode(buffer)

        # Split the message into chunks of size CHUNK_SIZE
        chunks = [message[i:i + CHUNK_SIZE] for i in range(0, len(message), CHUNK_SIZE)]
        for chunk in chunks:
            server_socket.sendto(chunk, client_addr)

        # Send end-of-frame marker
        server_socket.sendto(b'EOF', client_addr)

        frame = cv2.putText(frame, 'FPS: ' + str(fps), (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        cv2.imshow('TRANSMITTING VIDEO', frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            server_socket.sendto(b'EOS', client_addr)  # Sends an end of stream signal to the client
            server_socket.close()
            do_cap = False
            break
        if cnt == frames_to_count:
            try:
                fps = round(frames_to_count / (time.time() - st))
                st = time.time()
                cnt = 0
            except:
                pass
        cnt += 1

vid.release()
cv2.destroyAllWindows()