import cv2, socket, base64
import numpy as np
import os

os.environ['DISPLAY'] = '192.168.64.1:0.0'

BUFF_SIZE = 65536
client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
client_socket.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, BUFF_SIZE)
host_ip = '192.168.64.1'  # IP address of the multipass instance
port = 9999

client_socket.sendto(b'Hello', (host_ip, port))

assembled_data = b''

i = 1

while True:
    packet, _ = client_socket.recvfrom(BUFF_SIZE)
    if packet == b'EOS':
        print('EOS signal received. Closing connection.')
        client_socket.close()
        break
    elif packet == b'EOF':
        i +=1
        print("EOF" + str(i))
        # try:
            
        # Process the assembled data
        if assembled_data:
            data = base64.b64decode(assembled_data)
            npdata = np.frombuffer(data, dtype=np.uint8)
            frame = cv2.imdecode(npdata, 1)
            if frame is None:
                frame = np.zeros((480, 640, 3), dtype=np.uint8)
            cv2.imshow('VIRTUAL MACHINE', frame)
            if cv2.waitKey(25) & 0xFF == ord('q'):
                print("Sending EOS signal.")
                client_socket.sendto(b'EOS', (host_ip, port))
                cv2.destroyAllWindows()
                break
            assembled_data = b''
            client_socket.sendto(b'ACK', (host_ip, port))
        # except cv2.error:
        #     print("broken ")
    else:
        # Assemble the chunks
        "other"
        assembled_data += packet

