import cv2, socket, base64
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class VideoPublisher(Node):
    def __init__(self):
        super().__init__('video_publisher')
        self.publisher_ = self.create_publisher(Image, 'video_topic', 10)
        self.bridge = CvBridge()

    def publish_frame(self, frame):
        msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        self.publisher_.publish(msg)

def main():
    # ROS2 Node initialization
    rclpy.init(args=None)
    video_publisher = VideoPublisher()
    print("Init node")

    # UDP socket setup
    BUFF_SIZE = 65536
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    client_socket.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, BUFF_SIZE)
    host_ip = '172.27.197.126'  # IP address of the multipass instance
    port = 9999

    client_socket.sendto(b'Hello', (host_ip, port))
    assembled_data = b''
    print("done init")

    while rclpy.ok():
        print("spin")
        rclpy.spin_once(video_publisher, timeout_sec=0.1)
        try:
            packet, _ = client_socket.recvfrom(BUFF_SIZE)
            if packet == b'EOS':
                print('EOS signal received. Closing connection.')
                break
            elif packet == b'EOF':
                print("EOF received")
                # Process the assembled data
                if assembled_data:
                    data = base64.b64decode(assembled_data)
                    npdata = np.frombuffer(data, dtype=np.uint8)
                    frame = cv2.imdecode(npdata, 1)
                    if frame is None:
                        frame = np.zeros((480, 640, 3), dtype=np.uint8)
                    assembled_data = b''
                    client_socket.sendto(b'ACK', (host_ip, port))
                    video_publisher.publish_frame(frame)
            else:
                print("Assembling Chuncks")
                # Assemble the chunks
                assembled_data += packet
        except socket.timeout:
            print("socket timeout")
            continue

    # Send EOS to the server before shutting down
    client_socket.sendto(b'EOS', (host_ip, port))
    client_socket.close()

    # Shutdown ROS node
    rclpy.shutdown()

if __name__ == '__main__':
    main()
