import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import socket

class RawUdpForwarder(Node):
    def __init__(self):
        super().__init__("raw_udp_forwarder")
        
        self.target_ip = "10.68.0.133" 
        self.target_port = 5900
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        
        # Max size for a safe UDP chunk (under 64KB)
        self.chunk_size = 60000 

        self.create_subscription(
            Image,
            "/xtion/rgb/image_raw",
            self.callback,
            10
        )
        self.get_logger().info("Streaming raw chunks via UDP...")

    def callback(self, msg):
        raw_data = msg.data.tobytes()
        total_size = len(raw_data)
        
        # Send chunks
        for i in range(0, total_size, self.chunk_size):
            chunk = raw_data[i : i + self.chunk_size]
            try:
                self.sock.sendto(chunk, (self.target_ip, self.target_port))
            except Exception as e:
                self.get_logger().error(f"UDP Send failed: {e}")

def main():
    rclpy.init()
    node = RawUdpForwarder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.sock.close()
        rclpy.shutdown()

if __name__ == "__main__":
    main()