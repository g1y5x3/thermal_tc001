import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class ThermalCameraNode(Node):
    """
    A ROS 2 Node that captures thermal data from a V4L2 device,
    processes it (cropping and colormapping), and publishes it as a sensor_msgs/Image.
    """
    def __init__(self):
        super().__init__('thermal_camera_node')

        self.declare_parameter('device_id', 0)
        self.declare_parameter('frame_id', 'thermal_link')
        self.declare_parameter('publish_rate', 30.0) # Hz

        device_id = self.get_parameter('device_id').get_parameter_value().integer_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value

        # Initialize Video Capture
        device_path = f'/dev/video{device_id}'
        self.get_logger().info(f"Opening device: {device_path}")
        
        self.cap = cv2.VideoCapture(device_path, cv2.CAP_V4L)
        
        if not self.cap.isOpened():
            self.get_logger().error(f"Failed to open {device_path}. Check permissions or device index.")
            raise RuntimeError(f"Could not open {device_path}")

        # Initialize Bridge and Publisher
        self.bridge = CvBridge()
        self.publisher_ = self.create_publisher(Image, 'thermal/image_raw', 10)

        # Create Timer for the loop
        timer_period = 1.0 / publish_rate
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info("Thermal Node started successfully.")

    def timer_callback(self):
        """Main loop: capture, process, and publish."""
        ret, frame = self.cap.read()

        if ret:
            try:
                h, w, _ = frame.shape
                clean_thermal = frame[0:h//2, 0:w]

                thermal_colormap = cv2.applyColorMap(clean_thermal, cv2.COLORMAP_JET)

                msg = self.bridge.cv2_to_imgmsg(thermal_colormap, encoding="bgr8")
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = self.frame_id

                self.publisher_.publish(msg)

            except Exception as e:
                self.get_logger().error(f"Error processing frame: {str(e)}")
        else:
            self.get_logger().warn("Failed to capture frame from camera.")

    def __del__(self):
        if hasattr(self, 'cap') and self.cap.isOpened():
            self.cap.release()

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = ThermalCameraNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Node terminated with error: {e}")
    finally:
        if 'node' in locals():
            node.cap.release()
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()