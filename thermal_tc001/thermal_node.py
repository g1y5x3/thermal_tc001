import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from cv_bridge import CvBridge
import cv2
import math

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
        
        # Force a specific resolution if needed, though V4L usually detects the native one
        self.cap = cv2.VideoCapture(device_path, cv2.CAP_V4L)
        
        if not self.cap.isOpened():
            self.get_logger().error(f"Failed to open {device_path}. Check permissions or device index.")
            raise RuntimeError(f"Could not open {device_path}")

        # Initialize Bridge and Publisher
        self.bridge = CvBridge()
        self.publisher_ = self.create_publisher(Image, 'thermal/image', 10)
        self.camera_info_publisher = self.create_publisher(CameraInfo, 'thermal/camera_info', 10)

        # Create Timer for the loop
        timer_period = 1.0 / publish_rate
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info(f"Thermal Node started. Publishing to frame: {self.frame_id}")

    def timer_callback(self):
        """Main loop: capture, process, and publish."""
        ret, frame = self.cap.read()

        if ret:
            try:
                # 1. Process the Image (Keep top half)
                h_raw, w_raw, _ = frame.shape
                
                # Topdon TC001 sends visual data in top half, telemetry in bottom
                clean_thermal = frame[0:h_raw//2, 0:w_raw]
                
                # Apply colormap for visualization
                thermal_colormap = cv2.applyColorMap(clean_thermal, cv2.COLORMAP_JET)

                # 2. Publish Image
                msg = self.bridge.cv2_to_imgmsg(thermal_colormap, encoding="bgr8")
                
                # Critical: Use the same timestamp for both Image and CameraInfo
                current_time = self.get_clock().now().to_msg()
                
                msg.header.stamp = current_time
                msg.header.frame_id = self.frame_id
                self.publisher_.publish(msg)

                # 3. Publish Camera Info (The missing step!)
                # We pass the processed dimensions to ensure metadata matches the image
                h_processed, w_processed, _ = clean_thermal.shape
                self.publish_camera_info(current_time, w_processed, h_processed)

            except Exception as e:
                self.get_logger().error(f"Error processing frame: {str(e)}")
        else:
            self.get_logger().warn("Failed to capture frame from camera.")

    def publish_camera_info(self, header_stamp, width, height):
        """
        Publishes synthetic CameraInfo for Topdon TC001.
        Calculates intrinsics dynamically based on the image width/height provided.
        """
        msg = CameraInfo()
        msg.header.stamp = header_stamp
        msg.header.frame_id = self.frame_id # Use the parameter, not a hardcoded string

        msg.width = width
        msg.height = height

        # Distortion (Assuming 0 since we can't calibrate)
        msg.distortion_model = "plumb_bob"
        msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]

        # Calculate Focal Length from FOV
        # Topdon TC001 Spec: 56 degree horizontal FOV
        fov_h_deg = 56.0
        
        # f = (w/2) / tan(angle/2)
        f_pixels = (width / 2.0) / math.tan(math.radians(fov_h_deg / 2.0))

        # Principal Point (Center of image)
        cx = width / 2.0
        cy = height / 2.0

        # Intrinsic Matrix K (3x3)
        msg.k = [f_pixels, 0.0,      cx,
                 0.0,      f_pixels, cy,
                 0.0,      0.0,      1.0]

        # Rectification Matrix R (Identity)
        msg.r = [1.0, 0.0, 0.0,
                 0.0, 1.0, 0.0,
                 0.0, 0.0, 1.0]

        # Projection Matrix P (3x4)
        msg.p = [f_pixels, 0.0,      cx,      0.0,
                 0.0,      f_pixels, cy,      0.0,
                 0.0,      0.0,      1.0,     0.0]

        self.camera_info_publisher.publish(msg)

    def __del__(self):
        if hasattr(self, 'cap') and self.cap.isOpened():
            self.cap.release()

def main(args=None):
    rclpy.init(args=args)
    
    node = None
    try:
        node = ThermalCameraNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Node terminated with error: {e}")
    finally:
        if node is not None:
            # Ensure proper cleanup
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
