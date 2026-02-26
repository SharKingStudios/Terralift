import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CameraInfo

class YuyvToMono(Node):
    def __init__(self):
        super().__init__('yuyv_to_mono')

        self.declare_parameter('in_image', '/camera/image_raw')
        self.declare_parameter('in_info',  '/camera/camera_info')
        self.declare_parameter('out_image','/camera/image_mono')
        self.declare_parameter('out_info', '/camera/camera_info_sync')

        self.in_image = self.get_parameter('in_image').value
        self.in_info  = self.get_parameter('in_info').value
        self.out_image= self.get_parameter('out_image').value
        self.out_info = self.get_parameter('out_info').value

        self._last_info = None

        self.pub_img  = self.create_publisher(Image, self.out_image, 10)
        self.pub_info = self.create_publisher(CameraInfo, self.out_info, 10)

        self.sub_info = self.create_subscription(CameraInfo, self.in_info, self._on_info, 10)
        self.sub_img  = self.create_subscription(Image, self.in_image, self._on_image, 10)

        self.get_logger().info(f"Subscribing: {self.in_image}, {self.in_info}")
        self.get_logger().info(f"Publishing:  {self.out_image}, {self.out_info}")

    def _on_info(self, msg: CameraInfo):
        self._last_info = msg

    def _on_image(self, msg: Image):
        # Expect YUYV: 2 bytes per pixel
        w = msg.width
        h = msg.height
        data = msg.data

        expected = w * h * 2
        if len(data) < expected:
            self.get_logger().warn(f"Image data too small: got {len(data)} expected {expected}")
            return

        # YUYV packed: Y0 U0 Y1 V0 ... -> luma is every even byte
        y_bytes = data[0:expected:2]  # length = w*h

        out = Image()
        out.header = msg.header
        out.height = h
        out.width = w
        out.encoding = 'mono8'
        out.is_bigendian = 0
        out.step = w
        out.data = bytes(y_bytes)

        self.pub_img.publish(out)

        # Republish CameraInfo stamped exactly with the image time for sync
        if self._last_info is not None:
            ci = CameraInfo()
            ci = self._last_info
            ci.header.stamp = msg.header.stamp
            ci.header.frame_id = msg.header.frame_id
            self.pub_info.publish(ci)

def main():
    rclpy.init()
    node = YuyvToMono()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()