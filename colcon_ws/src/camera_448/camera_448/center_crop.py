import copy
import struct

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, CompressedImage, Image


class CenterCrop(Node):
    """Center-crop aligned RGB-D images and preserve their calibration."""

    def __init__(self):
        super().__init__('center_crop')
        self.declare_parameter('crop_size', 448)
        self.crop_size = int(self.get_parameter('crop_size').value)
        self.declare_parameter('jpeg_quality', 90)
        self.jpeg_quality = int(self.get_parameter('jpeg_quality').value)
        self.declare_parameter('depth_png_level', 3)
        self.depth_png_level = int(
            self.get_parameter('depth_png_level').value)
        if self.crop_size <= 0:
            raise ValueError('crop_size must be positive')
        if not 1 <= self.jpeg_quality <= 100:
            raise ValueError('jpeg_quality must be between 1 and 100')
        if not 0 <= self.depth_png_level <= 9:
            raise ValueError('depth_png_level must be between 0 and 9')

        self.bridge = CvBridge()
        self._warned_shapes = set()

        self.color_image_pub = self.create_publisher(
            Image, '/camera_448/color/image_raw', qos_profile_sensor_data)
        self.color_compressed_pub = self.create_publisher(
            CompressedImage, '/camera_448/color/image_raw/compressed',
            qos_profile_sensor_data)
        self.depth_image_pub = self.create_publisher(
            Image, '/camera_448/aligned_depth_to_color/image_raw',
            qos_profile_sensor_data)
        self.depth_compressed_pub = self.create_publisher(
            CompressedImage,
            '/camera_448/aligned_depth_to_color/image_raw/compressedDepth',
            qos_profile_sensor_data)
        self.color_info_pub = self.create_publisher(
            CameraInfo, '/camera_448/color/camera_info', 10)
        self.depth_info_pub = self.create_publisher(
            CameraInfo, '/camera_448/aligned_depth_to_color/camera_info', 10)

        self.create_subscription(
            Image, '/camera/camera/color/image_raw',
            lambda msg: self._crop_image(
                msg, self.color_image_pub, publish_jpeg=True),
            qos_profile_sensor_data)
        self.create_subscription(
            Image, '/camera/camera/aligned_depth_to_color/image_raw',
            lambda msg: self._crop_image(
                msg, self.depth_image_pub, publish_depth_png=True),
            qos_profile_sensor_data)
        self.create_subscription(
            CameraInfo, '/camera/camera/color/camera_info',
            lambda msg: self._crop_info(msg, self.color_info_pub), 10)
        self.create_subscription(
            CameraInfo, '/camera/camera/aligned_depth_to_color/camera_info',
            lambda msg: self._crop_info(msg, self.depth_info_pub), 10)

        self.get_logger().info(
            f'Publishing centered {self.crop_size}x{self.crop_size} RGB-D crops '
            f'under /camera_448; RGB JPEG quality={self.jpeg_quality}, '
            f'depth PNG level={self.depth_png_level}')

    def _offsets(self, width, height):
        if width < self.crop_size or height < self.crop_size:
            shape = (width, height)
            if shape not in self._warned_shapes:
                self.get_logger().error(
                    f'Cannot crop {self.crop_size}x{self.crop_size} from '
                    f'{width}x{height}')
                self._warned_shapes.add(shape)
            return None
        return ((width - self.crop_size) // 2,
                (height - self.crop_size) // 2)

    def _crop_image(self, msg, publisher, publish_jpeg=False,
                    publish_depth_png=False):
        offsets = self._offsets(msg.width, msg.height)
        if offsets is None:
            return
        x0, y0 = offsets
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        cropped = image[y0:y0 + self.crop_size,
                        x0:x0 + self.crop_size].copy()
        output = self.bridge.cv2_to_imgmsg(cropped, encoding=msg.encoding)
        output.header = msg.header
        output.is_bigendian = msg.is_bigendian
        publisher.publish(output)

        if publish_jpeg:
            if msg.encoding.lower() == 'rgb8':
                jpeg_image = cv2.cvtColor(cropped, cv2.COLOR_RGB2BGR)
                compressed_format = 'rgb8; jpeg compressed bgr8'
            elif msg.encoding.lower() == 'bgr8':
                jpeg_image = cropped
                compressed_format = 'bgr8; jpeg compressed bgr8'
            else:
                self.get_logger().error(
                    f'JPEG output requires rgb8 or bgr8, got {msg.encoding}')
                return
            success, encoded = cv2.imencode(
                '.jpg', jpeg_image,
                [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality])
            if not success:
                self.get_logger().error('OpenCV failed to encode RGB JPEG')
                return
            compressed = CompressedImage()
            compressed.header = msg.header
            compressed.format = compressed_format
            compressed.data = encoded.tobytes()
            self.color_compressed_pub.publish(compressed)

        if publish_depth_png:
            if msg.encoding.lower() not in ('16uc1', 'mono16'):
                self.get_logger().error(
                    f'Compressed depth requires 16UC1, got {msg.encoding}')
                return
            success, encoded = cv2.imencode(
                '.png', cropped,
                [cv2.IMWRITE_PNG_COMPRESSION, self.depth_png_level])
            if not success:
                self.get_logger().error('OpenCV failed to encode depth PNG')
                return
            compressed = CompressedImage()
            compressed.header = msg.header
            compressed.format = f'{msg.encoding}; compressedDepth'
            # ROS compressed_depth_image_transport ConfigHeader:
            # int format=INV_DEPTH(0), followed by two unused floats for 16UC1.
            compressed.data = (
                struct.pack('<iff', 0, 0.0, 0.0) + encoded.tobytes())
            self.depth_compressed_pub.publish(compressed)

    def _crop_info(self, msg, publisher):
        offsets = self._offsets(msg.width, msg.height)
        if offsets is None:
            return
        x0, y0 = offsets
        output = copy.deepcopy(msg)
        output.width = self.crop_size
        output.height = self.crop_size

        # Shift the optical center into the cropped image coordinate system.
        output.k[2] -= x0
        output.k[5] -= y0
        output.p[2] -= x0
        output.p[6] -= y0

        # The output itself is a complete image, not an ROI into another image.
        output.roi.x_offset = 0
        output.roi.y_offset = 0
        output.roi.width = 0
        output.roi.height = 0
        output.roi.do_rectify = False
        publisher.publish(output)


def main(args=None):
    rclpy.init(args=args)
    node = CenterCrop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
