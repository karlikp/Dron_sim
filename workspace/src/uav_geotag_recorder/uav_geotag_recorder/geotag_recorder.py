import os

import cv2
import piexif
import rclpy
from cv_bridge import CvBridge
from px4_msgs.msg import VehicleGlobalPosition
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image


def deg_to_dms_rational(deg):
    deg = float(deg)
    d = int(abs(deg))
    m_float = (abs(deg) - d) * 60.0
    m = int(m_float)
    s = (m_float - m) * 60.0
    return ((d, 1), (m, 1), (int(s * 100), 100))


class GeotagRecorder(Node):
    """Save the newest camera image at an equal time interval with GPS EXIF."""

    def __init__(self):
        super().__init__('geotag_recorder')

        self.out_dir = self.declare_parameter('output_dir', '/tmp/uav_geotag').value
        self.image_topic = self.declare_parameter('image_topic', '/camera/image_raw').value
        self.gpos_topic = self.declare_parameter(
            'gpos_topic',
            '/fmu/out/vehicle_global_position',
        ).value
        self.photo_interval_s = float(
            self.declare_parameter('photo_interval_s', 5.0).value
        )
        self.save_every_n = int(self.declare_parameter('save_every_n', 0).value)

        if self.photo_interval_s <= 0.0:
            self.get_logger().warn(
                'photo_interval_s must be > 0. Falling back to 5.0 seconds.'
            )
            self.photo_interval_s = 5.0

        os.makedirs(self.out_dir, exist_ok=True)
        self._ensure_csv_header()

        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        qos.history = HistoryPolicy.KEEP_LAST

        self.bridge = CvBridge()
        self.frame_idx = 0
        self.saved = 0
        self.last_gpos = None
        self.last_image = None

        self.image_sub = self.create_subscription(
            Image,
            self.image_topic,
            self.image_cb,
            qos,
        )
        self.gpos_sub = self.create_subscription(
            VehicleGlobalPosition,
            self.gpos_topic,
            self.gpos_cb,
            qos,
        )

        if self.save_every_n > 0:
            self.get_logger().warn(
                'save_every_n is deprecated; photo_interval_s is used for equal-time capture.'
            )

        self.photo_timer = self.create_timer(self.photo_interval_s, self.photo_timer_cb)

        self.get_logger().info(f'GeotagRecorder saving to {self.out_dir}')
        self.get_logger().info(f'Sources: {self.image_topic} + {self.gpos_topic}')
        self.get_logger().info(f'Photo interval: {self.photo_interval_s:.3f} s')

    def _ensure_csv_header(self):
        shots_path = os.path.join(self.out_dir, 'shots.csv')
        if not os.path.exists(shots_path) or os.path.getsize(shots_path) == 0:
            with open(shots_path, 'w', encoding='utf-8') as f:
                f.write('filename,lat,lon,alt,timestamp_ns\n')

    def gpos_cb(self, msg):
        self.last_gpos = (float(msg.lat), float(msg.lon), float(msg.alt))

    def image_cb(self, img_msg):
        self.frame_idx += 1
        self.last_image = img_msg

    def photo_timer_cb(self):
        if self.last_image is None:
            self.get_logger().warn('No image received yet; skipping timed photo.')
            return

        if self.last_gpos is None:
            self.get_logger().warn('No global position received yet; skipping timed photo.')
            return

        self.save_photo(self.last_image, self.last_gpos)

    def save_photo(self, img_msg, gpos):
        lat, lon, alt = gpos
        cv_img = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')
        fname = f'img_{self.saved:05d}.jpg'
        fpath = os.path.join(self.out_dir, fname)

        if not cv2.imwrite(fpath, cv_img):
            self.get_logger().error(f'Could not write image: {fpath}')
            return

        gps_ifd = {
            piexif.GPSIFD.GPSLatitudeRef: b'N' if lat >= 0 else b'S',
            piexif.GPSIFD.GPSLatitude: deg_to_dms_rational(lat),
            piexif.GPSIFD.GPSLongitudeRef: b'E' if lon >= 0 else b'W',
            piexif.GPSIFD.GPSLongitude: deg_to_dms_rational(lon),
            piexif.GPSIFD.GPSAltitudeRef: 0,
            piexif.GPSIFD.GPSAltitude: (int(abs(alt) * 100), 100),
        }
        piexif.insert(piexif.dump({'GPS': gps_ifd}), fpath)

        stamp_ns = img_msg.header.stamp.sec * 1_000_000_000 + img_msg.header.stamp.nanosec
        shots_path = os.path.join(self.out_dir, 'shots.csv')
        with open(shots_path, 'a', encoding='utf-8') as f:
            f.write(f'{fname},{lat:.8f},{lon:.8f},{alt:.3f},{stamp_ns}\n')

        self.saved += 1
        self.get_logger().info(
            f'Saved {fname} at lat={lat:.8f}, lon={lon:.8f}, alt={alt:.2f}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = GeotagRecorder()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
