import os
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from px4_msgs.msg import VehicleGlobalPosition
from cv_bridge import CvBridge
import cv2
import piexif
from message_filters import ApproximateTimeSynchronizer, Subscriber

def deg_to_dms_rational(deg):
    deg = float(deg)
    d = int(abs(deg))
    m_float = (abs(deg) - d) * 60.0
    m = int(m_float)
    s = (m_float - m) * 60.0
    return ((d, 1), (m, 1), (int(s * 100), 100))

class GeotagRecorder(Node):
    def __init__(self):
        super().__init__('geotag_recorder')

        self.out_dir = self.declare_parameter('output_dir', '/tmp/uav_geotag').get_parameter_value().string_value
        self.image_topic = self.declare_parameter('image_topic', '/camera/image_raw').get_parameter_value().string_value
        self.gpos_topic  = self.declare_parameter('gpos_topic',  '/fmu/out/vehicle_global_position').get_parameter_value().string_value
        self.save_every_n = self.declare_parameter('save_every_n', 5).get_parameter_value().integer_value  # co N klatek

        os.makedirs(self.out_dir, exist_ok=True)

        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        qos.history = HistoryPolicy.KEEP_LAST

        self.bridge = CvBridge()
        self.frame_idx = 0
        self.saved = 0

        self.img_sub  = Subscriber(self, Image, self.image_topic, qos_profile=qos)
        self.gpos_sub = Subscriber(self, VehicleGlobalPosition, self.gpos_topic, qos_profile=qos)

        self.sync = ApproximateTimeSynchronizer(
            [self.img_sub, self.gpos_sub],
            queue_size=100,
            slop=1.0,
            allow_headerless=True,  # <<< DODANE: pozwala na wiadomości bez nagłówka
        )
        self.sync.registerCallback(self.cb_sync)

        self.get_logger().info(f'GeotagRecorder: zapis do {self.out_dir}')
        self.get_logger().info(f'Źródła: {self.image_topic} + {self.gpos_topic}')

    def cb_sync(self, img_msg: Image, gpos: VehicleGlobalPosition):
        self.frame_idx += 1
        if self.frame_idx % self.save_every_n != 0:
            return

        # Upewnij się jak publikowane są lat/lon w Twojej wersji px4_msgs:
        # najczęściej: stopnie (deg). Jeśli są w 1e-7 deg, dodaj mnożnik 1e-7.
        lat = float(gpos.lat)
        lon = float(gpos.lon)
        alt = float(gpos.alt)  # m AMSL

        cv_img = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')
        fname = f'img_{self.saved:05d}.jpg'
        fpath = os.path.join(self.out_dir, fname)
        cv2.imwrite(fpath, cv_img)

        gps_ifd = {
            piexif.GPSIFD.GPSLatitudeRef: b'N' if lat >= 0 else b'S',
            piexif.GPSIFD.GPSLatitude: deg_to_dms_rational(lat),
            piexif.GPSIFD.GPSLongitudeRef: b'E' if lon >= 0 else b'W',
            piexif.GPSIFD.GPSLongitude: deg_to_dms_rational(lon),
            piexif.GPSIFD.GPSAltitudeRef: 0,
            piexif.GPSIFD.GPSAltitude: (int(abs(alt) * 100), 100),
        }
        exif_dict = {"GPS": gps_ifd}
        piexif.insert(piexif.dump(exif_dict), fpath)

        with open(os.path.join(self.out_dir, 'shots.csv'), 'a') as f:
            stamp_ns = img_msg.header.stamp.sec * 1e9 + img_msg.header.stamp.nanosec
            f.write(f'{fname},{lat:.8f},{lon:.8f},{alt:.3f},{int(stamp_ns)}\n')

        self.saved += 1
        self.get_logger().info(f'Zapisano {fname} @ {lat:.6f},{lon:.6f},{alt:.1f} m')

def main(args=None):
    rclpy.init(args=args)
    node = GeotagRecorder()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
