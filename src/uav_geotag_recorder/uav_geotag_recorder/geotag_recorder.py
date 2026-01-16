import os
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.time import Time
from sensor_msgs.msg import Image
from px4_msgs.msg import VehicleGlobalPosition
from cv_bridge import CvBridge
import cv2
import piexif

def deg_to_dms_rational(deg):
    deg = float(deg)
    d = int(abs(deg))
    m_float = (abs(deg) - d) * 60.0
    m = int(m_float)
    s = (m_float - m) * 60.0
    return ((d, 1), (m, 1), (int(s * 100), 100))

'''
Subscribes the camera image and GPS position from PX4,
every Nth frame saves as JPEG and writes GPS coordinates to its EXIF
​​and metadata (lat, lon, alt, timestamp) to the CSV file.
'''

class GeotagRecorder(Node):
    def __init__(self):
        super().__init__('geotag_recorder')

        # params
        self.out_dir = self.declare_parameter('output_dir', '/tmp/uav_geotag').get_parameter_value().string_value
        self.image_topic = self.declare_parameter('image_topic', '/camera/image_raw').get_parameter_value().string_value
        self.gpos_topic  = self.declare_parameter('gpos_topic',  '/fmu/out/vehicle_global_position').get_parameter_value().string_value
        self.save_every_n = self.declare_parameter('save_every_n', 5).get_parameter_value().integer_value

        os.makedirs(self.out_dir, exist_ok=True)

        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        qos.history = HistoryPolicy.KEEP_LAST

        self.bridge = CvBridge()
        self.frame_idx = 0
        self.saved = 0

        # last position
        self.last_gpos = None
        self.last_gpos_time = None

        self.image_sub = self.create_subscription(Image, self.image_topic, self.image_cb, qos)
        self.gpos_sub  = self.create_subscription(VehicleGlobalPosition, self.gpos_topic, self.gpos_cb, qos)

        self.get_logger().info(f'GeotagRecorder: zapis do {self.out_dir}')
        self.get_logger().info(f'Źródła: {self.image_topic} + {self.gpos_topic}')
        self.get_logger().info(f'save_every_n = {self.save_every_n}')

    def gpos_cb(self, msg: VehicleGlobalPosition):

        lat = float(msg.lat)
        lon = float(msg.lon)
        alt = float(msg.alt)

        self.last_gpos = (lat, lon, alt)

        self.last_gpos_time = self.get_clock().now()
        self.get_logger().debug(f'GPOS aktualizacja: lat={lat:.8f}, lon={lon:.8f}, alt={alt:.2f}')

    def image_cb(self, img_msg: Image):
        self.frame_idx += 1

        if self.frame_idx % self.save_every_n != 0:
            return

        if self.last_gpos is None:
            self.get_logger().warn('Brak GPOS – pomijam zapis klatki')
            return

        lat, lon, alt = self.last_gpos
        self.get_logger().info(f'SYNC: zapisuję klatkę {self.saved} z GPS lat={lat:.8f}, lon={lon:.8f}, alt={alt:.2f}')

        # image conversion and save
        cv_img = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')
        fname = f'img_{self.saved:05d}.jpg'
        fpath = os.path.join(self.out_dir, fname)
        cv2.imwrite(fpath, cv_img)

        # EXIF GPS
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

        # CSV
        shots_path = os.path.join(self.out_dir, 'shots.csv')
        stamp_ns = img_msg.header.stamp.sec * 1e9 + img_msg.header.stamp.nanosec
        with open(shots_path, 'a') as f:
            f.write(f'{fname},{lat:.8f},{lon:.8f},{alt:.3f},{int(stamp_ns)}\n')

        self.saved += 1
        self.get_logger().info(f'Zapisano {fname} -> {fpath}')

def main(args=None):
    rclpy.init(args=args)
    node = GeotagRecorder()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
