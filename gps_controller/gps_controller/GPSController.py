import serial
# TODO: Find a better u-blox library. This one is really slow + out of date
from ublox_gps import UbloxGps

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point


class GPSController(Node):
    '''Provides a ROS2 interface to a Sparkfun u-blox GPS module

    GPSController publishes latitude and longitude read from the GPS module.
    '''
 
    def __init__(
            self,
            serial_port: str = '/dev/serial0',
            baudrate: int = 38400,
            #use_cached_values: bool = False,
            #cache_update_interval: float = 1.0):
            ):
        super().__init__('gps_controller', parameter_overrides=[])
        self._timer = self.create_timer(1.0, self._pub_callback)
        self._coord_pub = self.create_publisher(Point, 'coordinates', 1)
        self._gps = UbloxGps(serial.Serial('/dev/serial0', baudrate=baudrate, timeout=1))
        self.log = self.get_logger()
        self.log.info('Initialized')

    def _pub_callback(self):
        geo = self._gps.geo_coords()
        self.log.info(f'{geo.lat=:.7} \t{geo.lon=:.7}')
        self._coord_pub.publish(Point(x=geo.lat, y=geo.lon))


def main(args=None):
    rclpy.init(args=args)

    node = GPSController()

    node.get_logger().info('Running')
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
