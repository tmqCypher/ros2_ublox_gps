import serial
# TODO: Find a better u-blox library. This one is really slow + out of date
from ublox_gps import UbloxGps

import rclpy
from rclpy.node import Node

from gps_interfaces.srv import Coordinates


class GPSController(Node):
    '''Provides a ROS2 interface to a Sparkfun u-blox GPS module

    GPSController provides a service that responds to clients with the
    latitude and longitude read from the GPS module.
    '''
 
    def __init__(
            self,
            serial_port: str = '/dev/serial0',
            baudrate: int = 38400,
            #use_cached_values: bool = False,
            #cache_update_interval: float = 1.0):
            ):
        super().__init__('gps_controller', parameter_overrides=[])

        self._coord_srv = self.create_service(Coordinates, 'coordinates', self._srv_callback)

        '''Cached coordinate retrieval currently unimplemented
        self._coord_srv = self.create_service(Coordinates, 'coordinates',
            self._cached_srv_callback if use_cached_values else self._srv_callback)
            
        if use_cached_values:
            self._timer = self.create_timer(cache_update_interval, self._cache_update_callback)
        '''

        self._gps = UbloxGps(serial.Serial('/dev/serial0', baudrate=baudrate, timeout=1))

        self.log = self.get_logger()
        self.log.info('Initialized')

    '''Cached coordinate retrieval currently unimplemented
    def _cache_update_callback(self):
        geo = self._gps.geo_coords()
        self._cached_lat = geo.lat
        self._cached_lon = geo.lon

    def _cached_srv_callback(self, request, response):
        response.lat = self._cached_lat
        response.lon = self._cached_lon
        return response
    '''

    def _srv_callback(self, request, response):
        geo = self._gps.geo_coords()
        response.lat = geo.lat
        response.lon = geo.lon
        return response


def main(args=None):
    rclpy.init(args=args)

    node = GPSController()

    node.get_logger().info('Running')
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
