import os
import time
import rclpy

import serial
from ublox_gps import UbloxGps

from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import Executor, MultiThreadedExecutor

from gps_interfaces.srv import Coordinates


class GPSBenchmarker(Node):

    def __init__(self, executor: Executor):
        super().__init__('gps_benchmarker', parameter_overrides=[])

        self.results = []
        self._call_counter = 0

        num_calls = input('Number of calls to make: ')
        self._num_calls = int(num_calls) if num_calls.isnumeric() else 100
        call_interval = input('Call intervals: ')
        self._call_interval = float(call_interval) if call_interval.isnumeric() else 0.5

        self._executor: Executor = executor;
        self._timer_group = MutuallyExclusiveCallbackGroup()
        self._gps_group = MutuallyExclusiveCallbackGroup()
        #self._timer_group = ReentrantCallbackGroup()
        #self._gps_group = ReentrantCallbackGroup()
        #self._gps_group = self._timer_group

        self._timer = self.create_timer(self._call_interval, self._timer_callback,
                callback_group=self._timer_group)
        self._coordinates_client = self.create_client(Coordinates, 'coordinates',
                callback_group=self._gps_group)
        self._gps = UbloxGps(serial.Serial('/dev/serial0', baudrate=38400, timeout=1))

        self.log = self.get_logger()
        self.log.info(f'Initialized. Testing {self._num_calls} calls at {self._call_interval}s intervals.')
        

    def _timer_callback(self):
        self._call_counter += 1
        start_time = time.time()

        # Call the /coordinates service
        #coords = self._coordinates_client.call(Coordinates.Request())
        #coords_future = self._coordinates_client.call_async(Coordinates.Request())
        #self._executor.spin_until_future_complete(coords_future)

        # Get the coordinates directly
        geo = self._gps.geo_coords()

        end_time = time.time()
        self.results.append((self._call_counter, start_time, end_time))

        #if self._call_counter % 10 == 0:
        self.log.info(f'Completed call {self._call_counter} of {self._num_calls}')

        if self._call_counter >= self._num_calls:
            self.destroy_timer(self._timer)

            output_path = 'data.csv'
            outfile = open(output_path, mode='w')

            self.log.info(f'Completed all calls. Writing results to {os.path.abspath(output_path)}')
            for call, start, end in self.results:
                print(f'{call},{start},{end}', file=outfile)
            outfile.close()

            self.log.info('Done. Destroying node')
            self.destroy_node()


def main(args=None):
    rclpy.init(args=args)

    executor = MultiThreadedExecutor()
    node = GPSBenchmarker(executor)
    executor.add_node(node)

    node.get_logger().info('Running')
    executor.spin()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
