import time
import rclpy

from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import Executor, MultiThreadedExecutor

from gps_interfaces.srv import Coordinates


class GPSBenchmarker(Node):

    def __init__(
            self,
            num_calls: int,
            call_interval: float,
            executor: Executor
            ):
        super().__init__('gps_benchmarker', parameter_overrides=[])

        self.results = []
        self._num_calls = num_calls
        self._call_counter = 0
        self._running = True

        self._executor: Executor = executor;
        self.gps_group = MutuallyExclusiveCallbackGroup()
        self.timer_group = MutuallyExclusiveCallbackGroup()

        self._timer = self.create_timer(call_interval, self._timer_callback,
                callback_group=self.timer_group)
        self._coordinates_client = self.create_client(Coordinates, 'coordinates',
                callback_group=self.gps_group)
        self.log = self.get_logger()
        self.log.info('Initialized')
        
    def _timer_callback(self):

        start_time = time.time()
        coords_future = self._coordinates_client.call_async(Coordinates.Request())
        self._executor.spin_until_future_complete(coords_future)
        end_time = time.time()

        self._call_counter += 1
        self.results.append((self._call_counter, end_time - start_time))

        if self._call_counter == self._num_calls:
            self.destroy_timer(self._timer)
            self.running = False


def main(args=None):
    rclpy.init(args=args)

    NUM_CALLS_PER_TEST = 100
    CALL_INTERVALS = [0.01, 0.1, 0.5, 1.0, 2.0]

    executor = MultiThreadedExecutor()
    for interval in CALL_INTERVALS:

        print(f'Running test: {NUM_CALLS_PER_TEST} calls, {interval}s intervals')
        node = GPSBenchmarker(NUM_CALLS_PER_TEST, interval, executor)
        while node.running:
            executor.spin_once()
        test_results = node.results
        node.destroy_node()

        with open(f'gps_test_{interval}.csv', mode='w') as outfile:
            for result in test_results:
                print(f'{result[0]},{result[1]}', file=outfile)
                



    rclpy.shutdown()

if __name__ == '__main__':
    main()
