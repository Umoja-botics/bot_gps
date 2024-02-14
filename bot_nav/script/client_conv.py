#! /usr/bin/env python3


import sys
import rclpy
from rclpy.node import Node
from bot_desc.srv import Gpstomap

class GpsToMapClient(Node):

    def __init__(self):
        super().__init__('gps_to_map_client')
        self.client = self.create_client(Gpstomap, 'gps_to_map')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.req = Gpstomap.Request()

    def send_request(self, latitude, longitude):
        self.req.latitude = latitude
        self.req.longitude = longitude
        self.future = self.client.call_async(self.req)

def main(args=None):
    rclpy.init(args=args)
    gps_to_map_client = GpsToMapClient()
    latitude = 48.75881220971535  
    longitude = 3.9951920592305044 
    gps_to_map_client.send_request(latitude, longitude)

    rclpy.spin_until_future_complete(gps_to_map_client, gps_to_map_client.future)
    if gps_to_map_client.future.result() is not None:
        response = gps_to_map_client.future.result()
        print(f'Result of GPS ({latitude}, {longitude}): x = {response.x}, y = {response.y}')
    else:
        gps_to_map_client.get_logger().error('Exception while calling service: %r' % gps_to_map_client.future.exception())

    gps_to_map_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
