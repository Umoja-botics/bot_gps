#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PointStamped
import utm
import yaml

class GPSToCartesianConverter(Node):
    def __init__(self):
        super().__init__('gps_to_cartesian_converter')
        yaml_file_path ='/home/klein/gps_waypoint.yaml'
  
        with open(yaml_file_path, 'r') as file:
            config = yaml.safe_load(file)

        self.reference_lat = None
        self.reference_lon = None
        self.reference_alt = None

        self.waypoints = config['waypoints']

        self.local_coordinates_publisher = self.create_publisher(PointStamped, 'local_coordinates', 10)

        self.gps_subscriber = self.create_subscription(
            NavSatFix, '/gps/fix', self.gps_callback, 10
        )

    def gps_callback(self, msg):
        if self.reference_lat is None or self.reference_lon or None or self.reference_lat is None:
            self.reference_lat = msg.latitude
            self.reference_lon = msg.longitude
            self.reference_alt = msg.altitude
            self.destroy_subscription(self.gps_subscriber)

        local_coordinates_list = []
        for gps_coordinate in self.waypoints:
            
            local_coordinate = self.convert_gps_to_local(gps_coordinate)
            #self.local_coordinates_publisher.publish(local_coordinate)
            local_coordinates_list.append(local_coordinate)
            self.get_logger().info(f"Local Coordinate: {local_coordinate}")


        # Enregistrer les coordonnées locales dans un nouveau fichier YAML
        output_filename = 'local_coordinates.yaml'
        with open(output_filename, 'w') as output_file:
            yaml.dump(local_coordinates_list, output_file)

    def convert_gps_to_local(self, gps_coordinate):
        # Convertir les coordonnées GPS en coordonnées UTM
        utm_coordinate = utm.from_latlon(gps_coordinate['latitude'], gps_coordinate['longitude'])

        # Calculer les écarts en x, y par rapport à la position initiale du robot
        delta_x = utm_coordinate[0] - utm.from_latlon(self.reference_lat, self.reference_lon)[0]
        delta_y = utm_coordinate[1] - utm.from_latlon(self.reference_lat, self.reference_lon)[1]

        return {'x': delta_x, 'y': delta_y, 'yaw': gps_coordinate['yaw']}

def main(args=None):
    rclpy.init(args=args)
    gps_converter = GPSToCartesianConverter()
    rclpy.spin(gps_converter)
    gps_converter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


