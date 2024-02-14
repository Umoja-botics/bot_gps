#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from bot_nav.srv import Gpstomap
import utm


class GpsToMapService(Node):

    def __init__(self):
        super().__init__('gps_to_map_service')
        self.srv = self.create_service(Gpstomap, 'gps_to_map', self.gps_to_map_callback)
        self.get_logger().info('Service is ready...')
        
    def gps_to_map_callback(self, request, response):

        self.get_logger().info('response sent...')

        # Initialisation
        # initial gps coordonate of the robot
        lat_ori = 48.75880004180039
        lon_ori = 3.9950162723024123     # definir un fichier de config pour recuperer les coordonnées initiales du robot
        alt_ori = 255.91022974159569

        U_ori = utm.from_latlon(lat_ori, lon_ori)


        #Conversion en UTM des requetes 
        U = utm.from_latlon(request.latitude, request.longitude)

        # calcule de la coordonnée cartesienne
        response.x  = U[0] - U_ori[0]
        response.y = U[1] - U_ori[1]

        return response

def main(args=None):
    rclpy.init(args=args)
    node = GpsToMapService()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
