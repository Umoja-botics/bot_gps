#! /usr/bin/env python3


import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
import tf2_ros
from geometry_msgs.msg import TransformStamped
import utm

class GPSToMapNode(Node):
    def __init__(self):
        super().__init__('gps_to_map_node')
        self.subscription = self.create_subscription(
            NavSatFix, '/gps/fix', self.gps_callback, 10)
        self.subscription  # Prevent unused variable warning
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.first_gps_received = False
        self.origin_x = 0
        self.origin_y = 0

    def gps_callback(self, msg):
        # Convertir les coordonnées GPS en coordonnées planaires (x, y)
        
        u = utm.from_latlon(msg.latitude, msg.longitude)

        if not self.first_gps_received:
            self.origin_x, self.origin_y = u[0], u[1]
            self.first_gps_received = True

        # Créer et envoyer la transformation
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_footprint'
        t.child_frame_id = 'maps'
        t.transform.translation.x = u[0] - self.origin_x
        t.transform.translation.y = u[1] - self.origin_y
        t.transform.translation.z = 0.0
        t.transform.rotation.w = 1.0  # Assumant aucune rotation pour cet exemple

        # Publier la transformation
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = GPSToMapNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
