#! /usr/bin/env python3

from nav2_simple_commander.robot_navigator import BasicNavigator,TaskResult
import rclpy
import sys
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
from bot_nav.srv import Gpstomap



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
    nav = BasicNavigator()

    latitude = 48.75881220971535   #target gps point
    longitude = 3.9951920592305044 

    gps_to_map_client.send_request(latitude, longitude)

    rclpy.spin_until_future_complete(gps_to_map_client, gps_to_map_client.future)
    if gps_to_map_client.future.result() is not None:
        response = gps_to_map_client.future.result()
        print(f'Result of GPS ({latitude}, {longitude}): x = {response.x}, y = {response.y}')
    else:
        gps_to_map_client.get_logger().error('Exception while calling service: %r' % gps_to_map_client.future.exception())

    

    # ...
        
    # Set our demo's initial pose
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = nav.get_clock().now().to_msg()
    initial_pose.pose.position.x = 0.0
    initial_pose.pose.position.y = 0.0
    initial_pose.pose.orientation.z = 0.0
    initial_pose.pose.orientation.w = 1.0
    

    nav.setInitialPose(initial_pose)
    nav.waitUntilNav2Active() # if autostarted, else use lifecycleStartup()

    # ...
    # Go to our demos first goal pose
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = nav.get_clock().now().to_msg()
    goal_pose.pose.position.x = response.x
    goal_pose.pose.position.y = response.y
    goal_pose.pose.orientation.w = 1.0

    #path = nav.getPath(initial_pose, goal_pose)
    #smoothed_path = nav.smoothPath(path)

    # ...

    nav.goToPose(goal_pose)
    # i = 0
    # while not nav.isTaskComplete():
    #     ################################################
    #     #
    #     # Implement some code here for your application!
    #     #
    #     ################################################

    #     # Do something with the feedback
    #     i = i + 1
    #     feedback = nav.getFeedback()
    #     if feedback and i % 5 == 0:
    #         print('Estimated time of arrival: ' + '{0:.0f}'.format(
    #               Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
    #               + ' seconds.')

    #         # Some navigation timeout to demo cancellation
    #         if Duration.from_msg(feedback.navigation_time) < Duration(seconds=600.0):
    #             nav.cancelTask()

    #         # Some navigation request change to demo preemption
    #         if Duration.from_msg(feedback.navigation_time) > Duration(seconds=18.0):
    #             goal_pose.pose.position.x = -3.0
    #             nav.goToPose(goal_pose)

    # ...

    result = nav.getResult()
    if result == TaskResult.SUCCEEDED:
        print('Goal succeeded!')
    elif result == TaskResult.CANCELED:
        print('Goal was canceled!')
    elif result == TaskResult.FAILED:
        print('Goal failed!')

    nav.lifecycleShutdown()

    gps_to_map_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()