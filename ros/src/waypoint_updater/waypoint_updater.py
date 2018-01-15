#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32

import math
import tf

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        rospy.Subscriber('/traffic_waypoint',Int32,self.traffic_cb)
        rospy.Subscriber('/current_velocity', TwistStamped, self.velocity_cb)
       

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below

        self.speed_limit = rospy.get_param('/waypoint_loader/velocity', 40)

        self.base_waypoints = None
        self.base_waypoints_count = 0
        self.current_pose = None
        self.current_velocity = None

        self.tf_index = -1 # index of closet traffic light waypoint

        self.car_x = None
        self.car_y = None

        #rospy.spin()
        self.loop()

    def loop(self):
        rate = rospy.Rate(10) #Hz

        while not rospy.is_shutdown():
            if self.base_waypoints != None and self.current_pose !=None:
                self.publish_final_waypoints()

            rate.sleep()

    def publish_final_waypoints(self):
        msg = Lane()
        msg.header.frame_id = '/world'
        msg.header.stamp = rospy.Time.now()
        msg.waypoints = []

        next_wp_idx = self.next_waypoint_index()

        max_index = next_wp_idx + LOOKAHEAD_WPS

        if max_index > self.base_waypoints_count - 1:
            max_index = self.base_waypoints_count - 1

        for i in range(next_wp_idx, max_index):

            target_velocity = self.speed_limit

            self.set_waypoint_velocity(self.base_waypoints,i,target_velocity)

            msg.waypoints.append(self.base_waypoints[i])


        self.final_waypoints_pub.publish(msg)
        #pass

    def next_waypoint_index(self):
        next_wp_index = 0
        
        min_distance = 100000

        car_x = self.current_pose.position.x
        car_y = self.current_pose.position.y
        car_yaw = self.get_yaw_from_pose(self.current_pose)

        # find index by compare lowest distance between two points
        for i in range(self.base_waypoints_count):

            this_wp_x = self.base_waypoints[i].pose.pose.position.x
            this_wp_y = self.base_waypoints[i].pose.pose.position.y

            # distance between car and this waypoint
            dist = math.hypot(car_x - this_wp_x , car_y - this_wp_y)

            if dist < min_distance:
                min_distance = dist
                next_wp_index = i

        # adjust the next wp index base on whether the car is behind or in front of the closest waypoint
        closest_wp = self.base_waypoints[next_wp_index]
        closest_wp_x = closest_wp.pose.pose.position.x
        closest_wp_y = closest_wp.pose.pose.position.y

        heading = math.atan2(closest_wp_y - car_y, closest_wp_x - car_x)

        if heading < 0:
            heading = heading + math.pi * 2

        heading_diff = abs(car_yaw - heading)

        if heading_diff > math.pi/4:
            next_wp_index += 1
            if next_wp_index > self.base_waypoints_count - 1:
                next_wp_index -= 1


        return next_wp_index % self.base_waypoints_count

    def get_yaw_from_pose(self, pose):
        quaternion = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
        euler =  tf.transformations.euler_from_quaternion(quaternion)
        return euler[2]
    
    

    def pose_cb(self, msg):
        # TODO: Implement
        self.current_pose = msg.pose
        #pass

    def waypoints_cb(self, msg):
        # TODO: Implement
        self.base_waypoints = msg.waypoints
        self.base_waypoints_count = len(msg.waypoints)
        #pass

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        self.tf_index = msg.data
        #pass

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass
    
    def velocity_cb(self, msg):

        linear_x = msg.twist.linear.x
        linear_y = msg.twist.linear.y
        self.current_velocity = math.sqrt(linear_x **2 + linear_y**2)

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
