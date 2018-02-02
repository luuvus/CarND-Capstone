#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32

import math
import tf
import copy

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
TRAFFIC_LIGHT_STOP_WPS = 15 # Number of waypoints from traffic light to stop point


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb, queue_size=1)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb, queue_size=1)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        rospy.Subscriber('/traffic_waypoint',Int32,self.traffic_cb, queue_size=1)
        rospy.Subscriber('/current_velocity', TwistStamped, self.velocity_cb, queue_size=1)
       

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below

        self.speed_limit = rospy.get_param('/waypoint_loader/velocity', 25)

        self.base_waypoints = None
        self.base_waypoints_count = 0
        self.current_pose = None
        self.current_velocity = None

        self.tf_index = -1 # index of traffic light waypoint

        self.car_x = None
        self.car_y = None

        #rospy.spin()
        self.loop()

    def loop(self):

        #set refresh rate 50Hz for Carla/real vehicle, 
        #set a low refresh rate to reduce lagging when running in simulator
        rate = rospy.Rate(5) 

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

        end_wp_idx = next_wp_idx + LOOKAHEAD_WPS

        if end_wp_idx > self.base_waypoints_count - 1:
            end_wp_idx = self.base_waypoints_count - 1

        decel_rate = 0 
        dist_to_tf = -1 # distance between waypoint in front of car and the stop light waypoint
        tf_final_stop_index = -1 # index of the waypoint that the car should come to a complete stop
        min_dist_gap_to_decel = 45 # min points/spaces gap to decelerate
        
        # adjust parameters for stop light situation
        if self.tf_index >= 0:   

            #dist_to_tf = self.distance(self.base_waypoints,next_wp_idx,self.tf_index)
            dist_to_tf = self.dist_to_tl(self.tf_index, next_wp_idx)
            
            if dist_to_tf > 0:
                decel_rate = (self.mph_to_mps(self.current_velocity) ** 2)/(dist_to_tf * 2)

            tf_final_stop_index = self.tf_index - TRAFFIC_LIGHT_STOP_WPS

        # process waypoints between waypoint in front of car and the final ending waypoint in the trajectory
        for wp_idx in range(next_wp_idx, end_wp_idx):
            
            # set to velocity to default speed limit
            target_velocity = self.mph_to_mps(self.speed_limit)*.6
            
            # adjust velocity for stop light
            if self.tf_index >= 0:

                if wp_idx >  tf_final_stop_index: # velocity is not needed for waypoints after traffic light waypoint stop point
                    target_velocity = 0

                elif tf_final_stop_index - wp_idx <= min_dist_gap_to_decel: # adjust velocity if the gap between this wp_idx and traffic light is within min points/spaces

                    if wp_idx == tf_final_stop_index: # set vel to zero if this wp_idx is the same as final stop point
                        target_velocity = 0
                    else:
                        #calc distance between this wp_idx and the next waypoint index
                        wp_to_wp_dist = self.distance(self.base_waypoints, wp_idx, wp_idx + 1)

                        #calc velocity between this wp_idx and the next waypoint index
                        wp_to_wp_vel = (self.mph_to_mps(self.current_velocity) ** 2) - (wp_to_wp_dist * 2 * decel_rate)

                        # reduce to zero for low threshold
                        if wp_to_wp_vel < 0.1:
                            wp_to_wp_vel = 0.0
                                         
                        target_velocity = math.sqrt(wp_to_wp_vel)
                        target_velocity = self.mph_to_mps(target_velocity)

            # apply velocity to this current processing waypoint
            self.set_waypoint_velocity(self.base_waypoints,wp_idx,target_velocity)
            
            # add this current processing waypoint to final set of waypoints
            msg.waypoints.append(self.base_waypoints[wp_idx])

        #publish the message
        self.final_waypoints_pub.publish(msg)

    def dist_to_tl(self, tl_index, next_index ):
        if tl_index == -1 or next_index == -1:
            return -1
        diff = tl_index - next_index
        if diff < 0:
            diff += self.base_waypoints_count
        return diff

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
        #if self.tf_index != msg.data:
        self.tf_index = msg.data
            #rospy.logwarn("traffic_cb | tf_index = %s", self.tf_index)

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

    def mph_to_mps(self, miles_per_hour):
        meter_per_second = 0.44704
        return miles_per_hour * meter_per_second

if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
