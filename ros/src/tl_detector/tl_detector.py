#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
#from light_classification.tl_classifier_sim import TLClassifierSim
import tf
import cv2
import yaml
import math

STATE_COUNT_THRESHOLD = 3
IMAGE_COUNTER = -1

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.lights = []
        self.stop_line_wp_indices = []
        self.light_indices = []


        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb, queue_size = 1)
        sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb, queue_size = 1)

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb, queue_size = 1)
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb, queue_size = 1)

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        self.bridge = CvBridge()
        self.light_classifier = TLClassifier()
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0
        self.update_lights = True

        rospy.spin()

    def pose_cb(self, msg):
        self.pose = msg
        #rospy.loginfo('received current pos (%s)',msg)

    def waypoints_cb(self, msg):
        self.waypoints = msg.waypoints[:]

        stop_line_positions = self.config['stop_line_positions']
        for stop_line_pos in stop_line_positions:
            index = self.get_closest_waypoint2(stop_line_pos[0],stop_line_pos[1])
            self.stop_line_wp_indices.append(index)


    def traffic_cb(self, msg):
        #if self.update_lights:
        #    self.lights = msg.lights
        self.lights = msg.lights

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """

        global IMAGE_COUNTER
        IMAGE_COUNTER += 1
        if IMAGE_COUNTER%4 != 0:
            return
        
        self.has_image = True
        self.camera_image = msg
        self.update_lights = False
        light_wp, state = self.process_traffic_lights()
        self.update_lights = True
        
        
        '''
        Publish upcoming red lights at camera frequency.
        Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        of times till we start using it. Otherwise the previous stable state is
        used.
        '''
        if self.state != state:
            self.state_count = 0
            self.state = state
        elif self.state_count >= STATE_COUNT_THRESHOLD:
            self.last_state = self.state
            light_wp = light_wp if state == TrafficLight.RED else -1
            self.last_wp = light_wp
            #rospy.loginfo('Publishing stopping waypoint:%d',light_wp)
            self.upcoming_red_light_pub.publish(Int32(light_wp))
        else:
            #rospy.loginfo('Publishing stopping waypoint:%d',self.last_wp)
            self.upcoming_red_light_pub.publish(Int32(self.last_wp))
        self.state_count += 1

    def get_distance(self, x1, y1, x2, y2):
        return math.sqrt((x1-x2)**2 + (y1-y2)**2)

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        len_wp = len(waypoints)
        if wp1 > wp2:
            wp2 += len_wp
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            wp_next = i%len_wp
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[wp_next].pose.pose.position)
            wp1 = wp_next
        return dist

    def get_closest_waypoint(self, pose):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
        min_dist = 9999999
        min_index = 0
        if self.waypoints:
            for i in range(len(self.waypoints)):
                w = self.waypoints[i]
                dist = self.get_distance(w.pose.pose.position.x,w.pose.pose.position.y,pose.position.x, pose.position.y)
                if dist < min_dist:
                    min_dist = dist
                    min_index = i


        return min_index
    def get_closest_waypoint2(self, in_x, in_y):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
        min_dist = 9999999
        min_index = 0
        if self.waypoints:
            for i in range(len(self.waypoints)):
                w = self.waypoints[i]
                dist = self.get_distance(w.pose.pose.position.x,w.pose.pose.position.y,in_x, in_y)
                if dist < min_dist:
                    min_dist = dist
                    min_index = i


        return min_index

    def convert_state_string(self, state):
        if state == TrafficLight.RED:
            return "Red"
        elif state == TrafficLight.YELLOW:
            return "Yellow"
        elif state == TrafficLight.GREEN:
            return "Green"
        else:
            return "Unknown"

    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """

        if(not self.has_image):
            self.prev_light_loc = None
            return False

        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "rgb8")

        #Get classification
        #cv2.imwrite("test_img/test_" + str(IMAGE_COUNTER) +".png",cv_image)
        #cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        ret_state =  self.light_classifier.get_classification(cv_image)

        return ret_state

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        light = None
        light_wp = -1

        # List of positions that correspond to the line to stop in front of for a given intersection
        stop_line_positions = self.config['stop_line_positions']

        if(self.pose):
            car_position_wp = self.get_closest_waypoint(self.pose.pose)
            next_light = None
            next_light_wp = -1
            l = len(self.stop_line_wp_indices)
            if l > 0:
                if car_position_wp > self.stop_line_wp_indices[-1]:
                    next_light_wp = self.stop_line_wp_indices[0]
                    next_light = self.lights[0]
                else:
                    for i in range(l):
                        if self.stop_line_wp_indices[i] > car_position_wp:
                            next_light_wp = self.stop_line_wp_indices[i]
                            next_light = self.lights[i]
                            break
            if next_light_wp > -1:
                d = self.distance(self.waypoints, car_position_wp, next_light_wp)

                if d <= 100:
                    light = next_light
                    light_wp = next_light_wp


        #TODO find the closest visible traffic light (if one exists)
        if light:
            state = self.get_light_state(light)
        else:
            state = TrafficLight.UNKNOWN

        return light_wp, state

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
