#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
import tf
import cv2
import yaml
import math

STATE_COUNT_THRESHOLD = 3
IMAGE_COUNTER = 0

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.lights = []
        self.stop_line_indices = []
        self.light_indices = []

        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb)

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

        rospy.spin()

    def pose_cb(self, msg):
        self.pose = msg
        #rospy.loginfo('received current pos (%s)',msg)

    def waypoints_cb(self, msg):
        self.waypoints = msg.waypoints[:]

        stop_line_positions = self.config['stop_line_positions']
        rospy.loginfo('stop_line_positions:(%s)',stop_line_positions)
        for stop_line_pos in stop_line_positions:
            index = self.get_closest_waypoint2(stop_line_pos[0],stop_line_pos[1])
            self.stop_line_indices.append(index)

        #for i in range(len(self.waypoints.waypoints)):
            rospy.loginfo('stop_line_index:%d',index)

    def traffic_cb(self, msg):
        rospy.loginfo('OK, got traffic lights')
        self.lights = msg.lights
        if len(self.light_indices) == 0:
            for l in self.lights:
                index = self.get_closest_waypoint(l.pose.pose)
                self.light_indices.append(index)
                rospy.loginfo('stop_line_index:%d',index)


    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
        self.has_image = True
        self.camera_image = msg
        light_wp, state = self.process_traffic_lights()
        #rospy.loginfo('Received image')
        
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
            rospy.loginfo('Publishing stopping waypoint:%d',light_wp)
            self.upcoming_red_light_pub.publish(Int32(light_wp))
        else:
            rospy.loginfo('Publishing stopping waypoint:%d',self.last_wp)
            self.upcoming_red_light_pub.publish(Int32(self.last_wp))
        self.state_count += 1

    def get_distance(self, x1, y1, x2, y2):
        return math.sqrt((x1-x2)**2 + (y1-y2)**2)

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist

    def get_closest_waypoint(self, pose):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
        #TODO implement
        min_dist = 9999999
        min_index = 0
        if self.waypoints:
            for i in range(len(self.waypoints)):
                w = self.waypoints[i]
                dist = self.get_distance(w.pose.pose.position.x,w.pose.pose.position.y,pose.position.x, pose.position.y)
                if dist < min_dist:
                    min_dist = dist
                    min_index = i
        else:
            rospy.loginfo('waypoints is null')


        return min_index
    def get_closest_waypoint2(self, in_x, in_y):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
        #TODO implement
        min_dist = 9999999
        min_index = 0
        if self.waypoints:
            for i in range(len(self.waypoints)):
                w = self.waypoints[i]
                dist = self.get_distance(w.pose.pose.position.x,w.pose.pose.position.y,in_x, in_y)
                if dist < min_dist:
                    min_dist = dist
                    min_index = i
        else:
            rospy.loginfo('waypoints is null')


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

        global IMAGE_COUNTER
        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")

        state_str = self.convert_state_string(light.state)

        if light.state == TrafficLight.RED:
            cv2.imwrite("/home/student/Project/CarND-Capstone/sim_img/Red/"+str(IMAGE_COUNTER)+"_" + state_str + ".png",cv_image)
        elif light.state == TrafficLight.YELLOW:
            cv2.imwrite("/home/student/Project/CarND-Capstone/sim_img/Yellow/"+str(IMAGE_COUNTER)+"_" + state_str+ ".png",cv_image)
        elif light.state == TrafficLight.GREEN:
            cv2.imwrite("/home/student/Project/CarND-Capstone/sim_img/Green/"+str(IMAGE_COUNTER)+"_" + state_str +".png",cv_image)
        else:
            cv2.imwrite("/home/student/Project/CarND-Capstone/sim_img/Unknown/"+str(IMAGE_COUNTER)+"_" + state_str +".png",cv_image)

        IMAGE_COUNTER += 1

        #Get classification
        ret_state =  self.light_classifier.get_classification(cv_image)

        if ret_state != light.state:
            cv2.imwrite("/home/student/Project/CarND-Capstone/misclassified_img/"+str(IMAGE_COUNTER)+ \
                "_" + state_str + ".png",cv_image)
            IMAGE_COUNTER += 1


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
            rospy.loginfo('Car is near waypoint %d',car_position_wp)
            next_light = None
            next_light_wp = -1
            l = len(self.stop_line_indices)
            if l > 0:
                if car_position_wp > self.stop_line_indices[-1]:
                    next_light_wp = self.stop_line_indices[0]
                    next_light = self.lights[0]
                else:
                    for i in range(l):
                        if self.stop_line_indices[i] > car_position_wp:
                            next_light_wp = self.stop_line_indices[i]
                            next_light = self.lights[i]
                            break
            if next_light_wp > -1:
                d = self.distance(self.waypoints, car_position_wp, next_light_wp)

                if d <= 50:
                    rospy.loginfo('Light nearby(%s),(%s)',next_light_wp,d)
                    light = next_light
                    light_wp = next_light_wp




        #TODO find the closest visible traffic light (if one exists)
        if light:
            
            state = self.get_light_state(light)
        else:
            rospy.loginfo('No Light nearby')
            state = TrafficLight.UNKNOWN

        return light_wp, state

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
