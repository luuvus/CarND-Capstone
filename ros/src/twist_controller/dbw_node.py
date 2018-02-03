#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool, Float64
from dbw_mkz_msgs.msg import ThrottleCmd, SteeringCmd, BrakeCmd, SteeringReport
from geometry_msgs.msg import TwistStamped
import math
from twist_controller import Controller

'''
You can build this node only after you have built (or partially built) the `waypoint_updater` node.

You will subscribe to `/twist_cmd` message which provides the proposed linear and angular velocities.
You can subscribe to any other message that you find important or refer to the document for list
of messages subscribed to by the reference implementation of this node.

One thing to keep in mind while building this node and the `twist_controller` class is the status
of `dbw_enabled`. While in the simulator, its enabled all the time, in the real car, that will
not be the case. This may cause your PID controller to accumulate error because the car could
temporarily be driven by a human instead of your controller.

We have provided two launch files with this node. Vehicle specific values (like vehicle_mass,
wheel_base) etc should not be altered in these files.

We have also provided some reference implementations for PID controller and other utility classes.
You are free to use them or build your own.

Once you have the proposed throttle, brake, and steer values, publish it on the various publishers
that we have created in the `__init__` function.

'''

class DBWNode(object):
    def __init__(self):
        rospy.init_node('dbw_node')

        vehicle_mass = rospy.get_param('~vehicle_mass', 1736.35)
        fuel_capacity = rospy.get_param('~fuel_capacity', 13.5)
        brake_deadband = rospy.get_param('~brake_deadband', .1)
        decel_limit = rospy.get_param('~decel_limit', -5)
        accel_limit = rospy.get_param('~accel_limit', 1.)
        wheel_radius = rospy.get_param('~wheel_radius', 0.2413)
        wheel_base = rospy.get_param('~wheel_base', 2.8498)
        steer_ratio = rospy.get_param('~steer_ratio', 14.8)
        max_lat_accel = rospy.get_param('~max_lat_accel', 3.)
        max_steer_angle = rospy.get_param('~max_steer_angle', 8.)
        # steering control publisher
        self.steer_pub = rospy.Publisher('/vehicle/steering_cmd',
                                         SteeringCmd, queue_size=1)
        # throttle control publisher
        self.throttle_pub = rospy.Publisher('/vehicle/throttle_cmd',
                                            ThrottleCmd, queue_size=1)
        # brake control publisher
        self.brake_pub = rospy.Publisher('/vehicle/brake_cmd',
                                         BrakeCmd, queue_size=1)
    	# 2 DoF controller for controlling the angular and linear velocities
        self.controller = Controller(vehicle_mass = vehicle_mass,
                                          fuel_capacity = fuel_capacity,
                                          brake_deadband = brake_deadband,
                                          decel_limit = decel_limit,
                                          accel_limit = accel_limit,
                                          wheel_radius = wheel_radius,
                                          wheel_base = wheel_base,
                                          steer_ratio = steer_ratio,
                                          max_lat_accel = max_lat_accel,
                                          max_steer_angle = max_steer_angle,
                                          )
        # TODO: Create `TwistController` object
        # self.controller = TwistController(<Arguments you wish to provide>)
        # some varibales for linear and angular velocities and their desired values
        self.linspd_current = 0
        self.linearvel_tar = 0
        self.angularvel_tar = 0
        self.dbw_enabled = False
        self.dbg_ref_vel = 10

        # TODO: Subscribe to all the topics you need to
        # current spd
        rospy.Subscriber('/current_velocity', TwistStamped, self.velocity_cb, queue_size=1) 
        # target linear and angular spd 
        rospy.Subscriber('/twist_cmd', TwistStamped, self.twist_cmd_cb, queue_size=1) 
        # control mode
        rospy.Subscriber('/vehicle/dbw_enabled', Bool, self.dbw_enabled_cb, queue_size=1) 
        #debugging topic for target velocities
        rospy.Subscriber('debugging_ref_vel', Float64, self.dbg_ref_vel_cb, queue_size=1)

        self.loop()
    # read the current velocity
    def velocity_cb(self, msg):
        self.linspd_current = msg.twist.linear.x
    # read the reference linear and angular velocities
    def twist_cmd_cb(self, msg):
        self.linearvel_tar = msg.twist.linear.x
        self.angularvel_tar = msg.twist.angular.z
    # read the current control mode (auto or manual)
    def dbw_enabled_cb(self, msg):
        self.dbw_enabled = msg.data
    # read a reference debugging velocity
    def dbg_ref_vel_cb(self, msg):
        self.dbg_ref_vel = msg.data

    def loop(self):
        # loop control frequency
        loop_freq = 50
        # sample time of the control loop
        dt = 1/float(loop_freq)
        rate = rospy.Rate(loop_freq) 
        while not rospy.is_shutdown():
            # TODO: Get predicted throttle, brake, and steering using `twist_controller`
            # You should only publish the control commands if dbw is enabled
            # throttle, brake, steering = self.controller.control(<proposed linear velocity>,
            #                                                     <proposed angular velocity>,
            #                                                     <current linear velocity>,
            #                                                     <dbw status>,
            #                                                     <any other argument you need>)
            # if <dbw is enabled>:
            # calculate the control inputs
            throttle, brake, steering = self.controller.control(linspd_current = self.linspd_current,
                                                                linspd_tar =self.linearvel_tar,
                                                                rotspd_tar = self.angularvel_tar,dt = dt)
			# if the autonomous mode is active publish the commands
            if self.dbw_enabled:
                self.publish(throttle, brake, steering)
			# if not reset the integral control in the controller
            else:
                self.controller.longitudinal.reset()    
            rate.sleep()

    def publish(self, throttle, brake, steer):
        tcmd = ThrottleCmd()
        tcmd.enable = True
        tcmd.pedal_cmd_type = ThrottleCmd.CMD_PERCENT
        tcmd.pedal_cmd = throttle
        self.throttle_pub.publish(tcmd)

        scmd = SteeringCmd()
        scmd.enable = True
        scmd.steering_wheel_angle_cmd = steer
        self.steer_pub.publish(scmd)

        bcmd = BrakeCmd()
        bcmd.enable = True
        bcmd.pedal_cmd_type = BrakeCmd.CMD_TORQUE
        bcmd.pedal_cmd = brake
        self.brake_pub.publish(bcmd)


if __name__ == '__main__':
    DBWNode()
