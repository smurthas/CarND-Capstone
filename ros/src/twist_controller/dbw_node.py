#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from dbw_mkz_msgs.msg import ThrottleCmd, SteeringCmd, BrakeCmd, SteeringReport
from geometry_msgs.msg import TwistStamped, PoseStamped
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

def dl(a , b):
  return math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)


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

    self.steer_pub = rospy.Publisher('/vehicle/steering_cmd',
                                     SteeringCmd, queue_size=1)
    self.throttle_pub = rospy.Publisher('/vehicle/throttle_cmd',
                                        ThrottleCmd, queue_size=1)
    self.brake_pub = rospy.Publisher('/vehicle/brake_cmd',
                                     BrakeCmd, queue_size=1)
    min_speed = 0.2

    self.controller = Controller(wheel_base, steer_ratio, min_speed,
            max_lat_accel, max_steer_angle, vehicle_mass, wheel_radius)
    self.pose = None
    self.twist = None
    self.twist_cmd = None
    self.dbw_enabled = False

    rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
    rospy.Subscriber('/current_velocity', TwistStamped, self.vel_cb)
    rospy.Subscriber('/twist_cmd', TwistStamped, self.twist_cb)
    rospy.Subscriber('/vehicle/dbw_enabled', Bool, self.dbw_status_cb)

    self.loop()

  def vel_cb(self, msg):
    self.twist = msg.twist

  def twist_cb(self, msg):
    self.twist_cmd = msg.twist

  def pose_cb(self, msg):
    self.pose = msg.pose

  def dbw_status_cb(self, msg):
    self.dbw_enabled = msg.data
    rospy.loginfo('dbw_status changed: %s', msg)

  def loop(self):
    rate = rospy.Rate(50) # 50Hz
    last_time = rospy.Time.now()
    while not rospy.is_shutdown():
      if self.dbw_enabled:
        time = rospy.Time.now()
        dt = time - last_time
        last_time = time
        cur_v = 0
        goal_lin_vel = self.twist_cmd.linear.x if self.twist_cmd is not None else 1
        goal_ang_vel = self.twist_cmd.angular.z if self.twist_cmd is not None else 0

        if self.twist is not None:
            cur_v = self.twist.linear.x

        throttle, brake, steer = self.controller.control(goal_lin_vel,
                goal_ang_vel, cur_v, 0, dt.to_sec())

        #rospy.loginfo("gv: {0:.2f}, cv: {1:.2f}, t: {2:.2f}, b: {3:.2f}, s: {4:.2f}"
        #        .format(goal_lin_vel, cur_v, throttle, brake, steer ))

        self.publish(throttle, brake, steer)

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
