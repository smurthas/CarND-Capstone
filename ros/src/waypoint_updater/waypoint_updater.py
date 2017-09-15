#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint

import math

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

        self.pose = None

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below

        rospy.spin()

    def pose_cb(self, msg):
        self.pose = msg.pose
        #rospy.loginfo("x: %s, y: %s", msg.pose.position.x, msg.pose.position.y)
        # TODO: Implement
        pass

    def waypoints_cb(self, waypoints):
        #rospy.loginfo("wps: %s", waypoints)
        closest_i = self.closest_waypoint(waypoints.waypoints)
        if closest_i is None:
            return
        lane = Lane()
        i = closest_i
        while len(lane.waypoints) < LOOKAHEAD_WPS:
            wp = waypoints.waypoints[i % len(waypoints.waypoints)]
            wp.twist.twist.linear.x = 1
            lane.waypoints.append(wp)
            i += 1
        self.final_waypoints_pub.publish(lane)

        c_wp = waypoints.waypoints[closest_i]
        rospy.loginfo("closest: i: %s, x: %s, y: %s", closest_i,
                c_wp.pose.pose.position.x, c_wp.pose.pose.position.y)

        # TODO: Implement
        pass

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        pass

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

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

    def closest_waypoint(self, waypoints):
        if self.pose is None:
            return None
        closest = -1
        closest_dist = 10000000
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(0, len(waypoints)):
            d = dl(self.pose.position, waypoints[i].pose.pose.position)
            if d < closest_dist:
                closest = i
                closest_dist = d

        if closest == -1:
            return None

        return closest



if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
