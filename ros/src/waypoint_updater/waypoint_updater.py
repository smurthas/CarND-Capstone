#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32
import tf

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

LOOKAHEAD_WPS = 40 # Number of waypoints we will publish. You can change this number


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        self.pose = None
        self.waypoints = None
        self.stopping_index = -1
	self.begin_decel_distance = 25.0
	self.max_vel = 4.47
	self.decel_rate = self.max_vel / self.begin_decel_distance

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
	rospy.Subscriber('/base_velocity', Int32, self.base_velocity_cb)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        self.loop()

    def update_waypoint_velocity(self, waypoints, waypoint, index):
        if(waypoints == None or waypoint == None or index < 0):
            return

        
        target_vel = self.max_vel

        #If there is a red light index, calculate target velocity based on distance to stopline.
        if(self.stopping_index != -1):
            distance_to_stop = self.distance(waypoints, index, self.stopping_index)
            
            if(distance_to_stop <= 2):
                
                target_vel = 0
            elif(distance_to_stop < self.begin_decel_distance):
                target_vel =  self.decel_rate * distance_to_stop

        new_vel = target_vel

        #Compare current velocity to target velocity and adjust accordingly
        current_vel = self.get_waypoint_velocity(waypoint)

        if(current_vel < target_vel):
            new_vel = min(target_vel, current_vel + 1.0)
	


        self.set_waypoint_velocity(waypoints, index, new_vel)





    def loop(self):
        rate = rospy.Rate(50) # 50Hz
        prev_i = 0
        while not rospy.is_shutdown():
            # find the next waypoint from the list
            next_i = self.next_waypoint(self.waypoints, prev_i, max_dist=5)
            if next_i is None:
                rate.sleep()
                continue

            prev_i = next_i



            # build a lane object with the next LOOKAHEAD_WPS points, looping
            # around to the beginning of the list if needed
            lane = Lane()
            i = next_i
            while len(lane.waypoints) < LOOKAHEAD_WPS:
                index = i % len(self.waypoints)
                wp = self.waypoints[index]
                self.update_waypoint_velocity(self.waypoints, wp, index)
                lane.waypoints.append(wp)
                i += 1

            self.final_waypoints_pub.publish(lane)


            rate.sleep()

    def base_velocity_cb(self, msg):
        self.max_vel = msg.data
	self.decel_rate = self.max_vel / self.begin_decel_distance
	#rospy.loginfo('Vel: %s, Decel: %s', self.max_vel, self.decel_rate)

    def pose_cb(self, msg):
        self.pose = msg.pose

    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints.waypoints

    def traffic_cb(self, index):
        # TODO: Callback for /traffic_waypoint message. Implement
        #if(self.stopping_index != index.data):
            #rospy.loginfo('Stopping index: %s',index.data)
        self.stopping_index = index.data
        #print("Traffic callback: ", msg)
        

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

    def in_vehicle_frame(self, other_pose):
        ''' convert a pose to the current vehicle frame'''
        x1 = self.pose.position.x
        y1 = self.pose.position.y
        dx = other_pose.position.x - x1
        dy = other_pose.position.y - y1
        q = [self.pose.orientation.x, self.pose.orientation.y,
                self.pose.orientation.z, self.pose.orientation.w]
        yaw = tf.transformations.euler_from_quaternion(q)[2]
        f = -1.0 * yaw
        x_vehicle = dx * math.cos(f) - dy * math.sin(f)
        y_vehicle = dy * math.cos(f) + dx * math.sin(f)
        return (x_vehicle, y_vehicle)

    def next_waypoint(self, waypoints, start_at, max_dist):
        ''' find the next waypoint on the path. Start searching at start_at, and
        stop short if we find more a point closer than max_dist'''
        if self.pose is None or waypoints is None:
            return None

        closest_i = self.closest_waypoint(waypoints, start_at, 10, max_dist)
        x_veh, y_veh = self.in_vehicle_frame(waypoints[closest_i].pose.pose)
        # if the found waypoint has a position X value in the vehicle frame,
        # that means it is ahead of the vehicle, otherwise we need to return the
        # next point in the path
        if x_veh < 0:
            return closest_i + 1
        else:
            return closest_i

    def closest_waypoint(self, waypoints, start_i=0, stop_after=100, max_dist=5):
        ''' find the closest waypoint to the current pose. start searching at
        index start_i, stop short if we've found a point closer that max_dist
        and have searched stop_after points'''
        if self.pose is None or self.waypoints is None:
            return None

        closest = -1
        closest_dist = 10000000
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for j in range(0, len(waypoints)):
            used_i = (start_i + j) % len(waypoints)
            d = dl(self.pose.position, waypoints[used_i].pose.pose.position)
            if d < closest_dist:
                closest = used_i
                closest_dist = d
            if j > stop_after and closest_dist < max_dist:
                return closest

        if closest == -1:
            return None

        return closest



if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
