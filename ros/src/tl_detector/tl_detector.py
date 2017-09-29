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
import numpy as np
from tf.transformations import euler_from_quaternion

STATE_COUNT_THRESHOLD = 3

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')
        rospy.loginfo('tl_detector node initialized')
        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.lights =
        self.stopline_wps = []

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

    def waypoints_cb(self, waypoints):
        #only load waypoints once
        if self.waypoints is None:
            self.waypoints = waypoints
            rospy.loginfo('base waypoints loaded')
            #print('base waypoints loaded')
            self.get_stopline_waypoint()
            rospy.loginfo('stopline waypoints loaded')
            #print('stopline waypoints loaded')
            print(self.stopline_wps)

    def traffic_cb(self, msg):
        self.lights = msg.lights

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
        self.has_image = True
        self.camera_image = msg
        light_wp, state = self.process_traffic_lights()

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
            if __name__ == '__main__':
                light_wp = light_wp if state == TrafficLight.RED else -1
            self.last_wp = light_wp
            self.upcoming_red_light_pub.publish(Int32(light_wp))
        else:
            self.upcoming_red_light_pub.publish(Int32(self.last_wp))


        rospy.loginfo('upcoming red light: %s',light_wp)
        #print('upcoming red light: ',light_wp)
        rospy.loginfo('state: %s',state)
        #print('state: ', state)
        self.state_count += 1


    def get_stopline_waypoint(self):
        get_dist = lambda a, b: math.sqrt((a.x - b[0]) ** 2 + (a.y - b[1]) ** 2)
        for stopline in range(len(self.config['stop_line_positions'])):
            closest_dist = 100000
            stopline_wp = -1
            for wp in range(len(self.waypoints.waypoints)):
                d = get_dist(self.waypoints.waypoints[wp].pose.pose.position,self.config['stop_line_positions'][stopline])
                if closest_dist > d:
                    closest_dist = d
                    stopline_wp = wp
            self.stopline_wps.append(stopline_wp)


    def get_closest_waypoint(self, pose):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
        #TODO implement
        closest_wp = -1
        closest_dist = 10000000

        get_dist = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)

        if self.waypoints:
            for wp in range(len(self.waypoints.waypoints)):
                d = get_dist(pose.position, self.waypoints.waypoints[wp].pose.pose.position)
                if d<closest_dist:
                    closest_dist = d
                    closest_wp = wp

            x = self.waypoints.waypoints[closest_wp].pose.pose.position.x
            y = self.waypoints.waypoints[closest_wp].pose.pose.position.y
            heading = np.arctan2((y - pose.position.y), (x - pose.position.x))
            euler = euler_from_quaternion([pose.orientation.x,
                                           pose.orientation.y,
                                           pose.orientation.z,
                                           pose.orientation.w])
            theta = euler[2]
            angle = np.abs(theta-heading)

            if angle > np.pi/4:
                closest_wp +=1

        return closest_wp


    def project_to_image_plane(self, point_in_world):
        """Project point from 3D world coordinates to 2D camera image location

        Args:
            point_in_world (Point): 3D location of a point in the world

        Returns:
            x (int): x coordinate of target point in image
            y (int): y coordinate of target point in image

        """

        fx = self.config['camera_info']['focal_length_x']
        fy = self.config['camera_info']['focal_length_y']
        image_width = self.config['camera_info']['image_width']
        image_height = self.config['camera_info']['image_height']

        # get transform between pose of camera and world frame
        trans = None
        try:
            now = rospy.Time.now()
            self.listener.waitForTransform("/base_link",
                  "/world", now, rospy.Duration(1.0))
            (trans, rot) = self.listener.lookupTransform("/base_link",
                  "/world", now)

        except (tf.Exception, tf.LookupException, tf.ConnectivityException):
            rospy.logerr("Failed to find camera to map transform")

        #TODO Use tranform and rotation to calculate 2D position of light in image

        x = 0
        y = 0

        return (x, y)

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

        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")
        #Get classification
        state = self.light_classifier.get_classification(cv_image)

        return state

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        stop_line_positions = self.config['stop_line_positions']

        if(self.pose):
            car_position = self.get_closest_waypoint(self.pose.pose)
            #print ('car pos: ', car_position)

        #TODO find the closest visible traffic light (if one exists)
        #use available info for now

        closest_dist = 100000
        closest_stopline = -1
        closest_stopline_wp = -1
        state = 4
        if(self.stopline_wps):
            for i in range(len(self.stopline_wps)):
                d = self.stopline_wps[i] - car_position
                #only consider lights that's ahead of the car
                if (d > 0) & (closest_dist > d):
                    closest_dist = d
                    closest_stopline = i

            closest_stopline_wp = self.stopline_wps[closest_stopline]

            #only run traffic light classifier when the upcoming light is within 100m range
            if closest_dist < 100:
                state = self.get_light_state(self.lights[closest_stopline])

        return closest_stopline_wp, state

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
