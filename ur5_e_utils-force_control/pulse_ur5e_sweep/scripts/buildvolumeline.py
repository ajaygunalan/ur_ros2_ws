#!/usr/bin/env python
from socket import *
import cv2
import datetime as dtime
import math, os
# import matlab.engine
import numpy as np
import pycurl
import random
import requests
import StringIO
import sys
import time

from sensor_msgs.msg import Image
from std_msgs.msg import String, Bool
import geometry_msgs.msg
#import PyKDL
import rospy
import tf2_ros

import intera_interface

sys.path.append(os.path.realpath(os.path.join(os.path.dirname(__file__), os.pardir)))
import sawyer_api.sawyer_api as sawyer
import sawyer_pykdl.sawyer_pykdl as sk
import utils.utils

import PyKDL

class SawyerScan:
    def __init__(self, scandist=2.0):
        self.foldername = utils.get_time_stamp()
        self.scandist = scandist
        data_dir_name = os.path.join(os.path.dirname(__file__), 'data', self.foldername)
        # Write parameters to log for later volume reconstruction
        os.makedirs(data_dir_name)
        f = open(os.path.join(data_dir_name, 'image_coordinates.txt'), 'w+')
        f.close()

        self.increment = 1.0
        self.completed_scans = 0
        self.obtained_imgs = 0

        sawyer.activate_robot()
        self.cuff = intera_interface.Cuff('right')
        self.limb = intera_interface.Limb('right')
        sawyer.tf_init()
        self.solver = sk.sawyer_kinematics('right')

        self.requested_imgs = self.scandist * 20  # 10 images per centimeter
        self.original_q, p = sawyer.get_tf()

        self.socket_num = 8081
        self.ip = "169.254.118.145"
        self.ahost = self.ip
        self.aport = 12001
        self.aaddr = (self.ahost, self.aport)
        self.aUDPSock = socket(AF_INET, SOCK_DGRAM)

        # initialize nodes and subscribers
        rospy.init_node('sawyerNode', anonymous=True)
        self.img_sub = rospy.Subscriber("/verasonics/channel_data", Image, self.update_img)
        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer)
        #self.tf_sub = rospy.Subscriber("tfFrame", String, self.update_tf) # TODO: FIND TOPIC NAME FROM SAWYER
        self.s = rospy.Service('save_img_tf', saveImgTF, handle_save_img_tf)
        self.tf = geometry_msgs.msg.Twist()
        self.img = Image()

        rospy.spin()
nce the driver is built and the externalcontrol URCap is installed and running on the robot, you are good to go ahead starting the driver. (Note: We do recommend, though, to calibrate your robot first.))

        return

    def run(self):
        while not self.cuff.lower_button():
            continue

        scan_start_joint_angles = self.limb.joint_angles()
        time.sleep(3)

        f = open(os.path.join(os.path.dirname(__file__), 'data', self.foldername, 'robotsstartjointangles.txt'), 'w+')
        f.write(str(self.limb.joint_angles()))
        f.close()

        self.save_img()

        #while self.obtained_imgs < self.requested_imgs:
        #debugging: force stop after 1 image
        while self.obtained_imgs < 2:
            self.scan()

            self.save_img_tf_client(self.tf, self.img)

        #self.end_scan()
        self.limb.move_to_joint_positions(scan_start_joint_angles)
        print("Scan complete!")

        return

    def scan(self):
        self.curr_dist = 0.0
        q, p = sawyer.get_tf()
        self.prev_pos = np.array(p)

        while self.curr_dist <= self.increment / 1000.0:
            self.vx = 0.0
            self.vy = 0.001
            self.vz = 0.0

            theta = self.velocity_ik(np.array([self.vx, self.vy, self.vz]))
            self.limb.set_joint_velocities(theta)
            print(theta)
            q, p = sawyer.get_tf()
            self.curr_pos = np.array(p)
            self.curr_dist = np.linalg.norm(self.prev_pos - self.curr_pos)

        print("Current distance: ", self.curr_dist)

    def save_img(self):
        self.message = '1'
        #print "Message Sent"
        self.aUDPSock.sendto(self.message, self.aaddr)
        time.sleep(0.5)
        #cv2.waitKey(100)

        timestamp = utils.get_time_stamp()

        fp = open(
                os.path.join(
                    os.path.dirname(__file__),
                    'data',
                    self.foldername,
                    'SWEimage{:s}.png'.format(timestamp)),
                'wb')
        curl = pycurl.Curl()
        curl.setopt(pycurl.URL, "http://" + self.ip + ":" + str(self.socket_num) + "/scanner_image.png")
        curl.setopt(pycurl.WRITEDATA, fp)
        curl.perform()
        curl.close()
        fp.close()

        f = open(os.path.join(os.path.dirname(__file__), 'data', self.foldername, 'image_coordinates.txt'), 'a')
        q, p = sawyer.get_tf()
        datapoint = '{:s},{:s},{:s},{:s}\n'.format(
                str(p).strip('()'),
                str(q).strip('()'),
                str(self.obtained_imgs),
                timestamp)
        f.write(datapoint)
        f.close()

        self.obtained_imgs += 1
        print ("self.obtained_imgs")

        return

    def end_scan(self):
        # Return to start
        self.limb.move_to_joint_positions(self.scan_start_joint_angles, speed=0.05)
        print ("Scan Complete!")
        return

    def velocity_ik(self, v, scanning=False):
        """
        Purpose: Given a 3x1 velocity vector, compute the joint velocities to realize end effector velocity v
        :param v: 3x1 velocity vector. type = numpy array
        :return: theta: Dict of joint velocities matched to the proper Sawyer joints
        """

        def vel2kdltwist(velocity):
            """
            Purpose: Converts a 6x1 complete velocity vector to a PyKDL Twist() object for use in inverse kinematics
            :param velocity: 6x1 velocity vector (cartesian x,y,z and rotational x,y,z)
            :return: V: Velocity Twist type = PyKDL Twist() object
            """
            V = PyKDL.Twist()

            for i in range(len(velocity)):
                V[i] = velocity[i]

            return V

        if scanning is True:
            q = self.original_q
        else:
            q, p = sawyer.get_tf()

        R = sawyer.quat2rot(q)
        v_ = R.dot(v[:3])
        #v = np.array([v_[0], v_[1], v_[2], v[3], v[4], v[5]])
        twist = vel2kdltwist(v_)
        theta = self.solver.inverse_velocity_kinematics(twist)
        theta = dict(zip(self.limb.joint_names(), np.array(theta).tolist()))

        return theta

    def handle_save_img_tf(self, req):
        timestamp = utils.get_time_stamp()
        try:
            cv_img = self.bridge.imgmsg_to_cv2(req.img, 'bgr8')
            cv2.imshow('SavedImage', cv_img)
            cv2.imwrite(os.path.join(os.path.dirname(__file__), 'data', self.foldername, 'image{:s}.png'.format(timestamp)), cv_img)

            cv2.destroyAllWindows()

        except CvBridgeError as e:
            print(e)

        f = open(os.path.join(os.path.dirname(__file__), 'data', self.foldername, 'tf_frame.txt'), 'a')

        datapoint = '{:s},{:s},{:s}\n'.format(
                str(req.trans).strip('()'),
                str(req.rot).strip('()'),
                timestamp)
        f.write(datapoint)
        f.close()

        return

    def save_img_tf_client(img):
        rospy.wait_for_service('save_img_tf')
        try:
            save_img_tf = rospy.ServiceProxy('save_img_tf', saveImgTF)
            (tf_trans, tf_rot) = self.tfBuffer.lookup_transform("/base", "/right_hand", rospy.Time(0))
            resp1 = save_img_tf(tf_trans, tf_rot, img)
            return resp1.response
        except (rospy.ServiceException, tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            print("Service call failed")

    def update_tf(self, data):
        self.tf = data

    def update_img(self, data):
        self.img = data

if __name__ == "__main__":
    sawyer.activate_robot()

    scandist = 3 #[cm]

    s = SawyerScan(scandist)

    s.run()
