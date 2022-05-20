import time
import math
import argparse

from dronekit import connect, VehicleMode, LocationGlobalRelative, Command, LocationGlobal
from pymavlink import mavutil
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from AP.cpp import *

calib_path = "AP/"
camera_matrix = np.loadtxt(calib_path + 'cameraMatrix.txt', delimiter=',')
camera_distortion = np.loadtxt(calib_path + 'cameraDistortion.txt', delimiter=',')
aruco_tracker = ArucoSingleTracker(id_to_find=5, marker_size=10, show_video=True, camera_matrix=camera_matrix,
                                   camera_distortion=camera_distortion)

parser = argparse.ArgumentParser()
parser.add_argument('--connect', default='')
args = parser.parse_args()
print('Connecting...')
vehicle = connect(args.connect)


class Controller:
    def __init__(self, P, I, D, max_out):
        self.P = P
        self.I = I
        self.D = D
        self.max_out = max_out
        self.I_out_x = 0
        self.I_out_y = 0
        self.cam_x_previous = 0
        self.cam_y_previous = 0

    def calculate(self, cam_x, cam_y, set_x, set_y):
        error_x = cam_x - set_x
        error_y = cam_y - set_y

        self.I_out_x += error_x * self.P
        self.I_out_y += error_y * self.P
        if self.I_out_x > self.max_out:
            self.I_out_x = self.max_out

        if self.I_out_y > self.max_out:
            self.I_out_y = self.max_out

        output_x = self.I_out_x + (self.P * error_x) + (self.D * (error_x - self.cam_x_previous))
        output_y = self.I_out_y + (self.P * error_y) + (self.D * (error_y - self.cam_y_previous))

        if output_x > self.max_out:
            output_x = self.max_out
        elif output_x < self.max_out * (-1):
            output_x = self.max_out * (-1)

        if output_y > self.max_out:
            output_y = self.max_out
        elif output_y < self.max_out * (-1):
            output_y = self.max_out * (-1)

        self.cam_x_previous = output_x
        self.cam_y_previous = output_y

        return output_x, output_y


def arm_and_takeoff(altitude):
    while not vehicle.is_armable:
        print("waiting to be armable")
        time.sleep(1)

    print("Arming motors")
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed: time.sleep(1)

    print("Taking Off")
    vehicle.simple_takeoff(altitude)

    while True:
        v_alt = vehicle.location.global_relative_frame.alt
        print(">> Altitude = %.1f m" % v_alt)
        if v_alt >= altitude - 1.0:
            print("Target altitude reached")
            break
        time.sleep(1)


control = Controller(0.9, 0.06, 20, 50)
bridge = CvBridge()

time_0 = time.time()
#arm_and_takeoff(5)

#vehicle.mode = 'LOITER'
#vehicle.channels.overrides['3'] = 1500

def send_land_message(vehicle, x, y, z, time_usec=0, target_num=0):
    msg = vehicle.message_factory.landing_target_encode(
        time_usec,  # time target data was processed, as close to sensor capture as possible
        target_num,  # target num, not used
        mavutil.mavlink.MAV_FRAME_BODY_NED,  # frame, not used
        x,  # X-axis angular offset, in radians
        y,  # Y-axis angular offset, in radians
        z,  # distance, in meters
        0,  # Target x-axis size, in radians
        0,  # Target y-axis size, in radians
        0,  # x	float	X Position of the landing target on MAV_FRAME
        0,  # y	float	Y Position of the landing target on MAV_FRAME
        0,  # z	float	Z Position of the landing target on MAV_FRAME
        (1, 0, 0, 0),
        # q	float[4]	Quaternion of landing target orientation (w, x, y, z order, zero-rotation is 1, 0, 0, 0)
        2,  # type of landing target: 2 = Fiducial marker
        1,  # position_valid boolean
    )
    vehicle.send_mavlink(msg)
    vehicle.flush()


class axis:
    def __init__(self, roll, pitch):
        self.roll = roll
        self.pitch = pitch


def hover_command(vehicle, pitch_adjust, roll_adjust):
    #ax = axis(vehicle.channels['1'], vehicle.channels['2'])
    pitch = 1500 + pitch_adjust
    roll = 1500 + roll_adjust
    if roll > 2000:
        roll = 2000
    if roll < 1000:
        roll = 1000
    if pitch > 2000:
        pitch = 2000
    if pitch < 1000:
        pitch = 1000

    vehicle.channels.overrides['1'] = pitch
    vehicle.channels.overrides['2'] = roll


def run(cv_image):
    marker_found, x_cm, y_cm, z_cm = aruco_tracker.track(loop=False, frame=cv_image)
    if marker_found:
        x, y = control.calculate(x_cm, y_cm, 0, 0)
        print(x, y)
        # send_land_message(vehicle, x, y, vehicle.location.global_relative_frame.alt, time_usec=0, target_num=0)
        hover_command(vehicle, x, y)

    if not vehicle.armed:
        exit()


rospy.init_node('opencv_example', anonymous=True)


def image_callback(img_msg):
    # log some info about the image topic
    rospy.loginfo(img_msg.header)

    # Try to convert the ROS Image message to a CV2 Image
    try:
        cv_image = bridge.imgmsg_to_cv2(img_msg, "passthrough")
    except CvBridgeError:
        rospy.logerr("CvBridge Error: ")

    # Flip the image 90deg
    # cv_image = cv2.transpose(cv_image)
    # cv_image = cv2.flip(cv_image,1)

    # Show the converted image
    run(cv_image)


sub_image = rospy.Subscriber("/webcam/image_raw", Image, image_callback)
rospy.spin()
