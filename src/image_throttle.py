#!/usr/bin/env python
# Lib
import rospy

from sensor_msgs.msg import Image, CameraInfo



class Image_Throttle:
    def __init__(self, name):
        # Params
        self.period = rospy.get_param('/image_throttle_period', default=3)

        # ROS Comms
        rospy.Subscriber("stereo_down/scaled_x2/left/image_raw", Image, self.img_left_cb, queue_size=1)
        rospy.Subscriber("stereo_down/scaled_x2/right/image_raw", Image, self.img_right_cb, queue_size=1)
        rospy.Subscriber("stereo_down/scaled_x2/left/camera_info", CameraInfo, self.info_left_cb, queue_size=1)
        rospy.Subscriber("stereo_down/scaled_x2/right/camera_info", CameraInfo, self.info_right_cb, queue_size=1)

        self.pub_img_left = rospy.Publisher('stereo_down/scaled_x2/left/image_raw_th', Image, queue_size=5)
        self.pub_img_right = rospy.Publisher('stereo_down/scaled_x2/right/image_raw_th', Image, queue_size=5)
        self.pub_info_left = rospy.Publisher('stereo_down/scaled_x2/left/camera_info_th', CameraInfo, queue_size=5)
        self.pub_info_right = rospy.Publisher('stereo_down/scaled_x2/right/camera_info_th', CameraInfo, queue_size=5)

        # Throttle
        rospy.sleep(5)
        rospy.Timer(rospy.Duration(self.period), self.publish)


    def img_left_cb(self, msg): 
        self.img_left = msg

    def img_right_cb(self, msg): 
        self.img_right = msg

    def info_left_cb(self, msg): 
        self.info_left = msg

    def info_right_cb(self, msg): 
        self.info_right = msg

    def publish(self,event=''): 
        self.pub_img_left.publish(self.img_left)
        self.pub_img_right.publish(self.img_right)
        self.pub_info_left.publish(self.info_left)
        self.pub_info_right.publish(self.info_right)

if __name__ == '__main__':
    try:
        name = 'image_throttle'
        rospy.init_node(name)

        Image_Throttle(name)
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
