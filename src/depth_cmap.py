#!/usr/bin/env python
from __future__ import print_function
import rospy
from cv_bridge import CvBridge
import numpy as np
from matplotlib import cm
from sensor_msgs.msg import Image


class DepthCmap:
    def __init__(self, dmin, dmax, cmap="jet"):
        self.cvbridge = CvBridge()
        self.dmin = dmin * 1000
        self.dmax = dmax * 1000
        self.cmap_lut = cm.get_cmap(cmap, 256)

    def img_cb(self, msg):
        depth_img = self.cvbridge.imgmsg_to_cv2(msg)

        # cut off min and max values
        depth_img.setflags(write=1)
        depth_img[depth_img < self.dmin] = self.dmin
        depth_img[depth_img > self.dmax] = self.dmax

        # scale requested depth range to [0,1]
        gray_scale = (depth_img - self.dmin) / (self.dmax - self.dmin)
        depth_mapped = self.cmap_lut(gray_scale)
        depth_mapped = (depth_mapped[:, :, :3] * 255).astype(np.uint8)

        # publish remapped colour image
        msg_mapped = self.cvbridge.cv2_to_imgmsg(depth_mapped, encoding="rgb8")
        self.pub.publish(msg_mapped)

    def main(self):
        rospy.init_node("depth_cmap", anonymous=True)
        self.pub = rospy.Publisher("depth_cmap", Image, queue_size=1)
        rospy.Subscriber("image_depth", Image, self.img_cb)
        rospy.spin()


if __name__ == '__main__':
    dhsv = DepthCmap(0.5, 1.5, "jet")
    dhsv.main()
