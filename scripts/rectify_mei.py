#!/usr/bin/env python3
"""Rectify a calibrated MEI camera to a pinhole camera with unchanged axes/stamps."""

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image


def build_rectification_maps(source, target):
    """Map each virtual pinhole pixel through the MEI forward projection."""
    if source['model_type'] != 'MEI':
        raise ValueError('Expected an MEI source camera')
    if target['cam_model'] != 'Pinhole' or target.get('scale', 1.0) != 1.0:
        raise ValueError('Expected a full-scale Pinhole target camera')
    if any(target.get('cam_d' + str(i), 0.0) != 0.0 for i in range(4)):
        raise ValueError('The rectified target must have zero distortion')
    u, v = np.meshgrid(np.arange(target['cam_width'], dtype=np.float64),
                       np.arange(target['cam_height'], dtype=np.float64))
    x = (u - target['cam_cx']) / target['cam_fx']
    y = (v - target['cam_cy']) / target['cam_fy']
    # Project ray [x, y, 1] onto the unified camera's normalized image plane.
    denominator = 1.0 + source['mirror_parameters']['xi'] * np.sqrt(x*x + y*y + 1.0)
    x, y = x / denominator, y / denominator
    d = source['distortion_parameters']
    r2 = x*x + y*y
    radial = 1.0 + d['k1'] * r2 + d['k2'] * r2*r2
    xd = x * radial + 2.0 * d['p1'] * x*y + d['p2'] * (r2 + 2.0*x*x)
    yd = y * radial + d['p1'] * (r2 + 2.0*y*y) + 2.0 * d['p2'] * x*y
    p = source['projection_parameters']
    return ((p['gamma1'] * xd + p['u0']).astype(np.float32),
            (p['gamma2'] * yd + p['v0']).astype(np.float32))


class MeiRectifier:
    def __init__(self):
        source = rospy.get_param('~rgb_camera')
        target = rospy.get_param('~')
        self.source_size = (source['image_width'], source['image_height'])
        self.map_x, self.map_y = build_rectification_maps(source, target)
        # Reject a target field of view that introduces invalid black image borders.
        if (self.map_x.min() < 0 or self.map_y.min() < 0 or
                self.map_x.max() >= self.source_size[0] - 1 or
                self.map_y.max() >= self.source_size[1] - 1):
            raise ValueError('Target field of view exceeds the calibrated source image')
        self.bridge = CvBridge()
        self.publisher = rospy.Publisher('~output', Image, queue_size=5)
        self.subscriber = rospy.Subscriber('~input', Image, self.callback,
                                           queue_size=5, buff_size=8*1024*1024)
        rospy.loginfo('MEI rectification ready: %dx%d -> %dx%d',
                      *self.source_size, target['cam_width'], target['cam_height'])

    def callback(self, message):
        if (message.width, message.height) != self.source_size:
            rospy.logerr_throttle(5.0, 'Image dimensions do not match MEI calibration')
            return
        try:
            raw = self.bridge.imgmsg_to_cv2(message, desired_encoding='bgr8')
            rectified = cv2.remap(raw, self.map_x, self.map_y, cv2.INTER_LINEAR)
            output = self.bridge.cv2_to_imgmsg(rectified, encoding='bgr8')
        except (CvBridgeError, cv2.error) as error:
            rospy.logerr_throttle(5.0, 'MEI rectification failed: %s', error)
            return
        output.header = message.header
        self.publisher.publish(output)


if __name__ == '__main__':
    rospy.init_node('mei_rectifier')
    MeiRectifier()
    rospy.spin()
