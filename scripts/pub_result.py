#!/usr/bin/env python
from __future__ import print_function

import roslib
roslib.load_manifest('term_project')
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from term_project.msg import BoundingBox
from term_project.msg import BoundingBoxes
from term_project.msg import FiducialArray
from term_project.msg import FiducialTransformArray

class image_converter:
  def __init__(self):
    self.image_pub = rospy.Publisher("/term_project/result_image",Image)
    self.bridge = CvBridge()
    # self.Line_sub = rospy.Subscriber("/term_project/line_image",Image,self.callback_line)
    self.Line_sub = rospy.Subscriber("/fiducial_images",Image,self.callback_line)
    self.darknet_sub = rospy.Subscriber("/darknet_ros/bounding_boxes",BoundingBoxes,self.callback_dark)
    self.depth_sub = rospy.Subscriber("/camera/depth/image_rect_raw",Image,self.callback_depth)
    self.aruco_transform = rospy.Subscriber("/fiducial_transforms",FiducialTransformArray,self.callback_transform)
    self.final_img = np.zeros(shape=(480,640,3))
    self.bounding_boxes = []
    self.pre_xy = [0, 0, 0, 0]
    self.fidu_trans = []

  def callback_line(self,data):
    try:
      self.final_img = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)
  
  def callback_dark(self,data):
    try:
      self.bounding_boxes=data.bounding_boxes
    except CvBridgeError as e:
      print(e)

  def callback_transform(self, data):
    try:
      self.fidu_trans=data.transforms
    except CvBridgeError as e:
      print(e)
    
  def callback_depth(self,data):
    try:
      depth_frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
      depth_frame = cv2.cvtColor(depth_frame, cv2.COLOR_BGR2GRAY)
    except CvBridgeError as e:
      print(e)
    
    for bbs in self.bounding_boxes:
      if self.pre_xy != [bbs.xmin, bbs.ymin, bbs.xmax, bbs.ymax]:
          distance=round(np.mean(depth_frame[bbs.xmin:bbs.xmax, bbs.ymin:bbs.ymax]), 2)
          # print('(',bbs.xmin, bbs.ymin, ') (', bbs.xmax, bbs.ymax, ')')
          self.final_img = cv2.rectangle(self.final_img, (bbs.xmin, bbs.ymin), (bbs.xmax, bbs.ymax), (0, 0, 255), 2)
          cv2.putText(self.final_img, 'Person', (bbs.xmin+10, bbs.ymin-5), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
          cv2.putText(self.final_img, '{}m'.format(distance), (bbs.xmin+10, bbs.ymin+15), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
      self.pre_xy = [bbs.xmin, bbs.ymin, bbs.xmax, bbs.ymax]

    print(self.fidu_trans)
    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(self.final_img, "bgr8"))
    except CvBridgeError as e:
      print(e)

    # cv2.imshow('final_img', self.final_img)
    # if cv2.waitKey(1) == ord('q'):
    #     return

def main(args):
  ic = image_converter()
  rospy.init_node('image_converter', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)