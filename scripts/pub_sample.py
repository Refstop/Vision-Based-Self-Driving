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
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError
# /mobile_base/commands/velocity
import glob

imgn = glob.glob('/home/jetbot/vscode/vision_term_project/image/*.png')
imgdn = glob.glob('/home/jetbot/vscode/vision_term_project/depth/*.png')
imgn = sorted(imgn)
imgdn = sorted(imgdn)

def main(args):
  rospy.init_node('image_converter', anonymous=True)
  bridge = CvBridge()
  image_pub = rospy.Publisher("/term_project/line_image",Image)
  depth_pub = rospy.Publisher("/camera/depth/image_rect_raw",Image)
  cmd_vel_pub = rospy.Publisher("/mobile_base/commands/velocity",Twist)
  vel_msg = Twist()
  while True:
    rospy.sleep(0.3)
    num=0
    for fname in imgn:
      image=cv2.imread(fname)
      depth=cv2.imread(imgdn[num])
      num+=1
      hsv=cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
      blur=cv2.GaussianBlur(hsv[:,:,1], (3, 3), 0)
  
      sxbinary = np.zeros_like(blur)
      sxbinary[(blur >= 70) & (blur <= 200)] = 255
      imshape = blur.shape
      
      src = np.array([[(0,imshape[0]),(150, imshape[0]-150), (imshape[1]-120, imshape[0]-150), (imshape[1], imshape[0])]], dtype=np.float32)
      dst = np.float32([[0, imshape[0]], [0, 0], [imshape[1], 0], [imshape[1], imshape[0]] ])
      M = cv2.getPerspectiveTransform(src, dst)
  
      warped = cv2.warpPerspective(sxbinary, M, (imshape[1], imshape[0]))
      warped = cv2.Canny(warped, 90, 170)

      rho = 1
      theta = np.pi/180 
      threshold = 50    
      min_line_length = 30 
      max_line_gap = 5
      line_image = np.copy(image)*0
      lines = cv2.HoughLinesP(warped, rho, theta, threshold, None, min_line_length, max_line_gap)
      right_deg=np.array([]); left_deg=np.array([])
      try:
        for line in lines:
          for x1,y1,x2,y2 in line:
              slope_deg=np.arctan2(y1-y2, x1-x2)*180/np.pi
              if x1>imshape[1]/2: right_deg = np.append(right_deg, slope_deg)
              else: left_deg = np.append(left_deg, slope_deg)
              cv2.line(line_image,(x1,y1),(x2,y2),(255,0,0),5)
      except:
          continue
      if len(left_deg) > 1:
          maxpos=np.where(left_deg == np.max(left_deg))[0][0]
          minpos=np.where(left_deg == np.min(left_deg))[0][0]
          if maxpos == 0:
              left_deg[maxpos] = left_deg[maxpos+1]
          else:
              left_deg[maxpos] = left_deg[0]
  
          if minpos == 0:
              left_deg[minpos] = left_deg[minpos+1]
          else:
              left_deg[minpos] = left_deg[0]
          
          mean_l=np.mean(left_deg)

      if len(right_deg) > 1:
          maxpos=np.where(right_deg == np.max(right_deg))[0][0]
          minpos=np.where(right_deg == np.min(right_deg))[0][0]
          if maxpos == 0:
              right_deg[maxpos] = right_deg[maxpos+1]
          else:
              right_deg[maxpos] = right_deg[0]
  
          if minpos == 0:
              right_deg[minpos] = right_deg[minpos+1]
          else:
              right_deg[minpos] = right_deg[0]
          mean_r=np.mean(right_deg)
      
      means = [mean_l, mean_r]
      linearx=0.35
      angz=0
      if mean_l < 0 and mean_r < 0:
          angz=-min(means)*6.6/400
          print('left curve:', min(means))
      elif mean_l > 0 and mean_r > 0:
          angz=-max(means)*6.6/400
          print('right curve:', max(means))
      elif len(left_deg) < 1 and mean_r < 0:
          angz=-min(means)*6.6/400
          print('left curve:', min(means))
      elif mean_l > 0 and len(right_deg) < 1:
          angz=-max(means)*6.6/400
          print('right curve:',max(means))
      else:
          # linearx=0.75 #max1.5
          angz=0 #max6.6
          print('straight')
      print('linearx:', linearx)
      print('angz:',angz)
      print()
      vel_msg.linear.x = linearx
      vel_msg.angular.z = angz
      invM = cv2.getPerspectiveTransform(dst, src)
      line_image_origin = cv2.warpPerspective(line_image, invM, (imshape[1], imshape[0]))
      result_img = cv2.addWeighted(image, 0.8, line_image_origin, 1, 0)
      # warped_3ch = np.dstack((warped, warped, warped))
      # warped_edges = cv2.addWeighted(warped_3ch, 0.8, line_image, 1, 0)
      try:
        image_pub.publish(bridge.cv2_to_imgmsg(result_img, "bgr8"))
        # cimage_pub.publish(bridge.cv2_to_imgmsg(result_img))
        depth_pub.publish(bridge.cv2_to_imgmsg(depth, "bgr8"))
        cmd_vel_pub.publish(vel_msg)
      except CvBridgeError as e:
        print(e)
      # cv2.imshow("Image window", image)
      cv2.waitKey(20)

  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)