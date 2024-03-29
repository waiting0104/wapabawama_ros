#!/usr/bin/env python

"""
    SORT: A Simple, Online and Realtime Tracker
    Copyright (C) 2016-2020 Alex Bewley alex@bewley.ai
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
"""
from __future__ import print_function
from itertools import cycle

from mimetypes import init

import numpy as np
import cv2
from skimage import io

import timeit
import argparse
from filterpy.kalman import KalmanFilter

import rospy
from darknet_ros_msgs.msg import BoundingBoxes
from darknet_ros_msgs.msg import BoundingBox
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import message_filters
import sys
import signal
def signal_handler(signal, frame): # ctrl + c -> exit program
        print('You pressed Ctrl+C!')
        sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)


try:
  from numba import jit
except:
  def jit(func):
    return func
np.random.seed(0)


def linear_assignment(cost_matrix):
  try:
    import lap
    _, x, y = lap.lapjv(cost_matrix, extend_cost=True)
    return np.array([[y[i],i] for i in x if i >= 0]) #
  except ImportError:
    from scipy.optimize import linear_sum_assignment
    x, y = linear_sum_assignment(cost_matrix)
    return np.array(list(zip(x, y)))


@jit
def iou(bb_test, bb_gt):
  """
  Computes IUO between two bboxes in the form [x1,y1,x2,y2]
  """
  xx1 = np.maximum(bb_test[0], bb_gt[0])
  yy1 = np.maximum(bb_test[1], bb_gt[1])
  xx2 = np.minimum(bb_test[2], bb_gt[2])
  yy2 = np.minimum(bb_test[3], bb_gt[3])
  w = np.maximum(0., xx2 - xx1)
  h = np.maximum(0., yy2 - yy1)
  wh = w * h
  o = wh / ((bb_test[2] - bb_test[0]) * (bb_test[3] - bb_test[1])
    + (bb_gt[2] - bb_gt[0]) * (bb_gt[3] - bb_gt[1]) - wh)
  return(o)


def convert_bbox_to_z(bbox):
  """
  Takes a bounding box in the form [x1,y1,x2,y2] and returns z in the form
    [x,y,s,r] where x,y is the centre of the box and s is the scale/area and r is
    the aspect ratio
  """
  w = bbox[2] - bbox[0]
  h = bbox[3] - bbox[1]
  x = bbox[0] + w/2.
  y = bbox[1] + h/2.
  s = w * h    #scale is just area
  r = w / float(h)
  return np.array([x, y, s, r]).reshape((4, 1))


def convert_x_to_bbox(x,score=None):
  """
  Takes a bounding box in the centre form [x,y,s,r] and returns it in the form
    [x1,y1,x2,y2] where x1,y1 is the top left and x2,y2 is the bottom right
  """
  w = np.sqrt(x[2] * x[3])
  h = x[2] / w
  if(score==None):
    return np.array([x[0]-w/2.,x[1]-h/2.,x[0]+w/2.,x[1]+h/2.]).reshape((1,4))
  else:
    return np.array([x[0]-w/2.,x[1]-h/2.,x[0]+w/2.,x[1]+h/2.,score]).reshape((1,5))


class KalmanBoxTracker(object):
  """
  This class represents the internal state of individual tracked objects observed as bbox.
  """
  count = 0
  def __init__(self,bbox):
    """
    Initialises a tracker using initial bounding box.
    """
    #define constant velocity model
    self.kf = KalmanFilter(dim_x=7, dim_z=4) 
    self.kf.F = np.array([[1,0,0,0,1,0,0],[0,1,0,0,0,1,0],[0,0,1,0,0,0,1],[0,0,0,1,0,0,0],  [0,0,0,0,1,0,0],[0,0,0,0,0,1,0],[0,0,0,0,0,0,1]])
    self.kf.H = np.array([[1,0,0,0,0,0,0],[0,1,0,0,0,0,0],[0,0,1,0,0,0,0],[0,0,0,1,0,0,0]])

    self.kf.R[2:,2:] *= 10.
    self.kf.P[4:,4:] *= 1000. #give high uncertainty to the unobservable initial velocities
    self.kf.P *= 10.
    self.kf.Q[-1,-1] *= 0.01
    self.kf.Q[4:,4:] *= 0.01

    self.kf.x[:4] = convert_bbox_to_z(bbox)
    self.time_since_update = 0
    self.id = KalmanBoxTracker.count
    KalmanBoxTracker.count += 1
    self.history = []
    self.hits = 0
    self.hit_streak = 0
    self.age = 0


  def update(self,bbox):
    """
    Updates the state vector with observed bbox.
    """
    self.time_since_update = 0
    self.history = []
    self.hits += 1
    self.hit_streak += 1
    self.kf.update(convert_bbox_to_z(bbox))


  def predict(self):
    """
    Advances the state vector and returns the predicted bounding box estimate.
    """
    if((self.kf.x[6]+self.kf.x[2])<=0):
      self.kf.x[6] *= 0.0
    self.kf.predict()
    self.age += 1
    if(self.time_since_update>0):
      self.hit_streak = 0
    self.time_since_update += 1
    self.history.append(convert_x_to_bbox(self.kf.x))
    return self.history[-1]


  def get_state(self):
    """
    Returns the current bounding box estimate.
    """
    return convert_x_to_bbox(self.kf.x)


def associate_detections_to_trackers(detections,trackers,iou_threshold = 0.3):
  """
  Assigns detections to tracked object (both represented as bounding boxes)
  Returns 3 lists of matches, unmatched_detections and unmatched_trackers
  """
  if(len(trackers)==0):
    return np.empty((0,2),dtype=int), np.arange(len(detections)), np.empty((0,5),dtype=int)
  iou_matrix = np.zeros((len(detections),len(trackers)),dtype=np.float32)

  for d,det in enumerate(detections):
    for t,trk in enumerate(trackers):
      iou_matrix[d,t] = iou(det,trk)

  if min(iou_matrix.shape) > 0:
    a = (iou_matrix > iou_threshold).astype(np.int32)
    if a.sum(1).max() == 1 and a.sum(0).max() == 1:
        matched_indices = np.stack(np.where(a), axis=1)
    else:
      matched_indices = linear_assignment(-iou_matrix)
  else:
    matched_indices = np.empty(shape=(0,2))

  unmatched_detections = []
  for d, det in enumerate(detections):
    if(d not in matched_indices[:,0]):
      unmatched_detections.append(d)
  unmatched_trackers = []
  for t, trk in enumerate(trackers):
    if(t not in matched_indices[:,1]):
      unmatched_trackers.append(t)

  #filter out matched with low IOU
  matches = []
  for m in matched_indices:
    if(iou_matrix[m[0], m[1]]<iou_threshold):
      unmatched_detections.append(m[0])
      unmatched_trackers.append(m[1])
    else:
      matches.append(m.reshape(1,2))
  if(len(matches)==0):
    matches = np.empty((0,2),dtype=int)
  else:
    matches = np.concatenate(matches,axis=0)

  return matches, np.array(unmatched_detections), np.array(unmatched_trackers)


class Sort(object):
  def __init__(self, max_age=5, min_hits=1):
    rospy.init_node('sort', anonymous=True)
    bbox_topic = rospy.get_param("~bbox","/darknet_ros/bounding_boxes")
    tracked_bbox_topic = rospy.get_param("~tracked_bbox","/tracked_boxes")
    display = rospy.get_param("~display", True)
    max_age = rospy.get_param("~max_age", max_age)
    min_hits = rospy.get_param("~min_hits", min_hits)
    self.iou_threshold = rospy.get_param("~iou_threshold", 0.3)
    # self.subb = rospy.Subscriber(bbox_topic, BoundingBoxes, self.boxcallback)
    self.subb = message_filters.Subscriber(bbox_topic,BoundingBoxes)
    self.pubb = rospy.Publisher(tracked_bbox_topic, BoundingBoxes, queue_size=50)
    self.rate = rospy.Rate(10)
    if display:
        img_topic = rospy.get_param('~img_topic', '/camera/image_raw')
        self.window_name = rospy.get_param('~window_name', 'image')
        self.display = display
        # self.subimage = rospy.Subscriber(img_topic, Image, self.imgcallback)
        self.subimage = message_filters.Subscriber(img_topic,Image)
        self.pubimage = rospy.Publisher('tracked_image', Image, queue_size=20)
    sync = message_filters.ApproximateTimeSynchronizer([self.subb,self.subimage],queue_size=10,slop=0.5,allow_headerless=True)
    sync.registerCallback(self.callback)
    """
    Sets key parameters for SORT
    """
    self.max_age = max_age
    self.min_hits = min_hits
    self.trackers = []
    self.frame_count = 0
    #self.colours = np.random.rand(32, 3) #used only for display

    self.img_in = 0
    self.bbox_checkin = 0
    self.bridge = CvBridge()

  def callback(self,Boundingboxes,Image):
    if self.display:
        try : 
            self.img = self.bridge.imgmsg_to_cv2(Image, "bgr8")
        except CvBridgeError as e:
            pass
    self.img_in = 1
    dets = []
    for i in range(len(Boundingboxes.bounding_boxes)):
        bbox = Boundingboxes.bounding_boxes[i]
        dets.append(np.array([bbox.xmin, bbox.ymin, bbox.xmax, bbox.ymax, bbox.probability]))
        
    self.dets = np.array(dets)
    self.bbox_checkin=1
    return
  

  def update(self, dets=np.empty((0, 6))):
    """
    Params:
      dets - a numpy array of detections in the format [[x1,y1,x2,y2,score],[x1,y1,x2,y2,score],...]
    Requires: this method must be called once for each frame even with empty detections (use np.empty((0, 5)) for frames without detections).
    Returns the a similar array, where the last column is the object ID.
    NOTE: The number of objects returned may differ from the number of detections provided.
    """
    self.frame_count += 1
    
    # get predicted locations from existing trackers.
    trks = np.zeros((len(self.trackers), 6))
    to_del = []
    ret = []
    for t, trk in enumerate(trks):
      pos = self.trackers[t].predict()[0]
      trk[:] = [pos[0], pos[1], pos[2], pos[3], 0 ,0]
      if np.any(np.isnan(pos)):
        to_del.append(t)
    trks = np.ma.compress_rows(np.ma.masked_invalid(trks))
    for t in reversed(to_del):
      self.trackers.pop(t)
    matched, unmatched_dets, unmatched_trks = associate_detections_to_trackers(dets,trks,self.iou_threshold)

    # update matched trackers with assigned detections
    for m in matched:
      self.trackers[m[1]].update(dets[m[0], :])

    # create and initialise new trackers for unmatched detections
    for i in unmatched_dets:
        trk = KalmanBoxTracker(dets[i,:])
        
        self.trackers.append(trk)
    i = len(self.trackers)
    
    for trk in reversed(self.trackers):

        d = trk.get_state()[0]
        if (trk.time_since_update < 1) and (trk.hit_streak >= self.min_hits or self.frame_count <= self.min_hits):
          ret.append(np.concatenate((d,[trk.id+1],[int(trk.hits)])).reshape(1,-1)) # +1 as MOT benchmark requires positive
        i -= 1
        # remove dead tracklet
        if(trk.time_since_update > self.max_age):
          self.trackers.pop(i)
    if(len(ret)>0):

      return np.concatenate(ret)
    return np.empty((0,5))





if __name__ == '__main__':
    np.random.seed(8)
    countt=0
    colours = np.random.rand(31, 3) #used only for display
    mot_tracker = Sort(max_age=200, min_hits=5) #create instance of the SORT tracker
    count = 0 
    init_r = BoundingBoxes()
    for i in range(0,500):
      x = BoundingBox()
      x.id = i 
      init_r.bounding_boxes.append(x)
    timecount=1
    cycle_time_sum = 0
    # for element in init_r.bounding_boxes:
    #   if element.id==3:
    #     print(element.id)
    # cv2.namedWindow(mot_tracker.window_name,0)
    while True:
        try:
          
            start = timeit.default_timer()
            if mot_tracker.bbox_checkin==1:
                trackers = mot_tracker.update(mot_tracker.dets)
                end = timeit.default_timer()
                cycle_time1 = end - start  

                # cycle_time_sum =  cycle_time_sum +cycle_time1
                # print(timecount)
                # if timecount==4000:
                #   print(cycle_time_sum/timecount)
                #   break
                timecount+=1
                r = BoundingBoxes()
                # print(trackers)
                for d in range(len(trackers)):
                    rb = BoundingBox()
                    rb.probability=1
                    rb.xmin = int(trackers[d,0])
                    rb.ymin = int(trackers[d,1])
                    rb.xmax = int(trackers[d,2])
                    rb.ymax = int(trackers[d,3])
                    rb.id = int(trackers[d,4])
                    rb.count = int(trackers[d,5])
                    # print("id=",rb.id,"count:",rb.count) 
                    if rb.id>count:
                      count = rb.id
                    rb.Class = 'tracked'
                    r.bounding_boxes.append(rb)
                    # print("trackers :",rb)
                    res = trackers[d].astype(np.int32)
                    rgb=colours[res[4]%31,:]*255
                    cv2.rectangle(mot_tracker.img, (res[0],res[1]), (res[2],res[3]), (rgb[0],rgb[1],rgb[2]), 6)
                    cv2.putText(mot_tracker.img, "ID : %d"%(res[4]), (res[0],res[1]), cv2.FONT_HERSHEY_COMPLEX, 2, (0 ,0 ,255), 5)
                        # print(mot_tracker.dets)
                
                if mot_tracker.img_in==1 and mot_tracker.bbox_checkin==1 and mot_tracker.display:
                    try : 
                        # print("start")
                        
                        # cv2.resizeWindow(mot_tracker.window_name,1920/2,1080/2)
                        # cv2.imshow(mot_tracker.window_name,mot_tracker.img)
                        # cv2.putText(mot_tracker.img, "Orchid counts : %d"%count, (20,120), cv2.FONT_HERSHEY_COMPLEX, 3.5, (114,228,120), 10)
                        # cv2.waitKey(3)
                        # # if(countt<300):
                        #   countt += 1
                        #   cv2.imwrite('/home/nvidia/Documents/mota/sort0622_3/'+str(countt)+".jpg",mot_tracker.img)
                       

                        
                #         # mot_tracker.image = mot_tracker.bridge.cv2_to_imgmsg(mot_tracker.img, "bgr8")
                #         # mot_tracker.image.header.stamp = rospy.Time.now()
                #         # mot_tracker.pubimage.publish(mot_tracker.image)
                        r.header.stamp = rospy.Time.now()
                        mot_tracker.pubb.publish(r)
                    except CvBridgeError as e:
                        pass      

              
         
                    
                
                mot_tracker.img_in = 0   
                mot_tracker.bbox_checkin=0
                
             
                mot_tracker.rate.sleep()
               
        except (rospy.ROSInterruptException, SystemExit, KeyboardInterrupt):
            sys.exit(0)
#        except:
#            pass