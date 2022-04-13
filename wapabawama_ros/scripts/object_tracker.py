#!/usr/bin/env python

# import the necessary packages
import numpy as np
from scipy.spatial import distance as dist
import cv2
import rospy
from sensor_msgs.msg import Image
from darknet_ros_msgs.msg import BoundingBoxes
from cv_bridge import CvBridge, CvBridgeError
from collections import OrderedDict
import sys
# construct the argument parse and parse the arguments
class CentroidTracker():
    def __init__(self, maxDisappeared=1):
        # initialize the next unique object ID along with two ordered
        # dictionaries used to keep track of mapping a given object
        # ID to its centroid and number of consecutive frames it has
        # been marked as "disappeared", respectively
        rospy.init_node('ct', anonymous=True)
        display = rospy.get_param("~display", True)
        bbox_topic = rospy.get_param("~bbox", "/right_camera/darknet_ros/bounding_boxes")
        tracked_bbox_topic = rospy.get_param("~tracked_bbox", "/tracked_boxes")
        self.subb = rospy.Subscriber(bbox_topic, BoundingBoxes, self.boxcallback)
        self.pubb = rospy.Publisher(tracked_bbox_topic, BoundingBoxes, queue_size=50)
        self.rate = rospy.Rate(10)
        self.nextObjectID = 0
        self.objects = OrderedDict()
        self.disappeared = OrderedDict()
        self.bbox_checkin = 0
        self.img_in = 0
        # store the number of maximum consecutive frames a given
        # object is allowed to be marked as "disappeared" until we
        # need to deregister the object from tracking
        self.maxDisappeared = maxDisappeared
        self.bridge = CvBridge()
        if display:
            img_topic = rospy.get_param('~img_topic', '/camera/image_raw')
            self.window_name = rospy.get_param('~window_name', 'image')
            self.display = display
            self.subimage = rospy.Subscriber(img_topic, Image, self.imgcallback)
            self.pubimage = rospy.Publisher('tracked_image', Image, queue_size=20)
        
    def imgcallback(self, msg):
        
        if self.display:
            try : 
                self.img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
                
            except CvBridgeError as e:
                pass
        self.img_in = 1
        return

    def boxcallback(self, msg):
       
        self.rects = []
        for i in range(len(msg.bounding_boxes)):
            bbox = msg.bounding_boxes[i]
            box =np.array([bbox.xmin, bbox.ymin, bbox.xmax, bbox.ymax])
            self.rects.append(box)
        # print(self.rects)
        
        self.bbox_checkin = 1
        return

    def register(self, centroid):
        # when registering an object we use the next available object
        # ID to store the centroid
        self.objects[self.nextObjectID] = centroid
        self.disappeared[self.nextObjectID] = 0
        self.nextObjectID += 1

    def deregister(self, objectID):
        # to deregister an object ID we delete the object ID from
        # both of our respective dictionaries
        del self.objects[objectID]
        del self.disappeared[objectID]

    def update(self, rects=np.empty((1,4))):
        # check to see if the list of input bounding box rectangles
        # is empty
        if len(rects) == 0:
            # loop over any existing tracked objects and mark them
            # as disappeared
            for objectID in self.disappeared.keys():
                self.disappeared[objectID] += 1

                # if we have reached a maximum number of consecutive
                # frames where a given object has been marked as
                # missing, deregister it
                if self.disappeared[objectID] > self.maxDisappeared:
                    self.deregister(objectID)

            # return early as there are no centroids or tracking info
            # to update
            return self.objects

        # initialize an array of input centroids for the current frame
        inputCentroids = np.zeros((len(rects), 2), dtype="int")

        # loop over the bounding box rectangles
        for (i, (startX, startY, endX, endY)) in enumerate(rects):
            # use the bounding box coordinates to derive the centroid
            cX = int((startX + endX) / 2.0)
            cY = int((startY + endY) / 2.0)
            inputCentroids[i] = (cX, cY)
            

        # if we are currently not tracking any objects take the input
        # centroids and register each of them
        if len(self.objects) == 0:
            for i in range(0, len(inputCentroids)):
                self.register(inputCentroids[i])

        # otherwise, are are currently tracking objects so we need to
        # try to match the input centroids to existing object
        # centroids
        else:
            # grab the set of object IDs and corresponding centroids
            objectIDs = list(self.objects.keys())
            objectCentroids = list(self.objects.values())

            # compute the distance between each pair of object
            # centroids and input centroids, respectively -- our
            # goal will be to match an input centroid to an existing
            # object centroid
            D = dist.cdist(np.array(objectCentroids), inputCentroids)

            # in order to perform this matching we must (1) find the
            # smallest value in each row and then (2) sort the row
            # indexes based on their minimum values so that the row
            # with the smallest value as at the *front* of the index
            # list
            rows = D.min(axis=1).argsort()

            # next, we perform a similar process on the columns by
            # finding the smallest value in each column and then
            # sorting using the previously computed row index list
            cols = D.argmin(axis=1)[rows]

            # in order to determine if we need to update, register,
            # or deregister an object we need to keep track of which
            # of the rows and column indexes we have already examined
            usedRows = set()
            usedCols = set()

            # loop over the combination of the (row, column) index
            # tuples
            for (row, col) in zip(rows, cols):
                # if we have already examined either the row or
                # column value before, ignore it
                # val
                if row in usedRows or col in usedCols:
                    continue

                # otherwise, grab the object ID for the current row,
                # set its new centroid, and reset the disappeared
                # counter
                objectID = objectIDs[row]
                self.objects[objectID] = inputCentroids[col]
                self.disappeared[objectID] = 0

                # indicate that we have examined each of the row and
                # column indexes, respectively
                usedRows.add(row)
                usedCols.add(col)

            # compute both the row and column index we have NOT yet
            # examined
            unusedRows = set(range(0, D.shape[0])).difference(usedRows)
            unusedCols = set(range(0, D.shape[1])).difference(usedCols)

            # in the event that the number of object centroids is
            # equal or greater than the number of input centroids
            # we need to check and see if some of these objects have
            # potentially disappeared
            if D.shape[0] >= D.shape[1]:
                # loop over the unused row indexes
                for row in unusedRows:
                    # grab the object ID for the corresponding row
                    # index and increment the disappeared counter
                    objectID = objectIDs[row]
                    self.disappeared[objectID] += 1

                    # check to see if the number of consecutive
                    # frames the object has been marked "disappeared"
                    # for warrants deregistering the object
                    if self.disappeared[objectID] > self.maxDisappeared:
                        self.deregister(objectID)

            # otherwise, if the number of input centroids is greater
            # than the number of existing object centroids we need to
            # register each new input centroid as a trackable object
            else:
                for col in unusedCols:
                    self.register(inputCentroids[col])

        # return the set of trackable objects
        return self.objects
if __name__ == '__main__':
    # initialize our centroid tracker and frame dimensions
    ct = CentroidTracker()


    # loop over the frames from the video stream
    while True:

        # update our centroid tracker using the computed set of bounding
        # box rectangles
        try:
            if ct.bbox_checkin==1:
                objects = ct.update(ct.rects)
            else:
                objects = ct.update(np.empty((0,4)))
            # # loop over the tracked objects
            for (objectID, centroid) in objects.items():
                # draw both the ID of the object and the centroid of the
                # object on the output frame
                text = "ID {}".format(objectID)
                cv2.rectangle(ct.img, (centroid[0]-80,centroid[1]-80), (centroid[0]+80,centroid[1]+80), (0, 255, 0), 6)
                cv2.putText(ct.img, text, (centroid[0] - 10, centroid[1] - 10),cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 255, 0), 5)
                cv2.circle(ct.img, (centroid[0], centroid[1]), 4, (0, 255, 0), -1)
                # print(text)
            if ct.img_in==1 and ct.display:    
                cv2.namedWindow("right",0)
                cv2.resizeWindow("right",1920/2,1080/2)
                # ct.image = ct.bridge.cv2_to_imgmsg(ct.img, "bgr8")
                # ct.image.header.stamp = rospy.Time.now()
                # ct.pubimage.publish(ct.image)
                # show the output frame
                cv2.imshow("right", ct.img)
                cv2.waitKey(3) 
            ct.rate.sleep()
        except (rospy.ROSInterruptException, SystemExit, KeyboardInterrupt):
            sys.exit(0)
cv2.destroyAllWindows()
    # do a bit of cleanup
    
