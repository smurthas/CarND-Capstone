from styx_msgs.msg import TrafficLight

from keras.models import load_model
from keras.preprocessing.image import img_to_array, load_img
import tensorflow as tensorflow
import cv2
import numpy as np

import rospy

class TLClassifier(object):
    def __init__(self):
        #DONE load classifier
        # Cv2 Haar cascade for TL detection
        self.cascade = cv2.CascadeClassifier('./cascade_gen.xml')
        self.test_model = load_model('./models/tl_model.h5')
        self.graph = tensorflow.get_default_graph()

    #Faster Non-Maximum Suppression
    # From http://www.pyimagesearch.com/2015/02/16/faster-non-maximum-suppression-python/
    # Malisiewicz et al.
    def non_max_suppression_fast(self, boxes, overlapThresh):
        if len(boxes) == 0:
            return []
            boxes = np.array(boxes)
        if boxes.dtype.kind == "i":
            boxes = boxes.astype("float")
        pick = []
        x1 = boxes[:,0]
        y1 = boxes[:,1]
        x2 = x1+boxes[:,2]
        y2 = y1+boxes[:,3]
        area = (x2 - x1 + 1) * (y2 - y1 + 1)
        idxs = np.argsort(y1)
        while len(idxs) > 0:
            last = len(idxs) - 1
            i = idxs[last]
            pick.append(i)
            xx1 = np.maximum(x1[i], x1[idxs[:last]])
            yy1 = np.maximum(y1[i], y1[idxs[:last]])
            xx2 = np.minimum(x2[i], x2[idxs[:last]])
            yy2 = np.minimum(y2[i], y2[idxs[:last]])
            w = np.maximum(0, xx2 - xx1 + 1)
            h = np.maximum(0, yy2 - yy1 + 1)
            overlap = (w * h) / area[idxs[:last]]
            idxs = np.delete(idxs, np.concatenate(([last],
                np.where(overlap > overlapThresh)[0])))
        return boxes[pick].astype("int")

    def get_classification(self, image):
        """Determines the color of the traffic light in the image
        Args:
            image (cv::Mat): image containing the traffic light
        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)
        """
        #DONE implement light color prediction

        box = self.cascade.detectMultiScale(image, 1.3, 3)
        box = self.non_max_suppression_fast(box, 0.2)
        state = TrafficLight.UNKNOWN
        img_width, img_height = 150, 150
        for (x,y,w,h) in box:
            # FP filter
            dh=int(round(h*0.1))
            line = image[(y+dh):(y+h-dh),int(round(x+w/2)),:]
            if np.std(line) < 32:
                rospy.loginfo("False Detection!")
                continue # FP detection
            tl_img = image[y:(y + h), x:(x + w)]
            tl_img_rgb = cv2.resize(tl_img, (img_width, img_height))
            tl_img_rgb = cv2.cvtColor(tl_img_rgb , cv2.COLOR_BGR2RGB)
            tl_img_data = img_to_array(tl_img_rgb)
            tl_img_data = np.expand_dims(tl_img_data, axis=0)
            with self.graph.as_default():
                predictedclass = self.test_model.predict_classes(tl_img_data, verbose=False)

            if int(predictedclass) == 2:
                state = TrafficLight.YELLOW
                rospy.loginfo("Yellow Light")
                continue
            elif int(predictedclass) == 1:
                state = TrafficLight.GREEN
                rospy.loginfo("Green light")
                continue
            elif int(predictedclass) == 3:
                state = TrafficLight.RED
                rospy.loginfo("Red Light")
                break  # Red has high priority, so, return it if it is seen
            else:
                continue
        return state
