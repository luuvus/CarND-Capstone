from styx_msgs.msg import TrafficLight
import tensorflow as tf
from label_image import read_tensor_from_image
from label_image import load_labels
import rospy
import os
import numpy as np
import time
import cv2

class TLClassifier(object):
    def __init__(self):
        #TODO load classifier
        #rospy.loginfo('cwd:%s',os.getcwd())
        self.model_file = "light_classification/retrained_mobilenet_1.0_224_005_no_img_mod.pb"
        self.graph = tf.Graph()
        self.graph_def = tf.GraphDef()
        self.labels = load_labels("light_classification/retrained_labels.txt")


        with open(self.model_file, "rb") as f:
            self.graph_def.ParseFromString(f.read())
        with self.graph.as_default():
            tf.import_graph_def(self.graph_def)


        

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        #TODO implement light color prediction
        input_layer = "input"
        output_layer = "final_result"
        input_name = "import/" + input_layer
        output_name = "import/" + output_layer
        input_operation = self.graph.get_operation_by_name(input_name)
        output_operation = self.graph.get_operation_by_name(output_name)

        #start = time.time()
        #t = read_tensor_from_image(image, 224, 224, 128, 128)
        image2 = cv2.resize(image,dsize=(224,224), interpolation=cv2.INTER_CUBIC)
        np_image_data = np.asarray(image2,dtype=np.float32)
        np_image_data = np.divide(np.subtract(np_image_data,128),128)
        image_data = np.expand_dims(np_image_data,axis=0)
        #end = time.time();
        #print("Time1: {:.3f}s".format(end-start))

        #start = time.time()
        with tf.Session(graph=self.graph) as sess:
            results = sess.run(output_operation.outputs[0], {input_operation.outputs[0]: image_data})

        #end = time.time();
        #print("Time2: {:.3f}s".format(end-start))


        results = np.squeeze(results)
        
        prediction = np.argmax(results)
        #rospy.loginfo("prediction: %s",self.labels[prediction])
        #os.system('python -m light_classification/label_image --graph=light_classification/retrained_mobilenet_1.0_224_005_no_img_mod.pb --image=test.png')
        
        if self.labels[prediction] == "green":
            return TrafficLight.GREEN
        elif self.labels[prediction] == "red":
            return TrafficLight.RED
        elif self.labels[prediction] == "yellow":
            return TrafficLight.YELLOW
        else:
            return TrafficLight.UNKNOWN
