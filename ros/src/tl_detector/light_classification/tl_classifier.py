from styx_msgs.msg import TrafficLight
import tensorflow as tf

class TLClassifier(object):
    def __init__(self):
        #TODO load classifier
        pass

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        #TODO implement light color prediction
        
        #Format the image for the Mul:0 Tensor
        image2 = cv2.resize(img2,dsize=(299,299), interpolation = cv2.INTER_CUBIC)
        #Numpy array
        np_image_data = np.asarray(image2)
        image_data = np.expand_dims(np_image_data,axis=0)
    

        # Loads label file, strips off carriage return
        label_lines = [line.rstrip() for line in tf.gfile.GFile("tl_labels.txt")]

        # Load model (Retrained Google Inception Model)
        with tf.gfile.FastGFile("tl_graph.pb", 'rb') as f:
            graph_def = tf.GraphDef()
            graph_def.ParseFromString(f.read())
            _ = tf.import_graph_def(graph_def, name='')

        with tf.Session() as sess:
            # Feed the image_data as input to the graph and get first prediction
            softmax_tensor = sess.graph.get_tensor_by_name('final_result:0')
    
            predictions = sess.run(softmax_tensor, {'Mul:0': image_data})
        
            # Sort to show labels of first prediction in order of confidence
            top_k = predictions[0].argsort()[-len(predictions[0]):][::-1]
        
            # check if the light is red
            light_state = label_lines[top_k[0]]
            if light_state == "red":
                return TrafficLight.RED
    
        # if the light is not red
        return -1
