#!/usr/bin/env python3

import rospy
import tensorflow as tf
from std_msgs.msg import Int32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np

from utils import Dataset, plotImages, plotWrongImages
from models import EfficientCapsNet

# Initialize the ROS node
rospy.init_node('capsnet_node', anonymous=True)

# ROS topic names
mnist_topic = "mnist_images"
result_topic = "capsnet_result"

# TensorFlow GPU configuration
gpus = tf.config.experimental.list_physical_devices('GPU')
if gpus:
    try:
        tf.config.experimental.set_visible_devices(gpus[0], 'GPU')
        tf.config.experimental.set_memory_growth(gpus[0], True)
    except RuntimeError as e:
        print(e)



# Model and dataset initialization
model_name = 'MNIST'
custom_path = None  # Change if you have a custom trained model
model_test = EfficientCapsNet(model_name, mode='test', verbose=True, custom_path=custom_path)
model_test.load_graph_weights()

bridge = CvBridge()

def mnist_callback(msg):
    try:
        # Convert ROS Image message to OpenCV image
        cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        # Preprocess the image as required by your model
        image = cv_image.reshape(1, 28, 28, 1)  # Assuming MNIST image size and single channel
        image = image / 255.0  # Normalize the image if required

        # Perform prediction
        y_pred = model_test.predict(image)[0]

        # Get the predicted number
        predicted_number = np.argmax(y_pred)

        # Publish the result
        result_pub.publish(predicted_number)
        
        # Print the result to the terminal
        rospy.loginfo(f"Predicted number: {predicted_number}")

    except Exception as e:
        rospy.logerr(f"Error processing MNIST image: {e}")

# ROS publisher and subscriber
result_pub = rospy.Publisher(result_topic, Int32, queue_size=10)
rospy.Subscriber(mnist_topic, Image, mnist_callback)

# Keep the node running
rospy.spin()
