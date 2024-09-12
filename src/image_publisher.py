#!/usr/bin/env python

import rospy
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import struct

def read_mnist_images(file_path):
    with open(file_path, 'rb') as f:
        magic, num, rows, cols = struct.unpack('>IIII', f.read(16))
        images = np.fromfile(f, dtype=np.uint8).reshape(num, rows, cols)
    return images

def read_mnist_labels(file_path):
    with open(file_path, 'rb') as f:
        magic, num = struct.unpack('>II', f.read(8))
        labels = np.fromfile(f, dtype=np.uint8)
    return labels

def publish_images(images, labels, rate):
    rospy.init_node('image_publisher', anonymous=True)
    image_pub = rospy.Publisher('mnist_images', Image, queue_size=10)
    bridge = CvBridge()
    rate = rospy.Rate(rate)
    
    while not rospy.is_shutdown():
        for img, label in zip(images, labels):
            if rospy.is_shutdown():
                break
            ros_image = bridge.cv2_to_imgmsg(img, "mono8")
            image_pub.publish(ros_image)
            rospy.loginfo(f'Published an MNIST image with label: {label}')
            rate.sleep()

if __name__ == '__main__':
    try:
        mnist_image_file = '/home/swaltmann/mirte_ws/src/mirte-ros-packages/mirte_efficient_capsnet/src/data/t10k-images-idx3-ubyte/t10k-images-idx3-ubyte'
        mnist_label_file = '/home/swaltmann/mirte_ws/src/mirte-ros-packages/mirte_efficient_capsnet/src/data/t10k-labels-idx1-ubyte/t10k-labels-idx1-ubyte'
        publish_rate = 0.5  # Adjust as needed (images per second)
        images = read_mnist_images(mnist_image_file)
        labels = read_mnist_labels(mnist_label_file)
        publish_images(images, labels, publish_rate)
    except rospy.ROSInterruptException:
        pass