#!/usr/bin/env python3

import rospy
import math
from std_msgs.msg import Float32
import random


class Location:
    def __init__(self, x, y):

        # Save the input arguments as object variables
        self.x = x
        self.y = y

class WeightedAverager:
    def __init__(self, origin, landmarks):

        # Save the input arguments as object variables
        self.origin = origin
        self.landmarks = landmarks

        # Define parameters
        threshold = 10.0 # outliers beyond this distance should be ignored

        # Setup ROS publisher
        self.average_pub = rospy.Publisher('average', Float32, queue_size=1)

        # Call the averaging function
        answer = self.weighted_average(threshold)

        msg = Float32()

        msg.data = answer
        self.average_pub.publish(msg)

        print("Weighted Average Distance: ", answer)

    def weighted_average(self, threshold):
        
        weights = [random.random() for _ in range(len(self.landmarks))]
        total_weight = sum(weights)
        weights = [w / total_weight for w in weights]

        weighted_sum = 0
        for loc,weight in zip(self.landmarks,weights):
            distance = math.sqrt(math.pow(loc.x - self.origin.x, 2) + math.pow(loc.y - self.origin.y, 2))
            
            if distance > threshold:
                continue

            weighted_sum += distance * weight
            
        return weighted_sum # Change this return value later


if __name__ == '__main__':
    # Create the ROS node
    rospy.init_node('python_tutorial')

    # Print message
    print('Finished setting up python_tutorial ROS node.')

    # Setup the data
    origin = Location(5.0, 5.0)
    landmarks = [Location(6.0, 7.0), Location(5.1, 4.9), Location(15.0, 20.0), Location(8.0, 0.0), Location(-3.0, 2.0), Location(-10.0, -10.0), Location(5.0, 5.0), Location(0.0, 0.0)]

    # for loc in landmarks:
    #     distance = math.sqrt(math.pow(loc.x - origin.x, 2) + math.pow(loc.y - origin.y, 2))
    #     print("Current Distance: ", distance)

    # Create the WeightedAverager object
    # weighted_averager = WeightedAverager(origin, landmarks)
    rate = rospy.Rate(2)
    while not rospy.is_shutdown():
        weighted_averager = WeightedAverager(origin, landmarks)
        rate.sleep()

    # Loop forever
    rospy.spin()
