#!/usr/bin/env python3

import rospy
import math
import cv2 as cv # OpenCV2
import numpy as np
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point
import copy
import random


class Node:
    def __init__(self, x, y, idx):

        # Index of the node in the graph
        self.idx = idx

        # Position of node
        self.x = x
        self.y = y

        # Neighbouring edges
        self.neighbours = [] # List of nodes
        self.neighbour_costs = [] # the ith neighbour in self.neighbours has an edge cost defined as the ith element in self.neighbour_costs

        # Random walk variables
        self.random_walk_visited = False

    def distance_to(self, other_node):
        return math.sqrt((self.x-other_node.x)**2 + (self.y-other_node.y)**2)

    def is_connected(self, img, other_node):
        p1 = [self.x, self.y]
        p2 = [other_node.x, other_node.y]
        return not is_occluded(img, p1, p2)

class Graph:
    def __init__(self, map):

        self.map_ = map

        self.nodes_ = []

        self.grid_step_size_ = rospy.get_param("~grid_step_size") # Grid spacing

        # Publishers
        self.path_pub_ = rospy.Publisher('/path_planner/plan', Path, queue_size=1)

        # Visualisation Marker (you can ignore this)
        self.marker_nodes_ = Marker()
        self.marker_nodes_.header.frame_id = "map"
        self.marker_nodes_.ns = "nodes"
        self.marker_nodes_.id = 0
        self.marker_nodes_.type = Marker.POINTS
        self.marker_nodes_.action = Marker.ADD
        self.marker_nodes_.pose.position.x = 0.0
        self.marker_nodes_.pose.position.y = 0.0
        self.marker_nodes_.pose.position.z = 0.0
        self.marker_nodes_.pose.orientation.x = 0.0
        self.marker_nodes_.pose.orientation.y = 0.0
        self.marker_nodes_.pose.orientation.z = 0.0
        self.marker_nodes_.pose.orientation.w = 1.0
        self.marker_nodes_.scale.x = .03
        self.marker_nodes_.scale.y = .03
        self.marker_nodes_.scale.z = .03
        self.marker_nodes_.color.a = 1.0
        self.marker_nodes_.color.r = 1.0
        self.marker_nodes_.color.g = 0.2
        self.marker_nodes_.color.b = 0.2

        self.marker_start_ = Marker()
        self.marker_start_.header.frame_id = "map"
        self.marker_start_.ns = "start"
        self.marker_start_.id = 0
        self.marker_start_.type = Marker.POINTS
        self.marker_start_.action = Marker.ADD
        self.marker_start_.pose.position.x = 0.0
        self.marker_start_.pose.position.y = 0.0
        self.marker_start_.pose.position.z = 0.0
        self.marker_start_.pose.orientation.x = 0.0
        self.marker_start_.pose.orientation.y = 0.0
        self.marker_start_.pose.orientation.z = 0.0
        self.marker_start_.pose.orientation.w = 1.0
        self.marker_start_.scale.x = .08
        self.marker_start_.scale.y = .08
        self.marker_start_.scale.z = .08
        self.marker_start_.color.a = 1.0
        self.marker_start_.color.r = 1.0
        self.marker_start_.color.g = 1.0
        self.marker_start_.color.b = 0.2

        self.marker_visited_ = Marker()
        self.marker_visited_.header.frame_id = "map"
        self.marker_visited_.ns = "visited"
        self.marker_visited_.id = 0
        self.marker_visited_.type = Marker.POINTS
        self.marker_visited_.action = Marker.ADD
        self.marker_visited_.pose.position.x = 0.0
        self.marker_visited_.pose.position.y = 0.0
        self.marker_visited_.pose.position.z = 0.0
        self.marker_visited_.pose.orientation.x = 0.0
        self.marker_visited_.pose.orientation.y = 0.0
        self.marker_visited_.pose.orientation.z = 0.0
        self.marker_visited_.pose.orientation.w = 1.0
        self.marker_visited_.scale.x = .05
        self.marker_visited_.scale.y = .05
        self.marker_visited_.scale.z = .05
        self.marker_visited_.color.a = 1.0
        self.marker_visited_.color.r = 0.2
        self.marker_visited_.color.g = 0.2
        self.marker_visited_.color.b = 1.0

        self.marker_unvisited_ = Marker()
        self.marker_unvisited_.header.frame_id = "map"
        self.marker_unvisited_.ns = "unvisited"
        self.marker_unvisited_.id = 0
        self.marker_unvisited_.type = Marker.POINTS
        self.marker_unvisited_.action = Marker.ADD
        self.marker_unvisited_.pose.position.x = 0.0
        self.marker_unvisited_.pose.position.y = 0.0
        self.marker_unvisited_.pose.position.z = 0.0
        self.marker_unvisited_.pose.orientation.x = 0.0
        self.marker_unvisited_.pose.orientation.y = 0.0
        self.marker_unvisited_.pose.orientation.z = 0.0
        self.marker_unvisited_.pose.orientation.w = 1.0
        self.marker_unvisited_.scale.x = .06
        self.marker_unvisited_.scale.y = .06
        self.marker_unvisited_.scale.z = .06
        self.marker_unvisited_.color.a = 1.0
        self.marker_unvisited_.color.r = 0.3
        self.marker_unvisited_.color.g = 1.0
        self.marker_unvisited_.color.b = 0.3
        
        self.marker_edges_ = Marker()
        self.marker_edges_.header.frame_id = "map"
        self.marker_edges_.ns = "edges"
        self.marker_edges_.id = 0
        self.marker_edges_.type = Marker.LINE_LIST
        self.marker_edges_.action = Marker.ADD
        self.marker_edges_.pose.position.x = 0.0
        self.marker_edges_.pose.position.y = 0.0
        self.marker_edges_.pose.position.z = 0.0
        self.marker_edges_.pose.orientation.x = 0.0
        self.marker_edges_.pose.orientation.y = 0.0
        self.marker_edges_.pose.orientation.z = 0.0
        self.marker_edges_.pose.orientation.w = 1.0
        self.marker_edges_.scale.x = 0.008
        self.marker_edges_.scale.y = 0.008
        self.marker_edges_.scale.z = 0.008
        self.marker_edges_.color.a = 1.0
        self.marker_edges_.color.r = 1.0
        self.marker_edges_.color.g = 1.0
        self.marker_edges_.color.b = 0.4
        
        self.marker_pub_ = rospy.Publisher('marker', Marker, queue_size=1)
        
        # Create a grid
        self.create_grid()
        
        self.visualise_graph()

    def create_grid(self):

        # Create nodes
        idx = 0
        for x in range(self.map_.min_x_, self.map_.max_x_-1, self.grid_step_size_):
            for y in range(self.map_.min_y_, self.map_.max_y_-1, self.grid_step_size_):

                if rospy.is_shutdown():
                    return

                # Check if it is occupied
                occupied = self.map_.is_occupied(x,y)

                # Create the node
                if not occupied:
                    self.nodes_.append(Node(x,y,idx))
                    idx = idx + 1

        # Create edges
        count = 0
        # distance_threshold = math.sqrt(2*(self.grid_step_size_*1.01)**2) # Chosen so that diagonals are connected, but not 2 steps away
        distance_threshold = self.grid_step_size_*1.01 # only 4 connected
        for node_i in self.nodes_:

            # Debug print status
            count = count + 1
            print(count, "of", len(self.nodes_))
            if rospy.is_shutdown():
                return

            for node_j in self.nodes_:

                # Don't create edges to itself
                if node_i != node_j:

                    # Check if the nodes are close to each other
                    distance = node_i.distance_to(node_j)
                    if distance < distance_threshold:

                        # Check edge is collision free
                        if node_i.is_connected(self.map_.image_, node_j):

                            # Create the edge
                            node_i.neighbours.append(node_j)
                            node_i.neighbour_costs.append(distance)

    def visualise_graph(self):
        # Create and publish visualisation markers for the graph

        rospy.sleep(0.5)

        self.marker_nodes_.points = []
        for node_i in self.nodes_:
            p = self.map_.pixel_to_world(node_i.x, node_i.y)
            point = Point(p[0], p[1], 0.01)
            self.marker_nodes_.points.append(point)
        self.marker_pub_.publish(self.marker_nodes_)

        rospy.sleep(0.5)

        self.marker_edges_.points = []
        for node_i in self.nodes_:
            for node_j in node_i.neighbours:
                p = self.map_.pixel_to_world(node_i.x, node_i.y)
                point = Point(p[0], p[1], 0)
                self.marker_edges_.points.append(point)
                p = self.map_.pixel_to_world(node_j.x, node_j.y)
                point = Point(p[0], p[1], 0)
                self.marker_edges_.points.append(point)
        self.marker_pub_.publish(self.marker_edges_)

        rospy.sleep(0.5)

    def visualise_path(self, path):
        msg = Path()
        msg.header.frame_id = 'map'
        for node in path:
            p = self.map_.pixel_to_world(node.x, node.y)
            pose = PoseStamped()
            pose.pose.position.x = p[0]
            pose.pose.position.y = p[1]
            pose.pose.position.z = 0.1
            pose.pose.orientation.w = 1.0
            pose.header.frame_id = 'map'
            msg.poses.append(pose)
        self.path_pub_.publish(msg)



class Map:
    def __init__(self):

        # Extract the image from a file
        filename = rospy.get_param('~filename')
        self.image_ = cv.imread(filename, cv.COLOR_BGR2GRAY)

        shape = self.image_.shape
        self.min_x_ = 0
        self.min_y_ = 0
        self.max_x_ = shape[0]
        self.max_y_ = shape[1]

        if len(shape) == 3:
            self.image_ = self.image_[:,:,0]

        # Rviz subscriber
        self.rviz_goal_sub_ = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.rviz_goal_callback, queue_size=1)
        self.rviz_goal = None

    def pixel_to_world(self, x, y):
        resolution = 0.01
        return [y*resolution, (self.max_x_-x)*resolution]

    def world_to_pixel(self, x, y):
        resolution = 0.01
        return [self.max_x_-(y/resolution), x/resolution]

    def rviz_goal_callback(self, msg):
        goal = self.world_to_pixel(msg.pose.position.x, msg.pose.position.y)
        self.rviz_goal = goal # Save it into global variable
        print("New goal received from rviz!")
        print(self.rviz_goal)

    def is_occupied(self, x, y):

        shape = self.image_.shape

        # Out of bounds
        if x < 0 or x >= shape[0] or y < 0 or y >= shape[1]:
            return True

        if self.image_[x,y] > 235:
            return False
        else:
            return True

def is_occluded(img, p1, p2, threshold=235):
    # Draws a line from p1 to p2
    # Stops at the first pixel that is a "hit", i.e. above the threshold
    # Returns True if a hit is found, False otherwise

    # Extract the vector
    x1 = float(p1[0])
    y1 = float(p1[1])
    x2 = float(p2[0])
    y2 = float(p2[1])

    step = 1.0

    dx = x2 - x1
    dy = y2 - y1
    l = math.sqrt(dx**2. + dy**2.)
    if l == 0:
        return False
    dx = dx / l
    dy = dy / l

    max_steps = int(l / step)

    for i in range(max_steps):

        # Get the next pixel
        x = int(round(x1 + dx*i))
        y = int(round(y1 + dy*i))

        # Check if it's outside the image
        if x < 0 or x >= img.shape[0] or y < 0 or y >= img.shape[1]:
            return False

        # Check for "hit"
        if img[x, y] <= threshold:
            return True

    # No hits found
    return False


class RandomWalk:
    def __init__(self, graph):
        self.graph_ = graph

        self.start_node_idx_ = rospy.get_param("~start_node_idx") # Start node
        self.random_walk_length_ = rospy.get_param("~random_walk_length")

        self.path_ = self.random_walk()

        self.graph_.visualise_path(self.path_)

    def random_walk(self):
        # Do a random walk 

        # Begin with an empty path
        path = []

        # Empty the "node.random_walk_visited" variables
        for n in self.graph_.nodes_:
            n.random_walk_visited = False

        # Add start to the path
        current = self.graph_.nodes_[self.start_node_idx_]
        path.append(current)
        # current.random_walk_visited = True

        # Loop for self.random_walk_length_ steps
        for i in range(self.random_walk_length_):

            # Stop if you hit "Ctrl+C"
            if rospy.is_shutdown():
                return

            ############
            ## STEP 1 ##
            ############
            # Extract the neighbours of the current node
            
            # neighbours = ??
            neighbours = [neighbour for neighbour in current.neighbours if not neighbour.random_walk_visited]

            # If there are no unvisited neighbours, break out of the loop
            if len(neighbours) == 0:
                neighbours = current.neighbours

            ############
            ## STEP 2 ##
            ############
            # Pick one of the neighbours randomly
            # HINT: use random.randrange()
            
            # neighbour_idx = ??
            neighbour_idx = random.randrange(len(neighbours))
            next_node = neighbours[neighbour_idx]

            ############
            ## STEP 3 ##
            ############
            # Move to the selected neighbour and add it to the path
            # Hint: the solution will be similar to how the start_node was added to the path
            
            # ??
            next_node.random_walk_visited = True
            path.append(next_node)

            # Set the current node to the selected neighbour
            current = next_node

            # Plot the current path in rviz
            self.graph_.visualise_path(path)

            # Sleep for a bit
            rospy.sleep(0.005)

        return path



if __name__ == '__main__':
    # Create the ROS node
    rospy.init_node('random_walk')

    # Sleep for a bit so rviz has time to open
    rospy.sleep(2.0)

    # Create a map from image
    map = Map()

    # Create a graph from the map
    graph = Graph(map)

    # Do a random walk
    random_walk = RandomWalk(graph)

    # Repeat indefinitely
    while not rospy.is_shutdown():

        # Sleep for 1 second
        rospy.sleep(1.0)

        # Do a random walk
        random_walk = RandomWalk(graph)


    # Loop forever while processing callbacks
    rospy.spin()
