#!/usr/bin/env python

import rospy
import actionlib
# from composit_planner.srv import *
# from composit_planner.msg import *
from nav_msgs.msg import Path, OccupancyGrid
from geometry_msgs.msg import *
from tf.transformations import quaternion_from_euler, euler_from_quaternion
import time
from collections import defaultdict
import random
import scipy.ndimage
import numpy

# create a class that would send waypoints to the car
class navigation_leader:

	#initialize class
	def __init__(self):
		# class variables
		self.flag = False

		# variables for checking if reached end of navigation
		self.current_x = 0.0 # in world frame
		self.current_y = 0.0
		self.roll = None
		self.pitch = None
		self.yaw = None


		self.goal_x = 0.0
		self.goal_y = 0.0
		self.reached_goal = True

		# self.current_path_index = -1;
		# paths lists, only for testing
		self.paths = [] # [[Point32(1,0,0),Point32(2, 0, 0)], [Point32(2.5, 0.5, 0.0), Point32(3.0, 0.5, 0.0)]]

		# variables for map
		self.map_origin_x = 0.0 # in the world frame
		self.map_origin_y = 0.0
		self.map_resolution = 0.1
		self.map_grid = [] # holds a list of occupancy probabilities. 
					  # inverted row major, starting at bottom left
					  # when looked at from world frame
		self.map_width = 0
		self.map_height = 0

		# filtered map data
		self.filtered_map_grid = [] # stores the map grid list after a gaussian filter



		#init ros
		rospy.init_node('car_nav')




		#publish to trajectory topics
		self.path_pub = rospy.Publisher("/trajectory/current", 
			PolygonStamped,
			queue_size=1)

		#subscribe to position topics.
		# move one meter forward when we get new pose. do it once ####
		self.pose_sub = rospy.Subscriber("/pose_body", 
			PoseStamped, 
			self.pose_cb,
			queue_size=1)

		# subscribe to the map topic - to get the map itself and the information about its origin
		self.map_sub = rospy.Subscriber("/projected_map",
			OccupancyGrid,
			self.map_cb,
			queue_size = 1)

		# publisher for the filtered map we create (to stay away from obstacles)

		self.filter_pub = rospy.Publisher('/gaussian_map', OccupancyGrid, queue_size=1)




		#while not shut down, keep this node spinning
		while not rospy.is_shutdown():
			rospy.spin()

	def follow_path(self, path_list):
	    # path list is a list of Point32 objects -- which are the waypoints to be followed
	    print('#####sending a sequence')
	    print path_list
	    
	    # store the last goal as the goal we are going to 
	    # self.goal_x = path_list[-1].x
	    # self.goal_y = path_list[-1].y

	    # send the sequence
	    #create a message
	    msg = PolygonStamped()
	    msg.header.stamp = rospy.Time(0)
	    msg.header.frame_id = 'world'
	    msg.polygon.points = path_list

	    #publish this path
	    self.path_pub.publish(msg)


    

	def pose_cb(self, pose_msg):
	    # this callback will handle the following:
            # * calling functions that publish paths
            # * checking if navigation has been completed
            # (will toggle the goal_reached bool variable)

            # update current location in the world frame
			self.current_x = pose_msg.pose.position.x
			self.current_y = pose_msg.pose.position.y

			# calculate the euler pose values from the quaternion
			quaternion = (
				pose_msg.pose.orientation.x,
				pose_msg.pose.orientation.y,
				pose_msg.pose.orientation.z,
				pose_msg.pose.orientation.w)
			euler = euler_from_quaternion(quaternion)
			self.roll = euler[0]
			self.pitch = euler[1]
			self.yaw = euler[2]

			print "yaw= ", self.yaw/3.1415*180

			# if this is the first time and we have enough data, generate a dijkstra path to a set point
			
			if self.map_grid != [] and self.paths == [] :
				print("constructing path")
				pixel_path = None
				while pixel_path == None:
					# print("int the loop")
					# go to (2, 0.5, 0)
					end_coord =  self.get_free_random_coord(); #(2,0.2)

					# first step is to convet start and end coord to pixels
					start_pixel = self.world_to_pixel(self.current_x, self.current_y)
					end_pixel = self.world_to_pixel(end_coord[0],end_coord[1])
					# get the pixel path from dijkstra

					# pixel_path = self.dijkstra_to_goal(self.weighted_graph(), start_pixel, end_pixel)
					# pixel_path = self.BFS(start_pixel, end_pixel)
					pixel_path =  self.a_star(self.filtered_map_grid, start_pixel, end_pixel)

				# convert to a follow-able path (of Point32 objects in the world frame. no longer pixels)
				path_example = self.path_from_pixels_list(pixel_path)
				# go there - add this path to the paths to follow
				# add the last coordinate in the world frame to the class variable goal - so checking whether
				# got there or not would be possible

				# end_coord = self.pixel_to_world(end_pixel[0], end_pixel[1]) ##############
				# self.goal_x = end_coord[0]
				# self.goal_y = end_coord[1]
				self.paths.append(path_example)
				print("success with dijkstra! got path")
				self.reached_goal = False
				self.follow_path(self.paths[0])





            # if we have reached the goal, go to the next path
			if self.reached_goal:
				print "in reached goal"

				# check if there are remaining paths
				if len(self.paths) > 0:
					print "Publishing path to follow"

					# got to the first path, and set the reached variable to false
					self.reached_goal = False;

					# get the path we want to follow
					path = self.paths[0]
					#remove it from the paths list
					self.paths.pop(0)
					# follow the path
					self.follow_path(path)
				
				else:
					# no more goals and reached goal
					print("navigation test finished")
					# generate a new random goal to go to


			else:
				# not reached goal yet, check if there now`

				if abs(self.current_x - self.goal_x) < 0.15 and abs(self.current_y - self.goal_y) < 0.15:
					# then we are close enough to the goal and can toggle the reach variable
					self.reached_goal = True
					print('reached goal,erasing all paths')
					self.paths = []
				else:

					print "navigating to (",self.goal_x,", ", self.goal_y, ")"           
					print "       d=(", abs(self.current_x - self.goal_x), ", ", abs(self.current_y - self.goal_y), ")" 
					print "     now=(", self.current_x, ", ", self.current_y, ")"

	def map_cb(self, map_msg):
    	# callback to be fired every time a map update is being published
    	# updates the class variables
		self.map_resolution = map_msg.info.resolution
		self.map_width = map_msg.info.width
		self.map_height = map_msg.info.height
		self.map_origin_x = map_msg.info.origin.position.x
		self.map_origin_y = map_msg.info.origin.position.y
		self.map_grid = map_msg.data

		# now that we have all the new information, also publish the guassian filter of the map
		self.gaussian_filter()

		# self.look_meter_pos_x()
		# print("=====\n map origin at x=", self.map_origin_x, " y=", self.map_origin_y)
		# robot_pixel = self.world_to_pixel(self.current_x, self.current_y);
		# print("=====\n robot at pixel x=", robot_pixel[0], " y=",robot_pixel[1])
		# print(" the value of the pixel=", self.get_pixel_value(robot_pixel[0], robot_pixel[1], self.map_grid))

		# robot_world= self.pixel_to_world(robot_pixel[0], robot_pixel[1])
		# print("=====\n should be at x =", self.current_x, "y=", self.current_y)
		# print(" calculated at x=", robot_world[0], "y=", robot_world[1])

	def look_meter_pos_x(self):
		# prints the occupancy values in the next meter in the positive x direction
		values = []
		for dist in range (0,10):
			# print(dist/10.0)
			x_value = self.current_x + dist/10.0
			y_value = self.current_y
			pixel = self.world_to_pixel(x_value, y_value)
			pixel_value = self.get_pixel_value(pixel[0], pixel[1], self.map_grid)
			values.append(pixel_value)
		print(values)

	def look_meter_pos_y(self):
		# prints the occupancy values in the next meter in the positive y direction
		values = []
		for dist in range (0,10):
			# print(dist/10.0)
			y_value = self.current_x + dist/10.0
			x_value = self.current_y
			pixel = self.world_to_pixel(x_value, y_value)
			pixel_value = self.get_pixel_value(pixel[0], pixel[1], self.map_grid)
			values.append(pixel_value)
		print(values)


	def world_to_pixel(self, robot_x, robot_y):
		# returns a tuple with the (row, col) position of the robot on the map grid
		pixel_x= round((robot_x - self.map_origin_x)/self.map_resolution)
		pixel_y = round((robot_y - self.map_origin_y)/self.map_resolution)
		return (pixel_x, pixel_y)

	def pixel_to_world(self, pixel_x, pixel_y):
		# returns a tuple with the robot (x,y) in the world frame
		robot_x = pixel_x*self.map_resolution + self.map_origin_x
		robot_y = pixel_y*self.map_resolution + self.map_origin_y
		return (robot_x, robot_y)

	def get_pixel_value(self, pixel_x, pixel_y, grid):
		# returns the value stored in the map_grid at that pixel
		index = int(pixel_x + self.map_width*pixel_y) # int because index cannot be float
		return grid[index]


	def get_possible_pixels(self, pixel_x, pixel_y, grid = None):
		# arguments are a pixel in the map_grid
		# returns a list of tuples of possible pixels the robot can progress to
		# these pixels have to be in the map_grid and be vacant (value = 0)

		if grid == None:
			grid = self.map_grid

		# the allowed moves are up down left right
		# return a list of tuples that only include coordinates of cells that are empty
		tries = [] #[(pixel_x, pixel_y+1), (pixel_x, pixel_y-1), (pixel_x+1, pixel_y), (pixel_x-1, pixel_y)]

		tries += [(pixel_x, pixel_y+1)]
		tries.append((pixel_x, pixel_y-1))
		tries.append((pixel_x+1, pixel_y))
		tries.append((pixel_x-1, pixel_y))

		tries.append((pixel_x-1, pixel_y-1))
		tries.append((pixel_x+1, pixel_y+1))
		tries.append((pixel_x-1, pixel_y+1))
		tries.append((pixel_x+1, pixel_y-1))

		# print tries

		finals = []
		for coord in tries:

			if coord[0] < self.map_width and coord[0] >= 0 and coord[1] < self.map_height and coord[1] >= 0:


				# print("for coord", coord, "value ", self.get_pixel_value(coord[0], coord[1], self.map_grid))

########################
				if self.get_pixel_value(coord[0], coord[1], grid) not in [100, -1, '0'] and self.surroundings_empty(coord[0], coord[1], 2):
					finals.append(coord)

		# print("neighbors for", pixel_x, pixel_y, finals,  "with map size is width=", self.map_width, "height=", self.map_height)
		# print "\n\n"
		return finals


	def weighted_graph(self):
		# function that creates a wighted graph
		# this object takes the form of {coord_tuple: {neighbor0_tuple: weight, neighbor1_tuple: weight1}}
		weighted_graph =  {};
		for pixel_x in range(self.map_width):
			for pixel_y in range (self.map_height):
				pixel = (pixel_x,pixel_y)
				# get the surrounding coords
				neighbors = self.get_possible_pixels(pixel_x, pixel_y, self.map_grid)
				# create the dictionary mapping adjacent nodes to their weight
				neighbors_dict = dict()
				for neighbor in neighbors:
					print "neighbor=", neighbor
					neighbors_dict[neighbor] = self.cost_function(self.filtered_map_grid, neighbor)


					#1 # all the weights are 1 in this example add cost function
				

				# add the pixel we are working with, with its adjacent nodes to the weighted_graph
				weighted_graph[pixel] = neighbors_dict

		return weighted_graph

	def dijkstra_to_goal(self, weighted_graph, start, end):
		# arguments:
		'''
		weighted_graph : see function above
		start : a tuple of pixel to start navigating from, of form (x,y) in the map_grid
		end : a tuple of pixel to end navigating at, of form (x,y) in the map_grid

		returns:
		a list of pixels of form [start, ... , pixels in between, ..., end]

		'''
		# set up initial values
		# always need to visit the start
		print "in dijkstra from ", start, "to", end
		print " which translates to from ", self.pixel_to_world(start[0], start[1]), " to ", self.pixel_to_world(end[0], end[1])

		goal_coord = self.pixel_to_world(end[0], end[1])
		self.goal_x = goal_coord[0]
		self.goal_y = goal_coord[1]

		print "SETTING GOAL ", self.goal_x, " ,  ", self.goal_y

		nodes_to_visit = {start}
		visited_nodes = set()
		distance_from_start = defaultdict(lambda: float("inf"))
		# distance from start to start is 0
		distance_from_start[start] = 0
		tentative_parents = {}

		while nodes_to_visit:
			# the next node should be the one with the smalles weight
			current = min(
				(distance_from_start[node],node) for node in nodes_to_visit)[1]

			#check if end was reached
			if current == end:
				break

			# otherwise
			nodes_to_visit.discard(current)
			visited_nodes.add(current)

			for neighbor, distance in weighted_graph[current].items():
				if neighbor in visited_nodes:
					continue
				neighbor_distance = distance_from_start[current] + distance
				if neighbor_distance < distance_from_start[neighbor]:
					distance_from_start[neighbor] = neighbor_distance
					tentative_parents[neighbor] = current
					nodes_to_visit.add(neighbor)

		
		return self.deconstruct_path(tentative_parents, end)


	def deconstruct_path(self, tentative_parents, end):
		if end not in tentative_parents:
			return None

		cursor = end
		path = []
		while cursor:
			path.append(cursor)
			cursor = tentative_parents.get(cursor)

		# reverse the path
		pixel_list = list(reversed(path))
		return pixel_list


	def path_from_pixels_list(self, pixels_list):
		# returns a list of Point32 objects that can be followed by racecar

		path_list = []
		for pixel in pixels_list:
			############### might take some points out here to make it easier on the robot to follow
			# convert to world frame
			coord = self.pixel_to_world(pixel[0], pixel[1])
			# convert the coord tuple to Point32 and add to the list
			path_list.append(Point32(coord[0], coord[1], 0.0))

		return path_list



	def get_free_random_coord(self):
		while True:
			#generate a random pixel
			random_pixel_x = random.randint(0,self.map_width)
			random_pixel_y = random.randint(0,self.map_height)
			# check if pixel is vacant
			if self.get_pixel_value(random_pixel_x, random_pixel_y, self.map_grid) == 0:

				# # check if the pixel is in the allowable segment in front of the robot
				# segment_angle = numpy.pi/2 # in radians!

				# #calculate angle of found point with the location of the robot, with respect to the y axis (forward when initializing)
				# point_angle = 


				# check that the surrounding coords are also empty
				if self.surroundings_empty(random_pixel_x, random_pixel_y):

					coord = self.pixel_to_world(random_pixel_x, random_pixel_y)
					return coord





	def surroundings_empty(self, pixel_x, pixel_y, radius = 5):
		pixel_x = int(pixel_x)
		pixel_y = int(pixel_y)
		for x in range(pixel_x-radius, pixel_x+radius):
			for y in range(pixel_y-radius, pixel_y+radius):

				if x<0 or x >= self.map_width-10 or y<0 or y >= self.map_height-10:
					return False

				value = self.get_pixel_value(x,y,self.map_grid)
				if value != 0:
					return False

		return True


	def a_star(self, grid, start_pixel, goal_pixel):
		# want to keep track of a couple of things:
		# * seen nodes, that will not be considered again
		# * parent, a dictionary mapping each node to its optimal parent
		# * priority queue, mapping each node to its current score (heuristic + cost from start)
		# * dist from start, mappind each node to its diestance from start (for calculations)


		# set goal for car
		goal_coord = self.pixel_to_world(goal_pixel[0], goal_pixel[1])
		self.goal_x = goal_coord[0]
		self.goal_y = goal_coord[1]


		parent = dict()
		seen = set()
		Q = {} # defaultdict(lambda: float("inf"))
		start_dist = dict()

		# we know that the distance from the start to itself is 0
		start_dist[start_pixel] = 0
		# we also know that the score of the start pixel is only the heuristic value, since start distance to itself is 0
		Q[start_pixel] = self.heuristic_function(start_pixel, goal_pixel)

		# now we can get to work:
		# as long as the queue is not empty, continue
		while Q:
			# pop the first item from the queue
			# since this is a priority queue, then the first item will be the one with the smallest score.
			node = tuple()
			node_score = float("inf")
			for node_in_Q, score in Q.items():
				if score < node_score:
					node = node_in_Q
					node_score= score
			print "in Q=", Q
			print "smallest node", node

			# now we have the item with the smallest score, pop it out of the Q
			Q.pop(node)

			# if this is the goal pixel, we can break and return a reconstructed path
			if node == goal_pixel:
				path = self.path_constructor(parent, start_pixel, goal_pixel)
				return path

			# otherwise, look through its children
			adjacent = self.get_possible_pixels(node[0], node[1], grid)

			for child in adjacent:
				# only work on child if not seen yet
				if child not in seen:
					# mark as seen
					seen.add(child)
					# calculate score for child:
					# score = heuristic + distance from start of parent + distance from parent to child
					child_start_dist = start_dist[node] + self.cost_function(grid, child)
					child_score = self.heuristic_function(child, goal_pixel) + child_start_dist

					# if this score is better than the one associated currently (if there is one associated at all)
					# then update parent, Q, and dist from start
					if child in Q:
						if child_score < Q[child]:
							parent[child] = node
							Q[child] = child_score
							start_dist[child] = start_child_dist
					else:
						# if not in Q at all, then the associated score of child in infinity and we can just assign the current score to child
						# since the new values will always be less than infinity

						parent[child] = node
						Q[child] = child_score
						start_dist[child] = child_start_dist


		print "returning none from A*"
		return None


	def BFS(self, start_pixel, end_pixel):
		print "called BFS start ", start_pixel, "end ", end_pixel
		# start and end are tuples of x and y in grid frame
		# initialize variables
		Q = list()
		seen = set()
		parent = dict()

		# set goal for car
		goal_coord = self.pixel_to_world(end_pixel[0], end_pixel[1])
		self.goal_x = goal_coord[0]
		self.goal_y = goal_coord[1]

		# put first node  in seen and in @
		seen.add(start_pixel)
		Q.append(start_pixel)

		while len(Q) > 0:
			# take the first node out of the Q
			node = Q.pop(0)
			# go through the children of the node
			children = self.get_possible_pixels(node[0], node[1])
			for child in children:
				# assign parent and mark seen
				if child not in seen:
					parent[child] = node
					seen.add(child)
					Q.append(child)
					# chick if child is the goal, if it is then return a deconostructed path
					if child == end_pixel:
						return self.path_constructor(parent, start_pixel, end_pixel)


		return None


	def path_constructor(self, parents, start_pixel, end_pixel):
		path = [end_pixel]
		while True:
			new_node = parents[path[0]]

			path = [new_node] + path

			# check if we just added the start node
			if new_node == start_pixel:
				print "path ", path
				return path;

	def gaussian_filter(self):
		# this function applies a gaussian filter on the map we got from /projected_map
		# it also publishes a filtered map to the topic /gaussian_map
		# rviz can subscribe to that topic and overlay the blurred map on the actual occupancy grid

		# convert map to a numpy array
		map_array = numpy.array(self.map_grid)
		map_array = numpy.reshape(map_array, (-1, self.map_width))

		# apply the filter
		filtered = scipy.ndimage.gaussian_filter(map_array, 2.4)

		# convert to a ROS OccupancyGrid msg object
		msg = OccupancyGrid()
		msg.header.stamp = rospy.Time(0)
		msg.header.frame_id = "/world"
		msg.info.resolution = self.map_resolution
		msg.info.width = self.map_width
		msg.info.height = self.map_height

		msg.info.origin.position.x = self.map_origin_x
		msg.info.origin.position.y = self.map_origin_y

		map_lists = filtered.tolist()

		# convert to one list
		msg.data = [item for row in map_lists for item in row]

		# publish the data
		self.filter_pub.publish(msg)

		# store the data locally too, so we won't have to subscribe to this topic
		self.filtered_map_grid = msg.data




	def cost_function(self, grid, node_pixel):
		# a function that returns the cost needed in order to reach certain node
		#this cost is not on a certain ecge, but rather the cost of getting to the
		# node from any adjacent node
		# this cost is represented in the map as the value stored in the pixel coordinate
		# node is tuple of form (pixel_x, pixel_y)
		return self.get_pixel_value(node_pixel[0], node_pixel[1], grid)

	def heuristic_function(self, start_pixel, goal_pixel):
		# this function will return the manhattan distance from the start to the goal
		# the distance will be given in units of pixels, and will be used as a (n admissable) heuristic for A*
		return abs(start_pixel[0] - goal_pixel[0] + start_pixel[1] - goal_pixel[1])







if __name__ == '__main__':
	#print a simple thing just to show that this script is alive
	print("car_nav_script is ALIVE")

	try:
		print("##### trying the navigation leaader class")
		navigation_leader()
	except rospy.ROSInterruptException:
		rospy.loginfo("#############Navigation test finished.")
