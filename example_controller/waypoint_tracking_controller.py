#!/usr/bin/env python

# ROS imports
import rospy
import roslib
import tf
from tf import transformations

# Python imports
import numpy as np
import csv

# custom RightHook messages
from rh_msgs.msg import VehicleActuation
from rh_msgs.msg import VehicleState

'''
Point class, just a convenience container for
2d points.
'''
class Point:
	def __init__(self, x, y):
		self.x = x
		self.y = y

	def get_x(self):
		return self.x

	def get_y(self):
		return self.y

	# get this point as a column vector
	def as_array(self):
		t = np.array([self.get_x(), self.get_y()])
		t.shape = (2,1)
		return t

'''
Edge class.  Edges connect two points.  We're
going to localize the car on one of these edges.
Edges are directed from source to sink.
'''
class Edge:
	def __init__(self, source, sink):
		self.source = source
		self.sink = sink

	def get_source(self):
		return self.source

	def get_sink(self):
		return self.sink

	# compute length (2-norm)
	def get_length(self):
		dx = self.sink.get_x() - self.source.get_x()
		dy = self.sink.get_y() - self.source.get_y()
		return np.linalg.norm([dx, dy])

	# treat like a ray and get the angle in the plane it points
	def get_angle(self):
		dx = self.sink.get_x() - self.source.get_x()
		dy = self.sink.get_y() - self.source.get_y()
		return np.arctan2(dy, dx)

	'''
	Figure out where the point p is relative to this
	edge and return as a LocationOnEdge.
	'''
	def locate(self, p, hdg):
		p_array = p.as_array()

		# translate vehicle location and sink so source is origin
		sink_trans = self.get_sink().as_array() - self.get_source().as_array()
		p_trans = p_array - self.get_source().as_array()

		# get angle of edge as ray
		ang = np.arctan2(sink_trans[1], sink_trans[0])

		# get cosine and sine of that angle
		c = np.cos(ang)[0]
		s = np.sin(ang)[0]

		# create rotation matrix that orients edge
		# in +x
		rot_mat = np.matrix([[c, s],[-s, c]])

		# rotate both vehicle location and sink around
		# origin so now edge is oriented in +x
		p_rot = rot_mat * p_trans
		sink_rot = rot_mat * sink_trans

		'''
		so now point's x is how far along edge longitudinally
		and point's y is how far from edge laterally
		'''

		# use heuristic to decide if we consider point
		# to be on this edge
		long_good = (p_rot[0] >= 0.0 and p_rot[0] <= sink_rot[0])
		lat_good = (np.abs(p_rot[1]) < 5.0)

		# met both critera?
		good = long_good and lat_good

		# return result
		return LocationOnEdge(self, good, p_rot[0], p_rot[1], hdg - ang)

	'''
	Return a point at distance d along this edge
	'''
	def interpolate(self, d):
		r = d / self.get_length()
		return Point((1.0 - r) * self.get_source().get_x() + r * self.get_sink().get_x(),
			(1.0 - r) * self.get_source().get_y() + r * self.get_sink().get_y())

'''
Container for result from localizing algo
'''
class LocationOnEdge:
	def __init__(self, edge, good, x_rel, y_rel, hdg_rel):
		self.edge = edge
		self.good = good
		self.x_rel = x_rel
		self.y_rel = y_rel
		self.hdg_rel = hdg_rel

	def get_edge(self):
		return self.edge

	def get_x_rel(self):
		return self.x_rel

	def is_good(self):
		return self.good

'''
Container composed of LocationOnEdge and what index in
our ring that edge is
'''
class LocationOnRing:
	def __init__(self, index, loc_on_edge):
		self.index = index
		self.loc_on_edge = loc_on_edge

	def get_location_on_ring(self):
		return self.loc_on_edge

	def get_index(self):
		return self.index

'''
Container for a closed ring of edges.  This is what we
use to represent the racing line.
'''
class EdgeRing:
	def __init__(self, edges):
		self.edges = edges
		self.last_index = 0

	'''
	Get the reference point that we're steering towards.
	First step is localize ourselves on the EdgeRing, then
	look forward a little bit.
	'''
	def get_ref_pt(self, p, hdg):
		# start with null location
		loc_on_ring = None

		# start at last known index and loop through edges on ring
		for i in range(self.last_index, len(self.edges)) + range(0, self.last_index):
			# compute localization result
			loc_on_edge = self.edges[i].locate(p, hdg)
			# if localization result for this edge is good, store
			# as LocationOnRing and exit for loop
			if loc_on_edge.is_good():
				loc_on_ring = LocationOnRing(i, loc_on_edge)
				self.last_index = i
				break
		# find what point we should steer towards
		if loc_on_ring:
			index = loc_on_ring.get_index()
			# keep track of cumulative distance from vehicle location
			# to point we'll pick to steer towards
			cum_dist = loc_on_edge.get_edge().get_length() - loc_on_edge.get_x_rel()
			# while that cumulate distance is less than desired, keep loop going through edges
			while cum_dist < 10.0:
				p_cum_dist = cum_dist
				index = (index + 1) % len(self.edges)
				cum_dist = cum_dist + self.edges[index].get_length()

			# cumulative distance is far enough, calculate the point to steer towards
			steer_to_point = self.edges[index].interpolate(10.0 - p_cum_dist)
			# return it
			return steer_to_point
		# couldn't localize the car?  steer towards start-finish
		else:
			return Point(0.0, 0.0)

'''
Convenience method for calculating steering angle
once we've localized the car and picked a point to
steer towards.
'''
def calc_steering_deg(p, hdg, ref):
	# translate reference point so vehicle is at origin
	ref_trans = ref.as_array() - p.as_array()

	# create rotation matrix to frame where vehicle
	# forward direction is +x
	c = np.cos(hdg)
	s = np.sin(hdg)
	rot_mat = np.matrix([[c, s],[-s, c]])

	ref_rot = rot_mat * ref_trans

	# So now angle from vehicle to reference point is like
	# an error term.  We'll slap a gain on it, convert it from
	# radians to degrees and call it the desired steering angle.
	return 0.2 * np.arctan2(ref_rot[1], ref_rot[0]) * 180.0 / np.pi

'''
Main function.  Import data, create a node, run
controller on a loop.
'''
if __name__ == '__main__':

	# import waypoints, which are in a csv file formatted x, y
	with open('./waypoints.xyz', 'rb') as f:
		reader = csv.reader(f, delimiter=',', quoting=csv.QUOTE_NONNUMERIC)
		i = 0
		# make empty arrays for points and edges
		points = []
		edges = []
		for row in reader:
			# read row in as a point
			points.append(Point(row[0], row[1]))
			# connect point to previous point and add as edge
			if i > 0:
				edges.append(Edge(points[i-1], points[i]))
			i += 1
		# connect final point to first point as edge
		edges.append(Edge(points[i-1], points[0]))
	# construct EdgeRing from list of edges
	ring = EdgeRing(edges)

	# start our translator node
	rospy.init_node('thunderhill_controller')

	# which publishes actuation_requests
	pub = rospy.Publisher('/kart/actuation/actuation_request', VehicleActuation, queue_size=1)

	# listen for vehicle location
	tf_listener = tf.TransformListener()

	# run fast
	rate = rospy.Rate(60.0)

	# initialize previous steering angle (for filter)
	p_steering_degrees = 0.0

	# run controller on a loop as long as ROS is up
	while not rospy.is_shutdown():
		# get translation from transform listener
		try:
			(trans, rot) = tf_listener.lookupTransform('/level', '/kart', rospy.Time(0))
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			continue

		# construct actuation message
		msg = VehicleActuation()

		# get yaw angle from rotation
		euler = transformations.euler_from_quaternion(rot)
		yaw = euler[2]

		# create point for vehicle location
		p = Point(trans[0], trans[1])

		# get reference point (that is, localize and look forward)
		steer_to_point = ring.get_ref_pt(p, yaw)

		# set throttle to half
		msg.normalized_throttle = 0.5
		
		# calculate steering angle based on reference point
		raw_steering_degrees = calc_steering_deg(p, yaw, steer_to_point)

		# filter
		new_steering_degrees = 0.7 * p_steering_degrees + 0.3 * raw_steering_degrees

		# set message data
		msg.steering_degrees = new_steering_degrees

		# publish message
		pub.publish(msg)

		# store last steering angle for filter
		p_steering_degrees = new_steering_degrees

		# sleep for a hot sec
		rate.sleep()
