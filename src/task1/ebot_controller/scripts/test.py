# this is an isolated test file because i am too scared the main code might break
import matplotlib.pyplot as plt 
import numpy as np
from math import *


# for way points
_x_low   =  0 
_x_high  = 100
_x_step  =  2
_x_scale =  2

def Waypoints(t):
	"""
		generates waypoints along a given 
		continuous and differentiable curve t
	"""
	global _x_low, _x_high, _x_step, _x_scale
	t_dash = lambda x: round((cos(x/2) * sin(x) + 2 * sin(x/2) * cos(x)), 2)
	xs = [x/_x_scale for x in range(_x_low, _x_high, _x_step)]
	# waypoint = [x, y, theta]
	waypoint_buffer = [[round((x),2), round((t(x)),2), round((atan(t_dash(x))),2)] for x in xs]
	return waypoint_buffer

def main():
	trajectory = lambda x:2*sin(x)*sin(x/2)
	wp = Waypoints(trajectory)
	# for w in wp:
		# print(w)
	print(len(wp))
	x = wp.pop(0)
	print(x)
	# print(len(wp))

if __name__ == '__main__':
	main()