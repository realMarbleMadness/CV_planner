from geometry_msgs.msg import (
                                PoseStamped,
                                Pose,
                                Point,
                                Quaternion,
                                )
import numpy as np
from pdb import set_trace



class Config(object):	
	# Specify bounds of movement in cartesian space of end effector
	X_LOWER = 0.32
	X_UPPER = 0.975
	Y_LOWER = -1.0
	Y_UPPER = 1.0
	Z_LOWER = -0.179
	Z_UPPER = 2.0


class PoseConfig(Config):

	OVERHEAD_ORIENTATION = Quaternion(
                                 x=-0.0249590815779,
                                 y=0.999649402929,
                                 z=0.00737916180073,
                                 w=0.00486450832011)
	OVERHEAD_ORIENTATION = Quaternion(
                                 x=0.0,
                                 y=1.0,
                                 z=0.0,
                                 w=0.0)
	# home position
	HOME_POSE = Pose()
	HOME_POSE.orientation.x = 0.0
	HOME_POSE.orientation.y = 1.0
	HOME_POSE.orientation.z = 0.0
	HOME_POSE.orientation.w = 0.0
	HOME_POSE.position.x = 0.503455047806
	HOME_POSE.position.y = 0.0170740011154
	HOME_POSE.position.z = -0.0879255999207

	CLOSE_POSE = Pose()
	CLOSE_POSE.orientation.x = 0.0
	CLOSE_POSE.orientation.y = 1.0
	CLOSE_POSE.orientation.z = 0.0
	CLOSE_POSE.orientation.w = 0.0
	CLOSE_POSE.position.x = 0.6
	CLOSE_POSE.position.y = 0.0170740011154
	CLOSE_POSE.position.z = -0.0879255999207

class CartesianConfig(Config):

	OVERHEAD_ORIENTATION = [
									0.0,
									1.0,
									0.0,
									0.0
                            ]
	# home position
	HOME_POSE = [
									0.503455047806,
									0.0170740011154,
									-0.0879255999207,
									0.0, 
									1.0,
									0.0,
									0.0
								]
	
	CLOSE_POSE = [
									0.6,
									0.0170740011154,
									-0.0879255999207,
									0.0,
									1.0,
									0.0,
									0.0
								]



