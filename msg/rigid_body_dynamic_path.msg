# This messgae is consisted of the standard path message for the position 
# and orientation(only yaw) of the robot. 
# drones distances : the distnaces between the 2 drones
# drones_angles : the angles between the 2 drones formed by the line which connects them and the 
#                 hotizontal axis line 
nav_msgs/Path Path
float32[] drones_distances
float32[] drones_angles 