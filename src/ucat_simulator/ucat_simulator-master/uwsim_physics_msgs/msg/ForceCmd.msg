# This is a message that holds data to describe the state of a set of vehicle force generators. 
#
# The state of each force generator (buoyance or thruster usually) is defined by:
#  * the 'force' vector in 'pose' coordinates (Vector3)
#  * the 'pose' (position/orientation transform) in the relativeTo link coordinates
#  * the 'duration' of force in seconds (<0 means inifinity)
#  * the 'radius' of force application sphere in metres, used for 'multiplier' calculation
#  * the 'multiplier' - ratio of how much force is actually applied, proportional to volume of 
#												force application sphere inside water, always 1 if radius<0
#  * the 'link' - name of parent link in URDF file
#  * the 'forceType' - 'z' for zero, 'a' for absolute (Z points up), 'g' for global (world) coordinates, 'l' for local
#
# Each force generator is uniquely identified by its name
# The header specifies the time at which the forces were recorded. All the forces
# in one message have to be recorded at the same time.
#
# This message consists of a multiple arrays, one for each part of the force generator state. 
# All arrays in this message should have the same size, or be empty.
# Updatable fields: force, duration (-1 for infinite, defaults to 1 (second) if empty), radius and pose
# Constant fields: link and forceType
# Calculated fields: multiplier and duration (reset on update)

# Name is required for command;  
# force+duration, radius and pose can be updated separately of each other
# It is recommended to update only force+duration, radius and pose are best left alone and are enabled only for future hacks, if needed :)

Header header

#updateable fileds
string[] name 
geometry_msgs/Vector3[] force
float32[] duration #if force is set and duration is empty, 1 second is taken as default...
float32[] radius
geometry_msgs/Pose[] pose

#read-only fields
string[] link
float32[] multiplier
string[] forceType

