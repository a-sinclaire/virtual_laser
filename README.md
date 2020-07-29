# virtual_laser

looks at 'laser_origin' tf frame.

projects virtual ray from that frame

travels along until that ray intersects a pointcloud in the world

publishes a new point cloud with the point of intersection

#

home version is meant for me (Sam)
camera_orient goes along with that version (since everything in mine is all rotated 90 degrees this fixes that)

virtual_laser.py is for scooter (idk if it is working in its current state)
