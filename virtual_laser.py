#!/usr/bin/env python
# python
import time

# ros
import rospy
from std_msgs.msg import Header as HeaderMsg
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler

# scipy
import numpy as np

# point cloud
import open3d as o3d

# self
from simple_ui import Scooter


class VirtualLaser:
    def __init__(self, point_cloud, radius, max_loops=50):
        """
        :param radius: search radius (thickness of laser in m)
        :param max_loops: how many iterations (how far the laser reaches)
        :param point_cloud: point cloud we shoot laser at
        """
        self.origin = [0, 0, 0]
        self.direction = [0, 0, 0]
        self.radius = radius
        self.max_loops = max_loops
        # convert to open3d pointcloud
        self.stitched_pointcloud_o3d = self.rosmsg_to_o3d(point_cloud)

        self.centroid_pub = rospy.Publisher('/laser_centroid', PointCloud2, queue_size=1)
        self.target_group_pub = rospy.Publisher('/laser_group_points', PointCloud2, queue_size=1)
        self.draw_ray_pub = rospy.Publisher('/draw_laser_ray', PointCloud2, queue_size=1)

    def o3d_to_rosmsg(self, o3d_pcl, frame_link):
        """
        converts a open3d object to a ros PointCloud2 msg
        :param o3d_pcl: open3d pointcloud
        :param frame_link: frame to attach newly created ros msg
        :return: ros PointCloud2 msg
        """
        header = HeaderMsg()
        header.frame_id = frame_link
        header.stamp = rospy.Time.now()
        cloud_arr = np.array(o3d_pcl.points)
        return pc2.create_cloud_xyz32(header, cloud_arr)

    def rosmsg_to_o3d(self, msg):
        """
        converts a ros PointCloud2 msg to a open3d pointcloud
        :param msg: ros PointCloud2 msg
        :return: open3d pointcloud
        """
        o3d_cloud = o3d.geometry.PointCloud()
        arr = np.array(list(pc2.read_points(msg)))[:, 0:3]
        o3d_cloud.points = o3d.utility.Vector3dVector(arr)
        return o3d_cloud

    def process(self):
        """
        publishes intersection point of virtual laser with point cloud
        """

        # voxelfilter downsample w/ o3d
        t1 = time.time()
        o3d_cloud_filtered = self.stitched_pointcloud_o3d.voxel_down_sample(voxel_size=0.03)
        t2 = time.time()

        # running the algorithm to get collision points of laser
        t3 = time.time()
        collision_points = self.get_collision_points(o3d_cloud_filtered)
        if collision_points is not None:
            centroid_point = [np.mean(collision_points, axis=0)]
            print 'centroid: '
            print centroid_point  # [[0,0,0]]
        t4 = time.time()

        #

        # vvv PUBLISH TO TOPICS vvv

        if collision_points is not None:
            # publish the found collisionPoints to /laser_group_points topic
            o3d_temp = o3d.geometry.PointCloud()
            o3d_temp.points = o3d.utility.Vector3dVector(collision_points)
            target_msg = self.o3d_to_rosmsg(o3d_temp, "base_link")
            self.target_group_pub.publish(target_msg)

            # publish centroid of collisionPoints to /laser_centroid topic
            o3d_temp = o3d.geometry.PointCloud()
            o3d_temp.points = o3d.utility.Vector3dVector(centroid_point)
            target_msg = self.o3d_to_rosmsg(o3d_temp, "base_link")
            self.centroid_pub.publish(target_msg)

        # draw laser ray
        counter = 0
        ray_arr = []
        while counter < self.max_loops * self.max_loops:
            test_point = [0, 0, 0]
            test_point[0] = self.origin[0] + (counter * self.direction[0])
            test_point[1] = self.origin[1] + (counter * self.direction[1])
            test_point[2] = self.origin[2] + (counter * self.direction[2])
            ray_arr.append(test_point)
            counter += self.max_loops / np.linalg.norm(self.direction)
        o3d_temp = o3d.geometry.PointCloud()
        o3d_temp.points = o3d.utility.Vector3dVector(ray_arr)
        target_msg = self.o3d_to_rosmsg(o3d_temp, "base_link")
        self.draw_ray_pub.publish(target_msg)

        # display algorithm time
        time_message = "Voxel Downsampling time: {} seconds \nAlgorithm time: {} " \
                       "seconds "
        total = "Total time: {} seconds"
        print(time_message.format(t2 - t1, t4 - t3))
        print(total.format(t2 - t1 + t4 - t3))

    def get_collision_points(self, o3d_cloud):
        """
        travels along a defined ray and returns a a collection of points where the ray intersects a given open3d pointcloud
        :param o3d_cloud: o3d pointcloud the laser is being shot at (down-sampled)
        """

        test_point = [0, 0, 0]
        counter = 0
        loops = 0
        extra_hops = 1  # keep as one

        # store pointcloud in kd-tree
        kd_tree = o3d.geometry.KDTreeFlann(o3d_cloud)

        # Algorithm loop:
        while True:
            # update current test point
            test_point[0] = self.origin[0] + (counter * self.direction[0])
            test_point[1] = self.origin[1] + (counter * self.direction[1])
            test_point[2] = self.origin[2] + (counter * self.direction[2])

            tup = kd_tree.search_radius_vector_3d(test_point, self.radius)
            if tup[0] > 0:  # if # of points in our k-neighborhood is greater than 0, then we hit the cloud!
                print('Found ' + str(tup[0]) + ' points within ' + str(self.radius) + ' of (' + str(
                    test_point[0]) + ', ' + str(test_point[1]) + ', ' + str(test_point[2]) + ')')
                points = np.asarray([o3d_cloud.points[tup[1][k]] for k in range(0, len(tup[1]))])
                for i in range(extra_hops):  # continue another step forward to see if there are more points nearby
                    counter += self.radius / np.linalg.norm(self.direction)
                    # update current test point
                    test_point[0] = self.origin[0] + (counter * self.direction[0])
                    test_point[1] = self.origin[1] + (counter * self.direction[1])
                    test_point[2] = self.origin[2] + (counter * self.direction[2])
                    tup = kd_tree.search_radius_vector_3d(test_point, self.radius)
                    if len(tup[1]) > 0:  # still near cloud after second test point!
                        # here are the points nearby!
                        points = np.concatenate(
                            (points, np.asarray([o3d_cloud.points[tup[1][k]] for k in range(0, len(tup[1]))])), axis=0)
                    else:  # Break if the second point is not close to any points at all
                        break
                    # if points are found
                    # return list of points found
                    return points
            elif loops == self.max_loops:
                # if no points found after moving along ray for x loops
                # then return None
                print("Missed the pointcloud.")
                return None
            else:
                # print('No points within ' + str(threshold) + ' of (' + str(test_point[0]) + ', ' + str(
                #     test_point[1]) + ', ' + str(test_point[2]) + ')')
                counter += self.radius / np.linalg.norm(
                    self.direction)  # iterate counter to continue moving forward along ray
                loops += 1

    def update_orig_and_dir(self, orig, dir):
        """
        updates origin and direction of virtual laser
        :param orig: (x, y, z)
        :param dir: (x, y, z) directional vector
        """
        self.origin = orig
        self.direction = dir

    def euler_to_vector(self, arg):
        """
        converts roll pitch yaw to a directional vector x y z
        :param arg: (roll, pitch, yaw)
        :return: (x, y, z)
        """
        roll = arg[0]
        pitch = arg[1]
        yaw = arg[2]

        x = np.cos(yaw) * np.cos(pitch)
        y = np.sin(yaw) * np.cos(pitch)
        z = np.sin(pitch)
        return [x, y, z]

    # # replaced with quaternion_from_euler() from tf.transformations
    # def euler_to_quaternion(self, arg):
    #     (roll, pitch, yaw) = (arg[0], arg[1], arg[2])
    #     qx = np.sin(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) - np.cos(roll / 2) * np.sin(pitch / 2) * np.sin(
    #         yaw / 2)
    #     qy = np.cos(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.cos(pitch / 2) * np.sin(
    #         yaw / 2)
    #     qz = np.cos(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2) - np.sin(roll / 2) * np.sin(pitch / 2) * np.cos(
    #         yaw / 2)
    #     qw = np.cos(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.sin(pitch / 2) * np.sin(
    #         yaw / 2)
    #     return [qx, qy, qz, qw]

    # # no longer needed
    # def vector_to_euler(self, arg):
    #     x = arg[0]
    #     y = arg[1]
    #     z = arg[2]
    #
    #     roll = 0
    #     # pitch
    #     if z is not 0:
    #         if x < 0:
    #             if z < 0:
    #                 pitch = -np.arctan((np.sqrt((x ** 2) + (y ** 2))) / z) + np.pi
    #             else:
    #                 pitch = -np.arctan((np.sqrt((x ** 2) + (y ** 2))) / z)
    #         else:
    #             if z < 0:
    #                 pitch = np.arctan((np.sqrt((x ** 2) + (y ** 2))) / z) + np.pi
    #             else:
    #                 pitch = np.arctan((np.sqrt((x ** 2) + (y ** 2))) / z)
    #     else:
    #         pitch = 0
    #     # yaw
    #     if y is not 0:
    #         if y < 0:
    #             yaw = np.arctan(x / -y) + np.pi
    #         else:
    #             yaw = np.arctan(x / -y)
    #     else:
    #         yaw = 0  # np.pi/2
    #     return [roll, pitch, yaw]


class Driver:
    def __init__(self, origin_frame, radius, max_loops):
        """
        :param origin_frame: tf frame that defines origin and dir of laser
        :param radius: search radius (thickness of laser in m)
        :param max_loops: how many iterations (how far the laser reaches)
        """
        self.origin_frame = origin_frame
        self.radius = radius
        self.max_loops = max_loops
        # the pointcloud2 rosmsg we will shoot laser at
        self.stitched_cloud = PointCloud2()

    def callback(self, in_cloud_msg):
        """
        gets stitched cloud and sets self.stitched_cloud to an rosmsg PointCloud2
        :param in_cloud_msg: ros PointCloud2 msg
        """
        self.stitched_cloud = in_cloud_msg

    def main(self):
        # subscribe to where we get the pointcloud from
        rospy.Subscriber('/cloud_rviz', PointCloud2, self.callback, queue_size=1)

        # stitches together pointcloud from all scooter cameras
        scooter = Scooter()
        scooter.go_to_fold_config()
        scooter.update_pointcloud()

        virtual_laser = VirtualLaser(self.stitched_cloud, self.radius, self.max_loops)

        listener = tf.TransformListener()

        rate = rospy.Rate(10.0)
        while not rospy.is_shutdown():
            virtual_laser.process(radius, max_loops)

            # get origin and direction from a tf frame
            try:
                (trans, rot) = listener.lookupTransform('/base_link', self.origin_frame, rospy.Time(0))
                vector_rot = virtual_laser.euler_to_vector(euler_from_quaternion(rot))
                # update laser's origin and direction based on tf frame
                virtual_laser.update_orig_and_dir(trans, vector_rot)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue

            rate.sleep()


if __name__ == '__main__':
    # radius: search radius (thickness of laser in m)
    # max_loops: how many iterations (how far the laser reaches)
    # origin_frame: tf frame that defines origin and direction of laser
    radius = 0.1
    max_loops = 20
    origin_frame = '/laser_origin'

    thing = Driver(origin_frame, radius, max_loops)
    thing.main()
