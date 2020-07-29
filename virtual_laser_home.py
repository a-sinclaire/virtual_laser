#!/usr/bin/env python
# python
import time

# ros
import rospy
from std_msgs.msg import Header as HeaderMsg
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import Joy
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler

# scipy
import numpy as np

# point cloud
import open3d as o3d


# self


class VirtualLaser:
    def __init__(self, point_cloud):
        self.origin = [0, 0, 0]
        self.direction = [0, 0, 0]
        self.stitched_pointcloud_o3d = self.rosmsg_to_o3d(point_cloud)

        # rospy.Subscriber('/joy', Joy, self.joy_callback, queue_size=1)

        self.filtered_cloud = rospy.Publisher('/filtered_stitched_cloud', PointCloud2, queue_size=1)
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

    def process(self, radius, total_loops):
        """
        publishes interesction point of virtual laser with point cloud
        :param radius: search radius (thickness of laser in m)
        :param total_loops: how many iterations (how far the laser looks)
        """

        # convert from rosmsg to o3d pointcloud
        # o3d_cloud = self.rosmsg_to_o3d(self.stitched_pointcloud)
        o3d_cloud = self.stitched_pointcloud_o3d

        # voxelfilter downsample w/ o3d
        t1 = time.time()
        o3d_cloud_filtered = o3d_cloud.voxel_down_sample(voxel_size=0.03)
        t2 = time.time()

        # running the algorithm to get collision points of laser
        t3 = time.time()
        collision_points = self.get_collision_points(o3d_cloud_filtered, self.origin, self.direction, radius,
                                                     total_loops)
        if collision_points is not None:
            centroid_point = [np.mean(collision_points, axis=0)]
            print 'centroid: '
            print centroid_point  # [[0,0,0]]
        t4 = time.time()

        #

        # vvv Everything else for visualization vvv

        # convert from o3d to rosmsg
        out_cloud_msg = self.o3d_to_rosmsg(o3d_cloud_filtered, "camera_link_2")

        # publish filtered cloud to /filtered_stitched_cloud topic
        self.filtered_cloud.publish(out_cloud_msg)

        # if a collision point was found we can publish to a topic
        if collision_points is not None:
            # publish the found collisionPoints to /laser_group_points topic
            o3d_temp = o3d.geometry.PointCloud()
            o3d_temp.points = o3d.utility.Vector3dVector(collision_points)
            target_msg = self.o3d_to_rosmsg(o3d_temp, "camera_link_2")
            self.target_group_pub.publish(target_msg)

            # publish centroid of collisionPoints to /laser_centroid topic
            o3d_temp = o3d.geometry.PointCloud()
            o3d_temp.points = o3d.utility.Vector3dVector(centroid_point)
            target_msg = self.o3d_to_rosmsg(o3d_temp, "camera_link_2")
            self.centroid_pub.publish(target_msg)

        # draw laser ray
        counter = 0
        ray_arr = []
        while counter < (total_loops * radius):
            test_point = [0, 0, 0]
            test_point[0] = self.origin[0] + (counter * self.direction[0])
            test_point[1] = self.origin[1] + (counter * self.direction[1])
            test_point[2] = self.origin[2] + (counter * self.direction[2])
            ray_arr.append(test_point)
            counter += radius / np.linalg.norm(self.direction)
        o3d_temp = o3d.geometry.PointCloud()
        o3d_temp.points = o3d.utility.Vector3dVector(ray_arr)
        target_msg = self.o3d_to_rosmsg(o3d_temp, "camera_link_2")
        self.draw_ray_pub.publish(target_msg)

        time_message = "Voxel Downsampling time: {} seconds \nAlgorithm time: {} " \
                       "seconds "
        total = "Total time: {} seconds"
        print(time_message.format(t2 - t1, t4 - t3))
        print(total.format(t2 - t1 + t4 - t3))

    def get_collision_points(self, o3d_cloud, orig, dir, threshold, total_loops=50):
        """
        travels along a defined ray and returns a a collection of points where the ray intersects a given open3d pointcloud
        :param o3d_cloud: o3d pointcloud
        :param orig: origin of search ray
        :param threshold: radius of search ray
        :param total_loops: number of checks performed
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
            test_point[0] = orig[0] + (counter * dir[0])
            test_point[1] = orig[1] + (counter * dir[1])
            test_point[2] = orig[2] + (counter * dir[2])

            tup = kd_tree.search_radius_vector_3d(test_point, threshold)
            if tup[0] > 0:  # if # of points in our k-neighborhood is greater than 0, then we hit the cloud!
                if loops < 2:
                    counter += threshold / np.linalg.norm(dir)
                    loops += 1
                    continue

                print('Found ' + str(tup[0]) + ' points within ' + str(threshold) + ' of (' + str(
                    test_point[0]) + ', ' + str(test_point[1]) + ', ' + str(test_point[2]) + ')')
                points = np.asarray([o3d_cloud.points[tup[1][k]] for k in range(0, len(tup[1]))])
                for i in range(extra_hops):  # continue another step forward to see if there are more points nearby
                    counter += threshold / np.linalg.norm(dir)
                    # update current test point
                    test_point[0] = orig[0] + (counter * dir[0])
                    test_point[1] = orig[1] + (counter * dir[1])
                    test_point[2] = orig[2] + (counter * dir[2])
                    tup = kd_tree.search_radius_vector_3d(test_point, threshold)
                    if len(tup[1]) > 0:  # still near cloud after second test point!
                        # here are the points nearby!
                        points = np.concatenate(
                            (points, np.asarray([o3d_cloud.points[tup[1][k]] for k in range(0, len(tup[1]))])), axis=0)
                    else:  # Break if the second point is not close to any points at all
                        break
                    # if points are found
                    # return list of points found
                    return points
            elif loops == total_loops:
                # if no points found after moving along ray for x loops
                # then return None
                print("Missed the pointcloud.")
                return None
            else:
                # print('No points within ' + str(threshold) + ' of (' + str(test_point[0]) + ', ' + str(
                #     test_point[1]) + ', ' + str(test_point[2]) + ')')
                counter += threshold / np.linalg.norm(dir)  # iterate counter to continue moving forward along ray
                loops += 1

    def vector_to_euler(self, arg):
        x = arg[0]
        y = arg[1]
        z = arg[2]

        roll = 0
        # pitch
        if z is not 0:
            if x < 0:
                if z < 0:
                    pitch = -np.arctan((np.sqrt((x ** 2) + (y ** 2))) / z) + np.pi
                else:
                    pitch = -np.arctan((np.sqrt((x ** 2) + (y ** 2))) / z)
            else:
                if z < 0:
                    pitch = np.arctan((np.sqrt((x ** 2) + (y ** 2))) / z) + np.pi
                else:
                    pitch = np.arctan((np.sqrt((x ** 2) + (y ** 2))) / z)
        else:
            pitch = 0
        # yaw
        if y is not 0:
            if y < 0:
                yaw = np.arctan(x / -y) + np.pi
            else:
                yaw = np.arctan(x / -y)
        else:
            yaw = 0  # np.pi/2
        return [roll, pitch, yaw]

    def euler_to_vector(self, arg):
        roll = arg[0]
        pitch = arg[1]
        yaw = arg[2]
        # msg = 'roll: {}\npitch: {}\nyaw: {}'
        # print(msg.format(roll, pitch, yaw))
        x = np.cos(yaw) * np.cos(pitch)
        y = np.sin(yaw) * np.cos(pitch)
        z = np.sin(pitch)
        # msg2 = 'x: {}\ny: {}\nz: {}'
        # print(msg2.format(x, y, z))
        return [x, y, -z]

    def euler_to_quaternion(self, arg):
        (roll, pitch, yaw) = (arg[0], arg[1], arg[2])
        qx = np.sin(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) - np.cos(roll / 2) * np.sin(pitch / 2) * np.sin(
            yaw / 2)
        qy = np.cos(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.cos(pitch / 2) * np.sin(
            yaw / 2)
        qz = np.cos(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2) - np.sin(roll / 2) * np.sin(pitch / 2) * np.cos(
            yaw / 2)
        qw = np.cos(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.sin(pitch / 2) * np.sin(
            yaw / 2)
        return [qx, qy, qz, qw]

    # def joy_callback(self, joy_msg):
    #     # joy data axes [0][1] are yaw and pitch
    #     vector_angle = self.euler_to_vector([0, joy_msg.axes[1], joy_msg.axes[0]])
    #     self.direction = vector_angle


class temp:
    def __init__(self):
        self.stitched_cloud = PointCloud2()
        self.virtual_laser = None

    def callback(self, in_cloud_msg):
        """
        gets stitched cloud and sets global_stitched cloud to an open3d pointcloud of it
        :param in_cloud_msg: ros PointCloud2 msg
        """
        self.stitched_cloud = in_cloud_msg
        self.virtual_laser = VirtualLaser(self.stitched_cloud)

    def main(self):
        rospy.init_node('virtualLaser')
        rospy.Subscriber('/camera/depth_registered/points', PointCloud2, self.callback, queue_size=1)

        origin = [0, 0, 0]
        direction = [0, 0, 0]
        radius = 0.1
        total_loops = 30

        print 'establishing cloud...'
        time.sleep(3)

        listener = tf.TransformListener()

        rate = rospy.Rate(20.0)
        while not rospy.is_shutdown():
            self.virtual_laser.process(radius, total_loops)

            # get origin and direction from a tf frame
            try:
                (trans, rot) = listener.lookupTransform('/camera_link_2', '/laser_origin', rospy.Time(0))
                self.virtual_laser.origin = trans
                self.virtual_laser.direction = self.virtual_laser.euler_to_vector(euler_from_quaternion(rot))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue

            rate.sleep()


if __name__ == '__main__':
    thing = temp()
    thing.main()