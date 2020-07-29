import rospy
import tf

if __name__ == '__main__':
    rospy.init_node('camrea_orient')

    br = tf.TransformBroadcaster()

    origin = [0, 0, 0]
    frame_dir = [ -0.5, 0.5, -0.5, 0.5 ]

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        br.sendTransform(origin, frame_dir, rospy.Time.now(), "/camera_link_2", "/camera_link")
        rate.sleep()
