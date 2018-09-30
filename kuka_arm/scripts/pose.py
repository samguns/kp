#!/usr/bin/env python

import rospy
import tf
from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose


def handle_demo_poses(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print("No valid poses received")
        return -1
    else:
        for x in xrange(0, len(req.poses)):
            joint_trajectory_point = JointTrajectoryPoint()
            px = req.poses[x].position.x
            py = req.poses[x].position.y
            pz = req.poses[x].position.z

            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                 req.poses[x].orientation.z, req.poses[x].orientation.w])

            print("x ", px, " y ", py, " z ", pz, " roll ", roll, " pitch ", pitch, " yaw ", yaw)
            print("")

        joint_trajectory_list = []
        return CalculateIKResponse(joint_trajectory_list)


def pose_server():
    # initialize node and declare calculate_ik service
    rospy.init_node('pose_server')
    s = rospy.Service('calculate_ik', CalculateIK, handle_demo_poses)
    print("Ready to receive an IK request")
    rospy.spin()


if __name__ == "__main__":
    pose_server()