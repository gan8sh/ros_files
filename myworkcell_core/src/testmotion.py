#!/usr/bin/python
#
# Send joint values to UR5 using messages
#

from std_msgs.msg import Header
from trajectory_msgs.msg import JointTrajectory

from trajectory_msgs.msg import JointTrajectoryPoint
import rospy
import math

in_array=[-3.14159265,-1.57039399,-0.78559933,-0.39269908169,0.39269908169,0.78559933,1.57039399,3.14159265]

#waypoints ={}

#for i in range(len(in_array)):
#    waypoints[i] = [0.0, in_array[i],in_array[i],in_array[i],0,in_array[i]]
#    i += 1
waypoints = [[0.0,-1.57,1.57,0.78,0,-0.39],[0.0,0,-1.57,0.0,0.0,0.39]]
#waypoints = [[0,1,2,3,4,5],[0.0,1.57,3.14,0.0,0.0,0.39]]

#print(waypoints)
def main():

    rospy.init_node('send_joints')
    pub = rospy.Publisher('/joint_path_command',
                          JointTrajectory,
                          queue_size=10)

    # Create the topic message
    traj = JointTrajectory()
    traj.header = Header()
    # Joint names for UR5
    traj.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint',
                        'elbow_joint', 'wrist_1_joint', 'wrist_2_joint',
                        'wrist_3_joint']

    rate = rospy.Rate(1)
    cnt = 0
    pts = JointTrajectoryPoint()
    traj.header.stamp = rospy.Time.now()

    while not rospy.is_shutdown():
        cnt += 1

        if cnt%2 == 1:
            pts.positions = waypoints[0]
            #pts.effort = waypoints[0]
        else:
            pts.positions = waypoints[1]
            #pts.effort = waypoints[0]

        pts.time_from_start = rospy.Duration(1.0)

        # Set the points to the trajectory
        traj.points = []
        traj.points.append(pts)
        # Publish the message
        pub.publish(traj)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        print ("Program interrupted before completion")
