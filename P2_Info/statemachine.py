#!/usr/bin/env python

import rospy
import smach
import smach_ros
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from time import sleep
from geometry_msgs.msg import Odometry

global DistanceX, DistanceY, cmd_vel_pub, heading

class Wait(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['start'])

    def execute(self, userdata):
        rospy.loginfo('Executing state WAIT')
        rospy.sleep(5)
        return 'start'

class Forward(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['collision', 'finish'])

    def execute(self, userdata):
        global heading
        twist = Twist()
        global cmd_vel_pub, DistanceX

        while DistanceX < 3:
            target_heading = (heading + 90) % 360

            turning = True
            previous_difference = None
            while turning:
                twist.linear.x = 0.5
                cmd_vel_pub.publish(twist)

                difference = minimum_angle_between_headings(target_heading, heading)
                if previous_difference is None:
                    twist.angular.z = 0.2
                    cmd_vel_pub.publish(twist)
                else:
                    if previous_difference < difference:
                        print("Done turn")
                        turning = False
                        twist.angular.z = 0
                        cmd_vel_pub.publish(twist)
                    else:
                        twist.angular.z = 0.2
                        cmd_vel_pub.publish(twist)

                if previous_difference != difference:
                    print("target"+str(target_heading))
                    print("yaw"+str(heading))
                    print("Difference"+str(difference))
                    previous_difference = difference
            rospy.sleep(5)


class ForwardTimed(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['collision', 'successLeft', 'successRight'])

    def execute(self, userdata):
        rospy.loginfo('Executing state FORWARDTIMED')
        global heading
        twist = Twist()
        global cmd_vel_pub
        executeStart = rospy.Time.now()
        duration = 3
        while rospy.Time.now() < executeStart + duration:
            twist.linear.x = 0.5
            cmd_vel_pub.publish(twist)
            difference = minimum_angle_between_headings(target_heading, heading)

            if previous_difference is None:
                twist.angular.z = 0.2
                cmd_vel_pub.publish(twist)
            else:
                if previous_difference < difference:
                    print("Done turn")
                    turning = False
                    twist.angular.z = 0
                    cmd_vel_pub.publish(twist)
                else:
                    twist.angular.z = 0.2
                    cmd_vel_pub.publish(twist)

            if previous_difference != difference:
                print("target"+str(target_heading))
                print("yaw"+str(heading))
                print("Difference"+str(difference))
                previous_difference = difference


class Backup(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['successLeft', 'successRight'])

    def execute(self, userdata):
        rospy.loginfo("executing state BACKUP")
        rospy.loginfo('Executing state FORWARDTIMED')
        global heading
        twist = Twist()
        global cmd_vel_pub
        executeStart = rospy.Time.now()
        duration = 3
        while rospy.Time.now() < executeStart + duration:
            twist.linear.x = -0.2
            cmd_vel_pub.publish(twist)



class TurnRight(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["collision", "successTimed", "success"])

    def execute(self, userdata):
        rospy.loginfo("executing state TURNRIGHT")

        global heading
        twist = Twist()
        global cmd_vel_pub

        target_heading = heading - 90
        if target_heading < 0:
            target_heading = target_heading + 360

        turning = True
        previous_difference = None
        while turning:
            difference = minimum_angle_between_headings(target_heading, heading)

            if previous_difference is None:
                twist.angular.z = -0.2
                cmd_vel_pub.publish(twist)
            else:
                if previous_difference < difference:
                    turning = False
                    twist.angular.z = 0
                    cmd_vel_pub.publish(twist)
                else:
                    twist.angular.z = -0.2
                    cmd_vel_pub.publish(twist)

            if previous_difference != difference:
                previous_difference = difference
        return "success"

class TurnLeft(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["collision", "successTimed", "success"])

    def execute(self, userdata):
        rospy.loginfo("executing state TURNLEFT")

        global heading
        twist = Twist()
        global cmd_vel_pub

        target_heading = (heading + 90) % 360

        turning = True
        previous_difference = None
        while turning:
            difference = minimum_angle_between_headings(target_heading, heading)

            if previous_difference is None:
                twist.angular.z = 0.2
                cmd_vel_pub.publish(twist)
            else:
                if previous_difference < difference:
                    turning = False
                    twist.angular.z = 0
                    cmd_vel_pub.publish(twist)
                else:
                    twist.angular.z = 0.2
                    cmd_vel_pub.publish(twist)

            if previous_difference != difference:
                previous_difference = difference
        return "success"


def minimum_angle_between_headings(a, b):
    heading_difference = a - b
    if heading_difference < 0:
        heading_difference += 360
    if heading_difference > 180:
        heading_difference = b - a
        if heading_difference < 0:
            heading_difference += 360
    return heading_difference



def odom_callback(msg):
    global heading
    yaw = euler_from_quaternion([
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w
    ])[2]
    #print("yaw"+str(yaw))
    heading = (yaw + math.pi)*(180/math.pi)
    #print("Heading"+str(heading))

# def bumper_callback(msg):
    # global

# main
def main():
    #print(minimum_angle_between_headings(0, (3 * math.pi) / 2))
    #return

    rospy.init_node('bumper_bot')

    global cmd_vel_pub
    cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=1)
    odom_sub = rospy.Subscriber("odom", Odometry, odom_callback)

    sm_traveller = smach.StateMachine(outcomes=['finished'])
    sm_traveller.userdata.robotX = 0
    sm_traveller.userdata.robotY = 0

    with sm_traveller:
        smach.StateMachine.add('WAIT', Wait(),
                               transitions={'start': 'FORWARD'})
        smach.StateMachine.add('FORWARD', Forward(),
                               transitions={'collision': 'BACKUP',
                                            'finish': 'WAIT'})
        smach.StateMachine.add('FORWARDTIMED', ForwardTimed(),
                               transitions={'collision': 'BACKUP',
                                            "successLeft":'TURNLEFT',
                                            'successRight':'TURNRIGHT'})
        smach.StateMachine.add('BACKUP', Backup(),
                               transitions={'successLeft':'TURNLEFT',
                                            'successRight': 'TURNRIGHT'})
        smach.StateMachine.add('TURNRIGHT', TurnRight(),
                               transitions={'collision':'BACKUP',
                                            'success':'FORWARD',
                                            'successTimed': 'FORWARDTIMED'})
        smach.StateMachine.add('TURNLEFT', TurnLeft(),
                               transitions={'collision':'BACKUP',
                                            'success':'FORWARD',
                                            'successTimed': 'FORWARDTIMED'})


    # Create and start the instrospection server - needed for smach_viewer
    sis = smach_ros.IntrospectionServer('TRAVELLER_server', sm_traveller, 'STATEMACHINE')
    sis.start()



    # Execute SMACH plan
    outcome = sm_traveller.execute()
    # wait for ctrl-c to stop the machine
    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()
