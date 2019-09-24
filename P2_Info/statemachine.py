#!/usr/bin/env python

import rospy
import smach
import smach_ros
import math
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry
from kobuki_msgs.msg import BumperEvent
from kobuki_msgs.msg import LED

global DistanceX, DistX, cmd_vel_pub, heading, bumperState, prevState
DistanceX = 0
bumperState = (0,0)

class Wait(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['start'])

    def execute(self, userdata):
        rospy.loginfo('Executing state WAIT')
        global prevState
        if prevState == 'Foward':
            led = LED()
            led.value = 2
            led_pub.publish(led)
            # turn light on
        else:
            led = LED()
            led.value = 3
            led_pub.publish(led)
        rospy.sleep(2)
        prevState = 'Wait'
        return 'start'

class Forward(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['collision', 'finish'])

    def execute(self, userdata):
        twist = Twist()
        global cmd_vel_pub, distX, bumperState, heading, prevState
        prevState = 'Forward'
        while distX < 3.2:
            twist.linear.x = 0.5
            cmd_vel_pub.publish(twist)
            if bumperState[1] == 1:
                rospy.loginfo("bumper hit")
                return 'collision'
        return 'finish'


class ForwardTimed(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['collision', 'successLeft', 'successRight'])

    def execute(self, userdata):
        rospy.loginfo('Executing state FORWARDTIMED')
        twist = Twist()
        global cmd_vel_pub, prevState
        executeStart = rospy.Time.now()
        duration = rospy.Duration(3)
        while rospy.Time.now() < executeStart + duration:
            twist.linear.x = 0.5
            cmd_vel_pub.publish(twist)
            if bumperState[1] == 1:
                prevState = 'ForwardTimed'
                rospy.loginfo("bumper hit")
                return 'collision'

        if prevState == 'TurnRight':
            prevState = 'ForwardTimed'
            return 'successLeft'
        elif prevState == 'TurnLeft':
            prevState = 'ForwardTimed'
            return 'successRight'



class Backup(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['successLeft', 'successRight'])

    def execute(self, userdata):
        rospy.loginfo("executing state BACKUP")
        global cmd_vel_pub, bumperState, prevState
        prevState = 'Backup'
        twist = Twist()
        executeStart = rospy.Time.now()
        duration = rospy.Duration(3)
        while rospy.Time.now() < executeStart + duration:
            twist.linear.x = -0.2
            cmd_vel_pub.publish(twist)

        if bumperState[0] == 0: return 'successRight'
        elif bumperState[0] == 1: return 'successLeft'
        elif bumperState[0] == 2: return 'successLeft'


class TurnRight(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["collision", "successTimed", "success"])

    def execute(self, userdata):
        rospy.loginfo("executing state TURNRIGHT")

        twist = Twist()
        global cmd_vel_pub, heading, prevState

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
                if difference < 0.5:
                    turning = False
                    twist.angular.z = 0
                    cmd_vel_pub.publish(twist)
                else:
                    twist.angular.z = -0.2
                    cmd_vel_pub.publish(twist)

            if previous_difference != difference:
                previous_difference = difference
        if prevState == 'Backup':
            prevState = 'TurnRight'
            return' successTimed'
        elif prevState == 'ForwardTimed':
            prevState = 'TurnRight'
            return 'success'


class TurnLeft(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["collision", "successTimed", "success"])

    def execute(self, userdata):
        rospy.loginfo("executing state TURNLEFT")

        global heading
        twist = Twist()
        global cmd_vel_pub
        global prevState

        target_heading = (heading + 90) % 360

        turning = True
        previous_difference = None
        while turning:
            difference = minimum_angle_between_headings(target_heading, heading)

            if previous_difference is None:
                twist.angular.z = 0.2
                cmd_vel_pub.publish(twist)
            else:
                if difference < 0.5:
                    turning = False
                    twist.angular.z = 0
                    cmd_vel_pub.publish(twist)
                else:
                    twist.angular.z = 0.2
                    cmd_vel_pub.publish(twist)

            if previous_difference != difference:
                previous_difference = difference
        if prevState == 'Backup':
            prevState = 'TurnLeft'
            return'successTimed'
        elif prevState == 'ForwardTimed':
            prevState = 'TurnLeft'
            return 'success'


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
    global heading, distX
    yaw = euler_from_quaternion([
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w
    ])[2]
    heading = (yaw + math.pi)*(180/math.pi)
    distX = msg.pose.pose.position.x


def bumper_callback(msg):
    global bumperState
    bumper = msg.bumper
    state = msg.state
    rospy.loginfo("bumper published! bumper: {} state: {}".format(bumper, state))
    bumperState = (bumper, state)


# main
def main():
    rospy.init_node('bumper_bot')

    global cmd_vel_pub
    cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=1)
    led_pub = reospy.Publisher("/mobile_base/commands/led1", LED, queue_size=1)
    odom_sub = rospy.Subscriber("odom", Odometry, odom_callback)
    bum_sub = rospy.Subscriber("/mobile_base/events/bumper", BumperEvent, bumper_callback)

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
