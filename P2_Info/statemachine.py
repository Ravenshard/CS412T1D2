#!/usr/bin/env python

import rospy
import smach
import smach_ros
from time import sleep
from geometry_msgs.msg import Odometry


class Wait(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['start'])

    def execute(self, userdata):
        rospy.loginfo('Executing state WAIT')
        return ''

class Forward(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['collision', 'finish'],
                                input_keys=['robotX', 'robotY'],
                                output_keys=['robotX', 'robotY'])
        self.x = 0
        self.y = 0

    def execute(self, userdata):
        rospy.loginfo('Executing state FORWARD')
        rospy.Subscriber("odom", Odometry, cb_odom)
        cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        green_light_twist = Twist()
        green_light_twist.linear.x = 0.5
        while userdata.robotX < 3:
            cmd_vel_pub.publish(green_light_twist)
            userdata.robotX += self.x
            userdata.robotY += self.y
            
        return 'finish'

    def cb_odom(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        return

class ForwardTimed(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['collision', 'successLeft', 'successRight'],
                                input_keys=['robotX', 'robotY'],
                                output_keys=['robotX', 'robotY'])

    def execute(self, userdata):
        rospy.loginfo('Executing state FORWARDTIMED')
        cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        green_light_twist = Twist()
        green_light_twist.linear.x = 0.5
        while userdata.robotX < 3:
            cmd_vel_pub.publish(green_light_twist)


class Backup(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['successLeft', 'successRight'])

    def execute(self, userdata):
        rospy.loginfo("executing state BACKUP")

class TurnRight(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["collision", "successTimed", "success"])

    def execute(self, userdata):
        rospy.loginfo("executing state TURNRIGHT")

class TurnLeft(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["collision", "successTimed", "success"])

    def execute(self, userdata):
        rospy.loginfo("executing state TURNLEFT")

# main
def main():
    rospy.init_node('robotMachine')
    rospy.Subscriber("odom", Odometry, cb_odom)
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
