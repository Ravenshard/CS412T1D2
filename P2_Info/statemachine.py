#!/usr/bin/env python

import rospy
import smach
import smach_ros
from time import sleep

global DistanceX, DistanceY

class Wait(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['start'])

    def execute(self, userdata):
        rospy.loginfo('Executing state WAIT')
        return ''

class Forward(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['collision', 'finish'])

    def execute(self, userdata):
        rospy.loginfo('Executing state FORWARD')

class ForwardTimed(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['collision', 'successLeft', 'successRight'])

    def execute(self, userdata):
        rospy.loginfo('Executing state FORWARDTIMED')

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
    rospy.init_node('donut_botSM')

    sm_traveller = smach.StateMachine(outcomes=['finished'])

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
