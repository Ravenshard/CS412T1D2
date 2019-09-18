#!/usr/bin/env python

import rospy
import smach
import smach_ros
from time import sleep

# The donut eating robot version 2
# this version eats only 50 donuts

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



# define state EatDonut
# class EatDonut(smach.State):
#     def __init__(self):
#         smach.State.__init__(self, outcomes=['not_full', 'full'])
#         self.counter = 0
#
#     def execute(self, userdata):
#         rospy.loginfo('Executing state EAT_DONUT')
#
#         if self.counter < 50:
#             self.counter += 1
#             sleep(2)
#             return 'not_full'
#         else:
#             self.counter = 0
#             return 'full'
#
#
# # define state GetDonut
# class GetDonut(smach.State):
#     def __init__(self):
#         smach.State.__init__(self, outcomes=['hungry'])
#
#     def execute(self, userdata):
#         rospy.loginfo('Executing state GET_DONUT')
#         sleep(2)
#         return 'hungry'




# main
def main():
    rospy.init_node('donut_botSM')

    # Create a SMACH state machine
    # sm_eat = smach.StateMachine(outcomes=['poop'])
    sm_traveller = smach.StateMachine(outcomes=['finished'])

    # Open the container
    # with sm_eat:
    # Add states to the container
    # smach.StateMachine.add('GET_DONUT', GetDonut(),
    #                     transitions={'hungry':'EAT_DONUT'})
    # smach.StateMachine.add('EAT_DONUT', EatDonut(),
    #                     transitions={'not_full':'GET_DONUT',
    #                         'full':'poop'})
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
    # outcome = sm_eat.execute()
    outcome = sm_traveller.execute()

    # wait for ctrl-c to stop the machine
    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()
