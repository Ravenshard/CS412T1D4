#!/usr/bin/env python

import signal
import rospy
import smach
import smach_ros
import math
from math import tanh
from geometry_msgs.msg import Twist
import numpy as np
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Joy

global button_start
global shutdown_requested


class Wait(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['box1', 'box2', 'box3', 'box4',
                                            'box5', 'box6', 'box7', 'box8'])

    def execute(self, userdata):
        return 'box1'

class Box1(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['wait', 'box2', 'box3', 'box4',
                                            'box5', 'box6', 'box7', 'box8'])

    def execute(self, userdata):
        return 'wait'

class Box2(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['wait', 'box1', 'box3', 'box4',
                                            'box5', 'box6', 'box7', 'box8'])

    def execute(self, userdata):
        return 'wait'

class Box3(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['wait', 'box2', 'box1', 'box4',
                                            'box5', 'box6', 'box7', 'box8'])

    def execute(self, userdata):
        return 'wait'

class Box4(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['wait', 'box2', 'box3', 'box1',
                                            'box5', 'box6', 'box7', 'box8'])

    def execute(self, userdata):
        return 'wait'

class Box5(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['wait', 'box2', 'box3', 'box4',
                                            'box1', 'box6', 'box7', 'box8'])

    def execute(self, userdata):
        return 'wait'

class Box6(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['wait', 'box2', 'box3', 'box4',
                                            'box5', 'box1', 'box7', 'box8'])

    def execute(self, userdata):
        return 'wait'

class Box7(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['wait', 'box2', 'box3', 'box4',
                                            'box5', 'box6', 'box1', 'box8'])

    def execute(self, userdata):
        return 'wait'

class Box8(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['wait', 'box2', 'box3', 'box4',
                                            'box5', 'box6', 'box7', 'box1'])

    def execute(self, userdata):
        return 'wait'

def controller_callback(event):
    global button_start
    if event.buttons[1] == 1:
        button_start = not button_start


def request_shutdown(sig, frame):
    global shutdown_requested
    shutdown_requested = True


def main():
    global button_start
    global shutdown_requested
    button_start = False
    shutdown_requested = False

    rospy.init_node('cop_bot')
    controller_sub = rospy.Subscriber('joy', Joy, controller_callback)

    # Create done outcome which will stop the state machine
    sm_turtle = smach.StateMachine(outcomes=['DONE'])

    with sm_turtle:
        smach.StateMachine.add('WAIT', Wait(),
                               transitions={'box1': 'BOX1', 'box2': 'BOX2',
                                    'box3': 'BOX3', 'box4': 'BOX4',
                                    'box5': 'BOX5', 'box6': 'BOX6',
                                    'box7': 'BOX7', 'box8': 'BOX8'})
        smach.StateMachine.add('BOX1', Box1(),
                               transitions={'wait': 'WAIT', 'box2': 'BOX2',
                                    'box3': 'BOX3', 'box4': 'BOX4',
                                    'box5': 'BOX5', 'box6': 'BOX6',
                                    'box7': 'BOX7', 'box8': 'BOX8'})
        smach.StateMachine.add('BOX2', Box2(),
                               transitions={'wait': 'WAIT', 'box1': 'BOX1',
                                    'box3': 'BOX3', 'box4': 'BOX4',
                                    'box5': 'BOX5', 'box6': 'BOX6',
                                    'box7': 'BOX7', 'box8': 'BOX8'})
        smach.StateMachine.add('BOX3', Box3(),
                               transitions={'wait': 'WAIT', 'box2': 'BOX2',
                                    'box1': 'BOX1', 'box4': 'BOX4',
                                    'box5': 'BOX5', 'box6': 'BOX6',
                                    'box7': 'BOX7', 'box8': 'BOX8'})
        smach.StateMachine.add('BOX4', Box4(),
                               transitions={'wait': 'WAIT', 'box2': 'BOX2',
                                    'box3': 'BOX3', 'box1': 'BOX1',
                                    'box5': 'BOX5', 'box6': 'BOX6',
                                    'box7': 'BOX7', 'box8': 'BOX8'})
        smach.StateMachine.add('BOX5', Box5(),
                               transitions={'wait': 'WAIT', 'box2': 'BOX2',
                                    'box3': 'BOX3', 'box4': 'BOX4',
                                    'box1': 'BOX1', 'box6': 'BOX6',
                                    'box7': 'BOX7', 'box8': 'BOX8'})
        smach.StateMachine.add('BOX6', Box6(),
                               transitions={'wait': 'WAIT', 'box2': 'BOX2',
                                    'box3': 'BOX3', 'box4': 'BOX4',
                                    'box5': 'BOX5', 'box1': 'BOX1',
                                    'box7': 'BOX7', 'box8': 'BOX8'})
        smach.StateMachine.add('BOX7', Box7(),
                               transitions={'wait': 'WAIT', 'box2': 'BOX2',
                                    'box3': 'BOX3', 'box4': 'BOX4',
                                    'box5': 'BOX5', 'box6': 'BOX6',
                                    'box1': 'BOX1', 'box8': 'BOX8'})
        smach.StateMachine.add('BOX8', Box8(),
                               transitions={'wait': 'WAIT', 'box2': 'BOX2',
                                    'box3': 'BOX3', 'box4': 'BOX4',
                                    'box5': 'BOX5', 'box6': 'BOX6',
                                    'box7': 'BOX7', 'box1': 'BOX1'})

    # Create and start the instrospection server - needed for smach_viewer
    sis = smach_ros.IntrospectionServer('BOX_server', sm_turtle, 'STATEMACHINE')
    sis.start()

    # Start state machine and run until SIGINT received
    signal.signal(signal.SIGINT, request_shutdown)
    sm_turtle.execute()

    # Stop server
    sis.stop()


if __name__ == '__main__':
    main()
