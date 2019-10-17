#!/usr/bin/env python

import signal
import rospy
import smach
import smach_ros
import math
from geometry_msgs.msg import Twist
import numpy as np
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Joy
from geometry_msgs.msg import PoseWithCovarianceStamped
import time
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

global shutdown_requested


'''
header: 
  seq: 125
  stamp: 
    secs: 1571336109
    nsecs: 238852016
  frame_id: "map"
pose: 
  pose: 
    position: 
      x: -1.4029582974
      y: -0.161230396952
      z: 0.0
    orientation: 
      x: 0.0
      y: 0.0
      z: -0.409603401919
      w: 0.912263697149
  covariance: [0.003383795001103751, 0.0030187297400489366, 0.0, 0.0, 0.0, 0.0, 0.0030187297400489366, 0.0106325757067692, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0017897886634780384]
'''


class Wait(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['box1', 'box2', 'box3', 'box4',
                                             'box5', 'box6', 'box7', 'box8', 'done'])
        self.initial_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=10)
        self.stateSwitch = None

        initial = PoseWithCovarianceStamped()
        initial.pose.pose.position.x = -1.3
        initial.pose.pose.position.y = -0
        initial.pose.pose.position.z = 0.0
        initial.pose.pose.orientation.x = 0.0
        initial.pose.pose.orientation.y = 0.0
        initial.pose.pose.orientation.z = -0.384482867487
        initial.pose.pose.orientation.w = 0.923132127384
        time.sleep(2)  # Do not remove or message will not publish!
        #self.initial_pub.publish(initial)

    def execute(self, userdata):
        global shutdown_requested
        joy_sub = rospy.Subscriber('joy', Joy, self.controller_callback)
        self.stateSwitch = None
        while not shutdown_requested:
            rospy.sleep(1)
            if self.stateSwitch is not None:
                rospy.loginfo("Switching to state BOX{}".format(self.stateSwitch))
                joy_sub.unregister()
                return 'box{}'.format(self.stateSwitch)
        joy_sub.unregister()
        return 'done'

    def controller_callback(self, event):
        if event.buttons[1] == 1:     self.stateSwitch = 1
        elif event.buttons[2] == 1:  self.stateSwitch = 2
        elif event.buttons[3] == 1:  self.stateSwitch = 3
        elif event.buttons[0] == 1:  self.stateSwitch = 4
        elif event.axes[0] == -1:    self.stateSwitch = 5
        elif event.axes[1] == 1:     self.stateSwitch = 6
        elif event.axes[0] == 1:     self.stateSwitch = 7
        elif event.axes[1] == -1:    self.stateSwitch = 8
        return

class Box1(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['wait', 'box2', 'box3', 'box4',
                                             'box5', 'box6', 'box7', 'box8', 'done'])
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()

        self.target = MoveBaseGoal()
        self.target.target_pose.header.frame_id = "map"
        self.target.target_pose.header.stamp = rospy.Time.now()
        self.target.target_pose.pose.position.x = -1.86572667681
        self.target.target_pose.pose.position.y = -1.23889378088
        self.target.target_pose.pose.orientation.x = 0.0
        self.target.target_pose.pose.orientation.y = 0.0
        self.target.target_pose.pose.orientation.z = -0.372922294621
        self.target.target_pose.pose.orientation.w = 0.927862577203

        self.stateSwitch = 'wait'

    def execute(self, userdata):
        global shutdown_requested
        joy_sub = rospy.Subscriber('joy', Joy, self.controller_callback)
        self.stateSwitch = 'wait'

        self.client.send_goal(self.target)
        self.client.wait_for_result()
        print("Goal reached")

        joy_sub.unregister()
        if shutdown_requested:
            return 'done'
        if self.stateSwitch != 'wait':
            return 'box{}'.format(self.stateSwitch)
        else:
            return 'wait'

    def controller_callback(self, event):
        if event.buttons[6] == 1 or event.buttons[7] == 1 : self.stateSwitch = 'wait'
        elif event.buttons[2] == 1:  self.stateSwitch = 2
        elif event.buttons[3] == 1:  self.stateSwitch = 3
        elif event.buttons[0] == 1:  self.stateSwitch = 4
        elif event.axes[0] == -1:    self.stateSwitch = 5
        elif event.axes[1] == 1:     self.stateSwitch = 6
        elif event.axes[0] == 1:     self.stateSwitch = 7
        elif event.axes[1] == -1:    self.stateSwitch = 8
        return

class Box2(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['wait', 'box1', 'box3', 'box4',
                                             'box5', 'box6', 'box7', 'box8', 'done'])
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()

        self.target = MoveBaseGoal()
        self.target.target_pose.header.frame_id = "map"
        self.target.target_pose.header.stamp = rospy.Time.now()
        self.target.target_pose.pose.position.x = -2.48666366809
        self.target.target_pose.pose.position.y = -1.87504164082
        self.target.target_pose.pose.orientation.x = 0.0
        self.target.target_pose.pose.orientation.y = 0.0
        self.target.target_pose.pose.orientation.z = -0.372922294621
        self.target.target_pose.pose.orientation.w = 0.927862577203

        self.stateSwitch = 'wait'

    def execute(self, userdata):
        global shutdown_requested
        joy_sub = rospy.Subscriber('joy', Joy, self.controller_callback)
        self.stateSwitch = 'wait'

        self.client.send_goal(self.target)
        self.client.wait_for_result()
        print("Goal reached")

        joy_sub.unregister()
        if shutdown_requested:
            return 'done'
        if self.stateSwitch != 'wait':
            return 'box{}'.format(self.stateSwitch)
        else:
            return 'wait'

    def controller_callback(self, event):
        if event.buttons[6] == 1 or event.buttons[7] == 1 : self.stateSwitch = 'wait'
        elif event.buttons[1] == 1:  self.stateSwitch = 1
        elif event.buttons[3] == 1:  self.stateSwitch = 3
        elif event.buttons[0] == 1:  self.stateSwitch = 4
        elif event.axes[0] == -1:    self.stateSwitch = 5
        elif event.axes[1] == 1:     self.stateSwitch = 6
        elif event.axes[0] == 1:     self.stateSwitch = 7
        elif event.axes[1] == -1:    self.stateSwitch = 8
        return

class Box3(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['wait', 'box2', 'box1', 'box4',
                                             'box5', 'box6', 'box7', 'box8', 'done'])
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()

        self.target = MoveBaseGoal()
        self.target.target_pose.header.frame_id = "map"
        self.target.target_pose.header.stamp = rospy.Time.now()
        self.target.target_pose.pose.position.x = -1.81187055824
        self.target.target_pose.pose.position.y = -2.37934982584
        self.target.target_pose.pose.orientation.x = 0.0
        self.target.target_pose.pose.orientation.y = 0.0
        self.target.target_pose.pose.orientation.z = -0.372922294621
        self.target.target_pose.pose.orientation.w = 0.927862577203

        self.stateSwitch = 'wait'

    def execute(self, userdata):
        global shutdown_requested
        joy_sub = rospy.Subscriber('joy', Joy, self.controller_callback)
        self.stateSwitch = 'wait'

        self.client.send_goal(self.target)
        self.client.wait_for_result()
        print("Goal reached")

        joy_sub.unregister()
        if shutdown_requested:
            return 'done'
        if self.stateSwitch != 'wait':
            return 'box{}'.format(self.stateSwitch)
        else:
            return 'wait'

    def controller_callback(self, event):
        if event.buttons[6] == 1 or event.buttons[7] == 1 : self.stateSwitch = 'wait'
        elif event.buttons[2] == 1:  self.stateSwitch = 2
        elif event.buttons[1] == 1:  self.stateSwitch = 1
        elif event.buttons[0] == 1:  self.stateSwitch = 4
        elif event.axes[0] == -1:    self.stateSwitch = 5
        elif event.axes[1] == 1:     self.stateSwitch = 6
        elif event.axes[0] == 1:     self.stateSwitch = 7
        elif event.axes[1] == -1:    self.stateSwitch = 8

class Box4(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['wait', 'box2', 'box3', 'box1',
                                             'box5', 'box6', 'box7', 'box8', 'done'])
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()

        self.target = MoveBaseGoal()
        self.target.target_pose.header.frame_id = "map"
        self.target.target_pose.header.stamp = rospy.Time.now()
        self.target.target_pose.pose.position.x = -0.617115325015
        self.target.target_pose.pose.position.y = -1.74971739038
        self.target.target_pose.pose.orientation.x = 0.0
        self.target.target_pose.pose.orientation.y = 0.0
        self.target.target_pose.pose.orientation.z = -0.372922294621
        self.target.target_pose.pose.orientation.w = 0.927862577203

        self.stateSwitch = 'wait'



    def execute(self, userdata):
        global shutdown_requested
        joy_sub = rospy.Subscriber('joy', Joy, self.controller_callback)
        self.stateSwitch = 'wait'

        self.client.send_goal(self.target)
        self.client.wait_for_result()
        print("Goal reached")

        joy_sub.unregister()
        if shutdown_requested:
            return 'done'
        if self.stateSwitch != 'wait':
            return 'box{}'.format(self.stateSwitch)
        else:
            return 'wait'

    def controller_callback(self, event):
        if event.buttons[6] == 1 or event.buttons[7] == 1 : self.stateSwitch = 'wait'
        elif event.buttons[2] == 1:  self.stateSwitch = 2
        elif event.buttons[3] == 1:  self.stateSwitch = 3
        elif event.buttons[1] == 1:  self.stateSwitch = 1
        elif event.axes[0] == -1:    self.stateSwitch = 5
        elif event.axes[1] == 1:     self.stateSwitch = 6
        elif event.axes[0] == 1:     self.stateSwitch = 7
        elif event.axes[1] == -1:    self.stateSwitch = 8

class Box5(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['wait', 'box2', 'box3', 'box4',
                                             'box1', 'box6', 'box7', 'box8', 'done'])
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()

        self.target = MoveBaseGoal()
        self.target.target_pose.header.frame_id = "map"
        self.target.target_pose.header.stamp = rospy.Time.now()
        self.target.target_pose.pose.position.x = -0.792065221849
        self.target.target_pose.pose.position.y = -3.43328318514
        self.target.target_pose.pose.orientation.x = 0.0
        self.target.target_pose.pose.orientation.y = 0.0
        self.target.target_pose.pose.orientation.z = -0.372922294621
        self.target.target_pose.pose.orientation.w = 0.927862577203

        self.stateSwitch = 'wait'

    def execute(self, userdata):
        global shutdown_requested
        joy_sub = rospy.Subscriber('joy', Joy, self.controller_callback)
        self.stateSwitch = 'wait'

        self.client.send_goal(self.target)
        self.client.wait_for_result()
        print("Goal reached")

        joy_sub.unregister()
        if shutdown_requested:
            return 'done'
        if self.stateSwitch != 'wait':
            return 'box{}'.format(self.stateSwitch)
        else:
            return 'wait'

    def controller_callback(self, event):
        if event.buttons[6] == 1 or event.buttons[7] == 1 : self.stateSwitch = 'wait'
        elif event.buttons[2] == 1:  self.stateSwitch = 2
        elif event.buttons[3] == 1:  self.stateSwitch = 3
        elif event.buttons[0] == 1:  self.stateSwitch = 4
        elif event.buttons[1] == 1:  self.stateSwitch = 1
        elif event.axes[1] == 1:     self.stateSwitch = 6
        elif event.axes[0] == 1:     self.stateSwitch = 7
        elif event.axes[1] == -1:    self.stateSwitch = 8

class Box6(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['wait', 'box2', 'box3', 'box4',
                                             'box5', 'box1', 'box7', 'box8', 'done'])
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()

        self.target = MoveBaseGoal()
        self.target.target_pose.header.frame_id = "map"
        self.target.target_pose.header.stamp = rospy.Time.now()
        self.target.target_pose.pose.position.x = -0.45466633388
        self.target.target_pose.pose.position.y = -3.91585715022
        #self.target.target_pose.pose.position.x = -0.344961246087
        #self.target.target_pose.pose.position.y = -4.06018585631
        self.target.target_pose.pose.orientation.x = 0.0
        self.target.target_pose.pose.orientation.y = 0.0
        self.target.target_pose.pose.orientation.z = -0.372922294621
        self.target.target_pose.pose.orientation.w = 0.927862577203

        self.stateSwitch = 'wait'

    def execute(self, userdata):
        global shutdown_requested
        joy_sub = rospy.Subscriber('joy', Joy, self.controller_callback)
        self.stateSwitch = 'wait'

        self.client.send_goal(self.target)
        self.client.wait_for_result()
        print("Goal reached")

        joy_sub.unregister()
        if shutdown_requested:
            return 'done'
        if self.stateSwitch != 'wait':
            return 'box{}'.format(self.stateSwitch)
        else:
            return 'wait'

    def controller_callback(self, event):
        if event.buttons[6] == 1 or event.buttons[7] == 1 : self.stateSwitch = 'wait'
        elif event.buttons[2] == 1:  self.stateSwitch = 2
        elif event.buttons[3] == 1:  self.stateSwitch = 3
        elif event.buttons[0] == 1:  self.stateSwitch = 4
        elif event.axes[0] == -1:    self.stateSwitch = 5
        elif event.buttons[1] == 1:  self.stateSwitch = 1
        elif event.axes[0] == 1:     self.stateSwitch = 7
        elif event.axes[1] == -1:    self.stateSwitch = 8

class Box7(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['wait', 'box2', 'box3', 'box4',
                                             'box5', 'box6', 'box1', 'box8', 'done'])
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()

        self.target = MoveBaseGoal()
        self.target.target_pose.header.frame_id = "map"
        self.target.target_pose.header.stamp = rospy.Time.now()
        self.target.target_pose.pose.position.x = -0.110382720208
        self.target.target_pose.pose.position.y = -2.42328070669
        self.target.target_pose.pose.orientation.x = 0.0
        self.target.target_pose.pose.orientation.y = 0.0
        self.target.target_pose.pose.orientation.z = -0.372922294621
        self.target.target_pose.pose.orientation.w = 0.927862577203

        self.stateSwitch = 'wait'

    def execute(self, userdata):
        global shutdown_requested
        joy_sub = rospy.Subscriber('joy', Joy, self.controller_callback)
        self.stateSwitch = 'wait'

        self.client.send_goal(self.target)
        self.client.wait_for_result()
        print("Goal reached")

        joy_sub.unregister()
        if shutdown_requested:
            return 'done'
        if self.stateSwitch != 'wait':
            return 'box{}'.format(self.stateSwitch)
        else:
            return 'wait'

    def controller_callback(self, event):
        if event.buttons[6] == 1 or event.buttons[7] == 1 : self.stateSwitch = 'wait'
        elif event.buttons[2] == 1:  self.stateSwitch = 2
        elif event.buttons[3] == 1:  self.stateSwitch = 3
        elif event.buttons[0] == 1:  self.stateSwitch = 4
        elif event.axes[0] == -1:    self.stateSwitch = 5
        elif event.axes[1] == 1:     self.stateSwitch = 6
        elif event.buttons[1] == 1:     self.stateSwitch = 1
        elif event.axes[1] == -1:    self.stateSwitch = 8

class Box8(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['wait', 'box2', 'box3', 'box4',
                                             'box5', 'box6', 'box7', 'box1', 'done'])
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()

        self.target = MoveBaseGoal()
        self.target.target_pose.header.frame_id = "map"
        self.target.target_pose.header.stamp = rospy.Time.now()
        self.target.target_pose.pose.position.x = -1.3672780018
        self.target.target_pose.pose.position.y = -2.99154837699
        self.target.target_pose.pose.orientation.x = 0.0
        self.target.target_pose.pose.orientation.y = 0.0
        self.target.target_pose.pose.orientation.z = -0.372922294621
        self.target.target_pose.pose.orientation.w = 0.927862577203

        self.stateSwitch = 'wait'

    def execute(self, userdata):
        global shutdown_requested
        joy_sub = rospy.Subscriber('joy', Joy, self.controller_callback)
        self.stateSwitch = 'wait'

        self.client.send_goal(self.target)
        self.client.wait_for_result()
        print("Goal reached")

        joy_sub.unregister()
        if shutdown_requested:
            return 'done'
        if self.stateSwitch != 'wait':
            return 'box{}'.format(self.stateSwitch)
        else:
            return 'wait'

    def controller_callback(self, event):
        if event.buttons[6] == 1 or event.buttons[7] == 1 : self.stateSwitch = 'wait'
        elif event.buttons[2] == 1:  self.stateSwitch = 2
        elif event.buttons[3] == 1:  self.stateSwitch = 3
        elif event.buttons[0] == 1:  self.stateSwitch = 4
        elif event.axes[0] == -1:    self.stateSwitch = 5
        elif event.axes[1] == 1:     self.stateSwitch = 6
        elif event.axes[0] == 1:     self.stateSwitch = 7
        elif event.buttons[1] == 1:  self.stateSwitch = 1

def request_shutdown(sig, frame):
    global shutdown_requested
    shutdown_requested = True

def main():
    global button_start
    global shutdown_requested
    button_start = False
    shutdown_requested = False

    rospy.init_node('box_map')

    # Create done outcome which will stop the state machine
    sm_turtle = smach.StateMachine(outcomes=['DONE'])

    with sm_turtle:
        smach.StateMachine.add('WAIT', Wait(),
                               transitions={'box1': 'BOX1', 'box2': 'BOX2',
                                            'box3': 'BOX3', 'box4': 'BOX4',
                                            'box5': 'BOX5', 'box6': 'BOX6',
                                            'box7': 'BOX7', 'box8': 'BOX8',
                                            'done': 'DONE'})
        smach.StateMachine.add('BOX1', Box1(),
                               transitions={'wait': 'WAIT', 'box2': 'BOX2',
                                            'box3': 'BOX3', 'box4': 'BOX4',
                                            'box5': 'BOX5', 'box6': 'BOX6',
                                            'box7': 'BOX7', 'box8': 'BOX8',
                                            'done': 'DONE'})
        smach.StateMachine.add('BOX2', Box2(),
                               transitions={'wait': 'WAIT', 'box1': 'BOX1',
                                            'box3': 'BOX3', 'box4': 'BOX4',
                                            'box5': 'BOX5', 'box6': 'BOX6',
                                            'box7': 'BOX7', 'box8': 'BOX8',
                                            'done': 'DONE'})
        smach.StateMachine.add('BOX3', Box3(),
                               transitions={'wait': 'WAIT', 'box2': 'BOX2',
                                            'box1': 'BOX1', 'box4': 'BOX4',
                                            'box5': 'BOX5', 'box6': 'BOX6',
                                            'box7': 'BOX7', 'box8': 'BOX8',
                                            'done': 'DONE'})
        smach.StateMachine.add('BOX4', Box4(),
                               transitions={'wait': 'WAIT', 'box2': 'BOX2',
                                            'box3': 'BOX3', 'box1': 'BOX1',
                                            'box5': 'BOX5', 'box6': 'BOX6',
                                            'box7': 'BOX7', 'box8': 'BOX8',
                                            'done': 'DONE'})
        smach.StateMachine.add('BOX5', Box5(),
                               transitions={'wait': 'WAIT', 'box2': 'BOX2',
                                            'box3': 'BOX3', 'box4': 'BOX4',
                                            'box1': 'BOX1', 'box6': 'BOX6',
                                            'box7': 'BOX7', 'box8': 'BOX8',
                                            'done': 'DONE'})
        smach.StateMachine.add('BOX6', Box6(),
                               transitions={'wait': 'WAIT', 'box2': 'BOX2',
                                            'box3': 'BOX3', 'box4': 'BOX4',
                                            'box5': 'BOX5', 'box1': 'BOX1',
                                            'box7': 'BOX7', 'box8': 'BOX8',
                                            'done': 'DONE'})
        smach.StateMachine.add('BOX7', Box7(),
                               transitions={'wait': 'WAIT', 'box2': 'BOX2',
                                            'box3': 'BOX3', 'box4': 'BOX4',
                                            'box5': 'BOX5', 'box6': 'BOX6',
                                            'box1': 'BOX1', 'box8': 'BOX8',
                                            'done': 'DONE'})
        smach.StateMachine.add('BOX8', Box8(),
                               transitions={'wait': 'WAIT', 'box2': 'BOX2',
                                            'box3': 'BOX3', 'box4': 'BOX4',
                                            'box5': 'BOX5', 'box6': 'BOX6',
                                            'box7': 'BOX7', 'box1': 'BOX1',
                                            'done': 'DONE'})

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
