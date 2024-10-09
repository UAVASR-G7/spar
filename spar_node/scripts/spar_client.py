#!/usr/bin/env python

import rospy
import actionlib
from movebasemsgs.msg import MoveBaseAction
from actionlibmsgs.msg import GoalStatus

class SparClientMonitor:
    def init(self):
        rospy.initnode('spar_client_monitor', anonymous=True)

        # Initialize the action client
        self.spar_client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        rospy.loginfo("Waiting for /move_base action server to start...")
        self.spar_client.wait_for_server()
        rospy.loginfo("/move_base action server started.")

    def print_state(self):
        # Loop to constantly print the state of the spar_client
        rate = rospy.Rate(1)  # 1 Hz rate
        while not rospy.is_shutdown():
            state = self.spar_client.get_state()
            state_str = self.get_state_string(state)
            rospy.loginfo(f"Current spar_client state: {state} ({state_str})")
            rate.sleep()

    @staticmethod
    def get_state_string(state):
        # Convert the state code to a readable string
        if state == GoalStatus.PENDING:
            return "PENDING"
        elif state == GoalStatus.ACTIVE:
            return "ACTIVE"
        elif state == GoalStatus.PREEMPTED:
            return "PREEMPTED"
        elif state == GoalStatus.SUCCEEDED:
            return "SUCCEEDED"
        elif state == GoalStatus.ABORTED:
            return "ABORTED"
        elif state == GoalStatus.REJECTED:
            return "REJECTED"
        elif state == GoalStatus.PREEMPTING:
            return "PREEMPTING"
        elif state == GoalStatus.RECALLING:
            return "RECALLING"
        elif state == GoalStatus.RECALLED:
            return "RECALLED"
        elif state == GoalStatus.LOST:
            return "LOST"
        else:
            return "UNKNOWN"

if __name == '__main':
    try:
        monitor = SparClientMonitor()
        monitor.print_state()
    except rospy.ROSInterruptException:
        pass