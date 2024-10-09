#!/usr/bin/env python

import rospy
import actionlib
from actionlib_msgs.msg import GoalStatus
from spar_msgs.msg import FlightMotionAction, FlightMotionGoal, ArucoLocalisation, TargetLocalisation 

class SparClientMonitor:
    def __init__(self):
        rospy.init_node('spar_client_monitor', anonymous=True)

        # Get the action server namespace from parameters or use the default
        action_ns = rospy.get_param("~action_topic", 'spar/flight')

        # Initialize the action client with the retrieved namespace
        self.spar_client = actionlib.SimpleActionClient(action_ns, FlightMotionAction)
        rospy.loginfo(f"Waiting for {action_ns} action server to start...")
        self.spar_client.wait_for_server()
        rospy.loginfo(f"{action_ns} action server started.")

    def print_state(self):
        # Continuously print the state of the action client
        rate = rospy.Rate(1)  # 1 Hz rate
        while not rospy.is_shutdown():
            state = self.spar_client.get_state()
            state_str = self.get_state_string(state)
            rospy.loginfo(f"Current spar_client state: {state} ({state_str})")
            rate.sleep()

    @staticmethod
    def get_state_string(state):
        # Convert the state code to a readable string using a dictionary for state mappings
        state_mappings = {
            GoalStatus.PENDING: "PENDING",
            GoalStatus.ACTIVE: "ACTIVE",
            GoalStatus.PREEMPTED: "PREEMPTED",
            GoalStatus.SUCCEEDED: "SUCCEEDED",
            GoalStatus.ABORTED: "ABORTED",
            GoalStatus.REJECTED: "REJECTED",
            GoalStatus.PREEMPTING: "PREEMPTING",
            GoalStatus.RECALLING: "RECALLING",
            GoalStatus.RECALLED: "RECALLED",
            GoalStatus.LOST: "LOST",
            -1: "UNKNOWN"
        }
        return state_mappings.get(state, "UNKNOWN")

if __name__ == '__main__':
    try:
        monitor = SparClientMonitor()
        monitor.print_state()
    except rospy.ROSInterruptException:
        pass
