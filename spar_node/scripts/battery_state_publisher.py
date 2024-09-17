#!/usr/bin/env python

import rospy
from sensor_msgs.msg import BatteryState

def publish_battery_state():
    rospy.init_node('battery_state_publisher', anonymous=True)
    battery_pub = rospy.Publisher('/battery_state', BatteryState, queue_size=10)

    rate = rospy.Rate(1)  # 1 Hz

    while not rospy.is_shutdown():
        battery_state = BatteryState()

        # Simulate a critical battery level
        battery_state.voltage = 11.1  # Example voltage (V)
        battery_state.current = -0.5  # Example current (A)
        battery_state.charge = 10.0  # Example remaining charge (Ah)
        battery_state.capacity = 100.0  # Example full capacity (Ah)
        battery_state.percentage = 0.05  # Critical battery level (5%)
        battery_state.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING

        # Log the published message
        rospy.loginfo("Publishing critical battery state: percentage=%.2f", battery_state.percentage)

        # Publish the battery state message
        battery_pub.publish(battery_state)

        rate.sleep()

if __name__ == '__main__':
    try:
        publish_battery_state()
    except rospy.ROSInterruptException:
        pass
