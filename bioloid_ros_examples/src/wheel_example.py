#!/usr/bin/env python
from bioloid_utils import *
from map import *

# ************** Available Comm Functions *****************
# sensor_value = get_sensor_value(port)
# motor_position = get_motor_position_command(motor_id)
# motor_is_moving = get_is_motor_moving_command(motor_id)
# motor_speed = get_motor_wheel_speed(motor_id)
# set_motor_target_position_command(motor_id, target_val)
# set_motor_target_speed(motor_id, target_value)
# set_multiple_motor_positions(motor_ids, target_values)
# set_motor_mode(motor_id, mode) -> Joint mode = 0, Wheel mode = 1
#
# (Only in Wheel Mode)
# target_val: 
#     0(stopped) - 1023(max) Counter Clockwise, 
#  1024(stopped) - 2047(max) Clockwise
# set_wheel_torque(motor_id, target_value)  
# set_multiple_wheel_torque(motor_ids, target_values)

# Main function
if __name__ == "__main__":
    rospy.init_node('example_node', anonymous=True)
    rospy.loginfo("Starting Group X Control Node...")

    # control loop running at 10hz
    set_motor_mode(1, 1)
    set_motor_mode(2, 1)

    motor_ids = (1, 2)
    target_values_1 = (600, 1500)
    target_values_2 = (0, 0)
    rospy.loginfo("Start Motors")
    set_multiple_wheel_torque(motor_ids, target_values_1)
    time.sleep(5.0)
    rospy.loginfo("Stop Motors")
    set_multiple_wheel_torque(motor_ids, target_values_2)
