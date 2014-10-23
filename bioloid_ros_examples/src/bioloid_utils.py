#!/usr/bin/env python
import roslib
roslib.load_manifest('bioloid_ros_examples')
import rospy
from bioloid_ros_driver.srv import *
import time
import sys

# wrapper function to call command service
def service_call(command_type, dev_id, target_val, num_ids, motor_ids, target_vals):
    rospy.wait_for_service('allcmd')
    try:
        send_command = rospy.ServiceProxy('allcmd', allcmd)
        resp1 = send_command(command_type, dev_id, target_val, num_ids, motor_ids, target_vals)
        return resp1.val
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e

# wrapper function to get sensor value
def get_sensor_value(port):
    return service_call('GetSensorValue', port, 0, 0, [0], [0])


# wrapper function to call service to set a motor target position
def set_motor_target_position_command(motor_id, target_val):
    return service_call('SetMotorTargetPosition', motor_id, target_val, 0, [0], [0])


# wrapper function to call service to get a motor's current position
def get_motor_position_command(motor_id):
    return service_call('GetMotorCurrentPosition', motor_id, 0, 0, [0], [0])

# wrapper function to call service to check if a motor is currently moving
def get_is_motor_moving_command(motor_id):
    return service_call('GetIsMotorMoving', motor_id, 0, 0, [0], [0])

# wrapper function to call service to check motor wheel speed, ONLY WORKS WHEN MOTOR IS IN WHEEL MODE
def get_motor_wheel_speed(motor_id):
    return service_call('GetMotorWheelSpeed', motor_id, 0, 0, [0], [0])

# wrapper function to set motor target speed
def set_motor_target_speed(motor_id, target_value):
    if (target_value < 0 or target_value > 1023):
        rospy.logwarn("Max motor speed is 0, Min motor speed is 1023")
        return -1
    return service_call('SetMotorTargetSpeed', motor_id, target_value, 0, [0], [0])

# wrapper function for setting multiple motor positions in a single call
def set_multiple_motor_positions(motor_ids, target_values):
    # error checking, list lengths must match!
    if len(motor_ids) != len(target_values):
        rospy.logwarn("[Motor Sync]: List lengths dont match")
        return -1
    return service_call('SetMotorTargetPositionsSync', 0, 0, len(motor_ids), motor_ids, target_values)

# Joint mode = 0, Wheel mode = 1
def set_motor_mode(motor_id, mode):
    return service_call('SetMotorMode', motor_id, mode, 0, [0], [0])

# Only in Wheel Mode.
# target_val 0(stopped) - 1023(max) Counter Clockwise, 1024(stopped) - 2047(max) Clockwise
def set_wheel_torque(motor_id, target_value):
    if (target_value < 0 or target_value > 2047):
        rospy.logwarn("Min wheel speed is 0 (or 1024), Max wheel speed is 1023 (or 2047)")
        return -1
    return service_call('SetMotorWheelSpeed', motor_id, target_value, 0, [0], [0])

def set_multiple_wheel_torque(motor_ids, target_values):
    if len(motor_ids) != len(target_values):
        rospy.logwarn("[Wheel Sync]: List lengths dont match")
        return -1
    return service_call('SetWheelSpeedSync', 0, 0, len(motor_ids), motor_ids, target_values)