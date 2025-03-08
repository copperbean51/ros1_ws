import numpy as np
import time
# import smbus2
from Adafruit_PCA9685 import PCA9685

from .Config import ServoParams, PWMParams


class HardwareInterface:
    def __init__(self):
        self.pwm_params = PWMParams()
        self.servo_params = ServoParams()
        self.i2c_bus = 1
        self.pwm = PCA9685(address=0x40, busnum=self.i2c_bus)
        self.pwm.set_pwm_freq(self.pwm_params.freq)

    def set_actuator_positions(self, joint_angles):
        send_servo_commands(self.pwm, joint_angles)

    def set_actuator_position(self, joint_angle, axis, leg):
        send_servo_command(self.pwm, joint_angle, axis, leg)

def angle_to_pwm(angle):
    """Convert angle to pwm """
    min_pulse = 102  
    max_pulse = 512 

    angle = max(-90, min(90, angle))

    return int(min_pulse + ((angle + 90) / 180.0) * (max_pulse - min_pulse))     

def send_servo_commands(pwm, joint_angles):
    """Send PWM commands to all servos using PCA9685"""
    for leg_index in range(4):
        for axis_index in range(3):
            # Calculate the channel number based on leg_index and axis_index
            channel = leg_index * 3 + axis_index
            # Set the PWM value for the corresponding servo
            motor_name = get_motor_name(axis_index, leg_index)
            print(joint_angles)
            print(channel)
            print(motor_name)
            pwm.set_pwm(channel, 0, angle_to_pwm(joint_angles[axis_index, leg_index]))

def send_servo_command(pwm, joint_angle, axis, leg):
    """Send PWM command to a single servo using PCA9685"""
    channel = leg * 3 + axis
    print(joint_angle)
    print(channel)
    # Set the PWM value for the corresponding servo
    pwm.set_pwm(channel, 0, angle_to_pwm(joint_angle))

def deactivate_servos(pwm, pwm_params):
    """Deactivate all servos by setting their PWM to 0"""
    for leg_index in range(4):
        for axis_index in range(3):
            channel = leg_index * 3 + axis_index
            pwm.set_pwm(channel, 0, angle_to_pwm(0))

def get_motor_name(i, j):
    motor_type = {0: "abduction", 1: "inner", 2: "outer"}  # Top  # Bottom
    leg_pos = {0: "front-right", 1: "front-left", 2: "back-right", 3: "back-left"}
    final_name = motor_type[i] + " " + leg_pos[j]
    return final_name

def get_motor_setpoint(i, j):
    # {0: "abduction", 1: "inner", 2: "outer"}
    # [[rf-0, lf-0, rb-0, lb-0]
    #  [rf-1, lf-1, br-1, lb-1] 
    #  [rf-2, lf-2, br-2, lb-2]]
    data = np.array([[0, 0, 0, 0], 
                    [45, 45, 45, 45], 
                    [-45, -45, -45, -45]])
    return data[i, j]

def move_multi_servos(hardware_interface):
    """Move servos between two positions"""
    # joint_angles = np.array([[0, 0, 0, 0], 
    #                          [45, 45, 45, 45], 
    #                          [-45, -45, -45, -45]])

    joint_angles_1 = np.array( [[0.04501474, -0.04931468, 0.04984245, -0.04501471], 
                             [10.17576797, 40.46696824, 40.46725562, 40.17576796], 
                             [-10.08937271, -40.46331102, -40.46356711, -50.08937272]])

    joint_angles_2 = np.array( [[0.04501474, -0.04931468, 0.04984245, -0.04501471], 
                             [0.17576797, 0.46696824, 0.46725562, 0.17576796], 
                             [-0.08937271, -0.46331102, -0.46356711, -0.08937272]])


    # Move all servos
    while True:
        hardware_interface.set_actuator_positions(joint_angles_1)
        time.sleep(0.5)
        hardware_interface.set_actuator_positions(joint_angles_2)
        time.sleep(0.5)


def calibrate_angle_offset(hardware_interface):
    """Calibrate the angle offset for the twelve motors on the robot. Note that servo_params is modified in-place.

    """
    hardware_interface.servo_params.neutral_angle_degrees = np.zeros((3, 4))

    for leg_index in range(4):
        for axis in range(3):
            set_point = get_motor_setpoint(axis, leg_index)
            #print(f"Set point: {set_point}")

            # Zero out the neutral angle
            hardware_interface.servo_params.neutral_angle_degrees[axis, leg_index] = 0

            # Move servo to set_point angle
            hardware_interface.set_actuator_position(
                set_point,
                axis,
                leg_index,
            )

if __name__ == "__main__":
    hardware_interface = HardwareInterface()
    try:
        #calibrate_angle_offset(hardware_interface)
        move_multi_servos(hardware_interface)
    except KeyboardInterrupt:
        print("Deactivating servos...")
        deactivate_servos(hardware_interface.pwm, hardware_interface.pwm_params)