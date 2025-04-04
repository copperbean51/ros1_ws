import pigpio
import RPi.GPIO as GPIO
from Config import ServoParams, PWMParams

LED_GREEN_GPIO = 0
LED_BLUE_GPIO = 1

class HardwareInterface:
    def __init__(self):
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(LED_GREEN_GPIO, GPIO.OUT)
        GPIO.setup(LED_BLUE_GPIO, GPIO.OUT)
        GPIO.output(LED_GREEN_GPIO, False)
        GPIO.output(LED_BLUE_GPIO, False)
        print('GPIO LED GREEN [ ', LED_GREEN_GPIO, 'pin ]', sep='')
        print('GPIO LED BLUE  [ ', LED_BLUE_GPIO, 'pin ]', sep='')

        self.pi = pigpio.pi()
        self.pwm_params = PWMParams()
        self.servo_params = ServoParams()
        self.initialize_pwm()
        return

    def set_actuator_postions(self, joint_angles):
        self.send_servo_commands(joint_angles)
        return

    def set_actuator_position(self, joint_angle, axis, leg):
        self.send_servo_command(joint_angle, axis, leg)
        return

    def deactivate(self):
        self.deactivate_servos()
        return

    def initialize_pwm(self):
        print('GPIO ', self.servo_params.pwm_freq, 'Hz ', self.servo_params.pwm_usec_range, 'range ', self.servo_params.pwm_usec_neutral, 'neutral',  sep='')
        for leg_index in range(4):
            if leg_index == 0:
                print('GPIO FR-[ ', end='')
            if leg_index == 1:
                print('GPIO FL-[ ', end='')
            if leg_index == 2:
                print('GPIO BR-[ ', end='')
            if leg_index == 3:
                print('GPIO BL-[ ', end='')
            for axis_index in range(3):
                self.pi.set_PWM_frequency(
                    self.pwm_params.pins[axis_index, leg_index], self.servo_params.pwm_freq
                )
                self.pi.set_PWM_range(self.pwm_params.pins[axis_index, leg_index], self.servo_params.pwm_usec_range)
                print('{:02d}'.format(self.pwm_params.pins[axis_index, leg_index]), 'pin ', sep='', end='')
            print(']')
        return

    def deactivate_servos(self):
        for leg_index in range(4):
            for axis_index in range(3):
                self.pi.set_PWM_dutycycle(self.pwm_params.pins[axis_index, leg_index], 0)
        return

    def angle_to_pwmdutycycle(self, angle, axis_index, leg_index):
        neutral = self.servo_params.neutral_angles[axis_index, leg_index]
        multi = self.servo_params.servo_multipliers[axis_index, leg_index]
        angle_deviation = (angle - neutral) * multi
        usec_val = self.servo_params.pwm_usec_neutral + (self.servo_params.pwm_usec_per_rad * angle_deviation)
        usec_val_limited = max(self.servo_params.pwm_usec_min, min(usec_val, self.servo_params.pwm_usec_max))
        return int(usec_val_limited)

    def angle_to_duty_cycle(self, angle, axis_index, leg_index):
        pulsewidth_micros = self.angle_to_pwm(angle, axis_index, leg_index)
        duty_cyle = int(pulsewidth_micros / 1e6 * self.servo_params.pwm_freq * self.servo_params.pwm_usec_range)
        return duty_cyle

    def angle_to_pwm(self, angle, axis_index, leg_index):
        neutral = self.servo_params.neutral_angles[axis_index, leg_index]
        multi = self.servo_params.servo_multipliers[axis_index, leg_index]
        angle_deviation = (angle - neutral) * multi
        pulse_width_micros = self.servo_params.pwm_usec_neutral + (self.servo_params.pwm_usec_per_rad * angle_deviation)
        return pulse_width_micros

    def send_servo_command(self, joint_angle, axis, leg):
        duty_cycle = self.angle_to_pwmdutycycle(joint_angle, axis, leg)
        self.pi.set_PWM_dutycycle(self.pwm_params.pins[axis, leg], duty_cycle)
        return

    def send_servo_commands(self, joint_angles):
        for leg_index in range(4):
            for axis_index in range(3):
                angle = joint_angles[axis_index, leg_index]
                duty_cycle = self.angle_to_pwmdutycycle(angle, axis_index, leg_index)
                self.pi.set_PWM_dutycycle(self.pwm_params.pins[axis_index, leg_index], duty_cycle)
        return

    def send_servo_commands_dbg(self, joint_angles):
        for leg_index in range(4):
            if leg_index == 0:
                print('FR-[ ', end='')
            if leg_index == 1:
                print('FL-[ ', end='')
            if leg_index == 2:
                print('BR-[ ', end='')
            if leg_index == 3:
                print('BL-[ ', end='')
            for axis_index in range(3):
                angle = joint_angles[axis_index, leg_index]
                duty_cycle = self.angle_to_pwmdutycycle(angle, axis_index, leg_index)
                self.pi.set_PWM_dutycycle(self.pwm_params.pins[axis_index, leg_index], duty_cycle)
                print('{:04d}'.format(duty_cycle), end=' ')
            print('] ', end='')
        print('')
        return

    def set_led_green(self, onoff):
        GPIO.output(LED_GREEN_GPIO, onoff)
        return

    def set_led_blue(self, onoff):
        GPIO.output(LED_BLUE_GPIO, onoff)
        return

