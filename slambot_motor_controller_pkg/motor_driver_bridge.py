#!/usr/bin/env python3

import argparse
import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist

from slambot_interfaces.msg import EncoderTicks

import serial

class MotorDriverBridge(Node):
    def __init__(self, motor_driver_port):
        super().__init__("motor_driver_bridge")

        self.ser = serial.Serial(motor_driver_port, 115200, timeout=10)

        # Physical robot constants
        self.wheel_separation = 0.28          # metres
        self.wheel_radius = 0.03              # metres
        self.ticks_per_revolution = 1400      # encoder ticks per full wheel revolution
        self.max_pwm = 255                    # maximum PWM value for motor control
        
        # Latest commanded robot motion
        self.target_linear_x = 0.0
        self.target_angular_z = 0.0

        # Measured wheel speeds in ticks/sec
        self.measured_left_ticks_per_sec = 0.0
        self.measured_right_ticks_per_sec = 0.0

        # Target wheel speeds in ticks/sec
        self.target_left_ticks_per_sec = 0.0
        self.target_right_ticks_per_sec = 0.0

        # Measured wheel speeds in ticks/sec
        self.measured_left_ticks_per_sec = 0.0
        self.measured_right_ticks_per_sec = 0.0

        # Previous encoder readings
        self.prev_left_ticks = None
        self.prev_right_ticks = None

        self.prev_control_time = self.get_clock().now()

        
        self.encoder_ticks_subscriber_ = self.create_subscription(EncoderTicks, "encoder_ticks", self.encoder_callback, 10)
        self.geometry_subscriber = self.create_subscription(Twist, "cmd_vel", self.cmd_vel_callback, 10)

        # Control loop timer at 1 Hz
        self.control_timer = self.create_timer(0.90, self.control_loop)

    def cmd_vel_callback(self, msg: Twist):
        self.target_linear_x = msg.linear.x
        self.target_angular_z = msg.angular.z

        left_mps, right_mps = self.twist_to_wheel_linear_speeds(
            self.target_linear_x,
            self.target_angular_z,
            self.wheel_separation,
        )

        self.target_left_ticks_per_sec = self.linear_speed_to_ticks_per_sec(
            left_mps,
            self.wheel_radius,
            self.ticks_per_revolution,
        )

        self.target_right_ticks_per_sec = self.linear_speed_to_ticks_per_sec(
            right_mps,
            self.wheel_radius,
            self.ticks_per_revolution,
        )

        self.get_logger().info(f"Target linear x: {self.target_linear_x:.2f} m/s, angular z: {self.target_angular_z:.2f} rad/s")
        self.get_logger().info(f"Target left ticks/s: {self.target_left_ticks_per_sec:.2f}, Target right ticks/s: {self.target_right_ticks_per_sec:.2f}")

    def encoder_callback(self, msg: EncoderTicks):
        now = self.get_clock().now()

        if self.prev_left_ticks is None:
            self.prev_left_ticks = msg.left_encoder
            self.prev_right_ticks = msg.right_encoder
            self.prev_control_time = now
            return

        dt = (now - self.prev_control_time).nanoseconds / 1e9
        if dt <= 0.0:
            return

        left_delta = msg.left_encoder - self.prev_left_ticks
        right_delta = msg.right_encoder - self.prev_right_ticks

        self.measured_left_ticks_per_sec = left_delta / dt
        self.measured_right_ticks_per_sec = right_delta / dt

        self.prev_left_ticks = msg.left_encoder
        self.prev_right_ticks = msg.right_encoder
        self.prev_control_time = now

    def control_loop(self):
        if self.target_left_ticks_per_sec == 0.0 and self.target_right_ticks_per_sec == 0.0:
            self.send_motor_command(0, 0)
            return

        left_motor_error = self.calculate_motor_speed_error(self.target_left_ticks_per_sec, self.measured_left_ticks_per_sec)
        right_motor_error = self.calculate_motor_speed_error(self.target_right_ticks_per_sec, self.measured_right_ticks_per_sec)

        left_pwm = self.calculate_pwm_from_error(left_motor_error)
        right_pwm = self.calculate_pwm_from_error(right_motor_error)

        self.send_motor_command(left_pwm, right_pwm)

    def twist_to_wheel_linear_speeds(self, linear_x: float, angular_z: float, wheel_separation: float):
        left = linear_x - (angular_z * wheel_separation / 2.0)
        right = linear_x + (angular_z * wheel_separation / 2.0)
        return left, right

    def linear_speed_to_ticks_per_sec(self, linear_speed: float, wheel_radius: float, ticks_per_revolution: int):
        wheel_revs_per_sec = linear_speed / (2.0 * math.pi * wheel_radius)
        return wheel_revs_per_sec * ticks_per_revolution
    
    def calculate_motor_speed_error(self, target_ticks_per_sec: float, measured_ticks_per_sec: float):
        return target_ticks_per_sec - measured_ticks_per_sec
    
    def calculate_pwm_from_error(self, error: float):
        # Simple proportional controller for demonstration
        Kp = 3.9  # Proportional gain, needs tuning
        pwm = Kp * error

        # Clamp PWM to max limits
        pwm = max(min(pwm, self.max_pwm), -self.max_pwm)
        return int(pwm)
    
    def send_motor_command(self, left_motor_pwm: int, right_motor_pwm: int):
        left_dir = 1
        right_dir = 1

        if left_motor_pwm < 0:
            left_dir = 0
            left_motor_pwm = abs(left_motor_pwm)

        if right_motor_pwm < 0:
            right_dir = 0
            right_motor_pwm = abs(right_motor_pwm)

        left_motor_pwm = max(0, min(left_motor_pwm, self.max_pwm))
        right_motor_pwm = max(0, min(right_motor_pwm, self.max_pwm))

        self.get_logger().info(f"Left PWM: {left_motor_pwm} dir={left_dir}, Right PWM: {right_motor_pwm} dir={right_dir}")

        command = f"V{left_motor_pwm}:{left_dir},{right_motor_pwm}:{right_dir}\n"
        self.ser.write(command.encode('utf-8'))
        self.get_logger().info(f"Sent motor command: {command.strip()}")

def main(args=None):
    parser = argparse.ArgumentParser(description='MotorDriverBridge')
    parser.add_argument('--motor_driver_port', type=str, default='/dev/ttyUSB0', help='Set motor driver (Arduino Mega) port')

    user_args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    node = MotorDriverBridge(motor_driver_port=user_args.motor_driver_port)
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()