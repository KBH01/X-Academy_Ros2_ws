#!/usr/bin/env python3


import sys
sys.path.append(".")

import rclpy
import time
import pi_servo_hat
import atexit
from rclpy.node import Node
from sensor_msgs.msg import Joy
import numpy as np
import math
import ms5837 #external pressure sensor
from .tsys01 import TSYS01 #external temp sensor


def map_value(value, in_min, in_max, out_min, out_max):
    return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


class PiHat:
    def __init__(self):
        self.hat = pi_servo_hat.PiServoHat()
        self.hat.restart()
        
        atexit.register(self.stop_all_motors) # this only works when run in terminal

    def send_pwm(self, channel, signal):
        # signal (in microseconds for PWM)
        
        angle = (signal - 922) / 10.8 #trim here
        
        self.hat.move_servo_position(channel, angle)
    
    def send_angle(self, channel, angle):
        #sends angle for servos

        self.hat.move_servo_position(channel, angle)

    def drive_motor(self, channel, velocity):
        # velocity is a float between -1 and 1.
        # -1 corresponds to full backwards,
        # 1 corresponds to full forwards,
        # 0 corresponds to stopped
        
        velocity = max(min(velocity, 1), -1)
        velocity = map_value(velocity, -1, 1, 1300, 1700)
        #print(f'Velocity: {velocity}')
        self.send_pwm(channel, velocity)
    
    def stop_all_motors(self):
        for channel in range(16):
            self.drive_motor(channel, 0)

class JoyControllerNode(Node):

    def __init__(self):
        super().__init__("joy_controller")
        self.joy_subscriber_ = self.create_subscription(
            Joy,
            "/joy",
            self.joy_callback,
            10 #queue size
            )
        self.get_logger().info("Controller has been started.")
        self.hat = PiHat()

        #gripper variables
        self.is_open = False
        self.previous_gripper_msg = 0

        #depth hold variables
        self.ext_pressure_sensor = ms5837.MS5837_02BA(bus=3)
        self.ext_pressure_sensor.init()
        self.depth_hold_on = False
        self.previous_depth_hold_msg = 0
        self.target_depth = 0
        self.p_gain = 15 #adjusts output based on current error
        self.i_gain = 30 #adjusts output based on past errors over time
        self.d_gain = 35 #predicts future errors based on current depth rate of change
        self.pid_integral = 0
        self.pid_previous_error = 0
        self.previous_time = time.time()
        self.pid_loop_interval = 0.001 #how often pid is calculated
        self.pid_output = 0
        self.pid_output_gain = 0.1

        #external temp sensor
        self.ext_temp_sensor = TSYS01(bus=2)
        self.ext_temp_sensor.init()

    def joy_callback(self, msg: Joy):
        self.get_logger().info(str(msg))

        #motor mapping
        speed_changer = (msg.axes[3] + 1) / 2
        raw_input = np.matrix([[msg.axes[0]],
                               [-msg.axes[1]]])
        raw_input *= speed_changer
        max_component = max(abs(raw_input[0,0]),abs(raw_input[1,0]))
        try:
            converted_input = (max_component/(math.sqrt((raw_input[0,0]**2) + (raw_input[1,0]**2)))) * raw_input
        except:
            converted_input = raw_input

        converted_input = (max_component/(math.sqrt((raw_input[0,0]**2) + (raw_input[1,0]**2)))) * raw_input

        if math.isnan(converted_input[0,0]):
            converted_input[0,0] = 0
        if math.isnan(converted_input[1,0]):
            converted_input[1,0] = 0

        sqrt_two = math.sqrt(2)
        rov_basis_change = np.matrix([[sqrt_two,sqrt_two],
                                      [-sqrt_two,sqrt_two]])
        motion_vector = rov_basis_change * converted_input
        motion_vector_lengthened = np.matrix([[motion_vector[0,0]],
                                              [motion_vector[1,0]],
                                              [motion_vector[1,0]],
                                              [motion_vector[0,0]]])
        
        turning_vector = (-0.65 * speed_changer)*np.matrix([[-msg.axes[2]],
                                    [msg.axes[2]],
                                    [-msg.axes[2]],
                                    [msg.axes[2]],])
        
        final_vector = ((max_component / (max_component + abs(msg.axes[2]))) * motion_vector_lengthened) + ((abs(msg.axes[2]) / (max_component + abs(msg.axes[2]))) * turning_vector)

        for i in range(final_vector.shape[0]):
            if math.isnan(final_vector[i,0]):
                final_vector[i,0] = 0

        self.hat.drive_motor(1,final_vector[0,0])
        self.hat.drive_motor(5,final_vector[1,0])
        self.hat.drive_motor(0,final_vector[2,0])
        self.hat.drive_motor(4,final_vector[3,0])

        #print(final_vector)
        
        #VERTICAL CONTROL
        if msg.buttons[4]==1:
            self.hat.drive_motor(2,msg.buttons[4]*(-1)*speed_changer)
            self.hat.drive_motor(3,msg.buttons[4]*(-1)*speed_changer)
        elif msg.buttons[2]==1:
            self.hat.drive_motor(2,msg.buttons[2]*speed_changer)
            self.hat.drive_motor(3,msg.buttons[2]*speed_changer)
        elif not self.depth_hold_on and msg.buttons[4]!=1 and msg.buttons[2]!=1:
            self.hat.drive_motor(2,0)
            self.hat.drive_motor(3,0)
        
        #GRIPPER CONTROL (TOGGLE)
        if msg.buttons[0]==1 and self.previous_gripper_msg==0 and self.is_open==False:
            self.hat.send_angle(9, 60)
            self.is_open = True
        elif msg.buttons[0]==1 and self.previous_gripper_msg==0 and self.is_open==True:
            self.hat.send_angle(9, -15)
            self.is_open = False

        self.previous_gripper_msg = msg.buttons[0]

        #DEPTH HOLD (BUTTON 1, TOGGLE)
        self.ext_pressure_sensor.read()

        if msg.buttons[1]==1 and self.previous_depth_hold_msg==0 and self.depth_hold_on==False:
            self.target_depth = self.ext_pressure_sensor.depth()
            self.pid_integral = 0
            self.depth_hold_on = True
        elif msg.buttons[1]==1 and self.previous_depth_hold_msg==0 and self.depth_hold_on==True:
            self.depth_hold_on = False

        self.previous_depth_hold_msg = msg.buttons[1]

        if self.depth_hold_on and (msg.buttons[4]==1 or msg.buttons[2]==1):
            self.target_depth = self.ext_pressure_sensor.depth()
            self.pid_integral = 0
        elif self.depth_hold_on and not (msg.buttons[4]==1 or msg.buttons[2]==1):
            #PID LOOP
            current_depth = self.ext_pressure_sensor.depth()
            depth_error = self.target_depth - current_depth

            #time difference (dt) calculation
            current_time = time.time()
            dt = current_time - self.previous_time
            
            if dt >= self.pid_loop_interval:
                #PID calculation
                prop = self.p_gain * depth_error
                self.pid_integral += self.i_gain * depth_error * dt
                deriv = self.d_gain * (depth_error - self.pid_previous_error) / dt

                #PID output
                self.pid_output = prop + self.pid_integral + deriv
                self.pid_output *= self.pid_output_gain

                self.previous_error = depth_error
                self.previous_time = current_time

            print(f'DH THRUSTER COMMAND: {self.pid_output}')
            self.hat.drive_motor(2,self.pid_output)
            self.hat.drive_motor(3,self.pid_output)


        print(f'DEPTH HOLD: {self.depth_hold_on}')
        print(f'TARGET DEPTH: {self.target_depth}m')
        print(f'CURRENT DEPTH: {self.ext_pressure_sensor.depth()}m')

        #TEMP SENSOR
        self.ext_temp_sensor.read()
        print(f'CURRENT TEMP: {self.ext_temp_sensor.temperature()}C')
        
def main(args = None):
    rclpy.init(args = args)
    node  = JoyControllerNode()
    rclpy.spin(node)
    rclpy.shutdown()
