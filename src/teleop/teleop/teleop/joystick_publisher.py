#!/usr/bin/env python3
import os
import time
import pygame
import numpy as np

import yaml
from ament_index_python.packages import get_package_share_directory


import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Joy

class JoystickPublisher(Node):

    def __init__(self):
   
        super().__init__('joystick')
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.controller_inputs)


        # Parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('joystick_type', rclpy.Parameter.Type.STRING),
                ('subsystem' , rclpy.Parameter.Type.STRING),
            ]
        )
        self.joystick_type = self.get_parameter('joystick_type').value
        self.subsystem = self.get_parameter('subsystem').value
        self.publisher_ = self.create_publisher(Joy, f'controller_input/{self.joystick_type}', 10)

        self.get_logger().info(f"Joystick type: {self.joystick_type}")
        self.get_logger().info(f"Subsystem: {self.subsystem}")

        joy_yaml_path = os.path.join(get_package_share_directory('teleop'), 'config', 'joy.yaml')
        with open(joy_yaml_path, 'r') as f:
            config = yaml.safe_load(f)


        axes_list = config['/**']['ros__parameters']['controllers'][self.joystick_type]['axes']

        self._axes_config = {e['name']: e for e in axes_list}
        slots = config['/**']['ros__parameters']['subsystems'][self.subsystem]['output_slots']
        self._slot_params = [self._axes_config[name] for name in slots]

        

        # Detect Jetson platform
        self.is_jetson = False
        try:
            with open('/proc/device-tree/model', 'r') as f:
                if 'Jetson' in f.read():
                    self.is_jetson = True
                    self.get_logger().info("Jetson platform detected - using axis remapping")
        except:
            pass


        joysticks = 0
        self.axis_data = None
        self.controller = None
        self.button_data = None
        self.hat_data = None
        self.activation = 0.08
        JOYSTICK_NAMES = {s
            "thrustmaster": "thrustmaster t.16000m",
            "xbox": "xbox 360 controller",
            "airbus": "thrustmaster t.a320 copilot"
        }
        target_name = JOYSTICK_NAMES[self.joystick_type]

        self.joystick_index = None

        # Pygame Controller
        pygame.init()
        pygame.joystick.init()
        joysticks = pygame.joystick.get_count()

        for i in range(joysticks):
            js = pygame.joystick.Joystick(i)
            name = js.get_name().lower()
            if(name == target_name):
                self.joystick_index = i
                break

        # Only begin once a joystick is connected
        while(joysticks == 0):
            # self.get_logger().info("No controllers are connected!")
            time.sleep(0.25)
            for event in pygame.event.get():
                if event.type == pygame.JOYDEVICEADDED:
                    print("Joystick connected.")
                    pygame.joystick.init()            
                    joysticks = pygame.joystick.get_count()
                    break
            
        self.controller = pygame.joystick.Joystick(self.joystick_index)
        self.get_logger().info(f"Using joystick: {self.controller.get_name()}")
        self.controller.init()

        if not self.axis_data:
            self.axis_data = {0: 0, 1: 0, 2: 0, 3: 0, 4: 0, 5: 0}

        if not self.button_data:
            self.button_data = {}
            for i in range(self.controller.get_numbuttons()):
                self.button_data[i] = False

        if not self.hat_data:
            self.hat_data = {}
            for i in range(self.controller.get_numhats()):
                self.hat_data[i] = (0, 0)

        self.previous_axes = np.zeros(len(self._slot_params))
        self.previous_buttons = np.zeros(len(self.button_data))

    def _normalize(self, raw, invert, trigger, range_inverted=False):
        if abs(raw) < self.activation:
            val = 0.0
        else:
            val = (abs(raw) - self.activation) / (1.0 - self.activation)
            val = val if raw > 0 else -val
        if trigger:
            val = (1.0 - val) / 2.0 if range_inverted else (val + 1.0) / 2.0
        if invert:
            val = -val
        return val

    def controller_inputs(self):
       
        joystick_vels = self.previous_axes
        button_activations = self.previous_buttons
        for event in pygame.event.get():
            if event.type == pygame.JOYAXISMOTION:
                self.axis_data[event.axis] = round(event.value,2)
            elif event.type == pygame.JOYBUTTONDOWN:
                self.button_data[event.button] = True
            elif event.type == pygame.JOYBUTTONUP:
                self.button_data[event.button] = False
            elif event.type == pygame.JOYHATMOTION:
                self.hat_data[event.hat] = event.value

            
            #axes normalization
            for i, slot in enumerate(self._slot_params):
                joystick_vels[i] = self._normalize(
                    self.axis_data.get(slot['axis'], 0.0),
                    slot['invert'],
                    slot['trigger'],
                    slot.get('range_inverted', False)
                )

            # Buttons
            for i in self.button_data:
                button_activations[i] = self.button_data[i]

        # Save current numpy array for joystick and buttons
        self.previous_axes = joystick_vels
        self.previous_buttons = button_activations

        axes_data = joystick_vels.tolist()
        buttons_data = [int(el) for el in button_activations.tolist()]
        msg = Joy()
        msg.axes = axes_data
        msg.buttons = buttons_data


        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    joystick_publisher = JoystickPublisher()
    rclpy.spin(joystick_publisher)
    joystick_publisher.destroy_node()
    pygame.joystick.quit()
    pygame.quit()
    rclpy.shutdown()




if __name__ == '__main__':
    main()
