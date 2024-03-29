#!/usr/bin/env python

# Author: Anton Deguet
# Created on: 2015-02-22
#
# Copyright (c) 2015-2021 Johns Hopkins University, University of Washington, Worcester Polytechnic Institute
# Released under MIT License

# Start a single arm using
# > ros2 run dvrk_robot dvrk_console_json -j <console-file>

# To communicate with the arm using ROS topics, see the python based example dvrk_arm_test.py:
# > ros2 run crtk crtk_servo_jp_example.py <arm-name>

import argparse
import crtk
import math
import numpy
import sys


class crtk_servo_jp_example:
    def __init__(self, ral):
        self.ral = ral

        # populate this class with all the ROS topics we need
        self.crtk_utils = crtk.utils(self, ral)
        self.crtk_utils.add_operating_state()
        self.crtk_utils.add_setpoint_js()
        self.crtk_utils.add_servo_jp()

        # for all examples
        self.duration = 10 # 10 seconds
        self.rate = 500    # aiming for 200 Hz

    def run(self):
        self.ral.check_connections()

        if not self.enable(30):
            print("Unable to enable the device, make sure it is connected.")
            return

        if not self.home(30):
            print('Unable to home the device, make sure it is connected.')
            return

        # create a new goal starting with current position
        start_jp = numpy.copy(self.setpoint_jp())
        goal = numpy.copy(self.setpoint_jp())
        amplitude = math.radians(10.0) # +/- 10 degrees

        sleep_rate = self.ral.create_rate(self.rate)
        samples = self.duration * self.rate
        for i in range(samples):
            sine = math.sin(math.radians(360.0) * float(i) / samples)
            angle = amplitude * sine
            goal[0] = start_jp[0] + angle
            goal[1] = start_jp[1] + angle
            self.servo_jp(goal)
            sleep_rate.sleep()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('namespace', type = str, help = 'ROS namespace for CRTK device')
    app_args = crtk.ral.parse_argv(sys.argv[1:]) # process and remove ROS args
    args = parser.parse_args(app_args) 

    example_name = type(crtk_servo_jp_example).__name__
    ral = crtk.ral(example_name, args.namespace)
    example = crtk_servo_jp_example(ral)
    ral.spin_and_execute(example.run)


if __name__ == '__main__':
    main()
