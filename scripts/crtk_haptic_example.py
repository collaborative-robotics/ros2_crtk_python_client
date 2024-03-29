#!/usr/bin/env python

# Author: Anton Deguet
# Date: 2018-09-29
#
# Copyright (c) 2018-2021 Johns Hopkins University, University of Washington, Worcester Polytechnic Institute
# Released under MIT License

# Start your crtk compatible device first!
# dVRK example:
# > ros2 run dvrk_robot dvrk_console_json -j <console-file>
# Phantom Omni example:
# > ros2 run sensable_phantom sensable_phantom -j sawSensablePhantomRight.json

# To communicate with the device using ROS topics, see the python based example:
# > ros2 run crtk crtk_haptic_example <device-namespace>
# For dVRK, add -b/--body flag when running the crtk_haptic_example

import argparse
import crtk
import PyKDL
import std_msgs.msg
import sys


if sys.version_info.major < 3:
    input = raw_input


class crtk_haptic_example:
    class Subframe:
        def __init__(self, ral):
            self.ral = ral
            self.crtk_utils = crtk.utils(self, ral)
            self.crtk_utils.add_servo_cf()

            self.set_cf_orientation_absolute_pub = self.ral.publisher(
                 'set_cf_orientation_absolute', std_msgs.msg.Bool)

        def set_cf_orientation_absolute(self, absolute = True):
            m = std_msgs.msg.Bool()
            m.data = absolute
            self.set_cf_orientation_absolute_pub.publish(m)

    def __init__(self, ral, has_body_orientation):
        self.ral = ral

        # populate this class with all the ROS topics we need
        self.crtk_utils = crtk.utils(self, ral)
        self.crtk_utils.add_operating_state()
        self.crtk_utils.add_measured_cp()
        self.crtk_utils.add_measured_cv()

        if has_body_orientation:
            self.body = crtk_haptic_example.Subframe(self.ral.create_child('/body'))
            self.servo_cf = lambda sp: self.body.servo_cf(sp)
        else:
            self.body = None
            self.crtk_utils.add_servo_cf()

        # for all examples
        self.duration = 20 # seconds
        self.rate = 200 # aiming for 200 Hz
        self.samples = self.duration * self.rate

    # main loop
    def run(self):
        self.ral.check_connections()

        if not self.enable(30):
            print('Unable to enable the device, make sure it is connected.')
            return

        if not self.home(30):
            print('Unable to home the device, make sure it is connected.')
            return

        if self.body is not None:
            self.body.set_cf_orientation_absolute()

        self.running = True
        while (self.running):
            msg = ('\n'
                   '- q: quit\n'
                   '- p: print position, velocity\n'
                   '- b: virtual box around current position with linear forces ({}s)\n'
                   '- v: viscosity ({}s)'
            )
            print(msg.format(self.duration, self.duration))
            answer = input('Enter your choice and [enter] to continue\n')
            if answer == 'q':
                self.running = False
                self.disable()
            elif answer == 'p':
                self.run_print()
            elif answer == 'b':
                self.run_box()
            elif answer == 'v':
                self.run_viscosity()
            else:
                print('Invalid choice\n')

    # print current position
    def run_print(self):
        print(self.measured_cp())
        print(self.measured_cv())

    # virtual box
    def run_box(self):
        dim = 0.01 # +/- 1 cm per dimension creates a 2 cm cube
        p_gain = -500.0

        # save current position
        center = PyKDL.Frame()
        center.p = self.measured_cp().p

        sleep_rate = self.ral.create_rate(self.rate)
        for i in range(self.samples):
            wrench = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            # foreach d dimension x, y, z
            for d in range(3):
                distance = self.measured_cp().p[d] - center.p[d]
                if (distance > dim):
                    wrench[d] = p_gain * (distance - dim)
                elif  (distance < -dim):
                    wrench[d] = p_gain * (distance + dim)
            self.servo_cf(wrench)
            sleep_rate.sleep()

        wrench = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.servo_cf(wrench)

    # viscosity
    def run_viscosity(self):
        d_gain = -10.0
        sleep_rate = self.ral.create_rate(self.rate)
        for i in range(self.samples):
            wrench = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            # foreach d dimension x, y, z
            for d in range(3):
                wrench[d] = d_gain * self.measured_cv()[d]
            self.servo_cf(wrench)
            sleep_rate.sleep()

        wrench = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.servo_cf(wrench)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('namespace', type = str, help = 'ROS namespace for CRTK device')
    parser.add_argument('-b', '--body', action = 'store_true', help = 'Indicates CRTK device body orientation needs to be set to absolute')
    app_args = crtk.ral.parse_argv(sys.argv[1:]) # process and remove ROS args
    args = parser.parse_args(app_args) 

    example_name = type(crtk_haptic_example).__name__
    ral = crtk.ral(example_name, args.namespace)
    example = crtk_haptic_example(ral, args.body)
    ral.spin_and_execute(example.run)

if __name__ == '__main__':
    main()
