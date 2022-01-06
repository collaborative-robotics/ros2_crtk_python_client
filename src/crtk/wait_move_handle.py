# Author(s):  Anton Deguet
# Created on: 2020-11-23
#
# Copyright (c) 2020-2022 Johns Hopkins University, University of Washington, Worcester Polytechnic Institute
# Released under MIT License

import rclpy

class wait_move_handle:
    def __init__(self, class_instance, ros_node):
        self.__class_instance = class_instance
        self.__ros_node = ros_node
        self.__start_time = self.__ros_node.get_clock().now()

    def wait(self, is_busy = False, timeout = 30.0):
        if not rclpy.ok():
            return False;
        if self.__class_instance:
            return self.__class_instance.wait_for_busy(is_busy = is_busy,
                                                       start_time = self.__start_time,
                                                       timeout = timeout)
        else:
            raise RuntimeWarning('can\'t wait, the class doesn\'t support CRTK operating state')

    def is_busy(self, timeout = 30.0):
        # if we keep asking past timeout, throw an exception
        if (self.__ros_node.get_clock().now() - self.__start_time) > rclpy.duration.Duration(seconds = timeout):
            raise RuntimeWarning('is_busy past timeout')
        # else, check if we have a new state after start time and return is_busy
        if self.__class_instance:
            return self.__class_instance.is_busy(start_time = self.__start_time)
        else:
            raise RuntimeWarning('can\'t wait, the class doesn\'t support CRTK operating state')
