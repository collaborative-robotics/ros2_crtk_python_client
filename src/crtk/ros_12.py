#  Author(s):  Anton Deguet
#  Created on: 2023-05-08
#
# Copyright (c) 2023 Johns Hopkins University, University of Washington, Worcester Polytechnic Institute
# Released under MIT License

import rclpy
import threading

class ros_12:
    """ Class used to wrap rclpy so we can write code independent of ROS version
    """
    def __init__(self, node_name, namespace = ''):
        self.init()
        self.__node_name = node_name
        self.__namespace = namespace
        self.__rate_in_Hz = 0.0
        self.__executor_thread = None
        # ros init node
        self.__node = rclpy.create_node(self.node_name(), namespace = self.namespace())

    @staticmethod
    def ros_version():
        return 2

    @staticmethod
    def parse_argv(argv):
        # strip ros arguments
        rclpy.init(args = argv)
        return argv

    def set_rate(self, rate_in_Hz):
        self.__rate_in_Hz = rate_in_Hz
        self.__ros_rate = self.__node.create_rate(rate_in_Hz)

    def sleep(self):
        if self.__rate_in_Hz == 0.0:
            raise RuntimeError('set_rate must be called before sleep')
        self.__ros_rate.sleep()

    def node(self):
        return self.__node

    def node_name(self):
        return self.__node_name

    def namespace(self):
        return self.__namespace

    def init(self):
        try:
            rclpy.init()
        except:
            pass

    def spin(self):
        if self.__executor_thread != None:
            return
        executor = rclpy.executors.MultiThreadedExecutor()
        executor.add_node(self.__node)
        self.__executor_thread = threading.Thread(target = executor.spin, daemon = True)
        self.__executor_thread.start()

    def shutdown(self):
        rclpy.shutdown()
        if self.__executor_thread != None:
            self.__executor_thread.join()
            self.__executor_thread = None

    def spin_and_execute(self, function, *arguments):
        self.spin()
        try:
            function(*arguments)
        except KeyboardInterrupt:
            pass
        self.shutdown()
