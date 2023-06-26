#  Author(s):  Anton Deguet
#  Created on: 2023-05-08
#
# Copyright (c) 2023 Johns Hopkins University, University of Washington, Worcester Polytechnic Institute
# Released under MIT License

import rclpy
import rclpy.publisher
import rclpy.qos
import rclpy.subscription
import rclpy.time
import threading


class ral:
    """RAL: ROS abstraction layer
    
    Provides many common parts of rospy/rclpy via a common API, to allow CRTK code to
    be written at a higher level, and to work for both ROS 1 and ROS 2.
    """

    @staticmethod
    def ros_version():
        return 2

    @staticmethod
    def parse_argv(argv):
        # strip ros arguments
        rclpy.init(args = argv)
        return argv

    def __init__(self, node_name, namespace = '', node = None):
        self.__node_name = node_name
        self.__namespace = namespace.strip("/")
        self.__rate_in_Hz = 0.0

        self.__children = {}
        self.__publishers = []
        self.__subscribers = []
        self.__services = []

        self.__executor_thread = None

        if node is not None:
            self.__node = node
        else:
            # initializes rclpy only if it hasn't been already (e.g. via parse_argv()),
            # otherwise does nothing (since rclpy will raise a RuntimeError)
            try:
                rclpy.init()
            except:
                pass
            # ros init node
            self.__node = rclpy.create_node(self.node_name())

    def __del__(self):
        for pub in self.__publishers:
            self.__node.destroy_publisher(pub)

        for sub in self.__subscribers:
            self.__node.destroy_subscription(sub)

    def create_child(self, child_namespace):
        if child_namespace in self.__children:
            raise RuntimeError('ral object already has child "{child_namespace}"!')

        child_namespace = child_namespace.strip('/')
        child_full_namespace = f'{self.namespace()}/{child_namespace}'
        child = ral(self.node_name(), child_full_namespace, self.__node)
        self.__children[child_namespace] = child
        return child

    def now(self):
        clock = self.__node.get_clock()
        return clock.now()

    def timestamp(self, t):
        if hasattr(t, 'header'):
            t = t.header
        if hasattr(t, 'stamp'):
            t = t.stamp

        try:
            return rclpy.time.Time.from_msg(t)
        except:
            return t
        
    def to_secs(self, t):
        return float(t.nanoseconds)/1e9
    
    def timestamp_secs(self, t):
        stamp = self.timestamp(t)
        return self.to_secs(stamp)
        
    def duration(self, d):
        return rclpy.time.Duration(seconds = d)

    def set_rate(self, rate_in_Hz):
        self.__rate_in_Hz = rate_in_Hz
        self.__ros_rate = self.__node.create_rate(rate_in_Hz)

    def sleep(self):
        if self.__rate_in_Hz == 0.0:
            raise RuntimeError('set_rate must be called before sleep')
        self.__ros_rate.sleep()

    def node_name(self):
        return self.__node_name

    def namespace(self):
        return self.__namespace

    def add_namespace(self, name):
        name = name.strip('/')
        return f'{self.namespace()}/{name}'

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

    def on_shutdown(self, callback):
        rclpy.get_default_context().on_shutdown(callback)

    def is_shutdown(self):
        return not rclpy.ok()

    def publisher(self, topic, msg_type, queue_size = 10, latch = False, *args, **kwargs):
        history = rclpy.qos.HistoryPolicy.KEEP_LAST
        qos = rclpy.qos.QoSProfile(depth = queue_size, history = history)
        if latch:
            # publisher will retain queue_size messages for future subscribers
            qos.durability = rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL

        pub = self.__node.create_publisher(msg_type, self.add_namespace(topic), qos)
        self.__publishers.append(pub)
        return pub

    def subscriber(self, topic, msg_type, callback, queue_size=10, *args, **kwargs):
        history = rclpy.qos.HistoryPolicy.KEEP_LAST
        qos = rclpy.qos.QoSProfile(depth = queue_size, history = history)
        sub = self.__node.create_subscription(msg_type, self.add_namespace(topic),
                                              callback, qos, *args, **kwargs)
        self.__subscribers.append(sub)
        return sub

    def service_client(self, name, srv_type):
        client = self.__node.create_client(srv_type, self.add_namespace(name))
        self.__services.append(client)
        return client
    
    def get_topic(self, pubsub):
        return pubsub.topic_name

    def check_connections(self, timeout_seconds):
        """Check that all publishers are connected to at least one subscriber,
        and that all subscribers are connected to at least on publisher.

        If timeout_seconds is zero, no checks will be done.
        """
        if timeout_seconds <= 0.0:
            return

        start_time = self.now()
        timeout_duration = self.duration(timeout_seconds)
        rate = self.__node.create_rate(100)

        def connected(pubsub):
            if isinstance(pubsub, rclpy.publisher.Publisher):
                # get_subscription_count() available in >= ROS 2 Dashing
                return pubsub.get_subscription_count() > 0
            elif isinstance(pubsub, rclpy.subscription.Subscription):
                try:
                    return pubsub.get_publisher_count() > 0
                except AttributeError:
                    # get_publisher_count() not available until ROS 2 Iron Irwini
                    return True
            else:
                raise TypeError("pubsub must be an rclpy Publisher or Subscription")

        # wait at most timeout_seconds for all connections to establish
        while (self.now() - start_time) < timeout_duration:
            pubsubs = self.__publishers + self.__subscribers
            unconnected = [ps for ps in pubsubs if not connected(ps)]
            if len(unconnected) == 0:
                break

            rate.sleep()

        # last check of connection status, raise error if any remain unconnected
        unconnected_pubs = [p for p in self.__publishers if not connected(p)]
        unconnected_subs = [s for s in self.__subscribers if not connected(s)]
        if len(unconnected_pubs) == 0 and len(unconnected_subs) == 0:
            return

        connected_pub_count = len(self.__publishers) - len(unconnected_pubs)
        connected_sub_count = len(self.__subscribers) - len(unconnected_subs)
        unconnected_pub_names = ', '.join([p.topic_name for p in unconnected_pubs])
        unconnected_sub_names = ', '.join([s.topic_name for s in unconnected_subs])

        err_msg = \
        (
            f"Timed out waiting for publisher/subscriber connections to establish\n"
            f"    Publishers:  {connected_pub_count} connected,"
            f" {len(unconnected_pubs)} not connected\n"
            f"                 not connected: [{unconnected_pub_names}]\n\n"
            f"    Subscribers: {connected_sub_count} connected,"
            f" {len(unconnected_subs)} not connected\n"
            f"                 not connected: [{unconnected_sub_names}]\n\n"
        )
        raise TimeoutError(err_msg)
