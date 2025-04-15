#!/usr/bin/env python
#
# Copyright (C) 2022 Hirain Technologies - All Rights Reserved
#

import rclpy.serialization
from ros2topic.verb.hz import ROSTopicHz

from .helpers import get_message_class
from .topic_bw import TopicBandwidth


class TopicInfo:

    def __init__(self, node, topic_name, topic_type):
        self._node = node
        self._clock = self._node.get_clock()
        self._topic_name = topic_name
        self._ros_topic_hz = ROSTopicHz(node, 100)
        self._ros_topic_bw = TopicBandwidth(node, 100)
        self.error = None
        self._subscriber = None
        self.monitoring = False
        self._reset_data()
        self.message_class = None
        self._topic_type = topic_type
        if topic_type is None:
            self.error = 'No topic types associated with topic: ' % topic_name
        try:
            self.message_class = get_message_class(topic_type)
        except Exception as e:
            print(e)
            self.message_class = None

        if self.message_class is None:
            self.error = 'can not get message class for type "%s"' % topic_type

    def _reset_data(self):
        self.last_message = None

    def toggle_monitoring(self):
        if self.monitoring:
            self.stop_monitoring()
        else:
            self.start_monitoring()

    def start_monitoring(self):
        if self.message_class is not None:
            self.monitoring = True
            self._subscriber = self._node.create_subscription(
                self.message_class, self._topic_name, self.message_callback,
                qos_profile=10, raw=True)

    def stop_monitoring(self):
        self.monitoring = False
        self._reset_data()
        if self._subscriber is not None:
            self._node.destroy_subscription(self._subscriber)
            self._subscriber = None

    def is_monitoring(self):
        return self.monitoring

    def get_hz(self):
        return self._ros_topic_hz.get_hz(self._topic_name)

    def get_last_printed_tn(self):
        return self._ros_topic_hz.get_last_printed_tn(self._topic_name)

    def message_callback(self, data):
        self.last_message = rclpy.serialization.deserialize_message(data, self.message_class)
        self._ros_topic_hz.callback_hz(self.last_message, self._topic_name)
        self._ros_topic_bw.callback(data)

    def get_bw_text(self):
        return self._ros_topic_bw.get_bw_text()

    def get_hz_text(self):
        rate = None
        hz_result = self.get_hz()
        if hz_result is not None:
            rate, _, _, _, _ = hz_result
            rate *= 1e9
        rate_text = '%1.2f' % rate if rate is not None else 'unknown'
        return rate_text

    def get_topic_type(self):
        return self._topic_type
