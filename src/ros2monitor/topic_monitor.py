#!/usr/bin/env python
#
# Copyright (C) 2022 Hirain Technologies - All Rights Reserved
#

from rclpy.node import Node

from .topic_info import TopicInfo


class TopicMonitor(Node):

    def __init__(self, name):
        super().__init__(name)
        self._current_topic_list = []
        self._topics = {}
        self.refresh_topics()

    def refresh_topics(self):
        topic_list = self.get_topic_names_and_types()
        if self._current_topic_list != topic_list:
            self._current_topic_list = topic_list

            new_topics = {}
            for topic_name, topic_types in topic_list:
                if topic_name not in self._topics \
                        or self._topics[topic_name]['type'] != topic_types[0]:
                    topic_info = TopicInfo(self, topic_name, topic_types[0])

                    message_instance = None
                    if topic_info.message_class:
                        message_instance = topic_info.message_class()

                    new_topics[topic_name] = {
                        'message': message_instance,
                        'info': topic_info,
                        'type': topic_types[0],
                    }
                else:
                    new_topics[topic_name] = self._topics[topic_name]
                    del self._topics[topic_name]

            # clean up old topics
            for topic_name in list(self._topics.keys()):
                del self._topics[topic_name]

            # switch to new topic dict
            self._topics = new_topics

    def get_info(self, topic_name):
        if topic_name not in self._topics:
            return
        return self._topics[topic_name]['info']

    def is_monitoring(self, topic_name):
        if topic_name not in self._topics:
            return False
        return self._topics[topic_name]['info'].is_monitoring()
