#!/usr/bin/env python

from .scroll_widget import ScrollWidget


class TopicWidget(ScrollWidget):

    def __init__(self, screen, topic_monitor):
        super().__init__(screen, 1)
        self._topic_monitor = topic_monitor
        self._end = len(self._topic_monitor._topics)

        self._topic_timeout = 10.0
        self._column_width = 50

    def get_cur_focus_topic(self):
        focus_topic, _ = list(self._topic_monitor._topics.items())[self._pos]
        return focus_topic

    def refresh_topic(self):
        self._topic_monitor.refresh_topics()
        self._end = len(self._topic_monitor._topics)
        if self._pos > self._end:
            self._pos = 0
            self._top = 0

    def render(self):
        self.refresh_topic()
        self._screen.addstr(0, 0, " " * (self._width - 1), "b")
        self._screen.addstr(0, self._column_width * 0, "topic_name", "b")
        self._screen.addstr(0, self._column_width * 1, "topic_type", "b")
        topics = self._topic_monitor._topics
        for idx, item in enumerate(list(topics.items())):
            topic_name, topic_item = item
            self.render_scroll_line(idx, self._column_width * 0, topic_name, "w")
            self.render_scroll_line(idx, self._column_width * 1, topic_item["type"], "w")
