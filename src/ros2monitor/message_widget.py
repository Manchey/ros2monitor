#!/usr/bin/env python
#
# Copyright (C) 2022 Hirain Technologies - All Rights Reserved
#

from .scroll_widget import ScrollWidget


class MessageWidget(ScrollWidget):

    def __init__(self, screen, topic_name, topic_info):
        super().__init__(screen, 5)
        self._screen = screen
        self._topic_name = topic_name
        self._topic_info = topic_info

        self._topic_timeout = 10.0
        self._last_rate_text = 'unknown'

        self._indent = 2
        self._column_width = 15

        self._height, self._width = self._screen.getmaxyx()

        self._show_value = {}
        self._split_sign = '@'
        self._topic_row = {self._split_sign: 0}

    def render(self):
        rate_text = self._topic_info.get_hz_text()
        bandwidth_text = self._topic_info.get_bw_text()

        last_valid_time = self._topic_info.get_last_printed_tn()
        current_rostime = self._topic_info._clock.now().nanoseconds
        if rate_text != 'unknown' \
                or last_valid_time + self._topic_timeout * 1e9 <= current_rostime:
            # If the last time this was valid was less than the topic timeout param
            # then ignore it
            self._last_rate_text = rate_text

        self._screen.addstr(0, 0, 'topic_name', 'w')
        self._screen.addstr(1, 0, 'type', 'w')
        self._screen.addstr(2, 0, 'hz', 'w')
        self._screen.addstr(3, 0, 'bw', 'w')
        self._screen.addstr(4, 0, 'data:', 'w')

        self._screen.addstr(0, self._column_width, self._topic_name, 'g')
        self._screen.addstr(1, self._column_width, self._topic_info.get_topic_type(), 'g')
        self._screen.addstr(2, self._column_width, self._last_rate_text, 'g')
        self._screen.addstr(3, self._column_width, bandwidth_text, 'g')

        self._end = self.render_message(
            self._split_sign, self._topic_name, self._topic_info.last_message) + 1

    def go_right(self):
        self.expand_topic(self._current_topic)

    def expand_topic(self, topic):
        if self._show_value[topic]:
            [self.expand_topic(t)
             for t in self._show_value.keys() if self.is_parent_topic(topic, t)]
        else:
            self._show_value[topic] = True

    def open_all_folder(self):
        self.show_all_sub_folder(True)

    def close_all_folder(self):
        self.show_all_sub_folder(False)

    def show_all_sub_folder(self, is_show):
        for topic in [t for t in self._show_value.keys() if self._current_topic in t]:
            self._show_value[topic] = is_show

    def go_left(self):
        # If current topic is open, close current topic, otherwise go back to parent topic.
        # If current topic is already root topic and is closed, go back to topic widget.
        if not self._show_value[self._current_topic] and self._topic_row[self._current_topic] == 0:
            return True
        elif self._show_value[self._current_topic]:
            self._show_value[self._current_topic] = False
        else:
            self.go_to_row(self._topic_row[self.parent_topic(self._current_topic)])
        return False

    def go_to_next_item(self):
        self.go_to_item(1)

    def go_to_previous_item(self):
        self.go_to_item(-1)

    def go_to_item(self, step):
        parent_topic = self.parent_topic(self._current_topic)
        items = [t for t in self._topic_row.keys() if parent_topic == self.parent_topic(t)]
        for i, topic in enumerate(items):
            if topic == self._current_topic and i + step in range(len(items)):
                self.go_to_row_and_move_top(self._topic_row[items[i+step]])
                return

    def parent_topic(self, topic):
        return topic[:topic.rfind(self._split_sign)]

    def is_parent_topic(self, topic, sub_topic):
        return self.parent_topic(sub_topic) == topic

    def render_message(self, parent_topic, topic_name, message, row=0, col=0):
        topic_full_name = self._split_sign.join([parent_topic, topic_name])
        if row == self._pos:
            self._current_topic = topic_full_name
        if topic_full_name not in self._show_value:
            self._show_value[topic_full_name] = False
        self._topic_row[topic_full_name] = row

        head = 'v ' if self._show_value[topic_full_name] else '> '
        tail = ''
        current_row = row

        if hasattr(message, 'get_fields_and_field_types'):
            if self._show_value[topic_full_name]:
                for slot_name in message.get_fields_and_field_types().keys():
                    row = self.render_message(topic_full_name, slot_name,
                                              getattr(message, slot_name),
                                              row + 1, col + self._indent)
            else:
                tail = '  [+%d...]' % len(message.get_fields_and_field_types().keys())

        elif type(message) in (list, tuple) and \
                (len(message) > 0) and \
                hasattr(message[0], '__slots__'):
            if self._show_value[topic_full_name]:
                for index, slot in enumerate(message):
                    row = self.render_message(topic_full_name, topic_name + '[%d]' % index,
                                              slot, row + 1, col + self._indent)
            else:
                tail = '  [+%d...]' % len(message)

        else:
            head = '  '
            tail = ': '
            self.render_scroll_line(row, col + len(head + topic_name + tail), repr(message), 'g')
            self._show_value[topic_full_name] = False

        self.render_scroll_line(current_row, col, head + topic_name + tail, 'w')
        return row
