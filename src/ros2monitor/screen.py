#!/usr/bin/env python

import curses
import os
import threading
import time

import rclpy
from rclpy.executors import MultiThreadedExecutor

from .info_widget import InfoWidget
from .message_widget import MessageWidget
from .topic_monitor import TopicMonitor
from .topic_widget import TopicWidget


class Screen:

    RENDER_MESSAGE = 0
    RENDER_TOPIC = 1
    RENDER_INTER_CMD_INFO = 2

    KEY_UP = [ord("w"), ord("k"), curses.KEY_UP]
    KEY_DOWN = [ord("s"), ord("j"), curses.KEY_DOWN]
    KEY_LEFT = [ord("a"), ord("h"), curses.KEY_LEFT]
    KEY_RIGHT = [ord("d"), ord("l"), curses.KEY_RIGHT, 10]  # 10 for ENTER
    KEY_QUIT = [ord("q"), ord("Q"), 27]  # 27 for ESC
    KEY_HELP = [ord("H")]
    KEY_TOPIC = [ord("t")]
    KEY_NEXT_ITEM = [ord("n")]
    KEY_PREVIOUS_ITEM = [ord("N")]
    KEY_OPEN = [ord("o")]
    KEY_CLOSE = [ord("c")]
    KEY_NEXT_PAGE = [curses.KEY_NPAGE, 4]  # 4 for ^d
    KEY_PREVIOUS_PAGE = [curses.KEY_PPAGE, 21]  # 21 for ^u

    def __init__(self):
        self._state = Screen.RENDER_INTER_CMD_INFO

        self._topic_monitor = TopicMonitor("ros2monitor")
        self._monitor_thread = threading.Thread(target=self.spinner, args=(self._topic_monitor,))

        self._handlers = {
            Screen.RENDER_INTER_CMD_INFO: self.info_widget_handle_input,
            Screen.RENDER_TOPIC: self.topic_widget_handle_input,
            Screen.RENDER_MESSAGE: self.message_widget_handle_input,
        }
        self._can_run = False
        self._refresh_rate = 100

    def __call__(self, stdscr):
        # create curses window object
        self._stdscr = stdscr

        # some terminal init setting
        self._stdscr.nodelay(True)
        curses.curs_set(0)
        curses.noecho()
        curses.cbreak()
        self._stdscr.keypad(True)

        # init color
        curses.start_color()
        self._stdscr.bkgd(curses.COLOR_BLACK)
        curses.init_pair(1, curses.COLOR_GREEN, curses.COLOR_BLACK)
        curses.init_pair(2, curses.COLOR_YELLOW, curses.COLOR_BLACK)
        curses.init_pair(3, curses.COLOR_RED, curses.COLOR_BLACK)
        curses.init_pair(4, curses.COLOR_WHITE, curses.COLOR_BLACK)
        curses.init_pair(5, curses.COLOR_BLACK, curses.COLOR_WHITE)
        self._color = {
            "g": curses.color_pair(1),
            "y": curses.color_pair(2),
            "r": curses.color_pair(3),
            "w": curses.color_pair(4),
            "b": curses.color_pair(5),
        }

        self._stdscr.refresh()
        self._stdscr.clear()

        self._topic_widget = TopicWidget(self, self._topic_monitor)
        self._message_widget = None
        self._info_widget = InfoWidget(self)
        self._cur_widget = self._info_widget

        self._can_run = True
        self.run()

    def spinner(self, node):
        self._executor = MultiThreadedExecutor()
        self._executor.add_node(node)
        self._executor.spin()

    def run(self):
        self._monitor_thread.start()
        while self._can_run:
            ch = self._stdscr.getch()
            if ch in Screen.KEY_QUIT:
                self._can_run = False
                break
            if ch in [curses.KEY_RESIZE]:
                self.resize()
                continue

            self._handlers[self._state](ch)
            self.render()

            time.sleep(1 / self._refresh_rate)

        self.shutdown()

    def switch_to_info_widget(self):
        self._state = Screen.RENDER_INTER_CMD_INFO
        self._cur_widget = self._info_widget

    def switch_to_topic_widget(self):
        self._state = Screen.RENDER_TOPIC
        self._cur_widget = self._topic_widget

    def switch_to_message_widget(self):
        topic_name = self._topic_widget.get_cur_focus_topic()
        topic_info = self._topic_monitor.get_info(topic_name)
        if topic_info is None:
            return
        self._state = Screen.RENDER_MESSAGE
        topic_info.start_monitoring()
        self._message_widget = MessageWidget(self, topic_name, topic_info)
        self._cur_widget = self._message_widget

    def info_widget_handle_input(self, ch):
        if ch in Screen.KEY_TOPIC:
            self.switch_to_topic_widget()

    def topic_widget_handle_input(self, ch):
        if ch in Screen.KEY_HELP:
            self.switch_to_info_widget()
        elif ch in Screen.KEY_RIGHT:
            self.switch_to_message_widget()
        elif ch in Screen.KEY_UP:
            self._cur_widget.go_up()
        elif ch in Screen.KEY_DOWN:
            self._cur_widget.go_down()
        elif ch in Screen.KEY_NEXT_PAGE:
            self._cur_widget.go_to_next_page()
        elif ch in Screen.KEY_PREVIOUS_PAGE:
            self._cur_widget.go_to_previous_page()

    def message_widget_handle_input(self, ch):
        if ch in Screen.KEY_HELP:
            self.switch_to_info_widget()
        elif ch in Screen.KEY_TOPIC:
            self.switch_to_topic_widget()
        elif ch in Screen.KEY_UP:
            self._cur_widget.go_up()
        elif ch in Screen.KEY_DOWN:
            self._cur_widget.go_down()
        elif ch in Screen.KEY_RIGHT:
            self._cur_widget.go_right()
        elif ch in Screen.KEY_LEFT:
            if self._cur_widget.go_left():
                self.switch_to_topic_widget()
        elif ch in Screen.KEY_NEXT_ITEM:
            self._cur_widget.go_to_next_item()
        elif ch in Screen.KEY_PREVIOUS_ITEM:
            self._cur_widget.go_to_previous_item()
        elif ch in Screen.KEY_NEXT_PAGE:
            self._cur_widget.go_to_next_page()
        elif ch in Screen.KEY_PREVIOUS_PAGE:
            self._cur_widget.go_to_previous_page()
        elif ch in Screen.KEY_OPEN:
            self._cur_widget.open_all_folder()
        elif ch in Screen.KEY_CLOSE:
            self._cur_widget.close_all_folder()

    def render(self):
        if self._cur_widget is not None:
            self._stdscr.erase()
            self._cur_widget.render()
            self._stdscr.refresh()

    def resize(self):
        curses.update_lines_cols()
        self._topic_widget.resize()
        if self._message_widget is not None:
            self._message_widget.resize()
        self._stdscr.clear()
        self._stdscr.refresh()

    def getmaxyx(self):
        return self._stdscr.getmaxyx()

    def shutdown(self):
        self._topic_monitor.destroy_node()
        self._executor.shutdown()
        self._monitor_thread.join()
        curses.nocbreak()
        self._stdscr.keypad(True)
        curses.echo()
        curses.endwin()

    def addstr(self, y, x, content, color):
        content = " ".join(content.split("\n"))
        if 0 <= y < curses.LINES and 0 <= x and x < curses.COLS - 1:
            self._stdscr.addstr(y, x, content[: curses.COLS - x - 1], self._color[color])


def main():
    rclpy.init()
    screen = Screen()
    try:
        curses.wrapper(screen)
    except curses.error as e:
        print("Your terminal lacks the ability to clear the screen or position the cursor.")
        print("\t-current TERM: {}\n\t-curses error: {}".format(os.environ.get("TERM"), str(e)))

        try:
            print("Try to force set env TERM to linux and restart ros2monitor")
            os.environ["TERM"] = "linux"
            curses.wrapper(screen)
        except curses.error as e:
            print("Restart failed, curses error: {}".format(os.environ.get("TERM"), str(e)))

    rclpy.shutdown()


if __name__ == "__main__":
    main()
