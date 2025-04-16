#!/usr/bin/env python


class ScrollWidget:

    def __init__(self, screen, start_row):
        self._screen = screen
        self._start_row = start_row

        self._height, self._width = self._screen.getmaxyx()
        self._widget_height = self._height - self._start_row
        self._pos = 0
        self._top = 0
        self._end = 0

        self._highlight_color = "y"

    def resize(self):
        self._height, self._width = self._screen.getmaxyx()
        self._widget_height = self._height - self._start_row

    def go(self, step):
        self._pos = self._pos + step
        self._pos = min(self._pos, self._end - 1)
        self._pos = max(0, self._pos)

        if self._pos >= self._top + self._widget_height:
            self._top = self._pos - self._widget_height + 1

        if self._pos < self._top:
            self._top = self._pos

    def go_up(self):
        self.go(-1)

    def go_down(self):
        self.go(1)

    def go_to_row(self, row):
        self.go(row - self._pos)

    def go_to_row_and_move_top(self, row):
        self.go(row - self._pos)
        self.go_to_next_page()
        self.go_to_previous_page()
        self.go(row - self._pos)

    def go_to_next_page(self):
        self.go(self._widget_height)

    def go_to_previous_page(self):
        self.go(-self._widget_height)

    def render_scroll_line(self, row, column, message, color):
        if row == self._pos:
            color = self._highlight_color
        if self._top <= row < (self._top + self._widget_height):
            self._screen.addstr(row + self._start_row - self._top, column, message, color)
