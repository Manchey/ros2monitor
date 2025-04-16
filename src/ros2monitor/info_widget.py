#!/usr/bin/env python


class InfoWidget:

    def __init__(self, screen, color="y"):
        self._screen = screen
        self._color = color

    def render(self):
        help_message = """
        Commands for all:
        [ q  | Esc ] -- quit
        [ t        ] -- go to show topic info
        [ H        ] -- go to show help info

        Commands for Topic Monitor and Message Monitor:
        [ Up Arrow    | w | k ] -- move up one line
        [ Down Arrow  | s | j ] -- move down one line
        [ PgDn        | ^d    ] -- show next page
        [ PgUp        | ^u    ] -- show previous page

        Commands for Topic Monitor:
        [ Right Arrow | d | l ] -- enter the selected Topic
        [ Left Arrow  | a | h ] -- go back to the upper level

        Commands for Message Monitor:
        [ Right Arrow | d | l ] -- expand the selected sub message
        [ Left Arrow  | a | h ] -- fold the selected sub message
        [ n                   ] -- go to next item
        [ N                   ] -- go to previous item
        [ o                   ] -- open sub topic folder
        [ c                   ] -- close sub topic folder
        """
        lines = help_message.splitlines()
        for y, line in enumerate(lines):
            self._screen.addstr(y, 0, line, self._color)
