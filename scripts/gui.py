#!/usr/bin/env python
from __future__ import absolute_import, division, print_function
from builtins import *

import os
from os import path
import sys

import rospy
from geometry_msgs import msg as geom

import gi
gi.require_version('Gtk', '3.0')
from gi.repository import Gtk, GdkPixbuf, Gdk, GLib, GObject

import cairo
from PIL import Image, ImageDraw, ImageFont

import tomlkit

CANVAS_SIZE = 1024
CONFIG_FILENAME = 'resilient_debug_gui.toml'

CONFIG_DIRPATH = path.join(path.expanduser('~'), '.config')
CONFIG_FILEPATH = path.join(CONFIG_DIRPATH, CONFIG_FILENAME)


class MainWindow (Gtk.Window):
    def __init__(self):
        Gtk.window.__init__(self)

        self.set_default_size(1280, 720)


def main():
    # Initialize ROS
    rospy.init_node('resilient_debug_gui', anonymous=True)
    rospy.on_shutdown(Gtk.main_quit)

    # Create and show window
    win = MainWindow()
    win.connect('destroy', Gtk.main_quit)
    win.show_all()

    # Main loop
    Gtk.main()

    # Join event threads
    if not rospy.is_shutdown():
        rospy.signal_shutdown('GTK+ exited main')
        try:
            rospy.spin()
        except rospy.ROSInitException:
            pass
    sys.stderr.write('Goodbye\n')
    raise SystemExit(0)


if __name__ == '__main__':
    main()
