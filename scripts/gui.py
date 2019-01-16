#!/usr/bin/env python
from __future__ import absolute_import, division, print_function
from builtins import *

import math
import os
from os import path
import sys

import rospy
from geometry_msgs import msg as geom
from sensor_msgs import msg as sensor

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


class SidebarWidget (Gtk.ScrolledWindow):
    def __init__(self):
        Gtk.ScrolledWindow.__init__(self)

        self.grid = Gtk.Grid()
        self.show_scan_check = Gtk.CheckButton.new_with_label(
                'Show current scan with landmarks')
        self.show_scan_check.set_sensitive(False)
        self.show_map_check = Gtk.CheckButton.new_with_label(
                'Show map instead of camera')
        self.show_map_check.set_sensitive(False)
        self.show_camera_check = Gtk.CheckButton.new_with_label(
                'Still show camera with map')
        self.show_camera_check.set_sensitive(False)

        self.grid.insert_column(0)
        self.grid.insert_row(0)
        self.grid.attach(self.show_scan_check, 0, 0, 1, 1)
        self.grid.attach(self.show_map_check, 0, 1, 1, 1)
        self.grid.attach(self.show_camera_check, 0, 2, 1, 1)
        
        self.add(self.grid)

    def do_get_preferred_width(self):
        return self.grid.get_preferred_width()

    def do_get_preferred_width_for_height(self, height):
        return self.grid.get_preferred_width_for_height(height)


def pil_to_pixbuf(im):
    pixels = GLib.Bytes(im.convert('RGB').tobytes('raw', 'RGB', 0, 1))
    return GdkPixbuf.Pixbuf.new_from_bytes(pixels, GdkPixbuf.Colorspace.RGB,
            False, 8, im.width, im.height, im.width * 3)


def transformed_polygon(polygon, rotate, translate):
    new_polygon = []
    for x, y in polygon:
        new_x = (x * math.cos(rotate)) - (y * math.sin(rotate))
        new_y = (x * math.cos(rotate)) + (y * math.cos(rotate))
        new_x += translate[0]
        new_y += translate[1]
        new_polygon.append((new_x, new_y))
    return new_polygon


class DisplayWidget (Gtk.EventBox):
    def __init__(self, win):
        Gtk.EventBox.__init__(self)

        self.win = win

        self.map_cx = 0
        self.map_cy = 0
        self.map_zoom = 2

        self.button1_down = False
        self.going_to_update = False
        self.crop_rect = (0, 0, 0, 0)
        self.motion_prev_x = None
        self.motion_prev_y = None

        self.image_widget = Gtk.Image()
        self.add(self.image_widget)

        self.camera_im = Image.new('RGB', (1, 1))
        self.im = Image.new('RGB', (CANVAS_SIZE,) * 2)
        self.update_image()
        self.connect('size-allocate', self.update_display)
        self.connect('button-press-event', self.update_buttons)
        self.connect('button-release-event', self.update_buttons)
        self.connect('motion-notify-event', self.handle_motion)
        self.connect('scroll-event', self.handle_scroll)
        GLib.timeout_add(50, self.request_full_update)

        self.camera_sub = rospy.Subscriber('/camera/rgb/image_raw',
                sensor.Image, self.camera_callback)

    def do_get_preferred_width(self):
        return self.win.get_size().width - 2

    def do_get_preferred_width_for_height(self, height):
        return self.do_get_preferred_width()

    def do_get_preferred_height(self):
        return self.win.get_size().height - 2

    def update_buttons(self, widget=None, event=None):
        prev_button1_down = self.button1_down
        self.button1_down = bool(event.state & Gdk.ModifierType.BUTTON1_MASK)
        if self.button1_down and not prev_button1_down:
            self.motion_prev_x = event.x
            self.motion_prev_y = event.y
            # TODO: update
            pass

    def handle_motion(self, widget, event):
        self.update_buttons(event=event)
        self.map_cx -= (event.x - self.motion_prev_x) * (2**self.map_zoom)
        self.map_cy -= (event.y - self.motion_prev_y) * (2**self.map_zoom)
        self.request_full_update()
        self.motion_prev_x = event.x
        self.motion_prev_y = event.y

    def handle_scroll(self, widget, event):
        if event.direction == Gdk.ScrollDirection.UP:
            if self.map_zoom > 0:
                self.map_zoom -= 1
        elif event.direction == Gdk.ScrollDirection.DOWN:
            self.map_zoom += 1
        self.request_full_update()

    def request_full_update(self):
        if not self.going_to_update:
            self.going_to_update = True
            self.update_display()
            GLib.timeout_add(40, self.do_full_update_now)
        return True

    def do_full_update_now(self, user_data=None):
        self.update_image()
        self.update_display()
        self.queue_draw()
        self.going_to_update = False
        return False

    pointer_polygon = [(0, -10), (8, 10), (0, 5), (-8, 10)]

    def draw_camera(self, resized, width, height):
        size = max(width, height)
        if size == 0:
            return
        camera_width = self.camera_im.width
        camera_height = self.camera_im.height
        if (camera_width / camera_height) > (width / height):
            new_width = width
            new_height = (width * camera_height) // camera_width
            x = (size - new_width) // 2
            y = (size - new_height) // 2
        else:
            new_width = (height * camera_width) // camera_height
            new_height = height
            x = (size - new_width) // 2
            y = (size - new_height) // 2
        camera_resized = self.camera_im.resize((new_width, new_height),
                Image.NEAREST)
        resized.paste(camera_resized, (x, y))

    def draw_on_virtual(self, resized, width, height):
        # Paste camera data
        self.draw_camera(resized, width, height)
        # TODO: draw
        draw = ImageDraw.Draw(resized)
        pass

    def update_display(self, widget=None, allocation=None, data=None):
        if allocation is None:
            allocation = self.get_allocation()
        win_width, win_height = self.win.get_size()
        width = min(win_width - 2, allocation.width)
        height = min(win_height - 2, allocation.height)
        size = max(width, height)
        resized = self.im.resize((size,) * 2, Image.NEAREST)
        self.draw_on_virtual(resized, width, height)
        x = int(math.ceil((size - width) / 2))
        y = int(math.ceil((size - height) / 2))
        if (2 * x) >= resized.width or (2 * y) > resized.height:
            return
        self.crop_rect = (x, y, resized.width - x, resized.height - y)
        cropped = resized.crop(self.crop_rect)
        pixbuf = pil_to_pixbuf(cropped)
        self.image_widget.set_from_pixbuf(pixbuf)

    def update_image(self):
        # TODO: render
        pass

    def camera_callback(self, msg):
        if msg.encoding != 'rgb8':
            sys.stderr.write('Camera must be rgb8, not {}.\n'.format(msg.encoding))
        self.camera_im = Image.frombytes('RGB', (msg.width, msg.height), msg.data)


class MainWindow (Gtk.Window):
    def __init__(self):
        Gtk.Window.__init__(self)

        # Create widgets
        self.display = DisplayWidget(self)
        self.sidebar = SidebarWidget()

        # Add widgets
        self.paned = Gtk.Paned()
        self.paned.pack1(self.sidebar, False, False)
        self.paned.pack2(self.display, True, False)
        self.add(self.paned)

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
