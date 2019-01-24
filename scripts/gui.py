#!/usr/bin/env python
from __future__ import absolute_import, division, print_function
from builtins import *

import array
import math
import os
from os import path
import sys

import rospy
from sensor_msgs import msg as sensor

from landmark_detection import msg as landmark

import auto_pose

import gi
gi.require_version('Gtk', '3.0')
from gi.repository import Gtk, Gdk, GLib

import cairo
from PIL import Image, ImageDraw

import tomlkit

MAP_CANVAS_SIZE = 1024
MAP_DEFAULT_ZOOM = 2
CONFIG_FILENAME = 'resilient_debug_gui.toml'

CONFIG_DIRPATH = path.join(path.expanduser('~'), '.config')
CONFIG_FILEPATH = path.join(CONFIG_DIRPATH, CONFIG_FILENAME)


class SidebarWidget (Gtk.ScrolledWindow):
    def __init__(self):
        Gtk.ScrolledWindow.__init__(self)

        self.grid = Gtk.Grid()
        self.show_scan_check = Gtk.CheckButton.new_with_label(
                'Show current scan with landmarks')
        self.show_scan_check.set_active(True)
        self.big_scan_check = Gtk.CheckButton.new_with_label(
                'Enlarge scan')
        self.big_scan_check.set_active(False)
        self.show_map_check = Gtk.CheckButton.new_with_label(
                'Show map instead of camera')
        self.show_map_check.set_sensitive(False)
        self.show_map_check.set_active(False)
        self.show_camera_check = Gtk.CheckButton.new_with_label(
                'Still show camera with map')
        self.show_camera_check.set_sensitive(False)
        self.show_camera_check.set_active(True)

        self.landmark_label = Gtk.Label.new('Landmark Colors')
        self.column_label = Gtk.Label.new('Column:')
        self.column_color = Gtk.ColorButton.new_with_rgba(Gdk.RGBA(0.8, 0.8, 0))
        self.wall_label = Gtk.Label.new('Wall:')
        self.wall_color = Gtk.ColorButton.new_with_rgba(Gdk.RGBA(0, 0.8, 0.8))

        self.grid.attach(self.show_scan_check, 0, 0, 5, 1)
        self.grid.attach(self.big_scan_check, 1, 1, 4, 1)
        self.grid.attach(self.show_map_check, 0, 2, 5, 1)
        self.grid.attach(self.show_camera_check, 1, 3, 4, 1)
        self.grid.attach(self.landmark_label, 0, 4, 5, 1)
        self.grid.attach(self.column_label, 1, 5, 1, 1)
        self.grid.attach(self.column_color, 2, 5, 1, 1)
        self.grid.attach(self.wall_label, 3, 5, 1, 1)
        self.grid.attach(self.wall_color, 4, 5, 1, 1)
        
        self.add(self.grid)

    def do_get_preferred_width(self):
        return self.grid.get_preferred_width()

    def do_get_preferred_width_for_height(self, height):
        return self.grid.get_preferred_width_for_height(height)


def pil_to_surface(im):
    r, g, b, a = im.convert('RGBA').split()
    data = Image.merge('RGBA', (b, g, r, a)).tobytes()
    return cairo.ImageSurface.create_for_data(array.array('B', data),
            cairo.FORMAT_ARGB32, im.width, im.height, im.width * 4)


def transformed_polygon(polygon, rotate, translate):
    new_polygon = []
    for x, y in polygon:
        new_x = (x * math.cos(rotate)) - (y * math.sin(rotate))
        new_y = (x * math.cos(rotate)) + (y * math.cos(rotate))
        new_x += translate[0]
        new_y += translate[1]
        new_polygon.append((new_x, new_y))
    return new_polygon


def get_color_tuple(color_chooser):
    rgba = color_chooser.get_rgba()
    use_alpha = color_chooser.get_use_alpha()
    if use_alpha:
        color = (rgba.red, rgba.green. rgba.blue, rgba.alpha)
    else:
        color = (rgba.red, rgba.green, rgba.blue)
    return tuple(int(255 * c) for c in color)


class DisplayWidget (Gtk.EventBox):
    def __init__(self, win):
        Gtk.EventBox.__init__(self)

        self.win = win

        self.map_cx = 0
        self.map_cy = 0
        self.map_zoom = MAP_DEFAULT_ZOOM

        self.button1_down = False
        self.going_to_update = False
        self.crop_rect = (0, 0, 0, 0)
        self.motion_prev_x = None
        self.motion_prev_y = None
        self.image_surface = None
        
        self.camera_im = Image.new('RGB', (1, 1))
        self.scan_msg = None
        self.landmarks = []

        self.image_widget = Gtk.DrawingArea()
        self.add(self.image_widget)
        self.im = Image.new('RGB', (MAP_CANVAS_SIZE,) * 2)
        self.update_image()
        self.connect('size-allocate', self.update_display)
        self.connect('button-press-event', self.update_buttons)
        self.connect('button-release-event', self.update_buttons)
        self.connect('motion-notify-event', self.handle_motion)
        self.connect('scroll-event', self.handle_scroll)

        GLib.timeout_add(50, self.request_full_update)
        self.connect('draw', self.draw_callback)
        self.auto_pose = auto_pose.AutoPose('/odom')
        self.camera_sub = rospy.Subscriber('/camera/rgb/image_raw',
                sensor.Image, self.camera_callback)
        self.scan_sub = rospy.Subscriber('/scan',
                sensor.LaserScan, self.scan_callback)
        self.rel_landmarks_sub = rospy.Subscriber('/relative_landmarks',
                landmark.LandmarkMsgs, self.rel_landmarks_callback)

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
        self.queue_draw()
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

    def draw_scan_landmarks(self, resized, width, height):
        scan = self.scan_msg
        if scan is None or not self.win.sidebar.show_scan_check.get_active():
            return
        draw = ImageDraw.Draw(resized)
        big_scan = self.win.sidebar.big_scan_check.get_active()
        size = int(min(width, height) * (0.6 if big_scan else 0.2))
        r = size / 2
        cx = r + ((resized.width - width) / 2)
        cy = r + ((resized.height - height) / 2)
        draw.ellipse([cx - r, cy - r, cx + r, cy + r], (0, 0, 0), (255, 0, 0))
        for i in xrange(len(scan.ranges)):
            hit_angle = scan.angle_min + (i * scan.angle_increment)
            hit_range = scan.ranges[i]
            if hit_range == 0 or hit_range > scan.range_max:
                hit_range = scan.range_max
            if hit_range < scan.range_min or hit_angle > scan.angle_max:
                continue
            hit_range *= r / scan.range_max
            x = cx - (hit_range * math.sin(hit_angle))
            y = cy - (hit_range * math.cos(hit_angle))
            draw.line([cx, cy, x, y], (255, 255, 255), 1)
        draw.line([cx, cy, cx, cy - r], (255, 0, 0), 1)
        for (wall, lx, ly) in self.landmarks:
            if wall:
                color = get_color_tuple(self.win.sidebar.wall_color)
            else:
                color = get_color_tuple(self.win.sidebar.column_color)
            x = cx - (ly * r / scan.range_max)
            y = cy - (lx * r / scan.range_max)
            draw.ellipse([x - 2, y - 2, x + 2, y + 2], color)

    def draw_on_virtual(self, resized, width, height):
        # Paste camera data
        self.draw_camera(resized, width, height)
        self.draw_scan_landmarks(resized, width, height)
        # TODO: draw
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
        self.image_surface = pil_to_surface(cropped)

    def update_image(self):
        # TODO: render
        pass

    def draw_callback(self, widget, cr, data=None):
        if self.image_surface is None:
            return True
        cr.set_source_surface(self.image_surface)
        cr.paint()
        return True

    def camera_callback(self, msg):
        if msg.encoding != 'rgb8':
            sys.stderr.write('Camera must be rgb8, not {}.\n'.format(msg.encoding))
        self.camera_im = Image.frombytes('RGB', (msg.width, msg.height), msg.data)
    
    def scan_callback(self, msg):
        self.scan_msg = msg
    
    def rel_landmarks_callback(self, msg):
        self.landmarks = []
        for column in msg.columns:
            self.landmarks.append((False, column.position.x, column.position.y))
        for wall in msg.walls:
            self.landmarks.append((True, wall.position.x, wall.position.y))


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
