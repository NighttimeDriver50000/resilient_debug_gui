#!/usr/bin/env python
from __future__ import absolute_import, division, print_function
from builtins import *

import gc
import math

import rospy
from rospy import numpy_msg
from nav_msgs import msg as nav
from sensor_msgs import msg as sensor

import auto_pose

from PIL import Image, ImageDraw

import numpy as np

# Relative resolution: unitless (max range)/(cell width)
# Default: (MAP_CANVAS_SIZE / 2) * 2**MAP_DEFAULT_ZOOM
RELATIVE_RESOLUTION = 48 #2048


class Mapper (object):
    def __init__(self):
        self.absolute_resolution = None
        self.im = Image.new('L', 2 * ((2 * RELATIVE_RESOLUTION),), color=255)
        self.first_px = self.first_py = RELATIVE_RESOLUTION

        self.auto_pose = auto_pose.AutoPose('/odom')
        self.map_pub = rospy.Publisher('map',
                numpy_msg.numpy_msg(nav.OccupancyGrid), queue_size=1)
        self.pub_timer = rospy.Timer(rospy.Duration(1), self.timer_callback)
        self.scan_sub = rospy.Subscriber('/scan',
                sensor.LaserScan, self.scan_callback)

    @property
    def origin(self):
        first = self.auto_pose.first
        x = first.position.x - (self.first_px * self.absolute_resolution)
        y = first.position.y - (self.first_py * self.absolute_resolution)
        return (x, y)

    def point_to_pixel(self, x, y, origin=None):
        if origin is None:
            origin = self.origin
        origin_x, origin_y = origin
        px = (x - origin_x) / self.absolute_resolution
        py = (y - origin_y) / self.absolute_resolution
        return (px, py)

    def timer_callback(self, event):
        #sys.stderr.write('Enter timer_callback\n')
        if self.absolute_resolution is None:
            #sys.stderr.write('Exit timer_callback early\n')
            return None
        im = self.im
        msg = nav.OccupancyGrid()
        msg.info.resolution = self.absolute_resolution
        msg.info.width = im.width
        msg.info.height = im.height
        origin_x, origin_y = self.origin
        msg.info.origin.position.x = origin_x
        msg.info.origin.position.y = origin_y
        msg.info.origin.orientation = self.auto_pose.first.orientation
        msg.data = np.array(im.getdata(), dtype=np.int8)
        #sys.stderr.write('Publishing\n')
        self.map_pub.publish(msg)
        gc.collect()
        #sys.stderr.write('Exit timer_callback\n')

    def draw_scan(self, scan):
        #sys.stderr.write('Enter draw_scan\n')
        origin = self.origin
        cx, cy = self.point_to_pixel(self.auto_pose.x, self.auto_pose.y, origin)
        draw = ImageDraw.Draw(self.im)
        for i in xrange(len(scan.ranges)):
            hit_angle = scan.angle_min + (i * scan.angle_increment)
            hit_range = scan.ranges[i]
            maxed = False
            if hit_range == 0 or hit_range > scan.range_max:
                hit_range = scan.range_max
                maxed = True
            if hit_range < scan.range_min or hit_angle > scan.angle_max:
                continue
            hit_angle += self.auto_pose.heading
            hit_range /= self.absolute_resolution
            x = cx - (hit_range * math.sin(hit_angle))
            y = cy - (hit_range * math.cos(hit_angle))
            draw.line([cx, cy, x, y], 0, 1)
            if not maxed:
                draw.point([x, y], 100)
        #sys.stderr.write('Exit draw_scan\n')

    def scan_callback(self, msg):
        #sys.stderr.write('Enter scan_callback\n')
        if self.auto_pose.first is None:
            #sys.stderr.write('Exit scan_callback early\n')
            return
        if self.absolute_resolution is None:
            self.absolute_resolution = msg.range_max / RELATIVE_RESOLUTION
        #sys.stderr.write('Determining new size\n')
        px, py = self.point_to_pixel(self.auto_pose.x, self.auto_pose.y)
        im = self.im
        size = im.size
        paste = (0, 0)
        if px - RELATIVE_RESOLUTION < 0:
            size = (size[0] * 2, size[1])
            paste = (im.width, 0)
        elif px + RELATIVE_RESOLUTION >= im.width:
            size = (size[0] * 2, size[1])
        if py - RELATIVE_RESOLUTION < 0:
            size = (size[0], size[1] * 2)
            paste = (paste[0], im.height)
        elif py + RELATIVE_RESOLUTION >= im.height:
            size = (size[0], size[1] * 2)
        if size != im.size:
            #sys.stderr.write('Size changed to {}\n'.format(size))
            #sys.stderr.write('Allocating new image\n')
            im = Image.new('L', size, color=255)
            #sys.stderr.write('Pasting image data\n')
            im.paste(self.im, paste)
            #sys.stderr.write('New image created\n')
            self.im = im
            self.first_px += paste[0]
            self.first_py += paste[1]
        self.draw_scan(msg)
        gc.collect()
        #sys.stderr.write('Exit scan_callback\n')


def main():
    rospy.init_node('resilient_debug_mapper')
    mapper = Mapper()
    rospy.spin()
    #sys.stderr.write('Goodbye\n')
    raise SystemExit(0)


if __name__ == '__main__':
    main()
