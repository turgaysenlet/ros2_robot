import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Header
from geometry_msgs.msg import Twist, Vector3
from gps_msgs.msg import GPSFix, GPSStatus
from sensor_msgs.msg import Joy
import logging
import sys
import numpy as np
import time
import os
from gps3 import gps3
import gps
import serial
import pynmea2
import io
from dateutil.parser import parse as date_parse
import calendar

class SimpleGps(Node):
    port = "/dev/ttyUSB0"
    gpsd = []
    gps_frame = "gps"
    def __init__(self):
        super().__init__('simple_gps_node')
        print('SimpleGps...')
        print('initHardware...')
        self.initHardware()
        print('initHardware.')
        print('initPublishers...')
        self.initPublishers()        
        print('initPublishers.')        
        print('SimpleGps.')

    def spinForever(self):
        self.get_logger().info('spinForever...')
        while True:            
            self.readGpsData()
            self.get_logger().debug('spinOnce...')
            rclpy.spin_once(self, timeout_sec=0.05)
            self.get_logger().debug('spinOnce.')
        self.get_logger().info('spinForever.')

    def publishData(self, report):
        self.get_logger().debug('publishData...')
        h = Header(stamp=self.get_clock().now().to_msg(), frame_id=self.gps_frame)
        s = GPSStatus(header = h)
        fix = GPSFix(header=h, status = s)
        fix.latitude = report['lat']
        fix.longitude = report['lon']
        dt = date_parse(report['time'])
        timestamp1 = calendar.timegm(dt.timetuple())
        fix.time = float(timestamp1)
        print(fix)
        self.publisher_.publish(fix)
        self.get_logger().debug('publishData.')

    def readGpsData(self):
        self.get_logger().info('readGpsData...')
        report = self.gpsd.next() #
        # print(report)
        # print('')
        if 'class' in report.keys() and report['class'] == 'TPV':
            
            if 'lon' in report.keys() and 'lat' in report.keys() and 'time' in report.keys():
                self.get_logger().info('Lon: %f, Lat: %f, Time: %s (%f)' % (report['lon'], report['lat'], report['time'], float(calendar.timegm(date_parse(report['time']).timetuple()))))
                self.publishData(report)
            elif 'time' in report.keys():
                self.get_logger().info('Time = %s' % (report['time']))
                
        self.get_logger().debug('readGpsData.')


    def initPublishers(self):
        self.get_logger().info('initPublishers...')
        self.publisher_ = self.create_publisher(GPSFix, '/gps_fix', 10)
        self.get_logger().info('initPublishers.')
        
    def initHardware(self):
        self.get_logger().info('initHardware...')
        self.gpsd = gps.gps(host='127.0.0.1', port=4000, verbose=1, mode=gps.WATCH_ENABLE|gps.WATCH_NEWSTYLE) 
        self.get_logger().info('initHardware.')

def main(args=None):
    # print(sys.path)
    print('main...')
    rclpy.init(args=args)

    node = SimpleGps()
    try:
        print('main spin...')
        node.spinForever()
        
    except KeyboardInterrupt:
        print('main keyboard exception.')
        pass
    print('main spin.')
    print('main destroy...')
    node.destroy_node()
    print('main destroy.')
    print('main shutdown...')
    rclpy.shutdown()
    print('main shutdown.')
    print('main.')


if __name__ == '__main__':
    main()
