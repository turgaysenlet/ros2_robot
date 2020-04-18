import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import Joy
import logging
import sys
import numpy as np
import maestro
import time
import os


class RemoteControl(Node):
    STEERING_SERVO_1_NO = 1
    STEERING_SERVO_2_NO = 2
    STEERING_SPEED = 50
    STEERING_ACCEL = 50
    CENTER_STEERING_TARGET = 6000
    MIN_STEERING = -1
    MAX_STEERING = 1
    MIN_STEERING_TARGET = 5000
    MAX_STEERING_TARGET = 7000

    MIN_DEADZONE = -0.02
    MAX_DEADZONE = 0.02

    SPEED_SERVO_NO = 0
    SPEED_SPEED = 3
    SPEED_ACCEL = 3
    STOP_SPEED_TARGET = 6000
    MIN_SPEED = -1
    MAX_SPEED = 1
    MIN_SPEED_TARGET = 5700
    MAX_SPEED_TARGET = 6200

    IGNITION_MIN_SPEED_TARGET = 4400
    IGNITION_MED_SPEED_TARGET = 6000
    IGNITION_MAX_SPEED_TARGET = 7200

    BUTTON_IGNITION = 9
    BUTTON_X = 0
    BUTTON_A = 1
    BUTTON_B = 2
    BUTTON_Y = 3

    buttons_old = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

    def __init__(self):
        super().__init__('remote_control_node')
        print('RemoteControl...')
        print('initServo...')
        self.initServo()
        print('initServo.')
        print('initSubscribers...')
        self.initSubscribers()        
        print('initSubscribers.')
        print('RemoteControl.')

    def initSubscribers(self):
        self.twistSubscriber = self.create_subscription(
            Twist,
            'cmd_vel',
            self.twistListenerCallback,
            10)
        self.twistSubscriber  # prevent unused variable warning
        self.joySubscriber = self.create_subscription(
            Joy,
            'joy',
            self.joyListenerCallback,
            10)
        self.twistSubscriber  # prevent unused variable warning
        
    def twistListenerCallback(self, msg):        
        self.get_logger().info('Received twist: "%s" "%s"' % (msg.linear.x, msg.angular.z))
        if msg.angular.z == 0 and msg.linear.x == 0:
            self.stop()
        else:
            self.setSpeed(msg.linear.x * 2.0)
            self.setSteering(msg.angular.z * 2.0)
    
    def joyListenerCallback(self, msg):                
        # self.get_logger().info('Received joy: "%s" "%s"' % (msg.axes, msg.buttons))
        if self.buttons_old[self.BUTTON_IGNITION] == 0 and msg.buttons[self.BUTTON_IGNITION] == 1:
            print('Ignition')
            self.ignitionMotor()
        if self.buttons_old[self.BUTTON_A] == 0 and msg.buttons[self.BUTTON_A] == 1:
            self.speak('Hello Kaan', 1)
        if self.buttons_old[self.BUTTON_X] == 0 and msg.buttons[self.BUTTON_X] == 1:
            self.speak('Bobby', 1)
        if self.buttons_old[self.BUTTON_Y] == 0 and msg.buttons[self.BUTTON_Y] == 1:
            self.speak('How are you today?', 1)
        if self.buttons_old[self.BUTTON_B] == 0 and msg.buttons[self.BUTTON_B] == 1:
            self.speak('My name is Bobby, what is yours?', 1)
        # if msg.angular.z == 0 and msg.linear.x == 0:
        #     self.stop()
        # else:
        #     self.setSpeed(msg.linear.x * 2.0)
        #     self.setSteering(msg.angular.z * 2.0)
        self.buttons_old = msg.buttons
    def speak(self, text, async):
        if async:
            os.system('espeak ' + '"' + text +'"' + '&')
        else:
            os.system('espeak ' + '"' + text +'"')

    def ignitionMotor(self):
        time.sleep(1)
        os.system("espeak Ignition")
        self.setServoConfig(self.SPEED_SERVO_NO, 0, 0)
        os.system("espeak 1")
        time.sleep(1)
        self.servo_.setTarget(self.SPEED_SERVO_NO, self.IGNITION_MED_SPEED_TARGET)
        os.system("espeak 2")
        time.sleep(1)
        self.servo_.setTarget(self.SPEED_SERVO_NO, self.IGNITION_MIN_SPEED_TARGET)
        os.system("espeak 3")        
        time.sleep(1)
        # self.servo_.setTarget(self.SPEED_SERVO_NO, self.IGNITION_MAX_SPEED_TARGET)
        # os.system("espeak 4")
        # time.sleep(1)
        self.servo_.setTarget(self.SPEED_SERVO_NO, self.IGNITION_MED_SPEED_TARGET)
        os.system("espeak Done")
        self.initServoConfig()

    def setSteering(self, steering):
        original_steering = steering
        # steering = -steering

        # Clip input speed
        if steering > self.MAX_STEERING:
            steering = self.MAX_STEERING
        elif steering < self.MIN_STEERING:
            steering = self.MIN_STEERING
        elif steering < self.MAX_DEADZONE and steering > self.MIN_DEADZONE:
            steering = 0
        
        # Stretch to match target
        if steering == 0:
            steering_target = self.CENTER_STEERING_TARGET;
        elif steering > 0:
            steering_target = steering * (self.MAX_STEERING_TARGET - self.CENTER_STEERING_TARGET) / self.MAX_STEERING + self.CENTER_STEERING_TARGET
        elif steering < 0:
            steering_target = steering * (self.MIN_STEERING_TARGET - self.CENTER_STEERING_TARGET) / self.MIN_STEERING + self.CENTER_STEERING_TARGET

        steering_target = int(steering_target)
        
        print('Input steering: ' + str(original_steering) + ', clipped:' + str(steering) + ', target: ' + str(steering_target))
        self.servo_.setTarget(self.STEERING_SERVO_1_NO, steering_target)
        self.servo_.setTarget(self.STEERING_SERVO_2_NO, steering_target)

    def setSpeed(self, speed):
        original_speed = speed

        # Clip input speed
        if speed > self.MAX_SPEED:
            speed = self.MAX_SPEED
        elif speed < self.MIN_SPEED:
            speed = self.MIN_SPEED
        elif speed < self.MAX_DEADZONE and speed > self.MIN_DEADZONE:
            speed = 0
        
        # Stretch to match target
        if speed == 0:
            speed_target = self.STOP_SPEED_TARGET;
        elif speed > 0:
            speed_target = speed * (self.MAX_SPEED_TARGET - self.STOP_SPEED_TARGET) / self.MAX_SPEED + self.STOP_SPEED_TARGET
        elif speed < 0:
            speed_target = speed * (self.MIN_SPEED_TARGET - self.STOP_SPEED_TARGET) / self.MIN_SPEED + self.STOP_SPEED_TARGET

        speed_target = int(speed_target)
        
        print('Input speed: ' + str(original_speed) + ', clipped:' + str(speed) + ', target: ' + str(speed_target))
        self.servo_.setTarget(self.SPEED_SERVO_NO, speed_target)

    def setServoConfig(self, servo_no, speed, accel):
        self.servo_.setAccel(servo_no,speed)
        self.servo_.setSpeed(servo_no,accel)
        self.stop()

    def stop(self):
        if self.servo_:
            self.setSpeed(0)
            self.setSteering(0)
            
    def initServoConfig(self):
        self.setServoConfig(self.SPEED_SERVO_NO, self.SPEED_SPEED, self.SPEED_ACCEL)
        self.setServoConfig(self.STEERING_SERVO_1_NO, self.STEERING_SPEED, self.STEERING_ACCEL)
        self.setServoConfig(self.STEERING_SERVO_2_NO, self.STEERING_SPEED, self.STEERING_ACCEL)

    def initServo(self):
        self.servo_ = maestro.Controller('/dev/ttyACM1')
        # Test if this is the correct port
        # If not, following statement hangs indefinitely
        print(self.servo_.getPosition(self.SPEED_SERVO_NO))
        self.initServoConfig()
        

def main(args=None):
    # print(sys.path)
    print('main...')
    rclpy.init(args=args)

    node = RemoteControl()
    try:
        print('main spin...')
        rclpy.spin(node)
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
