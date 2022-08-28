#!/usr/bin/env python

import rospy
import RPi.GPIO as GPIO

def test():
    # init ros node
    rospy.init_node('ros_test_node', anonymous=True)

    # loop rate
    rate = rospy.Rate(2)

    # initialize publisher
    # pub = rospy.Publisher('test', String, queue_size=10)

    # initialize gpio
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(11, GPIO.OUT)

    # main loop
    while not rospy.is_shutdown():
        # publlish test message
        # test message
        # msg = 'test: %s' % rospy.get_time()
        # pub.publish(msg)

        # blink led
        # turn on led
        GPIO.output(11, True)
        rate.sleep()

        # turn off led
        GPIO.output(11, False)
        rate.sleep()
    
    GPIO.cleanup()

if __name__ == '__main__':
    try:
        test()
    except rospy.ROSInterruptException:
        GPIO.cleanup()
