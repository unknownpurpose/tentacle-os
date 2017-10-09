#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Quaternion
from Adafruit_BNO055 import BNO055
from threading import Thread

class IMUPublisher:
    def __init__(self, bno, node_name, refresh_rate=10):
        self.bno = bno
        self.node_name = node_name
        self.refresh_rate = refresh_rate

    def __initialize_bno(self):
        # Initialize the BNO055 and stop if something went wrong.
        bno = self.bno

        # Hacked out of the Adafruit simpletest example innittt
        if not bno.begin():
            raise RuntimeError('Failed to initialize BNO055! Is the sensor connected?')

        # Print system status and self test result.
        status, self_test, error = bno.get_system_status()
        print('System status: {0}'.format(status))
        print('Self test result (0x0F is normal): 0x{0:02X}'.format(self_test))
        # Print out an error if system status is in error mode.
        if status == 0x01:
            print('System error: {0}'.format(error))
            print('See datasheet section 4.3.59 for the meaning.')

    def start(self):
        self.__initialize_bno()

        rospy.init_node('imu_publisher', anonymous=True)
        Thread(target=self.emit).start()

    def emit(self):
        pub = rospy.Publisher('imu/position', Quaternion, queue_size=10)
        rate = rospy.Rate(self.refresh_rate)
        while not rospy.is_shutdown():
            x,y,z,w = bno.read_quaternion()
            q = Quaternion(x=x,y=y,z=z,w=w)
            rospy.loginfo(q)
            pub.publish(q)
            rate.sleep()

if __name__ == '__main__':
    try:
        # Raspberry Pi configuration with serial UART and RST connected to GPIO 18:
        #bno = BNO055.BNO055(serial_port='/dev/ttyAMA0', rst=18)
        bno = BNO055.BNO055(busnum=2) # << or... Beaglebone on standard i2c
        imu = IMUPublisher(bno=bno, node_name='bno055')
        imu.start()

    except rospy.ROSInterruptException: pass
