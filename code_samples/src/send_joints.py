#!/usr/bin/env python

import rospy
import geometry_msgs.msg
import sensor_msgs.msg
import spidev
import time


class SendJointsDriver:
    # Send joint to driver
    # Adding supported protocols
    SupportedProtocols = ['spi']

    def __init__(self, protocol):
        self.protocol = protocol
        self.check_protocol()
        joint_goal_sub = rospy.Subscriber('goal_joints',sensor_msgs.msg.JointState , self.driver_callback)

    def driver_callback(self, joint_goal):
        self.spi_send(joint_goal) 

    def check_protocol(self):
        # Check which protocol is chosen
        # and call it's init function
        if self.protocol is 'spi':
            self.spi_init()
        else:
            raise "Protocol not supported"

    def spi_init(self):
        SPI_SPEED = 100000 # Hz

        bus = 0
        # SS0
        device = 0
        #Enable SPI
        self.spi_handle = spidev.SpiDev()
        self.spi_handle.open(bus, device)
        #Set speed and mode
        self.spi_handle.max_speed_hz = SPI_SPEED
        self.spi_handle.mode=0

    def spi_send(self, joint_goal):
        # Signaling that the last joint is send
        # this value will be used in the driver as well
        END_TRANS = 100 

        joints = list(joint_goal.position)
        for i in range(len(joints)):
            joints[i] = int(joints[i])
        print joints

        self.spi_handle.xfer2([ joints[0],joints[1] ])
        self.spi_handle.xfer2([ joints[2],joints[3] ])
        self.spi_handle.xfer2([ joints[4],joints[5] ])
        self.spi_handle.xfer2([END_TRANS])

def main():
    rospy.init_node('send_joints')
    SPIsend = SendJointsDriver('spi')
    rospy.spin()

if __name__ == '__main__':
    main()
