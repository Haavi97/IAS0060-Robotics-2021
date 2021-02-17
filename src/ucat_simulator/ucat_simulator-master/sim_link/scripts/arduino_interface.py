#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Imu
from uwsim_physics_msgs.msg import Pressure
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Bool
from tut_arrows_msgs.msg import Flipper
import serial
import struct
import math
import sys

messagelength = 49

orientation = Quaternion()
leftBeacon = False
rightBeacon = False
depth = 30 #cm

def extractMessage(buf):
    ros_messages = []
    messagestart = buf.find('SYNCSYNC')
    if len(buf) > 1024: # limit buffer length
        return buf[-1024:], ros_messages
    if messagestart == -1: #no sync
        return buf, ros_messages
#     print buf[:messagestart]
    if len(buf) - messagestart < messagelength: #only beginning found
        return buf[messagestart:], ros_messages
    message = buf[messagestart + len('SYNCSYNC') : messagestart + messagelength - 1]
    for motor in range(4):
        flipper = Flipper()
        flipper.motorNumber = motor
        flipper.frequency = frequency = struct.unpack('f',message[motor*10:motor*10+4])[0]
        flipper.zeroDirection = zero = int(message[motor*10+4+2-1:motor*10+4-1:-1].encode('hex'),16)/1024.0*2*math.pi
        flipper.amplitude = amplitude = int(message[motor*10+6+2-1:motor*10+6-1:-1].encode('hex'),16)/1024.0*2*math.pi
        flipper.phaseOffset = phase = int(message[motor*10+8+2-1:motor*10+8-1:-1].encode('hex'),16)/1024.0*2*math.pi
        ros_messages.append(flipper)
        #print "Motor", motor, ":", frequency, zero, amplitude, phase
    return buf[messagestart + messagelength : ], ros_messages

def imu_callback(data):
    global orientation
    orientation = data.orientation
#     print "GOT IT!!!!!!!!!!!!!!!!!!!!!!!!!", data.orientation.x, data.orientation.y, data.orientation.y, data.orientation.w  

def pressure_callback(data):
    global depth
    depth = data.pressure*100

def leftsensor_callback(data):
    global leftBeacon
    leftBeacon = data

def rightsensor_callback(data):
    global rightBeacon
    rightBeacon = data

def arduino():
    global depth
    if len(sys.argv) < 2:
        print "Wrong number of arguments to arduino_interface"
        print sys.argv
        return
    pub = rospy.Publisher('motors', Flipper)
    rospy.init_node('arduino')
    rospy.Subscriber("imu", Imu, imu_callback)
    rospy.Subscriber("pressure", Pressure, pressure_callback)
    rospy.Subscriber("leftsensor", Bool, leftsensor_callback)
    rospy.Subscriber("rightsensor", Bool, rightsensor_callback)
    port = sys.argv[1]
    ser = serial.Serial(port, 115200, timeout=0)
    buffer = r''
    while not rospy.is_shutdown():
        data = ser.read(9999)
        messages = []
        if len(data) > 0:
#             print data.encode('hex')
            buffer += data
            while True:
                old_length = len(buffer) 
                buffer, messages = extractMessage(buffer)
                if len(messages) == 4:
                    for msg in messages:
                        pub.publish(msg)
                if len(buffer) == old_length:
                    break
        outbuffer = r'syncsync'
        outbuffer += struct.pack('f', orientation.w)
        outbuffer += struct.pack('f', orientation.x)
        outbuffer += struct.pack('f', -orientation.y)
        outbuffer += struct.pack('f', -orientation.z)
        if depth < 0:
            depth = 0
        if depth > 255:
            depth = 255
        outbuffer += struct.pack('B', int(depth))
#         print orientation.x, orientation.y, orientation.z, orientation.w
        flags = 0
        if leftBeacon:
            flags += 1
        if rightBeacon:
            flags += 2
#         flags = 1*int(leftBeacon) + 2*int(rightBeacon)
        outbuffer += struct.pack('B', flags)
        outbuffer += '\n'
        ser.write(outbuffer)
#        print outbuffer.encode('hex')
#        str = "hello world %s" % rospy.get_time()
#        rospy.loginfo(str)
#        pub.publish(String(str))
        rospy.sleep(0.01)


if __name__ == '__main__':
    try:
        arduino()
    except rospy.ROSInterruptException:
        pass
