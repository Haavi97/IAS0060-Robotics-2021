#!/usr/bin/env python

# Copyright (c) 2011, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#    * Neither the name of the Willow Garage, Inc. nor the names of its
#      contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import roslib
#roslib.load_manifest('turtlebot_teleop')
roslib.load_manifest('tut_arrows')
import rospy

from geometry_msgs.msg import WrenchStamped
from geometry_msgs.msg import Wrench

import sys, select, termios, tty

msg = """
Control Your Robot with WrenchStamped!
---------------------------
Moving along x and y:
   u    i    o     
   j    k    l
   m    ,    .
   
Rotating around z:
   t    y

Moving along z:
   h
   n
   
q/z : increase/decrease max forces by 10%
w/x : increase/decrease only linear force by 10%
e/c : increase/decrease only angular force by 10%
space key, k : force stop

CTRL-C to quit
"""

moveBindings = {
        'i':(1,0,0,0,0,0),
        'j':(0,1,0,0,0,0),
        'u':(1,1,0,0,0,0),
        'l':(0,-1,0,0,0,0),
        'o':(1,-1,0,0,0,0),
        ',':(-1,0,0,0,0,0),
        'm':(-1,1,0,0,0,0),
        '.':(-1,-1,0,0,0,0),
        
        
        'h':(0,0,1,0,0,0),
        'n':(0,0,-1,0,0,0),
        
        't':(0,0,0,0,0,1),
        'y':(0,0,0,0,0,-1),
        
        'k':(0,0,0,0,0,0),
        ' ':(0,0,0,0,0,0),
           }

speedBindings={
        'q':(1.1,1.1),
        'z':(.9,.9),
        'w':(1.1,1),
        'x':(.9,1),
        'e':(1,1.1),
        'c':(1,.9),
          }

def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

speed = 4
turn = .5

def vels(speed,turn):
    return "currently:\tspeed %s\tturn %s " % (speed,turn)

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)
    
    rospy.init_node('key_teleop_wrench')
    pub = rospy.Publisher('force_req', WrenchStamped)
    frame_id = rospy.get_param('~frame_id', '')


    try:
        print msg
        print vels(speed,turn)
        while not rospy.is_shutdown():
            lastMove = None
            key = getKey()
            if key in moveBindings.keys():
                lastMove = moveBindings[key]
            elif key in speedBindings.keys():
                speed = speed * speedBindings[key][0]
                turn = turn * speedBindings[key][1]
                #print vels(speed,turn)
                #if (status == 14):
                #    print msg
                #status = (status + 1) % 15
            else:
                if (key == '\x03'):
                    break

            if lastMove:
                wrench = Wrench()
                wrench.force.x = lastMove[0]*speed
                wrench.force.y = lastMove[1]*speed
                wrench.force.z = lastMove[2]*speed
                wrench.torque.x = lastMove[3]*turn
                wrench.torque.y = lastMove[4]*turn
                wrench.torque.z = lastMove[5]*turn
                ws = WrenchStamped()
                ws.header.stamp = rospy.Time.now()
                ws.header.frame_id = frame_id
                ws.wrench = wrench
                pub.publish(ws)
                
    except Exception as e:
        print e
    finally:
        ws = WrenchStamped()
        ws.header.stamp = rospy.Time.now()
        ws.header.frame_id = frame_id
        pub.publish(ws)

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
