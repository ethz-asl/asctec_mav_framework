#!/usr/bin/env python



# Copyright (c) 2011, Stephan Weiss, ASL, ETH Zurich, Switzerland
# You can contact the author at <stephan dot weiss at mavt dot ethz dot ch>
# 
# All rights reserved.
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# * Redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer.
# * Redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in the
# documentation and/or other materials provided with the distribution.
# * Neither the name of ETHZ-ASL nor the
# names of its contributors may be used to endorse or promote products
# derived from this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL ETHZ-ASL BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


NAME='wpclient'

# ROS specific imports
import roslib; roslib.load_manifest('asctec_hl_interface')
import rospy
import actionlib
import re
import time
import sys
import argparse
import math

from asctec_hl_comm.msg import mav_ctrl
from geometry_msgs.msg import PoseStamped

global seq
global have_pose
global start_pose

class PoseXYZYaw:
    x = 0
    y = 0
    z = 0
    yaw = 0
    c_yaw = 1
    s_yaw = 0
    
    def __init__(self, pose):
        self.x = pose.pose.position.x
        self.y = pose.pose.position.y
        self.z = pose.pose.position.z
        q = pose.pose.orientation
        self.yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.c_yaw = math.cos(self.yaw)
        self.s_yaw = math.sin(self.yaw)

def parseWaypoints(wp_file):
    rospy.loginfo("parsing waypoint list...")
    # read file
    try:
        text = open(wp_file,"r").readlines()
    except:
        print 'unable to open file %s'%sys.argv[1]
        return False
    
    waypoints = list();

    linecounter = 0
    for line in text:
        linecounter += 1;
        wordcounter = 0
        goodpoint=True;
        if not (line[0]=="#" or not line.strip()):
            for word in line.split():                
                # check if integer (isdigit) or float
                if not (word.isdigit() or not re.match("^[+-]?\d+?\.\d+?$", word) is None):                
                #if not (word.isdigit() or not re.match("^(0|(-(((0|[1-9]\d*)\.\d+)|([1-9]\d*))))$", word) is None):
                    #print "corrupt waypoint: ", line
                    rospy.logerr("corrupt waypoint: " + line)
                    goodpoint=False
                    break
                wordcounter += 1;
            if wordcounter<7 and goodpoint:
                #print "warning: only ", wordcounter, " arguments provided"
                rospy.logwarn("only " + str(wordcounter) + " arguments provided for waypoint: " + line)
            if goodpoint:
                waypoints.append(line)
    rospy.loginfo("successfully parsed " + str(len(waypoints)) + " waypoints")
    return waypoints

def sendWaypoints(wp, pub, sp):
    global seq
    goal = mav_ctrl()

    goal.header.stamp = rospy.Time().now();
    goal.header.seq = seq;
    seq +=1

    goal.x = 0
    goal.y = 0
    goal.z = 0
    goal.yaw = 0
    goal.v_max_xy = 0
    goal.v_max_z = 0
    goal.type = mav_ctrl.position;
    
    wps = wp.split();
    
    try:
        x = float(wps[0])
        y = float(wps[1])
        z = float(wps[2])
        yaw = float(wps[3])
        goal.v_max_xy = float(wps[4])
        goal.v_max_z = float(wps[5])
    except: 
        pass

    goal.x = sp.c_yaw * x - sp.s_yaw * y + sp.x
    goal.y = sp.s_yaw * x + sp.c_yaw * y + sp.y
    goal.z = z + sp.z
    goal.yaw = yaw + sp.yaw
    
    if(goal.yaw > math.pi):
        goal.yaw = goal.yaw - 2*math.pi
    
    if(goal.yaw < -math.pi):
        goal.yaw = goal.yaw + 2*math.pi
        
        
    
#    rospy.loginfo("sending waypoint: " + "\t" + str(goal.x) + "\t" + str(goal.y) + "\t" + str(goal.z))
    pub.publish(goal)

    try:
        time.sleep(float(wps[6]))   # only remain at the reached position if goal successfully reached
    except:
        pass

def poseCallback(msg):
    global have_pose    
    global start_pose
    
    start_pose = msg
    have_pose = True

if __name__ == '__main__':
    try:

        seq = 0
        
        rospy.init_node('wpclient')
#        start_pose = PoseStamped()
#        start_pose.pose.position.x = 0
#        start_pose.pose.position.y = 0
#        start_pose.pose.position.z = 0
#        start_pose.pose.orientation.w = 1
#        start_pose.pose.orientation.x = 0
#        start_pose.pose.orientation.y = 0
#        start_pose.pose.orientation.z = 0
        
        parser = argparse.ArgumentParser()
        parser.add_argument('-f', '--file', help='trajectory file', required=True)
        parser.add_argument('-m', '--mode', default='abs', help='define if absolute (abs) trajectory coordinates are used or relative (rel) to the current position')
        
        args = parser.parse_args(rospy.myargv()[1:]);
        
        if args.mode == 'abs':
            have_pose = True
            print 'operating in absolute mode'
        elif args.mode == 'rel':
            sub = rospy.Subscriber('fcu/current_pose', PoseStamped, poseCallback)
            print 'operating in relative mode'
            have_pose = False
        else:
            rospy.logerr('Unknown mode')
            exit()
        
        n_wait = 0;
        while not have_pose:
            time.sleep(0.1)
            n_wait = n_wait + 1;
            if n_wait > 20:
                rospy.logerr('didn\'t receive start pose')
                exit()
  
        simple_start_pose = PoseXYZYaw(start_pose)        
        print 'starting from x=%f y=%f z=%f yaw=%f deg\n' % (simple_start_pose.x, simple_start_pose.y, simple_start_pose.z, simple_start_pose.yaw*180/math.pi)    
                
        pub = rospy.Publisher('fcu/control', mav_ctrl)        
         
        wp_list = parseWaypoints(args.file);
        
        if not (wp_list):
            exit()
                    
        rospy.loginfo("waiting for subscriber(s) to connect ")
        while pub.get_num_connections() < 1:
            time.sleep(0.1)
            
        rospy.loginfo("connected, sending waypoints")
        
        for line in wp_list:
            if rospy.is_shutdown():
                break
            sendWaypoints(line, pub, simple_start_pose)
    except rospy.ROSInterruptException:
        print "program interrupted before completion"


