#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

## Simple demo of a rospy service client that calls a service to add
## two integers. 

PKG = 'rospy_tutorials' # this package name

import roslib; roslib.load_manifest(PKG) 

import sys
import os

import rospy

# imports the AddTwoInts service 
from rospy_tutorials.srv import *

## add two numbers using the add_two_ints service
## @param x int: first number to add
## @param y int: second number to add
def add_two_ints_client(x, y):

    # NOTE: you don't have to call rospy.init_node() to make calls against
    # a service. This is because service clients do not have to be
    # nodes.

    # block until the add_two_ints service is available
    # you can optionally specify a timeout
    rospy.wait_for_service('add_two_ints')
    
    try:
        # create a handle to the add_two_ints service
        add_two_ints = rospy.ServiceProxy('add_two_ints', AddTwoInts)
        
        print "Requesting %s+%s"%(x, y)
        
        # simplified style
        resp1 = add_two_ints(x, y)

        # formal style
        resp2 = add_two_ints.call(AddTwoIntsRequest(x, y))

        if not resp1.sum == (x + y):
            raise Exception("test failure, returned sum was %s"%resp1.sum)
        if not resp2.sum == (x + y):
            raise Exception("test failure, returned sum was %s"%resp2.sum)
        return resp1.sum
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def usage():
    return "%s [x y]"%sys.argv[0]

if __name__ == "__main__":
    
    argv = rospy.myargv()
    if len(argv) == 3:
        try:
            x = int(argv[1])
            y = int(argv[2])
        except:
            print usage()
            sys.exit(1)
    else:
        print usage()
        sys.exit(1)
    print "%s + %s = %s"%(x, y, add_two_ints_client(x, y))
