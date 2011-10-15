# execution example: py proc-gmm.py puzzleimages/frame0010.jpg data/brown_0-lab.dat

import os
import sys

imgdir = "./training/"

def proc(imgfile,learndata,cmode):
    command = "./bin/gmm " + imgfile + " " + learndata + " " + cmode
    print command
    os.system(command)
    

if len(sys.argv) > 3:
    proc(sys.argv[1], sys.argv[2], sys.argv[3])
else:
    print "usage: py proc.py imgfile targetcolor cmode"

