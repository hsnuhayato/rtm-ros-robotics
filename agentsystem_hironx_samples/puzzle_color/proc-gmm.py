# execution example: py proc-gmm.py puzzleimages/frame0010.jpg data/brown_0-lab.dat

import os
import sys

imgdir = "./training/"

def proc(imgfile,learndata):
    cmode = learndata[:].split("/")[-1].split("-")[-1].split(".")[0]
    
    command = "./bin/gmm " + imgfile + " " + learndata + " " + cmode
    print command
    os.system(command)
    

if len(sys.argv) > 2:
    proc(sys.argv[1], sys.argv[2])
else:
    print "usage: py proc.py imgfile learndata"

