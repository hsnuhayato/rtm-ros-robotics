import os
import sys

imgdir = "./training/"

def lab(imgfile):
    proc(imgfile,"lab","./data/labcolor.learn")

def rgb(imgfile):
    proc(imgfile,"rgb","./data/rgbcolor.learn")

def proc(imgfile,cmode,linput):
    for line in  open(linput,"r"):
        items = line[:].split(" ")

        img = items[0][:].split("/")[-1]
        color = img[:].split("_")[0]
        
        param = " "
        for i in range(0,6):
            param += items[i+1] + " "

        command = "./bin/color " + imgfile + " " + cmode + param
        print command
        os.system(command)
    

if len(sys.argv) > 2:
    if sys.argv[2] == "rgb":
        rgb(sys.argv[1])
    elif sys.argv[2] == "lab":
        lab(sys.argv[1])
else:
    print "usage: py proc.py imgfile colormode"

