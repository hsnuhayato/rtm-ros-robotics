import os
import sys

datadir = "./data/"
imgdir = "./training/"
imglist=["brown_0","green_0","limegreen_0","purple_0","red_0","waterblue_0","yellow_0","table_0"]

def lab():
    proc("lab")

def rgb():
    proc("rgb")


def proc(cmode):
    exe = "./bin/learn"

    clibname = "colorlib.h"
    ifile = open(clibname,"w")
    for img in imglist:
        color = img[:].split("_")[0]
        lfile = datadir + img + "-" + cmode + ".dat"

        command = exe + " " + lfile + " " + cmode + " " + color
        print command
        os.system(command);

        command = "mv -f " + color + "_" + cmode + ".h" + " include/"
        print command
        os.system(command);

        ifile.write("#include \"" + color + "_" + cmode + ".h" + "\"\n")
        
    ifile.close()
    command = "mv -f " + clibname + " include/"
    print command
    os.system(command);

if len(sys.argv) > 1:
    for i in range(0,len(sys.argv)-1):
        if sys.argv[i+1] == "rgb":
            rgb()
        elif sys.argv[i+1] == "lab":
            lab()
else:
    print "usage: py proc.py rgb"
