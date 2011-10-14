import os
import sys

imgdir = "./training/"
imglist=["brown_1","green_1","limegreen_1","purple_1","red_1","waterblue_1","yellow_1","table_1",
				 "brown_2","green_2","limegreen_2","purple_2","red_2","waterblue_2","yellow_2","table_2"]

datadir = "data"
if not os.path.exists(datadir):
  os.system("mkdir " + datadir)

def lab():
    proc("lab","labcolor.learn","labcolor.gp")

def rgb():
    proc("rgb","rgbcolor.learn","rgbcolor.gp")


def proc(cmode,loutput,goutput):
    os.system("rm -f " + loutput + " " + goutput)

    gfile = open(goutput,"w")
    gfile.write("splot ")

    for img in imglist:
        imgpath = imgdir + img + ".ppm"
        if not os.path.exists(imgpath):
            print "no such file " + imgpath
            exit

        gfile.write("\"" + img + "-" + cmode + ".dat\" every 10")
        if img != imglist[-1]:
            gfile.write(", ")

        os.system("echo -n " + imgpath + "' ' >> " + loutput)
    
        command = ("./bin/" + cmode + " " + imgpath + " " + img + "-" + cmode + ".dat" +
									 " >> " + loutput)
        print command
        os.system(command);
        os.system("mv " + img + "-" + cmode + ".dat " + datadir)


    gfile.close()

    os.system("mv " + loutput + " " + datadir)
    os.system("mv " + goutput + " " + datadir)

if len(sys.argv) > 1:
    for i in range(0,len(sys.argv)-1):
        if sys.argv[i+1] == "rgb":
            rgb()
        elif sys.argv[i+1] == "lab":
            lab()
else:
    print "usage: py proc.py rgb"
