import os
import sys

rgblist = []
lablist = []
for root, dirs, files in os.walk("./"):
  for dir in files:
      main, ext = os.path.splitext(dir) # split into mainname and extention
      
      color_num_mode = main[:].split("-")
      if color_num_mode[0][-1] != "0":
          if color_num_mode[-1] == "rgb":
              rgblist.append(main)
          elif color_num_mode[-1] == "lab":
              lablist.append(main)

def catfiles(clist,cmode):
	rgblist.sort()
	lablist.sort()

	pastcolor = clist[0].split("_")
	command = "cat " + clist[0] + ".dat "
	for i in range(1,len(clist)):
		color = clist[i].split("_")
		if color[0] == pastcolor[0]:
			command += clist[i] + ".dat "
		else:
			output = pastcolor[0] + "_0-" + cmode + ".dat"
			print command + " > " + output
			os.system(command + " > " + output)
			
			command = "cat " + clist[i] + ".dat "
		pastcolor = color
		
	output = pastcolor[0] + "_0-" + cmode + ".dat"
	print command + " > " + output
	os.system(command + " > " + output)



catfiles(rgblist,"rgb")
catfiles(lablist,"lab")
