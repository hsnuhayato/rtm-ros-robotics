#ifndef _COPYPARAM_H_
#define _COPYPARAM_H_

#include <colorlib.h>

void getTrainingParam(string targetcolor,float ave[][3],float var[][3][3])
{
#ifdef COLOR_BROWN
	if(targetcolor.c_str() == "brown")
		for(int n=0; n<GaussN; n++)
			for(int i=0; i<3; i++)
				{
					if(targetcolor == "lab")
						{
							ave[n][i] = brown_lab_ave[n][i];
							for(int j=0; j<3; j++)
								var[n][i][j] = brown_lab_var[n][i];
						}
					else if(targetcolor == "rgb")
						{
							ave[n][i] = brown_rgb_ave[n][i];
							for(int j=0; j<3; j++)
								var[n][i][j] = brown_rgb_var[n][i];
						}
				}
#endif

#ifdef COLOR_GREEN
	if(targetcolor.c_str() == "green")
		for(int n=0; n<3; n++)
			for(int i=0; i<3; i++)
				{
					if(targetcolor == "lab")
						{
							ave[n][i] = green_lab_ave[n][i];
							for(int j=0; j<3; j++)
								var[n][i][j] = green_lab_var[n][i];
						}
					else if(targetcolor == "rgb")
						{
							ave[n][i] = green_rgb_ave[n][i];
							for(int j=0; j<3; j++)
								var[n][i][j] = green_rgb_var[n][i];
						}
				}
#endif

#ifdef COLOR_LIMEGREEN
	if(targetcolor.c_str() == "limegreen")
		for(int n=0; n<3; n++)
			for(int i=0; i<3; i++)
				{
					if(targetcolor == "lab")
						{
							ave[n][i] = limegreen_lab_ave[n][i];
							for(int j=0; j<3; j++)
								var[n][i][j] = limegreen_lab_var[n][i];
						}
					else if(targetcolor == "rgb")
						{
							ave[n][i] = limegreen_rgb_ave[n][i];
							for(int j=0; j<3; j++)
								var[n][i][j] = limegreen_rgb_var[n][i];
						}
				}
#endif

#ifdef COLOR_PURPLE
	if(targetcolor.c_str() == "purple")
		for(int n=0; n<3; n++)
			for(int i=0; i<3; i++)
				{
					if(targetcolor == "lab")
						{
							ave[n][i] = purple_lab_ave[n][i];
							for(int j=0; j<3; j++)
								var[n][i][j] = purple_lab_var[n][i];
						}
					else if(targetcolor == "rgb")
						{
							ave[n][i] = purple_rgb_ave[n][i];
							for(int j=0; j<3; j++)
								var[n][i][j] = purple_rgb_var[n][i];
						}
				}
#endif

#ifdef COLOR_RED
	if(targetcolor.c_str() == "red")
		for(int n=0; n<3; n++)
			for(int i=0; i<3; i++)
				{
					if(targetcolor == "lab")
						{
							ave[n][i] = red_lab_ave[n][i];
							for(int j=0; j<3; j++)
								var[n][i][j] = red_lab_var[n][i];
						}
					else if(targetcolor == "rgb")
						{
							ave[n][i] = red_rgb_ave[n][i];
							for(int j=0; j<3; j++)
								var[n][i][j] = red_rgb_var[n][i];
						}
				}
#endif

#ifdef COLOR_WATERBLUE
	if(targetcolor.c_str() == "waterblue")
		for(int n=0; n<3; n++)
			for(int i=0; i<3; i++)
				{
					if(targetcolor == "lab")
						{
							ave[n][i] = waterblue_lab_ave[n][i];
							for(int j=0; j<3; j++)
								var[n][i][j] = waterblue_lab_var[n][i];
						}
					else if(targetcolor == "rgb")
						{
							ave[n][i] = waterblue_rgb_ave[n][i];
							for(int j=0; j<3; j++)
								var[n][i][j] = waterblue_rgb_var[n][i];
						}
				}
#endif

#ifdef COLOR_YELLOW
	if(targetcolor.c_str() ==  "yellow")
		for(int n=0; n<3; n++)
			for(int i=0; i<3; i++)
				{
					if(targetcolor == "lab")
						{
							ave[n][i] = yellow_lab_ave[n][i];
							for(int j=0; j<3; j++)
								var[n][i][j] = yellow_lab_var[n][i];
						}
					else if(targetcolor == "rgb")
						{
							ave[n][i] = yellow_rgb_ave[n][i];
							for(int j=0; j<3; j++)
								var[n][i][j] = yellow_rgb_var[n][i];
						}
				}
#endif

#ifdef COLOR_TABLE
	if(targetcolor.c_str() ==  "table")
		for(int n=0; n<3; n++)
			for(int i=0; i<3; i++)
				{
					if(targetcolor == "lab")
						{
							ave[n][i] = table_lab_ave[n][i];
							for(int j=0; j<3; j++)
								var[n][i][j] = table_lab_var[n][i];
						}
					else if(targetcolor == "rgb")
						{
							ave[n][i] = table_rgb_ave[n][i];
							for(int j=0; j<3; j++)
								var[n][i][j] = table_rgb_var[n][i];
						}
				}
#endif
}

#endif
