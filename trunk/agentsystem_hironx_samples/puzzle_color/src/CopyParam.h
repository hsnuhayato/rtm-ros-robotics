#ifndef _COPYPARAM_H_
#define _COPYPARAM_H_

#include <colorlib-rgb.h>
#include <colorlib-lab.h>

void getTrainingParam(int gaussn, string targetcolor,string colormode,
											float ave[][3],float var[][3][3])
{
#ifdef COLOR_BROWN
	if(targetcolor == "brown")
		for(int n=0; n<gaussn; n++)
			for(int i=0; i<3; i++)
				{
					if(colormode == "lab")
						{
							ave[n][i] = brown_lab_ave[n][i];
							for(int j=0; j<3; j++)
								var[n][i][j] = brown_lab_var[n][i][j];
						}
					else if(colormode == "rgb")
						{
							ave[n][i] = brown_rgb_ave[n][i];
							for(int j=0; j<3; j++)
								var[n][i][j] = brown_rgb_var[n][i][j];
						}
				}
#endif

#ifdef COLOR_GREEN
	if(targetcolor == "green")
		for(int n=0; n<gaussn; n++)
			for(int i=0; i<3; i++)
				{
					if(colormode == "lab")
						{
							ave[n][i] = green_lab_ave[n][i];
							for(int j=0; j<3; j++)
								var[n][i][j] = green_lab_var[n][i][j];
						}
					else if(colormode == "rgb")
						{
							ave[n][i] = green_rgb_ave[n][i];
							for(int j=0; j<3; j++)
								var[n][i][j] = green_rgb_var[n][i][j];
						}
				}
#endif

#ifdef COLOR_LIMEGREEN
	if(targetcolor == "limegreen")
		for(int n=0; n<gaussn; n++)
			for(int i=0; i<3; i++)
				{
					if(colormode == "lab")
						{
							ave[n][i] = limegreen_lab_ave[n][i];
							for(int j=0; j<3; j++)
								var[n][i][j] = limegreen_lab_var[n][i][j];
						}
					else if(colormode == "rgb")
						{
							ave[n][i] = limegreen_rgb_ave[n][i];
							for(int j=0; j<3; j++)
								var[n][i][j] = limegreen_rgb_var[n][i][j];
						}
				}
#endif

#ifdef COLOR_PURPLE
	if(targetcolor == "purple")
		for(int n=0; n<gaussn; n++)
			for(int i=0; i<3; i++)
				{
					if(colormode == "lab")
						{
							ave[n][i] = purple_lab_ave[n][i];
							for(int j=0; j<3; j++)
								var[n][i][j] = purple_lab_var[n][i][j];
						}
					else if(colormode == "rgb")
						{
							ave[n][i] = purple_rgb_ave[n][i];
							for(int j=0; j<3; j++)
								var[n][i][j] = purple_rgb_var[n][i][j];
						}
				}
#endif

#ifdef COLOR_RED
	if(targetcolor == "red")
		for(int n=0; n<gaussn; n++)
			for(int i=0; i<3; i++)
				{
					if(colormode == "lab")
						{
							ave[n][i] = red_lab_ave[n][i];
							for(int j=0; j<3; j++)
								var[n][i][j] = red_lab_var[n][i][j];
						}
					else if(colormode == "rgb")
						{
							ave[n][i] = red_rgb_ave[n][i];
							for(int j=0; j<3; j++)
								var[n][i][j] = red_rgb_var[n][i][j];
						}
				}
#endif

#ifdef COLOR_WATERBLUE
	if(targetcolor == "waterblue")
		for(int n=0; n<gaussn; n++)
			for(int i=0; i<3; i++)
				{
					if(colormode == "lab")
						{
							ave[n][i] = waterblue_lab_ave[n][i];
							for(int j=0; j<3; j++)
								var[n][i][j] = waterblue_lab_var[n][i][j];
						}
					else if(colormode == "rgb")
						{
							ave[n][i] = waterblue_rgb_ave[n][i];
							for(int j=0; j<3; j++)
								var[n][i][j] = waterblue_rgb_var[n][i][j];
						}
				}
#endif

#ifdef COLOR_YELLOW
	if(targetcolor ==  "yellow")
		for(int n=0; n<gaussn; n++)
			for(int i=0; i<3; i++)
				{
					if(colormode == "lab")
						{
							ave[n][i] = yellow_lab_ave[n][i];
							for(int j=0; j<3; j++)
								var[n][i][j] = yellow_lab_var[n][i][j];
						}
					else if(colormode == "rgb")
						{
							ave[n][i] = yellow_rgb_ave[n][i];
							for(int j=0; j<3; j++)
								var[n][i][j] = yellow_rgb_var[n][i][j];
						}
				}
#endif

#ifdef COLOR_TABLE
	if(targetcolor ==  "table")
		for(int n=0; n<gaussn; n++)
			for(int i=0; i<3; i++)
				{
					if(colormode == "lab")
						{
							ave[n][i] = table_lab_ave[n][i];
							for(int j=0; j<3; j++)
								var[n][i][j] = table_lab_var[n][i][j];
						}
					else if(colormode == "rgb")
						{
							ave[n][i] = table_rgb_ave[n][i];
							for(int j=0; j<3; j++)
								var[n][i][j] = table_rgb_var[n][i][j];
						}
				}
#endif
}

#endif
