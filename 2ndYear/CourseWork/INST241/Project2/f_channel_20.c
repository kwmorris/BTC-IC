/*************************************************************************

Consult the "README.txt" file for help editing this function!

*************************************************************************/
/

#include <stdio.h>
#include <math.h>		// Necessary for any advanced math functions
#include "cascada.h"		// Contains all the declarations specific to caSCADA

int
f_channel_20 (void)
{
  float I9 = f_channel[9].value;
  int S9 = f_channel[9].status;
  
  if(I9 < 0){
	  I9=0;
  }
  
  float v;
  
  float P[20];
  P[0] = 0;
  P[1] = 0.0166666666666667;
  P[2] = 0.13;
  P[3] = 0.323333333333333;
  P[4] = 0.586666666666667;
  
  if(S9==0){
	int i;
	v = -1;
	for(i = 0; v == -1 && i < 4; i++){
		if(P[i] <= I9 && I9 < P[i+1]){
			v = ((I9-P[i])/(P[i+1]-P[i])+i)*3;
		}		
	}
	f_channel[20].comment = "Low Flow Rate";
  }else{
    float k = 15.407;
	v = k*sqrt(I9);
	f_channel[20].comment = "Good Flow Rate";
  }
  
  f_channel[20].value = v;
  f_channel[20].tag = "FY-12";
  f_channel[20].unit = "FLow %";
  f_channel[20].status = S9;

  return 1;
}
