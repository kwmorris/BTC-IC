/*************************************************************************

Consult the "README.txt" file for help editing this function!

*************************************************************************/
/

#include <stdio.h>
#include <math.h>		// Necessary for any advanced math functions
#include "cascada.h"		// Contains all the declarations specific to caSCADA

int
f_channel_09 (void)
{
  int vURV = 5; 	//input voltage Upper Range Value
  int vLRV = 1;		//input voltage Lower Range Value
  int pURV = 30;	//output pressure Upper Range Value
  int pLRV = 0;		//output pressure Lower Range Value
  float pLGV = 0.5;	//output pressure Lowest Good Value
  float p;
  int status;
  
  p = (ain[9]-vLRV)/(vURV-vLRV)*(pURV-pLRV)+pLRV;
  
  if(p < pLGV){
	  status = 0;
	  f_channel[9].comment = "Low Flow Rate";
  }else{
	  status = 1;
	  f_channel[9].comment = "Good Flow Rate";
  }
  
  f_channel[9].value = p;
  f_channel[9].tag = "FT-12";
  f_channel[9].unit = "inWC";
  f_channel[9].status = status;

  return 1;
}

