/*************************************************************************

Consult the "README.txt" file for help editing this function!

*************************************************************************/


#include <stdio.h>
#include <math.h>		// Necessary for any advanced math functions
#include "cascada.h"		// Contains all the declarations specific to caSCADA

int
f_channel_09 (void)
{
  int vURV = 5; 	//input voltage Upper Range Value
  int vLRV = 1;		//input voltage Lower Range Value
  int pURV = 30;	//output temperature Upper Range Value
  int pLRV = 0;		//output temperature Lower Range Value
  
  f_channel[9].value = (ain[9]-vLRV)/(vURV-vLRV)*(pURV-pLRV)+pLRV;
  f_channel[9].tag = "FT-12";
  f_channel[9].unit = "inWC";
  f_channel[9].status = 1;
  f_channel[9].comment = "Analog input AIN9";

  return 1;
}

