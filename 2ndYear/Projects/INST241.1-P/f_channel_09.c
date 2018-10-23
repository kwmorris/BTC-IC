/*************************************************************************

Consult the "README.txt" file for help editing this function!

*************************************************************************/


#include <stdio.h>
#include <math.h>		// Necessary for any advanced math functions
#include "cascada.h"		// Contains all the declarations specific to caSCADA

int
f_channel_09 (void)
{
  var vURV = 5; 	//input voltage Upper Range Value
  var vLRV = 1;		//input voltage Lower Range Value
  var tURV = 212;	//output temperature Upper Range Value
  var tLRV = 32;	//output temperature Lower Range Value
  
  f_channel[9].value = (ain[9]-vLRV)/(vURV-vLRV)*(tURV-tLRV)+tLRV;
  f_channel[9].tag = "TT-22";
  f_channel[9].unit = "degF";
  f_channel[9].status = 1;
  f_channel[9].comment = "Analog input AIN9";

  return 1;
}

