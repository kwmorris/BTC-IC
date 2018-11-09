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
  int pURV = 12;	//output pressure Upper Range Value
  int pLRV = 0;		//output pressure Lower Range Value
  int pOpen = -2;	//open loop pressure threshold
  
  float p;			//output pressure 
  
  p = (ain[9]-vLRV)/(vURV-vLRV)*(pURV-pLRV)+pLRV;
  
  if(p < pOpen){
	  f_channel[9].comment = "Open loop";
  }else{
	  f_channel[9].comment = "(none)";
  }
  
  f_channel[9].value = p;
  f_channel[9].tag = "FT-12";
  f_channel[9].unit = "inWC";
  f_channel[9].status = 1;

  return 1;
}

