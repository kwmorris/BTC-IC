/*************************************************************************

Consult the "README.txt" file for help editing this function!

*************************************************************************/
/

#include <stdio.h>
#include <math.h>		// Necessary for any advanced math functions
#include "cascada.h"	// Contains all the declarations specific to caSCADA

int
f_channel_20 (void)
{
  float I9 = f_channel[9].value;
  
  float pPWH = 0.5;		//input pressure Piece-wise Hand-off (0 disables, -1 forces)
  
  float v;
  
  float P[20];			//Piece-wise pressure array
  P[0] = 0;
  P[1] = 0.0166666666666667;
  P[2] = 0.13;
  P[3] = 0.323333333333333;
  P[4] = 0.586666666666667;
  P[5] = 0.933333333333333;
  P[6] = 1.38;
  P[7] = 1.85;
  P[8] = 2.44666666666667;
  P[9] = 3.09666666666667;
  P[10] = 3.82666666666667;
  P[11] = 4.62;
  P[12] = 5.48666666666667;
  P[13] = 6.44333333333333;
  P[14] = 7.47666666666667;
  P[15] = 8.53;
  P[16] = 9.70333333333333;
  P[17] = 10.9366666666667;
  P[18] = 12.2966666666667;
  P[19] = 13.69;
  P[20] = 15.0866666666667;
  
  if(I9 < 0){
	I9=0;
	f_channel[20].comment = "(none)";
  }else if(I9 < pPWH || pPWH == -1){
	int i;
	v = -1;
	for(i = 0; v == -1 && i < 20; i++){
		if(P[i] <= I9 && I9 < P[i+1]){
			v = ((I9-P[i])/(P[i+1]-P[i])+i)*3;
		}
	}
	f_channel[20].comment = f_channel[9].comment;
  }else{
    float k = 15.407;
	v = k*sqrt(I9);
	f_channel[20].comment = "Good Flow Rate";
  }
  
  f_channel[20].value = v;
  f_channel[20].tag = "FI-12";
  f_channel[20].unit = "Hertz";
  f_channel[20].status = 1;

  return 1;
}
