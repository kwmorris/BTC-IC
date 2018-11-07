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
  float k [20];
  k[0] = 23.2379000772445;
  k[1] = 16.6410058867569;
  k[2] = 15.8276803075348;
  k[3] = 15.6669890360128;
  k[4] = 15.526475085203;
  k[5] = 15.3226175536575;
  k[6] = 15.43950706397;
  k[7] = 15.3434788533843;
  k[8] = 15.3432207867823;
  k[9] = 15.3359589732974;
  k[10] = 15.3529894715748;
  k[11] = 15.3690920842622;
  k[12] = 15.3641771997174;
  k[13] = 15.360143779978;
  k[14] = 15.4077065414528;
  k[15] = 15.4092237721007;
  k[16] = 15.4215379971289;
  k[17] = 15.3992673728138;
  k[18] = 15.4054054054054;
  k[19] = 15.4473719175919;
  
  float P [21];
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
  
  float I9 = f_channel[9].value;
  int i;
  float K = -1;
//  float m;
//  float b;
  for(i = 0; i < 20; i++){
	if(P[i] <= I9 && I9 < P[i+1]){
		K = k[i];
		//m = (P[i+1]-P[i])/3;
		//b = i * 3;
		break;
	}
  }
  
  if(I9 < 0){
	  I9 = 0;
  }
  
//f_channel[20].value = (m*I9+b)/60;
  f_channel[20].value = K*sqrt(I9)/60;
  f_channel[20].tag = "FY-12";
  f_channel[20].unit = "FLow %";
  f_channel[20].status = 1;
  f_channel[20].comment = "(none)";

  return 1;
}
