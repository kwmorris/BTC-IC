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
  float I9 = f_channel[9].value;	//Stores the pressure value from channel 9

  float pPWH = 0.5;		//Input pressure Piece-wise Hand-off threshold (0 disables, -1 forces)

  float hStep = 3;		//Stores the hertz step size for the experimental pressure values

  //Declares and populates an array storing the experimentally determined pressures at known flow rates
  float P[] = {0, 0.0167, 0.13, 0.3233, 0.5867, 0.9333, 1.38, 1.85, 2.4467, 3.0967, 3.8267, 4.62, 5.4867, 6.4433, 7.4767, 8.53, 9.7033, 10.9367, 12.2967, 13.69, 15.0867};
  int pLength = 20;	//Stores the number of non-zero values in the experimental pressure array

  float v;			//Declaration for the output value

  if(I9 < 0){		//Sets the input value and output value to zero, and clears the channel comment
	I9=0;
	v=0;
	f_channel[20].comment = "(none)";

  }else if(I9 < pPWH || pPWH == -1){	//Interpolates the output value from the piece-wise array
	int i;
	v = -1;

	for(i = 0; v == -1 && i < pLength; i++){		//Steps through the piece-wise array to find the correct points for interpolation
		if(P[i] <= I9 && I9 < P[i+1]){
			v = ((I9 - P[i]) / (P[i+1] - P[i]) + i) * hStep;
		}
	}

	f_channel[20].comment = f_channel[9].comment;

  }else{				//Calculates the output value using a curve calculated from experimental data
    float k = 15.407;	//k-value calculated from experimental data
	v = k * sqrt(I9);
	f_channel[20].comment = "Good Flow Rate";

  }

  //Assigns values to the channel outputs
  f_channel[20].value = v;
  f_channel[20].tag = "FI-12";
  f_channel[20].unit = "Hertz";
  f_channel[20].status = 1;

  return 1;
}
