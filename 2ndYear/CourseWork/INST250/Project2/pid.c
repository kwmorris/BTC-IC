/*************************************************************************
caSCADA -- PID control system program based on libmodbus 3.0.6
By Tony R. Kuphaldt
Copyright (C) 2016-2018
Last update March 13, 2018

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.

This program is a PID loop controller, using Modbus/TCP to read the 
process variable value from a 1-5 VDC input and output a 1-5 VDC signal
as the manipulated variable.  An "ncurses" interface provides the user 
the means to run and tune the controller.

*************************************************************************/

#include <stdio.h>
#include <ncurses.h>		// Necessary for the "ncurses" library functions for the operator display
#include <time.h>		// Necessary for the "time" library functions to work (nanosleep)
#include "controller.h"		// Contains all the declarations specific to caSCADA's PID controller code
#include "pid.h"		// Contains all the declarations specific to caSCADA's PID control functionality
#include "modbus_open_close.h"	// Contains Modbus network parameters



// Declaring a time value structure for the nanosleep() function.
// The "timespec" structure is already defined in time.h, and 
// we're invoking an instance of it called "loopdelay" for our 
// purposes:
struct timespec loopdelay;


int
main (void)
{

  // Initialize all PID controller variables
  set_defaults ();


  // Set loop execution delay time for the nanosleep() delay function
  loopdelay.tv_sec = 0;		// whole seconds
  loopdelay.tv_nsec = 100000000;	// nanoseconds (100,000,000 nanoseconds = 0.1 seconds)
  					// nanoseconds (500,000,000 nanoseconds = 0.5 second)


  // This function prints an introductory message to the
  // user's screen, prior to establishing a network
  // connection or executing the PID algorithm.  It returns
  // a value for "looprun" which (if not equal to 1) causes
  // the program to cleanly exit.
  looprun = splash_screen ();

  ///////////////////////////////////////////////////////
  //
  // These instructions set things up in preparation for
  // looping the PID algorithm.  We need to establish a
  // Modbus connection to the data acquisition unit, set
  // default control parameter values, and then open up
  // the "ncurses" display mode so we have a usable
  // operator interface.
  //
  ///////////////////////////////////////////////////////


  if (looprun == 1)
    {

      // Opens up Modbus network connection
      open_modbus_connection ();


      // Opens up ncurses mode and initializes all the important variables
      open_ncurses ();

    }

  // If looprun does not equal 1, exit the main routine NOW!
  else
    return 0;


  ///////////////////////////////////////////////////////
  //
  // This is the "main loop" of the program that gets
  // executed over and over again so long as "looprun"
  // is equal to a value of 1.  Otherwise, the program
  // cleanly exits.
  //
  ///////////////////////////////////////////////////////

  while (looprun)
    {

      // Increments the scan counter, counting the number of times this
      // loop has been executed
      ++scan_count;


      // Get the current UNIX system time
      time_current = (int) (time (NULL));


      // Read analog input AIN0 on the LabJack model T7 DAQ, 
      // using "CDAB" byte swapping (specified by the "2" argument)      
      ain[0] = read_32float (40001, 2);


      // Scales the process variable for PID loop 0 in percent,
      // from the 1-5 VDC input seen at analog input 0
      pid[0].PV = (ain[0] * 25) - 25;


      // Read analog input AIN1 on the LabJack model T7 DAQ, 
      // using "CDAB" byte swapping (specified by the "2" argument)      
      ain[1] = read_32float (40003, 2);


      // Scales the feedforward load variable in percent,
      // from the 1-5 VDC input seen at analog input 1
      pid[0].FF_lv = (ain[1] * 25) - 25;
      pid[0].LOAD = pid[0].FF_lv;	// When this is a real PID controller, the FF load variable is the LOAD


      // Calculate the output of the PID algorithm
      pid_position (0);


      // Bounding trend interval to reasonable limits
      if (trend_interval < 1)
	trend_interval = 1;

      if (trend_interval > 9999)
	trend_interval = 9999;


      // Calls the trend-shifting function for PID loop 0
      // and plots trend data, every trend_interval number
      // of scans through the main while() loop
      if (scan_count % trend_interval == 0)
	trend_shift_plot (0);


      // Update the operator interface display for PID loop 0
      display_plot (0);


      // Read keyboard key strokes from the human operator
      // for PID loop 0
      keyboard_scan (0);

      // Sequences the dual analog output channels aout[0]
      // and aout[1] according to standard split-range
      // patterns.
      // dual_output (1) = "Parallel" outputs (no split-ranging)
      // dual_output (2) = "Complementary" split-ranging
      // dual_output (3) = "Exclusive" split-ranging
      // dual_output (4) = "Progressive" split-ranging
      dual_output (1);

      // Write analog outputs DAC0 and DAC1 on the LabJack model 
      // T7 DAQ, using "CDAB" byte swapping (specified by the 
      // "2" argument) 
      write_32float (aout[0], 41001, 2);
      write_32float (aout[1], 41003, 2);

      // Update the last scan time value
      time_lastscan = time_current;

      // Delays the PID and process simulation loop a specified 
      // number of nanoseconds using the awesome nanosleep() 
      // function.  This conserves processing power and network
      // bandwidth.
      nanosleep (&loopdelay, NULL);

    }


  ///////////////////////////////////////////////////////////
  //
  //  These instructions are executed only when the while()
  //  loop stops, which is triggered by looprun = 0.  Their
  //  purpose is to cleanly exit the program, including all
  //  the "housekeeping" instructions necessary to get out
  //  of ncurses mode and return to normal terminal mode,
  //  as well as closing any Modbus network connections.
  //
  ///////////////////////////////////////////////////////////

  // Closes out ncurses mode and returns to normal terminal mode
  close_ncurses ();


  // Closes out Modbus network connection
  close_modbus_connection ();


  // Force of habit here -- I like all functions to return *something*
  return 1;

}













int
set_defaults (void)
{

  // PID controller 0
  pid[0].SP = 50;		// Initialize setpoint to 50%
  pid[0].BIAS = 50;		// Initialize bias to 50%
  pid[0].K_P = 0.5;		// Initialize gain at a value of 0.5
  pid[0].K_I = 0;		// Zero repeats per minute integral
  pid[0].K_D = 0;		// Zero seconds derivative
  pid[0].PV_hihi = 95;		// Units of percent of the PV range
  pid[0].PV_hi = 75;		// Units of percent of the PV range
  pid[0].PV_lo = 25;		// Units of percent of the PV range
  pid[0].PV_lolo = 5;		// Units of percent of the PV range
  pid[0].URV = 12;		// Engineering units (e.g. number represents degrees, RPM, PSI, etc.)
  pid[0].LRV = 0;		// Engineering units (e.g. number represents degrees, RPM, PSI, etc.)
  pid[0].UNIT = "PSI";		// Engineering unit text label 
  pid[0].windup_hilimit = 99;	// Integral windup high limit
  pid[0].windup_lolimit = 1;	// Integral windup low limit
  pid[0].I_db = 0;		// Integral deadband (integration halts at error values less than this)
  pid[0].action = 0;		// 0 = Reverse      ;  1 = Direct
  pid[0].am_mode = 0;		// 0 = Manual       ;  1 = Automatic
  pid[0].equation = 0;		// 0 = Ideal        ;  1 = Parallel
  pid[0].type = 1;		// 0 = Simulation ; 1 = Single-loop ; 2 = Ratio ; 3 Cascade 

  /********************************************************
   *
   * The following code in this function is necessary only
   * when multiple PID loops are being executed, such as in
   * the case of a cascade control system.
   *

  // PID controller 1
  pid[1].SP = 50;		// Initialize setpoint to 50%
  pid[1].BIAS = 50;		// Initialize bias to 50%
  pid[1].K_P = 1;               // Initialize gain at a value of 1
  pid[1].K_I = 0;               // Zero repeats per minute integral
  pid[1].K_D = 0;               // Zero seconds derivative
  pid[1].PV_hihi = 95;		// Units of percent of the PV range
  pid[1].PV_hi = 90;		// Units of percent of the PV range
  pid[1].PV_lo = 10;		// Units of percent of the PV range
  pid[1].PV_lolo = 5;		// Units of percent of the PV range
  pid[1].URV = 100;		// Engineering units (e.g. number represents degrees, RPM, PSI, etc.)
  pid[1].LRV = 0;		// Engineering units (e.g. number represents degrees, RPM, PSI, etc.)
  pid[1].UNIT = "RPM";		// Engineering unit text label 
  pid[1].windup_hilimit = 99;   // Integral windup high limit
  pid[1].windup_lolimit = 1;    // Integral windup low limit
  pid[1].I_db = 0;              // Integral deadband (integration halts at error values less than this)
  pid[1].action = 0;		// 0 = Reverse      ;  1 = Direct
  pid[1].am_mode = 0;		// 0 = Manual       ;  1 = Automatic
  pid[1].equation = 0;		// 0 = Ideal        ;  1 = Parallel

  // PID controller 2
  pid[2].SP = 50;		// Initialize setpoint to 50%
  pid[2].BIAS = 50;		// Initialize bias to 50%
  pid[2].K_P = 1;               // Initialize gain at a value of 1
  pid[2].K_I = 0;               // Zero repeats per minute integral
  pid[2].K_D = 0;               // Zero seconds derivative
  pid[2].PV_hihi = 95;		// Units of percent of the PV range
  pid[2].PV_hi = 90;		// Units of percent of the PV range
  pid[2].PV_lo = 10;		// Units of percent of the PV range
  pid[2].PV_lolo = 5;		// Units of percent of the PV range
  pid[2].URV = 100;		// Engineering units (e.g. number represents degrees, RPM, PSI, etc.)
  pid[2].LRV = 0;		// Engineering units (e.g. number represents degrees, RPM, PSI, etc.)
  pid[2].UNIT = "RPM";		// Engineering unit text label 
  pid[2].windup_hilimit = 99;   // Integral windup high limit
  pid[2].windup_lolimit = 1;    // Integral windup low limit
  pid[2].I_db = 0;              // Integral deadband (integration halts at error values less than this)
  pid[2].action = 0;		// 0 = Reverse      ;  1 = Direct
  pid[2].am_mode = 0;		// 0 = Manual       ;  1 = Automatic
  pid[2].equation = 0;		// 0 = Ideal        ;  1 = Parallel

  ********************************************************/

  looprun = 1;			// Allows the PID loop to run
  select_mode = 0;		// User interface mode (0 = adjust OUT in manual or SP in automatic)
  scan_count = 0;		// Resets the scan counter at zero
  scans_per_second = 1.0;	// Initializes the scan/second value to 1
  trend_interval = 5;

  // Force of habit here -- I like all functions to return *something*
  return 1;
}









int
splash_screen (void)
{
  char key;
  int m, n;

  // Crude screen-clearing routine
  for (n = 0; n < 50; ++n)
    {
      printf ("\n");
    }

  printf ("\n");
  printf
    ("Welcome to the caSCADA PID controller!  This program implements \n");
  printf
    ("Proportional-Integral-Derivative (PID) control with a simple user \n");
  printf ("interface. \n");

  printf ("\n");
  printf ("Copyright (C) 2017 Tony R. Kuphaldt \n");
  printf ("This program comes with ABSOLUTELY NO WARRANTY! \n");
  printf ("This is free software, and you are welcome to distribute it \n");
  printf
    ("under certain conditions; view the file `GPL_v3.txt' for details. \n");

  printf ("\n \n \n");
  printf
    ("Edit the file `modbus_open_close.h' to change these network parameters, found as #define statements: \n\n");
  printf ("   LabJack IP address = %s \n", IP_ADDRESS);
  printf ("   TCP port number = %i \n", TCP_PORT);
  printf ("   Modbus slave address = %i \n", MODBUS_SLAVE);

  printf ("\n");
  printf
    ("Edit the file `pid.c' to change these range parameters, found in the set_defaults() function: \n\n");
  printf
    ("   Process variable lower range value = %f   High-high alarm = %f   High alarm = %f \n",
     pid[0].LRV, pid[0].PV_hihi, pid[0].PV_hi);
  printf
    ("   Process variable upper range value = %f   Low-Low alarm = %f   Low alarm = %f \n",
     pid[0].URV, pid[0].PV_lolo, pid[0].PV_lo);
  printf ("   Process variable engineering units = %s \n", pid[0].UNIT);


  // Give the user a chance to review the default settings before seeing the
  // trend graph size!
  printf ("\n");
  printf
    ("If any of these default settings is incorrect, please quit now and\n");
  printf ("edit the source code to your liking.\n");

  printf ("\nEnter Y to continue, anything else to quit ");
  scanf ("%c", &key);

  if (key == 'Y' || key == 'y')
  {
    printf("Network address, range values, and alarm values all accepted");
    scanf ("%c", &key); // This additional scanf instruction reads the Enter keystroke left over from the first one
  }

  else
    return 0;


  // Crude screen-clearing routine
  for (n = 0; n < 50; ++n)
    {
      printf ("\n");
    }


  printf ("+");

  for (m = 0; m < TRENDWIDTH; ++m)
    printf ("-");

  printf ("+\n");

  for (n = 0; n < TRENDHEIGHT; ++n)
    {
      printf ("|");

      for (m = 0; m < TRENDWIDTH; ++m)
	printf (" ");

      printf ("|");

      if (n == 2)
	printf ("HH");

      if (n == 4)
	printf ("H");

      if (n == TRENDHEIGHT - 5)
	printf ("L");

      if (n == TRENDHEIGHT - 3)
	printf ("LL");

      printf ("\n");

    }

  printf ("+");

  for (m = 0; m < TRENDWIDTH; ++m)
    printf ("-");

  printf ("+\n");

  printf ("\n");
  printf
    ("If the trend display size is not to your liking, please quit now and\n");
  printf ("edit the source code (file: controller.h).\n");

  printf ("\nEnter Y to continue, anything else to quit ");
  scanf ("%c", &key);

  if (key == 'Y' || key == 'y')
    return 1;

  else
    return 0;

}











int
dual_output (int type)
{

  ////////////////////////////////////////////////////////////////
  //
  // This function provides "split-range" sequencing of the dual
  // analog output channels on the DAQ.  The "type" variable 
  // specifies the type of sequencing:
  //
  // 1 = Parallel 
  //     pid[].OUT = 0 --- 25 --- 50 --- 75 --- 100
  //     aout[0] =   0 --- 25 --- 50 --- 75 --- 100
  //     aout[1] =   0 --- 25 --- 50 --- 75 --- 100
  //
  // 2 = Complementary 
  //     pid[].OUT = 0 --- 25 --- 50 --- 75 --- 100
  //     aout[0] =   0 --- 25 --- 50 --- 75 --- 100
  //     aout[1] = 100 --- 75 --- 50 --- 25 --- 0
  //
  // 3 = Exclusive 
  //     pid[].OUT = 0 --- 25 --- 50 --- 75 --- 100
  //     aout[0] =   0 --- 0  --- 0  --- 50 --- 100
  //     aout[1] = 100 --- 50 --- 0  --- 0  --- 0
  //
  // 4 = Progressive 
  //     pid[].OUT = 0 --- 25 --- 50 ---- 75 ---- 100
  //     aout[0] =   0 --- 50 --- 100 --- 100 --- 100
  //     aout[1] =   0 --- 0  --- 0  ---- 50  --- 100
  //
  ////////////////////////////////////////////////////////////////

  // Scales the two analog outputs PID loop 0 according to the
  // selected split-range sequencing pattern.  In all cases,
  // a floating-point range of 1.00 to 5.00 Volts is assumed
  // in accordance with the LabJack T7 DAQ's convention.

  switch (type)
    {
    case 1:			// Parallel split-ranging
      aout[0] = (pid[0].OUT / 25) + 1;
      aout[1] = (pid[0].OUT / 25) + 1;
      break;

    case 2:			// Complementary split-ranging 
      aout[0] = (pid[0].OUT / 25) + 1;
      aout[1] = (pid[0].OUT / -25) + 5;
      break;

    case 3:			// Exclusive split-ranging 
      aout[0] = (pid[0].OUT / 12.5) - 3;
      aout[1] = (pid[0].OUT / -12.5) + 5;
      break;

    case 4:			// Progressive split-ranging 
      aout[0] = (pid[0].OUT / 12.5) + 1;
      aout[1] = (pid[0].OUT / 12.5) - 3;
      break;
    }


  // Safeguards against an unreasonably low value being written to
  // analog output channel 0, with 0.00 volts being the low limit
  if (aout[0] < 0)
    aout[0] = 0.00;

  // Safeguards against an unreasonably high value being written to
  // analog output channel 0, with 5.00 volts being the high limit
  if (aout[0] > 5.00)
    aout[0] = 5.00;

  // Safeguards against an unreasonably low value being written to
  // analog output channel 1, with 0.00 volts being the low limit
  if (aout[1] < 0)
    aout[1] = 0.00;

  // Safeguards against an unreasonably high value being written to
  // analog output channel 1, with 5.00 volts being the high limit
  if (aout[1] > 5.00)
    aout[1] = 5.00;


  // Force of habit here -- I like all functions to return *something*
  return 1;
}
