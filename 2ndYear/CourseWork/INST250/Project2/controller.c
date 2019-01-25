/*************************************************************************
caSCADA -- PID controller program 
By Tony R. Kuphaldt
Copyright (C) 2016-2018
Last update March 15, 2018

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

This file contains all the functions necessary for the PID algorithm
and the ncurses-based user interface, including functions for cleanly
opening and closing the ncurses user environment.

*************************************************************************/

#include <stdio.h>
#include <math.h>		// Necessary for the fabs() function
#include <ncurses.h>		// Necessary for the "ncurses" library functions for the operator display
#include "controller.h"		// Contains all the declarations specific to caSCADA's PID controller code


int
open_ncurses (void)
{

  // Starts ncurses mode (the console-based operator interface display)
  initscr ();

  // This ncurses function disables line buffering, which makes pressing "Enter" unnecessary
  // raw();   // raw() disables all normal keyboard control sequences
  cbreak ();			// cbreak() allows control sequences such as Ctrl-C to still work like normal

  // This ncurses function gives us access to Function keys (F1, F2, etc.) on the keyboard
  keypad (stdscr, TRUE);

  // Don't echo characters to the screen whenever the getch() function is called to accept keyboard input
  noecho ();

  // Don't halt the program execution when the getch() function is called
  nodelay (stdscr, TRUE);

  curs_set (0);			// Makes the cursor invisible

  // Test to see if our ncurses environment supports colors, which is very nice
  if (has_colors () == FALSE)
    {
      endwin ();
      printf ("Sorry, but colors aren't supported in your terminal! \n");
      return 0;
    }

  // Starts ncurses color capability
  start_color ();

  // Defining text/background color pair combinations for ncurses display
  //
  // Color pair "0" is default: WHITE text on a BLACK background
  //
  // Defines color pair "1" as BLACK text on a GREEN background
  init_pair (1, COLOR_BLACK, COLOR_GREEN);
  //
  // Defines color pair "11" as WHITE text on a GREEN background
  init_pair (11, COLOR_WHITE, COLOR_GREEN);
  //
  // Defines color pair "2" as BLACK text on a YELLOW background
  init_pair (2, COLOR_BLACK, COLOR_YELLOW);
  //
  // Defines color pair "22" as WHITE text on a YELLOW background
  init_pair (22, COLOR_WHITE, COLOR_YELLOW);
  //
  // Defines color pair "3" as BLACK text on a WHITE background
  init_pair (3, COLOR_BLACK, COLOR_WHITE);
  //
  // Defines color pair "4" as RED text on a BLACK background
  init_pair (4, COLOR_RED, COLOR_BLACK);
  //
  // Defines color pair "44" as RED text on a WHITE background
  init_pair (44, COLOR_RED, COLOR_WHITE);


  // Force of habit here -- I like all functions to return *something*
  return 1;
}





int
close_ncurses (void)
{
  endwin ();

  // Force of habit here -- I like all functions to return *something*
  return 1;
}











int
pid_position (int c)
{

  // Declaring some of the variables used within this function

  float error;			// Difference between PV and SP
  float derivative;		// Rate-of-change with respect to time
  int integral_inhibit;		// 0 = Allow integration ; 1 = Stop integration
  static int scan_count_last;	// Scan count from the last scan
  static float PV_history[10];	// Array storing the last several PV values
  int n;


  // Bounding process variable between limits of -5% and +105%
  if (pid[c].PV > 105)
    pid[c].PV = 105;

  if (pid[c].PV < -5)
    pid[c].PV = -5;

  PV_history[0] = pid[c].PV;

  // Bounding setpoint between limits of -5% and +105%
  if (pid[c].SP > 105)
    pid[c].SP = 105;

  if (pid[c].SP < -5)
    pid[c].SP = -5;


  // Bounding tuning coefficients between reasonable limits
  if (pid[c].K_P > 50.0)
    pid[c].K_P = 50.0;

  if (pid[c].K_P < 0)
    pid[c].K_P = 0;

  if (pid[c].K_I > 400.0)
    pid[c].K_I = 400.0;

  if (pid[c].K_I < 0)
    pid[c].K_I = 0;

  if (pid[c].K_D > 99.0)
    pid[c].K_D = 99.0;

  if (pid[c].K_D < 0)
    pid[c].K_D = 0;

  if (pid[c].I_db > 9.9)
    pid[c].I_db = 9.9;

  if (pid[c].I_db < 0)
    pid[c].I_db = 0;

  if (pid[c].FF_gain > 9.9)
    pid[c].FF_gain = 9.9;

  if (pid[c].FF_gain < -9.9)
    pid[c].FF_gain = -9.9;

  if (pid[c].FF_bias > 99.0)
    pid[c].FF_bias = 99.0;

  if (pid[c].FF_bias < -99.0)
    pid[c].FF_bias = -99.0;





  // Calculating program loop scan rate.  This is important for accurate
  // calculation of time-based control elements such as integral and
  // derivative (the "I" and "D" terms of the PID algorithm).  Since the
  // operating system is *not* executing PID on a locked-time schedule
  // as is the case with most industrial control systems, we need our
  // program to determine how long each "loop" takes and use that live
  // scan-time value to compensate for anything that slows the processor
  // down (e.g. file read/write operations, user logins, etc.).

  if (time_current != time_lastscan)	// Every time the system clock increments
    {
      scans_per_second =
	(float) (scan_count - scan_count_last) / (float) (time_current -
							  time_lastscan);
      scan_count_last = scan_count;

      // This bounds scans_per_second to a reasonable value in case we get
      // a weird number (e.g. when the program first starts up)
      if (scans_per_second < 5.0)
	scans_per_second = 5.0;
    }


  // Calculate the feedforward contribution to the controller's output
  // (FF) regardless if it will even be used.
  pid[c].FF = (pid[c].FF_lv * pid[c].FF_gain) + pid[c].FF_bias;


  // EXECUTION IN AUTOMATIC MODE!
  if (pid[c].am_mode == 1)
    {

      // Calculating error, reverse action
      if (pid[c].action == 0)
	error = pid[c].SP - pid[c].PV;

      // Calculating error, direct action
      else
	error = pid[c].PV - pid[c].SP;


      // The following if/else conditionals will inhibit integral (reset) action
      // under certain conditions, such as the output exceeding windup limits, or
      // exceeding saturation limits (0 and 100%), or if the error between PV and
      // SP is less than the "integral deadband" value.  The latter condition is
      // useful in control applications where the final control element
      // (e.g. control valve, or motor-operated device) is incapable of precise
      // positioning, which will cause integral action to ceaselessly ramp up and
      // down in a futile effort to precisely achieve setpoint.  Integral deadband
      // simply says that getting close to setpoint is good enough, and stops the
      // integral action within that distance of PV.

      // Set inhibit = 1 if we try to exceed the high windup limit or 100 percent
      if (((pid[c].OUT > pid[c].windup_hilimit) || (pid[c].OUT > 100))
	  && error > 0)
	integral_inhibit = 1;
      // Set inhibit = 1 if we try to go below the low windup limit or 0 percent
      else if (((pid[c].OUT < pid[c].windup_lolimit) || (pid[c].OUT < 0))
	       && error < 0)
	integral_inhibit = 1;
      // Set inhibit = 1 if error is less than the integral deadband
      else if (fabs (error) < fabs (pid[c].I_db))
	integral_inhibit = 1;
      else
	integral_inhibit = 0;	// Otherwise, let integration occur


      // Integral summation, accumulated as the BIAS value as long as it is not inhibited.
      // For each scan, the BIAS value gets incremented or decremented
      // by an amount equal to the error times the difference in scan times (delta t)
      // times the integral coefficient divided by 60 (converting rpm to rps).
      // For the Ideal equation we also multiply by K_P.

      if (integral_inhibit == 0)
	{
	  if (pid[c].equation == 0)	// Ideal equation
	    pid[c].BIAS =
	      pid[c].BIAS +
	      (pid[c].K_P * error * pid[c].K_I / (60 * scans_per_second));

	  if (pid[c].equation == 1)	// Parallel equation
	    pid[c].BIAS =
	      pid[c].BIAS + (error * pid[c].K_I / (60 * scans_per_second));
	}


      // Derivative calculation, based on rate-of-change of PV.
      // In order to avoid differentiating over unreasonably short
      // time intervals (where unavoidable jumps in the PV stemming
      // from noise or digital count value changes result in huge
      // dPV/dt values), we will calculate the rate of PV change
      // over multiple scans, using the PV_history[] array which
      // is refreshed with every scan of this PID function.
      // PV_history[0] is the current PV value, while
      // PV_history[9] is the oldest.  We can set the dt interval
      // as wide as we like (PV_history[9] - PV_history[0]) or as
      // narrow as we like (PV_history[1] - PV_history[0]).  The
      // wider the interval (i.e. greater dt), the greater the
      // divider constant must be following "scans_per_second"

      if (pid[c].equation == 0)	// Ideal equation assuming reverse action
	derivative =
	  pid[c].K_P * pid[c].K_D * (PV_history[2] -
				     PV_history[0]) * scans_per_second / 2;

      if (pid[c].equation == 1)	// Parallel equation assuming reverse action
	derivative =
	  pid[c].K_D * (PV_history[2] - PV_history[0]) * scans_per_second / 2;

      if (pid[c].action == 1)	// Swaps sign of derivative term if direct action
	derivative = -derivative;


      //////////////////////////////////////////////////////////////////////
      // This section of the function puts the P, I, and D terms together
      //
      // Note that the integral term is actually called the "BIAS" because
      // that is what integral does in a PID controller: continually adjust
      // the bias value.
      //////////////////////////////////////////////////////////////////////

      // If Feedforward gain is essentially set to zero (less than 0.05 and
      // greater than -0.05), just do PID control and don't add any 
      // feedforward action at all!
      if (pid[c].FF_gain < 0.05 && pid[c].FF_gain > -0.05)
	pid[c].OUT = (pid[c].K_P * error) + pid[c].BIAS + derivative;

      // This assumes we want feedforward action in effect, so we calculate
      // the value of the feedforward contribution (FF) by multiplying the
      // measured load variable (FF_lv) by feedforward gain (FF_gain) and
      // add feedforward bias (FF_bias).  After than, we add FF to the
      // PID sum as a final term.
      else
	pid[c].OUT =
	  (pid[c].K_P * error) + pid[c].BIAS + derivative + pid[c].FF;

    }





  // EXECUTION IN MANUAL MODE!
  // Setting SP equal to PV provides setpoint tracking while in manual mode.
  // Setting OUT equal to BIAS provides output tracking.  While in manual mode,
  // the user's adjustments increment and decrement BIAS, which in turn is
  // used as the OUTput.  This way, when they switch back to automatic mode,
  // the BIAS value provides a starting point for the PID position algorithm
  // and you have bumpless transfer from manual to auto.
  if (pid[c].am_mode == 0)
    {
      pid[c].SP = pid[c].PV;	// This provides setpoint tracking in manual mode
      pid[c].OUT = pid[c].BIAS;
    }


  // Bounding output between limits of -5% and +105%
  if (pid[c].OUT > 105)
    pid[c].OUT = 105;

  if (pid[c].OUT < -5)
    pid[c].OUT = -5;

  // Updates "PV_history" array with every scan of this function
  for (n = 9; n > 0; --n)
    {
      PV_history[n] = PV_history[n - 1];
    }

  // Force of habit here -- I like all functions to return *something*
  return 1;
}






int
display_plot (int c)
{

  int blink_color;
  int n, m;

  ////////////////////////////////////////////////////////////////
  //
  // This function prints static and numerical data to screen
  //
  ////////////////////////////////////////////////////////////////

  // Using the attron() and attroff() functions to control ncurses coloring
  // Using the move() and printw() functions to place text on the screen
  //   syntax: move( y , x ) where 0,0 is the upper-left corner

  // Placing the title and function key legend at the program start,
  // rather than re-placing all this static text with every single
  // call of the display_plot() function (this saves times).
  if (scan_count < 2)
    {
      attron (COLOR_PAIR (3));
      move (3, 29);
      printw ("caSCADA PID version 3.5");
      move (4, 29);
      printw ("   Copyright (C) 2018  ");
      move (5, 29);
      printw ("    GNU GPL version 3  ");
      move ((TRENDHEIGHT + 11), 1);
      printw ("F1 (M) Manual");
      move ((TRENDHEIGHT + 12), 1);
      printw ("F2 (A) Auto  ");
      move ((TRENDHEIGHT + 14), 48);
      printw ("F4 (pgdwn) - 10 ");
      move ((TRENDHEIGHT + 13), 48);
      printw ("F5 (down)  - 1  ");
      move ((TRENDHEIGHT + 12), 48);
      printw ("F6 (left)  - 0.1");
      move ((TRENDHEIGHT + 11), 48);
      printw ("F7 (right) + 0.1");
      move ((TRENDHEIGHT + 10), 48);
      printw ("F8  (up)   + 1  ");
      move ((TRENDHEIGHT + 9), 48);
      printw ("F9 (pgup)  + 10 ");
      move ((TRENDHEIGHT + 9), 67);
      printw ("F11 (S) Select");	// This function key changes the select_mode variable
      move ((TRENDHEIGHT + 11), 71);
      printw ("(Q) Exit");
      move ((TRENDHEIGHT + 13), 66);
      printw ("F12 (T) Capture");
      move ((TRENDHEIGHT + 14), 66);
      printw ("  trend data   ");
      attroff (COLOR_PAIR (3));
    }


  // Placing the PV, SP, OUT, and LOAD numerical displays
  attron (COLOR_PAIR (1));
  move (0, 1);
  printw ("PV = %3.1f percent = %f %s  ", pid[c].PV,
	  (pid[c].PV / 100) * (pid[c].URV - pid[c].LRV) + pid[c].LRV,
	  pid[c].UNIT);

  if (select_mode == 0 && pid[c].am_mode == 1)
    attron (COLOR_PAIR (11));
  else
    attron (COLOR_PAIR (1));
  move (1, 1);
  printw ("SP = %3.1f percent = %f %s  ", pid[c].SP,
	  (pid[c].SP / 100) * (pid[c].URV - pid[c].LRV) + pid[c].LRV,
	  pid[c].UNIT);

  if (select_mode == 0 && pid[c].am_mode == 0)
    attron (COLOR_PAIR (11));
  else
    attron (COLOR_PAIR (1));
  move (2, 1);
  printw ("OUT = %3.1f percent ", pid[c].OUT);
  attroff (COLOR_PAIR (1));

  // Print the LOAD ("WILD") value only if we want 
  // it there.  Otherwise, print over it with black
  // on black.
  attron (COLOR_PAIR (1));
  move (3, 1);
  if (pid[c].FF_gain > 0.05 || pid[c].FF_gain < -0.05)
    printw ("WILD = %3.1f percent ", pid[c].LOAD);
  else
    printw ("                     ");


  // Placing the PID scan execution rate display
  attron (COLOR_PAIR (1));
  move (5, 1);
  printw ("PID scan rate = %.1f/sec ", scans_per_second);
  attroff (COLOR_PAIR (1));



  // PV alarm displays, placement and coloring

  // For high-high and low-low alarm conditions, we want the alarm
  // display to alternate background colors (white/black) to better
  // grab the user's attention.  This pair of if/else statements
  // makes the "blink_color" variable switch between values of 4
  // and 44 which are the color pair codes for red on black versus
  // red on white.
  if (time_current % 2 == 0)
    blink_color = 44;

  else
    blink_color = 4;



  if (pid[c].PV > pid[c].PV_hihi)
    {
      attron (COLOR_PAIR (blink_color));
      move (4, 1);
      printw ("PV high-high alarm!");
      attroff (COLOR_PAIR (blink_color));
    }

  else if (pid[c].PV > pid[c].PV_hi)
    {
      attron (COLOR_PAIR (44));
      move (4, 1);
      printw ("PV high alarm!     ");
      attroff (COLOR_PAIR (44));
    }

  else if (pid[c].PV < pid[c].PV_lolo)
    {
      attron (COLOR_PAIR (blink_color));
      move (4, 1);
      printw ("PV low-low alarm!  ");
      attroff (COLOR_PAIR (blink_color));
    }

  else if (pid[c].PV < pid[c].PV_lo)
    {
      attron (COLOR_PAIR (44));
      move (4, 1);
      printw ("PV low alarm!      ");
      attroff (COLOR_PAIR (44));
    }

  else
    {
      attron (COLOR_PAIR (0));
      move (4, 1);
      printw ("                   ");
      attroff (COLOR_PAIR (0));
    }

  // PV alarm markers to the right of the trend graph
  attron (COLOR_PAIR (0));

  move ((TRENDHEIGHT + 7) -
	(int) (((pid[c].PV_lolo + (50 / TRENDHEIGHT)) / 100) * (TRENDHEIGHT -
								1)),
	(TRENDWIDTH + 3));
  printw ("LL");

  move ((TRENDHEIGHT + 7) -
	(int) (((pid[c].PV_lo + (50 / TRENDHEIGHT)) / 100) * (TRENDHEIGHT -
							      1)),
	(TRENDWIDTH + 3));
  printw ("L");

  move ((TRENDHEIGHT + 7) -
	(int) (((pid[c].PV_hi + (50 / TRENDHEIGHT)) / 100) * (TRENDHEIGHT -
							      1)),
	(TRENDWIDTH + 3));
  printw ("H");

  move ((TRENDHEIGHT + 7) -
	(int) (((pid[c].PV_hihi + (50 / TRENDHEIGHT)) / 100) * (TRENDHEIGHT -
								1)),
	(TRENDWIDTH + 3));
  printw ("HH");

  attroff (COLOR_PAIR (0));




  // Placing the modes and tuning parameters display, the
  // color scheme of each parameter switching based on the
  // value of select_mode.  This shows the user which
  // parameter they are set to adjust with the increment/
  // decrement controls.

  // This simply makes a solid field of background color over
  // which the tuning parameters will be printed.  This solid
  // field covers coordinates (0,54) to (5,82).
  attron (COLOR_PAIR (2));
  for (n = 54; n < 83; ++n)
    {
      for (m = 0; m < 6; ++m)
	{
	  move (m, n);
	  printw (" ");
	}
    }

  if (select_mode == 0)
    attron (COLOR_PAIR (22));
  else
    attron (COLOR_PAIR (2));
  move (0, 54);
  if (pid[c].am_mode == 0)
    printw ("Man -- (adjust output)");
  else
    printw ("Auto -- (adjust setpoint)");

  if (select_mode == 1)
    attron (COLOR_PAIR (22));
  else
    attron (COLOR_PAIR (2));
  move (1, 54);
  printw ("K_P = %1.1f (gain)", pid[c].K_P);
  attroff (COLOR_PAIR (2));

  if (select_mode == 2)
    attron (COLOR_PAIR (22));
  else
    attron (COLOR_PAIR (2));
  move (1, 72);
  printw ("FFG= %1.1f", pid[c].FF_gain);
  attroff (COLOR_PAIR (2));

  if (select_mode == 3)
    attron (COLOR_PAIR (22));
  else
    attron (COLOR_PAIR (2));
  move (2, 54);
  printw ("K_I = %1.1f r/min", pid[c].K_I);
  attroff (COLOR_PAIR (2));

  if (select_mode == 4)
    attron (COLOR_PAIR (22));
  else
    attron (COLOR_PAIR (2));
  move (2, 72);
  printw ("IDB= %1.1f%%", pid[c].I_db);
  attroff (COLOR_PAIR (2));

  if (select_mode == 5)
    attron (COLOR_PAIR (22));
  else
    attron (COLOR_PAIR (2));
  move (3, 54);
  printw ("K_D = %1.1f sec", pid[c].K_D);
  attroff (COLOR_PAIR (2));

  if (select_mode == 6)
    attron (COLOR_PAIR (22));
  else
    attron (COLOR_PAIR (2));
  move (3, 72);
  printw ("FFB= %1.1f%%", pid[c].FF_bias);
  attroff (COLOR_PAIR (2));

  if (select_mode == 7)
    attron (COLOR_PAIR (22));
  else
    attron (COLOR_PAIR (2));
  move (4, 54);
  if (pid[c].action == 0)
    printw ("Reverse-acting, ");
  else
    printw ("Direct-acting, ");
  attroff (COLOR_PAIR (2));

  if (select_mode == 8)
    attron (COLOR_PAIR (22));
  else
    attron (COLOR_PAIR (2));
  move (4, 72);
  if (pid[c].equation == 0)
    printw ("Ideal");
  else
    printw ("Parallel");
  attroff (COLOR_PAIR (2));

  if (select_mode == 9)
    attron (COLOR_PAIR (22));
  else
    attron (COLOR_PAIR (2));
  move (5, 54);
  printw ("Trend interval = %i scans", trend_interval);
  attroff (COLOR_PAIR (2));


  // Calculating the timebase of the trend graph in real time units
  // (number of ticks per second of real time)
  timebase = scans_per_second / trend_interval;

  // Placing the trend timebase display
  attron (COLOR_PAIR (1));
  move ((TRENDHEIGHT + 9), 1);
  if (timebase >= 1)
    printw ("Trend update rate = %.1f ticks per second  ", timebase);
  else
    printw ("Trend update rate = %.1f seconds per tick", 1 / timebase);
  attroff (COLOR_PAIR (1));


  // The refresh() ncurses function prints all the preceding text to the screen
  refresh ();


  // Force of habit here -- I like all functions to return *something*
  return 1;
}









int
keyboard_scan (int c)
{

  int key;
  float step = 0.0;

  ///////////////////////////////////////////////////////
  // This function reads keyboard input with each call,
  // without halting the execution of the PID loop.
  ///////////////////////////////////////////////////////

  key = getch ();

  switch (key)
    {

      ///////////////////////////////////////////////////////
      //
      // Switching to Manual and Automatic modes involves 
      // manipulation of the BIAS value for bumpless transfer.
      // "Bumpless" transfer refers to the controller's OUT
      // value unchanging as the mode is switched from manual
      // to automatic or vice-versa.  I'm using the BIAS 
      // variable to do this: when switching from automatic
      // into manual mode, the BIAS is set to equal to last
      // OUT value.  While in manual mode, OUT is elsewhere
      // set equal to BIAS, and any adjustments by the user
      // are made to BIAS.  When we switch from manual mode
      // back into automatic, we need to anticipate any
      // feedforward action that may have been taking place
      // during our time in manual, and so we re-set BIAS to
      // be the current OUT minus the feedforward (FF) so 
      // that once back in automatic mode we don't experience
      // a "jump" in OUT when FF gets added back into the 
      // PID formula.
      // 
      // Incidentally, trickery like this is necessary when
      // you're calculating PID using the "position" 
      // algorithm.  The "velocity" algorithm of PID makes
      // all this transfer stuff easier, but the actual
      // P+I+D calculations become harder to understand.
      // My design decision was to simplify the P+I+D
      // algorithm and live with a more complicated 
      // algorithm for ensuring bumpless transfer between
      // automatic and manual modes.
      //
      ///////////////////////////////////////////////////////

      // F1 or M switches to Manual mode
    case KEY_F (1):
    case 'M':
    case 'm':
      pid[c].am_mode = 0;
      pid[c].BIAS = pid[c].OUT;	// Captures last value of OUT to be used as the new BIAS
      select_mode = 0;		// Switch to mode 0 (OUT adjust) when changing to manual mode
      break;

      // F2 or A switches to Automatic mode
    case KEY_F (2):
    case 'A':
    case 'a':
      pid[c].am_mode = 1;
      pid[c].BIAS = pid[c].OUT - pid[c].FF;	// Captures last value of OUT minus FF contribution
      select_mode = 0;		// Switch to mode 0 (SP adjust) when changing to automatic mode
      break;

      // F4 or Page Down (Next Page) decrements by -10
    case KEY_F (4):
    case KEY_NPAGE:
      step = -10;
      break;

      // F5 or down arrow decrements by -1
    case KEY_F (5):
    case KEY_DOWN:
      step = -1;
      break;

      // F6 or left arrow decrements by -0.1
    case KEY_F (6):
    case KEY_LEFT:
      step = -0.1;
      break;

      // F7 or right arrow increments by +0.1
    case KEY_F (7):
    case KEY_RIGHT:
      step = 0.1;
      break;

      // F8 or up arrow increments by +1
    case KEY_F (8):
    case KEY_UP:
      step = 1;
      break;

      // F9 or Page Up (Previous Page) increments by +10
    case KEY_F (9):
    case KEY_PPAGE:
      step = 10;
      break;

      // F11 or S selects parameter to change
    case KEY_F (11):
    case 'S':
    case 's':
      select_mode = tuning_entry ();
      break;

      // F12 or T captures the trend graph data and saves to a comma-delimited text file (.csv)
    case KEY_F (12):
    case 'T':
    case 't':
      capture_trend ();
      break;

      // Q cleanly exits the program
    case 'Q':
    case 'q':
      looprun = 0;		// A value of 1 is necessary for continued execution from the calling function
      break;
    }



  // These conditional statements declare what to do with the increment/decrement
  // variable "step" depending on the mode of the controller
  if (pid[c].am_mode == 0 && select_mode == 0)
    {
      pid[c].BIAS = pid[c].BIAS + step;	// When in manual mode, change the BIAS value

      // Bounding BIAS between limits of -5% and +105% when in manual mode
      if (pid[c].BIAS > 105)
	pid[c].BIAS = 105;
      if (pid[c].BIAS < -5)
	pid[c].BIAS = -5;
    }

  if (pid[c].am_mode == 1 && select_mode == 0)
    pid[c].SP = pid[c].SP + step;	// When in automatic mode, change the SP value

  if (select_mode == 1)
    pid[c].K_P = pid[c].K_P + step;	// Adjusts proportional gain coefficient

  if (select_mode == 2)
    pid[c].FF_gain = pid[c].FF_gain + step;	// Adjusts feedforward gain coefficient

  if (select_mode == 3)
    pid[c].K_I = pid[c].K_I + step;	// Adjusts integral coefficient

  if (select_mode == 4)
    pid[c].I_db = pid[c].I_db + step;	// Adjusts integral deadband

  if (select_mode == 5)
    pid[c].K_D = pid[c].K_D + step;	// Adjusts derivative coefficient

  if (select_mode == 6)
    pid[c].FF_bias = pid[c].FF_bias + step;	// Adjusts feedforward bias coefficient

  if (select_mode == 7 && pid[c].am_mode == 0)	// Only allows change of action when in manual mode!
    {
      if (step > 0)		// Sets direct action
	pid[c].action = 1;

      if (step < 0)		// Sets reverse action
	pid[c].action = 0;
    }

  if (select_mode == 8 && pid[c].am_mode == 0)	// Only allows change of algorithm when in manual mode!
    {
      if (step > 0)		// Sets "parallel" equation
	pid[c].equation = 1;

      if (step < 0)		// Sets "ideal" equation
	pid[c].equation = 0;
    }

  if (select_mode == 9)
    trend_interval = trend_interval + (int) (step);	// Adjusts trend scan interval


  // Force of habit here -- I like all functions to return *something*
  return 1;
}








int
tuning_entry (void)
{

  // select_mode values:
  // 0 = operating mode -- adjusting OUT in manual or SP in automatic
  // 1 = adjust K_P
  // 2 = adjust FF_gain
  // 3 = adjust K_I
  // 4 = adjust integral deadband
  // 5 = adjust K_D
  // 6 = adjust FF_bias
  // 7 = adjust action (direct/reverse)
  // 8 = adjust equation (ideal/parallel)
  // 9 = adjust trend interval

  if (select_mode == 9)
    return 0;

  else if (select_mode == 8)
    return 9;

  else if (select_mode == 7)
    return 8;

  else if (select_mode == 6)
    return 7;

  else if (select_mode == 5)
    return 6;

  else if (select_mode == 4)
    return 5;

  else if (select_mode == 3)
    return 4;

  else if (select_mode == 2)
    return 3;

  else if (select_mode == 1)
    return 2;

  else if (select_mode == 0)
    return 1;

  else
    return 0;

}









int
trend_shift_plot (int c)
{

  int m, n;

  // This for() loop shifts data in all three trend arrays, to make the
  // trend points "march" across the screen

  // The 0th element of each array is the most recent, while the TRENDWIDTH-1
  // element is the oldest.  The following "for" loop takes data from the n-1
  // element and copies that data to the n element, thus updating all the
  // older elements with information from the newer.

  for (n = (TRENDWIDTH - 1); n > 0; --n)
    {
      pen[n].PV = pen[n - 1].PV;
      pen[n].SP = pen[n - 1].SP;
      pen[n].LOAD = pen[n - 1].LOAD;
      pen[n].OUT = pen[n - 1].OUT;
    }

  // After shifting all the old data, we populate the 0th element with
  // fresh data from the pid[c] structure:

  pen[0].PV = pid[c].PV;
  pen[0].SP = pid[c].SP;
  pen[0].LOAD = pid[c].LOAD;
  pen[0].OUT = pid[c].OUT;

  // Bounds trend data between 0 and 100 percent so that nothing can be
  // drawn outside the borders of the trend graph
  if (pen[0].PV < 0)
    pen[0].PV = 0;

  if (pen[0].PV > 100)
    pen[0].PV = 100;

  if (pen[0].SP < 0)
    pen[0].SP = 0;

  if (pen[0].SP > 100)
    pen[0].SP = 100;

  if (pen[0].LOAD < 0)
    pen[0].LOAD = 0;

  if (pen[0].LOAD > 100)
    pen[0].LOAD = 100;

  if (pen[0].OUT < 0)
    pen[0].OUT = 0;

  if (pen[0].OUT > 100)
    pen[0].OUT = 100;


  /////////////////////////////////////////////////////////////////////
  //
  // This section of the function prints graphical trend data to screen
  //
  /////////////////////////////////////////////////////////////////////

  // Trend graph border:
  //   upper-left corner coordinates (y,x)  = 7 , 1
  //   lower-right corner coordinates (y,x) = TRENDHEIGHT+2 +7 , TRENDWIDTH+2
  // This leaves a trend graph vertical span from y=8 to y=TRENDHEIGHT+8
  // and a horizontal span from x=2 to x=TRENDWIDTH+1

  // Draws upper border of trend graph
  attron (COLOR_PAIR (0));
  for (n = 1; n < (TRENDWIDTH + 3); ++n)
    {
      move (7, n);
      printw ("-");
    }
  attroff (COLOR_PAIR (0));

  // Prints banner text along the upper border of trend graph for a SIMULATED controller
  if (pid[c].type == 0)
  {
      attron (COLOR_PAIR (1));
      move (7, (TRENDWIDTH/2)-5);
      printw ("SIMULATION");
      attroff (COLOR_PAIR (1));
  }

  // Prints banner text along the upper border of trend graph for a SINGLE-LOOP controller
  if (pid[c].type == 1)
  {
      attron (COLOR_PAIR (44));
      move (7, (TRENDWIDTH/2)-6);
      printw ("SINGLE-LOOP");
      attroff (COLOR_PAIR (44));
  }


  // Draws lower border of trend graph
  attron (COLOR_PAIR (0));
  for (n = 1; n < (TRENDWIDTH + 3); ++n)
    {
      move (TRENDHEIGHT + 8, n);
      printw ("-");
    }

  // Draws left border of trend graph
  for (n = 7; n < (TRENDHEIGHT + 8); ++n)
    {
      move (n, 1);
      printw ("|");
    }

  // Draws right border of trend graph
  for (n = 7; n < (TRENDHEIGHT + 8); ++n)
    {
      move (n, (TRENDWIDTH + 2));
      printw ("|");
    }

  // Draws border corners of trend graph
  move (7, 1);
  printw ("+");
  move (7, (TRENDWIDTH + 2));
  printw ("+");
  move ((TRENDHEIGHT + 8), 1);
  printw ("+");
  move ((TRENDHEIGHT + 8), (TRENDWIDTH + 2));
  printw ("+");

  // Draws blank trend screen background.  This solution,
  // crude as it is, actually occupies fewer processor
  // cycles and makes for a more readable display than
  // using the ncurses clear() function!
  for (n = 8; n < (TRENDHEIGHT + 8); ++n)
    {
      for (m = 2; m < (TRENDWIDTH + 2); ++m)
	{
	  move (n, m);
	  printw (" ");		// Prints an empty space (BLACK) to the screen
	}
    }

  attroff (COLOR_PAIR (0));


  // Now we plot pen data to the screen

  attron (COLOR_PAIR (0));

  for (n = 0; n < TRENDWIDTH; ++n)
    {
      // Plot the LOAD variable only if the feedforward gain value
      // is greater than 0.05 or less than -0.05 (i.e. if we actually
      // have an "active" feedforward action).  This is a convenient
      // way to turn off the load pen if it is inconsequential.
      // The "w" letter stands for "Wild Variable"
      if (pid[c].FF_gain > 0.05 || pid[c].FF_gain < -0.05)
	{
	  move ((TRENDHEIGHT + 7) -
		(int) (((pen[n].LOAD +
			 (50 / TRENDHEIGHT)) / 100) * (TRENDHEIGHT - 1)),
		(TRENDWIDTH + 1) - n);
	  printw ("w");
	}

      // Plot the Output (OUT) variable
      move ((TRENDHEIGHT + 7) -
	    (int) (((pen[n].OUT + (50 / TRENDHEIGHT)) / 100) * (TRENDHEIGHT -
								1)),
	    (TRENDWIDTH + 1) - n);
      printw ("o");

      // Plot the Setpoint (SP) variable
      move ((TRENDHEIGHT + 7) -
	    (int) (((pen[n].SP + (50 / TRENDHEIGHT)) / 100) * (TRENDHEIGHT -
							       1)),
	    (TRENDWIDTH + 1) - n);
      printw ("s");

      // Plot the Process Variable (PV)
      move ((TRENDHEIGHT + 7) -
	    (int) (((pen[n].PV + (50 / TRENDHEIGHT)) / 100) * (TRENDHEIGHT -
							       1)),
	    (TRENDWIDTH + 1) - n);
      printw ("p");
    }

  attroff (COLOR_PAIR (0));


  // The refresh() ncurses function prints all the preceding text to the screen
  refresh ();


  // Force of habit here -- I like all functions to return *something*
  return 1;
}








int
capture_trend (void)
{

  int n;
  FILE *fp;			// Pointer to a FILE type


  // opening the data file "0_pid_trend.csv" write-only
  if ((fp = fopen ("0_pid_trend.csv", "w")) == NULL)
    {
      fprintf (stderr, "Can't open `0_pid_trend.csv' text file \n");	// Do this if the file doesn't exist
      return 2;			// Return value of "2" means we cannot open the file
    }


  // Prints a "header" line to label each column of data
  fprintf (fp, "Sample , PV , SP , OUT , WILD \n");


  // Write pen data to trend capture file
  for (n = (TRENDWIDTH - 1); n > 0; --n)
    {
      fprintf (fp, "%i , %f , %f , %f , %f \n", TRENDWIDTH - n, pen[n].PV,
	       pen[n].SP, pen[n].OUT, pen[n].LOAD);
    }


  // Close the "0_pid_trend.csv" text file
  fclose (fp);


  // Force of habit here -- I like all functions to return *something*
  return 1;
}
