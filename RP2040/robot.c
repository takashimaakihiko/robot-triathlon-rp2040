/******************************************************************
   robot.c  (RP2040 port)

   Robot Triathlon standard robot control program template
   Ported from H8/3664F to RP2040 (Raspberry Pi Pico)

   Original by YOU-YOU ROBOTICS (2002/8/1)
*******************************************************************/

#include "robot.h"

void program0(void)
{
  /* Write your program here (executed when SWDATA=0) */
  
}

void program1(void)
{
  /* Write your program here (executed when SWDATA=1) */
  
}

void program2(void)
{
  /* Write your program here (executed when SWDATA=2) */
  
}

void program3(void)
{
  /* Write your program here (executed when SWDATA=3) */
  
}

int main(void)
{
  /* Initialize hardware (replaces H8 assembler RESET routine) */
  hw_init();

  /* Select program based on dip switch value */
  switch(SWDATA)
  {
    case 0:  program0();  break;
    case 1:  program1();  break;
    case 2:  program2();  break;
    default: program3();  break;
  }

  /* Loop forever after program completes */
  while(1) { tight_loop_contents(); }
  return 0;
}
