#include "LPC214x.h"
#include "main.h"
#include "system.h"
#include "uart.h"
#include "hardware.h"
#include "irq.h"


void LED(unsigned char nr, unsigned char onoff) //set or reset LED 0..3
{
  if (nr>=2)
  	return;
  if(onoff == OFF)
  {
    IOSET1 = (1<<(24+nr));
  }
  else
  {
    IOCLR1 = (1<<(24+nr));
  }
}
