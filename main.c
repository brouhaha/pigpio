#include <inttypes.h>
#include <stdbool.h>
#include <stdio.h>
#include <sysexits.h>

#include <unistd.h>

#include "gpio.h"


#ifdef __GNUC__
#  define UNUSED(x) UNUSED__ ## x __attribute__((__unused__))
#else
#  define UNUSED(x) UNUSED__ ## x
#endif


int main(int UNUSED(argc), char * UNUSED(argv[]))
{
  volatile gpio_regs_t *gpio;

  gpio = map_gpio();
  if (! gpio)
    return EX_UNAVAILABLE;

  int io_bit = 2;

  gpio_set_mode(gpio, io_bit, GPIO_OUTPUT);

  bool v = 0;
  
  while (true)
    {
      gpio_bit_set(gpio, io_bit, v);
      usleep(250000);
      v ^= 1;
    }

  return EX_OK;
}
