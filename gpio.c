/*
 * Copyright (c) 2016, Eric Smith <spacewar@gmail.com>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above
 * copyright notice, this list of conditions and the following
 * disclaimer in the documentation and/or other materials provided
 * with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <assert.h>
#include <inttypes.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include <errno.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>

#include "gpio.h"


gpio_regs_t *map_gpio(void)
{
  int fd;
  gpio_regs_t *p;

  if (0 > (fd = open("/dev/mem", O_RDWR | O_SYNC)))
    {
      fprintf(stderr, "Unable to open /dev/mem: %s\n", strerror(errno));
      return NULL;
    }

  p = mmap(NULL,
	   sizeof(gpio_regs_t), // getpatgesize(),
	   PROT_READ | PROT_WRITE,
	   MAP_SHARED,
	   fd,
	   RASPBERRY_PI_REGS + GPIO_REG_OFFSET);
  if (p == MAP_FAILED)
    {
      fprintf(stderr, "Unable to mmap() GPIO: %s\n", strerror(errno));
      return NULL;
    }

  return p;
}


void bf_insert(volatile uint32_t *reg, int rightmost_bit, int width, uint32_t value)
{
  uint32_t v;
  uint32_t mask;

  assert(rightmost_bit < 32);
  assert((rightmost_bit + width) <= 32);

  mask = ((1 << width) - 1) << rightmost_bit;
  v = *reg;
  v &= ~mask;
  v |= (value << rightmost_bit);
  *reg = v;
}


void gpio_set_mode(volatile gpio_regs_t *gpio, int io_bit, gpio_mode_t mode)
{
  int group, pos;

  assert(io_bit >= 0 && io_bit < MAX_GPIO_PINS);
  group = io_bit / 10;
  pos = io_bit % 10;

  bf_insert(& gpio->gp_fsel[group], pos * 3, 3, mode);
}


void gpio_bit_set(volatile gpio_regs_t *gpio, int io_bit, bool value)
{
  
  int group;
  uint32_t v;

  assert(io_bit >= 0 && io_bit < MAX_GPIO_PINS);
  group = io_bit >> 5;
  v = 1 << (io_bit & 0x1f);

  if (value)
    gpio->gp_set[group] = v;
  else
    gpio->gp_clr[group] = v;
}
