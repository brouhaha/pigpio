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

/*
  Raspbery Pi 3 GPIO pinout

   +------+------------------------------------------------+----------------+
   |  /\  | 2 4 6 ............................... 36 38 40 |  /\    +-------+
   |  \/  | 1 3 5 ................................35 37 39 |  \/    |       |
   |      +------------------------------------------------+        |  USB  |
   |                                                                |       |
   |                                                                +-------+
   +-+                                                                      |
   |D|            +---------+                                       +-------+
   |i|            |         |                                       |       |
   |s|            |Broadcom |                                       |  USB  |
   |p|            | BCM2837 |                                       |       |
   |l|            |         |                 +-+                   +-------+
   |a|            +---------+                 |C|  +-+                      |
   |y|                                        |a|  |A|             +--------+
   +-+                                        |m|  |u|             |        |
   |                   +----------+           |e|  |d|             |Ethernet|
   |  /\  +-----+      |          |           |r|  |i|        /\   |        |
   |  \/  | USB |      |   HDMI   |           |a|  |o|        \/   +--------+
   +------+-----+------+----------+-----------+-+--+-+----------------------+


pin   fcn      alt0       alt1      alt2       alt3       alt4       alt5
---   -------  ---------  --------  ---------  ---------  ---------  ---------
  1   3.3V
  2   5V
  3   GPIO 2   I2C1 SDA   SMI SA3   DPI Vsync  -          -          -
  4   5V
  5   GPIO 3   I2C1 SCL   SMI SA2   DPI Hsync  -          -          -
  6   ground
  7   GPIO 4   GPCLK0     SMI SA1   -          -          -          JTAG TDI
  8   GPIO 14  UART0 TXD  SMI SD6   DPI Grn 6  -          -          UART1 TXD
  9   ground
 10   GPIO 15  UART0 RXD  SMI SD7   DPI Grn 7  -          -          UART1 RXD
 11   GPIO 17  -          SMI SD9   DPI Red 3  UART0 RTS  SPI1 CE1   UART1 RTS
 12   GPIO 18  PCM CLK    SMI SD10  DPI Red 4  BSCSL      SPI1 CE0   PWM0
                                               SDA/MOSI
 13   GPIO 27  SD0 DAT3   -         -          SD1 DAT3   JTAG TMS   -
 14   ground
 15   GPIO 22  SD0 CLK    SMI SD14  -          SD1 CLK    JTAG TRST  -
 16   GPIO 23  SD0 CMD    SMI SD15  -          SD1 CMD    JTAG RTCK  -
 17   3.3V
 18   GPIO 24  SD0 DAT0   SMI SD16  -          SD1 DAT0   JTAG TDO   -
 19   GPIO 10  SPI0 MOSI  SMI SD2   DPI Grn 2  -          -          -
 20   ground
 21   GPIO 9   SPI0 MISO  SMI SD1   DPI Blu 7  -          -          -
 22   GPIO 25  SD0 DAT1   SMI SD17  0          SD1 DAT1   JTAG TCK   0
 23   GPIO 11  SPI0 SCLK  SMI SD3   DPI Grn 3  -          -          -
 24   GPIO 8   SPI0 CE0   SMI SD0   DPI Blu 6  -          -          -
 25   ground
 26   GPIO 7   SPI0 CE1   SMI SWEN  DPI Blu 5  -          -          -
                          SRWN
 27   GPIO 0   I2C0 SDA   SMI SA5   DPI CLK    -          -          -
 28   GPIO 1   I2C0 SCL   SMI SA4   DPI DEN    -          -          -
 29   GPIO 5   GPCLK1     SMI SA0   DPI Blu 3  -          -          JTAG TDO
 30   ground
 31   GPIO 6   GOCKJ2     SMI SOEN  DPI Blu 4  -          -          JTAG RTCK
                          SE
 32   GPIO 12  PWM0       SMI SD4   DPI Grn 4  -          -          JTAG TMS
 33   GPIO 13  PWM1       SMI SD5   DPI Grn 5  -          -          JTAG TCK
 34   ground
 35   GPIO 19  PCM FS     SMI SD11  DPI Red 5  BSCSL SCL  SPI1 MISO  PWM1
                                               SCLK
 36   GPIO 16  -          SMI SD8   DPI Red 2  UART0 CTS  SPI1 CE2   UART1 CTS
 37   GPIO 26  SD0 DAT2   -         -          SD1 DAT2   JTAG TDI   -
 38   GPIO 20  PCM DIN    SMI SD12  DPI Red 6  BSCSL MISO SPI1 MOSI  CPCLK0
 39   ground
 40   GPIO 21  PCM DOUT   SMI SD13  DPI Red 7  BSCSL CE   SPI1 SCLK  GPCLK1
 */


#if defined(RASPBERRY_PI_1)
#define RASPBERRY_PI_REGS 0x20000000
#else
#define RASPBERRY_PI_REGS 0x3F000000
#endif

#define GPIO_REG_OFFSET 0x200000

#define MAX_GPIO_PINS 27  /* chip actually has 54 */

typedef enum
{
  GPIO_INPUT,
  GPIO_OUTPUT,
  GPIO_ALT5,
  GPIO_ALT4,
  GPIO_ALT0,
  GPIO_ALT1,
  GPIO_ALT2,
  GPIO_ALT3
} gpio_mode_t;

typedef struct
{
  uint32_t gp_fsel[6];   // 00..14 function select
  uint32_t gp_res_18;    // 20     reserved
  uint32_t gp_set[2];    // 1c..20 pin output set
  uint32_t gp_res_24;    // 24     reserved
  uint32_t gp_clr[2];    // 28..2c pin output clear
  uint32_t gp_res_30;    // 30     reserved
  uint32_t gp_lev[2];    // 34..38 pin level
  uint32_t gp_res_3c;    // 3c     reserved
  uint32_t gp_eds[2];    // 40..44 pin event detect status
  uint32_t gp_res_48;    // 48     reserved
  uint32_t gp_ren[2];    // 4c..50 pin rising edge detect enable
  uint32_t gp_res_54;    // 54     reserved
  uint32_t gp_fen[2];    // 58..5c pin falling edge detect enable
  uint32_t gp_res_60;    // 60     reserved
  uint32_t gp_hen[2];    // 64..68 pin high detect enable
  uint32_t gp_res_6c;    // 6c     reserved
  uint32_t gp_len[2];    // 70..74 pin low detect enable
  uint32_t gp_res_78;    // 78     reserved
  uint32_t gp_aren[2];   // 7c..80 pin async risign edge detect
  uint32_t gp_res_84;    // 84     reserved
  uint32_t gp_afen[2];   // 88..8c pin async risign edge detect
  uint32_t gp_res_90;    // 90     reserved
  uint32_t gp_pud;       // 94     pin pull-up/down enable
  uint32_t gp_pudclk[2]; // 98..9c pin pull-up/down enable clock 0
  uint32_t gp_res_a0[4]; // a0..ac reserved
  uint32_t gp_test;      // b0     test
} gpio_regs_t;


gpio_regs_t *map_gpio(void);

void gpio_set_mode(volatile gpio_regs_t *gpio, int io_bit, gpio_mode_t mode);

void gpio_bit_set(volatile gpio_regs_t *gpio, int io_bit, bool value);
