// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

#include <stdint.h>
#include "simple_system_common.h"
#define CLK_FIXED_FREQ_HZ (50ULL * 1000 * 1000)
#include "strings.h"
#include "core_portme.h"

/**
 * Delay loop executing within 8 cycles on ibex
 */
static void delay_loop_ibex(unsigned long loops) {
  int out; /* only to notify compiler of modifications to |loops| */
  asm volatile(
      "1: nop             \n" // 1 cycle
      "   nop             \n" // 1 cycle
      "   nop             \n" // 1 cycle
      "   nop             \n" // 1 cycle
      "   addi %1, %1, -1 \n" // 1 cycle
      "   bnez %1, 1b     \n" // 3 cycles
      : "=&r" (out)
      : "0" (loops)
  );
}

static int usleep_ibex(unsigned long usec) {
  unsigned long usec_cycles;
  usec_cycles = CLK_FIXED_FREQ_HZ * usec / 1000 / 1000 / 8;

  delay_loop_ibex(usec_cycles);
  return 0;
}

static int usleep(unsigned long usec) {
  return usleep_ibex(usec);
}

int count_to_3 (int * counter_p) {
  // double number = 1.0;
  // ee_printf("number:%d\n", (int)number);
  ee_printf("counter value:%d\n",*counter_p);
  if(*counter_p == 3) {
    return 0;
  } else {
    (*counter_p)++;
    count_to_3(counter_p);
    return 0;
  }
  return 1;
}

int main(int argc, char **argv) {
  // The lowest four bits of the highest byte written to the memory region named
  // "stack" are connected to the LEDs of the board.
  volatile uint8_t *var = (volatile uint8_t *) 0x0000F000;
  int counter_p = 1;
  *var = 0x0a;
  // puthex(counter_p);
  puts("1");
  putchar(2);
  if(count_to_3(&counter_p) != 0) {
    // ee_printf("Failed\n");
    putchar(10);
  } else
  {
    // ee_printf("Success!\n");
    putchar(9);
  }
  
  while (1) {
    usleep(1000 * 100); // 1000 ms
    *var = ~(*var);
  }
}