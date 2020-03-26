// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

#include "simple_system_common.h"

int function_a(int a,int b) {
  int c = 0;

  a = a * a;
  c = a * b;
  puthex(c);
  putchar('\n');

  // Here the adversary tries to return to an illegal location
  // It simulates reading the tampered value of reg1 from the stack.
  // This should raise an irq and terminate the simulation.
  asm volatile ("addi x1, x1, 4 " : :);
  asm volatile ("c.jr x1" : :);
  return c;
}

int main(int argc, char **argv) {
  
  int a = 2;
  int b = 7;
  int c = 0;

  c= function_a(a,b);
  puthex(c);

  return 0;
}


