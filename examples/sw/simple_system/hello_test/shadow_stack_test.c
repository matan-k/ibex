// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

#include "simple_system_common.h"

void function_b(int c) {
  c = c * 2 + 1;

  // not taken
  if(c == 0) {
    puts("function_b: taken\n");
  } else {
    return;
  }
  return;
}

void malicious_func(int c) {
  puts("entered malicious_func\n");
  asm volatile ("addi x1, x1, 4 " : :);
  asm volatile ("c.jr x1" : :);
  return;
}

int function_a(int a,int b) {
  int c = 0;
  puts("a:\n");
  puthex(a);
  putchar('\n');
  puts("b:\n");
  puthex(b);
  putchar('\n');

  a = a * a;
  c = a * b;
  puthex(c);
  putchar('\n');

  if(c > 5) {
    function_b(c);
    puts("function a: taken\n");
    if(c > 6) {
      return c;
    }
  } else {
    malicious_func(c);
  }

  // Here the adversary tries to return to an illegal location
  // It simulates reading the tampered value of reg1 from the stack.
  // This should raise an irq and terminate the simulation.
  
  return c;
}

int main(int argc, char **argv) {
  int c = 0;

  c= function_a(2,7);
  puthex(c);
  putchar('\n');
  putchar('\n');

  c= function_a(0, 4);
  return 0;
}

// Test plan: go over all the following conditions, to see that the mechanism operates well even around branches.
// list:      without branch          after branch taken      after branch not taken
// jal             V                           V                        V
// jalr            V                           V                        V
