// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

#include "simple_system_common.h"

int main(int argc, char **argv) {
  int val;

  // write memory encryption key to register
  val = 0x445566ff;
  asm("csrw 0x7C0, %[x]" : :[x] "r" (val));

  // Set memory partition permissions: execute, write,read and encrypted.
  val = 0b00101111;
  asm("csrw 0x3A0, %[x]" : :[x] "r" (val));
  // val = SIM_CTRL_BASE * 4;
  val = SIM_CTRL_BASE * 16;
  asm("csrw 0x3B0, %[x]" : :[x] "r" (val));

  // try to read and write
  DEV_WRITE(0x100010, 'c');
  val = DEV_READ(0x100010, 0);

 // try to write the memory_encryption_key again to the register. should fail to update the value.
  val = 0xdeadbeef;
  asm("csrw 0x7c0, %[x]" : :[x] "r" (val));

  // Read-only enabled (lock the memory section)
  val = 0b10101101;
  asm("csrw 0x3A0, %[x]" : :[x] "r" (val));

  // try to write to a locked and read-only memory section. Should raise irq.
  DEV_WRITE(0x100014, 'k');
  val = DEV_READ(0x100014, 0);

  return 0;
}
