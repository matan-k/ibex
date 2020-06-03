// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

#include <fstream>
#include <iostream>

#include "ibex_pcounts.h"
#include "verilated_toplevel.h"
#include "verilator_memutil.h"
#include "verilator_sim_ctrl.h"

int main(int argc, char **argv) {
  top_cyclone10lp top;
  VerilatorMemUtil memutil;
  VerilatorSimCtrl &simctrl = VerilatorSimCtrl::GetInstance();
  simctrl.SetTop(&top, &top.IO_CLK, &top.IO_RST_N,
                 VerilatorSimCtrlFlags::ResetPolarityNegative);

  memutil.RegisterMemoryArea("ram", "TOP.top_cyclone10lp.u_ram");
  simctrl.RegisterExtension(&memutil);

  std::cout << "Simulation of Ibex" << std::endl
            << "==================" << std::endl
            << std::endl;

  if (simctrl.Exec(argc, argv)) {
    return 1;
  }

  // TODO: Exec can return with "true" (e.g. with `-h`), but that does not mean
  // `RunSimulation()` was executed. The folllowing values will not be useful
  // in this case.
  // std::cout << "\nPerformance Counters" << std::endl
  //           << "====================" << std::endl;
  // std::cout << ibex_pcount_string(top.top_cyclone10lp__DOT__mhpmcounter_vals,
  //                                 false);

  // std::ofstream pcount_csv("top_cyclone10lp_pcount.csv");
  // pcount_csv << ibex_pcount_string(
  //     top.top_cyclone10lp__DOT__mhpmcounter_vals, true);

  return 0;
}
