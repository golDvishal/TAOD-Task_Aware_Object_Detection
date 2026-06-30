//============================================================================
// taod_sva.sv
//----------------------------------------------------------------------------
// Assertion-based checker for the TAOD accelerator (EVIOS, DVCon India 2026).
//
// This file is TOOL-PORTABLE by design:
//
//   * Under a full SystemVerilog simulator (Vivado xsim, VCS, Questa, ...),
//     the checker uses CONCURRENT SVA (`assert property`) and is attached to
//     taod_top automatically via a `bind` (no testbench edit needed).
//
//   * Under Icarus Verilog (`__ICARUS__`), which supports neither concurrent
//     assertions nor `bind`, the same checks are expressed as clocked
//     IMMEDIATE assertions, and the checker is instantiated explicitly by the
//     testbench (see the `ifdef __ICARUS__` block in taod_tb.sv).
//
// The properties are a mix of control-flow invariants, AXI4 slave response
// hygiene, and -- most importantly -- the FUNCTIONAL decision-margin invariant
// that the runner-up score can never exceed the winning score.
//============================================================================
`ifndef TAOD_SVA_SV
`define TAOD_SVA_SV

module taod_sva
  import taod_pkg::*;
(
  input logic                clk,
  input logic                rst_n,
  // engine control / status
  input logic                eng_start,
  input logic                eng_busy,
  input logic                eng_done,
  input logic                eng_result_valid,
  input logic                done_sticky,
  // engine results (decision margin)
  input logic [31:0]         res_score,
  input logic [31:0]         res_runner,
  // engine memory addresses (range bounds)
  input logic [WMEM_AW-1:0]  w_addr,
  input logic [IDX_W-1:0]    obj_addr,
  // interrupt
  input logic                irq,
  input logic                irq_pending,
  input logic                irq_en,
  // AXI4 slave response channels
  input logic                s_bvalid,
  input logic [1:0]          s_bresp,
  input logic                s_rvalid,
  input logic [1:0]          s_rresp,
  input logic                s_rlast
);

  // running failure tally (read by the TB summary under Icarus)
  integer sva_fail = 0;

  initial $display("[%0t] SVA: TAOD assertion checker active", $time);

`ifndef __ICARUS__
  //==========================================================================
  // CONCURRENT SVA  (Vivado / VCS / Questa)
  //==========================================================================

  // A. decision-margin invariant: runner-up never beats the winner
  a_margin: assert property (@(posedge clk) disable iff (!rst_n)
    eng_result_valid |-> (res_runner <= res_score))
    else $error("SVA A: runner-up %0d > winner %0d", res_runner, res_score);

  // B. no valid detection => winning score must be zero (checked at done)
  a_novalid_zero: assert property (@(posedge clk) disable iff (!rst_n)
    (eng_done && !eng_result_valid) |-> (res_score == 32'd0))
    else $error("SVA B: no-detect but score=%0d", res_score);

  // C/D. engine memory addresses always in range *while the engine is active*
  // (compare against the int params directly; obj_addr/w_addr are zero-extended
  // -- do NOT cast the bound down to the address width, that truncates 64->0).
  a_waddr_range: assert property (@(posedge clk) disable iff (!rst_n)
    eng_busy |-> (w_addr  < WMEM_DEPTH))
    else $error("SVA C: w_addr %0d out of range", w_addr);
  a_oaddr_range: assert property (@(posedge clk) disable iff (!rst_n)
    eng_busy |-> (obj_addr < MAX_OBJ))
    else $error("SVA D: obj_addr %0d out of range", obj_addr);

  // E. done is a single-cycle pulse
  a_done_pulse: assert property (@(posedge clk) disable iff (!rst_n) eng_done |=> !eng_done)
    else $error("SVA E: eng_done held >1 cycle");

  // F. a fresh start clears the sticky-done flag (unless done fires same cycle)
  a_start_clears: assert property (@(posedge clk) disable iff (!rst_n)
    (eng_start && !eng_done) |=> !done_sticky)
    else $error("SVA F: start did not clear done_sticky");

  // G. software must not relaunch the engine while it is busy
  a_no_start_busy: assert property (@(posedge clk) disable iff (!rst_n) !(eng_start && eng_busy))
    else $error("SVA G: start asserted while engine busy");

  // H. interrupt line is exactly pending AND enabled
  a_irq_def: assert property (@(posedge clk) disable iff (!rst_n) irq == (irq_pending && irq_en))
    else $error("SVA H: irq != pending&enable");

  // I. AXI4 slave only ever answers OKAY
  a_bresp_ok: assert property (@(posedge clk) disable iff (!rst_n) s_bvalid |-> (s_bresp == 2'b00))
    else $error("SVA I1: bresp != OKAY");
  a_rresp_ok: assert property (@(posedge clk) disable iff (!rst_n) s_rvalid |-> (s_rresp == 2'b00))
    else $error("SVA I2: rresp != OKAY");

  // (No "one beat per burst" assertion: the slave legitimately supports INCR
  //  bursts -- multi-beat RLAST placement is checked directly in the B1 burst
  //  test in taod_tb.sv.)

`else
  //==========================================================================
  // IMMEDIATE-ASSERTION EQUIVALENTS  (Icarus Verilog)
  //   Same intent, expressed as clocked checks.  A 1-cycle history is kept
  //   for the "pulse" / "start-clears" properties.
  //==========================================================================
  logic eng_done_q, eng_start_q;
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin eng_done_q <= 1'b0; eng_start_q <= 1'b0; end
    else        begin eng_done_q <= eng_done; eng_start_q <= eng_start; end
  end

  // verilator lint_off BLKSEQ
  `define CHK(cond, msg) \
    if (!(cond)) begin sva_fail = sva_fail + 1; \
      $error("%s (t=%0t)", msg, $time); end

  always_ff @(posedge clk) if (rst_n) begin
    // A. decision-margin invariant
    if (eng_result_valid)               `CHK(res_runner <= res_score, "SVA A: runner>winner")
    // B. no detect => zero score
    if (eng_done && !eng_result_valid)  `CHK(res_score == 32'd0,       "SVA B: no-detect nonzero score")
    // C/D. address range (only meaningful while the engine is active)
    if (eng_busy)                       `CHK(w_addr  < WMEM_DEPTH, "SVA C: w_addr range")
    if (eng_busy)                       `CHK(obj_addr < MAX_OBJ,   "SVA D: obj_addr range")
    // E. done pulse
    if (eng_done_q)                     `CHK(!eng_done,                "SVA E: done held")
    // F. start clears sticky-done
    if (eng_start_q && !eng_done_q)     `CHK(!done_sticky,             "SVA F: start !clear done")
    // G. no relaunch while busy
    `CHK(!(eng_start && eng_busy),      "SVA G: start while busy")
    // H. irq definition
    `CHK(irq == (irq_pending && irq_en),"SVA H: irq mismatch")
    // I. AXI OKAY responses
    if (s_bvalid)                       `CHK(s_bresp == 2'b00,         "SVA I1: bresp!=OKAY")
    if (s_rvalid)                       `CHK(s_rresp == 2'b00,         "SVA I2: rresp!=OKAY")
  end

  `undef CHK
  // verilator lint_on BLKSEQ
`endif

endmodule

`endif
