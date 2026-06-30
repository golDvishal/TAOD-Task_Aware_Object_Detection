//============================================================================
// taod_cov.sv
//----------------------------------------------------------------------------
// Functional-coverage model for the TAOD accelerator (EVIOS, DVCon India 2026).
//
// TOOL-PORTABLE, same pattern as taod_sva.sv:
//   * Full simulators (Vivado xsim / VCS / Questa): real SystemVerilog
//     `covergroup`s, attached to taod_top via `bind`.
//   * Icarus Verilog (`__ICARUS__`), which supports neither covergroups nor
//     `bind`: a lightweight manual hit-tally with an end-of-test report,
//     instantiated by the testbench.
//
// The model captures the functional intent of the accelerator -- every engine
// FSM state, the task/object-count space, the result-valid outcomes, the
// area-term enable, the decision-margin classes, the interrupt path, and the
// temporal-memory clear -- so a coverage report reads as a feature checklist.
//============================================================================
`ifndef TAOD_COV_SV
`define TAOD_COV_SV

module taod_cov
  import taod_pkg::*;
(
  input logic              clk,
  input logic              rst_n,
  input logic [3:0]        eng_state,        // taod_engine FSM state
  input logic              eng_start,
  input logic              eng_clear,
  input logic              eng_busy,
  input logic              eng_done,
  input logic              eng_result_valid,
  input logic [TASK_W-1:0] task_id,
  input logic [CNT_W-1:0]  obj_count,
  input logic [31:0]       inv_fa,
  input logic [31:0]       res_runner,
  input logic              irq
);

`ifdef USE_COVERGROUPS
  //==========================================================================
  // REAL FUNCTIONAL COVERAGE  (Questa / VCS / newer Vivado -- opt in with
  //   +define+USE_COVERGROUPS; xsim 2018.2 does NOT support covergroups, so
  //   the manual tally below is the portable default.)
  //==========================================================================
  covergroup cg_state @(posedge clk);
    option.per_instance = 1;
    cp_state: coverpoint eng_state iff (rst_n) {
      bins idle  = {0};
      bins clr   = {1};
      bins fetch = {2};
      bins objw  = {3};
      bins memw  = {4};
      bins c1    = {5};
      bins c2    = {6};
      bins arg   = {7};
      bins done  = {8};
    }
  endgroup

  covergroup cg_frame @(posedge clk iff (eng_done && rst_n));
    option.per_instance = 1;
    cp_task: coverpoint task_id {
      bins low[]  = {[0:7]};
      bins high[] = {[8:15]};
    }
    cp_cnt: coverpoint obj_count {
      bins empty = {0};
      bins one   = {1};
      bins few   = {[2:8]};
      bins many  = {[9:64]};
    }
    cp_valid: coverpoint eng_result_valid { bins novalid = {0}; bins valid = {1}; }
    cp_area:  coverpoint (inv_fa != 32'd0) { bins disabled = {0}; bins enabled = {1}; }
    cp_margin: coverpoint (eng_result_valid && (res_runner != 32'd0)) {
      bins single   = {0};   // lone relevant object (runner-up 0)
      bins contested = {1};  // >=2 relevant objects (positive margin)
    }
    x_task_area: cross cp_task, cp_area;
  endgroup

  covergroup cg_event @(posedge clk iff (rst_n));
    option.per_instance = 1;
    cp_clear: coverpoint eng_clear { bins seen = {1}; }
    cp_irq:   coverpoint irq       { bins low = {0}; bins high = {1}; }
  endgroup

  cg_state u_cg_state = new();
  cg_frame u_cg_frame = new();
  cg_event u_cg_event = new();

  task automatic cov_report; begin
    $display("[COV] state=%0.1f%% frame=%0.1f%% event=%0.1f%% (overall %0.1f%%)",
             u_cg_state.get_inst_coverage(), u_cg_frame.get_inst_coverage(),
             u_cg_event.get_inst_coverage(),
             (u_cg_state.get_inst_coverage()+u_cg_frame.get_inst_coverage()
              +u_cg_event.get_inst_coverage())/3.0);
  end endtask

`else
  //==========================================================================
  // MANUAL HIT-TALLY  (portable default -- Icarus, xsim 2018.2, any simulator)
  //   One bit per coverage bin; set when the bin is exercised.  A genuine
  //   coverage HOLE (e.g. the empty-frame path, never stimulated by the
  //   5-frame suite) is reported honestly rather than hidden.
  //==========================================================================
  // FSM state bins (9)
  logic [8:0] c_state;
  // frame bins
  logic       c_task_lo, c_task_hi;
  logic       c_cnt_empty, c_cnt_one, c_cnt_small, c_cnt_many;
  logic       c_valid0, c_valid1;
  logic       c_area_off, c_area_on;
  logic       c_margin_single, c_margin_contested;
  // event bins
  logic       c_clear, c_irq_hi, c_irq_lo, c_irq_fall;
  logic       irq_q;

  initial begin
    c_state = '0;
    c_task_lo=0; c_task_hi=0;
    c_cnt_empty=0; c_cnt_one=0; c_cnt_small=0; c_cnt_many=0;
    c_valid0=0; c_valid1=0; c_area_off=0; c_area_on=0;
    c_margin_single=0; c_margin_contested=0;
    c_clear=0; c_irq_hi=0; c_irq_lo=0; c_irq_fall=0; irq_q=0;
  end

  always @(posedge clk) if (rst_n) begin
    // ---- FSM states ----
    if (!$isunknown(eng_state) && eng_state <= 4'd8)
      c_state[eng_state] <= 1'b1;

    // ---- per-frame bins, sampled at done ----
    if (eng_done) begin
      if (task_id <= 4'd7) c_task_lo <= 1'b1; else c_task_hi <= 1'b1;
      if      (obj_count == 7'd0)  c_cnt_empty <= 1'b1;
      else if (obj_count == 7'd1)  c_cnt_one   <= 1'b1;
      else if (obj_count <= 7'd8)  c_cnt_small <= 1'b1;
      else                         c_cnt_many  <= 1'b1;
      if (eng_result_valid) c_valid1 <= 1'b1; else c_valid0 <= 1'b1;
      if (eng_result_valid && (res_runner != 32'd0)) c_margin_contested <= 1'b1;
      else if (eng_result_valid)                     c_margin_single    <= 1'b1;
    end

    // ---- area-term enable, sampled at launch (config stable) ----
    if (eng_start) begin
      if (inv_fa != 32'd0) c_area_on <= 1'b1; else c_area_off <= 1'b1;
    end

    // ---- events ----
    if (eng_clear) c_clear <= 1'b1;
    if (irq)  c_irq_hi <= 1'b1; else c_irq_lo <= 1'b1;
    if (irq_q && !irq) c_irq_fall <= 1'b1;     // W1C clear observed
    irq_q <= irq;
  end

  // bin count helpers
  function automatic int popc9(input logic [8:0] v);
    int i, s; begin s = 0; for (i=0;i<9;i++) s += v[i]; popc9 = s; end
  endfunction

  task automatic cov_report;
    int hit, tot;
    begin
      hit = popc9(c_state)
          + c_task_lo + c_task_hi
          + c_cnt_empty + c_cnt_one + c_cnt_small + c_cnt_many
          + c_valid0 + c_valid1 + c_area_off + c_area_on
          + c_margin_single + c_margin_contested
          + c_clear + c_irq_hi + c_irq_lo + c_irq_fall;
      tot = 9 + 2 + 4 + 2 + 2 + 2 + 4;          // = 25 bins
      $display("------------------------------------------------------------------");
      $display(" FUNCTIONAL COVERAGE (manual tally)");
      $display("   FSM states      : %0d / 9   %s", popc9(c_state),
               (popc9(c_state)==9)?"(all)":"");
      $display("   task lo / hi    : %0d / %0d", c_task_lo, c_task_hi);
      $display("   objcnt 0/1/sm/mny: %0d %0d %0d %0d",
               c_cnt_empty, c_cnt_one, c_cnt_small, c_cnt_many);
      $display("   result_valid 0/1: %0d / %0d", c_valid0, c_valid1);
      $display("   area off / on   : %0d / %0d", c_area_off, c_area_on);
      $display("   margin sgl / con: %0d / %0d", c_margin_single, c_margin_contested);
      $display("   clear / irqHi/Lo/fall: %0d %0d %0d %0d",
               c_clear, c_irq_hi, c_irq_lo, c_irq_fall);
      $display("   OVERALL         : %0d / %0d bins  (%0.1f%%)",
               hit, tot, 100.0*hit/tot);
      if (!c_cnt_empty)
        $display("   NOTE: empty-frame (obj_count==0) path not stimulated -- known hole");
      $display("------------------------------------------------------------------");
    end
  endtask
`endif

endmodule

`endif
