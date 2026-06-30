//============================================================================
// taod_tb.sv
//----------------------------------------------------------------------------
// Self-checking testbench for the EVIOS Task-Aware Object Detection (TAOD)
// accelerator -- DVCon India 2026 Design Contest.
//
// Covers BOTH datapaths against a bit-accurate golden model:
//   * PUSH mode -- drives the DUT as the VEGA core would over the AXI4 *slave*
//     (program CSRs / weight LUT / object buffer, kick CTRL.start, poll STATUS,
//     read back RESULT/RUNNER/BBOX).  Frames F1..F8.
//   * DMA mode  -- a behavioural AXI4 memory BFM backs the *master* port; the
//     master reads object records from DDR, the engine scores them, and the
//     result is written back to DDR.  Frames D1, D2.
//
// A behavioural GOLDEN MODEL re-implements the engine's UQ2.14 datapath
// bit-for-bit -- ensemble fusion, task LUT, per-track EMA temporal stability
// (incl. cold-start), the 1+area-fraction accessibility term, the chained
// saturating multiplies and the 32-bit UQ4.28 argmax (winner + runner-up).
// Every frame is checked field-by-field against the model.
//
// Headline scenarios:
//   F1  "water the plants" : a 0.60-confidence potted plant must beat a
//        0.95 person and 0.90 chair, because only the plant is task-relevant.
//        (Naive max-confidence picks the wrong object; EVIOS does not.)
//   F2  spatial accessibility: a larger / nearer cup wins over a smaller,
//        higher-confidence cup via the area term.  IRQ path verified here.
//   F3  temporal stability: a noisy 0.30 reading on a track previously seen
//        at 0.60 is smoothed by the EMA (alpha=2) and stays selected.
//   F4  CLEAR returns the temporal memory to cold-start.
//   F5  all-invalid frame -> result_valid = 0 (no selection).
//============================================================================
`timescale 1ns/1ps

module taod_tb;
  import taod_pkg::*;

  //--------------------------------------------------------------------------
  // Register map offsets (must match taod_top.sv)
  //--------------------------------------------------------------------------
  localparam [15:0] A_CTRL   = 16'h0000;
  localparam [15:0] A_TASK   = 16'h0008;
  localparam [15:0] A_OBJCNT = 16'h0010;
  localparam [15:0] A_FUSE   = 16'h0018;
  localparam [15:0] A_INVFA  = 16'h0020;
  localparam [15:0] A_ALPHA  = 16'h0028;
  localparam [15:0] A_DMAOBJ = 16'h0030;
  localparam [15:0] A_DMARES = 16'h0038;
  localparam [15:0] A_IRQCTL = 16'h0040;
  localparam [15:0] A_VER    = 16'h00F8;
  localparam [15:0] A_STATUS = 16'h0100;
  localparam [15:0] A_RESULT = 16'h0108;
  localparam [15:0] A_RUNNER = 16'h0110;
  localparam [15:0] A_BBOX   = 16'h0118;
  localparam [15:0] LUT_OFF  = 16'h1000;
  localparam [15:0] OBJ_OFF  = 16'h4000;

  // DDR base addresses on the master port (independent address space, served
  // by the AXI memory BFM below).  Kept small so a flat array can back them.
  localparam [AXI_ADDR_W-1:0] DMA_OBJ_BASE = 32'h0000_1000;  // object records
  localparam [AXI_ADDR_W-1:0] DMA_RES_BASE = 32'h0000_2000;  // result write-back

  //--------------------------------------------------------------------------
  // COCO class indices used here
  //--------------------------------------------------------------------------
  localparam [CLASS_W-1:0] C_PERSON = 8'd0;
  localparam [CLASS_W-1:0] C_CUP    = 8'd41;
  localparam [CLASS_W-1:0] C_CHAIR  = 8'd56;
  localparam [CLASS_W-1:0] C_PLANT  = 8'd58;

  //--------------------------------------------------------------------------
  // UQ2.14 constants (raw = round(prob * 16384))
  //--------------------------------------------------------------------------
  localparam [QW-1:0] Q030 = 16'd4915;   // 0.30
  localparam [QW-1:0] Q050 = 16'd8192;   // 0.50
  localparam [QW-1:0] Q060 = 16'd9830;   // 0.60
  localparam [QW-1:0] Q065 = 16'd10650;  // 0.65
  localparam [QW-1:0] Q070 = 16'd11469;  // 0.70
  localparam [QW-1:0] Q090 = 16'd14746;  // 0.90
  localparam [QW-1:0] Q095 = 16'd15565;  // 0.95
  localparam [QW-1:0] Q100 = 16'd16384;  // 1.00

  // inv_fa = floor(2^32 / (640*480)) = floor(2^32 / 307200)
  localparam [31:0] INVFA_VGA = 32'd13981;

  //==========================================================================
  // Clock / reset
  //==========================================================================
  logic clk;
  logic rst_n;
  initial clk = 1'b0;
  always #5 clk = ~clk;                 // 100 MHz nominal (functional sim only)

  //==========================================================================
  // TB-driven slave stimulus
  //==========================================================================
  logic [AXI_ID_W-1:0]   s_awid;
  logic [AXI_ADDR_W-1:0] s_awaddr;
  logic [7:0]            s_awlen;
  logic [2:0]            s_awsize;
  logic [1:0]            s_awburst;
  logic                  s_awvalid;
  logic [AXI_DATA_W-1:0] s_wdata;
  logic [AXI_STRB_W-1:0] s_wstrb;
  logic                  s_wlast;
  logic                  s_wvalid;
  logic                  s_bready;
  logic [AXI_ID_W-1:0]   s_arid;
  logic [AXI_ADDR_W-1:0] s_araddr;
  logic [7:0]            s_arlen;
  logic [2:0]            s_arsize;
  logic [1:0]            s_arburst;
  logic                  s_arvalid;
  logic                  s_rready;

  // DUT-driven slave responses
  wire                   s_awready, s_wready, s_bvalid;
  wire [AXI_ID_W-1:0]    s_bid;
  wire [1:0]             s_bresp;
  wire                   s_arready, s_rvalid, s_rlast;
  wire [AXI_ID_W-1:0]    s_rid;
  wire [AXI_DATA_W-1:0]  s_rdata;
  wire [1:0]             s_rresp;

  //==========================================================================
  // Master side: inputs (m_*) are driven by the AXI4 memory BFM further below
  //==========================================================================
  logic                  m_arready, m_rvalid, m_rlast;
  logic [AXI_ID_W-1:0]   m_rid;
  logic [AXI_DATA_W-1:0] m_rdata;
  logic [1:0]            m_rresp;
  logic                  m_awready, m_wready, m_bvalid;
  logic [AXI_ID_W-1:0]   m_bid;
  logic [1:0]            m_bresp;

  wire [AXI_ID_W-1:0]    m_arid, m_awid;
  wire [AXI_ADDR_W-1:0]  m_araddr, m_awaddr;
  wire [7:0]             m_arlen, m_awlen;
  wire [2:0]             m_arsize, m_awsize;
  wire [1:0]             m_arburst, m_awburst;
  wire                   m_arvalid, m_awvalid, m_rready, m_wvalid, m_wlast, m_bready;
  wire [AXI_DATA_W-1:0]  m_wdata;
  wire [AXI_STRB_W-1:0]  m_wstrb;

  wire                   irq;

  //==========================================================================
  // DUT
  //==========================================================================
  taod_top u_dut (
    .clk(clk), .rst_n(rst_n),
    .s_awid(s_awid), .s_awaddr(s_awaddr), .s_awlen(s_awlen), .s_awsize(s_awsize),
    .s_awburst(s_awburst), .s_awvalid(s_awvalid), .s_awready(s_awready),
    .s_wdata(s_wdata), .s_wstrb(s_wstrb), .s_wlast(s_wlast), .s_wvalid(s_wvalid),
    .s_wready(s_wready),
    .s_bid(s_bid), .s_bresp(s_bresp), .s_bvalid(s_bvalid), .s_bready(s_bready),
    .s_arid(s_arid), .s_araddr(s_araddr), .s_arlen(s_arlen), .s_arsize(s_arsize),
    .s_arburst(s_arburst), .s_arvalid(s_arvalid), .s_arready(s_arready),
    .s_rid(s_rid), .s_rdata(s_rdata), .s_rresp(s_rresp), .s_rlast(s_rlast),
    .s_rvalid(s_rvalid), .s_rready(s_rready),
    .m_arid(m_arid), .m_araddr(m_araddr), .m_arlen(m_arlen), .m_arsize(m_arsize),
    .m_arburst(m_arburst), .m_arvalid(m_arvalid), .m_arready(m_arready),
    .m_rid(m_rid), .m_rdata(m_rdata), .m_rresp(m_rresp), .m_rlast(m_rlast),
    .m_rvalid(m_rvalid), .m_rready(m_rready),
    .m_awid(m_awid), .m_awaddr(m_awaddr), .m_awlen(m_awlen), .m_awsize(m_awsize),
    .m_awburst(m_awburst), .m_awvalid(m_awvalid), .m_awready(m_awready),
    .m_wdata(m_wdata), .m_wstrb(m_wstrb), .m_wlast(m_wlast), .m_wvalid(m_wvalid),
    .m_wready(m_wready),
    .m_bid(m_bid), .m_bresp(m_bresp), .m_bvalid(m_bvalid), .m_bready(m_bready),
    .irq(irq)
  );

  // Assertion + coverage checkers are instantiated directly here so the setup
  // is portable across all simulators (Icarus and full tools alike).  They read
  // DUT internals via hierarchical references (u_dut.*); no `bind` required.
  taod_sva u_sva (
    .clk              (clk),
    .rst_n            (rst_n),
    .eng_start        (u_dut.eng_start),
    .eng_busy         (u_dut.eng_busy),
    .eng_done         (u_dut.eng_done),
    .eng_result_valid (u_dut.eng_result_valid),
    .done_sticky      (u_dut.done_sticky),
    .res_score        (u_dut.res_score),
    .res_runner       (u_dut.res_runner),
    .w_addr           (u_dut.w_addr),
    .obj_addr         (u_dut.obj_addr),
    .irq              (u_dut.irq),
    .irq_pending      (u_dut.irq_pending),
    .irq_en           (u_dut.irq_en),
    .s_bvalid         (s_bvalid),
    .s_bresp          (s_bresp),
    .s_rvalid         (s_rvalid),
    .s_rresp          (s_rresp),
    .s_rlast          (s_rlast)
  );

  taod_cov u_cov (
    .clk              (clk),
    .rst_n            (rst_n),
    .eng_state        (u_dut.u_eng.st),
    .eng_start        (u_dut.eng_start),
    .eng_clear        (u_dut.eng_clear),
    .eng_busy         (u_dut.eng_busy),
    .eng_done         (u_dut.eng_done),
    .eng_result_valid (u_dut.eng_result_valid),
    .task_id          (u_dut.task_id),
    .obj_count        (u_dut.obj_count),
    .inv_fa           (u_dut.inv_fa),
    .res_runner       (u_dut.res_runner),
    .irq              (u_dut.irq)
  );

  //==========================================================================
  // AXI4 memory BFM on the master port
  //   Behavioural DDR model: serves the master's INCR read burst (object
  //   records) and accepts its INCR write burst (result write-back).  Backed
  //   by a small flat array (8-byte words) since the DMA bases are kept low.
  //   8 bytes/beat (SIZE_8B); word index = byte_addr[13:3].
  //==========================================================================
  logic [63:0] ddr [0:2047];

  // ---- read channel responder ----
  localparam logic R_IDLE = 1'b0, R_RUN = 1'b1;
  logic        r_state;
  logic [10:0] r_waddr;
  logic [8:0]  r_remain;          // beats still to send (incl. current)

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      r_state  <= R_IDLE;
      m_arready <= 1'b0;
      m_rvalid <= 1'b0; m_rlast <= 1'b0; m_rid <= '0; m_rresp <= 2'b00; m_rdata <= '0;
      r_waddr  <= '0; r_remain <= '0;
    end else begin
      case (r_state)
        R_IDLE: begin
          m_arready <= 1'b1;
          m_rvalid  <= 1'b0;
          m_rlast   <= 1'b0;
          if (m_arvalid && m_arready) begin
            m_arready <= 1'b0;
            r_waddr   <= m_araddr[13:3];
            r_remain  <= {1'b0, m_arlen} + 9'd1;     // beats = arlen+1
            m_rdata   <= ddr[m_araddr[13:3]];        // first beat
            m_rid     <= '0;
            m_rresp   <= 2'b00;
            m_rlast   <= (m_arlen == 8'd0);
            m_rvalid  <= 1'b1;
            r_state   <= R_RUN;
          end
        end
        R_RUN: begin
          if (m_rvalid && m_rready) begin
            if (r_remain == 9'd1) begin              // last beat just taken
              m_rvalid  <= 1'b0;
              m_rlast   <= 1'b0;
              m_arready <= 1'b1;
              r_state   <= R_IDLE;
            end else begin
              r_waddr  <= r_waddr + 11'd1;
              r_remain <= r_remain - 9'd1;
              m_rdata  <= ddr[r_waddr + 11'd1];
              m_rlast  <= (r_remain == 9'd2);         // next beat is last
            end
          end
        end
        default: r_state <= R_IDLE;
      endcase
    end
  end

  // ---- write channel responder ----
  localparam logic [1:0] W_IDLE = 2'd0, W_DATA = 2'd1, W_RESP = 2'd2;
  logic [1:0]  w_state;
  logic [10:0] w_waddr;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      w_state  <= W_IDLE;
      m_awready <= 1'b0; m_wready <= 1'b0;
      m_bvalid <= 1'b0; m_bid <= '0; m_bresp <= 2'b00;
      w_waddr  <= '0;
    end else begin
      case (w_state)
        W_IDLE: begin
          m_awready <= 1'b1;
          m_bvalid  <= 1'b0;
          m_wready  <= 1'b0;
          if (m_awvalid && m_awready) begin
            m_awready <= 1'b0;
            w_waddr   <= m_awaddr[13:3];
            m_wready  <= 1'b1;
            w_state   <= W_DATA;
          end
        end
        W_DATA: begin
          m_wready <= 1'b1;
          if (m_wvalid && m_wready) begin
            ddr[w_waddr] <= m_wdata;
            w_waddr      <= w_waddr + 11'd1;
            if (m_wlast) begin
              m_wready <= 1'b0;
              m_bvalid <= 1'b1; m_bresp <= 2'b00; m_bid <= '0;
              w_state  <= W_RESP;
            end
          end
        end
        W_RESP: begin
          if (m_bvalid && m_bready) begin
            m_bvalid  <= 1'b0;
            m_awready <= 1'b1;
            w_state   <= W_IDLE;
          end
        end
        default: w_state <= W_IDLE;
      endcase
    end
  end

  logic [QW-1:0] g_lut  [0:WMEM_DEPTH-1];   // mirror of weight LUT
  logic [QW-1:0] g_tval [0:NUM_TRACK-1];    // mirror of temporal value
  logic          g_tvld [0:NUM_TRACK-1];    // mirror of temporal valid bit

  // current-frame object list
  logic [COORD_W-1:0] f_x1 [0:MAX_OBJ-1], f_y1 [0:MAX_OBJ-1];
  logic [COORD_W-1:0] f_x2 [0:MAX_OBJ-1], f_y2 [0:MAX_OBJ-1];
  logic [QW-1:0]      f_c1 [0:MAX_OBJ-1], f_c2 [0:MAX_OBJ-1];
  logic [CLASS_W-1:0] f_cls[0:MAX_OBJ-1];
  logic [TRK_W-1:0]   f_trk[0:MAX_OBJ-1];
  logic               f_vld[0:MAX_OBJ-1];

  // golden expected result
  logic               g_best_set;
  logic [31:0]        g_best_score, g_second;
  logic [IDX_W-1:0]   g_best_idx;
  logic [CLASS_W-1:0] g_best_cls;
  logic [COORD_W-1:0] g_bx1, g_by1, g_bx2, g_by2;

  integer errcnt;

  //==========================================================================
  // Bit-accurate per-object scorer (side-effect: updates the EMA track mem)
  //==========================================================================
  function automatic logic [31:0] gold_score(
      input logic [QW-1:0]      c1,  input logic [QW-1:0]      c2,
      input logic [CLASS_W-1:0] cls, input logic [TRK_W-1:0]   trk,
      input logic [COORD_W-1:0] x1,  input logic [COORD_W-1:0] y1,
      input logic [COORD_W-1:0] x2,  input logic [COORD_W-1:0] y2,
      input logic [TASK_W-1:0]  tsk,
      input logic [QW-1:0]      gw1, input logic [QW-1:0]      gw2,
      input logic [31:0]        ginv, input logic [TASK_W-1:0] galpha);
    logic [63:0]          fsum;
    logic [QW-1:0]        fconf, wtask, areaterm, aw_q, tstable, tnew, told, p1, p2;
    logic                 tvld;
    logic [COORD_W-1:0]   dx, dy;
    logic [31:0]          area;
    logic [63:0]          aw64;
    logic signed [QW+1:0] ema_diff, ema_sh, ema_new;
    logic [QW-1:0]        ema_clamped;
    begin
      // ensemble fusion -> requantize (saturate at 4.0)
      fsum  = mul16(gw1, c1) + mul16(gw2, c2);
      fconf = (fsum >= (64'd1 << (FRAC+QW))) ? Q_MAX : QW'(fsum >> FRAC);

      // task x class relevance
      wtask = g_lut[tsk*NUM_CLASS + cls];

      // accessibility: 1 + clamp(area/frameArea, 1.0)
      dx       = (x2 > x1) ? (x2 - x1) : '0;
      dy       = (y2 > y1) ? (y2 - y1) : '0;
      area     = dx * dy;
      aw64     = (area * ginv) >> 18;
      aw_q     = (aw64 > 64'(Q_ONE)) ? Q_ONE : QW'(aw64);
      areaterm = Q_ONE + aw_q;

      // temporal EMA (cold-start uses raw fused confidence)
      told  = g_tval[trk];
      tvld  = g_tvld[trk];
      ema_diff = $signed({2'b00, fconf}) - $signed({2'b00, told});
      ema_sh   = ema_diff >>> galpha;
      ema_new  = $signed({2'b00, told}) + ema_sh;
      if      (ema_new < 0)                       ema_clamped = '0;
      else if (ema_new > $signed({2'b00, Q_MAX})) ema_clamped = Q_MAX;
      else                                        ema_clamped = QW'(ema_new);
      tnew    = tvld ? ema_clamped : fconf;
      tstable = tnew;

      // write back EMA + valid
      g_tval[trk] = tnew;
      g_tvld[trk] = 1'b1;

      // chained saturating score, kept 32-bit UQ4.28
      p1 = q_sat(mul16(fconf, wtask));
      p2 = q_sat(mul16(p1, tstable));
      gold_score = mul16(p2, areaterm);
    end
  endfunction

  task automatic gold_clear;
    integer i;
    begin
      for (i = 0; i < NUM_TRACK; i++) begin
        g_tval[i] = '0;
        g_tvld[i] = 1'b0;
      end
    end
  endtask

  task automatic compute_golden(input integer n, input logic [TASK_W-1:0] tsk,
                                input logic [QW-1:0] gw1, input logic [QW-1:0] gw2,
                                input logic [31:0] ginv, input logic [TASK_W-1:0] galpha);
    integer k;
    logic [31:0] sc;
    begin
      g_best_set = 1'b0; g_best_score = '0; g_second = '0;
      g_best_idx = '0;   g_best_cls = '0;
      g_bx1 = '0; g_by1 = '0; g_bx2 = '0; g_by2 = '0;
      for (k = 0; k < n; k++) begin
        if (f_vld[k]) begin                            // invalid -> skipped, no EMA
          sc = gold_score(f_c1[k], f_c2[k], f_cls[k], f_trk[k],
                          f_x1[k], f_y1[k], f_x2[k], f_y2[k],
                          tsk, gw1, gw2, ginv, galpha);
          if (!g_best_set || (sc > g_best_score)) begin  // strict '>' keeps first on tie
            g_second     = g_best_set ? g_best_score : 32'd0;
            g_best_score = sc;
            g_best_idx   = IDX_W'(k);
            g_best_cls   = f_cls[k];
            g_bx1 = f_x1[k]; g_by1 = f_y1[k]; g_bx2 = f_x2[k]; g_by2 = f_y2[k];
            g_best_set   = 1'b1;
          end else if (sc > g_second) begin
            g_second = sc;
          end
        end
      end
    end
  endtask

  //==========================================================================
  // AXI4 single-beat BFM (write / read)
  //==========================================================================
  task automatic axi_write(input logic [AXI_ADDR_W-1:0] addr, input logic [63:0] data);
    logic dn;
    begin
      @(posedge clk);
      s_awaddr <= addr; s_awlen <= 8'd0; s_awsize <= 3'd3; s_awburst <= 2'b01;
      s_awid   <= '0;   s_awvalid <= 1'b1;
      dn = 1'b0; while (!dn) begin @(posedge clk); if (s_awready) dn = 1'b1; end
      s_awvalid <= 1'b0;
      s_wdata <= data; s_wstrb <= 8'hFF; s_wlast <= 1'b1; s_wvalid <= 1'b1;
      dn = 1'b0; while (!dn) begin @(posedge clk); if (s_wready) dn = 1'b1; end
      s_wvalid <= 1'b0; s_wlast <= 1'b0;
      s_bready <= 1'b1;
      dn = 1'b0; while (!dn) begin @(posedge clk); if (s_bvalid) dn = 1'b1; end
      s_bready <= 1'b0;
    end
  endtask

  task automatic axi_read(input logic [AXI_ADDR_W-1:0] addr, output logic [63:0] data);
    logic dn;
    begin
      @(posedge clk);
      s_araddr <= addr; s_arlen <= 8'd0; s_arsize <= 3'd3; s_arburst <= 2'b01;
      s_arid   <= '0;   s_arvalid <= 1'b1;
      dn = 1'b0; while (!dn) begin @(posedge clk); if (s_arready) dn = 1'b1; end
      s_arvalid <= 1'b0;
      s_rready  <= 1'b1;
      dn = 1'b0; while (!dn) begin @(posedge clk); if (s_rvalid) begin data = s_rdata; dn = 1'b1; end end
      s_rready  <= 1'b0;
    end
  endtask

  //==========================================================================
  // AXI4 4-beat INCR burst BFM (exercises the slave's per-beat address
  // increment + WLAST/RLAST handling, which single-beat access never hits)
  //==========================================================================
  task automatic axi_write_burst4(input logic [AXI_ADDR_W-1:0] addr,
                                  input logic [63:0] d0, input logic [63:0] d1,
                                  input logic [63:0] d2, input logic [63:0] d3);
    logic dn; integer k; logic [63:0] bd [0:3];
    begin
      bd[0]=d0; bd[1]=d1; bd[2]=d2; bd[3]=d3;
      @(posedge clk);
      s_awaddr <= addr; s_awlen <= 8'd3; s_awsize <= 3'd3; s_awburst <= 2'b01;
      s_awid <= '0; s_awvalid <= 1'b1;
      dn=1'b0; while(!dn) begin @(posedge clk); if (s_awready) dn=1'b1; end
      s_awvalid <= 1'b0;
      for (k=0;k<4;k=k+1) begin
        s_wdata <= bd[k]; s_wstrb <= 8'hFF; s_wlast <= (k==3); s_wvalid <= 1'b1;
        dn=1'b0; while(!dn) begin @(posedge clk); if (s_wready) dn=1'b1; end
      end
      s_wvalid <= 1'b0; s_wlast <= 1'b0;
      s_bready <= 1'b1;
      dn=1'b0; while(!dn) begin @(posedge clk); if (s_bvalid) dn=1'b1; end
      s_bready <= 1'b0;
    end
  endtask

  task automatic axi_read_burst4(input logic [AXI_ADDR_W-1:0] addr,
                                 output logic [63:0] o0, output logic [63:0] o1,
                                 output logic [63:0] o2, output logic [63:0] o3,
                                 output bit lastok);
    logic dn; integer k; logic [63:0] rd [0:3];
    begin
      lastok = 1'b1;
      @(posedge clk);
      s_araddr <= addr; s_arlen <= 8'd3; s_arsize <= 3'd3; s_arburst <= 2'b01;
      s_arid <= '0; s_arvalid <= 1'b1;
      dn=1'b0; while(!dn) begin @(posedge clk); if (s_arready) dn=1'b1; end
      s_arvalid <= 1'b0;
      s_rready  <= 1'b1;
      k = 0;
      while (k < 4) begin
        @(posedge clk);
        if (s_rvalid) begin
          rd[k] = s_rdata;
          if ((k == 3) && !s_rlast) lastok = 1'b0;   // RLAST must mark final beat
          if ((k <  3) &&  s_rlast) lastok = 1'b0;   // and not fire early
          k = k + 1;
        end
      end
      s_rready <= 1'b0;
      o0=rd[0]; o1=rd[1]; o2=rd[2]; o3=rd[3];
    end
  endtask


  //==========================================================================
  task automatic prog_w(input logic [TASK_W-1:0] tsk, input logic [CLASS_W-1:0] cls,
                        input logic [QW-1:0] wv);
    integer idx;
    begin
      idx = tsk*NUM_CLASS + cls;
      axi_write(LUT_OFF + idx*8, {48'd0, wv});
      g_lut[idx] = wv;                              // mirror
    end
  endtask

  task automatic push_obj(input logic [IDX_W-1:0] rec,
                          input logic [COORD_W-1:0] x1, input logic [COORD_W-1:0] y1,
                          input logic [COORD_W-1:0] x2, input logic [COORD_W-1:0] y2,
                          input logic [QW-1:0] c1, input logic [QW-1:0] c2,
                          input logic [CLASS_W-1:0] cls, input logic [TRK_W-1:0] trk,
                          input logic vld);
    logic [63:0] w0, w1w;
    begin
      w0  = {y2, x2, y1, x1};                       // word0: x1/y1/x2/y2
      w1w = {15'd0, vld, trk, cls, c2, c1};         // word1: c1/c2/class/track/valid
      axi_write(OBJ_OFF + rec*16,     w0);
      axi_write(OBJ_OFF + rec*16 + 8, w1w);
    end
  endtask

  task automatic wait_done(output bit ok);
    integer t;
    logic [63:0] s;
    begin
      ok = 1'b0;
      repeat (6) @(posedge clk);                    // let start clear done_sticky
      t = 0;
      while (!ok && t < 5000) begin
        axi_read(A_STATUS, s);
        if (s[1]) ok = 1'b1;                         // STATUS.done (sticky)
        t = t + 1;
      end
    end
  endtask

  task automatic wait_dma(output bit ok);
    integer t;
    logic [63:0] s;
    begin
      ok = 1'b0;
      repeat (6) @(posedge clk);                    // let dma_start clear sticky
      t = 0;
      while (!ok && t < 5000) begin
        axi_read(A_STATUS, s);
        if (s[5]) ok = 1'b1;                         // STATUS.dma_done (sticky)
        t = t + 1;
      end
    end
  endtask

  task automatic do_clear;
    begin
      axi_write(A_CTRL, 64'h2);                      // CTRL.clear
      repeat (300) @(posedge clk);                   // 256-deep sweep + margin
      gold_clear();
    end
  endtask

  task automatic chk(input string nm, input string fld,
                     input logic [63:0] gotv, input logic [63:0] expv);
    begin
      if (gotv !== expv) begin
        $display("  [FAIL] %s / %s : got=%0d (0x%0h)  exp=%0d (0x%0h)",
                 nm, fld, gotv, gotv, expv, expv);
        errcnt = errcnt + 1;
      end
    end
  endtask

  //==========================================================================
  // Run one frame: program, push, kick, wait, read back, self-check
  //==========================================================================
  task automatic do_frame(input string nm, input integer n, input logic [TASK_W-1:0] tsk,
                          input logic [QW-1:0] cw1, input logic [QW-1:0] cw2,
                          input logic [31:0] cinv, input logic [TASK_W-1:0] cal);
    integer k;
    bit ok;
    logic [63:0] r_res, r_run, r_bb, r_st;
    logic [IDX_W-1:0]   got_idx;
    logic [CLASS_W-1:0] got_cls;
    logic [31:0]        got_score, got_run;
    logic               got_rv;
    logic [COORD_W-1:0] got_x1, got_y1, got_x2, got_y2;
    begin
      // ---- program config ----
      axi_write(A_OBJCNT, {57'd0, CNT_W'(n)});
      axi_write(A_TASK,   {60'd0, tsk});
      axi_write(A_FUSE,   {32'd0, cw2, cw1});
      axi_write(A_INVFA,  {32'd0, cinv});
      axi_write(A_ALPHA,  {60'd0, cal});
      // ---- push objects ----
      for (k = 0; k < n; k++)
        push_obj(IDX_W'(k), f_x1[k], f_y1[k], f_x2[k], f_y2[k],
                 f_c1[k], f_c2[k], f_cls[k], f_trk[k], f_vld[k]);
      // ---- golden ----
      compute_golden(n, tsk, cw1, cw2, cinv, cal);
      // ---- launch + wait ----
      axi_write(A_CTRL, 64'h1);                       // CTRL.start (PUSH)
      wait_done(ok);
      if (!ok) begin
        $display("  [FAIL] %s : TIMEOUT waiting for done", nm);
        errcnt = errcnt + 1;
      end
      // ---- read back ----
      axi_read(A_RESULT, r_res);
      axi_read(A_RUNNER, r_run);
      axi_read(A_BBOX,   r_bb);
      axi_read(A_STATUS, r_st);
      got_idx   = r_res[5:0];
      got_cls   = r_res[15:8];
      got_score = r_res[63:32];
      got_run   = r_run[31:0];
      got_x1    = r_bb[15:0];  got_y1 = r_bb[31:16];
      got_x2    = r_bb[47:32]; got_y2 = r_bb[63:48];
      got_rv    = r_st[2];
      // ---- self-check ----
      chk(nm, "result_valid", {63'd0, got_rv}, {63'd0, g_best_set});
      if (g_best_set) begin
        chk(nm, "winner idx",   {58'd0, got_idx},  {58'd0, g_best_idx});
        chk(nm, "winner class", {56'd0, got_cls},  {56'd0, g_best_cls});
        chk(nm, "winner score", {32'd0, got_score},{32'd0, g_best_score});
        chk(nm, "runner score", {32'd0, got_run},  {32'd0, g_second});
        chk(nm, "bbox x1", {48'd0, got_x1}, {48'd0, g_bx1});
        chk(nm, "bbox y1", {48'd0, got_y1}, {48'd0, g_by1});
        chk(nm, "bbox x2", {48'd0, got_x2}, {48'd0, g_bx2});
        chk(nm, "bbox y2", {48'd0, got_y2}, {48'd0, g_by2});
      end else begin
        chk(nm, "winner score(=0)", {32'd0, got_score}, 64'd0);
      end
      $display("[%0t] %s | task=%0d n=%0d -> winner idx=%0d class=%0d score=%0d (0x%08x) runner=%0d valid=%0b",
               $time, nm, tsk, n, got_idx, got_cls, got_score, got_score, got_run, got_rv);
    end
  endtask

  //==========================================================================
  // Run one frame via the DMA path: preload object records into DDR, kick the
  // master (CTRL.dma_start), wait for dma_done, then verify BOTH the engine
  // result CSRs AND the result words the master wrote back to DDR.
  //==========================================================================
  task automatic do_dma_frame(input string nm, input integer n, input logic [TASK_W-1:0] tsk,
                              input logic [QW-1:0] cw1, input logic [QW-1:0] cw2,
                              input logic [31:0] cinv, input logic [TASK_W-1:0] cal);
    integer k;
    bit ok;
    logic [63:0] r_res, r_run, r_bb, r_st, mw0, mw1;
    logic [10:0] ob, rb;
    logic [IDX_W-1:0]   got_idx;
    logic [CLASS_W-1:0] got_cls;
    logic [31:0]        got_score, got_run;
    logic               got_rv;
    logic [COORD_W-1:0] got_x1, got_y1, got_x2, got_y2;
    begin
      // ---- engine config (same CSRs as PUSH) ----
      axi_write(A_OBJCNT, {57'd0, CNT_W'(n)});
      axi_write(A_TASK,   {60'd0, tsk});
      axi_write(A_FUSE,   {32'd0, cw2, cw1});
      axi_write(A_INVFA,  {32'd0, cinv});
      axi_write(A_ALPHA,  {60'd0, cal});
      // ---- DMA descriptor ----
      axi_write(A_DMAOBJ, {32'd0, DMA_OBJ_BASE});
      axi_write(A_DMARES, {32'd0, DMA_RES_BASE});
      // ---- preload object records into DDR (same packing as push_obj) ----
      ob = DMA_OBJ_BASE[13:3];
      for (k = 0; k < n; k++) begin
        ddr[ob + 11'(2*k)]     = {f_y2[k], f_x2[k], f_y1[k], f_x1[k]};
        ddr[ob + 11'(2*k) + 11'd1] = {15'd0, f_vld[k], f_trk[k], f_cls[k], f_c2[k], f_c1[k]};
      end
      // poison the result region so stale data cannot masquerade as a pass
      rb = DMA_RES_BASE[13:3];
      ddr[rb]        = 64'hDEAD_BEEF_DEAD_BEEF;
      ddr[rb + 11'd1] = 64'hDEAD_BEEF_DEAD_BEEF;
      // ---- golden ----
      compute_golden(n, tsk, cw1, cw2, cinv, cal);
      // ---- launch DMA + wait ----
      axi_write(A_CTRL, 64'h4);                       // CTRL.dma_start (bit 2)
      wait_dma(ok);
      if (!ok) begin
        $display("  [FAIL] %s : DMA TIMEOUT waiting for dma_done", nm);
        errcnt = errcnt + 1;
      end
      // ---- read engine results from CSRs ----
      axi_read(A_RESULT, r_res);
      axi_read(A_RUNNER, r_run);
      axi_read(A_BBOX,   r_bb);
      axi_read(A_STATUS, r_st);
      got_idx   = r_res[5:0];   got_cls = r_res[15:8]; got_score = r_res[63:32];
      got_run   = r_run[31:0];
      got_x1    = r_bb[15:0];   got_y1 = r_bb[31:16];
      got_x2    = r_bb[47:32];  got_y2 = r_bb[63:48];
      got_rv    = r_st[2];
      // ---- self-check engine path ----
      chk(nm, "result_valid", {63'd0, got_rv}, {63'd0, g_best_set});
      if (g_best_set) begin
        chk(nm, "winner idx",   {58'd0, got_idx},  {58'd0, g_best_idx});
        chk(nm, "winner class", {56'd0, got_cls},  {56'd0, g_best_cls});
        chk(nm, "winner score", {32'd0, got_score},{32'd0, g_best_score});
        chk(nm, "runner score", {32'd0, got_run},  {32'd0, g_second});
        chk(nm, "bbox x1", {48'd0, got_x1}, {48'd0, g_bx1});
        chk(nm, "bbox y2", {48'd0, got_y2}, {48'd0, g_by2});
      end else begin
        chk(nm, "winner score(=0)", {32'd0, got_score}, 64'd0);
      end
      // ---- verify the DMA result WRITE-BACK in DDR ----
      mw0 = ddr[rb];          // {score[63:32], 0, class[15:8], 0, idx[5:0]}
      mw1 = ddr[rb + 11'd1];  // {0, runner[31:0]}
      chk(nm, "wb idx",    {58'd0, mw0[5:0]},   {58'd0, g_best_idx});
      chk(nm, "wb class",  {56'd0, mw0[15:8]},  {56'd0, g_best_cls});
      chk(nm, "wb score",  {32'd0, mw0[63:32]}, {32'd0, g_best_score});
      chk(nm, "wb runner", {32'd0, mw1[31:0]},  {32'd0, g_second});
      $display("[%0t] %s [DMA] | task=%0d n=%0d -> winner idx=%0d class=%0d score=0x%08x runner=%0d valid=%0b | wb score=0x%08x",
               $time, nm, tsk, n, got_idx, got_cls, got_score, got_run, got_rv, mw0[63:32]);
    end
  endtask

  //==========================================================================
  // Stimulus
  //==========================================================================
  integer i;
  logic [63:0] tmp;
  logic [63:0] brd0, brd1, brd2, brd3;   // burst-read capture

  // global watchdog
  initial begin
    #2000000;
    $display(" [FATAL] global timeout");
    $finish;
  end

  initial begin
    // init drive + golden mirrors
    errcnt    = 0;
    s_awid='0; s_awaddr='0; s_awlen='0; s_awsize='0; s_awburst='0; s_awvalid=1'b0;
    s_wdata='0; s_wstrb='0; s_wlast=1'b0; s_wvalid=1'b0; s_bready=1'b0;
    s_arid='0; s_araddr='0; s_arlen='0; s_arsize='0; s_arburst='0; s_arvalid=1'b0;
    s_rready=1'b0;
    // master-port inputs (m_*) are driven by the AXI memory BFM below.
    for (i = 0; i < WMEM_DEPTH; i++) g_lut[i]  = '0;
    for (i = 0; i < NUM_TRACK;  i++) begin g_tval[i] = '0; g_tvld[i] = 1'b0; end

    $dumpfile("taod_tb.vcd");
    $dumpvars(0, taod_tb);

    // reset
    rst_n = 1'b0;
    repeat (8) @(posedge clk);
    rst_n = 1'b1;
    repeat (4) @(posedge clk);

    $display("==================================================================");
    $display(" EVIOS TAOD accelerator - self-checking testbench (PUSH + DMA)");
    $display("==================================================================");

    // VERSION sanity
    axi_read(A_VER, tmp);
    chk("init", "VERSION", {32'd0, tmp[31:0]}, {32'd0, 32'h7A0D_0001});
    $display("[%0t] VERSION = 0x%08x", $time, tmp[31:0]);

    // bring temporal memory to a known cold state before first frame
    do_clear();

    //----------------------------------------------------------------------
    // FRAME 1 : task-aware selection beats max-confidence
    //----------------------------------------------------------------------
    prog_w(4'd3, C_PERSON, 16'd0);     // person  irrelevant to "water plants"
    prog_w(4'd3, C_CHAIR,  16'd0);     // chair   irrelevant
    prog_w(4'd3, C_PLANT,  Q100);      // plant   fully relevant
    f_x1[0]=16'd100; f_y1[0]=16'd100; f_x2[0]=16'd180; f_y2[0]=16'd260;
    f_c1[0]=Q095; f_c2[0]=Q095; f_cls[0]=C_PERSON; f_trk[0]=8'd1;  f_vld[0]=1'b1;
    f_x1[1]=16'd200; f_y1[1]=16'd200; f_x2[1]=16'd300; f_y2[1]=16'd320;
    f_c1[1]=Q090; f_c2[1]=Q090; f_cls[1]=C_CHAIR;  f_trk[1]=8'd2;  f_vld[1]=1'b1;
    f_x1[2]=16'd400; f_y1[2]=16'd150; f_x2[2]=16'd470; f_y2[2]=16'd250;
    f_c1[2]=Q060; f_c2[2]=Q060; f_cls[2]=C_PLANT;  f_trk[2]=8'd20; f_vld[2]=1'b1;
    $display("--- F1 'water the plants': person .95, chair .90, plant .60 (only plant is relevant)");
    do_frame("F1 water-plants", 3, 4'd3, Q050, Q050, 32'd0, 4'd0);
    if (g_best_cls == C_PLANT)
      $display("    -> selected POTTED PLANT (class %0d) over higher-confidence person/chair: task-aware win", C_PLANT);

    //----------------------------------------------------------------------
    // FRAME 2 : spatial accessibility (area term) + IRQ path
    //----------------------------------------------------------------------
    prog_w(4'd5, C_CUP, Q100);
    axi_write(A_IRQCTL, 64'h1);         // enable IRQ
    f_x1[0]=16'd300; f_y1[0]=16'd200; f_x2[0]=16'd340; f_y2[0]=16'd240;  // 40x40, far
    f_c1[0]=Q070; f_c2[0]=Q070; f_cls[0]=C_CUP; f_trk[0]=8'd10; f_vld[0]=1'b1;
    f_x1[1]=16'd50;  f_y1[1]=16'd80;  f_x2[1]=16'd350; f_y2[1]=16'd340;  // 300x260, near
    f_c1[1]=Q065; f_c2[1]=Q065; f_cls[1]=C_CUP; f_trk[1]=8'd11; f_vld[1]=1'b1;
    $display("--- F2 'grab the cup': cupA .70 (small/far), cupB .65 (large/near); area term favors the nearer cup");
    do_frame("F2 area-term", 2, 4'd5, Q050, Q050, INVFA_VGA, 4'd0);
    // IRQ checks
    chk("F2 irq", "irq line", {63'd0, irq}, 64'd1);
    axi_read(A_STATUS, tmp);
    chk("F2 irq", "irq_pending(STATUS[3])", {63'd0, tmp[3]}, 64'd1);
    axi_write(A_IRQCTL, 64'h3);         // W1C clear, keep enabled
    repeat (4) @(posedge clk);
    chk("F2 irq", "irq deasserted", {63'd0, irq}, 64'd0);
    axi_write(A_IRQCTL, 64'h0);         // disable for the rest
    if (g_best_idx == 1)
      $display("    -> selected the LARGER/nearer cup (idx 1) despite lower confidence: accessibility win; IRQ verified");

    //----------------------------------------------------------------------
    // FRAME 3 : temporal stability smooths a noisy reading
    //----------------------------------------------------------------------
    prog_w(4'd3, C_PLANT, Q100);
    prog_w(4'd3, C_CHAIR, 16'd0);
    f_x1[0]=16'd400; f_y1[0]=16'd150; f_x2[0]=16'd470; f_y2[0]=16'd250;  // plant, noisy .30
    f_c1[0]=Q030; f_c2[0]=Q030; f_cls[0]=C_PLANT; f_trk[0]=8'd20; f_vld[0]=1'b1;
    f_x1[1]=16'd210; f_y1[1]=16'd210; f_x2[1]=16'd300; f_y2[1]=16'd320;  // chair .90 (irrelevant)
    f_c1[1]=Q090; f_c2[1]=Q090; f_cls[1]=C_CHAIR; f_trk[1]=8'd2;  f_vld[1]=1'b1;
    $display("--- F3 temporal EMA (alpha=2): plant track 20 seen at .60 in F1 now reads a noisy .30");
    do_frame("F3 temporal-EMA", 2, 4'd3, Q050, Q050, 32'd0, 4'd2);
    $display("    -> plant track-20 EMA after dip = %0d (~0.525 in UQ2.14) vs raw .30=%0d: stability preserved",
             g_tval[20], Q030);

    //----------------------------------------------------------------------
    // FRAME 4 : CLEAR -> cold start returns
    //----------------------------------------------------------------------
    do_clear();
    prog_w(4'd3, C_PLANT, Q100);
    prog_w(4'd3, C_CHAIR, 16'd0);
    f_x1[0]=16'd400; f_y1[0]=16'd150; f_x2[0]=16'd470; f_y2[0]=16'd250;
    f_c1[0]=Q060; f_c2[0]=Q060; f_cls[0]=C_PLANT; f_trk[0]=8'd20; f_vld[0]=1'b1;
    f_x1[1]=16'd210; f_y1[1]=16'd210; f_x2[1]=16'd300; f_y2[1]=16'd320;
    f_c1[1]=Q090; f_c2[1]=Q090; f_cls[1]=C_CHAIR; f_trk[1]=8'd2;  f_vld[1]=1'b1;
    $display("--- F4 after CLEAR: plant track 20 is cold -> temporal stability == raw fused confidence");
    do_frame("F4 post-clear", 2, 4'd3, Q050, Q050, 32'd0, 4'd0);

    //----------------------------------------------------------------------
    // FRAME 5 : all-invalid detections -> no selection
    //----------------------------------------------------------------------
    f_x1[0]=16'd10; f_y1[0]=16'd10; f_x2[0]=16'd20; f_y2[0]=16'd20;
    f_c1[0]=Q090; f_c2[0]=Q090; f_cls[0]=C_PERSON; f_trk[0]=8'd30; f_vld[0]=1'b0;
    f_x1[1]=16'd30; f_y1[1]=16'd30; f_x2[1]=16'd40; f_y2[1]=16'd40;
    f_c1[1]=Q090; f_c2[1]=Q090; f_cls[1]=C_CHAIR;  f_trk[1]=8'd31; f_vld[1]=1'b0;
    $display("--- F5 all detections invalid -> result_valid must be 0");
    do_frame("F5 no-detect", 2, 4'd3, Q050, Q050, 32'd0, 4'd0);

    //----------------------------------------------------------------------
    // FRAME 6 : empty frame (obj_count == 0) -> engine takes the IDLE->DONE
    // short path; result must be invalid.  (closes the empty-frame hole)
    //----------------------------------------------------------------------
    $display("--- F6 empty frame: obj_count=0 -> result_valid must be 0");
    do_frame("F6 empty", 0, 4'd3, Q050, Q050, 32'd0, 4'd0);

    //----------------------------------------------------------------------
    // FRAME 7 : high task id + single object (exercises task[7:4] and the
    // one-object count bin); lone relevant object -> zero decision margin.
    //----------------------------------------------------------------------
    prog_w(4'd10, C_PLANT, Q100);
    f_x1[0]=16'd50; f_y1[0]=16'd60; f_x2[0]=16'd130; f_y2[0]=16'd200;
    f_c1[0]=Q070; f_c2[0]=Q070; f_cls[0]=C_PLANT; f_trk[0]=8'd100; f_vld[0]=1'b1;
    $display("--- F7 high task=10, single object");
    do_frame("F7 hi-task-1obj", 1, 4'd10, Q050, Q050, 32'd0, 4'd0);

    //----------------------------------------------------------------------
    // FRAME 8 : many objects (10) of one relevant class with descending
    // confidence (exercises the >=9 object-count bin); highest fused
    // confidence wins, second-highest is the runner-up.
    //----------------------------------------------------------------------
    prog_w(4'd12, C_PERSON, Q100);
    for (i = 0; i < 10; i++) begin
      f_x1[i]=COORD_W'(10*i);     f_y1[i]=16'd10;
      f_x2[i]=COORD_W'(10*i+40);  f_y2[i]=16'd80;
      f_c1[i]=QW'(Q095 - 16'(i)*16'd600);   // 0.95 down to ~0.62
      f_c2[i]=f_c1[i];
      f_cls[i]=C_PERSON; f_trk[i]=TRK_W'(110+i); f_vld[i]=1'b1;
    end
    $display("--- F8 ten objects, descending confidence (max-confidence wins)");
    do_frame("F8 many-obj", 10, 4'd12, Q050, Q050, 32'd0, 4'd0);

    //======================================================================
    // DMA PATH : same scoring engine, but objects are DMA'd in from DDR and
    // the result is DMA'd back to DDR (zero CPU push/pop per frame).
    //======================================================================
    $display("==================================================================");
    $display(" DMA PATH (master reads objects from DDR, writes result back)");
    do_clear();                                       // resync temporal mem + golden

    //----------------------------------------------------------------------
    // D1 : "water the plants" via DMA -> must match the PUSH-mode F1 result
    // (6-beat read burst: 3 objects x 2 words).
    //----------------------------------------------------------------------
    prog_w(4'd3, C_PERSON, 16'd0);
    prog_w(4'd3, C_CHAIR,  16'd0);
    prog_w(4'd3, C_PLANT,  Q100);
    f_x1[0]=16'd100; f_y1[0]=16'd100; f_x2[0]=16'd180; f_y2[0]=16'd260;
    f_c1[0]=Q095; f_c2[0]=Q095; f_cls[0]=C_PERSON; f_trk[0]=8'd1;  f_vld[0]=1'b1;
    f_x1[1]=16'd200; f_y1[1]=16'd200; f_x2[1]=16'd300; f_y2[1]=16'd320;
    f_c1[1]=Q090; f_c2[1]=Q090; f_cls[1]=C_CHAIR;  f_trk[1]=8'd2;  f_vld[1]=1'b1;
    f_x1[2]=16'd400; f_y1[2]=16'd150; f_x2[2]=16'd470; f_y2[2]=16'd250;
    f_c1[2]=Q060; f_c2[2]=Q060; f_cls[2]=C_PLANT;  f_trk[2]=8'd20; f_vld[2]=1'b1;
    $display("--- D1 'water the plants' over DMA");
    do_dma_frame("D1 dma-plants", 3, 4'd3, Q050, Q050, 32'd0, 4'd0);
    if (g_best_cls == C_PLANT)
      $display("    -> DMA path selected POTTED PLANT and wrote it back to DDR");

    //----------------------------------------------------------------------
    // D2 : ten objects, descending confidence, via DMA (20-beat read burst,
    // positive runner-up) -> stresses a long INCR read burst.
    //----------------------------------------------------------------------
    prog_w(4'd12, C_PERSON, Q100);
    for (i = 0; i < 10; i++) begin
      f_x1[i]=COORD_W'(10*i);     f_y1[i]=16'd10;
      f_x2[i]=COORD_W'(10*i+40);  f_y2[i]=16'd80;
      f_c1[i]=QW'(Q095 - 16'(i)*16'd600);
      f_c2[i]=f_c1[i];
      f_cls[i]=C_PERSON; f_trk[i]=TRK_W'(110+i); f_vld[i]=1'b1;
    end
    $display("--- D2 ten objects over DMA (20-beat read burst)");
    do_dma_frame("D2 dma-many", 10, 4'd12, Q050, Q050, 32'd0, 4'd0);

    //======================================================================
    // B1 : AXI4 burst path on the slave -- 4-beat INCR write then 4-beat
    // INCR read to consecutive LUT words (entries 100..103, unused by any
    // frame, run last so golden state is irrelevant).  Verifies per-beat
    // address increment and RLAST placement.
    //======================================================================
    $display("==================================================================");
    $display(" AXI BURST PATH (slave 4-beat INCR write + read)");
    axi_write_burst4(LUT_OFF + 16'd800,
                     64'h0000_0000_0000_AAAA, 64'h0000_0000_0000_BBBB,
                     64'h0000_0000_0000_CCCC, 64'h0000_0000_0000_DDDD);
    begin : b1_check
      bit lok;
      axi_read_burst4(LUT_OFF + 16'd800, brd0, brd1, brd2, brd3, lok);
      chk("B1 burst", "beat0",  {48'd0, brd0[15:0]}, 64'h0000_AAAA);
      chk("B1 burst", "beat1",  {48'd0, brd1[15:0]}, 64'h0000_BBBB);
      chk("B1 burst", "beat2",  {48'd0, brd2[15:0]}, 64'h0000_CCCC);
      chk("B1 burst", "beat3",  {48'd0, brd3[15:0]}, 64'h0000_DDDD);
      chk("B1 burst", "rlast",  {63'd0, lok},        64'd1);
      $display("[%0t] B1 burst | wrote AAAA/BBBB/CCCC/DDDD, read back %04x/%04x/%04x/%04x rlast_ok=%0b",
               $time, brd0[15:0], brd1[15:0], brd2[15:0], brd3[15:0], lok);
    end

    //----------------------------------------------------------------------
    $display("==================================================================");
    u_cov.cov_report();
`ifdef __ICARUS__
    $display(" SVA assertion failures: %0d", u_sva.sva_fail);
    if (errcnt == 0 && u_sva.sva_fail == 0)
      $display(" RESULT: ALL CHECKS PASSED  (golden==RTL: 8 PUSH + 2 DMA frames, SVA clean)");
    else
      $display(" RESULT: %0d self-check + %0d SVA FAILURE(S)", errcnt, u_sva.sva_fail);
`else
    if (errcnt == 0)
      $display(" RESULT: ALL CHECKS PASSED  (golden==RTL: 8 PUSH + 2 DMA frames; concurrent SVA reported above)");
    else
      $display(" RESULT: %0d CHECK(S) FAILED", errcnt);
`endif
    $display("==================================================================");
    repeat (4) @(posedge clk);
    $finish;
  end

endmodule
