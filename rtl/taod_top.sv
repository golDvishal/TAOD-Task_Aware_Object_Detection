//============================================================================
// taod_top.sv
//----------------------------------------------------------------------------
// Top-level integration for the Task-Aware Object Detection (TAOD) accelerator.
//
// Project : EVIOS  -- DVCon India 2026 Design Contest
// Target  : CDAC VEGA AS1061 RISC-V SoC on Genesys2 (Kintex-7 XC7K325T)
//
// This block presents the two AXI4 ports the VEGA "Example Accelerator" spec
// mandates -- a full AXI4 *slave* (CPU programs / reads back) and a full AXI4
// *master* (DMA of object records in and results out) -- plus a level IRQ to
// the VEGA PLIC.  It owns everything the scoring engine reads:
//
//   * Control/Status/Config registers (CSRs)
//   * Task x Class weight LUT          (NUM_TASKS*NUM_CLASS = 1280 x 16b)
//   * Object metadata buffer           (MAX_OBJ = 64 records x 128b)
//
// and routes the engine results back to both the slave (readback) and the
// master (DMA write-back).  The slave is generic (no map knowledge); ALL
// address decode lives here.
//
//----------------------------------------------------------------------------
// ADDRESS MAP  (offsets within the 64 KiB accelerator window @ base 0x2006_0000;
//               only the low 16 address bits are decoded, so the interconnect
//               may pass either the full or the base-stripped address)
//----------------------------------------------------------------------------
//   CSR region          0x0000 .. 0x0FFF
//     0x000  CTRL   (W)  [0]=start(PUSH)  [1]=clear  [2]=dma_start   (write-1-pulse)
//     0x008  TASK   (RW) [3:0]=task_id
//     0x010  OBJCNT (RW) [6:0]=obj_count (0..64)
//     0x018  FUSE   (RW) [15:0]=w1 (UQ2.14)   [31:16]=w2 (UQ2.14)
//     0x020  INVFA  (RW) [31:0]=floor(2^32 / frame_area)
//     0x028  ALPHA  (RW) [3:0]=EMA shift = log2(N)
//     0x030  DMAOBJ (RW) [31:0]=DMA object-base address (DDR)
//     0x038  DMARES (RW) [31:0]=DMA result-base address (DDR)
//     0x040  IRQCTL (W)  [0]=irq_enable   [1]=irq_clear (W1C)
//     0x0F8  VERSION(R)  0x7A0D_0001
//     0x100  STATUS (R)  [0]=busy [1]=done [2]=result_valid
//                        [3]=irq_pending [4]=dma_busy [5]=dma_done
//     0x108  RESULT (R)  [5:0]=idx [15:8]=class [63:32]=winning score (UQ4.28)
//     0x110  RUNNER (R)  [31:0]=runner-up score (UQ4.28)   -- decision margin
//     0x118  BBOX   (R)  [15:0]=x1 [31:16]=y1 [47:32]=x2 [63:48]=y2
//   Weight-LUT window    0x1000 .. (0x1000 + 1280*8 - 1) = 0x37FF
//     one 16-bit entry per 64-bit word; entry = (off-0x1000)>>3 ;
//     linear index = task_id*NUM_CLASS + class_id
//   Object buffer window 0x4000 .. (0x4000 + 64*16 - 1) = 0x43FF
//     record = (off-0x4000)>>4 ; word = bit[3] (0=low,1=high)
//
// OBJECT RECORD PACKING (128b = two 64-bit words; word0 @ lower address):
//   word0[15:0]=x1  [31:16]=y1  [47:32]=x2  [63:48]=y2
//   word1[15:0]=c1  [31:16]=c2  [39:32]=class  [47:40]=track  [48]=valid
// RESULT DMA WRITE-BACK (master writes 2 beats; bbox available via slave BBOX):
//   res_word0 == RESULT layout ; res_word1[31:0] == runner-up score
//============================================================================
module taod_top
  import taod_pkg::*;
(
  input  logic                     clk,
  input  logic                     rst_n,

  //==========================================================================
  // AXI4 SLAVE  (VEGA interconnect master -> accelerator: CSRs/LUT/objbuf)
  //==========================================================================
  input  logic [AXI_ID_W-1:0]      s_awid,
  input  logic [AXI_ADDR_W-1:0]    s_awaddr,
  input  logic [7:0]               s_awlen,
  input  logic [2:0]               s_awsize,
  input  logic [1:0]               s_awburst,
  input  logic                     s_awvalid,
  output logic                     s_awready,
  input  logic [AXI_DATA_W-1:0]    s_wdata,
  input  logic [AXI_STRB_W-1:0]    s_wstrb,
  input  logic                     s_wlast,
  input  logic                     s_wvalid,
  output logic                     s_wready,
  output logic [AXI_ID_W-1:0]      s_bid,
  output logic [1:0]               s_bresp,
  output logic                     s_bvalid,
  input  logic                     s_bready,
  input  logic [AXI_ID_W-1:0]      s_arid,
  input  logic [AXI_ADDR_W-1:0]    s_araddr,
  input  logic [7:0]               s_arlen,
  input  logic [2:0]               s_arsize,
  input  logic [1:0]               s_arburst,
  input  logic                     s_arvalid,
  output logic                     s_arready,
  output logic [AXI_ID_W-1:0]      s_rid,
  output logic [AXI_DATA_W-1:0]    s_rdata,
  output logic [1:0]               s_rresp,
  output logic                     s_rlast,
  output logic                     s_rvalid,
  input  logic                     s_rready,

  //==========================================================================
  // AXI4 MASTER  (accelerator -> DDR: DMA object-in / result-out)
  //==========================================================================
  output logic [AXI_ID_W-1:0]      m_arid,
  output logic [AXI_ADDR_W-1:0]    m_araddr,
  output logic [7:0]               m_arlen,
  output logic [2:0]               m_arsize,
  output logic [1:0]               m_arburst,
  output logic                     m_arvalid,
  input  logic                     m_arready,
  input  logic [AXI_ID_W-1:0]      m_rid,
  input  logic [AXI_DATA_W-1:0]    m_rdata,
  input  logic [1:0]               m_rresp,
  input  logic                     m_rlast,
  input  logic                     m_rvalid,
  output logic                     m_rready,
  output logic [AXI_ID_W-1:0]      m_awid,
  output logic [AXI_ADDR_W-1:0]    m_awaddr,
  output logic [7:0]               m_awlen,
  output logic [2:0]               m_awsize,
  output logic [1:0]               m_awburst,
  output logic                     m_awvalid,
  input  logic                     m_awready,
  output logic [AXI_DATA_W-1:0]    m_wdata,
  output logic [AXI_STRB_W-1:0]    m_wstrb,
  output logic                     m_wlast,
  output logic                     m_wvalid,
  input  logic                     m_wready,
  input  logic [AXI_ID_W-1:0]      m_bid,
  input  logic [1:0]               m_bresp,
  input  logic                     m_bvalid,
  output logic                     m_bready,

  //==========================================================================
  // Interrupt to VEGA PLIC (active high, level)
  //==========================================================================
  output logic                     irq
);

  //--------------------------------------------------------------------------
  // Region bounds (16-bit offsets).  Computed from package dims so the map
  // stays correct if MAX_OBJ / WMEM_DEPTH change.
  //--------------------------------------------------------------------------
  localparam logic [15:0] CSR_END = 16'h1000;
  localparam logic [15:0] LUT_OFF = 16'h1000;
  localparam logic [15:0] LUT_END = LUT_OFF + 16'(WMEM_DEPTH * 8);   // 0x3800
  localparam logic [15:0] OBJ_OFF = 16'h4000;
  localparam logic [15:0] OBJ_END = OBJ_OFF + 16'(MAX_OBJ * 16);     // 0x4400

  //==========================================================================
  // Slave <-> top generic interface
  //==========================================================================
  logic                  s_wr_en;
  logic [AXI_ADDR_W-1:0] s_wr_addr;
  logic [AXI_DATA_W-1:0] s_wr_data;
  logic                  s_rd_en;
  logic [AXI_ADDR_W-1:0] s_rd_addr;
  logic [AXI_DATA_W-1:0] s_rd_data;

  //==========================================================================
  // Configuration registers (programmed over the slave)
  //==========================================================================
  logic [TASK_W-1:0]     task_id;
  logic [CNT_W-1:0]      obj_count;
  logic [QW-1:0]         w1, w2;
  logic [31:0]           inv_fa;
  logic [TASK_W-1:0]     alpha;
  logic [AXI_ADDR_W-1:0] dma_obj_base, dma_res_base;
  logic                  irq_en;

  // single-cycle command pulses (decoded from CTRL / IRQCTL writes)
  logic                  cpu_start;   // PUSH-mode engine launch
  logic                  eng_clear;   // temporal-memory clear sweep
  logic                  dma_start;   // master DMA launch
  logic                  irq_clr;     // W1C of irq_pending

  //==========================================================================
  // Engine <-> top wiring
  //==========================================================================
  logic [IDX_W-1:0]      obj_addr;
  logic [QW-1:0]         obj_c1, obj_c2;
  logic [CLASS_W-1:0]    obj_class;
  logic [TRK_W-1:0]      obj_track;
  logic                  obj_valid;
  logic [COORD_W-1:0]    obj_x1, obj_y1, obj_x2, obj_y2;
  logic [WMEM_AW-1:0]    w_addr;
  logic [QW-1:0]         w_data;
  logic                  eng_busy, eng_done, eng_result_valid;
  logic [IDX_W-1:0]      res_idx;
  logic [CLASS_W-1:0]    res_class;
  logic [31:0]           res_score, res_runner;
  logic [COORD_W-1:0]    res_x1, res_y1, res_x2, res_y2;

  //==========================================================================
  // Master <-> top wiring
  //==========================================================================
  logic                  dma_busy, dma_done;
  logic                  m_eng_start;
  logic                  m_obj_wr_en;
  logic [AXI_ADDR_W-1:0] m_obj_wr_addr;
  logic [AXI_DATA_W-1:0] m_obj_wr_data;
  logic [AXI_DATA_W-1:0] res_word0, res_word1;

  // engine launch is shared: CPU pulse (PUSH) or master pulse (DMA)
  logic                  eng_start;
  assign eng_start = cpu_start | m_eng_start;

  //==========================================================================
  // On-chip memories owned by top
  //==========================================================================
  logic [QW-1:0]  wlut   [0:WMEM_DEPTH-1];   // task x class relevance weights
  logic [127:0]   objbuf [0:MAX_OBJ-1];      // per-frame object records

  //==========================================================================
  // CONFIG + COMMAND decode (slave writes to CSR region)
  //==========================================================================
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      task_id      <= '0;
      obj_count    <= '0;
      w1           <= 16'd8192;          // 0.5 in UQ2.14 (default = average)
      w2           <= 16'd8192;          // 0.5
      inv_fa       <= '0;                // 0 => area term == 1.0 (disabled)
      alpha        <= '0;                // 0 => EMA tracks current frame
      dma_obj_base <= '0;
      dma_res_base <= '0;
      irq_en       <= 1'b0;
      cpu_start    <= 1'b0;
      eng_clear    <= 1'b0;
      dma_start    <= 1'b0;
      irq_clr      <= 1'b0;
    end else begin
      // command/clear strobes are 1-cycle pulses
      cpu_start <= 1'b0;
      eng_clear <= 1'b0;
      dma_start <= 1'b0;
      irq_clr   <= 1'b0;

      if (s_wr_en && (s_wr_addr[15:0] < CSR_END)) begin
        case (s_wr_addr[15:0])
          16'h0000: begin
            cpu_start <= s_wr_data[0];
            eng_clear <= s_wr_data[1];
            dma_start <= s_wr_data[2];
          end
          16'h0008: task_id      <= s_wr_data[TASK_W-1:0];
          16'h0010: obj_count    <= s_wr_data[CNT_W-1:0];
          16'h0018: begin
            w1 <= s_wr_data[0   +: QW];   // [15:0]
            w2 <= s_wr_data[QW  +: QW];   // [31:16]
          end
          16'h0020: inv_fa       <= s_wr_data[31:0];
          16'h0028: alpha        <= s_wr_data[TASK_W-1:0];
          16'h0030: dma_obj_base <= s_wr_data[AXI_ADDR_W-1:0];
          16'h0038: dma_res_base <= s_wr_data[AXI_ADDR_W-1:0];
          16'h0040: begin
            irq_en  <= s_wr_data[0];
            irq_clr <= s_wr_data[1];
          end
          default: ;
        endcase
      end
    end
  end

  //==========================================================================
  // WEIGHT LUT write port (slave only; programmed before a run)
  //==========================================================================
  logic [WMEM_AW-1:0] lut_widx;
  always_comb lut_widx = WMEM_AW'((s_wr_addr[15:0] - LUT_OFF) >> 3);

  always_ff @(posedge clk) begin
    if (s_wr_en && (s_wr_addr[15:0] >= LUT_OFF) && (s_wr_addr[15:0] < LUT_END))
      wlut[lut_widx] <= s_wr_data[QW-1:0];
  end

  //==========================================================================
  // OBJECT BUFFER write port (master DMA has priority; else slave PUSH)
  //==========================================================================
  logic                  obj_we;
  logic [15:0]           obj_woff;       // byte offset inside window
  logic [AXI_DATA_W-1:0] obj_wdat;
  always_comb begin
    if (m_obj_wr_en) begin
      obj_we   = 1'b1;
      obj_woff = m_obj_wr_addr[15:0];
      obj_wdat = m_obj_wr_data;
    end else if (s_wr_en && (s_wr_addr[15:0] >= OBJ_OFF)
                        && (s_wr_addr[15:0] < OBJ_END)) begin
      obj_we   = 1'b1;
      obj_woff = s_wr_addr[15:0] - OBJ_OFF;
      obj_wdat = s_wr_data;
    end else begin
      obj_we   = 1'b0;
      obj_woff = '0;
      obj_wdat = '0;
    end
  end

  // NB: in DMA mode the master path uses an offset already relative to the
  // window base (m_obj_wr_addr = beat*8), so no OBJ_OFF subtraction there.
  wire [IDX_W-1:0] obj_wrec  = obj_woff[9:4];   // record index (auto-wraps 0..63)
  wire             obj_wword = obj_woff[3];     // 0 = low word, 1 = high word

  always_ff @(posedge clk) begin
    if (obj_we) begin
      if (!obj_wword) objbuf[obj_wrec][63:0]   <= obj_wdat;
      else            objbuf[obj_wrec][127:64] <= obj_wdat;
    end
  end

  //==========================================================================
  // OBJECT BUFFER read for the engine (1-cycle synchronous; no read/write
  // race: objects are fully loaded before the engine is launched)
  //==========================================================================
  logic [127:0] obj_rec_q;
  always_ff @(posedge clk) obj_rec_q <= objbuf[obj_addr];

  assign obj_x1    = obj_rec_q[15:0];
  assign obj_y1    = obj_rec_q[31:16];
  assign obj_x2    = obj_rec_q[47:32];
  assign obj_y2    = obj_rec_q[63:48];
  assign obj_c1    = obj_rec_q[79:64];
  assign obj_c2    = obj_rec_q[95:80];
  assign obj_class = obj_rec_q[103:96];
  assign obj_track = obj_rec_q[111:104];
  assign obj_valid = obj_rec_q[112];

  //==========================================================================
  // WEIGHT LUT read for the engine (1-cycle synchronous)
  //==========================================================================
  logic [QW-1:0] w_data_q;
  always_ff @(posedge clk) w_data_q <= wlut[w_addr];
  assign w_data = w_data_q;

  //==========================================================================
  // STATUS / IRQ sticky bits
  //==========================================================================
  logic done_sticky, dma_done_sticky, irq_pending;
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      done_sticky     <= 1'b0;
      dma_done_sticky <= 1'b0;
      irq_pending     <= 1'b0;
    end else begin
      if (eng_start)  done_sticky     <= 1'b0;   // new run clears
      if (eng_done)   done_sticky     <= 1'b1;
      if (dma_start)  dma_done_sticky <= 1'b0;
      if (dma_done)   dma_done_sticky <= 1'b1;

      if (eng_done | dma_done) irq_pending <= 1'b1;
      if (irq_clr)             irq_pending <= 1'b0;   // W1C wins on same cycle
    end
  end

  assign irq = irq_pending & irq_en;

  //==========================================================================
  // PERFORMANCE COUNTERS (observation only -- never feed any datapath)
  //   0x0120 PERF_COMPUTE : engine-busy cycles of last frame  (= 6N+2)
  //   0x0128 PERF_FRAME   : cycles from launch (CPU/DMA) to engine done
  //                         (includes DMA object-read latency in DMA mode)
  //==========================================================================
  logic [31:0] perf_comp_run, perf_comp_last;
  logic [31:0] perf_frame_run, perf_frame_last;
  logic        perf_frame_active;
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      perf_comp_run     <= '0; perf_comp_last  <= '0;
      perf_frame_run    <= '0; perf_frame_last <= '0;
      perf_frame_active <= 1'b0;
    end else begin
      // compute-cycle counter: how long eng_busy is asserted
      if (eng_busy) perf_comp_run <= perf_comp_run + 1'b1;
      else          perf_comp_run <= '0;
      if (eng_done) perf_comp_last <= perf_comp_run + 1'b1;   // incl. done cycle
      // frame-cycle counter: from launch to engine done
      if (cpu_start | dma_start) begin
        perf_frame_active <= 1'b1;
        perf_frame_run    <= '0;
      end else if (perf_frame_active) begin
        perf_frame_run    <= perf_frame_run + 1'b1;
      end
      if (eng_done) begin
        perf_frame_active <= 1'b0;
        perf_frame_last   <= perf_frame_run + 1'b1;
      end
    end
  end

  //==========================================================================
  // READBACK : capture on s_rd_en, present 1 cycle later (held until next req)
  //==========================================================================
  logic [AXI_DATA_W-1:0] csr_rd_c;       // combinational CSR select
  always_comb begin
    case (s_rd_addr[15:0])
      16'h0008: csr_rd_c = {{(AXI_DATA_W-TASK_W){1'b0}}, task_id};
      16'h0010: csr_rd_c = {{(AXI_DATA_W-CNT_W){1'b0}},  obj_count};
      16'h0018: csr_rd_c = {32'd0, w2, w1};
      16'h0020: csr_rd_c = {32'd0, inv_fa};
      16'h0028: csr_rd_c = {{(AXI_DATA_W-TASK_W){1'b0}}, alpha};
      16'h0030: csr_rd_c = {32'd0, dma_obj_base};
      16'h0038: csr_rd_c = {32'd0, dma_res_base};
      16'h00F8: csr_rd_c = 64'h0000_0000_7A0D_0001;
      16'h0100: csr_rd_c = {58'd0, dma_done_sticky, dma_busy,
                                   irq_pending, eng_result_valid,
                                   done_sticky, eng_busy};
      16'h0108: csr_rd_c = {res_score, 16'd0, res_class, 2'd0, res_idx};
      16'h0110: csr_rd_c = {32'd0, res_runner};
      16'h0118: csr_rd_c = {res_y2, res_x2, res_y1, res_x1};
      16'h0120: csr_rd_c = {32'd0, perf_comp_last};   // compute cycles (6N+2)
      16'h0128: csr_rd_c = {32'd0, perf_frame_last};  // frame cycles (incl DMA)
      default:  csr_rd_c = '0;
    endcase
  end

  logic [WMEM_AW-1:0] lut_ridx;
  logic [IDX_W-1:0]   obj_ridx;
  always_comb begin
    lut_ridx = ((s_rd_addr[15:0] >= LUT_OFF) && (s_rd_addr[15:0] < LUT_END))
             ? WMEM_AW'((s_rd_addr[15:0] - LUT_OFF) >> 3) : '0;
    obj_ridx = IDX_W'((s_rd_addr[15:0] - OBJ_OFF) >> 4);   // wraps 0..63
  end

  logic [15:0]           rd_off_q;
  logic [AXI_DATA_W-1:0] csr_rd_q;
  logic [QW-1:0]         lut_rd_q;
  logic [127:0]          obj_rd_q;
  always_ff @(posedge clk) begin
    if (s_rd_en) begin
      rd_off_q <= s_rd_addr[15:0];
      csr_rd_q <= csr_rd_c;
      lut_rd_q <= wlut[lut_ridx];
      obj_rd_q <= objbuf[obj_ridx];
    end
  end

  always_comb begin
    if      (rd_off_q < CSR_END) s_rd_data = csr_rd_q;
    else if (rd_off_q < LUT_END) s_rd_data = {{(AXI_DATA_W-QW){1'b0}}, lut_rd_q};
    else                         s_rd_data = rd_off_q[3] ? obj_rd_q[127:64]
                                                         : obj_rd_q[63:0];
  end

  //==========================================================================
  // RESULT packing for the master DMA write-back
  //==========================================================================
  assign res_word0 = {res_score, 16'd0, res_class, 2'd0, res_idx};
  assign res_word1 = {32'd0, res_runner};

  //==========================================================================
  // SUBMODULE INSTANCES
  //==========================================================================
  taod_axi_slave u_slv (
    .clk       (clk),
    .rst_n     (rst_n),
    .s_awid    (s_awid),    .s_awaddr  (s_awaddr),  .s_awlen   (s_awlen),
    .s_awsize  (s_awsize),  .s_awburst (s_awburst), .s_awvalid (s_awvalid),
    .s_awready (s_awready),
    .s_wdata   (s_wdata),   .s_wstrb   (s_wstrb),   .s_wlast   (s_wlast),
    .s_wvalid  (s_wvalid),  .s_wready  (s_wready),
    .s_bid     (s_bid),     .s_bresp   (s_bresp),   .s_bvalid  (s_bvalid),
    .s_bready  (s_bready),
    .s_arid    (s_arid),    .s_araddr  (s_araddr),  .s_arlen   (s_arlen),
    .s_arsize  (s_arsize),  .s_arburst (s_arburst), .s_arvalid (s_arvalid),
    .s_arready (s_arready),
    .s_rid     (s_rid),     .s_rdata   (s_rdata),   .s_rresp   (s_rresp),
    .s_rlast   (s_rlast),   .s_rvalid  (s_rvalid),  .s_rready  (s_rready),
    .s_wr_en   (s_wr_en),   .s_wr_addr (s_wr_addr), .s_wr_data (s_wr_data),
    .s_rd_en   (s_rd_en),   .s_rd_addr (s_rd_addr), .s_rd_data (s_rd_data)
  );

  taod_engine u_eng (
    .clk          (clk),
    .rst_n        (rst_n),
    .start        (eng_start),
    .clear        (eng_clear),
    .task_id      (task_id),
    .obj_count    (obj_count),
    .w1           (w1),
    .w2           (w2),
    .inv_fa       (inv_fa),
    .alpha        (alpha),
    .obj_addr     (obj_addr),
    .obj_c1       (obj_c1),
    .obj_c2       (obj_c2),
    .obj_class    (obj_class),
    .obj_track    (obj_track),
    .obj_valid    (obj_valid),
    .obj_x1       (obj_x1),
    .obj_y1       (obj_y1),
    .obj_x2       (obj_x2),
    .obj_y2       (obj_y2),
    .w_addr       (w_addr),
    .w_data       (w_data),
    .busy         (eng_busy),
    .done         (eng_done),
    .result_valid (eng_result_valid),
    .res_idx      (res_idx),
    .res_class    (res_class),
    .res_score    (res_score),
    .res_runner   (res_runner),
    .res_x1       (res_x1),
    .res_y1       (res_y1),
    .res_x2       (res_x2),
    .res_y2       (res_y2)
  );

  taod_axi_master u_mst (
    .clk           (clk),
    .rst_n         (rst_n),
    .dma_start     (dma_start),
    .dma_obj_base  (dma_obj_base),
    .dma_res_base  (dma_res_base),
    .obj_count     (obj_count),
    .dma_busy      (dma_busy),
    .dma_done      (dma_done),
    .eng_start     (m_eng_start),
    .eng_done      (eng_done),
    .m_obj_wr_en   (m_obj_wr_en),
    .m_obj_wr_addr (m_obj_wr_addr),
    .m_obj_wr_data (m_obj_wr_data),
    .res_word0     (res_word0),
    .res_word1     (res_word1),
    .m_arid        (m_arid),    .m_araddr  (m_araddr),  .m_arlen   (m_arlen),
    .m_arsize      (m_arsize),  .m_arburst (m_arburst), .m_arvalid (m_arvalid),
    .m_arready     (m_arready),
    .m_rid         (m_rid),     .m_rdata   (m_rdata),   .m_rresp   (m_rresp),
    .m_rlast       (m_rlast),   .m_rvalid  (m_rvalid),  .m_rready  (m_rready),
    .m_awid        (m_awid),    .m_awaddr  (m_awaddr),  .m_awlen   (m_awlen),
    .m_awsize      (m_awsize),  .m_awburst (m_awburst), .m_awvalid (m_awvalid),
    .m_awready     (m_awready),
    .m_wdata       (m_wdata),   .m_wstrb   (m_wstrb),   .m_wlast   (m_wlast),
    .m_wvalid      (m_wvalid),  .m_wready  (m_wready),
    .m_bid         (m_bid),     .m_bresp   (m_bresp),   .m_bvalid  (m_bvalid),
    .m_bready      (m_bready)
  );

endmodule