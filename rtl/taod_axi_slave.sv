//============================================================================
// taod_axi_slave.sv
//----------------------------------------------------------------------------
// Full AXI4 *slave* port for the TAOD accelerator (matches the VEGA
// "Example Accelerator" slave interface: 64-bit data, 12-bit ID, INCR bursts).
//
// It carries the complete AXI4 signal set but the register file behind it is
// single-beat / "Lite-like": each beat reads or writes one 64-bit word.  Burst
// (INCR) is supported by incrementing the word address every beat.
//
// The slave is generic: it does NOT know the register map.  It exposes a simple
// write strobe (s_wr_en / s_wr_addr / s_wr_data) and a read request
// (s_rd_en / s_rd_addr -> s_rd_data one cycle later) to the top level, which
// owns the registers, weight LUT and object buffer and does all decoding.
//
// Assumptions (documented, reasonable for a CPU-driven master):
//   * AW arrives before / with W (3-phase write).
//   * WSTRB is full (0xFF); sub-word writes are not supported -> use 64-bit
//     loads/stores from the RISC-V core (the driver is ours).
//============================================================================
module taod_axi_slave
  import taod_pkg::*;
(
  input  logic                     clk,
  input  logic                     rst_n,

  // ---- AXI4 write address channel ----
  input  logic [AXI_ID_W-1:0]      s_awid,
  input  logic [AXI_ADDR_W-1:0]    s_awaddr,
  input  logic [7:0]               s_awlen,
  input  logic [2:0]               s_awsize,
  input  logic [1:0]               s_awburst,
  input  logic                     s_awvalid,
  output logic                     s_awready,

  // ---- AXI4 write data channel ----
  input  logic [AXI_DATA_W-1:0]    s_wdata,
  input  logic [AXI_STRB_W-1:0]    s_wstrb,
  input  logic                     s_wlast,
  input  logic                     s_wvalid,
  output logic                     s_wready,

  // ---- AXI4 write response channel ----
  output logic [AXI_ID_W-1:0]      s_bid,
  output logic [1:0]               s_bresp,
  output logic                     s_bvalid,
  input  logic                     s_bready,

  // ---- AXI4 read address channel ----
  input  logic [AXI_ID_W-1:0]      s_arid,
  input  logic [AXI_ADDR_W-1:0]    s_araddr,
  input  logic [7:0]               s_arlen,
  input  logic [2:0]               s_arsize,
  input  logic [1:0]               s_arburst,
  input  logic                     s_arvalid,
  output logic                     s_arready,

  // ---- AXI4 read data channel ----
  output logic [AXI_ID_W-1:0]      s_rid,
  output logic [AXI_DATA_W-1:0]    s_rdata,
  output logic [1:0]               s_rresp,
  output logic                     s_rlast,
  output logic                     s_rvalid,
  input  logic                     s_rready,

  // ---- generic interface to top (register/LUT/buffer decode lives there) ----
  output logic                     s_wr_en,
  output logic [AXI_ADDR_W-1:0]    s_wr_addr,
  output logic [AXI_DATA_W-1:0]    s_wr_data,
  output logic                     s_rd_en,
  output logic [AXI_ADDR_W-1:0]    s_rd_addr,
  input  logic [AXI_DATA_W-1:0]    s_rd_data    // valid 1 cycle after s_rd_en
);

  localparam logic [1:0] RESP_OKAY = 2'b00;

  //==========================================================================
  // Write channel : AW -> W(beats) -> B
  //==========================================================================
  typedef enum logic [1:0] { W_AW, W_DATA, W_RESP } wstate_t;
  wstate_t wst;

  logic [AXI_ID_W-1:0]   awid_r;
  logic [AXI_ADDR_W-1:0] waddr_r;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      wst       <= W_AW;
      awid_r    <= '0;
      waddr_r   <= '0;
      s_wr_en   <= 1'b0;
      s_wr_addr <= '0;
      s_wr_data <= '0;
      s_bvalid  <= 1'b0;
      s_bid     <= '0;
    end else begin
      s_wr_en <= 1'b0;                       // default: write strobe is a pulse

      case (wst)
        W_AW: begin
          if (s_awvalid && s_awready) begin
            awid_r  <= s_awid;
            waddr_r <= s_awaddr;
            wst     <= W_DATA;
          end
        end

        W_DATA: begin
          if (s_wvalid && s_wready) begin
            s_wr_en   <= 1'b1;               // tell top to write
            s_wr_addr <= waddr_r;
            s_wr_data <= s_wdata;
            waddr_r   <= waddr_r + 'd8;       // next 64-bit word (INCR)
            if (s_wlast) begin
              s_bid    <= awid_r;
              s_bvalid <= 1'b1;
              wst      <= W_RESP;
            end
          end
        end

        W_RESP: begin
          if (s_bvalid && s_bready) begin
            s_bvalid <= 1'b0;
            wst      <= W_AW;
          end
        end

        default: wst <= W_AW;
      endcase
    end
  end

  assign s_awready = (wst == W_AW);
  assign s_wready  = (wst == W_DATA);
  assign s_bresp   = RESP_OKAY;

  //==========================================================================
  // Read channel : AR -> REQ -> WAIT(1 cyc for s_rd_data) -> VALID(beats)
  //==========================================================================
  typedef enum logic [1:0] { R_IDLE, R_REQ, R_WAIT, R_VALID } rstate_t;
  rstate_t rst_st;

  logic [AXI_ID_W-1:0]   arid_r;
  logic [AXI_ADDR_W-1:0] raddr_r;
  logic [7:0]            beat_cnt;           // remaining beats after current

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      rst_st    <= R_IDLE;
      arid_r    <= '0;
      raddr_r   <= '0;
      beat_cnt  <= '0;
      s_rd_en   <= 1'b0;
      s_rd_addr <= '0;
      s_rvalid  <= 1'b0;
      s_rid     <= '0;
      s_rlast   <= 1'b0;
      s_rdata   <= '0;
    end else begin
      s_rd_en <= 1'b0;                       // default

      case (rst_st)
        R_IDLE: begin
          s_rvalid <= 1'b0;
          s_rlast  <= 1'b0;
          if (s_arvalid && s_arready) begin
            arid_r   <= s_arid;
            raddr_r  <= s_araddr;
            beat_cnt <= s_arlen;             // AXI len = beats-1
            rst_st   <= R_REQ;
          end
        end

        R_REQ: begin
          s_rd_en   <= 1'b1;                 // request data at raddr_r
          s_rd_addr <= raddr_r;
          rst_st    <= R_WAIT;
        end

        R_WAIT: begin
          rst_st <= R_VALID;                 // s_rd_data lands now
        end

        R_VALID: begin
          s_rvalid <= 1'b1;
          s_rid    <= arid_r;
          s_rdata  <= s_rd_data;
          s_rlast  <= (beat_cnt == '0);
          if (s_rvalid && s_rready) begin
            if (beat_cnt == '0) begin
              s_rvalid <= 1'b0;
              s_rlast  <= 1'b0;
              rst_st   <= R_IDLE;
            end else begin
              beat_cnt <= beat_cnt - 1'b1;
              raddr_r  <= raddr_r + 'd8;
              s_rvalid <= 1'b0;
              rst_st   <= R_REQ;             // fetch next beat
            end
          end
        end

        default: rst_st <= R_IDLE;
      endcase
    end
  end

  assign s_arready = (rst_st == R_IDLE);
  assign s_rresp   = RESP_OKAY;

endmodule
