//============================================================================
// taod_axi_master.sv
//----------------------------------------------------------------------------
// Full AXI4 *master* port for the TAOD accelerator (matches the VEGA
// "Example Accelerator" master interface).  Implements the DMA datapath:
//
//   M_RD_OBJ   : INCR-burst read obj_count*2 64-bit words from dma_obj_base,
//                writing them into the object buffer (m_obj_wr_*).
//   (pulse engine start)
//   M_WAIT_ENG : wait for engine done.
//   M_WR_RES   : INCR-burst write the result words to dma_res_base.
//   M_DONE     : pulse dma_done.
//
// VERIFICATION SCOPE
//   Both datapaths are simulated and self-checked against a bit-accurate golden
//   model: PUSH mode (objects written / results read over the AXI slave) and
//   DMA mode (this master reads object records from DDR via an INCR read burst,
//   runs the identical scoring engine, and writes the result back to DDR via an
//   INCR write burst).  The DMA path is exercised against an AXI4 memory BFM in
//   taod_tb.sv (frames D1, D2), with results cross-checked both through the
//   result CSRs and through the words written back to memory.
//============================================================================
module taod_axi_master
  import taod_pkg::*;
(
  input  logic                     clk,
  input  logic                     rst_n,

  // control (from top)
  input  logic                     dma_start,     // 1-cycle: begin DMA frame
  input  logic [AXI_ADDR_W-1:0]    dma_obj_base,
  input  logic [AXI_ADDR_W-1:0]    dma_res_base,
  input  logic [CNT_W-1:0]         obj_count,
  output logic                     dma_busy,
  output logic                     dma_done,

  // engine handshake
  output logic                     eng_start,     // pulse to launch engine
  input  logic                     eng_done,

  // object-buffer write port (muxed with slave at top in DMA mode)
  output logic                     m_obj_wr_en,
  output logic [AXI_ADDR_W-1:0]    m_obj_wr_addr, // byte offset into buffer window
  output logic [AXI_DATA_W-1:0]    m_obj_wr_data,

  // result words to write back (from top: packed engine results)
  input  logic [AXI_DATA_W-1:0]    res_word0,
  input  logic [AXI_DATA_W-1:0]    res_word1,

  // ---- AXI4 read address channel ----
  output logic [AXI_ID_W-1:0]      m_arid,
  output logic [AXI_ADDR_W-1:0]    m_araddr,
  output logic [7:0]               m_arlen,
  output logic [2:0]               m_arsize,
  output logic [1:0]               m_arburst,
  output logic                     m_arvalid,
  input  logic                     m_arready,

  // ---- AXI4 read data channel ----
  input  logic [AXI_ID_W-1:0]      m_rid,
  input  logic [AXI_DATA_W-1:0]    m_rdata,
  input  logic [1:0]               m_rresp,
  input  logic                     m_rlast,
  input  logic                     m_rvalid,
  output logic                     m_rready,

  // ---- AXI4 write address channel ----
  output logic [AXI_ID_W-1:0]      m_awid,
  output logic [AXI_ADDR_W-1:0]    m_awaddr,
  output logic [7:0]               m_awlen,
  output logic [2:0]               m_awsize,
  output logic [1:0]               m_awburst,
  output logic                     m_awvalid,
  input  logic                     m_awready,

  // ---- AXI4 write data channel ----
  output logic [AXI_DATA_W-1:0]    m_wdata,
  output logic [AXI_STRB_W-1:0]    m_wstrb,
  output logic                     m_wlast,
  output logic                     m_wvalid,
  input  logic                     m_wready,

  // ---- AXI4 write response channel ----
  input  logic [AXI_ID_W-1:0]      m_bid,
  input  logic [1:0]               m_bresp,
  input  logic                     m_bvalid,
  output logic                     m_bready
);

  localparam logic [2:0] SIZE_8B   = 3'b011;   // 8 bytes/beat
  localparam logic [1:0] BURST_INC = 2'b01;

  typedef enum logic [3:0] {     // 9 states -> needs 4 bits (M_DONE was overflowing 3b)
    M_IDLE,
    M_RD_AR,     // issue read address
    M_RD_DATA,   // collect read beats into object buffer
    M_RUN,       // pulse engine start
    M_WAIT_ENG,  // wait engine done
    M_WR_AW,     // issue write address (results)
    M_WR_DATA,   // send result beats
    M_WR_B,      // wait write response
    M_DONE
  } mstate_t;

  mstate_t mst;

  logic [AXI_ADDR_W-1:0] obj_base_r, res_base_r;
  logic [CNT_W-1:0]      cnt_r;
  logic [8:0]            rd_beats;       // total read beats = obj_count*2
  logic [8:0]            rd_idx;         // beats received
  logic [1:0]            wr_idx;         // result beats sent (2)

  // total read beats = obj_count * 2  (two 64-bit words per object record)
  wire [8:0] rd_beats_w = {cnt_r, 1'b0}; // *2

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      mst         <= M_IDLE;
      obj_base_r  <= '0;
      res_base_r  <= '0;
      cnt_r       <= '0;
      rd_beats    <= '0;
      rd_idx      <= '0;
      wr_idx      <= '0;
      eng_start   <= 1'b0;
      dma_done    <= 1'b0;

      m_arid    <= '0; m_araddr <= '0; m_arlen <= '0;
      m_arsize  <= SIZE_8B; m_arburst <= BURST_INC; m_arvalid <= 1'b0;
      m_rready  <= 1'b0;
      m_awid    <= '0; m_awaddr <= '0; m_awlen <= '0;
      m_awsize  <= SIZE_8B; m_awburst <= BURST_INC; m_awvalid <= 1'b0;
      m_wdata   <= '0; m_wstrb <= '1; m_wlast <= 1'b0; m_wvalid <= 1'b0;
      m_bready  <= 1'b0;

      m_obj_wr_en   <= 1'b0;
      m_obj_wr_addr <= '0;
      m_obj_wr_data <= '0;
    end else begin
      eng_start   <= 1'b0;            // pulses
      dma_done    <= 1'b0;
      m_obj_wr_en <= 1'b0;

      case (mst)
        //------------------------------------------------------------------
        M_IDLE: begin
          if (dma_start) begin
            obj_base_r <= dma_obj_base;
            res_base_r <= dma_res_base;
            cnt_r      <= obj_count;
            rd_idx     <= '0;
            wr_idx     <= '0;
            mst        <= M_RD_AR;
          end
        end

        //------------------------------------------------------------------
        M_RD_AR: begin
          rd_beats  <= rd_beats_w;
          m_arid    <= '0;
          m_araddr  <= obj_base_r;
          m_arlen   <= rd_beats_w - 1'b1;  // AXI len = beats-1
          m_arsize  <= SIZE_8B;
          m_arburst <= BURST_INC;
          m_arvalid <= 1'b1;
          if (m_arvalid && m_arready) begin
            m_arvalid <= 1'b0;
            m_rready  <= 1'b1;
            mst       <= M_RD_DATA;
          end
        end

        //------------------------------------------------------------------
        M_RD_DATA: begin
          m_rready <= 1'b1;
          if (m_rvalid && m_rready) begin
            m_obj_wr_en   <= 1'b1;
            m_obj_wr_addr <= {rd_idx, 3'b000}; // beat*8 into buffer window
            m_obj_wr_data <= m_rdata;
            rd_idx        <= rd_idx + 1'b1;
            if (m_rlast) begin
              m_rready <= 1'b0;
              mst      <= M_RUN;
            end
          end
        end

        //------------------------------------------------------------------
        M_RUN: begin
          eng_start <= 1'b1;
          mst       <= M_WAIT_ENG;
        end

        //------------------------------------------------------------------
        M_WAIT_ENG: begin
          if (eng_done) mst <= M_WR_AW;
        end

        //------------------------------------------------------------------
        // Issue the write address and set up the FIRST data beat so WVALID /
        // WDATA / WLAST are presented together and held stable until WREADY
        // (AXI4 requires this; presenting beat0 here removes the one-cycle
        // WVALID bubble that a registered-output FSM would otherwise create).
        M_WR_AW: begin
          m_awid    <= '0;
          m_awaddr  <= res_base_r;
          m_awlen   <= 8'd1;               // 2 beats (result word0, word1)
          m_awsize  <= SIZE_8B;
          m_awburst <= BURST_INC;
          m_awvalid <= 1'b1;
          if (m_awvalid && m_awready) begin
            m_awvalid <= 1'b0;
            wr_idx    <= '0;
            m_wvalid  <= 1'b1;             // present beat 0 immediately
            m_wstrb   <= '1;
            m_wdata   <= res_word0;
            m_wlast   <= 1'b0;             // 2-beat burst: beat 0 is not last
            mst       <= M_WR_DATA;
          end
        end

        //------------------------------------------------------------------
        // Beat 0 is already on the bus; on each accepted beat advance to the
        // next (presenting WLAST with the final beat).
        M_WR_DATA: begin
          if (m_wvalid && m_wready) begin
            if (m_wlast) begin             // final beat accepted
              m_wvalid <= 1'b0;
              m_wlast  <= 1'b0;
              m_bready <= 1'b1;
              mst      <= M_WR_B;
            end else begin                 // beat 0 accepted -> present beat 1
              wr_idx   <= wr_idx + 1'b1;
              m_wdata  <= res_word1;
              m_wlast  <= 1'b1;            // beat 1 is last
            end
          end
        end

        //------------------------------------------------------------------
        M_WR_B: begin
          m_bready <= 1'b1;
          if (m_bvalid && m_bready) begin
            m_bready <= 1'b0;
            mst      <= M_DONE;
          end
        end

        //------------------------------------------------------------------
        M_DONE: begin
          dma_done <= 1'b1;
          mst      <= M_IDLE;
        end

        default: mst <= M_IDLE;
      endcase
    end
  end

  assign dma_busy = (mst != M_IDLE);

endmodule
