//============================================================================
// taod_pkg.sv
//----------------------------------------------------------------------------
// Task-Aware Object Detection (TAOD) accelerator -- shared package.
//
// Project : EVIOS  -- DVCon India 2026 Design Contest
// Target  : CDAC VEGA AS1061 RISC-V SoC on Genesys2 (Kintex-7 XC7K325T)
//
// Contains: global parameters, the UQ2.14 fixed-point convention, and the
// two arithmetic helper functions (saturating requantize, 16x16 multiply)
// used by the scoring engine and the testbench golden model.
//
// Fixed-point convention
// ----------------------
//   All probability-like quantities (detector confidences, fused confidence,
//   task weights, temporal stability, area fraction) are UQ2.14:
//       16-bit unsigned, 2 integer bits + 14 fractional bits.
//       Q_ONE = 16384 represents the value 1.0.
//   Products are kept WIDE (32-bit, UQ4.28) and only requantized back to
//   UQ2.14 via q_sat() where a value must re-enter the 16-bit datapath.
//   The final per-object score is kept as a full 32-bit UQ4.28 value so the
//   argmax comparison never loses ranking fidelity.
//============================================================================
`ifndef TAOD_PKG_SV
`define TAOD_PKG_SV

package taod_pkg;

  //--------------------------------------------------------------------------
  // Fixed-point format
  //--------------------------------------------------------------------------
  localparam int unsigned FRAC  = 14;            // fractional bits
  localparam int unsigned QW    = 16;            // probability word width
  localparam logic [QW-1:0] Q_ONE = 16'd16384;   // 1.0 in UQ2.14
  localparam logic [QW-1:0] Q_MAX = 16'hFFFF;    // saturation ceiling (UQ2.14)

  //--------------------------------------------------------------------------
  // Problem dimensions
  //--------------------------------------------------------------------------
  localparam int unsigned NUM_TASKS = 16;        // 14 contest tasks, 16 slots
  localparam int unsigned NUM_CLASS = 80;        // COCO classes
  localparam int unsigned MAX_OBJ   = 64;        // detections per frame
  localparam int unsigned NUM_TRACK = 256;       // temporal-memory tracks

  //--------------------------------------------------------------------------
  // Derived field widths
  //--------------------------------------------------------------------------
  localparam int unsigned TASK_W  = 4;           // task id          (0..15)
  localparam int unsigned CLASS_W = 8;           // class id          (0..79, 8b room)
  localparam int unsigned IDX_W   = 6;           // object index      (0..63)
  localparam int unsigned CNT_W   = 7;           // object count      (0..64)
  localparam int unsigned TRK_W   = 8;           // track id          (0..255)
  localparam int unsigned COORD_W = 16;          // bbox coordinate   (px)

  // Weight LUT: NUM_TASKS * NUM_CLASS entries -> 16*80 = 1280 deep, 11-bit addr
  localparam int unsigned WMEM_DEPTH = NUM_TASKS * NUM_CLASS; // 1280
  localparam int unsigned WMEM_AW    = 11;                    // ceil(log2(1280))

  //--------------------------------------------------------------------------
  // AXI4 parameters (match VEGA accelerator interface)
  //   VEGA presents 64-bit address; the accelerator only decodes the low 32.
  //--------------------------------------------------------------------------
  localparam int unsigned AXI_ID_W   = 12;
  localparam int unsigned AXI_ADDR_W = 32;       // internal decode width
  localparam int unsigned AXI_DATA_W = 64;
  localparam int unsigned AXI_STRB_W = AXI_DATA_W/8; // 8

  //--------------------------------------------------------------------------
  // q_sat : saturating requantize of a 32-bit UQ4.28 product back to UQ2.14.
  //   Take bits [FRAC +: QW] = [29:14]; if any bit above bit 29 is set the
  //   value is >= 4.0 and we saturate to Q_MAX.  Used after every multiply
  //   that must re-enter the 16-bit datapath.
  //--------------------------------------------------------------------------
  function automatic logic [QW-1:0] q_sat(input logic [31:0] prod);
    begin
      // overflow when value >= 4.0, i.e. prod >= 2^(FRAC+QW)=2^30
      if (prod >= (32'd1 << (FRAC+QW))) q_sat = Q_MAX;
      else                              q_sat = QW'(prod >> FRAC); // == prod[29:14]
    end
  endfunction

  //--------------------------------------------------------------------------
  // mul16 : unsigned 16x16 -> 32 multiply.  Max product 65535^2 < 2^32, so
  //   the full result always fits in 32 bits with no truncation.
  //--------------------------------------------------------------------------
  function automatic logic [31:0] mul16(input logic [QW-1:0] a,
                                        input logic [QW-1:0] b);
    begin
      mul16 = a * b;
    end
  endfunction

endpackage : taod_pkg

`endif
