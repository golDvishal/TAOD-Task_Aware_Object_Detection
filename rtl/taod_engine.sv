//============================================================================
// taod_engine.sv
//----------------------------------------------------------------------------
// Task-Aware scoring + selection engine (the heart of the accelerator).
//
// For each valid object in the frame it computes, in fixed-point UQ2.14:
//
//     Fconf   = sat( w1*C1 + w2*C2 )                 ensemble fusion
//     Wtask   = weightLUT[task_id, class_id]         task-conditioned relevance
//     Tstable = EMA( Fconf )  per track               temporal stability
//     areaTerm= 1 + clamp( area / frameArea , 1 )     spatial accessibility
//     Score   = Fconf * Wtask * Tstable * areaTerm    (kept 32-bit UQ4.28)
//
// and streams an argmax to pick BestObject = argmax(Score).  The winning and
// runner-up scores are both emitted so software can report the decision margin
// (explainability).
//
// Implementation: iterative FSM, ~6 cycles/object.  One object is in flight at
// a time, which makes the temporal read/modify/write hazard-free and the whole
// engine straightforward to verify.  The arithmetic is bit-identical to a
// fully-pipelined 1-obj/cycle version (documented as the scalability path);
// the CNN dominates system latency, so right-sizing this block is deliberate.
//
// Temporal stability uses a per-track EMA leaky integrator stored in BRAM:
//     T_new = T_old + ((Fconf - T_old) >>> alpha) , alpha = log2(N)
// i.e. O(1) state per track (one value + one valid bit), no N-deep history.
// A per-track valid bit gives correct cold-start (first sighting: T_new=Fconf).
//
// Memory ports (driven so 1-cycle synchronous reads land in the next state):
//   obj_*   : object record fields, unpacked by the top level (no structs on
//             the port boundary, for tool portability).
//   w_*     : task x class weight LUT (owned by top, read here).
//   temporal: 1R1W BRAM owned INSIDE the engine (so the clear sweep lives here).
//============================================================================/
module taod_engine
  import taod_pkg::*;
(
  input  logic                  clk,
  input  logic                  rst_n,

  // control
  input  logic                  start,        // 1-cycle pulse: begin a frame
  input  logic                  clear,        // 1-cycle pulse: sweep temporal mem
  input  logic [TASK_W-1:0]     task_id,
  input  logic [CNT_W-1:0]      obj_count,    // number of objects this frame
  input  logic [QW-1:0]         w1,           // fusion weight detector 1 (UQ2.14)
  input  logic [QW-1:0]         w2,           // fusion weight detector 2 (UQ2.14)
  input  logic [31:0]           inv_fa,       // floor(2^32 / frame_area)
  input  logic [TASK_W-1:0]     alpha,        // EMA shift = log2(N)

  // object-memory read port (top unpacks the 128-bit record into these)
  output logic [IDX_W-1:0]      obj_addr,     // index of object to fetch
  input  logic [QW-1:0]         obj_c1,
  input  logic [QW-1:0]         obj_c2,
  input  logic [CLASS_W-1:0]    obj_class,
  input  logic [TRK_W-1:0]      obj_track,
  input  logic                  obj_valid,
  input  logic [COORD_W-1:0]    obj_x1,
  input  logic [COORD_W-1:0]    obj_y1,
  input  logic [COORD_W-1:0]    obj_x2,
  input  logic [COORD_W-1:0]    obj_y2,

  // weight-LUT read port (owned by top)
  output logic [WMEM_AW-1:0]    w_addr,
  input  logic [QW-1:0]         w_data,

  // status
  output logic                  busy,
  output logic                  done,         // 1-cycle pulse on completion
  output logic                  result_valid,

  // results (held stable until next start)
  output logic [IDX_W-1:0]      res_idx,
  output logic [CLASS_W-1:0]    res_class,
  output logic [31:0]           res_score,    // winning  score  (UQ4.28)
  output logic [31:0]           res_runner,   // runner-up score  (UQ4.28)
  output logic [COORD_W-1:0]    res_x1,
  output logic [COORD_W-1:0]    res_y1,
  output logic [COORD_W-1:0]    res_x2,
  output logic [COORD_W-1:0]    res_y2
);

  //--------------------------------------------------------------------------
  // Temporal memory : NUM_TRACK x (QW+1).  Bit QW is the per-track valid flag.
  // 1R1W: continuous read of tmem[obj_track]; single write in E_C1 / E_CLR.
  //--------------------------------------------------------------------------
  logic [QW:0] tmem [0:NUM_TRACK-1];
  logic [QW:0] tmem_rd_q;                 // registered read data

  //--------------------------------------------------------------------------
  // FSM states
  //--------------------------------------------------------------------------
  typedef enum logic [3:0] {
    E_IDLE,   // wait for start / clear
    E_CLR,    // sweep temporal memory to 0
    E_FETCH,  // drive obj_addr = idx
    E_OBJW,   // capture object fields; launch weight + temporal reads
    E_MEMW,   // capture weight + temporal data; compute Fconf, area
    E_C1,     // areaTerm, EMA write-back, p1 = sat(Fconf*Wtask)
    E_C2,     // p2 = sat(p1*Tstable), score = p2*areaTerm
    E_ARG,    // argmax update; advance index
    E_DONE    // latch outputs, pulse done
  } state_t;

  state_t st;

  //--------------------------------------------------------------------------
  // Iteration / sweep counters
  //--------------------------------------------------------------------------
  logic [IDX_W-1:0]   idx;        // current object index
  logic [CNT_W-1:0]   cnt;        // object count latched at start
  logic [TRK_W:0]     clr_idx;    // temporal-clear sweep counter (0..NUM_TRACK)

  //--------------------------------------------------------------------------
  // Per-object captured fields / pipeline registers
  //--------------------------------------------------------------------------
  logic [QW-1:0]      c1_r, c2_r;
  logic [CLASS_W-1:0] class_r;
  logic [TRK_W-1:0]   track_r;
  logic               objv_r;
  logic [COORD_W-1:0] x1_r, y1_r, x2_r, y2_r;

  logic [QW-1:0]      wtask_r;          // task x class weight
  logic [QW-1:0]      t_old_r;          // previous temporal value
  logic               t_valid_old;      // previous temporal valid bit

  logic [QW-1:0]      fconf_r;          // fused confidence
  logic [31:0]        area_r;           // raw pixel area
  logic [QW-1:0]      areaterm_r;       // 1 + areaFrac  (UQ2.14, [1,2))
  logic [QW-1:0]      tnew_r;           // EMA result written back
  logic [QW-1:0]      p1_r;             // sat(Fconf*Wtask)
  logic [QW-1:0]      tstable_r;        // temporal stability used in score

  //--------------------------------------------------------------------------
  // argmax accumulators
  //--------------------------------------------------------------------------
  logic               best_set;
  logic [31:0]        best_score, second_score;
  logic [IDX_W-1:0]   best_idx;
  logic [CLASS_W-1:0] best_class;
  logic [COORD_W-1:0] best_x1, best_y1, best_x2, best_y2;

  //==========================================================================
  // Combinational datapath helpers
  //==========================================================================
  // Fused confidence: w1*c1 + w2*c2 (two UQ4.28 products summed), requantized.
  // Same saturate-at-4.0 rule as q_sat, applied to the wide sum directly.
  logic [63:0]   fsum;
  logic [QW-1:0] fconf_calc;
  always_comb begin
    fsum       = mul16(w1, c1_r) + mul16(w2, c2_r);
    fconf_calc = (fsum >= (64'd1 << (FRAC+QW))) ? Q_MAX : QW'(fsum >> FRAC);
  end

  // Area = (x2-x1)*(y2-y1), clamping each delta to 0 if negative.
  logic [COORD_W-1:0] dx, dy;
  logic [31:0]        area_calc;
  always_comb begin
    dx        = (x2_r > x1_r) ? (x2_r - x1_r) : '0;
    dy        = (y2_r > y1_r) ? (y2_r - y1_r) : '0;
    area_calc = dx * dy;                              // <= 32 bits
  end

  // areaTerm = 1 + clamp(area*inv_fa >> 18 , 1.0).  area*inv_fa needs 64 bits.
  // A single compare handles both the >=2^16 overflow and the >1.0 clamp,
  // since anything >= 2^16 is also > Q_ONE.
  logic [63:0]   aw64;
  logic [QW-1:0] aw_q;
  logic [QW-1:0] areaterm_calc;
  always_comb begin
    aw64          = (area_r * inv_fa) >> 18;          // -> UQ2.14 scale
    aw_q          = (aw64 > 64'(Q_ONE)) ? Q_ONE : QW'(aw64);
    areaterm_calc = Q_ONE + aw_q;                     // [1.0, 2.0]  fits 16b
  end

  // EMA: t_new = t_old + ((fconf - t_old) >>> alpha), signed, then clamp >=0.
  logic signed [QW+1:0] ema_diff, ema_sh, ema_new;
  logic [QW-1:0]        ema_clamped;
  logic [QW-1:0]        tnew_calc;
  always_comb begin
    ema_diff    = $signed({2'b00, fconf_r}) - $signed({2'b00, t_old_r});
    ema_sh      = ema_diff >>> alpha;                 // arithmetic shift
    ema_new     = $signed({2'b00, t_old_r}) + ema_sh;
    if (ema_new < 0)              ema_clamped = '0;
    else if (ema_new > $signed({2'b00, Q_MAX}))
                                  ema_clamped = Q_MAX;
    else                          ema_clamped = QW'(ema_new);
    // cold start: first sighting of this track uses raw fused confidence
    tnew_calc   = t_valid_old ? ema_clamped : fconf_r;
  end

  // Score stage 2 combinational pieces
  logic [QW-1:0] p2_calc;
  logic [31:0]   score_calc;
  always_comb begin
    p2_calc    = q_sat(mul16(p1_r, tstable_r));
    score_calc = mul16(p2_calc, areaterm_r);          // 32-bit UQ4.28
  end

  //==========================================================================
  // Weight-LUT address (combinational): task_id*NUM_CLASS + class.
  //   task_id(4b)*80 + class(8b) <= 15*80+79 = 1279 -> fits WMEM_AW=11.
  //==========================================================================
  always_comb begin
    obj_addr = idx;
    w_addr   = (st == E_OBJW)
             ? WMEM_AW'(task_id * NUM_CLASS + obj_class)
             : '0;
  end

  //==========================================================================
  // Temporal memory : registered read + single write port
  //==========================================================================
  always_ff @(posedge clk) begin
    // continuous synchronous read (old-data-on-collision is fine: one object
    // in flight at a time, so a read never races its own write)
    tmem_rd_q <= tmem[obj_track];

    if (st == E_CLR) begin
      tmem[TRK_W'(clr_idx)] <= '0;             // clear value + valid bit
    end else if (st == E_C1) begin
      tmem[track_r] <= {1'b1, tnew_calc};      // mark valid, store EMA
    end
  end

  //==========================================================================
  // Main FSM
  //==========================================================================
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      st           <= E_IDLE;
      idx          <= '0;
      cnt          <= '0;
      clr_idx      <= '0;
      best_set     <= 1'b0;
      best_score   <= '0;
      second_score <= '0;
      best_idx     <= '0;
      best_class   <= '0;
      best_x1      <= '0; best_y1 <= '0; best_x2 <= '0; best_y2 <= '0;
      done         <= 1'b0;
      result_valid <= 1'b0;
      res_idx      <= '0;
      res_class    <= '0;
      res_score    <= '0;
      res_runner   <= '0;
      res_x1       <= '0; res_y1 <= '0; res_x2 <= '0; res_y2 <= '0;
      // pipeline staging registers (reset for clean power-up / no 8-5788)
      c1_r         <= '0; c2_r <= '0; class_r <= '0; track_r <= '0;
      objv_r       <= 1'b0;
      x1_r         <= '0; y1_r <= '0; x2_r <= '0; y2_r <= '0;
      wtask_r      <= '0; t_old_r <= '0; t_valid_old <= 1'b0;
      fconf_r      <= '0; area_r <= '0;
      areaterm_r   <= '0; tnew_r <= '0; tstable_r <= '0; p1_r <= '0;
    end else begin
      done <= 1'b0;                            // default: done is a pulse

      case (st)
        //------------------------------------------------------------------
        E_IDLE: begin
          if (clear) begin
            clr_idx <= '0;
            st      <= E_CLR;
          end else if (start) begin
            cnt          <= obj_count;
            idx          <= '0;
            best_set     <= 1'b0;
            best_score   <= '0;
            second_score <= '0;
            if (obj_count == '0) st <= E_DONE; // empty frame
            else                 st <= E_FETCH;
          end
        end

        //------------------------------------------------------------------
        // Sweep the temporal memory.  One entry per cycle.
        E_CLR: begin
          if (clr_idx == (NUM_TRACK-1)) st <= E_IDLE;
          clr_idx <= clr_idx + 1'b1;
        end

        //------------------------------------------------------------------
        // obj_addr = idx was driven this cycle; data lands next cycle.
        E_FETCH: st <= E_OBJW;

        //------------------------------------------------------------------
        // Object fields valid now.  Skip invalid ones.  Launch weight +
        // temporal reads (addresses are driven combinationally / by obj_track).
        E_OBJW: begin
          c1_r    <= obj_c1;
          c2_r    <= obj_c2;
          class_r <= obj_class;
          track_r <= obj_track;
          objv_r  <= obj_valid;
          x1_r    <= obj_x1; y1_r <= obj_y1;
          x2_r    <= obj_x2; y2_r <= obj_y2;

          if (!obj_valid) begin
            if (idx == (cnt-1)) st <= E_DONE;
            else begin idx <= idx + 1'b1; st <= E_FETCH; end
          end else begin
            st <= E_MEMW;
          end
        end

        //------------------------------------------------------------------
        // Weight + temporal data valid now.  Compute Fconf and area.
        E_MEMW: begin
          wtask_r     <= w_data;
          t_old_r     <= QW'(tmem_rd_q);          // low QW bits = stored value
          t_valid_old <= (tmem_rd_q >> QW);       // bit QW = valid flag
          fconf_r     <= fconf_calc;
          area_r      <= area_calc;
          st          <= E_C1;
        end

        //------------------------------------------------------------------
        // areaTerm; EMA write-back (handled in the tmem block); p1.
        E_C1: begin
          areaterm_r <= areaterm_calc;
          tnew_r     <= tnew_calc;
          tstable_r  <= tnew_calc;                 // score uses the new EMA
          p1_r       <= q_sat(mul16(fconf_r, wtask_r));
          st         <= E_C2;
        end

        //------------------------------------------------------------------
        // p2 and final score.
        E_C2: begin
          st <= E_ARG;
        end

        //------------------------------------------------------------------
        // argmax.  Strict '>' keeps the first object on ties.
        E_ARG: begin
          if (!best_set || (score_calc > best_score)) begin
            second_score <= best_set ? best_score : '0;
            best_score   <= score_calc;
            best_idx     <= idx;
            best_class   <= class_r;
            best_x1      <= x1_r; best_y1 <= y1_r;
            best_x2      <= x2_r; best_y2 <= y2_r;
            best_set     <= 1'b1;
          end else if (score_calc > second_score) begin
            second_score <= score_calc;            // track runner-up
          end

          if (idx == (cnt-1)) st <= E_DONE;
          else begin idx <= idx + 1'b1; st <= E_FETCH; end
        end

        //------------------------------------------------------------------
        // Latch results, pulse done.
        E_DONE: begin
          res_idx      <= best_idx;
          res_class    <= best_class;
          res_score    <= best_score;
          res_runner   <= second_score;
          res_x1       <= best_x1; res_y1 <= best_y1;
          res_x2       <= best_x2; res_y2 <= best_y2;
          result_valid <= best_set;
          done         <= 1'b1;
          st           <= E_IDLE;
        end

        default: st <= E_IDLE;
      endcase
    end
  end

  assign busy = (st != E_IDLE);

endmodule