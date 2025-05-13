
module forward_primal_update_seq #(
  parameter int STATE_DIM   = 12,
  parameter int CONTROL_DIM = 4,
  parameter int W           = 16
)(
  input  logic                         clk,
  input  logic                         reset,
  input  logic                         start,

  // inputs
  input  logic signed [W-1:0]          x_k       [STATE_DIM],
  input  logic signed [W-1:0]          d_k       [CONTROL_DIM],
  input  logic signed [W-1:0]          Kinf      [CONTROL_DIM][STATE_DIM],
  input  logic signed [W-1:0]          A_mat     [STATE_DIM][STATE_DIM],
  input  logic signed [W-1:0]          B_mat     [STATE_DIM][CONTROL_DIM],

  // outputs
  output logic signed [W-1:0]          u_k       [CONTROL_DIM],
  output logic signed [W-1:0]          x_next    [STATE_DIM],
  output logic                         done
);

  // FSM states
  typedef enum logic [3:0] {
    IDLE,
    U_ACC,   U_STORE,
    AX_ACC,  AX_STORE,
    BU_ACC,  BU_STORE,
    COMBINE, DONE_STATE
  } state_t;

  state_t state;

  // Index counters
  localparam int LG_CTRL = $clog2(CONTROL_DIM);
  localparam int LG_STAT = $clog2(STATE_DIM);

  logic [LG_CTRL-1:0]   idx_ctrl;   // row index for u_k phase
  logic [LG_STAT-1:0]   idx_stat;   // row index for a_x and b_u phases
  logic [$clog2( (CONTROL_DIM>STATE_DIM)?CONTROL_DIM:STATE_DIM)-1:0]
                         idx_col;   // column index for inner loops

  // Accumulator
  logic signed [2*W-1:0] acc;

  // Temporaries
  logic signed [W-1:0] a_x   [STATE_DIM];
  logic signed [W-1:0] b_u   [STATE_DIM];

  integer i;

  always_ff @(posedge clk or posedge reset) begin
    if (reset) begin
      state     <= IDLE;
      idx_ctrl  <= '0;
      idx_stat  <= '0;
      idx_col   <= '0;
      acc        <= '0;
      done       <= 1'b0;
      for (i = 0; i < CONTROL_DIM; i++) u_k[i] <= '0;
      for (i = 0; i < STATE_DIM;   i++) x_next[i] <= '0;
    end else begin
      
      case (state)
        // Wait for start pulse
        IDLE: if (start) begin
                 done <= 1'b0;
                 idx_ctrl <= 0;
                 idx_col  <= 0;
                 acc      <= '0;
                 state    <= U_ACC;
               end

        // Phase 1: accumulate Kinf·x_k for row idx_ctrl
        U_ACC: begin
          acc <= acc + Kinf[idx_ctrl][idx_col] * x_k[idx_col];
          if (idx_col == STATE_DIM-1) state <= U_STORE;
          idx_col <= idx_col + 1;
        end

        // Store u_k[idx_ctrl] and advance or move to next phase
        U_STORE: begin
          u_k[idx_ctrl] <= -acc[W-1:0] - d_k[idx_ctrl];
          if (idx_ctrl == CONTROL_DIM-1) begin
            idx_stat <= 0;
            idx_col  <= 0;
            acc      <= '0;
            state    <= AX_ACC;
          end else begin
            idx_ctrl <= idx_ctrl + 1;
            idx_col  <= 0;
            acc      <= '0;
            state    <= U_ACC;
          end
        end

        // Phase 2: accumulate A_mat·x_k for row idx_stat
        AX_ACC: begin
          acc <= acc + A_mat[idx_stat][idx_col] * x_k[idx_col];
          if (idx_col == STATE_DIM-1) state <= AX_STORE;
          idx_col <= idx_col + 1;
        end

        // Store a_x[idx_stat]
        AX_STORE: begin
          a_x[idx_stat] <= acc[W-1:0];
          if (idx_stat == STATE_DIM-1) begin
            idx_stat <= 0;
            idx_col  <= 0;
            acc      <= '0;
            state    <= BU_ACC;
          end else begin
            idx_stat <= idx_stat + 1;
            idx_col  <= 0;
            acc      <= '0;
            state    <= AX_ACC;
          end
        end

        // Phase 3: accumulate B_mat·u_k for row idx_stat
        BU_ACC: begin
          acc <= acc + B_mat[idx_stat][idx_col] * u_k[idx_col];
          if (idx_col == CONTROL_DIM-1) state <= BU_STORE;
          idx_col <= idx_col + 1;
        end

        // Store b_u[idx_stat]
        BU_STORE: begin
          b_u[idx_stat] <= acc[W-1:0];
          if (idx_stat == STATE_DIM-1) begin
            idx_stat <= 0;
            state    <= COMBINE;
          end else begin
            idx_stat <= idx_stat + 1;
            idx_col  <= 0;
            acc      <= '0;
            state    <= BU_ACC;
          end
        end

        // Phase 4: combine into x_next
        COMBINE: begin
          x_next[idx_stat] <= a_x[idx_stat] + b_u[idx_stat];
          if (idx_stat == STATE_DIM-1) begin
            done  <= 1'b1;
            state <= DONE_STATE;
          end else begin
            idx_stat <= idx_stat + 1;
          end
        end

        // Back to IDLE when start deasserts
        DONE_STATE: if (!start) state <= IDLE;

        default: state <= IDLE;
      endcase
    end
  end
endmodule