module slack_update #(
  parameter int STATE_DIM   = 12,
  parameter int CONTROL_DIM = 4,
  parameter int W           = 16
)(
  input  logic                   clk,
  input  logic                   reset,
  input  logic                   start,

  input  logic signed [W-1:0]    x_k   [STATE_DIM],
  input  logic signed [W-1:0]    y_k   [STATE_DIM],
  input  logic signed [W-1:0]    u_k   [CONTROL_DIM],
  input  logic signed [W-1:0]    g_k   [CONTROL_DIM],

  input  logic signed [W-1:0]    x_min,
  input  logic signed [W-1:0]    x_max,
  input  logic signed [W-1:0]    u_min,
  input  logic signed [W-1:0]    u_max,

  output logic signed [W-1:0]    v_k   [STATE_DIM],
  output logic signed [W-1:0]    z_k   [CONTROL_DIM],
  output logic                   done
);

  logic signed [W-1:0] v_calc [STATE_DIM];
  logic signed [W-1:0] z_calc [CONTROL_DIM];

  genvar i;
  generate
    for (i = 0; i < STATE_DIM; i++) begin : STATE_CLIP
      logic signed [W-1:0] tmp;
      assign tmp         = x_k[i] + y_k[i];
      assign v_calc[i]   = (tmp <  x_min) ? x_min
                         : (tmp >  x_max) ? x_max
                         :                 tmp;
    end

    for (i = 0; i < CONTROL_DIM; i++) begin : CTRL_CLIP
      logic signed [W-1:0] tmp2;
      assign tmp2        = u_k[i] + g_k[i];
      assign z_calc[i]   = (tmp2 <  u_min) ? u_min
                         : (tmp2 >  u_max) ? u_max
                         :                  tmp2;
    end
  endgenerate

  always_ff @(posedge clk or posedge reset) begin
    if (reset) begin
      v_k  <= '{default:'0};
      z_k  <= '{default:'0};
      //start = 1'b0;
      done <= 1'b0;
    end
    else if (start) begin
      v_k  <= v_calc;
      z_k  <= z_calc;
      done <= 1'b1;
    end
    else begin
      done <= 1'b0;
    end
  end

endmodule
