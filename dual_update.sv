`timescale 1ns/1ps

module dual_update #(
    parameter int STATE_DIM   = 6,
    parameter int CONTROL_DIM = 12,
    parameter int W           = 16
) (
    input  logic                         clk,
    input  logic                         reset,
    input  logic                         start,

    // state‐dual inputs
    input  logic signed [W-1:0]          u_k   [STATE_DIM],
    input  logic signed [W-1:0]          z_k   [STATE_DIM],
    input  logic signed [W-1:0]          y_k   [STATE_DIM],

    // control‐dual inputs
    input  logic signed [W-1:0]          x_k   [CONTROL_DIM],
    input  logic signed [W-1:0]          v_k   [CONTROL_DIM],
    input  logic signed [W-1:0]          g_k   [CONTROL_DIM],

    // updated dual outputs
    output logic signed [W-1:0]          y_out [STATE_DIM],
    output logic signed [W-1:0]          g_out [CONTROL_DIM],
    output logic                         done
);

  // next‐value comb logic
  logic signed [W-1:0] y_calc [STATE_DIM];
  logic signed [W-1:0] g_calc [CONTROL_DIM];

  generate
    genvar i;
    for (i = 0; i < STATE_DIM; i++) begin : STATE_DUAL
      assign y_calc[i] = y_k[i] + (u_k[i] - z_k[i]);
    end
    for (i = 0; i < CONTROL_DIM; i++) begin : CTRL_DUAL
      assign g_calc[i] = g_k[i] + (x_k[i] - v_k[i]);
    end
  endgenerate

  // register stage
  always_ff @(posedge clk or posedge reset) begin
    if (reset) begin
      y_out <= '{default:'0};
      g_out <= '{default:'0};
      done  <= 1'b0;
    end
    else if (start) begin
      y_out <= y_calc;
      g_out <= g_calc;
      done  <= 1'b1;
    end
    else begin
      done <= 1'b0;
    end
  end

endmodule