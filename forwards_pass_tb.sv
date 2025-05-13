`timescale 1ns/1ps
`include "forwards_pass.sv"

module forward_primal_update_seq_tb;
  parameter int STATE_DIM   = 12;
  parameter int CONTROL_DIM = 4;
  parameter int W           = 16;

  // Clock, reset, start
  logic clk, reset, start;

  // DUT inputs
  logic signed [W-1:0] x_k   [STATE_DIM];
  logic signed [W-1:0] d_k   [CONTROL_DIM];
  logic signed [W-1:0] Kinf  [CONTROL_DIM][STATE_DIM];
  logic signed [W-1:0] A_mat [STATE_DIM][STATE_DIM];
  logic signed [W-1:0] B_mat [STATE_DIM][CONTROL_DIM];

  // DUT outputs
  logic signed [W-1:0] u_k      [CONTROL_DIM];
  logic signed [W-1:0] x_next   [STATE_DIM];
  logic                done;

  // Expected results
  logic signed [W-1:0] exp_u      [CONTROL_DIM] = '{-16'sd1, -16'sd2, -16'sd3, -16'sd4};
  logic signed [W-1:0] exp_x_next[STATE_DIM]  = '{16'sd0, 16'sd0, 16'sd0, 16'sd0,
                                              16'sd5, 16'sd6, 16'sd7, 16'sd8,
                                              16'sd9,16'sd10,16'sd11,16'sd12};

  // Instantiate DUT
  forward_primal_update_seq #(
    .STATE_DIM(STATE_DIM),
    .CONTROL_DIM(CONTROL_DIM),
    .W(W)
  ) dut (
    .clk    (clk),
    .reset  (reset),
    .start  (start),
    .x_k    (x_k),
    .d_k    (d_k),
    .Kinf   (Kinf),
    .A_mat  (A_mat),
    .B_mat  (B_mat),
    .u_k    (u_k),
    .x_next(x_next),
    .done   (done)
  );

  // Clock generation (100MHz)
  initial clk = 0;
  always #5 clk = ~clk;

  integer i,j;
  initial begin
    // Initialize reset
    reset = 1; start = 0;
    #20;
    reset = 0;

    // Initialize inputs:
    // x_k = [1..12]
    for (i = 0; i < STATE_DIM; i++) x_k[i] = i + 1;
    // d_k = [1..4]
    for (i = 0; i < CONTROL_DIM; i++) d_k[i] = i + 1;
    // Kinf = zeros (so u_k = -d_k)
    for (i = 0; i < CONTROL_DIM; i++)
      for (j = 0; j < STATE_DIM; j++)
        Kinf[i][j] = 0;
    // A_mat = identity
    for (i = 0; i < STATE_DIM; i++)
      for (j = 0; j < STATE_DIM; j++)
        A_mat[i][j] = (i==j) ? 1 : 0;
    // B_mat: first CONTROL_DIM rows identity, others zero
    for (i = 0; i < STATE_DIM; i++)
      for (j = 0; j < CONTROL_DIM; j++)
        B_mat[i][j] = (i < CONTROL_DIM && i==j) ? 1 : 0;

    // Pulse start
    #10 start = 1;
    #10 start = 0;

    // Wait for done
    wait(done == 1);
    #5;

    // Display and check results
    $display("--- u_k outputs vs expected ---");
    for (i = 0; i < CONTROL_DIM; i++) begin
      $display("u_k[%0d] = %0d (exp %0d)", i, u_k[i], exp_u[i]);
    end
    $display("--- x_next outputs vs expected ---");
    for (i = 0; i < STATE_DIM; i++) begin
      $display("x_next[%0d] = %0d (exp %0d)", i, x_next[i], exp_x_next[i]);
    end

    #20 $finish;
  end
endmodule