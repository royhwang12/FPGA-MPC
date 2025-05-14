`timescale 1ns/1ps
`include "slack_update.sv"

module slack_update_tb;
  parameter STATE_DIM   = 12;
  parameter CONTROL_DIM = 4;
  parameter W           = 16;

  logic clk, reset, start;

  typedef logic signed [W-1:0] fixed_t;

  fixed_t x_k   [STATE_DIM];
  fixed_t y_k   [STATE_DIM];
  fixed_t u_k   [CONTROL_DIM];
  fixed_t g_k   [CONTROL_DIM];

  fixed_t x_min, x_max;
  fixed_t u_min, u_max;

  fixed_t v_k   [STATE_DIM];
  fixed_t z_k   [CONTROL_DIM];
  logic    done;

  slack_update #(
    .STATE_DIM(STATE_DIM),
    .CONTROL_DIM(CONTROL_DIM),
    .W(W)
  ) dut (
    .clk    (clk),
    .reset  (reset),
    .start  (start),
    .x_k    (x_k),
    .y_k    (y_k),
    .u_k    (u_k),
    .g_k    (g_k),
    .x_min  (x_min),
    .x_max  (x_max),
    .u_min  (u_min),
    .u_max  (u_max),
    .v_k    (v_k),
    .z_k    (z_k),
    .done   (done)
  );

  initial clk = 0;
  always #5 clk = ~clk;

  initial begin
    integer i;
    reset = 1;
    start = 0;
    u_min = 5; u_max =  6;
    x_min = 10; x_max =  12;
    #20;
    reset = 0;

    u_k = '{16'sd1, 16'sd2, 16'sd3, 16'sd4};
    g_k = '{16'sd4, 16'sd3, 16'sd2, 16'sd1};

    x_k = '{16'sd1, 16'sd2, 16'sd3, 16'sd4,
            16'sd5, 16'sd6, 16'sd7, 16'sd8,
            16'sd9, 16'sd10,16'sd11,16'sd12};
    y_k = '{16'sd12,16'sd11,16'sd10,16'sd9,
            16'sd8, 16'sd7, 16'sd6, 16'sd5,
            16'sd4, 16'sd3, 16'sd2, 16'sd1};

    #10 start = 1;
    #10 start = 0;

    wait(done);

    $display("--- v_k (state slack) ---");
    for (i = 0; i < STATE_DIM; i++)
      $display("v_k[%0d] = %0d", i, v_k[i]);

    $display("--- z_k (control slack) ---");
    for (i = 0; i < CONTROL_DIM; i++)
      $display("z_k[%0d] = %0d", i, z_k[i]);

    #20 $finish;
  end
endmodule
