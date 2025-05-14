`timescale 1ns/1ps

module dual_update #(
    parameter int CONTROL_DIM     = 4,
    parameter int STATE_DIM       = 12,
    parameter int W               = 16,
    parameter int EXT_DATA_WIDTH  = 32,
    parameter int ADDR_WIDTH      = 16
)(
    input  logic                   clk,
    input  logic                   reset,

    // Avalon memory-mapped slave interface
    input  logic [EXT_DATA_WIDTH-1:0] writedata,
    input  logic read,
    input  logic write,
    input  logic [ADDR_WIDTH-1:0] addr,
    input  logic chipselect,
    output logic [EXT_DATA_WIDTH-1:0] readdata
);

  // Internal control
  logic start, done;

  // Flattened state-dual inputs
  logic signed [W*CONTROL_DIM-1:0] u_k_flat, z_k_flat, y_k_flat;
  logic signed [W*STATE_DIM-1:0]   x_k_flat, v_k_flat, g_k_flat;

  // Flattened outputs
  logic signed [W*CONTROL_DIM-1:0] y_out_flat;
  logic signed [W*STATE_DIM-1:0]   g_out_flat;

  // Internal unpacked arrays
  logic signed [W-1:0] u_k     [CONTROL_DIM];
  logic signed [W-1:0] z_k     [CONTROL_DIM];
  logic signed [W-1:0] y_k     [CONTROL_DIM];
  logic signed [W-1:0] y_calc  [CONTROL_DIM];
  logic signed [W-1:0] y_out   [CONTROL_DIM];

  logic signed [W-1:0] x_k     [STATE_DIM];
  logic signed [W-1:0] v_k     [STATE_DIM];
  logic signed [W-1:0] g_k     [STATE_DIM];
  logic signed [W-1:0] g_calc  [STATE_DIM];
  logic signed [W-1:0] g_out   [STATE_DIM];

  // Unpack flattened inputs
  always_comb begin
    for (int i = 0; i < CONTROL_DIM; i++) begin
      u_k[i] = u_k_flat[i*W +: W];
      z_k[i] = z_k_flat[i*W +: W];
      y_k[i] = y_k_flat[i*W +: W];
    end
    for (int i = 0; i < STATE_DIM; i++) begin
      x_k[i] = x_k_flat[i*W +: W];
      v_k[i] = v_k_flat[i*W +: W];
      g_k[i] = g_k_flat[i*W +: W];
    end
  end

  // Combinational updates
  generate
    genvar i;
    for (i = 0; i < CONTROL_DIM; i++) begin : y_update
      assign y_calc[i] = y_k[i] + (u_k[i] - z_k[i]);
    end
    for (i = 0; i < STATE_DIM; i++) begin : g_update
      assign g_calc[i] = g_k[i] + (x_k[i] - v_k[i]);
    end
  endgenerate

  // Sequential register updates
  always_ff @(posedge clk or posedge reset) begin
    if (reset) begin
      for (int i = 0; i < CONTROL_DIM; i++) y_out[i] <= '0;
      for (int i = 0; i < STATE_DIM; i++) g_out[i] <= '0;
      done <= 1'b0;
    end else if (start) begin
      for (int i = 0; i < CONTROL_DIM; i++) y_out[i] <= y_calc[i];
      for (int i = 0; i < STATE_DIM; i++) g_out[i] <= g_calc[i];
      done <= 1'b1;
    end else begin
      done <= 1'b0;
    end
  end

  // Pack outputs
  always_comb begin
    for (int i = 0; i < CONTROL_DIM; i++) begin
      y_out_flat[i*W +: W] = y_out[i];
    end
    for (int i = 0; i < STATE_DIM; i++) begin
      g_out_flat[i*W +: W] = g_out[i];
    end
  end

  // Avalon write interface
  always_ff @(posedge clk or posedge reset) begin
    if (reset) begin
      start <= 1'b0;
      u_k_flat <= '0;
      z_k_flat <= '0;
      y_k_flat <= '0;
      x_k_flat <= '0;
      v_k_flat <= '0;
      g_k_flat <= '0;
    end else if (chipselect && write) begin
      case (addr)
        16'h0000: start <= writedata[0];

        // u_k_flat (64-bit)
        16'h0010: u_k_flat[31:0]     <= writedata;
        16'h0014: u_k_flat[63:32]    <= writedata;

        // z_k_flat (64-bit)
        16'h0018: z_k_flat[31:0]     <= writedata;
        16'h001C: z_k_flat[63:32]    <= writedata;

        // y_k_flat (64-bit)
        16'h0020: y_k_flat[31:0]     <= writedata;
        16'h0024: y_k_flat[63:32]    <= writedata;

        // x_k_flat (192-bit)
        16'h0030: x_k_flat[31:0]     <= writedata;
        16'h0032: x_k_flat[63:32]    <= writedata;
        16'h0034: x_k_flat[95:64]    <= writedata;
        16'h0036: x_k_flat[127:96]   <= writedata;
        16'h0038: x_k_flat[159:128]  <= writedata;
        16'h003A: x_k_flat[191:160]  <= writedata;

        // v_k_flat (192-bit)
        16'h0040: v_k_flat[31:0]     <= writedata;
        16'h0042: v_k_flat[63:32]    <= writedata;
        16'h0044: v_k_flat[95:64]    <= writedata;
        16'h0046: v_k_flat[127:96]   <= writedata;
        16'h0048: v_k_flat[159:128]  <= writedata;
        16'h004A: v_k_flat[191:160]  <= writedata;

        // g_k_flat (192-bit)
        16'h0050: g_k_flat[31:0]     <= writedata;
        16'h0052: g_k_flat[63:32]    <= writedata;
        16'h0054: g_k_flat[95:64]    <= writedata;
        16'h0056: g_k_flat[127:96]   <= writedata;
        16'h0058: g_k_flat[159:128]  <= writedata;
        16'h005A: g_k_flat[191:160]  <= writedata;

        default:;
      endcase
    end
  end

  // Avalon read interface
  always_ff @(posedge clk) begin
    if (chipselect && read) begin
      case (addr)
        16'h0000: readdata <= {{(EXT_DATA_WIDTH-1){1'b0}}, done};

        // y_out_flat (64-bit)
        16'h0060: readdata <= y_out_flat[31:0];
        16'h0064: readdata <= y_out_flat[63:32];

        // g_out_flat (192-bit)
        16'h0070: readdata <= g_out_flat[31:0];
        16'h0072: readdata <= g_out_flat[63:32];
        16'h0074: readdata <= g_out_flat[95:64];
        16'h0076: readdata <= g_out_flat[127:96];
        16'h0078: readdata <= g_out_flat[159:128];
        16'h007A: readdata <= g_out_flat[191:160];

        default: readdata <= 32'hDEADBEEF;
      endcase
    end
  end

endmodule
