// Optimized memory_interface using RAM blocks only
module memory_interface #(
    parameter STATE_DIM       = 12,
    parameter INPUT_DIM       = 4,
    parameter HORIZON         = 30,
    parameter EXT_DATA_WIDTH  = 32,
    parameter DATA_WIDTH      = 16,
    parameter FRAC_BITS       = 8,
    parameter ADDR_WIDTH      = 16,
    parameter MEM_ADDR_WIDTH  = 9
)(
    input  logic clk,
    input  logic rst,

    // Avalon memory-mapped slave interface
    input  logic [EXT_DATA_WIDTH-1:0] writedata,
    input  logic read,
    input  logic write,
    input  logic [ADDR_WIDTH-1:0] addr,
    input  logic chipselect,
    output logic [EXT_DATA_WIDTH-1:0] readdata,

    // Solver control
    input  logic solver_done,
    input  logic [31:0] current_iter,
    input  logic [31:0] active_horizon,
    output logic [31:0] active_horizon_new,
    output logic active_horizon_wren,
    input  logic converged,
    output logic start_solving,

    // Residuals
    input  logic [DATA_WIDTH-1:0] pri_res_u,
    input  logic [DATA_WIDTH-1:0] pri_res_x,
    input  logic [DATA_WIDTH-1:0] dual_res,
    input logic [DATA_WIDTH-1:0] pri_tol,
    input logic [DATA_WIDTH-1:0] dual_tol,
    output logic [DATA_WIDTH-1:0] pri_tol_new,
    output logic [DATA_WIDTH-1:0] dual_tol_new,
    input logic [DATA_WIDTH-1:0] rho,
    output logic [DATA_WIDTH-1:0] rho_new,

    // Bounds
    output logic [DATA_WIDTH-1:0] u_min [INPUT_DIM],
    output logic [DATA_WIDTH-1:0] u_max [INPUT_DIM],
    output logic [DATA_WIDTH-1:0] x_min [STATE_DIM],
    output logic [DATA_WIDTH-1:0] x_max [STATE_DIM],

    // Initial state and references
    output logic [DATA_WIDTH-1:0] x_init [STATE_DIM],
    output logic [DATA_WIDTH-1:0] x_ref [STATE_DIM][HORIZON],
    output logic [DATA_WIDTH-1:0] u_ref [INPUT_DIM][HORIZON-1],

    // RAM control ports
    output logic [MEM_ADDR_WIDTH-1:0] A_rdaddress, A_wraddress,
    output logic [DATA_WIDTH-1:0] A_data_in,
    input  logic [DATA_WIDTH-1:0] A_data_out,
    output logic A_wren,

    output logic [MEM_ADDR_WIDTH-1:0] B_rdaddress, B_wraddress,
    output logic [DATA_WIDTH-1:0] B_data_in,
    input  logic [DATA_WIDTH-1:0] B_data_out,
    output logic B_wren,

    output logic [MEM_ADDR_WIDTH-1:0] Q_rdaddress, Q_wraddress,
    output logic [DATA_WIDTH-1:0] Q_data_in,
    input  logic [DATA_WIDTH-1:0] Q_data_out,
    output logic Q_wren,

    output logic [MEM_ADDR_WIDTH-1:0] R_rdaddress, R_wraddress,
    output logic [DATA_WIDTH-1:0] R_data_in,
    input  logic [DATA_WIDTH-1:0] R_data_out,
    output logic R_wren,

    output logic [MEM_ADDR_WIDTH-1:0] x_rdaddress,
    input  logic [DATA_WIDTH-1:0] x_data_out,

    output logic [MEM_ADDR_WIDTH-1:0] u_rdaddress,
    input  logic [DATA_WIDTH-1:0] u_data_out
);

    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            readdata <= 0;
            start_solving <= 0;
            A_wren <= 0;
            B_wren <= 0;
            Q_wren <= 0;
            R_wren <= 0;
            pri_tol_new  <= 16'h0019;
            dual_tol_new <= 16'h0019;
            rho_new      <= 16'h0100;
        end else if (chipselect) begin
            A_wren <= 0;
            B_wren <= 0;
            Q_wren <= 0;
            R_wren <= 0;

            if (read) begin
                case (addr[15:12])
                    4'h0: begin
                        case (addr[11:0])
                            12'h000: readdata <= {31'b0, solver_done};
                            12'h004: readdata <= current_iter;
                            12'h008: readdata <= active_horizon;
                            12'h00C: readdata <= converged;
                            12'h010: readdata <= pri_res_u;
                            12'h018: readdata <= pri_res_x;
                            12'h020: readdata <= dual_res;
                            default: readdata <= 0;
                        endcase
                    end
                    4'h1: begin
                        x_rdaddress <= addr[7:0] * STATE_DIM + addr[11:8];
                        readdata <= x_data_out;
                    end
                    4'h2: begin
                        u_rdaddress <= addr[7:0] * INPUT_DIM + addr[11:8];
                        readdata <= u_data_out;
                    end
                    default: readdata <= 0;
                endcase
            end else if (write) begin
                case (addr[15:12])
                    4'h0: begin
                        case (addr[11:0])
                            12'h000: start_solving <= writedata[0];
                            12'h004: begin
				active_horizon_new <= writedata;
				active_horizon_wren <= 1;
			    end
                            12'h008: pri_tol_new <= writedata[15:0];
                            12'h010: dual_tol_new <= writedata[15:0];
                            12'h018: rho_new <= writedata[15:0];
                        endcase
                    end
                    4'h1: begin
                        A_wraddress <= addr[11:8] * STATE_DIM + addr[7:4];
                        A_data_in <= writedata[15:0];
                        A_wren <= 1;
                    end
                    4'h2: begin
                        B_wraddress <= addr[11:8] * INPUT_DIM + addr[7:4];
                        B_data_in <= writedata[15:0];
                        B_wren <= 1;
                    end
                    4'h3: begin
                        Q_wraddress <= addr[11:8] * STATE_DIM + addr[7:4];
                        Q_data_in <= writedata[15:0];
                        Q_wren <= 1;
                    end
                    4'h4: begin
                        R_wraddress <= addr[11:8] * INPUT_DIM + addr[7:4];
                        R_data_in <= writedata[15:0];
                        R_wren <= 1;
                    end
                    4'h5: x_init[addr[11:8]] <= writedata[15:0];
                    4'h6: x_ref[addr[11:8]][addr[7:0]] <= writedata[15:0];
                    4'h7: u_ref[addr[11:8]][addr[7:0]] <= writedata[15:0];
                    4'h8: begin
                        if (addr[7]) x_max[addr[11:8]] <= writedata[15:0];
                        else x_min[addr[11:8]] <= writedata[15:0];
                    end
                    4'h9: begin
                        if (addr[7]) u_max[addr[11:8]] <= writedata[15:0];
                        else u_min[addr[11:8]] <= writedata[15:0];
                    end
                endcase
            end
        end
    end
endmodule
