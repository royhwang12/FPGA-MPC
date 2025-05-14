module memory_interface #(
    parameter STATE_DIM      = 12,         // Dimension of state vector (nx)
    parameter INPUT_DIM     = 4,          // Dimension of input vector (nu)
    parameter HORIZON       = 30,         // Maximum MPC horizon length (N)
    parameter EXT_DATA_WIDTH = 32,        // Bus data width
    parameter DATA_WIDTH    = 16,         // Internal calculation width (16-bit fixed point)
    parameter FRAC_BITS     = 8,          // Number of fractional bits for fixed point
    parameter ADDR_WIDTH    = 16,         // Address width for bus interface
    parameter MEM_ADDR_WIDTH = 9          // Internal memory address width address width
)(
    // Clock and reset  
    input logic clk,
    input logic rst,
    
    // Avalon memory-mapped slave interface
    input logic [EXT_DATA_WIDTH-1:0] writedata,
    input logic read,
    input logic write,
    input logic [ADDR_WIDTH-1:0] addr,
    input logic chipselect,
    output logic [EXT_DATA_WIDTH-1:0] readdata,
    
    // Control signals from/to solver
    input logic solver_done,
    input logic [31:0] current_iter,
    input logic [31:0] active_horizon,
    input logic converged,
    output logic start_solving,
    
    // Residuals for status reporting
    input logic [DATA_WIDTH-1:0] pri_res_u,
    input logic [DATA_WIDTH-1:0] pri_res_x,
    input logic [DATA_WIDTH-1:0] dual_res,
    output logic [DATA_WIDTH-1:0] pri_tol,
    output logic [DATA_WIDTH-1:0] dual_tol,
    
    // ADMM parameter
    output logic [DATA_WIDTH-1:0] rho,
    
    // Bounds
    output logic [DATA_WIDTH-1:0] u_min [INPUT_DIM],
    output logic [DATA_WIDTH-1:0] u_max [INPUT_DIM],
    output logic [DATA_WIDTH-1:0] x_min [STATE_DIM],
    output logic [DATA_WIDTH-1:0] x_max [STATE_DIM],
    
    // Initial state
    output logic [DATA_WIDTH-1:0] x_init [STATE_DIM],
    
    // Reference trajectories
    output logic [DATA_WIDTH-1:0] x_ref [STATE_DIM][HORIZON],
    output logic [DATA_WIDTH-1:0] u_ref [INPUT_DIM][HORIZON-1],
    
    // Memory signals for system matrices
    output logic [MEM_ADDR_WIDTH-1:0] A_rdaddress, A_wraddress,
    output logic [DATA_WIDTH-1:0] A_data_in,
    input logic [DATA_WIDTH-1:0] A_data_out,
    output logic A_wren,
    
    output logic [MEM_ADDR_WIDTH-1:0] B_rdaddress, B_wraddress,
    output logic [DATA_WIDTH-1:0] B_data_in,
    input logic [DATA_WIDTH-1:0] B_data_out,
    output logic B_wren,
    
    output logic [MEM_ADDR_WIDTH-1:0] Q_rdaddress, Q_wraddress,
    output logic [DATA_WIDTH-1:0] Q_data_in,
    input logic [DATA_WIDTH-1:0] Q_data_out,
    output logic Q_wren,
    
    output logic [MEM_ADDR_WIDTH-1:0] R_rdaddress, R_wraddress,
    output logic [DATA_WIDTH-1:0] R_data_in,
    input logic [DATA_WIDTH-1:0] R_data_out,
    output logic R_wren,
    
    // Memory signals for trajectories
    output logic [MEM_ADDR_WIDTH-1:0] x_rdaddress,
    input logic [DATA_WIDTH-1:0] x_data_out,
    
    output logic [MEM_ADDR_WIDTH-1:0] u_rdaddress,
    input logic [DATA_WIDTH-1:0] u_data_out
);

    // Temporary storage for system matrices
    logic [DATA_WIDTH-1:0] A_matrix [STATE_DIM][STATE_DIM];
    logic [DATA_WIDTH-1:0] B_matrix [STATE_DIM][INPUT_DIM];
    
    // Memory-mapped bus interface for configuration and results
    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            readdata <= 0;
            start_solving <= 0;
            
            // Reset memory write enables
            A_wren <= 0;
            B_wren <= 0;
            Q_wren <= 0;
            R_wren <= 0;
            
        //     // Reset indices for memory operations
        //     int i, j;
        //     for (i = 0; i < STATE_DIM; i++) begin
        //         for (j = 0; j < STATE_DIM; j++) begin
        //             A_matrix[i][j] <= 0;
        //         end
        //         for (j = 0; j < INPUT_DIM; j++) begin
        //             B_matrix[i][j] <= 0;
        //         end
        //         x_init[i] <= 0;
        //         x_min[i] <= {$bits(DATA_WIDTH){1'b0}} - 1'b1; 
        //         x_max[i] <= {$bits(DATA_WIDTH){1'b1}} - 1'b1; 
        //     end
            
        //     for (i = 0; i < INPUT_DIM; i++) begin
        //         u_min[i] <= {$bits(DATA_WIDTH){1'b0}} - 1'b1;
        //         u_max[i] <= {$bits(DATA_WIDTH){1'b1}} - 1'b1;
        //     end
            
            // Reset tolerances to default values (0.001 in fixed point)
            pri_tol  <= 16'h0019; // 0.001 in 16-bit fixed point with 8 fractional bits
            dual_tol <= 16'h0019; // 0.001 in 16-bit fixed point with 8 fractional bits
            
            // Reset rho parameter (1.0 in fixed point)
            rho      <= 16'h0100; // 1.0 in 16-bit fixed point with 8 fractional bits
            
        end else if (chipselect) begin
            // Default values for memory write enables
            A_wren <= 0;
            B_wren <= 0;
            Q_wren <= 0;
            R_wren <= 0;
            
            if (read) begin
                // Read operations
                case (addr[15:12]) // Use upper 4 bits for register type
                    // Status and configuration registers (0x0000-0x0FFF)
                    4'h0: begin
                        case (addr[11:0])
                            12'h000: readdata <= {31'b0, solver_done};  // Status register
                            12'h004: readdata <= current_iter;          // Current iteration count
                            12'h008: readdata <= active_horizon;        // Active horizon length
                            12'h00C: readdata <= converged;             // Convergence flag
                            12'h010: readdata <= pri_res_u[15:0];       // Primal residual (inputs)
                            12'h018: readdata <= pri_res_x[15:0];       // Primal residual (states)
                            12'h020: readdata <= dual_res[15:0];        // Dual residual
                            default: readdata <= 0;
                        endcase
                    end
                    
                    // Optimized state trajectory results (0x1000-0x1FFF)
                    4'h1: begin
                        int state_idx = addr[11:8];    // Which state variable (0 to STATE_DIM-1)
                        int horizon_idx = addr[7:0];   // Which horizon step (0 to HORIZON-1)
                        
                        if (state_idx < STATE_DIM && horizon_idx < HORIZON) begin
                            // Set read address for x memory
                            x_rdaddress <= horizon_idx * STATE_DIM + state_idx;
                            // Return state value
                            readdata <= x_data_out[15:0];
                        end else begin
                            readdata <= 0;
                        end
                    end
                    
                    // Optimized input trajectory results (0x2000-0x2FFF)
                    4'h2: begin
                        int input_idx = addr[11:8];    // Which input variable (0 to INPUT_DIM-1)
                        int horizon_idx = addr[7:0];   // Which horizon step (0 to HORIZON-2)
                        
                        if (input_idx < INPUT_DIM && horizon_idx < HORIZON-1) begin
                            // Set read address for u memory
                            u_rdaddress <= horizon_idx * INPUT_DIM + input_idx;
                            // Return input value
                            readdata <= u_data_out[15:0];
                        end else begin
                            readdata <= 0;
                        end
                    end
                    
                    default: readdata <= 0;
                endcase
            end else if (write) begin // Write operations
                
                case (addr[15:12]) // register type
                
                    // Status and configuration registers (0x0000-0x0FFF)
                    4'h0: begin
                        case (addr[11:0])
                            12'h000: start_solving <= writedata[0];     // Start solver
                            12'h004: active_horizon <= writedata;       // Set horizon length
                            12'h008: pri_tol <= writedata[15:0];        // Set primal tolerance
                            12'h010: dual_tol <= writedata[15:0];       // Set dual tolerance
                            12'h018: rho <= writedata[15:0];            // Set rho parameter
                        endcase
                    end
                    
                    // System matrix A (0x1000-0x1FFF)
                    4'h1: begin
                        int row = addr[11:8];    // Row index (0 to STATE_DIM-1)
                        int col = addr[7:4];     // Column index (0 to STATE_DIM-1)
                        
                        if (row < STATE_DIM && col < STATE_DIM) begin
                            // Store value and write to memory
                            A_matrix[row][col] <= writedata[15:0];
                            A_wraddress <= row * STATE_DIM + col;
                            A_data_in <= writedata[15:0];
                            A_wren <= 1;
                        end
                    end
                    
                    // System matrix B (0x2000-0x2FFF)
                    4'h2: begin
                        int row = addr[11:8];    // Row index (0 to STATE_DIM-1)
                        int col = addr[7:4];     // Column index (0 to INPUT_DIM-1)
                        
                        if (row < STATE_DIM && col < INPUT_DIM) begin
                            // Store value and write to memory
                            B_matrix[row][col] <= writedata[15:0];
                            B_wraddress <= row * INPUT_DIM + col;
                            B_data_in <= writedata[15:0];
                            B_wren <= 1;
                        end
                    end
                    
                    // Cost matrix Q (0x3000-0x3FFF)
                    4'h3: begin
                        int row = addr[11:8];    // Row index (0 to STATE_DIM-1)
                        int col = addr[7:4];     // Column index (0 to STATE_DIM-1)
                        
                        if (row < STATE_DIM && col < STATE_DIM) begin
                            // Store value and write to memory
                            Q_wraddress <= row * STATE_DIM + col;
                            Q_data_in <= writedata[15:0];
                            Q_wren <= 1;
                        end
                    end
                    
                    // Cost matrix R (0x4000-0x4FFF)
                    4'h4: begin
                        int row = addr[11:8];    // Row index (0 to INPUT_DIM-1)
                        int col = addr[7:4];     // Column index (0 to INPUT_DIM-1)
                        
                        if (row < INPUT_DIM && col < INPUT_DIM) begin
                            // Store value and write to memory
                            R_wraddress <= row * INPUT_DIM + col;
                            R_data_in <= writedata[15:0];
                            R_wren <= 1;
                        end
                    end
                    
                    // Initial state x_init (0x5000-0x5FFF)
                    4'h5: begin
                        int state_idx = addr[11:8];    // Which state variable (0 to STATE_DIM-1)
                        
                        if (state_idx < STATE_DIM) begin
                            // Store value
                            x_init[state_idx] <= writedata[15:0];
                        end
                    end
                    
                    // State reference trajectory x_ref (0x6000-0x6FFF)
                    4'h6: begin
                        int state_idx = addr[11:8];    // Which state variable (0 to STATE_DIM-1)
                        int horizon_idx = addr[7:0];   // Which horizon step (0 to HORIZON-1)
                        
                        if (state_idx < STATE_DIM && horizon_idx < HORIZON) begin
                            // Store reference value
                            x_ref[state_idx][horizon_idx] <= writedata[15:0];
                        end
                    end
                    
                    // Input reference trajectory u_ref (0x7000-0x7FFF)
                    4'h7: begin
                        int input_idx = addr[11:8];    // Which input variable (0 to INPUT_DIM-1)
                        int horizon_idx = addr[7:0];   // Which horizon step (0 to HORIZON-2)
                        
                        if (input_idx < INPUT_DIM && horizon_idx < HORIZON-1) begin
                            // Store reference value
                            u_ref[input_idx][horizon_idx] <= writedata[15:0];
                        end
                    end
                    
                    // State bounds (0x8000-0x8FFF)
                    4'h8: begin
                        int state_idx = addr[11:8];    // Which state variable (0 to STATE_DIM-1)
                        int is_max = addr[7];          // 0 for min bound, 1 for max bound
                        
                        if (state_idx < STATE_DIM) begin
                            if (is_max) begin
                                // Set max bound
                                x_max[state_idx] <= writedata[15:0];
                            end else begin
                                // Set min bound
                                x_min[state_idx] <= writedata[15:0];
                            end
                        end
                    end
                    
                    // Input bounds (0x9000-0x9FFF)
                    4'h9: begin
                        int input_idx = addr[11:8];    // Which input variable (0 to INPUT_DIM-1)
                        int is_max = addr[7];          // 0 for min bound, 1 for max bound
                        
                        if (input_idx < INPUT_DIM) begin
                            if (is_max) begin
                                // Set max bound
                                u_max[input_idx] <= writedata[15:0];
                            end else begin
                                // Set min bound
                                u_min[input_idx] <= writedata[15:0];
                            end
                        end
                    end
                endcase
            end
        end
    end

endmodule
