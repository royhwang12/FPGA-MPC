// Computes primal and dual residuals to check for ADMM convergence

module residual_calculator #(
    parameter STATE_DIM = 6,             // Dimension of state vector (nx)
    parameter INPUT_DIM = 3,             // Dimension of input vector (nu)
    parameter HORIZON = 30,              // Maximum MPC horizon length (N)
    parameter DATA_WIDTH = 64,           
    parameter ADDR_WIDTH = 9             
)(
    input logic clk,                     // Clock
    input logic rst,                     // Reset
    input logic start,                   // Start signal
    
    // Inputs for residual calculation
    input logic [DATA_WIDTH-1:0] pri_res_u,    // Primal residual for inputs from dual_update
    input logic [DATA_WIDTH-1:0] pri_res_x,    // Primal residual for states from dual_update
    
    // Memory interfaces for z and z_prev
    // Current z values
    output logic [ADDR_WIDTH-1:0] z_rdaddress,
    input logic [DATA_WIDTH-1:0] z_data_out,
    
    // Previous z values
    output logic [ADDR_WIDTH-1:0] z_prev_rdaddress,
    input logic [DATA_WIDTH-1:0] z_prev_data_out,
    
    // Tolerance thresholds
    input logic [DATA_WIDTH-1:0] pri_tol,      // Primal residual tolerance
    input logic [DATA_WIDTH-1:0] dual_tol,     // Dual residual tolerance
    
    // Configuration
    input logic [31:0] active_horizon,  // Current horizon length to use
    
    // Outputs
    output logic [DATA_WIDTH-1:0] dual_res,    // Dual residual 
    output logic converged,                    // Convergence flag
    output logic done                          // Done signal
);

    // State machine states
    localparam IDLE = 3'd0;
    localparam CALC_DUAL_RESIDUAL = 3'd1;
    localparam CHECK_CONVERGENCE = 3'd2;
    localparam DONE_STATE = 3'd3;

    // State variables
    logic [2:0] state;
    logic [31:0] i;                     // Generic counter
    logic [31:0] k;                     // Step counter for horizon
    logic [31:0] state_timer;           // State timer
    
    // Memory access state variables
    logic [31:0] read_stage;            // Tracks memory read sequencing
    
    // Temporary storage for values read from memory
    logic [DATA_WIDTH-1:0] temp_z;
    logic [DATA_WIDTH-1:0] temp_z_prev;
    
    // Temporary computation variables
    logic [DATA_WIDTH-1:0] temp_diff;    // Temporary diff value for calculations
    logic [DATA_WIDTH-1:0] max_dual_res; // Maximum dual residual 
    
    // Convergence flags
    logic pri_converged_u;              
    logic pri_converged_x;              
    logic dual_converged;               
    
    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            state <= IDLE;
            done <= 0;
            converged <= 0;
            i <= 0;
            k <= 0;
            state_timer <= 0;
            read_stage <= 0;
            max_dual_res <= 0;
            pri_converged_u <= 0;
            pri_converged_x <= 0;
            dual_converged <= 0;
            dual_res <= 0;
            
            // Initialize memory control signals
            z_rdaddress <= 0;
            z_prev_rdaddress <= 0;
        end else begin
            case (state)
                IDLE: begin
                    if (start) begin
                        state <= CALC_DUAL_RESIDUAL;
                        done <= 0;
                        converged <= 0;
                        i <= 0;
                        k <= 0;
                        read_stage <= 0;
                        state_timer <= 0;
                        max_dual_res <= 0;
                    end
                end
                
                CALC_DUAL_RESIDUAL: begin
                    // Calculate dual residual based on z change: max_i |z_k - z_{k-1}|
                    state_timer <= state_timer + 1;
                    
                    if (state_timer == 1) begin
                        // Initialize the dual residual calculation process
                        read_stage <= 1;
                        k <= 0;
                        i <= 0;
                    end
                    
                    if (read_stage == 1) begin
                        // Read z and z_prev values for current element
                        if (k < active_horizon-1 && i < INPUT_DIM) begin
                            // Calculate current index
                            int index = k*INPUT_DIM + i;
                            
                            // Memory read sequence
                            if (state_timer % 5 == 1) begin
                                // Stage 1: Set read addresses
                                z_rdaddress <= index;
                                z_prev_rdaddress <= index;
                            end else if (state_timer % 5 == 2) begin
                                // Stage 2: Wait for read to complete
                            end else if (state_timer % 5 == 3) begin
                                // Stage 3: Capture values
                                temp_z <= z_data_out;
                                temp_z_prev <= z_prev_data_out;
                            end else if (state_timer % 5 == 4) begin
                                // Stage 4: Calculate difference and update max residual
                                temp_diff = temp_z - temp_z_prev;
                                if (temp_diff < 0) begin
                                    temp_diff = -temp_diff; // Absolute value
                                end
                                
                                // Update maximum dual residual
                                if (temp_diff > max_dual_res) begin
                                    max_dual_res <= temp_diff;
                                end
                            end else begin
                                // Stage 5: Advance to next element
                                i <= i + 1;
                                if (i == INPUT_DIM-1) begin
                                    i <= 0;
                                    k <= k + 1;
                                end
                            end
                        end else begin
                            // Done calculating maximum dual residual
                            read_stage <= 0;
                            dual_res <= max_dual_res; // Store the final dual residual
                            state <= CHECK_CONVERGENCE;
                        end
                    end
                end
                
                CHECK_CONVERGENCE: begin
                    // Check primal residuals against tolerance
                    pri_converged_u <= (pri_res_u <= pri_tol);
                    pri_converged_x <= (pri_res_x <= pri_tol);
                    
                    // Check dual residual against tolerance
                    dual_converged <= (dual_res <= dual_tol);
                    
                    // Set overall convergence flag
                    converged <= pri_converged_u && pri_converged_x && dual_converged;
                    
                    state <= DONE_STATE;
                end
                
                DONE_STATE: begin
                    done <= 1;
                    if (!start) begin
                        state <= IDLE;
                        done <= 0;
                    end
                end
                
                default: state <= IDLE;
            endcase
        end
    end

endmodule
