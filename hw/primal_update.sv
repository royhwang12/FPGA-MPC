// Riccati backward pass and forward rollout for X-update step
// Based on TinyMPC algorithm

module primal_update #(
    parameter STATE_DIM = 6,             // Dimension of state vector (nx)
    parameter INPUT_DIM = 3,             // Dimension of input vector (nu)
    parameter HORIZON = 30,              // Maximum MPC horizon length (N)
    parameter DATA_WIDTH = 64,           
    parameter ADDR_WIDTH = 9            
)(
    input logic clk,                     // Clock
    input logic rst,                     // Reset
    input logic start,                   // Start signal
    
    // System matrices and precomputed values memory access
    output logic [ADDR_WIDTH-1:0] A_rdaddress,
    input logic [DATA_WIDTH-1:0] A_data_out,
    
    output logic [ADDR_WIDTH-1:0] B_rdaddress,
    input logic [DATA_WIDTH-1:0] B_data_out,
    
    output logic [ADDR_WIDTH-1:0] K_rdaddress,
    input logic [DATA_WIDTH-1:0] K_data_out,
    
    output logic [ADDR_WIDTH-1:0] C1_rdaddress,
    input logic [DATA_WIDTH-1:0] C1_data_out,
    
    output logic [ADDR_WIDTH-1:0] C2_rdaddress,
    input logic [DATA_WIDTH-1:0] C2_data_out,

    // Initial state
    input logic [DATA_WIDTH-1:0] x_init [STATE_DIM],
    
    // Linear cost memory access
    output logic [ADDR_WIDTH-1:0] p_rdaddress,
    input logic [DATA_WIDTH-1:0] p_data_out,
    
    output logic [ADDR_WIDTH-1:0] r_rdaddress,
    input logic [DATA_WIDTH-1:0] r_data_out,
    
    output logic [ADDR_WIDTH-1:0] q_rdaddress,
    input logic [DATA_WIDTH-1:0] q_data_out,

    // Output trajectory memory access
    output logic [ADDR_WIDTH-1:0] x_wraddress,
    output logic [DATA_WIDTH-1:0] x_data_in,
    output logic x_wren,
    
    output logic [ADDR_WIDTH-1:0] u_wraddress,
    output logic [DATA_WIDTH-1:0] u_data_in,
    output logic u_wren,
    
    output logic [ADDR_WIDTH-1:0] d_wraddress,
    output logic [DATA_WIDTH-1:0] d_data_in,
    output logic d_wren,
    
    // Configuration
    input logic [31:0] active_horizon,  // Current horizon length to use

    output logic done                   // Done signal
);

    // State machine states
    localparam IDLE = 3'd0;
    localparam INIT = 3'd1;              // Initialize variables
    localparam BACKWARD_PASS = 3'd2;     // Backward pass (Riccati recursion)
    localparam FORWARD_PASS = 3'd3;      // Forward pass (trajectory rollout)
    localparam DONE_STATE = 3'd4;

    // Substates for backward pass
    localparam BP_LOAD_VECTORS = 2'd0;   // Load r and p vectors
    localparam BP_COMPUTE_D = 2'd1;      // Compute d vector
    localparam BP_COMPUTE_P = 2'd2;      // Compute p vector
    localparam BP_STORE_RESULTS = 2'd3;  // Store results and move to next timestep

    // Substates for forward pass
    localparam FP_LOAD_MATRICES = 2'd0;  // Load matrices for current timestep
    localparam FP_COMPUTE_U = 2'd1;      // Compute control input u
    localparam FP_COMPUTE_X = 2'd2;      // Compute next state x
    localparam FP_STORE_RESULTS = 2'd3;  // Store results and move to next timestep

    // State and control variables
    logic [2:0] state;
    logic [1:0] substate;                // Substate for detailed control
    logic [31:0] k;                      // Step counter
    logic [31:0] i, j;                   // Generic loop counters
    logic [31:0] cycle_counter;          // Cycle counter for timing operations
    
    // Temporary storage for matrices and vectors
    logic [DATA_WIDTH-1:0] A_matrix [STATE_DIM][STATE_DIM];
    logic [DATA_WIDTH-1:0] B_matrix [STATE_DIM][INPUT_DIM];
    logic [DATA_WIDTH-1:0] K_matrix [INPUT_DIM][STATE_DIM];
    logic [DATA_WIDTH-1:0] C1_matrix [INPUT_DIM][INPUT_DIM];
    logic [DATA_WIDTH-1:0] C2_matrix [STATE_DIM][STATE_DIM];
    
    // Vectors for current timestep
    logic [DATA_WIDTH-1:0] r_vector [INPUT_DIM];
    logic [DATA_WIDTH-1:0] p_vector [STATE_DIM];
    logic [DATA_WIDTH-1:0] q_vector [STATE_DIM];
    logic [DATA_WIDTH-1:0] d_vector [INPUT_DIM];
    logic [DATA_WIDTH-1:0] next_p_vector [STATE_DIM];
    
    // Trajectory storage
    logic [DATA_WIDTH-1:0] x_trajectory [HORIZON][STATE_DIM];
    logic [DATA_WIDTH-1:0] u_trajectory [HORIZON-1][INPUT_DIM];
    
    // Computation variables
    logic [DATA_WIDTH-1:0] temp_sum;
    logic [DATA_WIDTH-1:0] Bp_vector [INPUT_DIM];  // B^T * p vector
    
    // Memory access control signals
    logic load_matrices_done;
    logic compute_done;
    logic store_done;
    
    // Temporary computation variables
    logic [DATA_WIDTH-1:0] temp_val;     // Temporary value for scalar computations
    
    // Floating point operations
    // Note: In a real implementation, these would be replaced with proper floating point units
    // For simplicity, we're using simple assignments here
    function automatic logic [DATA_WIDTH-1:0] fp_add(logic [DATA_WIDTH-1:0] a, logic [DATA_WIDTH-1:0] b);
        return a + b; // Simplified - would need actual FP adder
    endfunction
    
    function automatic logic [DATA_WIDTH-1:0] fp_sub(logic [DATA_WIDTH-1:0] a, logic [DATA_WIDTH-1:0] b);
        return a - b; // Simplified - would need actual FP subtractor
    endfunction
    
    function automatic logic [DATA_WIDTH-1:0] fp_mul(logic [DATA_WIDTH-1:0] a, logic [DATA_WIDTH-1:0] b);
        return a * b; // Simplified - would need actual FP multiplier
    endfunction
    
    function automatic logic [DATA_WIDTH-1:0] fp_neg(logic [DATA_WIDTH-1:0] a);
        return -a; // Simplified - would need actual FP negation
    endfunction
    
    // State machine implementation
    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            // Reset state machine
            state <= IDLE;
            substate <= 0;
            done <= 0;
            k <= 0;
            i <= 0;
            j <= 0;
            cycle_counter <= 0;
            load_matrices_done <= 0;
            compute_done <= 0;
            store_done <= 0;
            
            // Reset memory addresses
            A_rdaddress <= 0;
            B_rdaddress <= 0;
            K_rdaddress <= 0;
            C1_rdaddress <= 0;
            C2_rdaddress <= 0;
            p_rdaddress <= 0;
            r_rdaddress <= 0;
            q_rdaddress <= 0;
            
            // Reset memory write signals
            x_wraddress <= 0;
            x_data_in <= 0;
            x_wren <= 0;
            
            u_wraddress <= 0;
            u_data_in <= 0;
            u_wren <= 0;
            
            d_wraddress <= 0;
            d_data_in <= 0;
            d_wren <= 0;
            
            // Reset computation variables
            temp_val <= 0;
            temp_sum <= 0;
        end else begin
            // Default values for control signals
            x_wren <= 0;
            u_wren <= 0;
            d_wren <= 0;
            
            case (state)
                IDLE: begin
                    // Wait for start signal
                    if (start) begin
                        state <= INIT;
                        done <= 0;
                        cycle_counter <= 0;
                        i <= 0;
                        j <= 0;
                    end
                end
                
                INIT: begin
                    // Initialize the state trajectory with initial state
                    if (i < STATE_DIM) begin
                        // Store initial state in x_trajectory[0]
                        x_trajectory[0][i] <= x_init[i];
                        i <= i + 1;
                    end else begin
                        // Prepare for backward pass
                        state <= BACKWARD_PASS;
                        substate <= BP_LOAD_VECTORS;
                        k <= active_horizon - 2;  // Start from N-2 (0-indexed)
                        i <= 0;
                        j <= 0;
                        cycle_counter <= 0;
                    end
                end
                
                BACKWARD_PASS: begin
                    // Backward pass of Riccati recursion (from N-2 down to 0)
                    cycle_counter <= cycle_counter + 1;
                    
                    case (substate)
                        BP_LOAD_VECTORS: begin
                            // Load r and p vectors for current timestep k
                            if (i < INPUT_DIM) begin
                                // Load r vector for timestep k
                                r_rdaddress <= k*INPUT_DIM + i;
                                if (cycle_counter > 1) begin  // Wait for memory read
                                    r_vector[i] <= r_data_out;
                                    i <= i + 1;
                                    cycle_counter <= 0;
                                end
                            end else if (i < INPUT_DIM + STATE_DIM) begin
                                // Load p vector for timestep k+1
                                p_rdaddress <= (k+1)*STATE_DIM + (i - INPUT_DIM);
                                if (cycle_counter > 1) begin  // Wait for memory read
                                    p_vector[i - INPUT_DIM] <= p_data_out;
                                    i <= i + 1;
                                    cycle_counter <= 0;
                                end
                            end else if (i < INPUT_DIM + STATE_DIM + STATE_DIM*INPUT_DIM) begin
                                // Load B matrix for B'*p computation
                                // Row-major order: B[row][col] = B_mem[row*INPUT_DIM + col]
                                B_rdaddress <= (i - INPUT_DIM - STATE_DIM);
                                if (cycle_counter > 1) begin  // Wait for memory read
                                    // Calculate indices for B matrix
                                    int row = (i - INPUT_DIM - STATE_DIM) / INPUT_DIM;
                                    int col = (i - INPUT_DIM - STATE_DIM) % INPUT_DIM;
                                    B_matrix[row][col] <= B_data_out;
                                    i <= i + 1;
                                    cycle_counter <= 0;
                                end
                            end else if (i < INPUT_DIM + STATE_DIM + STATE_DIM*INPUT_DIM + INPUT_DIM*INPUT_DIM) begin
                                // Load C1 matrix
                                C1_rdaddress <= (i - INPUT_DIM - STATE_DIM - STATE_DIM*INPUT_DIM);
                                if (cycle_counter > 1) begin  // Wait for memory read
                                    // Calculate indices for C1 matrix
                                    int row = (i - INPUT_DIM - STATE_DIM - STATE_DIM*INPUT_DIM) / INPUT_DIM;
                                    int col = (i - INPUT_DIM - STATE_DIM - STATE_DIM*INPUT_DIM) % INPUT_DIM;
                                    C1_matrix[row][col] <= C1_data_out;
                                    i <= i + 1;
                                    cycle_counter <= 0;
                                end
                            end else if (i < INPUT_DIM + STATE_DIM + STATE_DIM*INPUT_DIM + INPUT_DIM*INPUT_DIM + STATE_DIM*STATE_DIM) begin
                                // Load C2 matrix
                                C2_rdaddress <= (i - INPUT_DIM - STATE_DIM - STATE_DIM*INPUT_DIM - INPUT_DIM*INPUT_DIM);
                                if (cycle_counter > 1) begin  // Wait for memory read
                                    // Calculate indices for C2 matrix
                                    int row = (i - INPUT_DIM - STATE_DIM - STATE_DIM*INPUT_DIM - INPUT_DIM*INPUT_DIM) / STATE_DIM;
                                    int col = (i - INPUT_DIM - STATE_DIM - STATE_DIM*INPUT_DIM - INPUT_DIM*INPUT_DIM) % STATE_DIM;
                                    C2_matrix[row][col] <= C2_data_out;
                                    i <= i + 1;
                                    cycle_counter <= 0;
                                end
                            end else if (i < INPUT_DIM + STATE_DIM + STATE_DIM*INPUT_DIM + INPUT_DIM*INPUT_DIM + STATE_DIM*STATE_DIM + INPUT_DIM*STATE_DIM) begin
                                // Load K matrix (Kinf)
                                K_rdaddress <= (i - INPUT_DIM - STATE_DIM - STATE_DIM*INPUT_DIM - INPUT_DIM*INPUT_DIM - STATE_DIM*STATE_DIM);
                                if (cycle_counter > 1) begin  // Wait for memory read
                                    // Calculate indices for K matrix
                                    int row = (i - INPUT_DIM - STATE_DIM - STATE_DIM*INPUT_DIM - INPUT_DIM*INPUT_DIM - STATE_DIM*STATE_DIM) / STATE_DIM;
                                    int col = (i - INPUT_DIM - STATE_DIM - STATE_DIM*INPUT_DIM - INPUT_DIM*INPUT_DIM - STATE_DIM*STATE_DIM) % STATE_DIM;
                                    K_matrix[row][col] <= K_data_out;
                                    i <= i + 1;
                                    cycle_counter <= 0;
                                end
                            end else begin
                                // All vectors and matrices loaded, move to compute d
                                substate <= BP_COMPUTE_D;
                                i <= 0;
                                j <= 0;
                                cycle_counter <= 0;
                            end
                        end
                        
                        BP_COMPUTE_D: begin
                            // Compute d vector: d[:, k] = C1 @ (B.T @ p[:, k+1] + r[:, k])
                            if (i < INPUT_DIM) begin
                                if (j == 0) begin
                                    // First compute B.T @ p[:, k+1]
                                    Bp_vector[i] <= 0;  // Initialize
                                    j <= 1;
                                end else if (j <= STATE_DIM) begin
                                    // Accumulate B.T @ p[:, k+1]
                                    Bp_vector[i] <= fp_add(Bp_vector[i], fp_mul(B_matrix[j-1][i], p_vector[j-1]));
                                    j <= j + 1;
                                end else if (j == STATE_DIM + 1) begin
                                    // Add r[:, k]
                                    Bp_vector[i] <= fp_add(Bp_vector[i], r_vector[i]);
                                    j <= STATE_DIM + 2;
                                end else if (j == STATE_DIM + 2) begin
                                    // Initialize d_vector[i]
                                    d_vector[i] <= 0;
                                    j <= STATE_DIM + 3;
                                end else if (j < STATE_DIM + 3 + INPUT_DIM) begin
                                    // Compute C1 @ (B.T @ p[:, k+1] + r[:, k])
                                    d_vector[i] <= fp_add(d_vector[i], fp_mul(C1_matrix[i][j-(STATE_DIM+3)], Bp_vector[j-(STATE_DIM+3)]));
                                    j <= j + 1;
                                end else begin
                                    // Move to next row
                                    i <= i + 1;
                                    j <= 0;
                                end
                            end else begin
                                // d vector computed, move to compute p
                                substate <= BP_COMPUTE_P;
                                i <= 0;
                                j <= 0;
                            end
                        end
                        
                        BP_COMPUTE_P: begin
                            // Compute p vector: p[:, k] = q[:, k] + C2 @ p[:, k+1] - K.T @ r[:, k]
                            if (i < STATE_DIM) begin
                                if (j == 0) begin
                                    // Load q[:, k]
                                    q_rdaddress <= k*STATE_DIM + i;
                                    j <= 1;
                                end else if (j == 1) begin
                                    // Wait for memory read
                                    if (cycle_counter > 1) begin
                                        // Initialize p with q
                                        next_p_vector[i] <= q_data_out;
                                        j <= 2;
                                        cycle_counter <= 0;
                                    end
                                end else if (j < 2 + STATE_DIM) begin
                                    // Add C2 @ p[:, k+1]
                                    next_p_vector[i] <= fp_add(next_p_vector[i], fp_mul(C2_matrix[i][j-2], p_vector[j-2]));
                                    j <= j + 1;
                                end else if (j < 2 + STATE_DIM + INPUT_DIM) begin
                                    // Subtract K.T @ r[:, k]
                                    next_p_vector[i] <= fp_sub(next_p_vector[i], fp_mul(K_matrix[j-(2+STATE_DIM)][i], r_vector[j-(2+STATE_DIM)]));
                                    j <= j + 1;
                                end else begin
                                    // Move to next element
                                    i <= i + 1;
                                    j <= 0;
                                end
                            end else begin
                                // p vector computed, move to store results
                                substate <= BP_STORE_RESULTS;
                                i <= 0;
                            end
                        end
                        
                        BP_STORE_RESULTS: begin
                            // Store d and p vectors to memory
                            if (i < INPUT_DIM) begin
                                // Store d vector
                                d_wraddress <= k*INPUT_DIM + i;
                                d_data_in <= d_vector[i];
                                d_wren <= 1;
                                i <= i + 1;
                            end else if (i < INPUT_DIM + STATE_DIM) begin
                                // Store p vector
                                p_wraddress <= k*STATE_DIM + (i - INPUT_DIM);
                                p_data_in <= next_p_vector[i - INPUT_DIM];
                                p_wren <= 1;
                                i <= i + 1;
                            end else begin
                                // Done storing results
                                d_wren <= 0;
                                p_wren <= 0;
                                
                                if (k > 0) begin
                                    // Move to previous timestep
                                    k <= k - 1;
                                    substate <= BP_LOAD_VECTORS;
                                    i <= 0;
                                    j <= 0;
                                    cycle_counter <= 0;
                                end else begin
                                    // Backward pass complete, move to forward pass
                                    state <= FORWARD_PASS;
                                    substate <= FP_LOAD_MATRICES;
                                    k <= 0;
                                    i <= 0;
                                    j <= 0;
                                    cycle_counter <= 0;
                                end
                            end
                        end
                    endcase
                end
                
                FORWARD_PASS: begin
                    // Forward pass - rollout trajectory from k=0 to N-1
                    cycle_counter <= cycle_counter + 1;
                    
                    case (substate)
                        FP_LOAD_MATRICES: begin
                            // For the first step, we already have the matrices loaded from backward pass
                            // For subsequent steps, we would reload if needed
                            if (k == 0) begin
                                // Just move to compute u for first step
                                substate <= FP_COMPUTE_U;
                                i <= 0;
                                j <= 0;
                                cycle_counter <= 0;
                            end else begin
                                // For subsequent steps, we would reload matrices here if needed
                                // For simplicity, we'll assume they're still loaded
                                substate <= FP_COMPUTE_U;
                                i <= 0;
                                j <= 0;
                                cycle_counter <= 0;
                            end
                        end
                        
                        FP_COMPUTE_U: begin
                            // Compute control input u: u[:, k] = -K_inf @ x[:, k] - d[:, k]
                            if (i < INPUT_DIM) begin
                                if (j == 0) begin
                                    // Initialize accumulator
                                    temp_sum <= 0;
                                    j <= 1;
                                end else if (j <= STATE_DIM) begin
                                    // Accumulate K_inf @ x[:, k]
                                    temp_sum <= fp_add(temp_sum, fp_mul(K_matrix[i][j-1], x_trajectory[k][j-1]));
                                    j <= j + 1;
                                end else if (j == STATE_DIM + 1) begin
                                    // Load d vector for timestep k
                                    d_rdaddress <= k*INPUT_DIM + i;
                                    j <= STATE_DIM + 2;
                                end else if (j == STATE_DIM + 2) begin
                                    // Wait for memory read
                                    if (cycle_counter > 1) begin
                                        // Compute full control with feedforward term: u = -K_inf @ x - d
                                        u_trajectory[k][i] <= fp_sub(fp_neg(temp_sum), d_data_out);
                                        j <= 0;
                                        i <= i + 1;
                                        cycle_counter <= 0;
                                    end
                                end
                            end else begin
                                // Control input computed, move to compute next state
                                substate <= FP_COMPUTE_X;
                                i <= 0;
                                j <= 0;
                            end
                        end
                        
                        FP_COMPUTE_X: begin
                            // Compute next state: x[:, k+1] = A @ x[:, k] + B @ u[:, k]
                            if (k < active_horizon - 1) begin
                                if (i < STATE_DIM) begin
                                    if (j == 0) begin
                                        // Initialize accumulator for A @ x[:, k]
                                        temp_sum <= 0;
                                        j <= 1;
                                    end else if (j <= STATE_DIM) begin
                                        // Accumulate A @ x[:, k]
                                        temp_sum <= fp_add(temp_sum, fp_mul(A_matrix[i][j-1], x_trajectory[k][j-1]));
                                        j <= j + 1;
                                    end else if (j <= STATE_DIM + INPUT_DIM) begin
                                        // Accumulate B @ u[:, k]
                                        temp_sum <= fp_add(temp_sum, fp_mul(B_matrix[i][j-STATE_DIM-1], u_trajectory[k][j-STATE_DIM-1]));
                                        j <= j + 1;
                                    end else begin
                                        // Store next state
                                        x_trajectory[k+1][i] <= temp_sum;
                                        i <= i + 1;
                                        j <= 0;
                                    end
                                end else begin
                                    // Next state computed, move to store results
                                    substate <= FP_STORE_RESULTS;
                                    i <= 0;
                                }
                            end else begin
                                // Reached end of horizon, move to store results
                                substate <= FP_STORE_RESULTS;
                                i <= 0;
                            end
                        end
                        
                        FP_STORE_RESULTS: begin
                            // Store trajectory to memory
                            if (i < STATE_DIM) begin
                                // Store state vector x for current timestep
                                x_wraddress <= k*STATE_DIM + i;
                                x_data_in <= x_trajectory[k][i];
                                x_wren <= 1;
                                i <= i + 1;
                            end else if (i < STATE_DIM + INPUT_DIM) begin
                                // Store control input u for current timestep
                                u_wraddress <= k*INPUT_DIM + (i - STATE_DIM);
                                u_data_in <= u_trajectory[k][i - STATE_DIM];
                                u_wren <= 1;
                                i <= i + 1;
                            end else begin
                                // Done storing results for current timestep
                                x_wren <= 0;
                                u_wren <= 0;
                                
                                if (k < active_horizon - 1) begin
                                    // Move to next timestep
                                    k <= k + 1;
                                    substate <= FP_LOAD_MATRICES;
                                    i <= 0;
                                    j <= 0;
                                    cycle_counter <= 0;
                                end else begin
                                    // Forward pass complete
                                    state <= DONE_STATE;
                                }
                            end
                        end
                    endcase
                end
                
                DONE_STATE: begin
                    done <= 1;
                    // Ensure all write enables are off
                    x_wren <= 0;
                    u_wren <= 0;
                    d_wren <= 0;
                    
                    if (!start) begin
                        state <= IDLE;
                        done <= 0;
                    end
                end
                
                default: state <= IDLE;
            endcase
        end
    end
    
    // Add memory write ports for p vector
    // These would typically be connected to the appropriate memory in the top module
    output logic [ADDR_WIDTH-1:0] p_wraddress;
    output logic [DATA_WIDTH-1:0] p_data_in;
    output logic p_wren;

endmodule
