`timescale 1ns/1ps

module cost_update #(
    parameter STATE_DIM = 12,             // Dimension of state vector (nx)
    parameter INPUT_DIM = 4,              // Dimension of input vector (nu)
    parameter HORIZON = 30,               // Maximum MPC horizon length (N)
    parameter DATA_WIDTH = 16,            // 16-bit fixed point
    parameter FRAC_BITS = 8,              // Number of fractional bits for fixed point
    parameter ADDR_WIDTH = 9              // Address width for memory access
)(
    input logic clk,
    input logic rst,
    input logic start, 
    
    // State and input trajectories
    output logic [ADDR_WIDTH-1:0] x_rdaddress,
    input logic [DATA_WIDTH-1:0] x_data_out, // x uses DATA_WIDTH for state values
    
    output logic [ADDR_WIDTH-1:0] u_rdaddress,
    input logic [DATA_WIDTH-1:0] u_data_out, // u uses DATA_WIDTH for input values
    
    // Reference trajectories for cost term calculation
    input logic [DATA_WIDTH-1:0] x_ref [STATE_DIM][HORIZON],
    input logic [DATA_WIDTH-1:0] u_ref [INPUT_DIM][HORIZON-1],
    
    // Cost matrices
    output logic [ADDR_WIDTH-1:0] R_rdaddress,
    input logic [DATA_WIDTH-1:0] R_data_out, // R for control cost
    output logic [ADDR_WIDTH-1:0] Q_rdaddress,
    input logic [DATA_WIDTH-1:0] Q_data_out, // Q for state cost
    output logic [ADDR_WIDTH-1:0] P_rdaddress,
    input logic [DATA_WIDTH-1:0] P_data_out, // P for terminal cost
    
    // Regularization parameter for updates
    input logic [DATA_WIDTH-1:0] rho,

    // Residual calculation outputs
    output logic [DATA_WIDTH-1:0] pri_res_u,    // Primal residual for inputs
    output logic [DATA_WIDTH-1:0] pri_res_x,    // Primal residual for states

    // Active horizon length
    input logic [31:0] active_horizon, // Current horizon length

    output logic done
);

    // State machine states
    localparam IDLE = 3'd0;
    localparam UPDATE_R = 3'd1;         // Update r (input residual)
    localparam UPDATE_Q = 3'd2;         // Update q (state residual)
    localparam UPDATE_P = 3'd3;         // Update p (terminal cost residual)
    localparam CALC_RESIDUALS = 3'd4;   // Calculate residuals
    localparam DONE_STATE = 3'd5;

    // State variables
    logic [2:0] state;
    logic [31:0] k;                     // Step counter for horizon
    logic [31:0] i;                     // Generic counter
    logic [31:0] state_timer;           // State timer for sequencing
    
    // Temporary variables for residual and cost term computation
    logic [DATA_WIDTH-1:0] temp_r, temp_q, temp_p;
    logic [DATA_WIDTH-1:0] temp_val_u, temp_val_x;
    logic [DATA_WIDTH-1:0] max_pri_res_u, max_pri_res_x;
    
    // Temporary storage for values read from memory
    logic [DATA_WIDTH-1:0] temp_u;
    logic [DATA_WIDTH-1:0] temp_x;
    
    // Memory access state variables
    logic [31:0] read_stage;            // Tracks memory read sequencing
    logic [31:0] write_stage;           // Tracks memory write sequencing
    
    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            state <= IDLE;
            done <= 0;
            i <= 0;
            k <= 0;
            state_timer <= 0;
            read_stage <= 0;
            write_stage <= 0;
            pri_res_u <= 0;
            pri_res_x <= 0;
            max_pri_res_u <= 0;
            max_pri_res_x <= 0;
            
            // Initialize memory control signals
            x_rdaddress <= 0;
            u_rdaddress <= 0;
            R_rdaddress <= 0;
            Q_rdaddress <= 0;
            P_rdaddress <= 0;
            
            r_rdaddress <= 0;
            q_rdaddress <= 0;
            p_rdaddress <= 0;
            
            r_wren <= 0;
            q_wren <= 0;
            p_wren <= 0;
        end else begin
            case (state)
                IDLE: begin
                    if (start) begin
                        state <= UPDATE_R;
                        done <= 0;
                        k <= 0;
                        i <= 0;
                        state_timer <= 0;
                        read_stage <= 0;
                        write_stage <= 0;
                        max_pri_res_u <= 0;
                        max_pri_res_x <= 0;
                    end
                end
                
                UPDATE_R: begin
                    // Update input residuals: r = -R * u_ref - rho * (z - y)
                    state_timer <= state_timer + 1;
                    
                    if (state_timer == 1) begin
                        // Initialize r update process
                        read_stage <= 1;
                        k <= 0;
                        i <= 0;
                    end
                    
                    if (read_stage == 1) begin
                        // Read values for r calculation (R, u_ref, and z/y)
                        if (k < active_horizon && i < INPUT_DIM) begin
                            // Calculate current index
                            int index;
                            index = k*INPUT_DIM + i;
                            
                            if (state_timer % 6 == 1) begin
                                // Stage 1: Set read addresses for R and u_ref
                                R_rdaddress <= i;  // Access R diagonal elements
                                u_rdaddress <= index;
                            end else if (state_timer % 6 == 2) begin
                                // Stage 2: Wait for read to complete
                            end else if (state_timer % 6 == 3) begin
                                // Stage 3: Capture values for r update
                                temp_u <= u_data_out;
                            end else if (state_timer % 6 == 4) begin
                                // Stage 4: Compute r update
                                temp_r = -R_data_out * temp_u;
                                r_data_in <= temp_r - rho * (temp_u - temp_u);
                                
                                // Set write address for r update
                                r_wraddress <= index;
                                r_wren <= 1;
                            end else begin
                                // Stage 5: Advance to next element
                                r_wren <= 0;
                                
                                i <= i + 1;
                                if (i == INPUT_DIM-1) begin
                                    i <= 0;
                                    k <= k + 1;
                                end
                            end
                        end else begin
                            // Done updating r
                            read_stage <= 0;
                            r_wren <= 0;
                            state <= UPDATE_Q;
                        end
                    end
                end
                
                UPDATE_Q: begin
                    // Update state residuals: q = -Q * x_ref - rho * (v - g)
                    state_timer <= state_timer + 1;
                    
                    if (state_timer == 1) begin
                        // Initialize q update process
                        read_stage <= 1;
                        k <= 0;
                        i <= 0;
                    end
                    
                    if (read_stage == 1) begin
                        // Read values for q calculation (Q, x_ref, and v/g)
                        if (k < active_horizon && i < STATE_DIM) begin
                            // Calculate current index
                            int index;
                            index = k*STATE_DIM + i;
                            
                            if (state_timer % 6 == 1) begin
                                // Stage 1: Set read addresses for Q and x_ref
                                Q_rdaddress <= i;  // Access Q diagonal elements
                                x_rdaddress <= index;
                            end else if (state_timer % 6 == 2) begin
                                // Stage 2: Wait for read to complete
                            end else if (state_timer % 6 == 3) begin
                                // Stage 3: Capture values for q update
                                temp_x <= x_data_out;
                            end else if (state_timer % 6 == 4) begin
                                // Stage 4: Compute q update
                                temp_q = -Q_data_out * temp_x;
                                q_data_in <= temp_q - rho * (temp_x - temp_x);
                                
                                // Set write address for q update
                                q_wraddress <= index;
                                q_wren <= 1;
                            end else begin
                                // Stage 5: Advance to next element
                                q_wren <= 0;
                                
                                i <= i + 1;
                                if (i == STATE_DIM-1) begin
                                    i <= 0;
                                    k <= k + 1;
                                end
                            end
                        end else begin
                            // Done updating q
                            read_stage <= 0;
                            q_wren <= 0;
                            state <= UPDATE_P;
                        end
                    end
                end
                
                UPDATE_P: begin
                    // Update terminal cost residuals: p = -P * x_ref
                    state_timer <= state_timer + 1;
                    
                    if (state_timer == 1) begin
                        // Initialize p update process
                        read_stage <= 1;
                        k <= 0;
                        i <= 0;
                    end
                    
                    if (read_stage == 1) begin
                        // Read values for p calculation (P, x_ref)
                        if (k < active_horizon && i < STATE_DIM) begin
                            // Calculate current index
                            int index;
                            index = k*STATE_DIM + i;
                            
                            if (state_timer % 6 == 1) begin
                                // Stage 1: Set read address for P and x_ref
                                P_rdaddress <= i;
                                x_rdaddress <= index;
                            end else if (state_timer % 6 == 2) begin
                                // Stage 2: Wait for read to complete
                            end else if (state_timer % 6 == 3) begin
                                // Stage 3: Capture values for p update
                                temp_x <= x_data_out;
                            end else if (state_timer % 6 == 4) begin
                                // Stage 4: Compute p update
                                temp_p = -P_data_out * temp_x;
                                p_data_in <= temp_p;
                                
                                // Set write address for p update
                                p_wraddress <= index;
                                p_wren <= 1;
                            end else begin
                                // Stage 5: Advance to next element
                                p_wren <= 0;
                                
                                i <= i + 1;
                                if (i == STATE_DIM-1) begin
                                    i <= 0;
                                    k <= k + 1;
                                end
                            end
                        end else begin
                            // Done updating p
                            read_stage <= 0;
                            p_wren <= 0;
                            state <= CALC_RESIDUALS;
                        end
                    end
                end
                
                CALC_RESIDUALS: begin
                    // Calculate final residuals (inputs and states)
                    pri_res_u <= max_pri_res_u;
                    pri_res_x <= max_pri_res_x;
                    state <= DONE_STATE;
                end
                
                DONE_STATE: begin
                    done <= 1;
                    // Reset write enables and done signal
                    r_wren <= 0;
                    q_wren <= 0;
                    p_wren <= 0;
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
