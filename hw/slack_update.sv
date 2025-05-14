// Implements constraint projection for both u and x variables

`timescale 1 ps / 1 ps
module slack_update #(
    parameter STATE_DIM = 6,             // Dimension of state vector (nx)
    parameter INPUT_DIM = 3,             // Dimension of input vector (nu)
    parameter HORIZON = 30,              // Maximum MPC horizon length (N)
    parameter DATA_WIDTH = 64,           
    parameter ADDR_WIDTH = 9,            
)(
    input logic clk,                     // Clock
    input logic rst,                     // Reset
    input logic start,                   // Start signal
    
    // ADMM variables - memory interfaces for main trajectories
    // State trajectory (x)
    output logic [ADDR_WIDTH-1:0] x_rdaddress,
    input logic [DATA_WIDTH-1:0] x_data_out,
    
    // Input trajectory (u)
    output logic [ADDR_WIDTH-1:0] u_rdaddress,
    input logic [DATA_WIDTH-1:0] u_data_out,
    
    // Input dual variables (y)
    output logic [ADDR_WIDTH-1:0] y_rdaddress,
    input logic [DATA_WIDTH-1:0] y_data_out,
    
    // State dual variables (g)
    output logic [ADDR_WIDTH-1:0] g_rdaddress,
    input logic [DATA_WIDTH-1:0] g_data_out,

    // Bounds
    input logic [DATA_WIDTH-1:0] u_min [INPUT_DIM],                  // Minimum input bounds
    input logic [DATA_WIDTH-1:0] u_max [INPUT_DIM],                  // Maximum input bounds
    input logic [DATA_WIDTH-1:0] x_min [STATE_DIM],                  // Minimum state bounds
    input logic [DATA_WIDTH-1:0] x_max [STATE_DIM],                  // Maximum state bounds

    // Output projected values - memory interfaces
    // Input auxiliary variables (z)
    output logic [ADDR_WIDTH-1:0] z_wraddress,
    output logic [DATA_WIDTH-1:0] z_data_in,
    output logic z_wren,
    
    // State auxiliary variables (v)
    output logic [ADDR_WIDTH-1:0] v_wraddress,
    output logic [DATA_WIDTH-1:0] v_data_in,
    output logic v_wren,
    
    // Previous z values for residuals
    output logic [ADDR_WIDTH-1:0] z_prev_wraddress,
    output logic [DATA_WIDTH-1:0] z_prev_data_in,
    output logic z_prev_wren,
    
    // Configuration
    input logic [31:0] active_horizon,  // Current horizon length to use

    output logic done                    // Done signal
);

    // State machine states
    localparam IDLE = 3'd0;
    localparam SAVE_PREV = 3'd1;        // Save previous values for residual calculation
    localparam PROJECT_Z = 3'd2;        // Project u onto bounds (z-update)
    localparam PROJECT_V = 3'd3;        // Project x onto bounds (v-update)
    localparam DONE_STATE = 3'd4;

    // State variables
    logic [2:0] state;
    logic [31:0] k;                     // Step counter
    logic [31:0] i;                     // Generic counter
    logic [31:0] state_timer;           // State timer
    
    // Memory access state variables
    logic [31:0] read_stage;             // Tracks memory read sequencing
    logic [31:0] write_stage;            // Tracks memory write sequencing
    
    // Temporary storage for values read from memory
    logic [DATA_WIDTH-1:0] temp_u;
    logic [DATA_WIDTH-1:0] temp_y;
    logic [DATA_WIDTH-1:0] temp_x;
    logic [DATA_WIDTH-1:0] temp_g;
    logic [DATA_WIDTH-1:0] temp_z;        // For saving previous z value
    
    // Temporary computation variables
    logic [DATA_WIDTH-1:0] temp_val;    // Temporary value for computations
    
    // State machine implementation
    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            state <= IDLE;
            done <= 0;
            k <= 0;
            i <= 0;
            state_timer <= 0;
            read_stage <= 0;
            write_stage <= 0;
            
            // Initialize memory control signals
            x_rdaddress <= 0;
            u_rdaddress <= 0;
            y_rdaddress <= 0;
            g_rdaddress <= 0;
            
            z_wraddress <= 0;
            z_data_in <= 0;
            z_wren <= 0;
            
            v_wraddress <= 0;
            v_data_in <= 0;
            v_wren <= 0;
            
            z_prev_wraddress <= 0;
            z_prev_data_in <= 0;
            z_prev_wren <= 0;
            
            // Initialize outputs to zero
            for (i = 0; i < INPUT_DIM*(HORIZON-1); i++) begin
                z[i] <= 0;
                z_prev[i] <= 0;
            end
            for (i = 0; i < STATE_DIM*HORIZON; i++) begin
                v[i] <= 0;
                v_prev[i] <= 0;
            end
        end else begin
            case (state)
                IDLE: begin
                    if (start) begin
                        state <= SAVE_PREV;
                        done <= 0;
                        k <= 0;
                        i <= 0;
                        state_timer <= 0;
                        read_stage <= 0;
                        write_stage <= 0;
                        
                        // Reset memory control signals
                        z_wren <= 0;
                        v_wren <= 0;
                        z_prev_wren <= 0;
                    end
                end
                
                SAVE_PREV: begin
                    // Save previous z and v values for residual computation
                        i <= 0;
                        write_stage <= 1;
                    end
                    
                    if (write_stage == 1) begin
                        // Copy z to z_prev, one element at a time
                        if (k < active_horizon-1 && i < INPUT_DIM) begin
                            // Calculate current index
                            int index = k*INPUT_DIM + i;
                            
                            // Memory access happens in multiple stages
                            if (state_timer % 3 == 1) {
                                // Stage 1: Set read address
                                z_prev_wraddress <= index;
                                z_wren <= 0;
                                z_prev_wren <= 0;
                            } else if (state_timer % 3 == 2) {
                                // Stage 2: Write to z_prev
                                z_prev_data_in <= temp_z;
                                z_prev_wren <= 1;
                            } else {
                                // Stage 3: Advance to next element
                                z_prev_wren <= 0;
                                
                                // Next element
                                i <= i + 1;
                                if (i == INPUT_DIM-1) begin
                                    i <= 0;
                                    k <= k + 1;
                                end
                            }
                        end else begin
                            // Done copying z to z_prev
                            z_prev_wren <= 0;
                            
                            // Move to next state
                            state <= PROJECT_Z;
                            k <= 0;
                            i <= 0;
                            state_timer <= 0;
                            write_stage <= 0;
                        end
                    end
                end
                
                PROJECT_Z: begin
                    // Project u onto constraints: z = proj(u + y/rho)
                    state_timer <= state_timer + 1;
                    
                    if (state_timer == 1) begin
                        // Begin the process of computing z
                        read_stage <= 1;
                        k <= 0;
                        i <= 0;
                    end
                    
                    if (read_stage == 1) begin
                        // Read u and y values for current element
                        if (k < active_horizon-1 && i < INPUT_DIM) begin
                            // Calculate current index
                            int index = k*INPUT_DIM + i;
                            
                            // Read sequence
                            if (state_timer % 5 == 1) {
                                // Set read addresses
                                u_rdaddress <= index;
                                y_rdaddress <= index;
                            } else if (state_timer % 5 == 2) {
                                // Wait for read to complete
                            } else if (state_timer % 5 == 3) {
                                // Capture values
                                temp_u <= u_data_out;
                                temp_y <= y_data_out;
                            } else if (state_timer % 5 == 4) {
                                // Calculate z = proj(u + y)
                                temp_val = temp_u + temp_y; // (note: rho=1 in simplified version)
                                
                                // Project onto bounds
                                if (temp_val < u_min[i]) begin
                                    z_data_in <= u_min[i];
                                end else if (temp_val > u_max[i]) begin
                                    z_data_in <= u_max[i];
                                end else begin
                                    z_data_in <= temp_val;
                                end
                                
                                // Set write address
                                z_wraddress <= index;
                                z_wren <= 1;
                            } else {
                                // Move to next element
                                z_wren <= 0;
                                
                                i <= i + 1;
                                if (i == INPUT_DIM-1) begin
                                    i <= 0;
                                    k <= k + 1;
                                end
                            }
                        end else begin
                            // Done projecting all inputs
                            read_stage <= 0;
                            z_wren <= 0;
                            
                            // Move to next stage
                            state <= PROJECT_V;
                            k <= 0;
                            i <= 0;
                            state_timer <= 0;
                        end
                    end
                end
                
                PROJECT_V: begin
                    // Project states onto their constraints: v = clip(x + g, x_min, x_max)
                    if (i < STATE_DIM*HORIZON) begin
                        // Calculate timestep and dimension from flat index
                        k = i / STATE_DIM; // Timestep
                        int dim = i % STATE_DIM; // Dimension
                        
                        // Projection operation
                        temp_val = x[i] + g[i]; // x[k*STATE_DIM + dim] + g[k*STATE_DIM + dim]
                        
                        // Clip to bounds
                        if (temp_val < x_min[dim]) begin
                            v_data_in <= x_min[dim];
                        end else if (temp_val > x_max[dim]) begin
                            v_data_in <= x_max[dim];
                        end else begin
                            v_data_in <= temp_val;
                        end
                        
                        // Set write address
                        v_wraddress <= i;
                        v_wren <= 1;
                        
                        i <= i + 1;
                    end else begin
                        // Done projecting all states
                        v_wren <= 0;
                        
                        // Move to next state
                        state <= DONE_STATE;
                    end
                end
                
                DONE_STATE: begin
                    done <= 1;
                    // Ensure all write enables are off
                    z_wren <= 0;
                    v_wren <= 0;
                    z_prev_wren <= 0;
                    
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
