// y-update and g-update (dual variable update) steps

`timescale 1 ps / 1 ps
module dual_update #(
    parameter STATE_DIM = 12,             // Dimension of state vector (nx)
    parameter INPUT_DIM = 4,             // Dimension of input vector (nu)
    parameter HORIZON = 30,              // Maximum MPC horizon length (N)

    parameter DATA_WIDTH = 16,           // 16-bit fixed point
    parameter FRAC_BITS = 8,             // Number of fractional bits for fixed point
    parameter ADDR_WIDTH = 9            
)(
    input logic clk,                     // Clock
    input logic rst,                     // Reset
    input logic start,                   // Start signal
    
    // ADMM variables - memory interfaces for trajectories and auxiliaries
    // State trajectory (x)
    output logic [ADDR_WIDTH-1:0] x_rdaddress,
    input logic [DATA_WIDTH_STATE-1:0] x_data_out, 
    
    // Input trajectory (u)
    output logic [ADDR_WIDTH-1:0] u_rdaddress,
    input logic [DATA_WIDTH_INPUT-1:0] u_data_out, 
    
    // Input auxiliary variables (z)
    output logic [ADDR_WIDTH-1:0] z_rdaddress,
    input logic [DATA_WIDTH_INPUT-1:0] z_data_out, 
    
    // State auxiliary variables (v)
    output logic [ADDR_WIDTH-1:0] v_rdaddress,
    input logic [DATA_WIDTH_STATE-1:0] v_data_out,

    // Dual variables - memory interfaces
    // Input dual variables (y)
    output logic [ADDR_WIDTH-1:0] y_rdaddress,
    input logic [DATA_WIDTH_INPUT-1:0] y_data_out, 
    output logic [ADDR_WIDTH-1:0] y_wraddress,
    output logic [DATA_WIDTH_INPUT-1:0] y_data_in,
    output logic y_wren,
    
    // State dual variables (g)
    output logic [ADDR_WIDTH-1:0] g_rdaddress,
    input logic [DATA_WIDTH_STATE-1:0] g_data_out,
    output logic [ADDR_WIDTH-1:0] g_wraddress,
    output logic [DATA_WIDTH_STATE-1:0] g_data_in,
    output logic g_wren,

    // Linear cost terms - memory interfaces
    // Input cost terms (r)
    output logic [ADDR_WIDTH-1:0] r_rdaddress,
    input logic [DATA_WIDTH-1:0] r_data_out,
    output logic [ADDR_WIDTH-1:0] r_wraddress,
    output logic [DATA_WIDTH-1:0] r_data_in,
    output logic r_wren,
    
    // State cost terms (q)
    output logic [ADDR_WIDTH-1:0] q_rdaddress,
    input logic [DATA_WIDTH-1:0] q_data_out,
    output logic [ADDR_WIDTH-1:0] q_wraddress,
    output logic [DATA_WIDTH-1:0] q_data_in,
    output logic q_wren,
    
    // Terminal cost terms (p)
    output logic [ADDR_WIDTH-1:0] p_rdaddress,
    input logic [DATA_WIDTH-1:0] p_data_out,
    output logic [ADDR_WIDTH-1:0] p_wraddress,
    output logic [DATA_WIDTH-1:0] p_data_in,
    output logic p_wren,
    
    // Reference trajectories
    input logic [DATA_WIDTH-1:0] x_ref [STATE_DIM][HORIZON],
    input logic [DATA_WIDTH-1:0] u_ref [INPUT_DIM][HORIZON-1],
    
    // Cost matrices
    output logic [ADDR_WIDTH-1:0] R_rdaddress,
    input logic [DATA_WIDTH-1:0] R_data_out,
    output logic [ADDR_WIDTH-1:0] Q_rdaddress,
    input logic [DATA_WIDTH-1:0] Q_data_out,
    output logic [ADDR_WIDTH-1:0] P_rdaddress,
    input logic [DATA_WIDTH-1:0] P_data_out,
    
    // ADMM parameter
    input logic [DATA_WIDTH-1:0] rho,

    // Residual calculation outputs
    output logic [DATA_WIDTH_INPUT-1:0] pri_res_u,                           // Primal residual for inputs
    output logic [DATA_WIDTH_STATE-1:0] pri_res_x,                           // Primal residual for states
    
    // Configuration
    input logic [31:0] active_horizon,  // Current horizon length to use
    
    output logic done                    // Done signal
);

    // State machine states
    localparam IDLE = 3'd0;
    localparam UPDATE_Y = 3'd1;         // Update y dual variables (for inputs)
    localparam UPDATE_G = 3'd2;         // Update g dual variables (for states)
    localparam CALC_RESIDUALS = 3'd3;   // Calculate residuals
    localparam UPDATE_LINEAR_COST = 3'd4; // Update linear cost terms
    localparam DONE_STATE = 3'd5;

    // State variables
    logic [2:0] state;
    logic [31:0] i;                     // Generic counter
    logic [31:0] k;                     // Step counter for horizon
    logic [31:0] state_timer;           // State timer
    
    // Memory access state variables
    logic [31:0] read_stage;            // Tracks memory read sequencing
    logic [31:0] write_stage;           // Tracks memory write sequencing
    
    // Temporary storage for values read from memory

    logic [DATA_WIDTH-1:0] temp_u;
    logic [DATA_WIDTH-1:0] temp_z;
    logic [DATA_WIDTH-1:0] temp_x;
    logic [DATA_WIDTH-1:0] temp_v;
    logic [DATA_WIDTH-1:0] temp_y;
    logic [DATA_WIDTH-1:0] temp_g;
    logic [DATA_WIDTH-1:0] temp_r;
    logic [DATA_WIDTH-1:0] temp_q;
    logic [DATA_WIDTH-1:0] temp_p;
    logic [DATA_WIDTH-1:0] temp_R;
    logic [DATA_WIDTH-1:0] temp_Q;
    logic [DATA_WIDTH-1:0] temp_P;
    
    // Temporary computation variables
    logic [DATA_WIDTH_STATE-1:0] temp_val_state;     // Temporary value for computations //making larger for now
    logic [DATA_WIDTH_INPUT-1:0] temp_val_input;     // Temporary value for computations //making larger for now
    logic [DATA_WIDTH_STATE-1:0] max_pri_res_u; // Maximum primal residual for inputs
    logic [DATA_WIDTH_INPUT-1:0] max_pri_res_x; // Maximum primal residual for states
    
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
            z_rdaddress <= 0;
            v_rdaddress <= 0;
            y_rdaddress <= 0;
            g_rdaddress <= 0;
            r_rdaddress <= 0;
            q_rdaddress <= 0;
            p_rdaddress <= 0;
            R_rdaddress <= 0;
            Q_rdaddress <= 0;
            P_rdaddress <= 0;
            
            y_wraddress <= 0;
            y_data_in <= 0;
            y_wren <= 0;
            
            g_wraddress <= 0;
            g_data_in <= 0;
            g_wren <= 0;
            
            r_wraddress <= 0;
            r_data_in <= 0;
            r_wren <= 0;
            
            q_wraddress <= 0;
            q_data_in <= 0;
            q_wren <= 0;
            
            p_wraddress <= 0;
            p_data_in <= 0;
            p_wren <= 0;
        end else begin
            case (state)
                IDLE: begin
                    if (start) begin
                        state <= UPDATE_Y;
                        done <= 0;
                        i <= 0;
                        k <= 0;
                        state_timer <= 0;
                        read_stage <= 0;
                        write_stage <= 0;
                        max_pri_res_u <= 0;
                        max_pri_res_x <= 0;
                        
                        // Reset all memory write enables
                        y_wren <= 0;
                        g_wren <= 0;
                        r_wren <= 0;
                        q_wren <= 0;
                        p_wren <= 0;
                    end
                end
                
                UPDATE_Y: begin
                    // Update y dual variables: y += u - z
                    state_timer <= state_timer + 1;
                    
                    if (state_timer == 1) begin
                        // Initialize the y-update process
                        read_stage <= 1;
                        k <= 0;
                        i <= 0;
                    end
                    
                    if (read_stage == 1) begin
                        // Read u, z, and y values for current element
                        if (k < active_horizon-1 && i < INPUT_DIM) begin
                            // Calculate current index
                            int index;
                            index = k*INPUT_DIM + i;
                            
                            // Memory read/write sequence
                            if (state_timer % 6 == 1) begin
                                // Stage 1: Set read addresses
                                u_rdaddress <= index;
                                z_rdaddress <= index;
                                y_rdaddress <= index;
                            end else if (state_timer % 6 == 2) begin
                                // Stage 2: Wait for read to complete
                            end else if (state_timer % 6 == 3) begin
                                // Stage 3: Capture values
                                temp_u <= u_data_out;
                                temp_z <= z_data_out;
                                temp_y <= y_data_out;
                            end else if (state_timer % 6 == 4) begin
                                // Stage 4: Compute y update and residual
                                // y += u - z
                                temp_val_input = temp_u - temp_z;
                                y_data_in <= temp_y + temp_val_input;
                                 
                                // Set write address
                                y_wraddress <= index;
                                y_wren <= 1;
                                
                                // Track maximum residual (in absolute value)
                                if (temp_val_input < 0) begin
                                    temp_val_input = -temp_val_input; // Absolute value
                                end
                                if (temp_val_input > max_pri_res_u) begin
                                    max_pri_res_u <= temp_val_input;
                                end
                            end else if (state_timer % 6 == 0) begin
                                // Stage 6: Advance to next element
                                y_wren <= 0;
                                 
                                i <= i + 1;
                                if (i == INPUT_DIM-1) begin
                                    i <= 0;
                                    k <= k + 1;
                                end
                            end
                        end else begin
                            // Done updating all y values
                            read_stage <= 0;
                            y_wren <= 0;
                             
                            // Move to next state
                            state <= UPDATE_G;
                            k <= 0;
                            i <= 0;
                            state_timer <= 0;
                        end
                    end
                end
                
                UPDATE_G: begin
                    // Update g dual variables: g += x - v
                    state_timer <= state_timer + 1;
                    
                    if (state_timer == 1) begin
                        // Initialize the g-update process
                        read_stage <= 1;
                        k <= 0;
                        i <= 0;
                    end
                    
                    if (read_stage == 1) begin
                        // Read x, v, and g values for current element
                        if (k < active_horizon && i < STATE_DIM) begin
                            // Calculate current index
                            int index;
                            index = k*STATE_DIM + i;
                            
                            // Memory read/write sequence
                            if (state_timer % 6 == 1) begin
                                // Stage 1: Set read addresses
                                x_rdaddress <= index;
                                v_rdaddress <= index;
                                g_rdaddress <= index;
                            end else if (state_timer % 6 == 2) begin
                                // Stage 2: Wait for read to complete
                            end else if (state_timer % 6 == 3) begin
                                // Stage 3: Capture values
                                temp_x <= x_data_out;
                                temp_v <= v_data_out;
                                temp_g <= g_data_out;
                            end else if (state_timer % 6 == 4) begin
                                // Stage 4: Compute g update and residual
                                // g += x - v
                                temp_val_state = temp_x - temp_v;
                                g_data_in <= temp_g + temp_val_state;
                                 
                                // Set write address
                                g_wraddress <= index;
                                g_wren <= 1;
                                
                                // Track maximum residual (in absolute value)
                                if (temp_val_state < 0) begin
                                    temp_val_state = -temp_val_state; // Absolute value
                                end
                                if (temp_val_state > max_pri_res_x) begin
                                    max_pri_res_x <= temp_val_state;
                                end
                            end else if (state_timer % 6 == 0) begin
                                // Stage 6: Advance to next element
                                g_wren <= 0;
                                 
                                i <= i + 1;
                                if (i == STATE_DIM-1) begin
                                    i <= 0;
                                    k <= k + 1;
                                end
                            end
                        end else begin
                            // Done updating all g values
                            read_stage <= 0;
                            g_wren <= 0;
                             
                            // Move to next state
                            state <= CALC_RESIDUALS;
                            state_timer <= 0;
                        end
                    end
                end
                
                CALC_RESIDUALS: begin
                    // Store final residual values
                    pri_res_u <= max_pri_res_u;
                    pri_res_x <= max_pri_res_x;
                    state <= UPDATE_LINEAR_COST;
                    k <= 0;
                    i <= 0;
                    state_timer <= 0;
                end
                
                UPDATE_LINEAR_COST: begin
                    // Update linear cost terms: r, q, and p
                    state_timer <= state_timer + 1;
                    
                    if (state_timer == 1) begin
                        // Initialize the linear cost update process
                        read_stage <= 1;
                        k <= 0;
                        i <= 0;
                    end
                    
                    if (read_stage == 1) begin
                        // First update r and q for k=0 to N-2
                        if (k < active_horizon-1) begin
                            // Update r vector (for inputs)
                            if (i < INPUT_DIM) begin
                                // Calculate current index
                                int index = k*INPUT_DIM + i;
                                
                                // Memory read/write sequence
                                if (state_timer % 8 == 1) begin
                                    // Stage 1: Set read addresses for r update
                                    R_rdaddress <= i*INPUT_DIM + i; // Diagonal element of R
                                    z_rdaddress <= index;
                                    y_rdaddress <= index;
                                end else if (state_timer % 8 == 2) begin
                                    // Stage 2: Wait for read to complete
                                end else if (state_timer % 8 == 3) begin
                                    // Stage 3: Capture values
                                    temp_R <= R_data_out;
                                    temp_z <= z_data_out;
                                    temp_y <= y_data_out;
                                end else if (state_timer % 8 == 4) begin
                                    // Stage 4: Compute r update
                                    // r[:, k] = -R @ u_ref[:, k]
                                    // r[:, k] -= rho * (z[:, k] - y[:, k])
                                    temp_val = -temp_R * u_ref[i][k]; // Simplified matrix-vector product
                                    r_data_in <= temp_val - rho * (temp_z - temp_y);
                                    
                                    // Set write address
                                    r_wraddress <= index;
                                    r_wren <= 1;
                                end else if (state_timer % 8 == 5) begin
                                    // Stage 5: Set read addresses for q update
                                    Q_rdaddress <= i*STATE_DIM + i; // Diagonal element of Q
                                    v_rdaddress <= k*STATE_DIM + i;
                                    g_rdaddress <= k*STATE_DIM + i;
                                end else if (state_timer % 8 == 6) begin
                                    // Stage 6: Wait for read to complete
                                    r_wren <= 0;
                                end else if (state_timer % 8 == 7) begin
                                    // Stage 7: Capture values
                                    temp_Q <= Q_data_out;
                                    temp_v <= v_data_out;
                                    temp_g <= g_data_out;
                                end else begin
                                    // Stage 8: Compute q update
                                    // q[:, k] = -Q @ x_ref[:, k]
                                    // q[:, k] -= rho * (v[:, k] - g[:, k])
                                    temp_val = -temp_Q * x_ref[i][k]; // Simplified matrix-vector product
                                    q_data_in <= temp_val - rho * (temp_v - temp_g);
                                    
                                    // Set write address
                                    q_wraddress <= k*STATE_DIM + i;
                                    q_wren <= 1;
                                    
                                    // Advance to next element
                                    i <= i + 1;
                                    if (i == STATE_DIM-1) begin
                                        i <= 0;
                                        k <= k + 1;
                                    end
                                    q_wren <= 0;
                                end
                            end
                        end else if (k == active_horizon-1) begin
                            // Update p vector for the terminal state (k=N-1)
                            if (i < STATE_DIM) begin
                                // Calculate current index
                                int index = k*STATE_DIM + i;
                                
                                // Memory read/write sequence
                                if (state_timer % 6 == 1) begin
                                    // Stage 1: Set read addresses
                                    P_rdaddress <= i*STATE_DIM + i; // Diagonal element of P
                                    v_rdaddress <= index;
                                    g_rdaddress <= index;
                                end else if (state_timer % 6 == 2) begin
                                    // Stage 2: Wait for read to complete
                                end else if (state_timer % 6 == 3) begin
                                    // Stage 3: Capture values
                                    temp_P <= P_data_out;
                                    temp_v <= v_data_out;
                                    temp_g <= g_data_out;
                                end else if (state_timer % 6 == 4) begin
                                    // Stage 4: Compute p update
                                    // p[:,N-1] = -P @ x_ref[:, N-1]
                                    // p[:,N-1] -= rho * (v[:, N-1] - g[:, N-1])
                                    temp_val = -temp_P * x_ref[i][k]; // Simplified matrix-vector product
                                    p_data_in <= temp_val - rho * (temp_v - temp_g);
                                    
                                    // Set write address
                                    p_wraddress <= index;
                                    p_wren <= 1;
                                end else if (state_timer % 6 == 0) begin
                                    // Stage 6: Advance to next element
                                    p_wren <= 0;
                                    
                                    i <= i + 1;
                                    if (i == STATE_DIM-1) begin
                                        // Done with all updates
                                        read_stage <= 0;
                                        state <= DONE_STATE;
                                    end
                                end
                            end
                        end else begin
                            // Done with all updates
                            read_stage <= 0;
                            r_wren <= 0;
                            q_wren <= 0;
                            p_wren <= 0;
                            state <= DONE_STATE;
                        end
                    end
                end
                
                DONE_STATE: begin
                    done <= 1;
                    // Ensure all write enables are off
                    y_wren <= 0;
                    g_wren <= 0;
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
