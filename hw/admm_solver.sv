module admm_solver #(
    parameter STATE_DIM = 12,              // Dimension of state vector (nx)
    parameter INPUT_DIM = 4,              // Dimension of input vector (nu)
    parameter HORIZON = 30,               // Maximum MPC horizon length (N)
    parameter MAX_ITER = 500,             // Maximum ADMM iterations
    parameter EXT_DATA_WIDTH = 32,        // Bus data width
    parameter DATA_WIDTH = 16,            // Internal calculation width (16-bit fixed point)
    parameter FRAC_BITS = 8,              // Number of fractional bits for fixed point
    parameter ADDR_WIDTH = 16,            // Address width for bus interface
    parameter MEM_ADDR_WIDTH = 9          // internal memory
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
    output logic [EXT_DATA_WIDTH-1:0] readdata
);

    // State machine states
    typedef enum logic [2:0] {
        IDLE = 3'd0,
        INIT = 3'd1,
        STAGE1 = 3'd2,  // X-Update (Riccati recursion & trajectory rollout)
        STAGE2 = 3'd3,  // Z-Update (Projection)
        STAGE3 = 3'd4,  // Y-Update (Dual variables)
        CHECK_CONVERGENCE = 3'd5,
        OUTPUT_RESULTS = 3'd6,
        DONE = 3'd7
    } solver_state_t;
    
    solver_state_t state;
    
    // Control signals
    logic start_solving;
    logic solver_done;
    logic [31:0] current_iter;
    logic [31:0] active_horizon;
    logic converged;
    
    // Memory signals for system matrices
    logic [MEM_ADDR_WIDTH-1:0] A_rdaddress, A_wraddress;
    logic [DATA_WIDTH-1:0] A_data_out, A_data_in;
    logic A_wren;
    
    logic [MEM_ADDR_WIDTH-1:0] B_rdaddress, B_wraddress;
    logic [DATA_WIDTH-1:0] B_data_out, B_data_in;
    logic B_wren;
    
    logic [MEM_ADDR_WIDTH-1:0] Q_rdaddress, Q_wraddress;
    logic [DATA_WIDTH-1:0] Q_data_out, Q_data_in;
    logic Q_wren;
    
    logic [MEM_ADDR_WIDTH-1:0] R_rdaddress, R_wraddress;
    logic [DATA_WIDTH-1:0] R_data_out, R_data_in;
    logic R_wren;
    
    // Memory signals for precomputed terms
    logic [MEM_ADDR_WIDTH-1:0] K_rdaddress, K_wraddress;
    logic [DATA_WIDTH-1:0] K_data_out, K_data_in;
    logic K_wren;
    
    logic [MEM_ADDR_WIDTH-1:0] C1_rdaddress, C1_wraddress;
    logic [DATA_WIDTH-1:0] C1_data_out, C1_data_in;
    logic C1_wren;
    
    logic [MEM_ADDR_WIDTH-1:0] C2_rdaddress, C2_wraddress;
    logic [DATA_WIDTH-1:0] C2_data_out, C2_data_in;
    logic C2_wren;
    
    logic [MEM_ADDR_WIDTH-1:0] P_rdaddress, P_wraddress;
    logic [DATA_WIDTH-1:0] P_data_out, P_data_in;
    logic P_wren;
    
    // Memory signals for trajectories
    logic [MEM_ADDR_WIDTH-1:0] x_rdaddress, x_wraddress;
    logic [DATA_WIDTH-1:0] x_data_out, x_data_in;
    logic x_wren;
    
    logic [MEM_ADDR_WIDTH-1:0] u_rdaddress, u_wraddress;
    logic [DATA_WIDTH-1:0] u_data_out, u_data_in;
    logic u_wren;
    
    // Memory signals for auxiliary variables
    logic [MEM_ADDR_WIDTH-1:0] z_rdaddress, z_wraddress;
    logic [DATA_WIDTH-1:0] z_data_out, z_data_in;
    logic z_wren;
    
    logic [MEM_ADDR_WIDTH-1:0] v_rdaddress, v_wraddress;
    logic [DATA_WIDTH-1:0] v_data_out, v_data_in;
    logic v_wren;
    
    // Memory signals for previous values (for residuals)
    logic [MEM_ADDR_WIDTH-1:0] z_prev_rdaddress, z_prev_wraddress;
    logic [DATA_WIDTH-1:0] z_prev_data_out, z_prev_data_in;
    logic z_prev_wren;
    
    logic [MEM_ADDR_WIDTH-1:0] v_prev_rdaddress, v_prev_wraddress;
    logic [DATA_WIDTH-1:0] v_prev_data_out, v_prev_data_in;
    logic v_prev_wren;
    
    // Memory signals for dual variables
    logic [MEM_ADDR_WIDTH-1:0] y_rdaddress, y_wraddress;
    logic [DATA_WIDTH-1:0] y_data_out, y_data_in;
    logic y_wren;
    
    logic [MEM_ADDR_WIDTH-1:0] g_rdaddress, g_wraddress;
    logic [DATA_WIDTH-1:0] g_data_out, g_data_in;
    logic g_wren;
    
    // Memory signals for linear cost terms
    logic [MEM_ADDR_WIDTH-1:0] q_rdaddress, q_wraddress;
    logic [DATA_WIDTH-1:0] q_data_out, q_data_in;
    logic q_wren;
    
    logic [MEM_ADDR_WIDTH-1:0] r_rdaddress, r_wraddress;
    logic [DATA_WIDTH-1:0] r_data_out, r_data_in;
    logic r_wren;
    
    logic [MEM_ADDR_WIDTH-1:0] p_rdaddress, p_wraddress;
    logic [DATA_WIDTH-1:0] p_data_out, p_data_in;
    logic p_wren;
    
    logic [MEM_ADDR_WIDTH-1:0] d_rdaddress, d_wraddress;
    logic [DATA_WIDTH-1:0] d_data_out, d_data_in;
    logic d_wren;
    
    // Stage control signals
    logic stage1_start, stage1_done;
    logic stage2_start, stage2_done;
    logic stage3_start, stage3_done;
    logic residual_calc_start, residual_calc_done;
    
    // Residuals
    logic [DATA_WIDTH-1:0] pri_res_u, pri_res_x, dual_res;
    logic [DATA_WIDTH-1:0] pri_tol, dual_tol;
    
    // Bounds
    logic [DATA_WIDTH-1:0] u_min [INPUT_DIM];
    logic [DATA_WIDTH-1:0] u_max [INPUT_DIM];
    logic [DATA_WIDTH-1:0] x_min [STATE_DIM];
    logic [DATA_WIDTH-1:0] x_max [STATE_DIM];
    
    // Initial state
    logic [DATA_WIDTH-1:0] x_init [STATE_DIM];
    
    // Reference trajectories
    logic [DATA_WIDTH-1:0] x_ref [STATE_DIM][HORIZON];
    logic [DATA_WIDTH-1:0] u_ref [INPUT_DIM][HORIZON-1];
    
    // ADMM parameter
    logic [DATA_WIDTH-1:0] rho;
    
    // Memory blocks for system matrices and trajectories using RAM modules
    
    // System matrices - A matrix (STATE_DIM x STATE_DIM)
    RAM A_ram (
        .clock(clk),
        .data(A_data_in),
        .rdaddress(A_rdaddress),
        .wraddress(A_wraddress),
        .wren(A_wren),
        .q(A_data_out)
    );
    
    // System matrices - B matrix (STATE_DIM x INPUT_DIM)
    RAM B_ram (
        .clock(clk),
        .data(B_data_in),
        .rdaddress(B_rdaddress),
        .wraddress(B_wraddress),
        .wren(B_wren),
        .q(B_data_out)
    );
    
    // System matrices - Q matrix (STATE_DIM x STATE_DIM)
    RAM Q_ram (
        .clock(clk),
        .data(Q_data_in),
        .rdaddress(Q_rdaddress),
        .wraddress(Q_wraddress),
        .wren(Q_wren),
        .q(Q_data_out)
    );
    
    // System matrices - R matrix (INPUT_DIM x INPUT_DIM)
    RAM R_ram (
        .clock(clk),
        .data(R_data_in),
        .rdaddress(R_rdaddress),
        .wraddress(R_wraddress),
        .wren(R_wren),
        .q(R_data_out)
    );
    
    // Precomputed cache terms - K matrix (Kinf) (INPUT_DIM x STATE_DIM)
    RAM K_ram (
        .clock(clk),
        .data(K_data_in),
        .rdaddress(K_rdaddress),
        .wraddress(K_wraddress),
        .wren(K_wren),
        .q(K_data_out)
    );
    
    // Precomputed cache terms - C1 matrix ((R + B'*P*B)^-1) (INPUT_DIM x INPUT_DIM)
    RAM C1_ram (
        .clock(clk),
        .data(C1_data_in),
        .rdaddress(C1_rdaddress),
        .wraddress(C1_wraddress),
        .wren(C1_wren),
        .q(C1_data_out)
    );
    
    // Precomputed cache terms - C2 matrix ((A - B*K)') (STATE_DIM x STATE_DIM)
    RAM C2_ram (
        .clock(clk),
        .data(C2_data_in),
        .rdaddress(C2_rdaddress),
        .wraddress(C2_wraddress),
        .wren(C2_wren),
        .q(C2_data_out)
    );
    
    // Precomputed cache terms - P matrix (Pinf) (STATE_DIM x STATE_DIM)
    RAM P_ram (
        .clock(clk),
        .data(P_data_in),
        .rdaddress(P_rdaddress),
        .wraddress(P_wraddress),
        .wren(P_wren),
        .q(P_data_out)
    );
    
    // Trajectories - x trajectory (STATE_DIM x HORIZON)
    RAM x_ram (
        .clock(clk),
        .data(x_data_in),
        .rdaddress(x_rdaddress),
        .wraddress(x_wraddress),
        .wren(x_wren),
        .q(x_data_out)
    );
    
    // Trajectories - u trajectory (INPUT_DIM x (HORIZON-1))
    RAM u_ram (
        .clock(clk),
        .data(u_data_in),
        .rdaddress(u_rdaddress),
        .wraddress(u_wraddress),
        .wren(u_wren),
        .q(u_data_out)
    );
    
    // Trajectories - z trajectory (INPUT_DIM x (HORIZON-1))
    RAM z_ram (
        .clock(clk),
        .data(z_data_in),
        .rdaddress(z_rdaddress),
        .wraddress(z_wraddress),
        .wren(z_wren),
        .q(z_data_out)
    );
    
    // Trajectories - v trajectory (STATE_DIM x HORIZON)
    RAM v_ram (
        .clock(clk),
        .data(v_data_in),
        .rdaddress(v_rdaddress),
        .wraddress(v_wraddress),
        .wren(v_wren),
        .q(v_data_out)
    );
    
    // Previous values for residuals - z_prev (INPUT_DIM x (HORIZON-1))
    RAM z_prev_ram (
        .clock(clk),
        .data(z_prev_data_in),
        .rdaddress(z_prev_rdaddress),
        .wraddress(z_prev_wraddress),
        .wren(z_prev_wren),
        .q(z_prev_data_out)
    );
    
    // Previous values for residuals - v_prev (STATE_DIM x HORIZON)
    RAM v_prev_ram (
        .clock(clk),
        .data(v_prev_data_in),
        .rdaddress(v_prev_rdaddress),
        .wraddress(v_prev_wraddress),
        .wren(v_prev_wren),
        .q(v_prev_data_out)
    );
    
    // Dual variables - y (INPUT_DIM x (HORIZON-1))
    RAM y_ram (
        .clock(clk),
        .data(y_data_in),
        .rdaddress(y_rdaddress),
        .wraddress(y_wraddress),
        .wren(y_wren),
        .q(y_data_out)
    );
    
    // Dual variables - g (STATE_DIM x HORIZON)
    RAM g_ram (
        .clock(clk),
        .data(g_data_in),
        .rdaddress(g_rdaddress),
        .wraddress(g_wraddress),
        .wren(g_wren),
        .q(g_data_out)
    );
    
    // Linear cost terms - q (STATE_DIM x HORIZON)
    RAM q_ram (
        .clock(clk),
        .data(q_data_in),
        .rdaddress(q_rdaddress),
        .wraddress(q_wraddress),
        .wren(q_wren),
        .q(q_data_out)
    );
    
    // Linear cost terms - r (INPUT_DIM x (HORIZON-1))
    RAM r_ram (
        .clock(clk),
        .data(r_data_in),
        .rdaddress(r_rdaddress),
        .wraddress(r_wraddress),
        .wren(r_wren),
        .q(r_data_out)
    );
    
    // Linear cost terms - p (STATE_DIM x HORIZON)
    RAM p_ram (
        .clock(clk),
        .data(p_data_in),
        .rdaddress(p_rdaddress),
        .wraddress(p_wraddress),
        .wren(p_wren),
        .q(p_data_out)
    );
    
    // Linear cost terms - d (INPUT_DIM x (HORIZON-1))
    RAM d_ram (
        .clock(clk),
        .data(d_data_in),
        .rdaddress(d_rdaddress),
        .wraddress(d_wraddress),
        .wren(d_wren),
        .q(d_data_out)
    );
    
    // Instantiate solver stage 1 - X-Update (Riccati recursion & trajectory rollout)
    primal_update #(
        .STATE_DIM(STATE_DIM),
        .INPUT_DIM(INPUT_DIM),
        .HORIZON(HORIZON),
        .DATA_WIDTH(DATA_WIDTH),
        .ADDR_WIDTH(MEM_ADDR_WIDTH)
    ) stage1_inst (
        .clk(clk),
        .rst(rst),
        .start(stage1_start),
        
        // System matrices
        .A_rdaddress(A_rdaddress),
        .A_data_out(A_data_out),
        .B_rdaddress(B_rdaddress),
        .B_data_out(B_data_out),
        .K_rdaddress(K_rdaddress),
        .K_data_out(K_data_out),
        .C1_rdaddress(C1_rdaddress),
        .C1_data_out(C1_data_out),
        .C2_rdaddress(C2_rdaddress),
        .C2_data_out(C2_data_out),
        
        // Initial state
        .x_init(x_init),
        
        // Linear cost terms
        .p_rdaddress(p_rdaddress),
        .p_data_out(p_data_out),
        .r_rdaddress(r_rdaddress),
        .r_data_out(r_data_out),
        .q_rdaddress(q_rdaddress),
        .q_data_out(q_data_out),
        
        // Output trajectories
        .x_wraddress(x_wraddress),
        .x_data_in(x_data_in),
        .x_wren(x_wren),
        
        .u_wraddress(u_wraddress),
        .u_data_in(u_data_in),
        .u_wren(u_wren),
        
        .d_wraddress(d_wraddress),
        .d_data_in(d_data_in),
        .d_wren(d_wren),
        
        // Added p vector write ports
        .p_wraddress(p_wraddress),
        .p_data_in(p_data_in),
        .p_wren(p_wren),
        
        // Configuration
        .active_horizon(active_horizon),
        
        .done(stage1_done)
    );
    
    // Instantiate solver stage 2 - Z-Update (Projection)
    slack_update #(
        .STATE_DIM(STATE_DIM),
        .INPUT_DIM(INPUT_DIM),
        .HORIZON(HORIZON),
        .DATA_WIDTH(DATA_WIDTH),
        .ADDR_WIDTH(MEM_ADDR_WIDTH)
    ) stage2_inst (
        .clk(clk),
        .rst(rst),
        .start(stage2_start),
        
        // Trajectory inputs
        .x_rdaddress(x_rdaddress),
        .x_data_out(x_data_out),
        .u_rdaddress(u_rdaddress),
        .u_data_out(u_data_out),
        .y_rdaddress(y_rdaddress),
        .y_data_out(y_data_out),
        .g_rdaddress(g_rdaddress),
        .g_data_out(g_data_out),
        
        // Bounds
        .u_min(u_min),
        .u_max(u_max),
        .x_min(x_min),
        .x_max(x_max),
        
        // Output auxiliary variables
        .z_wraddress(z_wraddress),
        .z_data_in(z_data_in),
        .z_wren(z_wren),
        .v_wraddress(v_wraddress),
        .v_data_in(v_data_in),
        .v_wren(v_wren),
        
        // Previous values for residuals
        .z_prev_wraddress(z_prev_wraddress),
        .z_prev_data_in(z_prev_data_in),
        .z_prev_wren(z_prev_wren),
        
        // Configuration
        .active_horizon(active_horizon),
        
        .done(stage2_done)
    );
    
    // Instantiate solver stage 3 - Y-Update (Dual variables)
    dual_update #(
        .STATE_DIM(STATE_DIM),
        .INPUT_DIM(INPUT_DIM),
        .HORIZON(HORIZON),
        .DATA_WIDTH(DATA_WIDTH),
        .ADDR_WIDTH(MEM_ADDR_WIDTH)
    ) stage3_inst (
        .clk(clk),
        .rst(rst),
        .start(stage3_start),
        
        // Trajectory inputs
        .x_rdaddress(x_rdaddress),
        .x_data_out(x_data_out),
        .u_rdaddress(u_rdaddress),
        .u_data_out(u_data_out),
        .z_rdaddress(z_rdaddress),
        .z_data_out(z_data_out),
        .v_rdaddress(v_rdaddress),
        .v_data_out(v_data_out),
        
        // Dual variables
        .y_rdaddress(y_rdaddress),
        .y_data_out(y_data_out),
        .y_wraddress(y_wraddress),
        .y_data_in(y_data_in),
        .y_wren(y_wren),
        
        .g_rdaddress(g_rdaddress),
        .g_data_out(g_data_out),
        .g_wraddress(g_wraddress),
        .g_data_in(g_data_in),
        .g_wren(g_wren),
        
        // Linear cost terms
        .r_rdaddress(r_rdaddress),
        .r_data_out(r_data_out),
        .r_wraddress(r_wraddress),
        .r_data_in(r_data_in),
        .r_wren(r_wren),
        
        .q_rdaddress(q_rdaddress),
        .q_data_out(q_data_out),
        .q_wraddress(q_wraddress),
        .q_data_in(q_data_in),
        .q_wren(q_wren),
        
        .p_rdaddress(p_rdaddress),
        .p_data_out(p_data_out),
        .p_wraddress(p_wraddress),
        .p_data_in(p_data_in),
        .p_wren(p_wren),
        
        // Reference trajectories
        .x_ref(x_ref),
        .u_ref(u_ref),
        
        // Cost matrices
        .R_rdaddress(R_rdaddress),
        .R_data_out(R_data_out),
        .Q_rdaddress(Q_rdaddress),
        .Q_data_out(Q_data_out),
        .P_rdaddress(P_rdaddress),
        .P_data_out(P_data_out),
        
        // ADMM parameter
        .rho(rho),
        
        // Residuals
        .pri_res_u(pri_res_u),
        .pri_res_x(pri_res_x),
        
        // Configuration
        .active_horizon(active_horizon),
        
        .done(stage3_done)
    );
    
    // Instantiate residual calculator for convergence check
    residual_calculator #(
        .STATE_DIM(STATE_DIM),
        .INPUT_DIM(INPUT_DIM),
        .HORIZON(HORIZON),
        .DATA_WIDTH(DATA_WIDTH),
        .ADDR_WIDTH(MEM_ADDR_WIDTH)
    ) residual_calc_inst (
        .clk(clk),
        .rst(rst),
        .start(residual_calc_start),
        
        // Residual inputs from stage 3
        .pri_res_u(pri_res_u),
        .pri_res_x(pri_res_x),
        
        // Z and Z_prev for dual residual
        .z_rdaddress(z_rdaddress),
        .z_data_out(z_data_out),
        .z_prev_rdaddress(z_prev_rdaddress),
        .z_prev_data_out(z_prev_data_out),
        
        // Tolerance thresholds
        .pri_tol(pri_tol),
        .dual_tol(dual_tol),
        
        // Configuration
        .active_horizon(active_horizon),
        
        // Outputs
        .dual_res(dual_res),
        .converged(converged),
        .done(residual_calc_done)
    );
    
    // Main state machine to control the ADMM solver
    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            state <= IDLE;
            current_iter <= 0;
            active_horizon <= HORIZON;
            solver_done <= 0;
            
            // Initialize control signals
            stage1_start <= 0;
            stage2_start <= 0;
            stage3_start <= 0;
            residual_calc_start <= 0;
            
            // Initialize tolerances (default values)
            pri_tol <= 64'h3F50624DD2F1A9FC; // 0.001 in double precision
            dual_tol <= 64'h3F50624DD2F1A9FC; // 0.001 in double precision
            
            // Initialize rho parameter
            rho <= 64'h3FF0000000000000; // 1.0 in double precision
            
        end else begin
            // Default values for control signals
            stage1_start <= 0;
            stage2_start <= 0;
            stage3_start <= 0;
            residual_calc_start <= 0;
            
            case (state)
                IDLE: begin
                    if (start_solving) begin
                        state <= INIT;
                        current_iter <= 0;
                        solver_done <= 0;
                    end
                end
                
                INIT: begin
                    // TODO: INIT
                    state <= STAGE1;
                end
                
                STAGE1: begin
                    // X-Update stage (Riccati recursion and forward rollout)
                    if (!stage1_start && !stage1_done) begin
                        stage1_start <= 1;
                    end else if (stage1_done) begin
                        state <= STAGE2;
                    end
                end
                
                STAGE2: begin
                    // Z-Update stage (Projection)
                    if (!stage2_start && !stage2_done) begin
                        stage2_start <= 1;
                    end else if (stage2_done) begin
                        state <= STAGE3;
                    end
                end
                
                STAGE3: begin
                    // Y-Update stage (Dual variables)
                    if (!stage3_start && !stage3_done) begin
                        stage3_start <= 1;
                    end else if (stage3_done) begin
                        state <= CHECK_CONVERGENCE;
                    end
                end
                
                CHECK_CONVERGENCE: begin
                    // Check convergence criteria
                    if (!residual_calc_start && !residual_calc_done) begin
                        residual_calc_start <= 1;
                    end else if (residual_calc_done) begin
                        if (converged || current_iter >= MAX_ITER-1) begin
                            state <= OUTPUT_RESULTS;
                        end else begin
                            current_iter <= current_iter + 1;
                            state <= STAGE1; // Start next iteration
                        end
                    end
                end
                
                OUTPUT_RESULTS: begin
                    //TODO: FINISH
                    state <= DONE;
                end
                
                DONE: begin
                    solver_done <= 1;
                    if (!start_solving) begin
                        state <= IDLE; // Return to IDLE when start is deasserted
                    end
                end
                
                default: state <= IDLE;
            endcase
        end
    end
    
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
            
            // Reset indices for memory operations
            int i, j;
            for (i = 0; i < STATE_DIM; i++) begin
                for (j = 0; j < STATE_DIM; j++) begin
                    A_matrix[i][j] <= 0;
                end
                for (j = 0; j < INPUT_DIM; j++) begin
                    B_matrix[i][j] <= 0;
                end
                x_init[i] <= 0;
                x_min[i] <= -64'h7FEFFFFFFFFFFFFF; // -MAX_FLOAT
                x_max[i] <= 64'h7FEFFFFFFFFFFFFF;  // MAX_FLOAT
            end
            
            for (i = 0; i < INPUT_DIM; i++) begin
                u_min[i] <= -64'h7FEFFFFFFFFFFFFF; // -MAX_FLOAT
                u_max[i] <= 64'h7FEFFFFFFFFFFFFF;  // MAX_FLOAT
            end
            
            // Reset tolerances to default values
            pri_tol <= 64'h3F50624DD2F1A9FC; // 0.001 in double precision
            dual_tol <= 64'h3F50624DD2F1A9FC; // 0.001 in double precision
            
            // Reset rho parameter
            rho <= 64'h3FF0000000000000; // 1.0 in double precision
            
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
                            12'h010: readdata <= pri_res_u[31:0];       // Primal residual (inputs) - lower 32 bits
                            12'h014: readdata <= pri_res_u[63:32];      // Primal residual (inputs) - upper 32 bits
                            12'h018: readdata <= pri_res_x[31:0];       // Primal residual (states) - lower 32 bits
                            12'h01C: readdata <= pri_res_x[63:32];      // Primal residual (states) - upper 32 bits
                            12'h020: readdata <= dual_res[31:0];        // Dual residual - lower 32 bits
                            12'h024: readdata <= dual_res[63:32];       // Dual residual - upper 32 bits
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
                            // Return lower 32 bits of the state value
                            readdata <= x_data_out[31:0];
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
                            // Return lower 32 bits of the input value
                            readdata <= u_data_out[31:0];
                        end else begin
                            readdata <= 0;
                        end
                    end
                    
                    // Upper 32 bits of optimized state trajectory (0x3000-0x3FFF)
                    4'h3: begin
                        int state_idx = addr[11:8];    // Which state variable (0 to STATE_DIM-1)
                        int horizon_idx = addr[7:0];   // Which horizon step (0 to HORIZON-1)
                        
                        if (state_idx < STATE_DIM && horizon_idx < HORIZON) begin
                            // Set read address for x memory
                            x_rdaddress <= horizon_idx * STATE_DIM + state_idx;
                            // Return upper 32 bits of the state value
                            readdata <= x_data_out[63:32];
                        end else begin
                            readdata <= 0;
                        end
                    end
                    
                    // Upper 32 bits of optimized input trajectory (0x4000-0x4FFF)
                    4'h4: begin
                        int input_idx = addr[11:8];    // Which input variable (0 to INPUT_DIM-1)
                        int horizon_idx = addr[7:0];   // Which horizon step (0 to HORIZON-2)
                        
                        if (input_idx < INPUT_DIM && horizon_idx < HORIZON-1) begin
                            // Set read address for u memory
                            u_rdaddress <= horizon_idx * INPUT_DIM + input_idx;
                            // Return upper 32 bits of the input value
                            readdata <= u_data_out[63:32];
                        end else begin
                            readdata <= 0;
                        end
                    end
                    
                    default: readdata <= 0;
                endcase
            end else if (write) begin
                // Write operations
                case (addr[15:12]) // Use upper 4 bits for register type
                    // Status and configuration registers (0x0000-0x0FFF)
                    4'h0: begin
                        case (addr[11:0])
                            12'h000: start_solving <= writedata[0];     // Start solver
                            12'h004: active_horizon <= writedata;       // Set horizon length
                            12'h008: pri_tol[31:0] <= writedata;        // Set primal tolerance (lower 32 bits)
                            12'h00C: pri_tol[63:32] <= writedata;       // Set primal tolerance (upper 32 bits)
                            12'h010: dual_tol[31:0] <= writedata;       // Set dual tolerance (lower 32 bits)
                            12'h014: dual_tol[63:32] <= writedata;      // Set dual tolerance (upper 32 bits)
                            12'h018: rho[31:0] <= writedata;            // Set rho parameter (lower 32 bits)
                            12'h01C: rho[63:32] <= writedata;           // Set rho parameter (upper 32 bits)
                        endcase
                    end
                    
                    // System matrix A (0x1000-0x1FFF)
                    4'h1: begin
                        int row = addr[11:8];    // Row index (0 to STATE_DIM-1)
                        int col = addr[7:4];     // Column index (0 to STATE_DIM-1)
                        int is_upper = addr[3];  // 0 for lower 32 bits, 1 for upper 32 bits
                        
                        if (row < STATE_DIM && col < STATE_DIM) begin
                            if (is_upper) begin
                                // Store upper 32 bits and write to memory
                                A_matrix[row][col][63:32] <= writedata;
                                A_wraddress <= row * STATE_DIM + col;
                                A_data_in <= A_matrix[row][col];
                                A_wren <= 1;
                            end else begin
                                // Store lower 32 bits (don't write to memory yet)
                                A_matrix[row][col][31:0] <= writedata;
                            end
                        end
                    end
                    
                    // System matrix B (0x2000-0x2FFF)
                    4'h2: begin
                        int row = addr[11:8];    // Row index (0 to STATE_DIM-1)
                        int col = addr[7:4];     // Column index (0 to INPUT_DIM-1)
                        int is_upper = addr[3];  // 0 for lower 32 bits, 1 for upper 32 bits
                        
                        if (row < STATE_DIM && col < INPUT_DIM) begin
                            if (is_upper) begin
                                // Store upper 32 bits and write to memory
                                B_matrix[row][col][63:32] <= writedata;
                                B_wraddress <= row * INPUT_DIM + col;
                                B_data_in <= B_matrix[row][col];
                                B_wren <= 1;
                            end else begin
                                // Store lower 32 bits (don't write to memory yet)
                                B_matrix[row][col][31:0] <= writedata;
                            end
                        end
                    end
                    
                    // Cost matrix Q (0x3000-0x3FFF)
                    4'h3: begin
                        int row = addr[11:8];    // Row index (0 to STATE_DIM-1)
                        int col = addr[7:4];     // Column index (0 to STATE_DIM-1)
                        int is_upper = addr[3];  // 0 for lower 32 bits, 1 for upper 32 bits
                        
                        if (row < STATE_DIM && col < STATE_DIM) begin
                            if (is_upper) begin
                                // Store upper 32 bits and write to memory
                                Q_wraddress <= row * STATE_DIM + col;
                                Q_data_in <= {writedata, Q_data_in[31:0]};
                                Q_wren <= 1;
                            end else begin
                                // Store lower 32 bits (don't write to memory yet)
                                Q_data_in[31:0] <= writedata;
                            end
                        end
                    end
                    
                    // Cost matrix R (0x4000-0x4FFF)
                    4'h4: begin
                        int row = addr[11:8];    // Row index (0 to INPUT_DIM-1)
                        int col = addr[7:4];     // Column index (0 to INPUT_DIM-1)
                        int is_upper = addr[3];  // 0 for lower 32 bits, 1 for upper 32 bits
                        
                        if (row < INPUT_DIM && col < INPUT_DIM) begin
                            if (is_upper) begin
                                // Store upper 32 bits and write to memory
                                R_wraddress <= row * INPUT_DIM + col;
                                R_data_in <= {writedata, R_data_in[31:0]};
                                R_wren <= 1;
                            end else begin
                                // Store lower 32 bits (don't write to memory yet)
                                R_data_in[31:0] <= writedata;
                            end
                        end
                    end
                    
                    // Initial state x_init (0x5000-0x5FFF)
                    4'h5: begin
                        int state_idx = addr[11:8];    // Which state variable (0 to STATE_DIM-1)
                        int is_upper = addr[7];        // 0 for lower 32 bits, 1 for upper 32 bits
                        
                        if (state_idx < STATE_DIM) begin
                            if (is_upper) begin
                                // Store upper 32 bits
                                x_init[state_idx][63:32] <= writedata;
                            end else begin
                                // Store lower 32 bits
                                x_init[state_idx][31:0] <= writedata;
                            end
                        end
                    end
                    
                    // State reference trajectory x_ref (0x6000-0x6FFF)
                    4'h6: begin
                        int state_idx = addr[11:8];    // Which state variable (0 to STATE_DIM-1)
                        int horizon_idx = addr[7:0];   // Which horizon step (0 to HORIZON-1)
                        
                        if (state_idx < STATE_DIM && horizon_idx < HORIZON) begin
                            // Store reference value (only lower 32 bits for simplicity)
                            x_ref[state_idx][horizon_idx][31:0] <= writedata;
                        end
                    end
                    
                    // Input reference trajectory u_ref (0x7000-0x7FFF)
                    4'h7: begin
                        int input_idx = addr[11:8];    // Which input variable (0 to INPUT_DIM-1)
                        int horizon_idx = addr[7:0];   // Which horizon step (0 to HORIZON-2)
                        
                        if (input_idx < INPUT_DIM && horizon_idx < HORIZON-1) begin
                            // Store reference value (only lower 32 bits for simplicity)
                            u_ref[input_idx][horizon_idx][31:0] <= writedata;
                        end
                    end
                    
                    // State bounds (0x8000-0x8FFF)
                    4'h8: begin
                        int state_idx = addr[11:8];    // Which state variable (0 to STATE_DIM-1)
                        int is_max = addr[7];          // 0 for min bound, 1 for max bound
                        int is_upper = addr[6];        // 0 for lower 32 bits, 1 for upper 32 bits
                        
                        if (state_idx < STATE_DIM) begin
                            if (is_max) begin
                                // Set max bound
                                if (is_upper) begin
                                    x_max[state_idx][63:32] <= writedata;
                                end else begin
                                    x_max[state_idx][31:0] <= writedata;
                                end
                            end else begin
                                // Set min bound
                                if (is_upper) begin
                                    x_min[state_idx][63:32] <= writedata;
                                end else begin
                                    x_min[state_idx][31:0] <= writedata;
                                end
                            end
                        end
                    end
                    
                    // Input bounds (0x9000-0x9FFF)
                    4'h9: begin
                        int input_idx = addr[11:8];    // Which input variable (0 to INPUT_DIM-1)
                        int is_max = addr[7];          // 0 for min bound, 1 for max bound
                        int is_upper = addr[6];        // 0 for lower 32 bits, 1 for upper 32 bits
                        
                        if (input_idx < INPUT_DIM) begin
                            if (is_max) begin
                                // Set max bound
                                if (is_upper) begin
                                    u_max[input_idx][63:32] <= writedata;
                                end else begin
                                    u_max[input_idx][31:0] <= writedata;
                                end
                            end else begin
                                // Set min bound
                                if (is_upper) begin
                                    u_min[input_idx][63:32] <= writedata;
                                end else begin
                                    u_min[input_idx][31:0] <= writedata;
                                end
                            end
                        end
                    end
                endcase
            end
        end
    end

endmodule
