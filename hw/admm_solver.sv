module admm_solver #(
    parameter STATE_DIM     = 12,        // Dimension of state vector (nx)
    parameter INPUT_DIM     = 4,         // Dimension of input vector (nu)
    parameter HORIZON       = 30,        // Maximum MPC horizon length (N)
    parameter MAX_ITER      = 100,       // Maximum ADMM iterations
    parameter EXT_DATA_WIDTH = 32,       // Bus data width
    parameter DATA_WIDTH    = 16,        // Internal calculation width (16-bit fixed point)
    parameter FRAC_BITS     = 8,         // Number of fractional bits for fixed point
    parameter ADDR_WIDTH    = 16,        // Address width for bus interface
    parameter MEM_ADDR_WIDTH = 9         // Internal memory address width
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
        PRIMAL_UPDATE = 3'd2,  // X-Update (Riccati recursion & trajectory rollout)
        SLACK_UPDATE = 3'd3,  // Z-Update (Projection)
        DUAL_UPDATE = 3'd4,  // Y-Update (Dual variables)
        CHECK_CONVERGENCE = 3'd5,
        OUTPUT_RESULTS = 3'd6,
        DONE = 3'd7
    } solver_state_t;
    
    solver_state_t state;
    
    // Control signals
    logic start_solving;
    logic solver_done;
    logic [31:0] current_iter;
    logic [31:0] active_horizon_reg; // INTERNAL REG ONLY
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
    logic primal_update_start, primal_update_done;
    logic slack_update_start, slack_update_done;
    logic dual_update_start, dual_update_done;
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
    // A_ram (already provided)
    soc_system_RAM A_ram (
        .clk(clk),
        .writedata(A_data_in),
        .address(A_wren ? A_wraddress : A_rdaddress),
        .write(A_wren),
        .readdata(A_data_out),
        .reset(rst),
        .chipselect(1'b1),
        .clken(1'b1),
        .freeze(1'b0),
        .reset_req(1'b0),
        .byteenable(2'b11)
    );

    // System matrices - B matrix (STATE_DIM x INPUT_DIM)
    soc_system_RAM B_ram (
        .clk(clk),
        .writedata(B_data_in),
        .address(B_wren ? B_wraddress : B_rdaddress),
        .write(B_wren),
        .readdata(B_data_out),
        .reset(rst),
        .chipselect(1'b1),
        .clken(1'b1),
        .freeze(1'b0),
        .reset_req(1'b0),
        .byteenable(2'b11)
    );

    // System matrices - Q matrix (STATE_DIM x STATE_DIM)
    soc_system_RAM Q_ram (
        .clk(clk),
        .writedata(Q_data_in),
        .address(Q_wren ? Q_wraddress : Q_rdaddress),
        .write(Q_wren),
        .readdata(Q_data_out),
        .reset(rst),
        .chipselect(1'b1),
        .clken(1'b1),
        .freeze(1'b0),
        .reset_req(1'b0),
        .byteenable(2'b11)
    );

    // System matrices - R matrix (INPUT_DIM x INPUT_DIM)
    soc_system_RAM R_ram (
        .clk(clk),
        .writedata(R_data_in),
        .address(R_wren ? R_wraddress : R_rdaddress),
        .write(R_wren),
        .readdata(R_data_out),
        .reset(rst),
        .chipselect(1'b1),
        .clken(1'b1),
        .freeze(1'b0),
        .reset_req(1'b0),
        .byteenable(2'b11)
    );

    // Precomputed cache terms - K matrix (Kinf) (INPUT_DIM x STATE_DIM)
    soc_system_RAM K_ram (
        .clk(clk),
        .writedata(K_data_in),
        .address(K_wren ? K_wraddress : K_rdaddress),
        .write(K_wren),
        .readdata(K_data_out),
        .reset(rst),
        .chipselect(1'b1),
        .clken(1'b1),
        .freeze(1'b0),
        .reset_req(1'b0),
        .byteenable(2'b11)
    );

    // Precomputed cache terms - C1 matrix ((R + B'*P*B)^-1) (INPUT_DIM x INPUT_DIM)
    soc_system_RAM C1_ram (
        .clk(clk),
        .writedata(C1_data_in),
        .address(C1_wren ? C1_wraddress : C1_rdaddress),
        .write(C1_wren),
        .readdata(C1_data_out),
        .reset(rst),
        .chipselect(1'b1),
        .clken(1'b1),
        .freeze(1'b0),
        .reset_req(1'b0),
        .byteenable(2'b11)
    );

    // Precomputed cache terms - C2 matrix ((A - B*K)') (STATE_DIM x STATE_DIM)
    soc_system_RAM C2_ram (
        .clk(clk),
        .writedata(C2_data_in),
        .address(C2_wren ? C2_wraddress : C2_rdaddress),
        .write(C2_wren),
        .readdata(C2_data_out),
        .reset(rst),
        .chipselect(1'b1),
        .clken(1'b1),
        .freeze(1'b0),
        .reset_req(1'b0),
        .byteenable(2'b11)
    );

    // Precomputed cache terms - P matrix (Pinf) (STATE_DIM x STATE_DIM)
    soc_system_RAM P_ram (
        .clk(clk),
        .writedata(P_data_in),
        .address(P_wren ? P_wraddress : P_rdaddress),
        .write(P_wren),
        .readdata(P_data_out),
        .reset(rst),
        .chipselect(1'b1),
        .clken(1'b1),
        .freeze(1'b0),
        .reset_req(1'b0),
        .byteenable(2'b11)
    );

    // Trajectories - x trajectory (STATE_DIM x HORIZON)
    soc_system_RAM x_ram (
        .clk(clk),
        .writedata(x_data_in),
        .address(x_wren ? x_wraddress : x_rdaddress),
        .write(x_wren),
        .readdata(x_data_out),
        .reset(rst),
        .chipselect(1'b1),
        .clken(1'b1),
        .freeze(1'b0),
        .reset_req(1'b0),
        .byteenable(2'b11)
    );

    // Trajectories - u trajectory (INPUT_DIM x (HORIZON-1))
    soc_system_RAM u_ram (
        .clk(clk),
        .writedata(u_data_in),
        .address(u_wren ? u_wraddress : u_rdaddress),
        .write(u_wren),
        .readdata(u_data_out),
        .reset(rst),
        .chipselect(1'b1),
        .clken(1'b1),
        .freeze(1'b0),
        .reset_req(1'b0),
        .byteenable(2'b11)
    );

    // Trajectories - z trajectory (INPUT_DIM x (HORIZON-1))
    soc_system_RAM z_ram (
        .clk(clk),
        .writedata(z_data_in),
        .address(z_wren ? z_wraddress : z_rdaddress),
        .write(z_wren),
        .readdata(z_data_out),
        .reset(rst),
        .chipselect(1'b1),
        .clken(1'b1),
        .freeze(1'b0),
        .reset_req(1'b0),
        .byteenable(2'b11)
    );

    // Trajectories - v trajectory (STATE_DIM x HORIZON)
    soc_system_RAM v_ram (
        .clk(clk),
        .writedata(v_data_in),
        .address(v_wren ? v_wraddress : v_rdaddress),
        .write(v_wren),
        .readdata(v_data_out),
        .reset(rst),
        .chipselect(1'b1),
        .clken(1'b1),
        .freeze(1'b0),
        .reset_req(1'b0),
        .byteenable(2'b11)
    );

    // Previous values for residuals - z_prev (INPUT_DIM x (HORIZON-1))
    soc_system_RAM z_prev_ram (
        .clk(clk),
        .writedata(z_prev_data_in),
        .address(z_prev_wren ? z_prev_wraddress : z_prev_rdaddress),
        .write(z_prev_wren),
        .readdata(z_prev_data_out),
        .reset(rst),
        .chipselect(1'b1),
        .clken(1'b1),
        .freeze(1'b0),
        .reset_req(1'b0),
        .byteenable(2'b11)
    );

    // Previous values for residuals - v_prev (STATE_DIM x HORIZON)
    soc_system_RAM v_prev_ram (
        .clk(clk),
        .writedata(v_prev_data_in),
        .address(v_prev_wren ? v_prev_wraddress : v_prev_rdaddress),
        .write(v_prev_wren),
        .readdata(v_prev_data_out),
        .reset(rst),
        .chipselect(1'b1),
        .clken(1'b1),
        .freeze(1'b0),
        .reset_req(1'b0),
        .byteenable(2'b11)
    );

    // Dual variables - y (INPUT_DIM x (HORIZON-1))
    soc_system_RAM y_ram (
        .clk(clk),
        .writedata(y_data_in),
        .address(y_wren ? y_wraddress : y_rdaddress),
        .write(y_wren),
        .readdata(y_data_out),
        .reset(rst),
        .chipselect(1'b1),
        .clken(1'b1),
        .freeze(1'b0),
        .reset_req(1'b0),
        .byteenable(2'b11)
    );

    // Dual variables - g (STATE_DIM x HORIZON)
    soc_system_RAM g_ram (
        .clk(clk),
        .writedata(g_data_in),
        .address(g_wren ? g_wraddress : g_rdaddress),
        .write(g_wren),
        .readdata(g_data_out),
        .reset(rst),
        .chipselect(1'b1),
        .clken(1'b1),
        .freeze(1'b0),
        .reset_req(1'b0),
        .byteenable(2'b11)
    );

    // Linear cost terms - q (STATE_DIM x HORIZON)
    soc_system_RAM q_ram (
        .clk(clk),
        .writedata(q_data_in),
        .address(q_wren ? q_wraddress : q_rdaddress),
        .write(q_wren),
        .readdata(q_data_out),
        .reset(rst),
        .chipselect(1'b1),
        .clken(1'b1),
        .freeze(1'b0),
        .reset_req(1'b0),
        .byteenable(2'b11)
    );

    // Linear cost terms - r (INPUT_DIM x (HORIZON-1))
    soc_system_RAM r_ram (
        .clk(clk),
        .writedata(r_data_in),
        .address(r_wren ? r_wraddress : r_rdaddress),
        .write(r_wren),
        .readdata(r_data_out),
        .reset(rst),
        .chipselect(1'b1),
        .clken(1'b1),
        .freeze(1'b0),
        .reset_req(1'b0),
        .byteenable(2'b11)
    );

    // Linear cost terms - p (STATE_DIM x HORIZON)
    soc_system_RAM p_ram (
        .clk(clk),
        .writedata(p_data_in),
        .address(p_wren ? p_wraddress : p_rdaddress),
        .write(p_wren),
        .readdata(p_data_out),
        .reset(rst),
        .chipselect(1'b1),
        .clken(1'b1),
        .freeze(1'b0),
        .reset_req(1'b0),
        .byteenable(2'b11)
    );

    // Linear cost terms - d (INPUT_DIM x (HORIZON-1))
    soc_system_RAM d_ram (
        .clk(clk),
        .writedata(d_data_in),
        .address(d_wren ? d_wraddress : d_rdaddress),
        .write(d_wren),
        .readdata(d_data_out),
        .reset(rst),
        .chipselect(1'b1),
        .clken(1'b1),
        .freeze(1'b0),
        .reset_req(1'b0),
        .byteenable(2'b11)
    );

    
    // Instantiate solver stage 1 - X-Update (Riccati recursion & trajectory rollout)
    primal_update #(
        .STATE_DIM(STATE_DIM),
        .INPUT_DIM(INPUT_DIM),
        .HORIZON(HORIZON),
        .DATA_WIDTH(DATA_WIDTH),
        .ADDR_WIDTH(MEM_ADDR_WIDTH)
    ) primal_update_inst (
        .clk(clk),
        .rst(rst),
        .start(primal_update_start),
        
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
        .active_horizon(active_horizon_reg),
        
        .done(primal_update_done)
    );
    
    // Instantiate solver stage 2 - Z-Update (Projection)
    slack_update #(
        .STATE_DIM(STATE_DIM),
        .INPUT_DIM(INPUT_DIM),
        .HORIZON(HORIZON),
        .DATA_WIDTH(DATA_WIDTH),
        .ADDR_WIDTH(MEM_ADDR_WIDTH)
    ) slack_update_inst (
        .clk(clk),
        .rst(rst),
        .start(slack_update_start),
        
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
        .active_horizon(active_horizon_reg),
        
        .done(slack_update_done)
    );
    
    // Instantiate solver stage 3 - Y-Update (Dual variables)
    dual_update #(
        .STATE_DIM(STATE_DIM),
        .INPUT_DIM(INPUT_DIM),
        .HORIZON(HORIZON),
        .DATA_WIDTH(DATA_WIDTH),
        .ADDR_WIDTH(MEM_ADDR_WIDTH)
    ) dual_update_inst (
        .clk(clk),
        .rst(rst),
        .start(dual_update_start),
        
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
        .active_horizon(active_horizon_reg),
        
        .done(dual_update_done)
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
        
        // Residual inputs from dual_update
        .pri_res_u(pri_res_u),
        .pri_res_x(pri_res_x),
        
        .z_rdaddress(z_rdaddress),
        .z_data_out(z_data_out),
        .z_prev_rdaddress(z_prev_rdaddress),
        .z_prev_data_out(z_prev_data_out),
        .pri_tol(pri_tol),
        .dual_tol(dual_tol),
        .active_horizon(active_horizon_reg),
        
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
            active_horizon_reg <= HORIZON;
            solver_done <= 0;
            
            // Initialize control signals
            primal_update_start <= 0;
            slack_update_start <= 0;
            dual_update_start <= 0;
            residual_calc_start <= 0;
            
            // Initialize tolerances
            pri_tol <= 16'h0080; // 0.001 in 16-bit fixed point
            dual_tol <= 16'h0080; 
            
            rho <= 16'h1000; // 1.0
            
        end else begin
            // Default values for control signals
            primal_update_start <= 0;
            slack_update_start <= 0;
            dual_update_start <= 0;
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
                    state <= PRIMAL_UPDATE;
                end
                
                PRIMAL_UPDATE: begin
                    // X-Update stage (Riccati recursion and forward rollout)
                    if (!primal_update_start && !primal_update_done) begin
                        primal_update_start <= 1;
                    end else if (primal_update_done) begin
                        state <= SLACK_UPDATE;
                    end
                end
                
                SLACK_UPDATE: begin
                    // Z-Update stage (Projection)
                    if (!slack_update_start && !slack_update_done) begin
                        slack_update_start <= 1;
                    end else if (slack_update_done) begin
                        state <= DUAL_UPDATE;
                    end
                end
                
                DUAL_UPDATE: begin
                    // Y-Update stage (Dual variables)
                    if (!dual_update_start && !dual_update_done) begin
                        dual_update_start <= 1;
                    end else if (dual_update_done) begin
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
                            state <= PRIMAL_UPDATE; // Start next iteration
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
    
    // Instantiate memory interface module
    memory_interface #(
        .STATE_DIM(STATE_DIM),
        .INPUT_DIM(INPUT_DIM),
        .HORIZON(HORIZON),
        .EXT_DATA_WIDTH(EXT_DATA_WIDTH),
        .DATA_WIDTH(DATA_WIDTH),
        .FRAC_BITS(FRAC_BITS),
        .ADDR_WIDTH(ADDR_WIDTH),
        .MEM_ADDR_WIDTH(MEM_ADDR_WIDTH)
    ) mem_if (
        // Clock and reset
        .clk(clk),
        .rst(rst),
        
        // Avalon memory-mapped slave interface
        .writedata(writedata),
        .read(read),
        .write(write),
        .addr(addr),
        .chipselect(chipselect),
        .readdata(readdata),
        
        // Control signals
        .solver_done(solver_done),
        .current_iter(current_iter),
        .active_horizon(active_horizon_reg),
        .converged(converged),
        .start_solving(start_solving),
        
        // Residuals
        .pri_res_u(pri_res_u),
        .pri_res_x(pri_res_x),
        .dual_res(dual_res),
        .pri_tol(pri_tol),
        .dual_tol(dual_tol),
        
        // ADMM parameter
        .rho(rho),
        
        // Bounds
        .u_min(u_min),
        .u_max(u_max),
        .x_min(x_min),
        .x_max(x_max),
        
        // Initial state
        .x_init(x_init),
        
        // Reference trajectories
        .x_ref(x_ref),
        .u_ref(u_ref),
        
        // Memory signals for system matrices
        .A_rdaddress(A_rdaddress),
        .A_wraddress(A_wraddress),
        .A_data_in(A_data_in),
        .A_data_out(A_data_out),
        .A_wren(A_wren),
        
        .B_rdaddress(B_rdaddress),
        .B_wraddress(B_wraddress),
        .B_data_in(B_data_in),
        .B_data_out(B_data_out),
        .B_wren(B_wren),
        
        .Q_rdaddress(Q_rdaddress),
        .Q_wraddress(Q_wraddress),
        .Q_data_in(Q_data_in),
        .Q_data_out(Q_data_out),
        .Q_wren(Q_wren),
        
        .R_rdaddress(R_rdaddress),
        .R_wraddress(R_wraddress),
        .R_data_in(R_data_in),
        .R_data_out(R_data_out),
        .R_wren(R_wren),
        
        // Memory signals for trajectories
        .x_rdaddress(x_rdaddress),
        .x_data_out(x_data_out),
        
        .u_rdaddress(u_rdaddress),
        .u_data_out(u_data_out)
    );

endmodule
