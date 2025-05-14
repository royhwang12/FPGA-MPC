

## System Overview

Hardware-accelerated Model Predictive Control (MPC) solver using the Alternating Direction Method of Multipliers (ADMM) algorithm. Uses a 16-bit fixed-point representation with 8 fractional bits.

## System Architecture

The system is organized into these modules:

1. **admm_solver**: Top-level module coordinates ADMM algorithm execution
2. **memory_interface**: Handles interface with Avalon bus
3. **primal_update**: X-update step (Riccati recursion and trajectory rollout)
4. **slack_update**: Z-update step (projection onto constraint sets)
5. **dual_update**: Y-update step (dual variable updates)
6. **residual_calculator**: Computes residuals and checks convergence criteria

## Module Descriptions

### admm_solver

**PARAMETERS**:
- STATE_DIM: 12
- INPUT_DIM: 4
- HORIZON: 30
- MAX_ITER: 100
- DATA_WIDTH: 16
- FRAC_BITS: 8

**State Machine States**:
- IDLE: Waiting for start signal
- INIT: Initialization phase
- PRIMAL_UPDATE: X-update step (Riccati recursion & trajectory rollout)
- SLACK_UPDATE: Z-update step (projection onto constraints)
- DUAL_UPDATE: Y-update step (dual variable updates)
- CHECK_CONVERGENCE: Check if algorithm has converged
- OUTPUT_RESULTS: Prepare results for output
- DONE: Algorithm completed

**Memory Organization**:
- System matrices (A, B, Q, R)
- Precomputed terms (K, C1, C2, P)
- Trajectories (x, u, z, v)
- Dual variables (y, g)
- Linear cost terms (q, r, p, d)

### memory_interface

**Interface**:
- Avalon memory-mapped slave interface (32-bit data width)
- Control signals to/from solver
- Memory interfaces for matrices and trajectories

**Memory Map**:
- 0x0000-0x0FFF: Status and configuration registers
  - 0x000: Status register (solver_done)
  - 0x004: Current iteration count
  - 0x008: Active horizon length
  - 0x00C: Convergence flag
  - 0x010: Primal residual (inputs)
  - 0x018: Primal residual (states)
  - 0x020: Dual residual
- 0x1000-0x1FFF: System matrix A (STATE_DIM x STATE_DIM)
- 0x2000-0x2FFF: System matrix B (STATE_DIM x INPUT_DIM)
- 0x3000-0x3FFF: Cost matrix Q (STATE_DIM x STATE_DIM)
- 0x4000-0x4FFF: Cost matrix R (INPUT_DIM x INPUT_DIM)
- 0x5000-0x5FFF: Initial state x_init (STATE_DIM)
- 0x6000-0x6FFF: State reference trajectory x_ref (STATE_DIM x HORIZON)
- 0x7000-0x7FFF: Input reference trajectory u_ref (INPUT_DIM x (HORIZON-1))
- 0x8000-0x8FFF: State bounds (x_min, x_max)
- 0x9000-0x9FFF: Input bounds (u_min, u_max)

### primal_update
1. Backward pass: Riccati recursion to compute feedback gains
2. Forward pass: Trajectory rollout using the computed gains

**State Machine States**:
- IDLE: Waiting for start signal
- INIT: Initialize variables
- BACKWARD_PASS: Backward Riccati recursion
- FORWARD_PASS: Forward trajectory rollout
- DONE_STATE: Done

### slack_update

1. Save previous values for residual calculation
2. Project input variables onto input constraints
3. Project state variables onto state constraints

**State Machine States**:
- IDLE: Waiting for start signal
- SAVE_PREV: Save previous values for residual calculation
- PROJECT_Z: Project inputs onto bounds (z-update)
- PROJECT_V: Project states onto bounds (v-update)
- DONE_STATE: Done

### dual_update

1. Update input dual variables (y)
2. Update state dual variables (g)
3. Update linear cost terms

**State Machine States**:
- IDLE: Waiting for start signal
- UPDATE_Y: Update input dual variables
- UPDATE_G: Update state dual variables
- UPDATE_LINEAR_COST: Update linear cost terms
- DONE_STATE: Done

### residual_calculator

1. Calculate dual residual based on z and z_prev
2. Check if primal and dual residuals are below tolerances

**Key Operations**:
1. Calculate dual residual based on z and z_prev
2. Check if primal and dual residuals are below tolerances

**State Machine States**:
- IDLE: Waiting for start signal
- CALC_DUAL_RESIDUAL: Calculate dual residual
- CHECK_CONVERGENCE: Check convergence criteria
- DONE_STATE: Done

## Memory Configuration

### RAM Modules

The system uses M10K memory modules:
- Data width: 16 bits
- Address width: 9 bits (512 memory locations)
- Total memory per RAM: 512 x 16 bits = 8192 bits

### Memory Sizes

- System matrices:
  - A: STATE_DIM x STATE_DIM = 12 x 12 = 144 elements
  - B: STATE_DIM x INPUT_DIM = 12 x 4 = 48 elements
  - Q: STATE_DIM x STATE_DIM = 12 x 12 = 144 elements
  - R: INPUT_DIM x INPUT_DIM = 4 x 4 = 16 elements

- Trajectories:
  - x: STATE_DIM x HORIZON = 12 x 30 = 360 elements
  - u: INPUT_DIM x (HORIZON-1) = 4 x 29 = 116 elements
  - z: INPUT_DIM x (HORIZON-1) = 4 x 29 = 116 elements
  - v: STATE_DIM x HORIZON = 12 x 30 = 360 elements

- Dual variables:
  - y: INPUT_DIM x (HORIZON-1) = 4 x 29 = 116 elements
  - g: STATE_DIM x HORIZON = 12 x 30 = 360 elements

## Fixed-Point Representation

The system uses 16-bit fixed-point representation with 8 fractional bits:
- Range: -128.0 to +127.996 (8 integer + sign)
- Resolution: 2^-8 = 0.00390625 (8 fractional)

Constants:
- 1.0 = 16'h0100
- 0.001 = 16'h0019

## Avalon Bus Interface

The system interfaces with a host processor through a memory-mapped slave interface:
- Data width: 32 bits
- Address width: 16 bits
- Read/write operations for control/configuration and results

## Execution Flow

1. Host processor configures the solver (if needed) and starts:
   - Sets system matrices (A, B, Q, R)
   - Sets initial state (x_init)
   - Sets reference trajectories (x_ref, u_ref)
   - Sets constraints (u_min, u_max, x_min, x_max)
   - Sets ADMM parameters (rho, tolerances)

2. ADMM solver executes:
   - Primal update (X-update)
   - Slack update (Z-update)
   - Dual update (Y-update)
   - Convergence check

3. Host processor reads results:
   - Optimized state trajectory (x)
   - Optimized input trajectory (u)
   - Convergence status and residuals
