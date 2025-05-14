#ifndef _ADMM_H
#define _ADMM_H

#include <linux/ioctl.h>

#ifdef __KERNEL__
#include <linux/kernel.h>
#include <linux/types.h>
#else
#include <stdint.h>
#endif

// System dimensions
#define STATE_DIM 12
#define INPUT_DIM 4
#define MAX_HORIZON 30

typedef struct {
  int16_t x[STATE_DIM];  
  int16_t u[INPUT_DIM];   
} state_input_t;

typedef struct {
  int16_t x_min[STATE_DIM];  
  int16_t x_max[STATE_DIM]; 
} state_bounds_t;

typedef struct {
  int16_t u_min[INPUT_DIM]; 
  int16_t u_max[INPUT_DIM]; 
} input_bounds_t;

typedef struct {
  int16_t A[STATE_DIM][STATE_DIM];  // State transition matrix
} system_matrix_a_t;
typedef struct {
  int16_t B[STATE_DIM][INPUT_DIM];  // Input matrix
} system_matrix_b_t;

typedef struct {
  int16_t Q[STATE_DIM][STATE_DIM];  // State cost matrix
} cost_matrix_q_t;
typedef struct {
  int16_t R[INPUT_DIM][INPUT_DIM];  // Input cost matrix
} cost_matrix_r_t;

typedef struct {
  uint32_t horizon;   
  int16_t pri_tol;    
  int16_t dual_tol;   
  int16_t rho;        
} solver_params_t;

// Fixed-point configuration
#define DATA_WIDTH 16
#define FRAC_BITS 8

typedef struct {
    int16_t x[STATE_DIM];  // State vector
    int16_t u[INPUT_DIM];   // Input vector
} admm_trajectory_point_t;

// Configuration structure
typedef struct {
    uint32_t horizon;       
    int16_t pri_tol;        
    int16_t dual_tol;       
    int16_t rho;            
    state_input_t initial_state;  
    state_input_t reference;
} admm_config_t;

// Solver results
typedef struct {
    admm_trajectory_point_t trajectory[MAX_HORIZON];  // Optimal trajectory
    uint32_t iterations;    // Number of iterations
    int16_t pri_res_u;      // Primal residual (inputs)
    int16_t pri_res_x;      // Primal residual (states)
    int16_t dual_res;       // Dual residual
    uint8_t converged;      // Converged flag
} admm_result_t;

typedef struct {
    admm_config_t config;
} admm_config_arg_t;

typedef struct {
    admm_result_t result;
} admm_result_arg_t;

// IOCTL commands
#define ADMM_MAGIC 'q'

// IOCTls
#define ADMM_WRITE_CONFIG _IOW(ADMM_MAGIC, 1, admm_config_arg_t)
#define ADMM_READ_RESULT _IOR(ADMM_MAGIC, 2, admm_result_arg_t)
#define ADMM_START       _IO(ADMM_MAGIC, 3)

// ************ OFFSETS ************
#define STATUS_REG_BASE       0x0000
#define START_SOLVER(x)       ((x) + 0x00)
#define SOLVER_DONE(x)        ((x) + 0x04)
#define CUR_ITER(x)           ((x) + 0x08)
#define CONVERGED(x)          ((x) + 0x0C)

#define CONFIG_REG_BASE       0x0100
#define HORIZON(x)            ((x) + 0x00)
#define PRI_TOL(x)            ((x) + 0x04)
#define DUAL_TOL(x)           ((x) + 0x08)
#define RHO(x)                ((x) + 0x0C)

#define RESIDUAL_REG_BASE     0x0200
#define PRI_RES_U(x)          ((x) + 0x00)
#define PRI_RES_X(x)          ((x) + 0x04)
#define DUAL_RES(x)           ((x) + 0x08)

#define TRAJECTORY_BASE       0x1000
#define X_READ(x, i, j)       ((x) + 0x0000 + (i * STATE_DIM + j) * sizeof(int16_t))
#define U_READ(x, i, j)       ((x) + 0x1000 + (i * INPUT_DIM + j) * sizeof(int16_t))
#define X_INIT(x, i)          ((x) + 0x2000 + (i * sizeof(int16_t)))
#define U_INIT(x, i)          ((x) + 0x2100 + (i * sizeof(int16_t)))
#define X_REF(x, i)           ((x) + 0x2200 + (i * sizeof(int16_t)))
#define U_REF(x, i)           ((x) + 0x2300 + (i * sizeof(int16_t)))

#define MATRIX_A(x, i, j)      ((x) + 0x3000 + (i << 8) + (j << 4))
#define MATRIX_B(x, i, j)      ((x) + 0x4000 + (i << 8) + (j << 4))

#define MATRIX_Q(x, i, j)      ((x) + 0x5000 + (i << 8) + (j << 4))
#define MATRIX_R(x, i, j)      ((x) + 0x6000 + (i << 8) + (j << 4))

#define X_MIN(x, i)            ((x) + 0x7000 + (i << 4))
#define X_MAX(x, i)            ((x) + 0x7080 + (i << 4))
#define U_MIN(x, i)            ((x) + 0x8000 + (i << 4))
#define U_MAX(x, i)            ((x) + 0x8080 + (i << 4))

// Fixed-point conversion
#define TO_FIXED(x)   ((int16_t)((x) * (1 << FRAC_BITS)))
#define FROM_FIXED(x) (((float)(x)) / (1 << FRAC_BITS))

#endif // _ADMM_H
