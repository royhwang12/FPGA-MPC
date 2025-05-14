#ifndef _ADMM_SOLVER_H_
#define _ADMM_SOLVER_H_

#include <linux/ioctl.h>
#include <stdint.h>

#define ADMM_SOLVER_IOCTL_MAGIC 'a'

#define ADMM_SOLVER_WRITE _IOW(ADMM_SOLVER_IOCTL_MAGIC, 1, struct admm_solver_arg)
#define ADMM_SOLVER_READ  _IOR(ADMM_SOLVER_IOCTL_MAGIC, 2, struct admm_solver_arg)

struct admm_solver_arg {
    uint32_t horizon;     // Horizon length (e.g., 30)
    uint32_t iter_count;  // Output: number of iterations run
    uint8_t  start;       // Set to 1 to start the solver
    uint8_t  done;        // Output: 1 if solver is done
};

typedef struct admm_solver_arg admm_solver_arg_t;

#endif // _ADMM_SOLVER_H_