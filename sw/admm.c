/*
 * Device driver for the ADMM FPGA Accelerator
 *
 * Adapted from vga_ball.c
 */

#include "admm.h"
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/version.h>

#define DRIVER_NAME "admm"

struct admm_dev {
  struct resource res;
  void __iomem *virtbase; // memory-mapped base address of registers
  admm_config_t config;
  admm_result_t result;
} dev;

// ******************** RUNTIME FUNCTIONS *********************

static void set_initial_state(const int16_t x_init[STATE_DIM],
                              const int16_t u_init[INPUT_DIM]) {
  int i;
  for (i = 0; i < STATE_DIM; i++) {
    iowrite32(x_init[i], X_INIT(dev.virtbase, i));
    dev.config.initial_state.x[i] = x_init[i];
  }

  for (i = 0; i < INPUT_DIM; i++) {
    iowrite32(u_init[i], U_INIT(dev.virtbase, i));
    dev.config.initial_state.u[i] = u_init[i];
  }
}

static void set_reference_trajectory(const int16_t x_ref[STATE_DIM],
                                     const int16_t u_ref[INPUT_DIM]) {
  int i;
  for (i = 0; i < STATE_DIM; i++) {
    iowrite32(x_ref[i], X_REF(dev.virtbase, i));
    dev.config.reference.x[i] = x_ref[i];
  }

  for (i = 0; i < INPUT_DIM; i++) {
    iowrite32(u_ref[i], U_REF(dev.virtbase, i));
    dev.config.reference.u[i] = u_ref[i];
  }
}

static void read_result(admm_result_t *result) {
  // status
  result->iterations = ioread32(CUR_ITER(dev.virtbase));
  result->converged = ioread32(CONVERGED(dev.virtbase));
  result->pri_res_u = ioread32(PRI_RES_U(dev.virtbase));
  result->pri_res_x = ioread32(PRI_RES_X(dev.virtbase));
  result->dual_res = ioread32(DUAL_RES(dev.virtbase));

  // trajectory
  int i, j;
  for (i = 0; i < dev.config.horizon; i++) {
    for (j = 0; j < STATE_DIM; j++) {
      result->trajectory[i].x[j] = ioread32(X_READ(dev.virtbase, i, j));
    }
    for (j = 0; j < INPUT_DIM; j++) {
      result->trajectory[i].u[j] = ioread32(U_READ(dev.virtbase, i, j));
    }
  }

  dev.result = *result;
}

// ******************** SETUP/CONFIG FUNCTIONS *********************

static void set_solver_params(uint32_t horizon, int16_t pri_tol,
                              int16_t dual_tol, int16_t rho) {
  iowrite32(horizon, HORIZON(dev.virtbase));
  iowrite32(pri_tol, PRI_TOL(dev.virtbase));
  iowrite32(dual_tol, DUAL_TOL(dev.virtbase));
  iowrite32(rho, RHO(dev.virtbase));

  // Update the local copy
  dev.config.horizon = horizon;
  dev.config.pri_tol = pri_tol;
  dev.config.dual_tol = dual_tol;
  dev.config.rho = rho;
}

static void set_state_bounds(const int16_t x_min[STATE_DIM],
                             const int16_t x_max[STATE_DIM]) {
  int i;
  for (i = 0; i < STATE_DIM; i++) {
    iowrite32(x_min[i], X_MIN(dev.virtbase, i));
    iowrite32(x_max[i], X_MAX(dev.virtbase, i));
  }
}

static void set_input_bounds(const int16_t u_min[INPUT_DIM],
                             const int16_t u_max[INPUT_DIM]) {
  int i;
  for (i = 0; i < INPUT_DIM; i++) {
    iowrite32(u_min[i], U_MIN(dev.virtbase, i));
    iowrite32(u_max[i], U_MAX(dev.virtbase, i));
  }
}

static void set_system_matrix_A(const int16_t A[STATE_DIM][STATE_DIM]) {
  int i, j;
  for (i = 0; i < STATE_DIM; i++) {
    for (j = 0; j < STATE_DIM; j++) {
      iowrite32(A[i][j], MATRIX_A(dev.virtbase, i, j));
    }
  }
}

static void set_system_matrix_B(const int16_t B[STATE_DIM][INPUT_DIM]) {
  int i, j;
  for (i = 0; i < STATE_DIM; i++) {
    for (j = 0; j < INPUT_DIM; j++) {
      iowrite32(B[i][j], MATRIX_B(dev.virtbase, i, j));
    }
  }
}

static void set_cost_matrix_Q(const int16_t Q[STATE_DIM][STATE_DIM]) {
  int i, j;
  for (i = 0; i < STATE_DIM; i++) {
    for (j = 0; j < STATE_DIM; j++) {
      iowrite32(Q[i][j], MATRIX_Q(dev.virtbase, i, j));
    }
  }
}

static void set_cost_matrix_R(const int16_t R[INPUT_DIM][INPUT_DIM]) {
  int i, j;
  for (i = 0; i < INPUT_DIM; i++) {
    for (j = 0; j < INPUT_DIM; j++) {
      iowrite32(R[i][j], MATRIX_R(dev.virtbase, i, j));
    }
  }
}

static void write_config(admm_config_t *config) {
  set_solver_params(config->horizon, config->pri_tol, config->dual_tol,
                    config->rho);
  set_initial_state(config->initial_state.x, config->initial_state.u);
  set_reference_trajectory(config->reference.x, config->reference.u);

  dev.config = *config;
}

// ******************** IOCTL *********************

// Handle ioctl() calls from userspace
static long admm_ioctl(struct file *f, unsigned int cmd, unsigned long arg) {
  admm_config_arg_t config_arg;
  admm_result_arg_t result_arg;

  switch (cmd) {
  case ADMM_WRITE_CONFIG:
    if (copy_from_user(&config_arg, (admm_config_arg_t *)arg,
                       sizeof(admm_config_arg_t)))
      return -EACCES;
    write_config(&config_arg.config);
    break;

  case ADMM_READ_RESULT:
    read_result(&dev.result);
    result_arg.result = dev.result;
    if (copy_to_user((admm_result_arg_t *)arg, &result_arg,
                     sizeof(admm_result_arg_t)))
      return -EACCES;
    break;

  case ADMM_START:
    iowrite32(1, START_SOLVER(dev.virtbase));
    break;

  default:
    return -EINVAL;
  }

  return 0;
}

/* The operations our device knows how to do */
static const struct file_operations admm_fops = {
    .owner = THIS_MODULE,
    .unlocked_ioctl = admm_ioctl,
};

/* Information about our device for the "misc" framework -- like a char dev */
static struct miscdevice admm_misc_device = {
    .minor = MISC_DYNAMIC_MINOR,
    .name = DRIVER_NAME,
    .fops = &admm_fops,
};

static int __init admm_probe(struct platform_device *pdev) {
  int ret;

  /* Register ourselves as a misc device: creates /dev/admm */
  ret = misc_register(&admm_misc_device);

  /* Get the address of our registers from the device tree */
  ret = of_address_to_resource(pdev->dev.of_node, 0, &dev.res);
  if (ret) {
    ret = -ENOENT;
    goto out_deregister;
  }

  /* Make sure we can use these registers */
  if (request_mem_region(dev.res.start, resource_size(&dev.res), DRIVER_NAME) ==
      NULL) {
    ret = -EBUSY;
    goto out_deregister;
  }

  /* Arrange access to our registers */
  dev.virtbase = of_iomap(pdev->dev.of_node, 0);
  if (dev.virtbase == NULL) {
    ret = -ENOMEM;
    goto out_release_mem_region;
  }

  /* Setup default configuration */
  memset(&dev.config, 0, sizeof(dev.config));

  /* Set default solver parameters */
  dev.config.horizon = 10;
  dev.config.pri_tol = TO_FIXED(0.01);
  dev.config.dual_tol = TO_FIXED(0.01);
  dev.config.rho = TO_FIXED(1.0);

  return 0;

out_release_mem_region:
  release_mem_region(dev.res.start, resource_size(&dev.res));
out_deregister:
  misc_deregister(&admm_misc_device);
  return ret;
}

/* Clean-up code: release resources */
static int admm_remove(struct platform_device *pdev) {
  iounmap(dev.virtbase);
  release_mem_region(dev.res.start, resource_size(&dev.res));
  misc_deregister(&admm_misc_device);
  return 0;
}

/* Which "compatible" string(s) to search for in the Device Tree */
#ifdef CONFIG_OF
static const struct of_device_id admm_of_match[] = {
    {.compatible = "csee4840,admm-1.0"},
    {},
};
MODULE_DEVICE_TABLE(of, admm_of_match);
#endif

/* Information for registering ourselves as a "platform" driver */
static struct platform_driver admm_driver = {
    .driver =
        {
            .name = DRIVER_NAME,
            .owner = THIS_MODULE,
            .of_match_table = of_match_ptr(admm_of_match),
        },
    .remove = __exit_p(admm_remove),
};

/* Called when the module is loaded: set things up */
static int __init admm_init(void) {
  pr_info(DRIVER_NAME ": init\n");
  return platform_driver_probe(&admm_driver, admm_probe);
}

/* Called when the module is unloaded: release resources */
static void __exit admm_exit(void) {
  platform_driver_unregister(&admm_driver);
  pr_info(DRIVER_NAME ": exit\n");
}

module_init(admm_init);
module_exit(admm_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Alex Du");
MODULE_DESCRIPTION("Driver for FPGA-MPC's ADMM Accelerator");
