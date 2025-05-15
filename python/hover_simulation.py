# utils/simulation.py
import sys
from pathlib import Path
sys.path.append(str(Path(__file__).parent.parent))

import numpy as np
from src.quadrotor import QuadrotorDynamics
import autograd.numpy as np
from autograd.numpy.linalg import norm

# Global reference states
rg = np.array([0.0, 0, 0.0])
qg = np.array([1.0, 0, 0, 0])
vg = np.zeros(3)
omgg = np.zeros(3)
xg = np.hstack([rg, qg, vg, omgg])

# Get hover thrust from QuadrotorDynamics parameters
quad = QuadrotorDynamics()
uhover = quad.hover_thrust
Nx = quad.nx

def delta_x_quat(x_curr, x_ref=None):
    """Compute state error for hover"""
    # Always use hover reference for hover case
    pos_ref = rg
    vel_ref = vg
    omg_ref = omgg
    q_ref = qg
    
    # Current state - normalize quaternion using autograd.numpy
    q = x_curr[3:7]/norm(x_curr[3:7])  # Use autograd norm instead of np.linalg.norm
    phi = QuadrotorDynamics.qtorp(QuadrotorDynamics.L(q_ref).T @ q)
    
    delta_x = np.hstack([
        x_curr[0:3]-pos_ref,     # Position error
        phi,                      # Attitude error
        x_curr[7:10]-vel_ref,    # Velocity error
        x_curr[10:13]-omg_ref    # Angular velocity error
    ])
    return delta_x

def tinympc_controller(x_curr, x_nom, u_nom, mpc):
    """MPC controller for hover"""
    # Compute error state
    delta_x = delta_x_quat(x_curr)
    
    # Initialize MPC problem
    x_init = np.copy(mpc.x_prev)
    x_init[:,0] = delta_x
    u_init = np.copy(mpc.u_prev)
    
    # Solve MPC
    x_out, u_out, status, k = mpc.solve_admm(x_init, u_init)
    
    return uhover + u_out[:,0], k, status, x_out

def generate_wind(t):
    """Generate time-varying wind disturbance"""
    wind_mean = np.array([0.5, 0.1, 0.03])
    wind_freq = 0.5
    wind_amp = 0.05
    wind = wind_mean + wind_amp * np.array([
        np.sin(wind_freq * t),
        np.cos(wind_freq * t),
        0.2 * np.sin(2 * wind_freq * t)
    ])
    return wind

def simulate_with_controller(x0, x_nom, u_nom, mpc, quad, NSIM=100, use_wind=False):
    """Simulate system with MPC controller"""
    x_all = []
    u_all = []
    x_curr = np.copy(x0)
    iterations = []
    rho_history = [] if mpc.rho_adapter is not None else None
    current_time = 0.0
    dt = quad.dt
    
    # Initialize metrics dictionary with 2D violations like trajectory
    metrics = {
        'trajectory_costs': [],
        'control_efforts': [],
        'solve_costs': [],  # Will store [state_cost, input_cost, total_cost]
        'violations': [],   # Will store [u_violation, x_violation]
        'iterations': []
    }

    for i in range(NSIM):
        # Run MPC step
        u, k, status, x_traj = tinympc_controller(x_curr, x_nom, u_nom, mpc)
        
        # Compute state error with normalized quaternion
        state_error = delta_x_quat(x_curr)
        
        # Add numerical safeguards for cost computation
        try:
            state_cost = float(state_error.T @ mpc.cache['Q'] @ state_error)
            input_cost = float(u.T @ mpc.cache['R'] @ u)
            total_cost = state_cost + input_cost
        except:
            state_cost = float('inf')
            input_cost = float('inf')
            total_cost = float('inf')
            
        metrics['solve_costs'].append([state_cost, input_cost, total_cost])

        # Compute violations with normalized quaternion
        u_violation = np.maximum(0, np.maximum(
            np.abs(u) - mpc.umax, 
            mpc.umin - np.abs(u)
        ))
        
        # Use normalized quaternion for reduced state
        q_normalized = x_curr[3:7]/norm(x_curr[3:7])
        x_reduced = np.hstack([
            x_curr[0:3],
            quad.qtorp(q_normalized),
            x_curr[7:10],
            x_curr[10:13]
        ])
        x_violation = np.maximum(0, np.maximum(
            np.abs(x_reduced) - mpc.xmax, 
            mpc.xmin - np.abs(x_reduced)
        ))
        
        metrics['violations'].append([np.sum(u_violation), np.sum(x_violation)])
        metrics['iterations'].append(k)

        # Simulate system
        if use_wind:
            wind_vec = generate_wind(current_time)
            x_curr = quad.dynamics_rk4(x_curr, u, dt=dt, wind_vec=wind_vec)
        else:
            x_curr = quad.dynamics_rk4(x_curr, u)
        
        current_time += dt
        
        # Store results with normalized quaternion for metrics
        x_all.append(x_curr)
        u_all.append(u)
        iterations.append(k)
        
        # Compute metrics with normalized quaternion
        q_normalized = x_curr[3:7]/norm(x_curr[3:7])
        pos_error = norm(x_curr[0:3] - xg[0:3])
        att_error = norm(q_normalized - xg[3:7])
        metrics['trajectory_costs'].append(pos_error + att_error)
        metrics['control_efforts'].append(np.sum(np.abs(u[1:])))
        
        if mpc.rho_adapter is not None:
            rho_history.append(mpc.cache['rho'])

    return np.array(x_all), np.array(u_all), iterations, rho_history, metrics