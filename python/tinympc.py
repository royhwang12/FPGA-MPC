import numpy as np

class TinyMPC:
    def __init__(self, A, B, Q, R, Nsteps, rho=1.0, n_dlqr_steps=500, mode = 'hover'):
        """
        Args:
            A (np.ndarray): System dynamics matrix
            B (np.ndarray): Input matrix
            Q (np.ndarray): State cost matrix
            R (np.ndarray): Input cost matrix
            Nsteps (int): Horizon length
            rho (float): Initial rho value
            n_dlqr_steps (int): Number of steps for DLQR computation
        """
        self.nx, self.nu = A.shape[0], B.shape[1]
        self.N = Nsteps
        self.mode = mode # hover or trajectory following
        
        if self.mode == 'hover':
            self.max_iter = 500
            self.abs_pri_tol = 1e-2
            self.abs_dua_tol = 1e-2
        else:
            self.max_iter = 10
            self.abs_pri_tol = 1e-3
            self.abs_dua_tol = 1e-3
        
        
        # Compute DLQR solution for terminal cost
        P_lqr = self._compute_dlqr(A, B, Q, R, n_dlqr_steps)

        # Initialize cache with computed values
        self.cache = {
            'rho': rho,
            'A': A,
            'B': B,
            'Q': P_lqr,  # Use DLQR solution for terminal cost
            'R': R
        }

        # Initialize state variables
        self.v_prev, self.z_prev = np.zeros((self.nx, self.N)), np.zeros((self.nu, self.N-1))
        self.g_prev, self.y_prev = np.zeros((self.nx, self.N)), np.zeros((self.nu, self.N-1))
        self.q_prev = np.zeros((self.nx, self.N))
        
        # Initialize previous solutions for warm start
        self.x_prev, self.u_prev = np.zeros((self.nx, self.N)), np.zeros((self.nu, self.N-1))

        self.Q, self.R = Q, R

        # Compute cache terms (Kinf, Pinf, C1, C2)
        self._compute_cache_terms()

    def _compute_dlqr(self, A, B, Q, R, n_steps):
        """Compute Discrete-time LQR solution"""
        P = Q
        for _ in range(n_steps):
            #K = np.linalg.inv(R + B.T @ P @ B) @ B.T @ P @ A
            K = np.linalg.solve(
                R + B.T @ P @ B + 1e-8*np.eye(B.shape[1]),  # Add regularization
                B.T @ P @ A
            )
            P = Q + A.T @ P @ (A - B @ K)
        return P
    
    def _compute_cache_terms(self):
        """Compute and cache terms for ADMM"""
        Q_rho, R_rho = self.cache['Q'], self.cache['R']
        R_rho += self.cache['rho'] * np.eye(R_rho.shape[0])
        Q_rho += self.cache['rho'] * np.eye(Q_rho.shape[0])

        A, B = self.cache['A'], self.cache['B']
        Kinf = np.zeros(B.T.shape)
        Pinf = np.copy(self.cache['Q'])

        # Compute infinite horizon solution (Kinf, Pinf)
        for k in range(5000):
            Kinf_prev = np.copy(Kinf)
            Kinf = np.linalg.inv(R_rho + B.T @ Pinf @ B) @ B.T @ Pinf @ A
            Pinf = Q_rho + A.T @ Pinf @ (A - B @ Kinf)
            
            if np.linalg.norm(Kinf - Kinf_prev, 2) < 1e-10:
                break
        
        # Compute Riccati matrices
        AmBKt = (A - B @ Kinf).T
        Quu_inv = np.linalg.inv(R_rho + B.T @ Pinf @ B)

        print(Kinf.shape) # (nu, nx), since u_k_* = -Kinf @ x_k - d_k
        print(Pinf.shape) # (nx, nx)    
        print(Quu_inv.shape) # (nu, nu)
        print(AmBKt.shape) # (nx, nx)

        # Cache computed terms
        self.cache['Kinf'] = Kinf
        self.cache['Pinf'] = Pinf
        self.cache['C1'] = Quu_inv
        self.cache['C2'] = AmBKt

    def set_bounds(self, umax=None, umin=None, xmax=None, xmin=None):
        if (umin is not None) and (umax is not None):
            self.umin = np.array(umin)
            self.umax = np.array(umax)
        if (xmin is not None) and (xmax is not None):
            self.xmin = np.array(xmin)
            self.xmax = np.array(xmax)

    def solve_admm(self, x_init, u_init, x_ref=None, u_ref=None):
        x, u = np.copy(x_init), np.copy(u_init)
        
        # x_ref and u_ref can be passed in for trajectory following, otherwise use zero reference
        x_ref = np.zeros(x.shape) if x_ref is None else x_ref
        u_ref = np.zeros(u.shape) if u_ref is None else u_ref
        
        # Initialize variables from previous solve
        v, z = np.copy(self.v_prev), np.copy(self.z_prev)
        g, y = np.copy(self.g_prev), np.copy(self.y_prev)
        q = np.copy(self.q_prev)

        # Keep track of previous values for residuals
        v_prev, z_prev = np.copy(v), np.copy(z)

        r = np.zeros(u.shape) # control input residual
        p = np.zeros(x.shape) # linear term of cost-to-go (value) function
        d = np.zeros(u.shape) # feedforward term

        for _ in range(self.max_iter):
            # ----- Primal update (to get x' and u') -----
            # d: feedforward term
            # p: linear term of cost-to-go (value) function
            
            # Solve LQR problem with Riccati recursion
            # Riccati matrices (C1, C2) and Kinf are precomputed and cached, so here we only update linear terms
            # C1 = (R + B.T @ Pinf @ B)^-1
            # C2 = (A - B @ Kinf).T
            # Kinf: infinite horizon gain
            for k in range(self.N-2, -1, -1):
                d[:, k] = np.dot(self.cache['C1'], np.dot(self.cache['B'].T, p[:, k + 1]) + r[:, k])
                p[:, k] = q[:, k] + np.dot(self.cache['C2'], p[:, k + 1]) - np.dot(self.cache['Kinf'].T, r[:, k])
                
            # Forward pass to rollout trajectory (u_0, x_1, u_1, x_2, ..., u_N-1, x_N)
            for k in range(self.N - 1):
                # Compute control (u_k) with feedback controller gain (K_inf) and feedforward term (d)
                u[:, k] = -np.dot(self.cache['Kinf'], x[:, k]) - d[:, k]
                # Transition to next state (x_k+1) with control (uk) and linearized dynamics (A, B)
                x[:, k + 1] = np.dot(self.cache['A'], x[:, k]) + np.dot(self.cache['B'], u[:, k])
                
            # ------------------------------------------------------------
            # ----- Slack update (to get z' and v') -----
            
            # Linear projection of updated state and control onto their feasible sets
            # Used to enforce constraints
            # Note: y and g are swapped in the TinyMPC paper
            for k in range(self.N - 1):
                z[:, k] = np.clip(u[:, k] + y[:, k], self.umin, self.umax)
                v[:, k] = np.clip(x[:, k] + g[:, k], self.xmin, self.xmax)
            v[:, self.N-1] = np.clip(x[:, self.N-1] + g[:, self.N-1], self.xmin, self.xmax)
            
            # ------------------------------------------------------------
            # ----- Dual update (to get y' and g') -----
            # Adjust dual variables (lagrange multipliers) to penalize mismatch between x and v, u and z
            # - Gradient ascent on the dual variables
            # y, g: scaled dual variables (lambda_k / rho, mu_k / rho)
            for k in range(self.N - 1):
                y[:, k] += u[:, k] - z[:, k]
                g[:, k] += x[:, k] - v[:, k]
            g[:, self.N-1] += x[:, self.N-1] - v[:, self.N-1]
            
            # ------------------------------------------------------------
            # ----- Linear cost update (to get r', q', p') -----
            # Update vectors used in next iteration's Riccati recursion:
            # q, r: state and control input residuals
            # p: terminal cost residual
            for k in range(self.N - 1):
                r[:, k] = -self.cache['R'] @ u_ref[:, k]
                r[:, k] -= self.cache['rho'] * (z[:, k] - y[:, k])
                
                q[:, k] = -self.cache['Q'] @ x_ref[:, k]
                q[:, k] -= self.cache['rho'] * (v[:, k] - g[:, k])

            p[:,self.N-1] = -np.dot(self.cache['Pinf'], x_ref[:, self.N-1])
            p[:,self.N-1] -= self.cache['rho'] * (v[:, self.N-1] - g[:, self.N-1])
            
            # ------------------------------------------------------------
            # ----- Compute residuals and check convergence -----
            pri_res_input = np.max(np.abs(u - z))
            pri_res_state = np.max(np.abs(x - v))
            dua_res_input = np.max(np.abs(self.cache['rho'] * (z_prev - z)))
            dua_res_state = np.max(np.abs(self.cache['rho'] * (v_prev - v)))

            z_prev = np.copy(z)
            v_prev = np.copy(v)

            # Exit condition (if all residuals are below tolerance)
            if (pri_res_input < self.abs_pri_tol and dua_res_input < self.abs_dua_tol and
                pri_res_state < self.abs_pri_tol and dua_res_state < self.abs_dua_tol):
                break
            
            # ------------------------------------------------------------

        # Save variables for next solve
        self.x_prev, self.u_prev = x, u
        self.v_prev, self.z_prev = v, z
        self.g_prev, self.y_prev = g, y
        self.q_prev = q

        return x, u

    
    