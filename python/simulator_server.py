#!/usr/bin/env python3
# simulator_server.py
import socket
import struct
import time
import numpy as np
import argparse
from quadrotor import QuadrotorDynamics

class QuadrotorSimulatorServer:
    def __init__(self, host='0.0.0.0', port=12345, client_host='127.0.0.1', client_port=12346, simulator_hz=50.0):
        self.host = host
        self.port = port
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind((self.host, self.port))
        
        # Set client address if provided
        if client_host and client_port:
            self.client_address = (client_host, client_port)
            print(f"Client address set to {client_host}:{client_port}")
        else:
            self.client_address = None
        
        # Simulator setup
        self.quad = QuadrotorDynamics()
        self.simulator_hz = simulator_hz
        self.dt = 1.0 / simulator_hz
        
        # Initialize state
        # Default initial state: [position, quaternion, velocity, angular velocity]
        self.x = 0.00001 * np.ones(13)
        self.x[0] = 1.0  # Unit quaternion w component
        
        # Get hover thrust from QuadrotorDynamics parameters
        self.uhover = self.quad.hover_thrust
        
        print(f"Simulator server initialized on {host}:{port}")
        print(f"Running at {simulator_hz} Hz (dt = {self.dt})")
        
    def send_state(self):
        """Send current simulator state to client"""
        if self.client_address is None:
            return
            
        # Pack the 12 state values into a binary message
        # Format: 12 floats (position[3], quaternion[4], velocity[3], angular velocity[3])
        fmt = '!12f'  # Network byte order (big-endian)
        
        # Normalize quaternion for safety
        q_norm = self.x[4:8] / np.linalg.norm(self.x[4:8])
        
        # Create state array to send: [position, quaternion, velocity, angular velocity]
        state_to_send = np.concatenate([
            self.x[0:3],    # position
            q_norm,         # normalized quaternion
            self.x[7:10],   # velocity
            self.x[10:12]   # angular velocity
        ])
        
        # Pack and send
        binary_data = struct.pack(fmt, *state_to_send)
        self.socket.sendto(binary_data, self.client_address)
        
    def receive_control(self, timeout=0.01):
        """
        Receive control input from client
        Returns control input if received, otherwise None
        """
        self.socket.settimeout(timeout)
        try:
            # Format: 4 floats (motor commands)
            fmt = '!4f'
            expected_size = struct.calcsize(fmt)
            
            data, addr = self.socket.recvfrom(expected_size)
            self.client_address = addr  # Store client address for replies
            
            if len(data) == expected_size:
                # Unpack control values
                controls = struct.unpack(fmt, data)
                return np.array(controls)
            else:
                print(f"Warning: Received packet of unexpected size {len(data)}")
                return None
                
        except socket.timeout:
            return None
        except Exception as e:
            print(f"Error receiving control: {e}")
            return None
            
    def step_simulation(self, u):
        """Step the simulation forward using the provided control input"""
        # Apply the dynamics using RK4 integration
        self.x = self.quad.dynamics_rk4(self.x, u, dt=self.dt)
        
    def run(self):
        """Main simulation loop"""
        print("Starting simulator server. Press Ctrl+C to stop.")
        
        # Send an initial state to the client if we have their address
        if self.client_address is not None:
            print(f"Sending initial state to {self.client_address}")
            self.send_state()
        
        last_step_time = time.time()
        
        try:
            while True:
                # Calculate time since last step
                current_time = time.time()
                elapsed = current_time - last_step_time
                
                # Wait for next step if needed
                if elapsed < self.dt:
                    time.sleep(self.dt - elapsed)
                
                # Try to receive control input from hardware client
                u = self.receive_control()
                
                # If no control received, use hover thrust
                if u is None:
                    u = self.uhover
                
                # Step simulation
                self.step_simulation(u)
                
                # Send updated state back to client
                self.send_state()
                
                # Update last step time
                last_step_time = time.time()
                
        except KeyboardInterrupt:
            print("\nSimulator server stopped.")
        finally:
            self.socket.close()
            
if __name__ == "__main__":
    # Parse command line arguments
    parser = argparse.ArgumentParser(description="Quadrotor Simulator Server")
    parser.add_argument("--host", type=str, default="0.0.0.0",
                        help="Host to bind the server to")
    parser.add_argument("--port", type=int, default=12345,
                        help="Port to bind the server to")
    parser.add_argument("--client-host", type=str, default="127.0.0.1",
                        help="Client host address")
    parser.add_argument("--client-port", type=int, default=12346,
                        help="Client port")
    parser.add_argument("--hz", type=float, default=50.0,
                        help="Simulation frequency in Hz")
    
    args = parser.parse_args()
    
    server = QuadrotorSimulatorServer(
        host=args.host,
        port=args.port,
        client_host=args.client_host,
        client_port=args.client_port,
        simulator_hz=args.hz
    )
    
    server.run()
