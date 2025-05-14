#!/usr/bin/env python3
# hardware_mpc_client.py
import socket
import struct
import time
import numpy as np
import argparse

class HardwareMPCClient:
    def __init__(self, server_host='127.0.0.1', server_port=12345, client_port=12346):
        """
        Initialize the Hardware MPC client
        
        Args:
            server_host: IP address of the simulator server
            server_port: Port of the simulator server
            client_port: Local port to bind for receiving responses
        """
        # Network setup
        self.server_address = (server_host, server_port)
        
        # Create UDP socket
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind(('0.0.0.0', client_port))  # Bind to receive responses
        
        print(f"HardwareMPC client initialized, connecting to {server_host}:{server_port}")
        print(f"Listening for responses on port {client_port}")
        
    def receive_state(self, timeout=1.0):
        """
        Receive current state from the simulator
        
        Returns:
            numpy array with 13 elements [position(3), quaternion(4), velocity(3), angular_velocity(3)]
            or None if no data received
        """
        self.socket.settimeout(timeout)
        try:
            # Format: 12 floats
            fmt = '!12f'
            expected_size = struct.calcsize(fmt)
            
            data, addr = self.socket.recvfrom(expected_size)
            
            if len(data) == expected_size:
                # Unpack state values
                state_tuple = struct.unpack(fmt, data)
                
                # Convert to numpy array
                state = np.array(state_tuple)
                return state
            else:
                print(f"Warning: Received packet of unexpected size {len(data)}")
                return None
                
        except socket.timeout:
            print("Timeout waiting for state from simulator")
            return None
        except Exception as e:
            print(f"Error receiving state: {e}")
            return None
    
    def send_control(self, control):
        """
        Send control commands to the simulator
        
        Args:
            control: numpy array with 4 elements (motor commands)
        """
        if len(control) != 4:
            raise ValueError("Control must have exactly 4 elements")
            
        # Pack the 4 control values into a binary message
        fmt = '!4f'  # Network byte order (big-endian)
        binary_data = struct.pack(fmt, *control)
        
        # Send to simulator
        self.socket.sendto(binary_data, self.server_address)
    
    def close(self):
        """Close the socket"""
        self.socket.close()
        
def example_hover_controller(state):
    """
    Example controller that just returns hover thrust
    
    In a real implementation, this would call your hardware MPC solver
    """
    # Create a simple hover control - in real implementation, 
    # this would interface with your hardware MPC
    # Assuming the quadrotor parameters from QuadrotorDynamics
    mass = 0.035
    g = 9.81
    scale = 65535
    kt = 2.245365e-6 * scale
    
    # Calculate hover thrust (same as in QuadrotorDynamics)
    hover_thrust = (mass * g / kt / 4.0) * np.ones(4)
    
    # You could add simple PD control here as an example
    # For now, just return hover thrust
    return hover_thrust

def main():
    """Main function to run the hardware MPC client"""
    parser = argparse.ArgumentParser(description="Hardware MPC Client")
    parser.add_argument("--server", type=str, default="127.0.0.1", 
                        help="Simulator server IP address")
    parser.add_argument("--server-port", type=int, default=12345,
                        help="Simulator server port")
    parser.add_argument("--client-port", type=int, default=12346, 
                        help="Local port to bind for receiving responses")
    args = parser.parse_args()
    
    client = HardwareMPCClient(
        server_host=args.server,
        server_port=args.server_port,
        client_port=args.client_port
    )
    
    try:
        while True:
            # Receive current state from simulator
            state = client.receive_state()
            
            if state is not None:
                # Print some debug info
                position = state[0:3]
                quaternion = state[3:7]
                velocity = state[7:10]
                angular_velocity = state[10:13]
                
                print(f"Position: {position}")
                print(f"Quaternion: {quaternion}")
                print(f"Velocity: {velocity}")
                print(f"Angular Velocity: {angular_velocity}")
                
                # Calculate control input using the example controller
                # In a real implementation, this would interface with your hardware MPC
                u = example_hover_controller(state)
                print(f"Sending control: {u}")
                
                # Send control back to simulator
                client.send_control(u)
            
            # Short sleep to prevent tight loop
            time.sleep(0.01)
            
    except KeyboardInterrupt:
        print("\nClient stopped.")
    finally:
        client.close()

if __name__ == "__main__":
    main()
