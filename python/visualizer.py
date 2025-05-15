#!/usr/bin/env python3
# visualizer.py
import numpy as np
import meshcat
import meshcat.geometry as g
import meshcat.transformations as tf
import time
import subprocess
import webbrowser
import threading

class QuadrotorVisualizer:
    def __init__(self, auto_open_browser=True):
        # Initialize the visualizer
        self.vis = meshcat.Visualizer()
        
        # Automatically open the visualizer in the browser
        self.url = self.vis.url()
        print(f"Meshcat visualizer running at: {self.url}")
        
        if auto_open_browser:
            # Open browser in a separate thread to not block
            threading.Thread(target=self._open_browser).start()
        
        # Set up the scene immediately
        self.setup_scene()
        
        # Initialize trajectory points
        self.trajectory_points = []
        self.max_trajectory_points = 1000  # Maximum number of points to show
        
        # Add initial quadrotor position
        self.update_quadrotor_state([0, 0, 0], [1, 0, 0, 0])
    
    def _open_browser(self):
        """Open the browser without blocking the main thread"""
        try:
            webbrowser.open(self.url, new=2)  # Open in new tab
            print("Browser window should open automatically")
        except Exception as e:
            print(f"Could not open browser automatically: {e}")
            print(f"Please manually navigate to: {self.url}")
        
    def setup_scene(self):
        """Set up the visualization scene with coordinate frame and quadrotor model"""
        # Add coordinate frame
        self.vis["world/frame"].set_object(g.triad())
        
        # Create a simple quadrotor model
        self.create_quadrotor_model()
        
        # Add a ground plane
        self.vis["world/ground"].set_object(
            g.Box([10.0, 10.0, 0.01]),
            g.MeshLambertMaterial(color=0x808080, opacity=0.5)
        )
        
        # Position the ground plane
        self.vis["world/ground"].set_transform(
            tf.translation_matrix([0, 0, -0.5])
        )
        
    def create_quadrotor_model(self):
        """Create a simple quadrotor model with body and rotors"""
        # Create the body as a box
        body_size = [0.2, 0.2, 0.05]
        self.vis["world/quadrotor/body"].set_object(
            g.Box(body_size),
            g.MeshLambertMaterial(color=0x4040FF)
        )
        
        # Create the rotors as cylinders
        rotor_radius = 0.05
        rotor_height = 0.01
        rotor_positions = [
            [0.1, 0.1, 0.03],   # front left
            [0.1, -0.1, 0.03],  # front right
            [-0.1, -0.1, 0.03], # back right
            [-0.1, 0.1, 0.03]   # back left
        ]
        
        rotor_colors = [0xFF0000, 0x00FF00, 0x0000FF, 0xFFFF00]
        
        for i, pos in enumerate(rotor_positions):
            self.vis[f"world/quadrotor/rotor{i}"].set_object(
                g.Cylinder(rotor_height, rotor_radius),
                g.MeshLambertMaterial(color=rotor_colors[i])
            )
            
            self.vis[f"world/quadrotor/rotor{i}"].set_transform(
                tf.translation_matrix(pos)
            )
    
    def update_quadrotor_state(self, position, quaternion):
        """Update the quadrotor visualization with new position and orientation"""
        # Create transformation matrix from position and quaternion
        rotation_matrix = tf.quaternion_matrix([quaternion[0], quaternion[1], quaternion[2], quaternion[3]])
        translation_matrix = tf.translation_matrix([position[0], position[1], position[2]])
        transform = np.dot(translation_matrix, rotation_matrix)
        
        # Apply the transform to the quadrotor
        self.vis["world/quadrotor"].set_transform(transform)
        
        # Add point to trajectory
        self.add_trajectory_point(position)
    
    def add_trajectory_point(self, position):
        """Add a point to the trajectory visualization"""
        # Add the new point
        self.trajectory_points.append(position)
        
        # Limit the number of points
        if len(self.trajectory_points) > self.max_trajectory_points:
            self.trajectory_points.pop(0)
        
        # Create a line strip from the points
        points = np.array(self.trajectory_points)
        
        # Only update the visualization if we have at least 2 points
        if len(points) >= 2:
            self.vis["world/trajectory"].set_object(
                g.LineSegments(
                    g.PointsGeometry(points),
                    g.MeshBasicMaterial(color=0xFF8800)
                )
            )
    
    def reset_trajectory(self):
        """Clear the trajectory visualization"""
        self.trajectory_points = []
        self.vis["world/trajectory"].delete()

# Test the visualizer if run directly
if __name__ == "__main__":
    print("Starting quadrotor visualization demo...")
    vis = QuadrotorVisualizer(auto_open_browser=True)
    
    # Animate a simple trajectory
    print("Animating a simple trajectory...")
    for i in range(100):
        # Create a circular path
        t = i * 0.1
        position = [np.cos(t), np.sin(t), 0.1 * t]
        
        # Create a simple rotation
        angle = t * 0.5
        qw = np.cos(angle/2)
        qx = 0
        qy = 0
        qz = np.sin(angle/2)
        quaternion = [qw, qx, qy, qz]
        
        # Update the visualization
        vis.update_quadrotor_state(position, quaternion)
        
        # Sleep to control the animation speed
        time.sleep(0.05)
    
    print("Demo complete. Keep the browser window open to use with the simulator server.")
    print("Press Ctrl+C to exit.")
