#!/usr/bin/env python3
# visualizer_urdf.py
import numpy as np
import meshcat
import meshcat.geometry as g
import meshcat.transformations as tf
import time
import os
import subprocess
import webbrowser
import threading
from urllib.request import urlretrieve
import tempfile

try:
    import pybullet as p
    PYBULLET_AVAILABLE = True
except ImportError:
    PYBULLET_AVAILABLE = False
    print("PyBullet not installed - URDF loading may not work properly")
    print("Install with: pip install pybullet")

class URDFQuadrotorVisualizer:
    def __init__(self, urdf_path=None, mesh_dir=None, auto_open_browser=True, scale=1.0):
        """
        Initialize a visualizer for URDF models
        
        Args:
            urdf_path: Path to the URDF file
            mesh_dir: Directory containing mesh files (optional)
            auto_open_browser: Whether to open browser automatically
            scale: Scale factor for the model
        """
        # Initialize the visualizer
        self.vis = meshcat.Visualizer()
        self.urdf_path = urdf_path
        self.mesh_dir = mesh_dir
        self.scale = scale
        
        # Automatically open the visualizer in the browser
        self.url = self.vis.url()
        print(f"Meshcat visualizer running at: {self.url}")
        
        if auto_open_browser:
            # Open browser in a separate thread to not block
            threading.Thread(target=self._open_browser).start()
        
        # Set up the scene
        self.setup_scene()
        
        # Initialize trajectory points
        self.trajectory_points = []
        self.max_trajectory_points = 1000  # Maximum number of points to show
        
        # Load the URDF if provided
        if urdf_path:
            if PYBULLET_AVAILABLE:
                self.load_urdf_model(urdf_path, mesh_dir)
            else:
                self._create_default_quadrotor_model()
                print("WARNING: PyBullet not available. Using default quadrotor model.")
        else:
            self._create_default_quadrotor_model()
        
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
        """Set up the visualization scene with coordinate frame and ground plane"""
        # Add coordinate frame
        self.vis["world/frame"].set_object(g.triad())
        
        # Add a ground plane
        self.vis["world/ground"].set_object(
            g.Box([10.0, 10.0, 0.01]),
            g.MeshLambertMaterial(color=0x808080, opacity=0.5)
        )
        
        # Position the ground plane
        self.vis["world/ground"].set_transform(
            tf.translation_matrix([0, 0, -0.5])
        )
    
    def _create_default_quadrotor_model(self):
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
    
    def load_urdf_model(self, urdf_path, mesh_dir=None):
        """
        Load a URDF model using PyBullet and convert to Meshcat visualization
        
        Args:
            urdf_path: Path to the URDF file
            mesh_dir: Directory containing mesh files (optional)
        """
        # Configure PyBullet
        physics_client = p.connect(p.DIRECT)  # No GUI, just for loading URDF
        p.setAdditionalSearchPath(mesh_dir if mesh_dir else os.path.dirname(urdf_path))
        
        # Load the URDF
        try:
            model_id = p.loadURDF(urdf_path, [0, 0, 0], useFixedBase=1, globalScaling=self.scale)
            print(f"Loaded URDF model from {urdf_path}")
            
            # Iterate through all links
            num_joints = p.getNumJoints(model_id)
            print(f"Loaded model with {num_joints} joints")
            
            # Process each link in the model
            for joint_id in range(num_joints):
                joint_info = p.getJointInfo(model_id, joint_id)
                link_name = joint_info[12].decode('utf-8')
                print(f"Processing link: {link_name}")
                
                # Get visual shapes for this link
                visual_shapes = p.getVisualShapeData(model_id, joint_id)
                for i, visual in enumerate(visual_shapes):
                    # Extract mesh file path if available
                    if visual[2] == p.GEOM_MESH:  # If it's a mesh
                        mesh_file = visual[4].decode('utf-8')
                        mesh_scale = visual[3]
                        mesh_position = visual[5]
                        mesh_orientation = visual[6]
                        
                        # Load mesh into Meshcat
                        try:
                            mesh_path = os.path.join(mesh_dir if mesh_dir else os.path.dirname(urdf_path), mesh_file)
                            print(f"Loading mesh: {mesh_path}")
                            if os.path.exists(mesh_path):
                                self._load_mesh_to_meshcat(link_name, mesh_path, mesh_scale, mesh_position, mesh_orientation)
                            else:
                                print(f"Mesh file not found: {mesh_path}")
                        except Exception as e:
                            print(f"Error loading mesh for link {link_name}: {e}")
                    
                    # Handle other geometry types
                    elif visual[2] == p.GEOM_BOX:
                        half_extents = visual[3]
                        position = visual[5]
                        orientation = visual[6]
                        self._add_box_to_meshcat(link_name, half_extents, position, orientation)
                    
                    elif visual[2] == p.GEOM_CYLINDER:
                        size = visual[3]  # [radius, height]
                        position = visual[5]
                        orientation = visual[6]
                        self._add_cylinder_to_meshcat(link_name, size, position, orientation)
                    
                    elif visual[2] == p.GEOM_SPHERE:
                        radius = visual[3][0]  # Radius is the first element
                        position = visual[5]
                        orientation = visual[6]
                        self._add_sphere_to_meshcat(link_name, radius, position, orientation)
            
        except Exception as e:
            print(f"Failed to load URDF model: {e}")
            # Fallback to default model
            self._create_default_quadrotor_model()
        
        finally:
            # Disconnect from PyBullet
            p.disconnect()
    
    def _load_mesh_to_meshcat(self, link_name, mesh_path, scale, position, orientation):
        """Load a mesh file into Meshcat"""
        try:
            # Check file extension
            if mesh_path.endswith('.obj'):
                mesh = g.ObjMeshGeometry.from_file(mesh_path)
            elif mesh_path.endswith('.stl'):
                mesh = g.StlMeshGeometry.from_file(mesh_path)
            elif mesh_path.endswith('.dae'):
                print(f"DAE (Collada) format not directly supported - using converter")
                # Try to convert DAE to OBJ using a temporary file
                with tempfile.NamedTemporaryFile(suffix='.obj') as tmp:
                    # For now, just display a primitive instead
                    mesh = g.Box([0.1, 0.1, 0.1])  # Placeholder
            else:
                print(f"Unsupported mesh format: {mesh_path}")
                mesh = g.Box([0.1, 0.1, 0.1])  # Placeholder
            
            # Create material
            material = g.MeshLambertMaterial(color=0x8888FF)
            
            # Add to visualizer
            self.vis[f"world/quadrotor/{link_name}"].set_object(mesh, material)
            
            # Apply transform
            pos_matrix = tf.translation_matrix(position)
            rot_matrix = tf.quaternion_matrix([orientation[3], orientation[0], orientation[1], orientation[2]])
            scale_matrix = np.diag([scale[0], scale[1], scale[2], 1.0])
            transform = pos_matrix @ rot_matrix @ scale_matrix
            
            self.vis[f"world/quadrotor/{link_name}"].set_transform(transform)
            
        except Exception as e:
            print(f"Error loading mesh {mesh_path}: {e}")
    
    def _add_box_to_meshcat(self, link_name, half_extents, position, orientation):
        """Add a box to Meshcat"""
        # Create box geometry
        box = g.Box([2*half_extents[0], 2*half_extents[1], 2*half_extents[2]])
        material = g.MeshLambertMaterial(color=0x8888FF)
        
        # Add to visualizer
        self.vis[f"world/quadrotor/{link_name}"].set_object(box, material)
        
        # Apply transform
        pos_matrix = tf.translation_matrix(position)
        rot_matrix = tf.quaternion_matrix([orientation[3], orientation[0], orientation[1], orientation[2]])
        transform = pos_matrix @ rot_matrix
        
        self.vis[f"world/quadrotor/{link_name}"].set_transform(transform)
    
    def _add_cylinder_to_meshcat(self, link_name, size, position, orientation):
        """Add a cylinder to Meshcat"""
        # Create cylinder geometry (radius, height)
        cylinder = g.Cylinder(size[1], size[0])
        material = g.MeshLambertMaterial(color=0x8888FF)
        
        # Add to visualizer
        self.vis[f"world/quadrotor/{link_name}"].set_object(cylinder, material)
        
        # Apply transform
        pos_matrix = tf.translation_matrix(position)
        rot_matrix = tf.quaternion_matrix([orientation[3], orientation[0], orientation[1], orientation[2]])
        transform = pos_matrix @ rot_matrix
        
        self.vis[f"world/quadrotor/{link_name}"].set_transform(transform)
    
    def _add_sphere_to_meshcat(self, link_name, radius, position, orientation):
        """Add a sphere to Meshcat"""
        # Create sphere geometry
        sphere = g.Sphere(radius)
        material = g.MeshLambertMaterial(color=0x8888FF)
        
        # Add to visualizer
        self.vis[f"world/quadrotor/{link_name}"].set_object(sphere, material)
        
        # Apply transform
        pos_matrix = tf.translation_matrix(position)
        rot_matrix = tf.quaternion_matrix([orientation[3], orientation[0], orientation[1], orientation[2]])
        transform = pos_matrix @ rot_matrix
        
        self.vis[f"world/quadrotor/{link_name}"].set_transform(transform)
    
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
    import argparse
    
    parser = argparse.ArgumentParser(description="URDF Quadrotor Visualizer")
    parser.add_argument("--urdf", type=str, help="Path to URDF file")
    parser.add_argument("--mesh_dir", type=str, help="Path to mesh directory")
    parser.add_argument("--scale", type=float, default=1.0, help="Scale factor for the model")
    
    args = parser.parse_args()
    
    print("Starting URDF quadrotor visualization demo...")
    
    if args.urdf:
        vis = URDFQuadrotorVisualizer(args.urdf, args.mesh_dir, scale=args.scale)
    else:
        print("No URDF file specified. Using default quadrotor model.")
        vis = URDFQuadrotorVisualizer()
    
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
