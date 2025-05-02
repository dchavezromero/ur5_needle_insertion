#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from moveit_msgs.msg import CollisionObject, PlanningScene
from shape_msgs.msg import Mesh, MeshTriangle
from geometry_msgs.msg import Point, Pose
import tf2_ros
import os
from ament_index_python.packages import get_package_share_directory
import trimesh
import numpy as np

class CollisionPublisher(Node):
    """
    Node that publishes collision objects for hospital models to the MoveIt planning scene.
    This node loads collision meshes from the hospital_models package and publishes them
    as collision objects to the planning scene, enabling collision avoidance during motion planning.
    """
    def __init__(self):
        super().__init__('collision_publisher')
        
        # Setup
        self.planning_scene_pub = self.create_publisher(PlanningScene, '/planning_scene', 10)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Models to publish (these need to match the static TF frames we set up in our launch file, to be less spammy)
        self.models = [
            ('elderMalePatient', 'ElderMalePatient/meshes/ElderMalePatient.obj'),
            ('bedsideTable', 'BedsideTable/meshes/BedsideTable.obj'),
            ('bedTable', 'BedTable/meshes/BedTable.obj'),
            ('opScrubs', 'OpScrubs/meshes/OpScrubs.obj'),
            ('divider', 'ElderMalePatient/meshes/Divider.stl')
        ]
        
        # Get models path
        self.models_path = os.path.join(get_package_share_directory('hospital_models'), 'models')
        self.get_logger().info(f'Models path: {self.models_path}')
        
        # Start publishing
        self.timer = self.create_timer(1.0, self.publish_objects)
        self.get_logger().info('Collision publisher initialized')

    def publish_objects(self):
        """
        Periodically publishes collision objects to the planning scene.
        Loads meshes, gets their poses from TF, and publishes them as collision objects.
        """
        # self.get_logger().info('Starting to publish collision objects...')
        scene = PlanningScene()
        scene.is_diff = True
        
        for name, mesh_path in self.models:
            try:
                # self.get_logger().info(f'Processing {name}...')
                
                # Get transform
                # self.get_logger().info(f'Looking up transform from world to {name}')
                try:
                    transform = self.tf_buffer.lookup_transform('world', name, rclpy.time.Time())
                    # self.get_logger().info(f'Got transform for {name}: {transform}')
                except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                    # self.get_logger().warn(f'Could not get transform for {name}: {str(e)}')
                    continue
                
                # Load and convert mesh
                full_path = os.path.join(self.models_path, mesh_path)
                # self.get_logger().info(f'Loading mesh from {full_path}')
                mesh = trimesh.load(full_path)
                
                if isinstance(mesh, trimesh.Scene):
                    # self.get_logger().info(f'{name} is a Scene object, combining all meshes')
                    # Combine all meshes in the scene into one
                    combined_mesh = trimesh.util.concatenate(list(mesh.geometry.values()))
                    mesh = combined_mesh
                
                # self.get_logger().info(f'Mesh loaded: {len(mesh.vertices)} vertices, {len(mesh.faces)} faces')
                
                # Create collision object
                co = CollisionObject()
                co.id = name
                co.header.frame_id = 'world'
                
                # Add mesh
                mesh_msg = Mesh()
                for vertex in mesh.vertices:
                    point = Point()
                    point.x, point.y, point.z = vertex
                    mesh_msg.vertices.append(point)
                
                # Convert faces to uint32
                faces = np.array(mesh.faces, dtype=np.uint32)
                for face in faces:
                    triangle = MeshTriangle()
                    triangle.vertex_indices = face
                    mesh_msg.triangles.append(triangle)
                
                co.meshes.append(mesh_msg)
                
                # Create proper Pose message from transform
                pose = Pose()
                pose.position.x = transform.transform.translation.x
                pose.position.y = transform.transform.translation.y
                pose.position.z = transform.transform.translation.z
                pose.orientation = transform.transform.rotation
                co.mesh_poses.append(pose)
                
                co.operation = CollisionObject.ADD
                
                scene.world.collision_objects.append(co)
                # self.get_logger().info(f'Added collision object for {name} to planning scene')
                
            except Exception as e:
                self.get_logger().error(f'Error processing {name}: {str(e)}')
        
        # self.get_logger().info(f'Publishing planning scene with {len(scene.world.collision_objects)} collision objects')
        self.planning_scene_pub.publish(scene)
        # self.get_logger().info('Planning scene published')

def main(args=None):
    rclpy.init(args=args)
    node = CollisionPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 