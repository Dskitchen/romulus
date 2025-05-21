#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PolygonStamped, PointStamped, Point
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np
import os
import svgpathtools

class PatternNode(Node):
    def __init__(self):
        super().__init__('pattern_node')
        
        self.declare_parameter('svg_directory', '/root/Documents/Cake_Drawing')
        self.declare_parameter('pattern_scale', 0.8)
        
        self.svg_directory = self.get_parameter('svg_directory').value
        self.pattern_scale = self.get_parameter('pattern_scale').value
        
        self.available_patterns = self.load_patterns()
        self.current_pattern = None
        
        self.surface_sub = self.create_subscription(
            PolygonStamped,
            '/cake_vision/surface',
            self.surface_callback,
            10)
        self.pattern_select_sub = self.create_subscription(
            String,
            '/cake_vision/select_pattern',
            self.pattern_select_callback,
            10)
        
        self.pattern_marker_pub = self.create_publisher(
            MarkerArray,
            '/cake_vision/pattern_markers',
            10)
        self.decoration_points_pub = self.create_publisher(
            PointStamped,
            '/cake_vision/decoration_point',
            10)
        
        self.get_logger().info(f"Initialized with {len(self.available_patterns)} patterns")
        for pattern in self.available_patterns:
            self.get_logger().info(f"  - {pattern}")
    
    def load_patterns(self):
        patterns = []
        if os.path.exists(self.svg_directory):
            for file in os.listdir(self.svg_directory):
                if file.endswith('.svg'):
                    patterns.append(os.path.splitext(file)[0])
        else:
            self.get_logger().error(f"SVG directory not found: {self.svg_directory}")
        return patterns
    
    def pattern_select_callback(self, msg):
        pattern_name = msg.data
        if pattern_name in self.available_patterns:
            self.current_pattern = pattern_name
            self.get_logger().info(f"Selected pattern: {pattern_name}")
        else:
            self.get_logger().error(f"Pattern '{pattern_name}' not found")
    
    def surface_callback(self, msg):
        if not self.current_pattern:
            self.get_logger().warn("No pattern selected")
            return
        
        pattern_file = os.path.join(self.svg_directory, f"{self.current_pattern}.svg")
        if not os.path.exists(pattern_file):
            self.get_logger().error(f"Pattern file not found: {pattern_file}")
            return
        
        surface_points = [(p.x, p.y, p.z) for p in msg.polygon.points]
        if len(surface_points) < 3:
            self.get_logger().warn("Insufficient surface points")
            return
        
        try:
            pattern_3d = self.project_svg_to_surface(pattern_file, surface_points)
            self.publish_pattern_markers(pattern_3d, msg.header.frame_id)
            
            if pattern_3d and 'paths' in pattern_3d and pattern_3d['paths']:
                first_path = pattern_3d['paths'][0]
                if first_path:
                    point_msg = PointStamped()
                    point_msg.header = msg.header
                    point_msg.point.x = first_path[0][0]
                    point_msg.point.y = first_path[0][1]
                    point_msg.point.z = first_path[0][2]
                    self.decoration_points_pub.publish(point_msg)
                    self.get_logger().info(
                        f"Published decoration point: ({point_msg.point.x:.3f}, {point_msg.point.y:.3f}, {point_msg.point.z:.3f})"
                    )
        except Exception as e:
            self.get_logger().error(f"Error projecting SVG: {str(e)}")
    
    def project_svg_to_surface(self, svg_file, surface_points):
        paths, _ = svgpathtools.svg2paths(svg_file)
        
        surface_center, _, surface_basis = self.calculate_surface_properties(surface_points)
        
        svg_min_x = min([path.bbox()[0] for path in paths if path])
        svg_max_x = max([path.bbox()[1] for path in paths if path])
        svg_min_y = min([path.bbox()[2] for path in paths if path])
        svg_max_y = max([path.bbox()[3] for path in paths if path])
        
        svg_width = svg_max_x - svg_min_x
        svg_height = svg_max_y - svg_min_y
        
        surface_x = np.array(surface_basis[0])
        surface_y = np.array(surface_basis[1])
        
        points = np.array(surface_points)
        points_centered = points - surface_center
        x_coords = np.abs(np.dot(points_centered, surface_x))
        y_coords = np.abs(np.dot(points_centered, surface_y))
        
        surface_width = 2 * np.max(x_coords)
        surface_height = 2 * np.max(y_coords)
        
        scale_x = (surface_width * self.pattern_scale) / svg_width
        scale_y = (surface_height * self.pattern_scale) / svg_height
        scale = min(scale_x, scale_y)
        
        svg_center_x = (svg_min_x + svg_max_x) / 2
        svg_center_y = (svg_min_y + svg_max_y) / 2
        
        pattern_paths = []
        for path in paths:
            if not path:
                continue
            path_3d = []
            num_samples = max(10, int(path.length() / 10))
            
            for i in range(num_samples + 1):
                t = i / num_samples
                point = path.point(t)
                x_svg = point.real - svg_center_x
                y_svg = point.imag - svg_center_y
                x_scaled = x_svg * scale
                y_scaled = -y_svg * scale
                point_3d = surface_center + x_scaled * surface_x + y_scaled * surface_y
                path_3d.append((point_3d[0], point_3d[1], point_3d[2]))
            
            pattern_paths.append(path_3d)
        
        return {'paths': pattern_paths, 'center': surface_center, 'basis': surface_basis}
    
    def calculate_surface_properties(self, points):
        points_array = np.array(points)
        center = np.mean(points_array, axis=0)
        
        points_centered = points_array - center
        _, _, vh = np.linalg.svd(points_centered)
        normal = vh[2, :]
        if normal[1] < 0:
            normal = -normal
        
        if abs(normal[0]) < abs(normal[1]):
            v1 = np.array([1, 0, 0])
        else:
            v1 = np.array([0, 1, 0])
        
        basis_x = np.cross(normal, v1)
        basis_x = basis_x / np.linalg.norm(basis_x)
        basis_y = np.cross(normal, basis_x)
        basis_y = basis_y / np.linalg.norm(basis_y)
        
        return center, normal, (basis_x, basis_y)
    
    def publish_pattern_markers(self, pattern_3d, frame_id):
        if not pattern_3d or 'paths' not in pattern_3d:
            return
        
        marker_array = MarkerArray()
        for i, path in enumerate(pattern_3d['paths']):
            if not path:
                continue
            
            marker = Marker()
            marker.header.frame_id = frame_id
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "svg_pattern"
            marker.id = i
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.scale.x = 0.002
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 1.0
            marker.color.a = 1.0
            
            for point in path:
                p = Point()
                p.x = point[0]
                p.y = point[1]
                p.z = point[2]
                marker.points.append(p)
            
            marker_array.markers.append(marker)
            
            points_marker = Marker()
            points_marker.header = marker.header
            points_marker.ns = "svg_points"
            points_marker.id = i
            points_marker.type = Marker.POINTS
            points_marker.action = Marker.ADD
            points_marker.scale.x = points_marker.scale.y = 0.003
            points_marker.color.r = 1.0
            points_marker.color.g = 1.0
            points_marker.color.b = 0.0
            points_marker.color.a = 1.0
            
            for point in path:
                p = Point()
                p.x = point[0]
                p.y = point[1]
                p.z = point[2]
                points_marker.points.append(p)
            
            marker_array.markers.append(points_marker)
        
        self.pattern_marker_pub.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = PatternNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
