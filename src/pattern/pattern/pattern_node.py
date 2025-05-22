#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from cake_interfaces.msg import PatternPoints
from geometry_msgs.msg import Point32
from std_msgs.msg import Header
import os
import xml.etree.ElementTree as ET
import numpy as np
import math

class PatternNode(Node):
    def __init__(self):
        super().__init__('pattern_node')
        
        # Subscriber for pattern selection
        self.pattern_sub = self.create_subscription(
            String,
            '/cake_vision/select_pattern',
            self.pattern_callback,
            10
        )
        
        # Publisher for pattern points
        self.points_pub = self.create_publisher(
            PatternPoints,
            '/cake_vision/pattern_points',
            10
        )
        
        # SVG directory
        self.svg_dir = "/root/Documents/Cake_Drawing"
        if not os.path.exists(self.svg_dir):
            os.makedirs(self.svg_dir)
        
        self.get_logger().info("Pattern Node initialized")
    
    def pattern_callback(self, msg):
        pattern = msg.data
        self.get_logger().info(f"Received pattern selection: {pattern}")
        
        # Generate or load SVG (simplified example for circle)
        svg_path = os.path.join(self.svg_dir, f"{pattern}.svg")
        if pattern == "circle":
            # Generate a larger circle SVG (radius 200 pixels, centered at 960,600 for 1920x1200 image)
            svg_content = '''<svg width="1920" height="1200" xmlns="http://www.w3.org/2000/svg">
                             <circle cx="960" cy="600" r="200" fill="none" stroke="black" stroke-width="2"/>
                             </svg>'''
            with open(svg_path, 'w') as f:
                f.write(svg_content)
        
        # Parse SVG and extract points
        points = self.parse_svg(svg_path)
        
        # Publish points
        points_msg = PatternPoints()
        points_msg.header = Header()
        points_msg.header.stamp = self.get_clock().now().to_msg()
        points_msg.header.frame_id = "zed_left_camera_optical_frame"
        for u, v in points:
            p = Point32()
            p.x = float(u)
            p.y = float(v)
            p.z = 0.0
            points_msg.points.append(p)
        
        self.points_pub.publish(points_msg)
        self.get_logger().info(f"Published {len(points)} pattern points for {pattern}")
    
    def parse_svg(self, svg_path):
        # Parse SVG file and extract points (simplified for a circle)
        tree = ET.parse(svg_path)
        root = tree.getroot()
        
        points = []
        for elem in root.iter('{http://www.w3.org/2000/svg}circle'):
            cx = float(elem.get('cx'))
            cy = float(elem.get('cy'))
            r = float(elem.get('r'))
            
            # Sample points along the circle
            num_points = 50
            for i in range(num_points):
                theta = (2 * math.pi * i) / num_points
                u = cx + r * math.cos(theta)
                v = cy + r * math.sin(theta)
                points.append((u, v))
                self.get_logger().debug(f"Point {i}: u={u:.2f}, v={v:.2f}")
        
        return points

def main(args=None):
    rclpy.init(args=args)
    node = PatternNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down due to KeyboardInterrupt")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
