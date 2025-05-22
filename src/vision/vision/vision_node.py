#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, Image, CameraInfo
from geometry_msgs.msg import PolygonStamped, Point32
from std_msgs.msg import String
from visualization_msgs.msg import Marker
from cv_bridge import CvBridge
import numpy as np
import cv2
import math
import sensor_msgs_py.point_cloud2 as pc2
import open3d as o3d
from cake_interfaces.msg import PatternPoints

class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')
        
        # Parameters
        self.declare_parameter('max_surface_angle', 95.0)
        self.declare_parameter('min_surface_area', 0.00005)
        self.declare_parameter('max_surface_area', 0.05)
        self.declare_parameter('plane_distance_threshold', 0.02)
        self.declare_parameter('camera_resolution', 'SVGA')
        self.declare_parameter('cake_hue_min', 10)  # Example for beige cake
        self.declare_parameter('cake_hue_max', 30)
        self.declare_parameter('cake_sat_min', 50)
        self.declare_parameter('cake_sat_max', 255)
        self.declare_parameter('cake_val_min', 50)
        self.declare_parameter('cake_val_max', 255)
        
        self.max_surface_angle = self.get_parameter('max_surface_angle').value
        self.min_surface_area = self.get_parameter('min_surface_area').value
        self.max_surface_area = self.get_parameter('max_surface_area').value
        self.plane_distance_threshold = self.get_parameter('plane_distance_threshold').value
        self.cake_hue_min = self.get_parameter('cake_hue_min').value
        self.cake_hue_max = self.get_parameter('cake_hue_max').value
        self.cake_sat_min = self.get_parameter('cake_sat_min').value
        self.cake_sat_max = self.get_parameter('cake_sat_max').value
        self.cake_val_min = self.get_parameter('cake_val_min').value
        self.cake_val_max = self.get_parameter('cake_val_max').value
        camera_resolution = self.get_parameter('camera_resolution').value
        
        # CvBridge for image conversion
        self.bridge = CvBridge()
        
        # Publishers
        self.surface_pub = self.create_publisher(PolygonStamped, '/cake_vision/surface', 10)
        self.shape_type_pub = self.create_publisher(String, '/cake_vision/shape_type', 10)
        self.marker_pub = self.create_publisher(Marker, '/cake_vision/surface_marker', 10)
        self.image_pub = self.create_publisher(Image, '/cake_vision/processed_image', 10)
        
        # Subscribers
        self.pc_sub = self.create_subscription(
            PointCloud2,
            '/zed/zed_node/point_cloud/cloud_registered',
            self.point_cloud_callback,
            10
        )
        self.image_sub = self.create_subscription(
            Image,
            '/zed/zed_node/rgb/image_rect_color',
            self.image_callback,
            10
        )
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/zed/zed_node/rgb/camera_info',
            self.camera_info_callback,
            10
        )
        self.pattern_sub = self.create_subscription(
            String,
            '/cake_vision/select_pattern',
            self.pattern_callback,
            10
        )
        self.pattern_points_sub = self.create_subscription(
            PatternPoints,
            '/cake_vision/pattern_points',
            self.pattern_points_callback,
            10
        )
        
        # Store latest data
        self.latest_point_cloud = None
        self.latest_image = None
        self.latest_image_msg = None
        self.selected_pattern = None
        self.pattern_points = []  # List of (u, v) points in image coordinates
        
        # Camera intrinsics
        self.left_intrinsics = {'fx': 734.1024780273438, 'fy': 734.1024780273438, 'cx': 986.13525390625, 'cy': 587.7171630859375}
        
        # Timer for periodic detection
        self.timer = self.create_timer(0.1, self.detect_surfaces)
        
        self.get_logger().info("Vision Node initialized")
    
    def point_cloud_callback(self, msg):
        self.latest_point_cloud = msg
        self.get_logger().debug("Received point cloud")
    
    def image_callback(self, msg):
        self.latest_image_msg = msg
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.get_logger().debug("Received image")
        except Exception as e:
            self.get_logger().error(f"Error converting image: {str(e)}")
    
    def camera_info_callback(self, msg):
        self.left_intrinsics['fx'] = msg.k[0]
        self.left_intrinsics['fy'] = msg.k[4]
        self.left_intrinsics['cx'] = msg.k[2]
        self.left_intrinsics['cy'] = msg.k[5]
        self.get_logger().info(f"Updated intrinsics: fx={self.left_intrinsics['fx']}, fy={self.left_intrinsics['fy']}, "
                              f"cx={self.left_intrinsics['cx']}, cy={self.left_intrinsics['cy']}")
    
    def pattern_callback(self, msg):
        self.selected_pattern = msg.data
        self.get_logger().info(f"Received pattern selection: {self.selected_pattern}")
    
    def pattern_points_callback(self, msg):
        self.pattern_points = [(p.x, p.y) for p in msg.points]
        self.get_logger().info(f"Received {len(self.pattern_points)} pattern points")
    
    def detect_surfaces(self):
        if self.latest_point_cloud is None or self.latest_image is None:
            self.get_logger().warn("Waiting for point cloud and image data")
            return
        
        # Step 1: Color-based segmentation in HSV space
        try:
            hsv = cv2.cvtColor(self.latest_image, cv2.COLOR_BGR2HSV)
            lower_bound = np.array([self.cake_hue_min, self.cake_sat_min, self.cake_val_min])
            upper_bound = np.array([self.cake_hue_max, self.cake_sat_max, self.cake_val_max])
            mask = cv2.inRange(hsv, lower_bound, upper_bound)
            
            # Apply morphological operations to reduce noise
            kernel = np.ones((5, 5), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            
            # Step 2: Find contours in the mask
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            if not contours:
                self.get_logger().warn("No contours detected after color segmentation")
                return
            
            # Filter contours by area (approximate pixel area for 230 mm cake)
            expected_area_pixels = 5000  # Adjusted based on trial
            best_contour = None
            max_area = 0
            for contour in contours:
                area = cv2.contourArea(contour)
                if 0.5 * expected_area_pixels < area < 2 * expected_area_pixels:
                    if area > max_area:
                        max_area = area
                        best_contour = contour
            
            if best_contour is None:
                self.get_logger().warn("No contours match the expected area")
                return
            
            # Step 3: Compute bounding box for the contour
            x, y, w, h = cv2.boundingRect(best_contour)
            u_min = x
            u_max = x + w
            v_min = y
            v_max = y + h
            
            # Step 4: Map the bounding box to the point cloud
            points_gen = pc2.read_points(self.latest_point_cloud, field_names=("x", "y", "z", "rgb"), skip_nans=True)
            points_list = []
            for p in points_gen:
                x, y, z, rgb = p
                if z <= 0:
                    continue
                u = (x * self.left_intrinsics['fx'] / z) + self.left_intrinsics['cx']
                v = (y * self.left_intrinsics['fy'] / z) + self.left_intrinsics['cy']
                if u_min <= u <= u_max and v_min <= v <= v_max:
                    points_list.append((x, y, z))
            
            if len(points_list) < 5:
                self.get_logger().warn("Not enough points in the cropped point cloud")
                return
            
            points = np.array(points_list, dtype=np.float32)
            self.get_logger().debug(f"Cropped point cloud points shape: {points.shape}")
            
            # Step 5: RANSAC plane detection on cropped points
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(points)
            
            plane_model, inliers = pcd.segment_plane(
                distance_threshold=self.plane_distance_threshold,
                ransac_n=5,
                num_iterations=2000
            )
            if len(inliers) < 5:
                self.get_logger().warn("Too few inliers for plane detection")
                return
            
            plane_points = points[inliers]
            self.get_logger().debug(f"Plane points shape: {plane_points.shape}")
            
            # Compute plane normal and check angle
            normal = np.array(plane_model[:3])
            angle_rad = math.acos(abs(normal[1]))
            angle_deg = math.degrees(angle_rad)
            self.get_logger().debug(f"Plane normal: {normal}, angle: {angle_deg:.1f}°")
            
            if angle_deg > self.max_surface_angle:
                self.get_logger().info(f"Plane angle {angle_deg:.1f}° exceeds max_surface_angle {self.max_surface_angle}°")
                return
            
            # Compute plane area and boundary
            plane_pcd = o3d.geometry.PointCloud()
            plane_pcd.points = o3d.utility.Vector3dVector(plane_points)
            plane_pcd.normals = o3d.utility.Vector3dVector(np.tile(normal, (len(plane_points), 1)))
            
            plane_points_2d = plane_points[:, [0, 2]]
            hull = cv2.convexHull(plane_points_2d.astype(np.float32))
            boundary_points = [(p[0], plane_points[inliers[i]][1], p[1]) for i, p in enumerate(hull[:, 0, :])]
            area_pixels = cv2.contourArea(hull)
            
            avg_z = np.mean(plane_points[:, 2])
            area_m2 = (area_pixels / (self.left_intrinsics['fx'] * self.left_intrinsics['fy'])) * (avg_z ** 2)
            self.get_logger().debug(f"Computed plane area: {area_m2:.4f}m² (pixels: {area_pixels}, avg_z: {avg_z:.2f})")
            self.get_logger().debug(f"Boundary points: {boundary_points}")
            
            if not (self.min_surface_area <= area_m2 <= self.max_surface_area):
                self.get_logger().info(f"Plane area {area_m2:.4f}m² outside constraints [{self.min_surface_area}, {self.max_surface_area}]")
                return
            
            # Step 6: Classify shape as cylinder or box
            shape_type = self.determine_shape_type(best_contour)
            
            center = np.mean(plane_points, axis=0)
            self.get_logger().debug(f"Plane center: {center}")
            
            plane_info = {
                'boundary': boundary_points,
                'center': center,
                'normal': normal,
                'area': area_m2,
                'shape_type': shape_type
            }
            
            self.get_logger().info(f"Detected plane: area={area_m2:.4f}m², shape={shape_type}")
            self.publish_surface(plane_info, self.latest_image.copy())
        
        except Exception as e:
            self.get_logger().error(f"Error in surface detection: {str(e)}")
    
    def determine_shape_type(self, contour):
        perimeter = cv2.arcLength(contour, True)
        area = cv2.contourArea(contour)
        
        if area == 0 or perimeter == 0:
            self.get_logger().warn("Zero contour area or perimeter")
            return "unknown"
        
        circularity = 4 * math.pi * area / (perimeter * perimeter)
        
        # Fit an ellipse if possible
        if len(contour) >= 5:
            ellipse = cv2.fitEllipse(contour)
            ellipse_ratio = min(ellipse[1][0], ellipse[1][1]) / max(ellipse[1][0], ellipse[1][1])
        else:
            ellipse_ratio = 0
            self.get_logger().debug("Not enough points for ellipse fitting")
        
        # Fit a rectangle
        rect = cv2.minAreaRect(contour)
        rect_area = rect[1][0] * rect[1][1]
        rect_ratio = min(rect[1][0], rect[1][1]) / max(rect[1][0], rect[1][1])
        
        self.get_logger().debug(
            f"Shape analysis: circularity={circularity:.2f}, ellipse_ratio={ellipse_ratio:.2f}, "
            f"rect_ratio={rect_ratio:.2f}, area_ratio={area/rect_area if rect_area else 0:.2f}"
        )
        
        if circularity > 0.6 and ellipse_ratio > 0.6:
            return "cylinder"
        elif rect_area and area / rect_area > 0.65 and rect_ratio < 0.9:
            return "box"
        return "unknown"
    
    def publish_surface(self, plane, image):
        polygon_msg = PolygonStamped()
        polygon_msg.header.stamp = self.get_clock().now().to_msg()
        polygon_msg.header.frame_id = "zed_left_camera_optical_frame"
        
        for point in plane['boundary']:
            p = Point32()
            p.x = float(point[0])
            p.y = float(point[1])
            p.z = float(point[2])
            polygon_msg.polygon.points.append(p)
        
        self.surface_pub.publish(polygon_msg)
        
        type_msg = String()
        type_msg.data = plane['shape_type']
        self.shape_type_pub.publish(type_msg)
        
        self.draw_surface_on_image(plane, image)
        
        marker = Marker()
        marker.header.frame_id = "zed_left_camera_optical_frame"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "cake_surfaces"
        marker.id = 0
        marker.type = Marker.TRIANGLE_LIST
        marker.action = Marker.ADD
        
        marker.color.r = 1.0 if plane['shape_type'] == "cylinder" else 0.0
        marker.color.g = 0.0
        marker.color.b = 0.0 if plane['shape_type'] == "cylinder" else 1.0
        marker.color.a = 0.7
        marker.scale.x = marker.scale.y = marker.scale.z = 1.0
        
        center = plane['center']
        if len(plane['boundary']) >= 3:
            for i in range(len(plane['boundary']) - 1):
                p1 = plane['boundary'][i]
                p2 = plane['boundary'][i + 1]
                marker.points.extend([
                    self.create_point(center[0], center[1], center[2]),
                    self.create_point(p1[0], p1[1], p1[2]),
                    self.create_point(p2[0], p2[1], p2[2])
                ])
            p1 = plane['boundary'][-1]
            p2 = plane['boundary'][0]
            marker.points.extend([
                self.create_point(center[0], center[1], center[2]),
                self.create_point(p1[0], p1[1], p1[2]),
                self.create_point(p2[0], p2[1], p2[2])
            ])
        
        self.marker_pub.publish(marker)
    
    def draw_surface_on_image(self, plane, image):
        points_2d = []
        for point in plane['boundary']:
            x, y, z = point[0], point[1], point[2]
            if z <= 0:
                continue
            u = (x * self.left_intrinsics['fx'] / z) + self.left_intrinsics['cx']
            v = (y * self.left_intrinsics['fy'] / z) + self.left_intrinsics['cy']
            points_2d.append([int(u), int(v)])
        
        if len(points_2d) >= 3:
            contour = np.array(points_2d).reshape(-1, 1, 2).astype(np.int32)
            cv2.drawContours(image, [contour], 0, (0, 255, 0), 2)
            
            cx = int(sum(p[0] for p in points_2d) / len(points_2d))
            cy = int(sum(p[1] for p in points_2d) / len(points_2d))
            cv2.circle(image, (cx, cy), 5, (0, 0, 255), -1)
            
            cv2.putText(image, f"Type: {plane['shape_type']}", (cx + 10, cy),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
            cv2.putText(image, f"Area: {plane['area']:.3f} m²", (cx + 10, cy + 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
            
            # Overlay pattern points if available, translated to plane center
            if self.pattern_points:
                translated_points = []
                for u, v in self.pattern_points:
                    u_trans = int(u + (cx - 960))
                    v_trans = int(v + (cy - 600))
                    u_trans = max(0, min(u_trans, 1919))
                    v_trans = max(0, min(v_trans, 1199))
                    translated_points.append((u_trans, v_trans))
                    self.get_logger().debug(f"Translated point: u={u_trans:.2f}, v={v_trans:.2f}")
                for i in range(len(translated_points) - 1):
                    u1, v1 = translated_points[i]
                    u2, v2 = translated_points[i + 1]
                    cv2.line(image, (u1, v1), (u2, v2), (255, 0, 0), 2)
                u1, v1 = translated_points[-1]
                u2, v2 = translated_points[0]
                cv2.line(image, (u1, v1), (u2, v2), (255, 0, 0), 2)
        
        try:
            image_msg = self.bridge.cv2_to_imgmsg(image, "bgr8")
            image_msg.header.stamp = self.get_clock().now().to_msg()
            image_msg.header.frame_id = "zed_left_camera_optical_frame"
            self.image_pub.publish(image_msg)
        except Exception as e:
            self.get_logger().error(f"Error publishing image: {str(e)}")
    
    def create_point(self, x, y, z):
        from geometry_msgs.msg import Point
        p = Point()
        p.x = float(x)
        p.y = float(y)
        p.z = float(z)
        return p
    
    def cleanup(self):
        pass
    
    def __del__(self):
        self.cleanup()

def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down due to KeyboardInterrupt")
    finally:
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
