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
        
        # Parameters for plane detection
        self.declare_parameter('max_surface_angle', 10.0) # Max deviation from Z-axis (degrees)
        self.declare_parameter('min_surface_area', 0.00005) # meters^2
        self.declare_parameter('max_surface_area', 0.08) # meters^2 
        self.declare_parameter('plane_distance_threshold', 0.02) # meters for RANSAC
        self.declare_parameter('camera_resolution', 'SVGA') 
        
        self.max_surface_angle = self.get_parameter('max_surface_angle').value
        self.min_surface_area = self.get_parameter('min_surface_area').value
        self.max_surface_area = self.get_parameter('max_surface_area').value
        self.plane_distance_threshold = self.get_parameter('plane_distance_threshold').value
        
        self.bridge = CvBridge()
        
        self.surface_pub = self.create_publisher(PolygonStamped, '/cake_vision/surface', 10)
        self.shape_type_pub = self.create_publisher(String, '/cake_vision/shape_type', 10)
        self.marker_pub = self.create_publisher(Marker, '/cake_vision/surface_marker', 10)
        self.image_pub = self.create_publisher(Image, '/cake_vision/processed_image', 10)
        self.mask_pub = self.create_publisher(Image, '/cake_vision/debug_mask', 10)
        
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
        
        self.latest_point_cloud = None
        self.latest_image = None
        self.latest_image_msg = None 
        self.selected_pattern = None
        self.pattern_points = []
        
        self.left_intrinsics = {'fx': 734.0, 'fy': 734.0, 'cx': 960.0, 'cy': 600.0} 
        
        self.timer = self.create_timer(0.1, self.detect_surfaces) 
        
        self.get_logger().info("Vision Node initialized")
    
    def point_cloud_callback(self, msg):
        self.latest_point_cloud = msg
    
    def image_callback(self, msg):
        self.latest_image_msg = msg 
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Error converting image: {str(e)}")
    
    def camera_info_callback(self, msg):
        if self.left_intrinsics['fx'] != msg.k[0]: 
            self.left_intrinsics['fx'] = msg.k[0]
            self.left_intrinsics['fy'] = msg.k[4]
            self.left_intrinsics['cx'] = msg.k[2]
            self.left_intrinsics['cy'] = msg.k[5]
            self.get_logger().info(f"Updated intrinsics: fx={self.left_intrinsics['fx']:.2f}, fy={self.left_intrinsics['fy']:.2f}, "
                                  f"cx={self.left_intrinsics['cx']:.2f}, cy={self.left_intrinsics['cy']:.2f}")
    
    def pattern_callback(self, msg):
        self.selected_pattern = msg.data
        self.get_logger().info(f"Received pattern selection: {self.selected_pattern}")
    
    def pattern_points_callback(self, msg):
        self.pattern_points = [(p.x, p.y) for p in msg.points]
        self.get_logger().info(f"Received {len(self.pattern_points)} pattern points")
    
    def detect_surfaces(self):
        if self.latest_point_cloud is None or self.latest_image is None or self.latest_image_msg is None:
            return
        
        current_image = self.latest_image.copy()
        current_point_cloud = self.latest_point_cloud 
        current_image_header = self.latest_image_msg.header

        try:
            hsv = cv2.cvtColor(current_image, cv2.COLOR_BGR2HSV)
            
            red_tuned_lower = np.array([0, 94, 136]) # HSV from your tuner
            red_tuned_upper = np.array([17, 229, 220]) # HSV from your tuner
            
            mask_red = cv2.inRange(hsv, red_tuned_lower, red_tuned_upper)
            mask = mask_red 
            
            # --- MORPHOLOGICAL OPERATION RE-ENABLED ---
            kernel = np.ones((3, 3), np.uint8) 
            # Try MORPH_CLOSE to fill holes and smooth the blob from the raw HSV mask
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel) 
            # If MORPH_CLOSE is not good, you can try MORPH_OPEN again:
            # mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            
            try:
                mask_msg = self.bridge.cv2_to_imgmsg(mask, "mono8")
                mask_msg.header = current_image_header 
                self.mask_pub.publish(mask_msg)
            except Exception as e:
                self.get_logger().error(f"Error publishing debug mask: {str(e)}")

            debug_image = current_image.copy()
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            if not contours:
                try: 
                    image_msg = self.bridge.cv2_to_imgmsg(debug_image, "bgr8")
                    image_msg.header = current_image_header
                    self.image_pub.publish(image_msg)
                except Exception as e:
                    self.get_logger().error(f"Error publishing debug image (no contours): {str(e)}")
                return
            
            expected_areas = {
                '9.5_inch_round': 55000,  # UPDATED based on last logs (Area ~54-56k)
            }
            
            best_contour = None
            best_shape = None

            for i, contour_candidate in enumerate(contours):
                area = cv2.contourArea(contour_candidate)
                
                if area < 5000: 
                    continue
                
                perimeter = cv2.arcLength(contour_candidate, True)
                if perimeter == 0:
                    continue
                circularity = 4 * math.pi * area / (perimeter ** 2)
                
                self.get_logger().info(f"Contour {i}: Area={area:.0f}, Circularity={circularity:.3f}, BBox={cv2.boundingRect(contour_candidate)}")

                shape_type = None
                # KEEP THIS THRESHOLD LOW FOR NOW. ADJUST AFTER SEEING NEW CIRCULARITY VALUES WITH MORPH_CLOSE
                if circularity > 0.30: 
                    (x_c, y_c), radius = cv2.minEnclosingCircle(contour_candidate)
                    self.get_logger().info(f"Contour {i} (circ_passed): Area={area:.0f}, Radius_px={radius:.1f}")
                    
                    # Radius check for 9.5-inch cake (Area ~55000 -> Radius ~132px)
                    if radius > 120 and radius < 145: # Range around 132px
                         shape_type = '9.5_inch_round'
                    
                if shape_type:
                    self.get_logger().debug(f"Potential shape for contour {i}: {shape_type}")
                    if shape_type == '9.5_inch_round':
                        if best_contour is None or area > cv2.contourArea(best_contour):
                            best_contour = contour_candidate
                            best_shape = shape_type
            
            if best_contour is None:
                self.get_logger().warn("No suitable '9.5_inch_round' shape found after filtering")
                try: 
                    image_msg = self.bridge.cv2_to_imgmsg(debug_image, "bgr8")
                    image_msg.header = current_image_header
                    self.image_pub.publish(image_msg)
                except Exception as e:
                    self.get_logger().error(f"Error publishing debug image (no suitable shape): {str(e)}")
                return
            
            self.get_logger().info(f"SELECTED: {best_shape} with contour area {cv2.contourArea(best_contour):.0f}")
            
            x, y, w, h = cv2.boundingRect(best_contour)
            cv2.drawContours(debug_image, [best_contour], -1, (0, 255, 0), 3)
            cv2.rectangle(debug_image, (x, y), (x+w, y+h), (255, 0, 0), 2)
            cv2.putText(debug_image, best_shape, (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
            try:
                image_msg = self.bridge.cv2_to_imgmsg(debug_image, "bgr8")
                image_msg.header = current_image_header
                self.image_pub.publish(image_msg)
            except Exception as e:
                self.get_logger().error(f"Error publishing processed image: {str(e)}")
            
            u_min = x
            u_max = x + w
            v_min = y
            v_max = y + h
            
            points_gen = pc2.read_points(current_point_cloud, field_names=("x", "y", "z"), skip_nans=True)
            points_list = []
            invalid_depth_points = 0
            total_points_in_cloud = 0
            
            for p_idx, p_xyz in enumerate(points_gen):
                total_points_in_cloud +=1
                p_x, p_y, p_z = p_xyz

                if not (0.1 < p_z < 2.0): 
                    invalid_depth_points += 1
                    continue
                
                u_proj = (p_x * self.left_intrinsics['fx'] / p_z) + self.left_intrinsics['cx']
                v_proj = (p_y * self.left_intrinsics['fy'] / p_z) + self.left_intrinsics['cy']
                
                padding = 10 
                if (u_min - padding) <= u_proj <= (u_max + padding) and \
                   (v_min - padding) <= v_proj <= (v_max + padding):
                    points_list.append((p_x, p_y, p_z))
            
            self.get_logger().info(f"Found {len(points_list)} points in cropped cloud for RANSAC.")
            
            if len(points_list) < 100: 
                self.get_logger().warn("Not enough points in cropped cloud for RANSAC.")
                return
            
            points_for_ransac = np.array(points_list, dtype=np.float32)
            
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(points_for_ransac)
            
            plane_model, inliers = pcd.segment_plane(
                distance_threshold=self.plane_distance_threshold,
                ransac_n=3, 
                num_iterations=1000 
            )
            self.get_logger().info(f"RANSAC: {len(inliers)} inliers for plane model out of {len(points_for_ransac)}.")

            if len(inliers) < 50: 
                self.get_logger().warn(f"Too few RANSAC inliers ({len(inliers)}) for reliable plane.")
                return
            
            plane_points = points_for_ransac[inliers]
            normal = np.array(plane_model[:3])
            
            normal_unit = normal / np.linalg.norm(normal)
            angle_with_camera_z_axis_rad = math.acos(abs(normal_unit[2])) 
            angle_with_camera_z_axis_deg = math.degrees(angle_with_camera_z_axis_rad)
            
            self.get_logger().info(f"Plane normal: {normal_unit}, Angle with camera Z-axis: {angle_with_camera_z_axis_deg:.1f}°")
            
            if angle_with_camera_z_axis_deg > self.max_surface_angle:
                self.get_logger().info(f"Plane angle {angle_with_camera_z_axis_deg:.1f}° wrt Z-axis exceeds max_surface_angle {self.max_surface_angle}°.")
                return
            
            center_3d = np.mean(plane_points, axis=0)
            self.get_logger().info(f"DETECTED 3D PLANE: Center={center_3d}, Shape={best_shape}")
            
            min_x, min_y, _ = np.min(plane_points, axis=0)
            max_x, max_y, _ = np.max(plane_points, axis=0)
            avg_z_plane = center_3d[2] 
            boundary_3d_for_marker = [
                (min_x, min_y, avg_z_plane), (max_x, min_y, avg_z_plane),
                (max_x, max_y, avg_z_plane), (min_x, max_y, avg_z_plane)
            ]
            area_m2_placeholder = (max_x - min_x) * (max_y - min_y) 

            plane_info = {
                'boundary': boundary_3d_for_marker, 
                'center': center_3d,
                'normal': normal_unit,
                'area': area_m2_placeholder, 
                'shape_type': best_shape
            }
            
            self.publish_surface(plane_info, debug_image) 
        
        except Exception as e:
            self.get_logger().error(f"Error in surface detection: {str(e)}", exc_info=True)
    
    def publish_surface(self, plane, image_to_publish):
        polygon_msg = PolygonStamped()
        polygon_msg.header = self.latest_image_msg.header 
        
        for point_coords in plane['boundary']:
            p = Point32()
            p.x = float(point_coords[0])
            p.y = float(point_coords[1])
            p.z = float(point_coords[2])
            polygon_msg.polygon.points.append(p)
        
        self.surface_pub.publish(polygon_msg)
        
        type_msg = String()
        type_msg.data = plane['shape_type']
        self.shape_type_pub.publish(type_msg)
        
        marker = Marker()
        marker.header = self.latest_image_msg.header 
        marker.ns = "cake_surfaces"
        marker.id = 0
        marker.type = Marker.LINE_STRIP 
        marker.action = Marker.ADD
        
        marker.color.r = 0.0 
        marker.color.g = 1.0 
        marker.color.b = 0.0
        marker.color.a = 0.9
        marker.scale.x = 0.02 

        for point_coords in plane['boundary']:
            marker.points.append(self.create_ros_point(point_coords[0], point_coords[1], point_coords[2]))
        if len(plane['boundary']) > 1: 
             marker.points.append(self.create_ros_point(plane['boundary'][0][0], plane['boundary'][0][1], plane['boundary'][0][2]))
        
        self.marker_pub.publish(marker)
    
    def create_ros_point(self, x, y, z): 
        from geometry_msgs.msg import Point as RosPoint 
        p = RosPoint()
        p.x = float(x)
        p.y = float(y)
        p.z = float(z)
        return p
    
    def cleanup(self):
        self.get_logger().info("Vision node cleaning up.")
    
    def __del__(self):
        self.cleanup()

def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down vision_node due to KeyboardInterrupt")
    finally:
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
