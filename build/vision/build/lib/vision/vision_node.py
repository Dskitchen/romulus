#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, Image
from geometry_msgs.msg import PolygonStamped, Point32
from std_msgs.msg import String
from visualization_msgs.msg import Marker

from cv_bridge import CvBridge
import numpy as np
import pyzed.sl as sl
import cv2
import math

class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')
        
        # Parameters
        self.declare_parameter('max_surface_angle', 60.0)
        self.declare_parameter('min_surface_area', 0.002)
        self.declare_parameter('max_surface_area', 1.5)
        self.declare_parameter('plane_distance_threshold', 0.03)
        self.declare_parameter('camera_resolution', 'SVGA')
        
        self.max_surface_angle = self.get_parameter('max_surface_angle').value
        self.min_surface_area = self.get_parameter('min_surface_area').value
        self.max_surface_area = self.get_parameter('max_surface_area').value
        self.plane_distance_threshold = self.get_parameter('plane_distance_threshold').value
        camera_resolution = self.get_parameter('camera_resolution').value
        
        # Initialize ZED camera
        self.zed = sl.Camera()
        init_params = sl.InitParameters()
        init_params.camera_resolution = (
            sl.RESOLUTION.SVGA if camera_resolution == 'SVGA' else sl.RESOLUTION.HD720
        )
        init_params.depth_mode = sl.DEPTH_MODE.ULTRA
        init_params.coordinate_units = sl.UNIT.METER
        init_params.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Y_UP
        init_params.depth_maximum_distance = 3.0
        init_params.depth_minimum_distance = 0.3
        init_params.input_type = sl.INPUT_TYPE.GMSL  # Explicitly set for GMSL
        
        status = self.zed.open(init_params)
        if status != sl.ERROR_CODE.SUCCESS:
            self.get_logger().error(f"Failed to open ZED camera: {status}")
            raise RuntimeError("ZED camera initialization failed")
        
        tracking_params = sl.PositionalTrackingParameters()
        status = self.zed.enable_positional_tracking(tracking_params)
        if status != sl.ERROR_CODE.SUCCESS:
            self.get_logger().warn(f"Positional tracking failed: {status}")
        
        self.runtime_params = sl.RuntimeParameters()
        self.runtime_params.confidence_threshold = 30  # Lower for less filtering
        
        self.bridge = CvBridge()
        
        self.surface_pub = self.create_publisher(PolygonStamped, '/cake_vision/surface', 10)
        self.shape_type_pub = self.create_publisher(String, '/cake_vision/shape_type', 10)
        self.marker_pub = self.create_publisher(Marker, '/cake_vision/surface_marker', 10)
        self.image_pub = self.create_publisher(Image, '/cake_vision/processed_image', 10)
        
        self.timer = self.create_timer(0.1, self.detect_surfaces)
        
        self.get_logger().info("Vision Node initialized")
    
    def detect_surfaces(self):
        if self.zed.grab(self.runtime_params) != sl.ERROR_CODE.SUCCESS:
            self.get_logger().warn("Failed to grab ZED frame")
            return
        
        point_cloud = sl.Mat()
        self.zed.retrieve_measure(point_cloud, sl.MEASURE.XYZRGBA)
        rgb_image = sl.Mat()
        self.zed.retrieve_image(rgb_image, sl.VIEW.LEFT)
        
        display_img = rgb_image.get_data()
        display_img = cv2.cvtColor(display_img, cv2.COLOR_RGBA2BGR)
        
        planes = self.detect_horizontal_planes(point_cloud)
        
        self.get_logger().info(f"Detected {len(planes)} planes")
        if planes:
            planes.sort(key=lambda p: p['area'], reverse=True)
            for i, plane in enumerate(planes):
                self.get_logger().debug(
                    f"Plane {i}: area={plane['area']:.4f}m², shape={plane['shape_type']}, "
                    f"angle={math.degrees(math.acos(abs(plane['normal'][1]))):.1f}°"
                )
                if self.min_surface_area <= plane['area'] <= self.max_surface_area:
                    self.publish_surface(plane, display_img)
                    self.get_logger().info(
                        f"Selected plane {i}: area={plane['area']:.4f}m², shape={plane['shape_type']}"
                    )
                    break
            else:
                self.get_logger().info("No surfaces within area constraints")
        else:
            self.get_logger().info("No planes detected")
        
        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(display_img, "bgr8"))
        except Exception as e:
            self.get_logger().error(f"Error publishing image: {str(e)}")
    
    def detect_horizontal_planes(self, point_cloud):
        plane_params = sl.PlaneDetectionParameters()
        plane_params.max_distance_threshold = self.plane_distance_threshold
        
        zed_planes = sl.Planes()
        self.zed.find_planes(zed_planes, plane_params)
        
        detected_planes = []
        for plane in zed_planes.plane_list:
            normal = plane.get_normal()
            angle_rad = math.acos(abs(normal[1]))
            angle_deg = math.degrees(angle_rad)
            
            if angle_deg <= self.max_surface_angle:
                boundary = plane.get_bounds()
                boundary_points = [(p[0], p[1], p[2]) for p in boundary]
                
                area = plane.get_extents()[0] * plane.get_extents()[1]
                shape_type = self.determine_shape_type(boundary_points)
                
                detected_planes.append({
                    'boundary': boundary_points,
                    'center': plane.get_center(),
                    'normal': normal,
                    'area': area,
                    'shape_type': shape_type
                })
        
        return detected_planes
    
    def determine_shape_type(self, boundary):
        if len(boundary) < 3:
            self.get_logger().warn("Insufficient boundary points for shape classification")
            return "unknown"
        
        points_2d = np.array([(p[0], p[2]) for p in boundary])
        contour = points_2d.reshape(-1, 1, 2).astype(np.float32)
        
        perimeter = cv2.arcLength(contour, True)
        area = cv2.contourArea(contour)
        
        if area == 0:
            self.get_logger().warn("Zero contour area")
            return "unknown"
        
        circularity = 4 * math.pi * area / (perimeter * perimeter)
        
        if len(contour) >= 5:
            ellipse = cv2.fitEllipse(contour)
            ellipse_ratio = min(ellipse[1][0], ellipse[1][1]) / max(ellipse[1][0], ellipse[1][1])
        else:
            ellipse_ratio = 0
            self.get_logger().debug("Not enough points for ellipse fitting")
        
        rect = cv2.minAreaRect(contour)
        rect_area = rect[1][0] * rect[1][1]
        rect_ratio = min(rect[1][0], rect[1][1]) / max(rect[1][0], rect[1][1])
        
        self.get_logger().debug(
            f"Shape analysis: circularity={circularity:.2f}, ellipse_ratio={ellipse_ratio:.2f}, "
            f"rect_ratio={rect_ratio:.2f}, area_ratio={area/rect_area if rect_area else 0:.2f}"
        )
        
        if circularity > 0.7 and ellipse_ratio > 0.7:
            return "cylinder"
        elif rect_area and area / rect_area > 0.75 and rect_ratio < 0.8:
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
            camera_point = sl.Translation()
            camera_point.init_vector(point[0], point[1], point[2])
            pixel = self.zed.convert_3d_point_to_2d(camera_point)
            points_2d.append([int(pixel[0]), int(pixel[1])])
        
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
    
    def create_point(self, x, y, z):
        from geometry_msgs.msg import Point
        p = Point()
        p.x = float(x)
        p.y = float(y)
        p.z = float(z)
        return p
    
    def cleanup(self):
        self.zed.disable_positional_tracking()
        self.zed.close()
    
    def __del__(self):
        self.cleanup()

def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
