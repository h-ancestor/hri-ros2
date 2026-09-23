#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import math

from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from builtin_interfaces.msg import Duration


class UR3eDrawHDirectNode(Node):

    def __init__(self):
        super().__init__('draw_h_node')


        self.traj_pub = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            10
        )

        

        self.marker_pub = self.create_publisher(
            Marker,
            '/visualization_marker',
            10
        )

       

        self.joint_names = [
            'shoulder_pan_joint',
            'shoulder_lift_joint',
            'elbow_joint',
            'wrist_1_joint',
            'wrist_2_joint',
            'wrist_3_joint'
        ]

        

        self.z_draw = 0.12
        self.z_lift = 0.20

        self.x_start = 0.30
        self.x_end = 0.45
        self.x_mid = 0.375

        self.y_left = -0.04
        self.y_right = 0.04

        

        self.cartesian_points = [

            

            (self.x_start, self.y_left, self.z_lift),

            (self.x_start, self.y_left, self.z_draw),

            (self.x_end, self.y_left, self.z_draw),

            (self.x_end, self.y_left, self.z_lift),

          

            (self.x_start, self.y_right, self.z_lift),

            (self.x_start, self.y_right, self.z_draw),

            (self.x_end, self.y_right, self.z_draw),

            (self.x_end, self.y_right, self.z_lift),

            

            (self.x_mid, self.y_left, self.z_lift),

            (self.x_mid, self.y_left, self.z_draw),

            (self.x_mid, self.y_right, self.z_draw),

            (self.x_mid, self.y_right, self.z_lift),
        ]

       

        self.joint_waypoints = []

        for x, y, z in self.cartesian_points:

            angles = self.ur3e_ik(x, y, z)

            self.joint_waypoints.append(angles)

     

        self.current_waypoint = 0

        self.waypoint_timer = None

        self.started = False

        # Mỗi điểm robot có khoảng 1.5 giây để di chuyển
        self.move_time = 1.5

       

     

        self.marker_center_x = 0.504
        self.marker_center_y = 0.107

        self.marker_z = 0.050

        

        self.marker_half_width = 0.125
        self.marker_half_height = 0.080

        self.marker_x_start = (
            self.marker_center_x
            - self.marker_half_width
        )

        self.marker_x_end = (
            self.marker_center_x
            + self.marker_half_width
        )

        self.marker_x_mid = self.marker_center_x

        self.marker_y_left = (
            self.marker_center_y
            - self.marker_half_height
        )

        self.marker_y_right = (
            self.marker_center_y
            + self.marker_half_height
        )

      

        self.get_logger().info(
            "=== CHỜ 10 GIÂY ĐỂ BẮT ĐẦU ==="
        )

        self.start_timer = self.create_timer(
            10.0,
            self.start_drawing
        )

    

    def ur3e_ik(self, x, y, z):

        d1 = 0.15185
        a2 = 0.24390
        a3 = 0.21320
        d6 = 0.09210

        theta1 = math.atan2(y, x)

        r = math.sqrt(
            x ** 2 + y ** 2
        )

        rw = r

        zw = z + d6 - d1

        D = (
            rw ** 2
            + zw ** 2
            - a2 ** 2
            - a3 ** 2
        ) / (
            2 * a2 * a3
        )

        D = max(
            -1.0,
            min(1.0, D)
        )

        theta3 = math.atan2(
            -math.sqrt(1 - D ** 2),
            D
        )

        theta2 = (
            math.atan2(zw, rw)
            -
            math.atan2(
                a3 * math.sin(theta3),
                a2 + a3 * math.cos(theta3)
            )
        )

        theta4 = (
            -math.pi / 2.0
            - theta2
            - theta3
        )

        theta5 = -math.pi / 2.0

        theta6 = 0.0

        return [
            theta1,
            theta2,
            theta3,
            theta4,
            theta5,
            theta6
        ]


    def start_drawing(self):

        self.start_timer.cancel()

        self.started = True

        self.current_waypoint = 0

        self.get_logger().info(
            "=== BẮT ĐẦU DI CHUYỂN UR3e ==="
        )

        self.get_logger().info(
            "Robot sẽ di chuyển từng waypoint."
        )

        # Gửi điểm đầu tiên ngay
        self.send_current_waypoint()

        # Cứ 1.6 giây gửi điểm tiếp theo
        self.waypoint_timer = self.create_timer(
            1.6,
            self.next_waypoint
        )


    def send_current_waypoint(self):

        if self.current_waypoint >= len(
            self.joint_waypoints
        ):
            return

        angles = self.joint_waypoints[
            self.current_waypoint
        ]

        traj = JointTrajectory()

        traj.joint_names = self.joint_names

        point = JointTrajectoryPoint()

        point.positions = angles

        # Robot có 1.5 giây để tới điểm này

        point.time_from_start = Duration(
            sec=1,
            nanosec=500000000
        )

        traj.points.append(point)

        self.traj_pub.publish(traj)

        x, y, z = self.cartesian_points[
            self.current_waypoint
        ]

        self.get_logger().info(
            "Waypoint %d/%d -> "
            "x=%.3f y=%.3f z=%.3f"
            % (
                self.current_waypoint + 1,
                len(self.cartesian_points),
                x,
                y,
                z
            )
        )

  

    def next_waypoint(self):

        if not self.started:
            return

        self.current_waypoint += 1

        

        if self.current_waypoint >= len(
            self.joint_waypoints
        ):

            self.get_logger().info(
                "=== ROBOT ĐÃ HOÀN THÀNH QUỸ ĐẠO CHỮ H ==="
            )

            if self.waypoint_timer is not None:
                self.waypoint_timer.cancel()

            self.started = False

            return

        

        self.send_current_waypoint()


    def create_marker(self):

        marker = Marker()

        marker.header.frame_id = "base_link"

        marker.header.stamp = (
            self.get_clock().now().to_msg()
        )

        marker.ns = "letter_h"

        marker.id = 0

        marker.type = Marker.LINE_LIST

        marker.action = Marker.ADD

        marker.scale.x = 0.015

        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        return marker

  

    def publish_marker(self):

        marker = self.create_marker()


        if self.current_waypoint >= 2:

            marker.points.append(
                Point(
                    x=self.marker_x_start,
                    y=self.marker_y_left,
                    z=self.marker_z
                )
            )

            marker.points.append(
                Point(
                    x=self.marker_x_end,
                    y=self.marker_y_left,
                    z=self.marker_z
                )
            )

        

        if self.current_waypoint >= 6:

            marker.points.append(
                Point(
                    x=self.marker_x_start,
                    y=self.marker_y_right,
                    z=self.marker_z
                )
            )

            marker.points.append(
                Point(
                    x=self.marker_x_end,
                    y=self.marker_y_right,
                    z=self.marker_z
                )
            )

       

        if self.current_waypoint >= 10:

            marker.points.append(
                Point(
                    x=self.marker_x_mid,
                    y=self.marker_y_left,
                    z=self.marker_z
                )
            )

            marker.points.append(
                Point(
                    x=self.marker_x_mid,
                    y=self.marker_y_right,
                    z=self.marker_z
                )
            )

        self.marker_pub.publish(marker)




def main(args=None):

    rclpy.init(args=args)

    node = UR3eDrawHDirectNode()

    try:

        # Timer riêng để cập nhật Marker theo waypoint
        marker_timer = node.create_timer(
            0.1,
            node.publish_marker
        )

        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:

        node.destroy_node()

        rclpy.shutdown()


if __name__ == '__main__':
    main()
