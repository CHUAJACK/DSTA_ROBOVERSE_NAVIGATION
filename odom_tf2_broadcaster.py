import rclpy
from rclpy.node import Node
import tf2_ros
from geometry_msgs.msg import TransformStamped
import asyncio
import threading

class OdomTFBroadcaster(Node):
    def __init__(self):
        super().__init__('px4_tf_broadcaster')
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

    def publish_tf(self, position, attitude):
        """Call when new position/attitude is obtained from MAVSDK."""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'base_link'

        # NED → ROS (ENU): x=east=y_ned, y=north=x_ned, z=up=-z_ned
        t.transform.translation.x = float(position.east_m)
        t.transform.translation.y = float(position.north_m)
        t.transform.translation.z = float(-position.down_m)

        # Yaw: NED yaw → ENU yaw
        import math
        from tf_transformations import quaternion_from_euler
        yaw_enu = -math.radians(attitude.yaw_deg) + math.pi / 2
        q = quaternion_from_euler(0.0, 0.0, yaw_enu)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self.tf_broadcaster.sendTransform(t)