import rclpy
import rclpy.node
import rclpy.qos
import tf2_ros
from geometry_msgs.msg import PoseStamped

class AprilTagDockDetector(rclpy.node.Node):
    def __init__(self):
        super().__init__("apriltag_dock_detector")

        # Initialize TF2 buffer and listener.
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Publish the pose of the detected dock
        self.pub = self.create_publisher(
            PoseStamped, "detected_dock_pose", 10
        )

        self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        try:
            # Get the transform from the camera frame to the dock frame.
            transform = self.tf_buffer.lookup_transform(
                "odom", "detected_dock_link", rclpy.time.Time(), rclpy.duration.Duration(seconds=0.1)
            )
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            return

        # Publish the pose of the detected dock.
        pose = PoseStamped()
        pose.header.frame_id = "odom"
        pose.header.stamp = transform.header.stamp
        pose.pose.position.x = transform.transform.translation.x
        pose.pose.position.y = transform.transform.translation.y
        pose.pose.position.z = transform.transform.translation.z
        pose.pose.orientation = transform.transform.rotation
        self.pub.publish(pose)

def main():
    try:
        rclpy.init()
        node = AprilTagDockDetector()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass
