import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2

class LidarInspector(Node):
    def __init__(self):
        super().__init__("lidar_inspector")
        self.sub = self.create_subscription(PointCloud2, "/lidar_points", self.cb, 10)
        self.received = False

    def cb(self, msg):
        if not self.received:
            self.received = True
            self.get_logger().info("Frame ID: %s" % msg.header.frame_id)
            self.get_logger().info("Width: %d, Height: %d" % (msg.width, msg.height))
            self.get_logger().info("Points per frame: %d" % (msg.width * msg.height))
            self.get_logger().info("Fields: %s" % [f.name for f in msg.fields])
            raise SystemExit

def main():
    rclpy.init()
    node = LidarInspector()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
