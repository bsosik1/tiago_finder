from nav2_simple_commander.costmap_2d import PyCostmap2D
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class Cov_Nav(Node):
    def __init__(self):
        super().__init__('cov_nav_node')
        self.publisher = self.create_publisher(String, 'costmap_cell', 10)
        self.subscriber = self.create_subscription(OccupancyGrid,
                                                   '/global_costmap/costmap',
                                                   self.subscriber_callback,
                                                   10)
        self.timer = self.create_timer(1, self.timer_callback)
        self.basic_navigator = BasicNavigator()
        self.occupancy_grid = OccupancyGrid()
        self.costmap = None

    def timer_callback(self):
        msg = String()
        msg.data = "cos cos cos"
        self.publisher.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)

    def subscriber_callback(self, msg):
        self.costmap = PyCostmap2D(msg)
        # Test 1
        X = self.costmap.getSizeInCellsX()
        Y = self.costmap.getSizeInCellsY()
        log = "X: " + str(X) + " Y: " + str(Y)
        # Test 2
        X_meters = self.costmap.getSizeInMetersX()
        log2 = "Size X [m]: " + str(X_meters)
        # Test 3
        X_origin = self.costmap.getOriginX()
        Y_origin = self.costmap.getOriginY()
        log3 = "Orig_X: " + str(X_origin) + " Orig_Y: " + str(Y_origin)
        # Test 4
        # mx, my = self.costmap.worldToMap(-0.3, -4.7)
        # log4 = "mx: " + str(mx) + " my: " + str(my)

        self.get_logger().info(log)
        self.get_logger().info(log2)
        self.get_logger().warn(log3)
        # self.get_logger().info(log4)

    def set_cost(self, mx, my, cost):
        self.costmap.setCost(mx, my, cost)

    def go_to_point(self):
        initial_pose = PoseStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.header.stamp = self.basic_navigator.get_clock().now().to_msg()
        initial_pose.pose.orientation.w = 1.0
        initial_pose.pose.orientation.x = 0.0
        initial_pose.pose.orientation.y = 0.0
        initial_pose.pose.orientation.z = 0.0
        initial_pose.pose.position.x = 1.0
        initial_pose.pose.position.y = 1.0
        initial_pose.pose.position.z = 0.0
        self.basic_navigator.setInitialPose(initial_pose)
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.basic_navigator.get_clock().now().to_msg()
        goal_pose.pose.orientation.w = 1.0
        goal_pose.pose.orientation.x = 0.0
        goal_pose.pose.orientation.y = 0.0
        goal_pose.pose.orientation.z = 0.0
        goal_pose.pose.position.x = 1.0
        goal_pose.pose.position.y = 1.0
        goal_pose.pose.position.z = 0.0

        self.basic_navigator.goToPose(goal_pose)

        if self.basic_navigator.isTaskComplete():
            result = self.basic_navigator.getResult()
            if result == TaskResult.SUCCEEDED:
                print('Goal succeeded!')
            if result == TaskResult.CANCELED:
                print('Goal was canceled!')
            if result == TaskResult.FAILED:
                print('Goal failed!')


def main(args=None):
    rclpy.init(args=None)
    cov_nav = Cov_Nav()
    cov_nav.go_to_point()
    rclpy.spin(cov_nav)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
