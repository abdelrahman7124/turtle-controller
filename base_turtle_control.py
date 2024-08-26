import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import Kill, Spawn
from math import sqrt, atan2
from random import uniform

class BaseTurtleControl(Node):

    def __init__(self):
        super().__init__('base_turtle_control')

        self.base_turtle_pose_sub = self.create_subscription(
            Pose, '/turtle1/pose', self.base_turtle_pose_callback, 10)

        self.target_turtle_pose_sub = self.create_subscription(
            Pose, '/turtle2/pose', self.target_turtle_pose_callback, 10)

        self.vel_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        self.base_turtle_pose = None
        self.target_turtle_pose = None

        
        self.kill_client = self.create_client(Kill, 'kill')
        self.spawn_client = self.create_client(Spawn, 'spawn')
        self.spawn_new_turtle()

        
        self.control_timer = self.create_timer(0.1, self.control_loop)

    def base_turtle_pose_callback(self, msg):
        self.base_turtle_pose = msg

    def target_turtle_pose_callback(self, msg):
        self.target_turtle_pose = msg

    def control_loop(self):

        distance = sqrt(
            (self.target_turtle_pose.x - self.base_turtle_pose.x) ** 2 +
            (self.target_turtle_pose.y - self.base_turtle_pose.y) ** 2
        )

        if distance < 0.5:
            self.kill_turtle()
        else:
            self.move_towards_target()

    def move_towards_target(self):
        msg = Twist()
        angle_to_target = atan2(
            self.target_turtle_pose.y - self.base_turtle_pose.y,
            self.target_turtle_pose.x - self.base_turtle_pose.x
        )

        angle_diff = angle_to_target - self.base_turtle_pose.theta

        if abs(angle_diff) > 0.1:
            msg.angular.z = 2.0 * angle_diff
        else:
            msg.linear.x = 2.0

        self.vel_pub.publish(msg)

    def kill_turtle(self):
        kill_request = Kill.Request()
        kill_request.name = 'turtle2'

        if self.kill_client.service_is_ready():
            future = self.kill_client.call_async(kill_request)
            future.add_done_callback(self.turtle_killed)

    def turtle_killed(self, future):
        try:
            future.result()
            self.get_logger().info("Killed turtle2 successfully")
            self.spawn_new_turtle()
        except Exception as e:
            self.get_logger().error(f"Failed to kill turtle2: {e}")

    def spawn_new_turtle(self):
        spawn_request = Spawn.Request()
        spawn_request.name = 'turtle2'
        spawn_request.x = uniform(1.0, 10.0)
        spawn_request.y = uniform(1.0, 10.0)
        spawn_request.theta = 0.0

        if self.spawn_client.service_is_ready():
            future = self.spawn_client.call_async(spawn_request)
            future.add_done_callback(self.turtle_spawned)
        else:
            self.get_logger().error("Spawn service is not ready!")

    def turtle_spawned(self, future):
        try:
            future.result()
            self.get_logger().info("Spawned turtle2 successfully")
        except Exception as e:
            self.get_logger().error(f"Failed to spawn turtle2: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = BaseTurtleControl()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
