import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class TurtleSquare(Node):
    def __init__(self):
        super().__init__('turtle_square')
        self.publisher = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)
        self.timer = self.create_timer(0.5, self.move_square)
        self.step = 0
        self.turning = False
        self.turn_start = None

    def move_square(self):
        msg = Twist()

        if not self.turning:
            # Move forward
            msg.linear.x = 2.0
            self.publisher.publish(msg)
            self.step += 1
            if self.step % 10 == 0:  # after ~5 seconds forward
                self.turning = True
                self.turn_start = time.time()
        else:
            # Turn 90 degrees
            msg.angular.z = 1.57  # ~90 deg/sec
            self.publisher.publish(msg)
            if time.time() - self.turn_start > 1.0:  # turn for ~1 sec
                self.turning = False

def main(args=None):
    rclpy.init(args=args)
    node = TurtleSquare()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
