import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from std_msgs.msg import Float64


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_acceleration_ = self.create_publisher(Float64, 'acceleration', 10)
        timer_period = 0.01  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.x = 0.0
        self.y = 0.0
        self.orientation = 0.0
        self.v = 0.0

        # Feedback gains:
        self.K_x = 0.5
        self.K_y = 0.5
        self.K_orient = 0.5
        self.K_v = 0.5

        # Reference points
        self.x_des = 5
        self.y_des = 5
        self.orient_des = 5
        self.v_des = 0.0 # in the goal point, vecolit should be zero

        # Subscriber
        self.subscription_x = self.create_subscription(
            Float64,
            'x_position',
            self.callback_x,
            10)
        self.subscription_x  # prevent unused variable warning

    def callback_x(self, msg):
        self.x = msg.data

    # Repeat for y, orientation and velocity
        
    def timer_callback(self):
        error_x = self.x - self.x_des
         
        msg_a = Float64()
        msg_a.data = -self.K_x*(error_x) - self.K_v*self.v
        self.publisher_acceleration_.publish(msg_a)

        # Repreat it for the steering


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()