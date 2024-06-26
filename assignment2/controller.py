import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64

class PID_Controller(Node):

    def __init__(self):
        super().__init__("pid_controller")
        
        self.setpoint = 0
        self.depth = 0
        
        self.pub = self.create_publisher(Float64, 'thrust', 10)
        self.sub_setpoint = self.create_subscription(
                Float64,
                'setpoint',
                self.setpoint_callback,
                10)
        self.sub_depth = self.create_subscription(
                Float64,
                'depth',
                self.depth_callback,
                10)
        
        # Change your coefficients here
        self.KP = 0
        self.KI = 0
        self.KD = 0
        self.bias = 0 # Is a bias necessary?

        # Add more variables that you need here
        # e.g. self.past_error = 0

        self.timer_period = 0.05
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
    
    def setpoint_callback(self, msg):
        self.setpoint = msg.data

    def depth_callback(self, msg):
        self.depth = msg.data 

    def timer_callback(self):
        # Add any calculations you may need here
        # e.g. self.past_error = ?

        # Replace the following accordingly
        P = 0 * self.KP
        I = 0 * self.KI
        D = 0 * self.KD
        
        # Add any calculations you may need here
        # e.g. self.past_error = ?
        
        # We set thrust to 0 if moving upwards, letting buoyancy do the work
        thrust = Float64() 
        thrust.data = 0.0 # Add the PID equation (including bias if present) here
        self.pub.publish(thrust)
        self.get_logger().info(f"Publishing thrust: {thrust.data}")

def main(args=None):
    rclpy.init(args=args)
    controller = PID_Controller()
    rclpy.spin(controller)

if __name__ == "__main__":
    main()
