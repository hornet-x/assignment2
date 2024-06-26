import pygame
import random
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import time


# global variables
time_interval = 0.05
display_offset = 40
display_scale = 30
tolerance = 0.2
display_size = (250, 560)
max_depth = 15
drag_coefficient = 3
gravity = 9.81
fluid_density = 41.6666666667 * (gravity+2) / gravity
start_time = time.time()
blue = (41, 128, 185)
green = (46, 204, 113)
red = (231, 76, 60)
screen = None
surface = None


class Motion:
    """Class representing the 1D motion of the vehicle"""

    def __init__(
            self,
            y=0, # y coord
            vy=0, # velocity
            drag_coeff=3,
            mass=1,
            fluid_density=997,
            gravity=9.81,
            height=0.6,
            width=0.2,
            dt=0.05, # time interval
            env_depth=10,
            thrust_limit=2):
        self.y = y
        self.vy = vy
        self.drag_coeff = drag_coeff
        self.mass = mass
        self.fluid_density = fluid_density
        self.g = gravity
        self.width = width
        self.height = height
        self.dt = dt
        self.env_constraints = (-5, env_depth)
        self.thrust_limit = thrust_limit
    
    @staticmethod
    def constrain(val, low, high):
        return min(high, max(low, val))

    def volume_submerged(self):
        return (self.height+min(self.y, 0)) * self.width**2
    
    def update(self, vertical_thrust=0):
        thrust = self.constrain(vertical_thrust,
                                -self.thrust_limit,
                                self.thrust_limit)
        # buoyant force, gravity, drag, F_net
        Fb = -self.fluid_density * self.g * self.volume_submerged()
        Fg = self.mass * self.g
        Fd = -.5 * (-1 if self.vy<0 else 1 if self.vy>1 else 0) * \
                self.drag_coeff * self.width**2 * self.vy**2
        F = Fb + Fg + Fd - thrust * self.mass + random.normalvariate(0, 0.1)
        # new v, y
        nv = self.vy + (F/self.mass * self.dt)
        ny = self.constrain(self.y + nv * self.dt, *self.env_constraints)
        nv *= (self.env_constraints[0] < ny) * (ny < self.env_constraints[1])
        self.vy = nv
        self.y = ny
    
    def get_pos(self):
        return self.y 
    

class Vehicle(Node):
    """ROS Node class representing the vehicle simulation"""

    def __init__(self):
        super().__init__("vehicle_simulation")
        self.setpoint = 5
        self.thrust = 0
        self.vehicle_width = 0.2
        self.vehicle_height = 0.6
        self.vehicle_mass = 1
        self.motion = Motion(
                y=0,
                vy=0,
                drag_coeff=drag_coefficient,
                mass=self.vehicle_mass,
                fluid_density=fluid_density,
                gravity=gravity,
                height=self.vehicle_height,
                width=self.vehicle_width,
                dt=time_interval,
                env_depth=max_depth,
                thrust_limit=10)
        self.pub_depth = self.create_publisher(
                Float64,
                'depth',
                10)
        self.pub_setpoint = self.create_publisher(
                Float64,
                'setpoint',
                10)
        self.sub_thrust = self.create_subscription(
                Float64,
                'thrust',
                self.vertical_callback,
                10)     
        # TODO SERVICE
        self.timer = self.create_timer(
                time_interval,
                self.timer_callback)
        self.count_within_threshold = 0
    
    def vertical_callback(self, msg):
        self.thrust = min(4, max(-4, msg.data))
        # self.get_logger().info(f"Vehicle thrust: {self.thrust}")

    def reached_goal(self):
        return abs(self.depth - self.setpoint) < tolerance

    def update(self):
        self.depth = self.motion.get_pos()
        # publish telemetry
        depth = Float64()
        depth.data = float(self.depth)
        self.pub_depth.publish(depth)
        setpoint = Float64()
        setpoint.data = float(self.setpoint)  
        self.pub_setpoint.publish(setpoint)
        if not self.reached_goal():
            self.count_within_threshold = 0
        else:
            self.count_within_threshold += 1
        if self.count_within_threshold == 40:
            self.get_logger().info(
                    f"Reached depth = {self.setpoint:.2f}m in " +
                    f"{time.time()-start_time:.2f} seconds")
        self.motion.update(self.thrust)
    
    def get_y(self, depth):
        """Get the y pos of a given depth in the window"""
        return depth * display_scale + display_offset
    
    def update_setpoint(self, depth):
        self.setpoint = depth

    def timer_callback(self):
        self.update()
        setpoint_line_colour = green if self.reached_goal() else red
        screen.fill((0, 0, 0))
        w, h = surface.get_width(), surface.get_height()
        pygame.draw.rect(
                screen, 
                blue, 
                (0, 
                self.get_y(0), 
                w, 
                self.get_y(max_depth+self.vehicle_height) - display_offset), 
                0)  # water
        pygame.draw.rect(
                screen, 
                (255, 255, 255),
                (0, 0, w, self.get_y(0)),
                0)  # water surface
        pygame.draw.rect(
                screen, 
                (230, 126, 34), 
                (w/2, 
                self.get_y(self.depth),
                self.vehicle_width * display_scale, 
                self.vehicle_height * display_scale), 
                0)  # vehicle
        pygame.draw.lines(
                screen, 
                setpoint_line_colour, 
                False,
                ((0, self.get_y(self.setpoint)), 
                (w, self.get_y(self.setpoint))), 
                3)  # line
        pygame.display.update()

        # user input
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                raise KeyboardInterrupt
            if event.type == pygame.MOUSEBUTTONUP:
                pos = pygame.mouse.get_pos()
                self.update_setpoint(
                        Motion.constrain(
                                (pos[1] - display_offset) / display_scale, 
                                0, 
                                max_depth))
                global start_time
                start_time = time.time()
                self.get_logger().info(f"New setpoint: {self.setpoint:.2f}m")
                
    

def main(args=None):
    global screen
    global surface
    rclpy.init(args=args)
    pygame.init()
    screen = pygame.display.set_mode(display_size)
    surface = pygame.display.get_surface()
    pygame.display.set_caption("Vehicle Simulation")
    vehicle = Vehicle()
    try:
        start_time = time.time()
        rclpy.spin(vehicle)
    except KeyboardInterrupt:
        pass
    pygame.quit()

if __name__ == "__main__":
    main()