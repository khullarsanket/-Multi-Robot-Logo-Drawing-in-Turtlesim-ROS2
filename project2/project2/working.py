import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import SetPen, Kill
import math
from rcl_interfaces.srv import SetParameters
from rclpy.parameter import Parameter, ParameterType
from std_srvs.srv import Empty
from rcl_interfaces.msg import Parameter, ParameterValue

class Turtle_GTG(Node):
    def __init__(self, goals: list):
        super().__init__("Go_to_Goal_Node")
        self.cmd_vel_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.pose_sub = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
        self.timer = self.create_timer(0.1, self.go_to_goal)
        self.pose = Pose()
        self.goals = goals
        self.current_goal_index = 0
        #self.clear_drawing_flag = False  # Flag to keep track of drawing cleared

    def pose_callback(self, data):
        self.pose = data

    def go_to_goal(self):
        if self.current_goal_index >= len(self.goals):
            self.get_logger().info("All goals reached")
            self.remove_turtle()
            quit()
            
        else:
            goal = self.goals[self.current_goal_index]

            

            new_vel = Twist()

            # Euclidean Distance
            distance_to_goal = math.sqrt((goal[0] - self.pose.x)**2 + (goal[1] - self.pose.y)**2)
            # Angle to Goal
            angle_to_goal = math.atan2(goal[1] - self.pose.y, goal[0] - self.pose.x)

            distance_tolerance = 0.01
            angle_tolerance = 0.001

            if self.current_goal_index == 0 or self.current_goal_index == 21 or self.current_goal_index == 25 or self.current_goal_index == 42:
                        self.set_pen_service_call(draw=1)
                    
            else: 
                self.set_pen_service_call(draw=0)

            angle_error = angle_to_goal - self.pose.theta
            kp_angle = 6
            kp_distance = 7
            if abs(angle_error) > angle_tolerance:
                new_vel.angular.z = kp_angle * angle_error
            else:
                if distance_to_goal >= distance_tolerance:
                    new_vel.linear.x = kp_distance * distance_to_goal
                else:
                    new_vel.linear.x = 0.0
                    self.get_logger().info(f"Goal {self.current_goal_index+1} reached")
                    
                    #if self.current_goal_index == 0 or self.current_goal_index == 20 or self.current_goal_index == 24 or self.current_goal_index == 41:
                    #    self.set_pen_service_call(draw=1)
                    
                    #else: 
                    #    self.set_pen_service_call(draw=0)
                    
                    self.current_goal_index += 1

            self.cmd_vel_pub.publish(new_vel)

    def remove_turtle(self):
        self.get_logger().info("Removing turtle from simulation screen")
        kill_client = self.create_client(Kill, "/kill")
        while not kill_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        request = Kill.Request()
        request.name = 'turtle1'
        kill_client.call_async(request)

    def set_pen_service_call(self, draw):
        pen_client = self.create_client(SetPen, "/turtle1/set_pen")
        while not pen_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

        request = SetPen.Request()
        request.r = 255
        request.g = 255
        request.b = 255
        request.width = 2
        request.off = draw  # Set pen to 'up'

        pen_client.call_async(request)

def set_background_color(node, r, g, b):
    """Set the background color of the turtlesim simulator."""
    
    param_client = node.create_client(SetParameters, '/turtlesim/set_parameters')
    while not param_client.wait_for_service(timeout_sec=1.0):
        print('SetParameters service not available, waiting again...')


    # Creating Parameter messages


    r_value = ParameterValue(type=ParameterType.PARAMETER_INTEGER, integer_value=r)
    g_value = ParameterValue(type=ParameterType.PARAMETER_INTEGER, integer_value=g)
    b_value = ParameterValue(type=ParameterType.PARAMETER_INTEGER, integer_value=b)


    r_param = Parameter(name='background_r', value=r_value)
    g_param = Parameter(name='background_g', value=g_value)
    b_param = Parameter(name='background_b', value=b_value)

    # Defining the background color parameters
    parameters = [r_param, g_param, b_param]
    
    request = SetParameters.Request()
    request.parameters = parameters
    
    future = param_client.call_async(request)
    rclpy.spin_until_future_complete(node, future)
    
    if future.result() is not None:
        print(f'Background color set to RGB({r}, {g}, {b})')
    else:
        print('Failed to set background color')

def reset_screen(self):
    ## reset screen service
    clear_client = self.create_client(Empty, '/reset')
    while not clear_client.wait_for_service(timeout_sec=1.0):
        self.get_logger().info('Reset service not available, waiting again...')
    request = Empty.Request()
    clear_client.call_async(request)


def main(args=None):
    rclpy.init(args=args)
    
    # Define your list of goals here
    goal_T = [(4.13,2.00,3.14),(4.13,3.29,3.14),(4.85,3.29,3.14),(4.85, 7.84, 3.14), (3.48, 7.84, 3.14), (3.48,7.11,3.14), (2.17,7.11,3.14), (2.17,9.01,3.14), (8.81,9.01,3.14), (8.81,7.11,3.14), (7.52,7.11,3.14), (7.52,7.84,3.14), (6.15,7.84, 3.14), (6.15, 3.29, 3.14), (6.88, 3.29, 3.14), (6.88, 2.00, 3.14), (4.13, 2.00, 3.14)]
    goal_A = [
    (1.22, 3.70, 3.14),
    (1.22, 4.29, 3.14),
    (1.54, 4.29, 3.14),
    (2.33, 6.13, 3.14),
    (2.19, 6.13, 3.14),
    (2.19, 6.72, 3.14),
    (3.46, 6.72, 3.14),
    (3.46, 6.13, 3.14),
    (3.33, 6.13, 3.14),
    (4.11, 4.29, 3.14),
    (4.43, 4.29, 3.14),
    (4.43, 3.70, 3.14),
    (3.28, 3.70, 3.14),
    (3.28, 4.29, 3.14),
    (3.45, 4.29, 3.14),
    (3.31, 4.60, 3.14),
    (2.36, 4.60, 3.14),
    (2.24, 4.29, 3.14),
    (2.39, 4.29, 3.14),
    (2.39, 3.70, 3.14),
    (1.22, 3.70, 3.14)
]
    
    goal_a = [
    (2.63, 5.19, 3.14),
    (2.83, 5.69, 3.14),
    (3.06, 5.19, 3.14),
    (2.63, 5.19, 3.14)
]

    goal_M = [
    (6.57, 3.70, 3.14),
    (6.57, 4.29, 3.14),
    (6.81, 4.29, 3.14),
    (6.81, 6.13, 3.14),
    (6.59, 6.13, 3.14),
    (6.59, 6.72, 3.14),
    (7.52, 6.72, 3.14),
    (8.17, 5.40, 3.14),
    (8.81, 6.72, 3.14),
    (9.78, 6.72, 3.14),
    (9.78, 6.13, 3.14),
    (9.54, 6.13, 3.14),
    (9.54, 4.29, 3.14),
    (9.78, 4.29, 3.14),
    (9.78, 3.70, 3.14),
    (8.71, 3.70, 3.14),
    (8.71, 4.29, 3.14),
    (8.93, 4.29, 3.14),
    (8.93, 5.58, 3.14),
    (8.17, 4.02, 3.14),
    (7.42, 5.57, 3.14),
    (7.42, 4.29, 3.14),
    (7.66, 4.29, 3.14),
    (7.66, 3.70, 3.14),
    (6.57, 3.70, 3.14)
]
    temp_node = rclpy.create_node('temp_node_for_resetting_screen')
    reset_screen(temp_node)
    temp_node.destroy_node()


    # Set the background color to your desired RGB values
    temp_node = rclpy.create_node('temp_node_for_background_color')
    temp_node.declare_parameter('background_r', rclpy.Parameter.Type.INTEGER) 
    temp_node.declare_parameter('background_g', rclpy.Parameter.Type.INTEGER) 
    temp_node.declare_parameter('background_b', rclpy.Parameter.Type.INTEGER)

    set_background_color(temp_node, 80, 0, 0)  # Replacing with the Aggie Maroon Values
    temp_node.destroy_node()

    goals = goal_A + goal_a + goal_T + goal_M
    minimal_publisher = Turtle_GTG(goals)
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()