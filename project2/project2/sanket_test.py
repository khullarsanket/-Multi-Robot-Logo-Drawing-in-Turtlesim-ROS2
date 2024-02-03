import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from std_srvs.srv import Empty  # Import Empty service
from turtlesim.srv import SetPen, Kill, Spawn
import math
from rcl_interfaces.srv import SetParameters
from rclpy.parameter import Parameter, ParameterType
from std_srvs.srv import Empty
from rcl_interfaces.msg import Parameter, ParameterValue


class Turtle_GTG(Node):
    def __init__(self,turtle_name, goals: list, start_index):


        # super().__init__(f"{turtle_name}_Go_to_Goal_Node")
        super().__init__(f"turtle_Go_to_Goal_Node")

        self.turtle_name = turtle_name
        self.goals = goals
        self.global_goal_index = start_index

        self.cumulative_linear_error = 0.0
        self.previous_linear_error = 0.0

        self.cumulative_angular_error = 0.0 ## for the integral error
        self.previous_angular_error = 0.0 ## for the rate of change of error


        self.cmd_vel_pub = self.create_publisher(Twist, f'/{turtle_name}/cmd_vel', 10)
        self.pose_sub = self.create_subscription(Pose, f'/{turtle_name}/pose', self.pose_callback, 10)


        self.timer = self.create_timer(0.1, self.go_to_goal)
        self.pose = Pose()
        self.current_goal_index = 0


    def pose_callback(self, data):
        self.pose = data

    def go_to_goal(self):
        ## method to make the turtle move the goal position

        if self.current_goal_index >= len(self.goals):
            self.get_logger().info("All goals reached")
            self.remove_turtle()
            quit()
            
        else:
            goal = self.goals[self.current_goal_index]
            
            print(f"{self.turtle_name} - Current Pose: {self.pose.x}, {self.pose.y}, {self.pose.theta}")
            print(f"{self.turtle_name} - Target Goal: {goal[0]}, {goal[1]}")
            

            new_vel = Twist()

            # Euclidean Distance to the Goal
            distance_to_goal = math.sqrt((goal[0] - self.pose.x)**2 + (goal[1] - self.pose.y)**2)

            # Angle to Goal
            angle_to_goal = math.atan2(goal[1] - self.pose.y, goal[0] - self.pose.x)
            

            distance_tolerance = 0.01
            angle_tolerance = 0.001

            ## defining the points where we wanna reach without drawing on the console (these indexes are  the global goal indexes)
            if self.current_goal_index ==0 or self.global_goal_index == 0 or self.global_goal_index == 21 or self.global_goal_index == 25 or self.global_goal_index == 42:
                        self.set_pen_service_call(draw=1)
                    
            else: 
                self.set_pen_service_call(draw=0)

            angle_error = (angle_to_goal - self.pose.theta)

            while angle_error > math.pi:
                angle_error -= 2*math.pi
            while angle_error < -math.pi:
                angle_error += 2*math.pi
            
            print(f"{angle_error = }")

            # Linear PID Controller
            kp_distance = 5.2 #proportional gain needs to be tuned
            ki_distance = 0.0
            kd_distance = 0.0
            
            self.cumulative_linear_error += distance_to_goal
            differential_linear_error = distance_to_goal - self.previous_linear_error
            
            linear_control_effort = (kp_distance * distance_to_goal) + \
                                    (ki_distance * self.cumulative_linear_error*0.1) + \
                                    (kd_distance * differential_linear_error)/0.1

            self.previous_linear_error = distance_to_goal
            
            

            # Angular PID Controller
            
            kp_angle = 5.1
            ki_angle = 0.0
            kd_angle = 0.0
            
            self.cumulative_angular_error += angle_error
            differential_angular_error = angle_error - self.previous_angular_error
            
            angular_control_effort = (kp_angle * angle_error) + \
                                    (ki_angle * self.cumulative_angular_error * 0.1) + \
                                    (kd_angle * (differential_angular_error)/0.1)
                                    
            self.previous_angular_error = angle_error
            
            # print(f"{self.turtle_name = }{angle_to_goal = }")
            # print(f"{self.turtle_name = }{distance_to_goal = }")
            # print(f"{self.turtle_name = }{angle_error = }")
            # print(f"{self.turtle_name = }{angular_control_effort = }")
            # print(f"{self.turtle_name = }{linear_control_effort = }")
             
        # Condition to decide when to move forward even if angle isn't perfectly aligned
        #     MOVE_FORWARD_ANGLE_TOLERANCE = 0.1  # Adjust as necessary
        #     MAX_ANGULAR_VELOCITY = 2.0  # This can be adjusted based on your observation
        #   # Reduce angular speed as the turtle aligns more with the target
        #     if abs(angle_error) < MOVE_FORWARD_ANGLE_TOLERANCE:
        #         new_vel.linear.x = linear_control_effort
        #         angular_control_effort *= (abs(angle_error) / MOVE_FORWARD_ANGLE_TOLERANCE)  # Non-linear scaling factor

            if abs(angle_error) > angle_tolerance:
                new_vel.angular.z = angular_control_effort
            else:
                if distance_to_goal > distance_tolerance:
                    new_vel.linear.x = linear_control_effort
                    new_vel.angular.z = 0.0

                else:
                    new_vel.linear.x = 0.0
                    self.get_logger().info(f"Goal reached")
                    self.current_goal_index += 1
                    self.global_goal_index +=1
           
            # if self.prev_theta == self.pose.theta:
            #     new_vel.angular.z = 0.0

            if abs(angle_error) > angle_tolerance and distance_to_goal < distance_tolerance:
                    new_vel.linear.x = 0.0
                    self.get_logger().info(f"Goal reached")
                    self.current_goal_index += 1
                    self.global_goal_index +=1
            

            self.cmd_vel_pub.publish(new_vel)
            # self.prev_theta = self.pose.theta


    def remove_turtle(self):

        ## service called to remove the turtle when the goal is completed 
        self.get_logger().info(f"Removing {self.turtle_name} from simulation screen")
        kill_client = self.create_client(Kill, "/kill")
        while not kill_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        request = Kill.Request()
        request.name = self.turtle_name
        kill_client.call_async(request)

    def set_pen_service_call(self, draw):
        ## service to make the pen go up and down on mentioned global_index_points during the service call
        pen_client = self.create_client(SetPen, f"/{self.turtle_name}/set_pen")
        while not pen_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

        request = SetPen.Request()
        request.r = 255
        request.g = 255
        request.b = 255
        request.width = 3
        request.off = draw  # Set pen to 'up'

        pen_client.call_async(request)



def spawn_turtle(turtle_name):
    ## spawning service
    node = rclpy.create_node('spawn_node')  # Temporary node for the spawning service
    spawn_client = node.create_client(Spawn, "/spawn")
    
    while not spawn_client.wait_for_service(timeout_sec=1.0):
        print('Spawn service not available, waiting again...')
    
    request = Spawn.Request()
    request.x = 5.544445  # Default spawn position
    request.y = 5.544445
    request.theta = 0.0
    request.name = turtle_name
    
    future = spawn_client.call_async(request)
    rclpy.spin_until_future_complete(node, future)  # Wait until turtle is spawned
    
    if future.result() is not None:
        print(f'{turtle_name} spawned successfully')
    else:
        print(f'Failed to spawn {turtle_name}')

    node.destroy_node()  # Destroy the temporary node

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
    
    ## To take in the num_turtles as arguments 
    node = rclpy.create_node('temp_node_for_param_retrieval')
    node.declare_parameter("num_turtles", 1)
    num_turtles = node.get_parameter("num_turtles").value
    node.destroy_node()

    ## To reset the screen
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




    # Defining the list of global goal coordinates here (found using gimp image editor on Linux)
    goal_T = [ (4.13,2.00,3.14),(4.13,3.29,3.14),(4.85,3.29,3.14),(4.85, 7.84, 3.14), (3.48, 7.84, 3.14), (3.48,7.11,3.14), (2.17,7.11,3.14), 
               (2.17,9.01,3.14),(8.81,9.01,3.14),(8.81,7.11,3.14),(7.52,7.11,3.14), (7.52,7.84,3.14), (6.15,7.84, 3.14), (6.15, 3.29, 3.14), 
               (6.88, 3.29, 3.14), (6.88, 2.00, 3.14), (4.13, 2.00, 3.14)]
    goal_A = [ (1.22, 3.70, 3.14), (1.22, 4.29, 3.14),(1.54, 4.29, 3.14),(2.33, 6.13, 3.14), (2.19, 6.13, 3.14),(2.19, 6.72, 3.14), (3.46, 6.72, 3.14),
               (3.46, 6.13, 3.14), (3.33, 6.13, 3.14), (4.11, 4.29, 3.14), (4.43, 4.29, 3.14), (4.43, 3.70, 3.14), (3.28, 3.70, 3.14), (3.28, 4.29, 3.14),
               (3.45, 4.29, 3.14), (3.31, 4.60, 3.14), (2.36, 4.60, 3.14), (2.24, 4.29, 3.14), (2.39, 4.29, 3.14), (2.39, 3.70, 3.14), (1.22, 3.70, 3.14)]
    goal_a = [ (2.63, 5.19, 3.14), (2.83, 5.69, 3.14), (3.06, 5.19, 3.14), (2.63, 5.19, 3.14)]

    goal_M = [ (6.57, 3.70, 3.14), (6.57, 4.29, 3.14), (6.81, 4.29, 3.14), (6.81, 6.13, 3.14), (6.59, 6.13, 3.14), (6.59, 6.72, 3.14), (7.52, 6.72, 3.14),
               (8.17, 5.40, 3.14), (8.81, 6.72, 3.14), (9.78, 6.72, 3.14), (9.78, 6.13, 3.14), (9.54, 6.13, 3.14), (9.54, 4.29, 3.14), (9.78, 4.29, 3.14),
               (9.78, 3.70, 3.14), (8.71, 3.70, 3.14), (8.71, 4.29, 3.14), (8.93, 4.29, 3.14), (8.93, 5.58, 3.14), (8.17, 4.02, 3.14), (7.42, 5.57, 3.14),
               (7.42, 4.29, 3.14), (7.66, 4.29, 3.14), (7.66, 3.70, 3.14), (6.57, 3.70, 3.14)]

    goals = goal_A + goal_a + goal_T + goal_M
    print(f"Total number of goals: {len(goals)}")


    # Split goals among turtles
    goals_per_turtle = len(goals) // num_turtles
    turtle_nodes = []
    processes = []

    remainder = len(goals) % num_turtles
    for i in range(num_turtles):
        turtle_name = f"turtle{i+1}" # Because turtle1 is default

        start_index = i * goals_per_turtle

        if i < remainder:
            end_index = start_index + goals_per_turtle + 1
        else:
            end_index = start_index + goals_per_turtle
            end_index = (i + 1) * goals_per_turtle if i != num_turtles - 1 else len(goals)



        turtle_goals = goals[start_index:end_index]
        print(f"Number of goals assigned to {turtle_name}: {len(turtle_goals)}")
        
        if i != 0:  # We don't spawn for turtle1 as it's default
            spawn_turtle(turtle_name)
        
        turtle_node = Turtle_GTG(turtle_name, turtle_goals, start_index)
        turtle_nodes.append(turtle_node)

    executor = rclpy.executors.MultiThreadedExecutor()

    ## executor to make the turtles appear to move simultaneusly on the turtlesim window
    for node in turtle_nodes:
        executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass

    for node in turtle_nodes:
        node.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()