# simulation.py
# Description: This module defines the Simulation class for running robot navigation simulations using Pygame.

# Import necessary libraries
import pygame
import random
import time
from genrobot import Robot
from genobstacle import Obstacle
from gengoal import Goal

# --- Colors ---
# To make things easy, we define some colors that we will use in the simulation.
# These colors are represented as RGB tuples.
BLACK      = (0, 0, 0)
WHITE      = (255, 255, 255)
RED        = (255, 0, 0)
DARK_RED   = (139, 0, 0)
BLUE       = (0, 0, 255)
GREEN      = (0, 255, 0)
DARK_GREEN = (0, 100, 0)
TEAL       = (0, 255, 255)

# --------------------------------------------------------------------------------------------
# The Simulation class is responsible for managing the simulation environment, including obstacles, goals, and robot navigation.
# It initializes the simulation parameters, generates obstacles, and runs the simulation with a given robot configuration
class Simulation:
    
    def __init__(self, screen_width, screen_height, visualize=False):
        # Inputs:
        #   screen_width (int): The width of the simulation window.
        #   screen_height (int): The height of the simulation window.
        #   visualize (bool): Whether to visualize the simulation using Pygame (default is False).
        # Outputs:  
        #   None: This method initializes the simulation environment, including screen dimensions, goal position, and obstacles.

        # Initialize simulation parameters
        self.width = screen_width
        self.height = screen_height
        self.visualize = visualize

        # If visualization is enabled, initialize Pygame and set up the display
        if self.visualize:
            # Check if a display is already initialized
            if not pygame.display.get_init():
                pygame.init()
            self.screen = pygame.display.set_mode((self.width, self.height))
            pygame.display.set_caption("Robot Navigation Simulation")
            self.clock = pygame.time.Clock()
        
        # Set the start and goal positions for the robot
        # The start position is fixed at (50, 50) and the goal position is fixed at (width - 50, height - 50).
        self.start_pos = (50, 50)
        self.goal_pos = (self.width - 50, self.height - 50)
        self.goal = Goal(self.goal_pos[0], self.goal_pos[1])
        
        # Generate initial obstacles
        # The obstacles are randomly placed in the simulation area, ensuring they are not too close to the start or goal positions.
        self.obstacles = []
        self.generate_new_obstacles()

    # --------------------------------------------------------------------------------------------
    # The generate_new_obstacles method clears existing obstacles and creates a new random layout.
    # It ensures that the new obstacles are not too close to the start or goal positions.
    def generate_new_obstacles(self):
        # Input:
        #   None: This method does not take any input parameters.
        # Output:
        #   None: This method clears existing obstacles and generates a new set of obstacles in the simulation area.
        
        # Clear existing obstacles
        # This is useful for resetting the simulation environment when needed.
        self.obstacles.clear()

        # Generate a new set of N obstacles
        # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    
        N = 10
        # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        for _ in range(N):
            # We use the while true loop to ensure that we find a valid position for each obstacle, with the loop being exited only when a valid position is found.
            # The position is valid if it is at least 120 pixels away from both the start and goal positions.
            # The radius of each obstacle is randomly chosen between 25 and 45 pixels.
            while True:
                x = random.randint(0, self.width)
                y = random.randint(0, self.height)
                pos = pygame.math.Vector2(x, y)
                if pos.distance_to(pygame.math.Vector2(self.start_pos)) > 120 and \
                   pos.distance_to(pygame.math.Vector2(self.goal_pos)) > 120:
                    self.obstacles.append(Obstacle(x, y, radius=random.randint(25, 45)))
                    break

    # --------------------------------------------------------------------------------------------
    # The run_sim_with_chromosome method runs a single simulation for a given chromosome configuration.
    # It initializes a robot with the specified chromosome, updates its state in the simulation, and evaluates its performance.
    # The method returns a fitness score based on the robot's performance in reaching the goal or avoiding obstacles.
    def run_sim_with_chromosome(self, chromosome, num_sectors, max_time_s=15):
        # Input:
        #   chromosome (list): The list of genes representing the robot's configuration.   
        #   num_sectors (int): The number of sectors the robot should have.
        #   max_time_s (int): The maximum time allowed for the simulation in seconds (default is 15).
        # Output:
        #   float: The fitness score for this run, which indicates how well the robot performed in the simulation.
        
        # Initialize the robot with the given chromosome and start position
        # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
         #Adjust the sensor range to see how it affects the evolved controller
        sensor_range = 200
        # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        robot = Robot(self.start_pos[0], self.start_pos[1], num_sectors, chromosome, sensor_range)
        
        # We need to know the star time of the simulation to calculate the time elapsed.
        start_time = time.time()
        
        # We have a flag called "running" to control the simulation loop.
        # This allows us to exit the loop gracefully when the simulation is done or when the user closes the window.
        running = True
        while running:
            # If visualization is enabled, we handle Pygame events to allow quitting the simulation.
            if self.visualize:
                for event in pygame.event.get():
                    if event.type == pygame.QUIT:
                        running = False

            # Time stuff
            dt = 1/60.0
            time_elapsed = time.time() - start_time

            # Update the robot's state based on the current time step, goal position, obstacles, and screen dimensions.
            # The robot's update method will handle its movement, collision detection, and goal seeking behavior.
            # The robot will also check if it has crashed
            robot.update(dt, self.goal.position, self.obstacles, self.width, self.height)
            crashed = robot.crashed
            
            # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
            # 
            # Adjust the scoring system to see how it affects the evolved controller
            # If the robot has crashed, we can calculate the fitness score based on its distance to the goal.
            if crashed:
                dist_to_goal = robot.position.distance_to(self.goal.position)
                return 100 / (dist_to_goal + 1)
            
            # If the robot has reached the goal, we return a high fitness score.
            if robot.position.distance_to(self.goal.position) < robot.radius + self.goal.radius:
                return 10000 / (time_elapsed + 1)

            # If the time elapsed exceeds the maximum allowed time, we return a score based on the distance to the goal.
            # This allows us to evaluate the robot's performance even if it didn't reach the goal within the time limit.
            if time_elapsed > max_time_s:
                dist_to_goal = robot.position.distance_to(self.goal.position)
                return 1000 / (dist_to_goal + 1)
            # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

            # If visualization is enabled, we draw the current state of the simulation.
            # This includes drawing the goal, obstacles, and the robot itself.
            if self.visualize:
                self.screen.fill(WHITE) # Dark blue background
                self.goal.draw(self.screen)
                for obs in self.obstacles:
                    obs.draw(self.screen)
                robot.draw(self.screen) # Draw robot last so sensors are on top
                pygame.display.flip()
                self.clock.tick(60)
        return 0
