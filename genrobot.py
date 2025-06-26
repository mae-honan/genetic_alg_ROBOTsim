# robot.py
# This module defines the Robot class, which represents an autonomous robot that navigates using a sector-based reactive controller.
# It includes methods for updating the robot's state, handling collisions, and visualizing its sensors

# Import necessary libraries
import pygame
import math

# --------------------------------------------------------------------------------------------
# The robot class represents an autonomous robot that navigates using a sector-based reactive controller.
# It is designed to be controlled by a genetic algorithm, with parameters that can be tuned for optimal performance.
class Robot:
    
    # --------------------------------------------------------------------------------------------
    # The constructor initializes the robot's position, heading, sensor configuration, and genetic algorithm parameters.
    def __init__(self, x, y, num_sectors, ga_chromosome, sensor_range=150, radius=20):
        # Input:
        #   x: Initial x-coordinate of the robot.
        #   y: Initial y-coordinate of the robot.
        #   num_sectors: Number of sectors for the robot's sensors.
        #   ga_chromosome: A list of parameters from the genetic algorithm that define the robot's behavior.
        #   sensor_range: The range of the robot's sensors (default is 150).
        #   radius: The radius of the robot (default is 20).
        # Output:
        #   None: This constructor initializes the robot's attributes and prepares it for simulation.

        # --- Robot Configuration ---
        # Position is a 2D vector, radius is the size of the robot
        self.position = pygame.math.Vector2(x, y)
        self.radius = radius
        self.heading = 0  # Angle in radians, 0 is pointing right

        # --- Controller Configuration ---
        # Number of sectors for the robot's sensors, sensor range, and sector angle
        self.num_sectors = num_sectors
        self.sensor_range = sensor_range
        self.sector_angle = 2 * math.pi / self.num_sectors
        self.chromosome = ga_chromosome

        # --- Logic Configuration ---
        # Initialize the robot's state
        self.crashed = False  # Flag to indicate if the robot has crashed
        
        # --- GA Parameter Parsing ---
        # The last 2 genes are for goal seeking
        self.goal_rotation_scale = self.chromosome[-2]
        self.base_forward_speed = self.chromosome[-1]
        
        # The first N*4 genes are for obstacle avoidance, 4 per sector
        self.sector_params = []
        for i in range(self.num_sectors):
            start_index = i * 4
            # Params: (A, B, C, D)
            # fwd += A/(dist^B), rot += C/(dist^D)
            params = tuple(self.chromosome[start_index : start_index + 4])
            self.sector_params.append(params)
            
        # --- Sensor State Initialization ---
        self.closest_in_sector = [float('inf')] * self.num_sectors

    # --------------------------------------------------------------------------------------------
    # The update method updates the robot's state based on its sensors and the environment.
    # It calculates the robot's movement towards a goal position while avoiding obstacles.
    def update(self, dt, goal_pos, obstacles, screen_width, screen_height):
        # Input:
        #   dt: Time step for the update (in seconds).
        #   goal_pos: The target position the robot is trying to reach (as a pygame.math.Vector2).
        #   obstacles: A list of obstacles in the environment, each with a position and radius.
        #   screen_width: Width of the simulation area (for boundary checking).
        #   screen_height: Height of the simulation area (for boundary checking).
        # Output:
        #   None: This method updates the robot's position, heading, and state based on the sensors and environment.

        # --- 1. Sensing ---
        # The sensing phase detects obstacles and boundaries in the robot's environment and determines the closest obstacles in each sector.
        # Reset closest distances for each sector
        self.closest_in_sector = [float('inf')] * self.num_sectors

        # Loop through all obstacles to determine if they are within the sensor range and if so which sector they fall into.
        for obs in obstacles:
            # Calculate the distance to the obstacle
            vec_to_obs = obs.position - self.position
            dist = vec_to_obs.length() - obs.radius

            # If the obstacle is within sensor range, calculate its sector
            if dist < self.sensor_range:
                # Angle of obstacle relative to world frame (0 is right)
                world_angle = math.atan2(vec_to_obs.y, vec_to_obs.x)
                # Angle relative to robot's heading
                relative_angle = world_angle - self.heading
                # Normalize to [-pi, pi]
                relative_angle = (relative_angle + math.pi) % (2 * math.pi) - math.pi
                
                # Determine which sector the obstacle falls into
                sector_index = int((relative_angle + math.pi) / self.sector_angle)
                
                # Check if the observed obstacle is the closest in its sector
                if dist < self.closest_in_sector[sector_index]:
                    self.closest_in_sector[sector_index] = dist

        # Sense the window boundaries as obstacles
        for i in range(self.num_sectors):
            # Calculate the world angle for the middle of the current sector
            middle_angle = self.heading - math.pi + (i + 0.5) * self.sector_angle
            cos_a = math.cos(middle_angle)
            sin_a = math.sin(middle_angle)

            # Find distances to each of the four walls along this ray
            distances_to_walls = []
            # Check for division by zero with a small epsilon
            # Distance to left wall (x=0)
            if cos_a < -1e-6:
                t = -self.position.x / cos_a
                distances_to_walls.append(t)
            # Distance to right wall (x=screen_width)
            if cos_a > 1e-6:
                t = (screen_width - self.position.x) / cos_a
                distances_to_walls.append(t)
            # Distance to top wall (y=0)
            if sin_a < -1e-6:
                t = -self.position.y / sin_a
                distances_to_walls.append(t)
            # Distance to bottom wall (y=screen_height)
            if sin_a > 1e-6:
                t = (screen_height - self.position.y) / sin_a
                distances_to_walls.append(t)
            
            # If we have any valid distances, find the minimum one
            if distances_to_walls:
                dist_to_wall = min(distances_to_walls)
                # If this wall is within sensor range and closer than any physical obstacle
                # already detected in this sector, update the sector's distance.
                if dist_to_wall < self.closest_in_sector[i] and dist_to_wall < self.sensor_range:
                    self.closest_in_sector[i] = dist_to_wall

        # --- 2. Control Calculation ---
        # These are the underlying control commands that we are going to be modifying. 
        # Note that this is pretty similar to what we did before, but instead of using arrow keys, we are making it autonomous!
        rot_vel = 0
        fwd_vel = 0

        # a) Base velocity from goal-seeking behavior
        # Calculate the vector to the goal and the angle to it
        vec_to_goal = goal_pos - self.position
        angle_to_goal = math.atan2(vec_to_goal.y, vec_to_goal.x)
        distance_to_goal = vec_to_goal.length()
        robot_heading_to_goal = angle_to_goal - self.heading
        robot_heading_to_goal = (robot_heading_to_goal + math.pi) % (2 * math.pi) - math.pi # Convert to [-pi, pi]
        # Note how having the robot's heading in the range [-pi, pi] allows for easier rotation calculations later on.

        # Calculate forward and rotational velocities
        # Base forward speed is a constant, and rotation speed is scaled by the angle difference
        # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        #  How should we formulate the forward velocity and rotational velocities?
        # Hint: we have self.goal_rotation_scale and self.base_forward_speed as the genes 
        rot_vel +=self.goal_rotation_scale*robot_heading_to_goal
        fwd_vel +=self.base_forward_speed
        # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

        # b) Modify velocities based on obstacle sectors
        for i in range(self.num_sectors):
            dist = self.closest_in_sector[i]
            if dist < self.sensor_range:
                A, B, C, D = self.sector_params[i]
                # Add a small epsilon to distance to prevent division by zero
                dist_safe = dist + 1e-6
                # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
                #  How are we going to set the forward and rotational velocities based on the distance measurements?
                # There are no wrong answers; the GA will find a way to optimize things.
                fwd_vel += A/(dist_safe**B)
                rot_vel +=C/(dist_safe**D)
                # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

        # Clamp velocities to reasonable limits
        max_vel = 400
        fwd_vel = max(0, min(max_vel, fwd_vel))
        rot_vel = max(-math.pi, min(math.pi, rot_vel))

        # --- 3. Kinematic Update ---
        self.heading += rot_vel * dt
        self.heading %= (2 * math.pi) 
        # Note how this keeps the heading angle from growing indefinitely
        # For floating point numbers, this is a good way to keep numbers in a range that doesn't have resolution problems

        # Note that this is NOT the actual position update, just a potential new position that we will later check for collisions. Make a variable called "new_position" to store this.
        # Remember that there are four main things that go into this: dt, the heading,  the forward velocity, and the current position.
        move_vec = pygame.math.Vector2(fwd_vel * dt, 0).rotate_rad(self.heading)
        new_position = self.position + move_vec

        # --- 4. Collision Checking ---
        # Set a flag called "crashed" to True if the robot is about to crash into a wall or an obstacle.
        # Previously, we didn't "crash", but rather just didn't move if we would collide with something.
        # The crash flag is important for the genetic algorithm to evaluate the robot's performance but in other applications, you might want to just not move instead.
        self.crashed = False

        # If it is outside the screen boundaries, set crashed to True.
        if not (self.radius <= new_position.x <= screen_width - self.radius and
                self.radius <= new_position.y <= screen_height - self.radius):
            self.crashed = True

        # If it is too close to any obstacle, set crashed to True.
        for obs in obstacles:
            if new_position.distance_to(obs.position) < self.radius + obs.radius:
                self.crashed = True
                break
        
        # If the robot has not crashed, update its position.
        # This prevents the robot from moving if it would crash into a wall or obstacle.
        if not self.crashed:
            self.position = new_position
        

    # --------------------------------------------------------------------------------------------
    # The draw method visualizes the robot and its sensor state on the given Pygame screen.
    # It draws the robot's body, heading line, sensor range, and detected obstacles in each sector.
    # This method is useful for debugging and understanding the robot's behavior in the simulation.
    def draw(self, screen):
        # Input:
        #   screen: The Pygame surface where the robot will be drawn.
        # Output:
        #   None: This method does not return anything; it draws the robot and its sensors on the screen.
        
        # Body
        pygame.draw.circle(screen, (0, 128, 255), self.position, self.radius)
        # Heading line
        heading_end = self.position + pygame.math.Vector2(self.radius, 0).rotate_rad(self.heading)
        pygame.draw.line(screen, (255, 255, 255), self.position, heading_end, 3)

        # --- Sensor Visualization ---
        # Draw the transparent sensor range circle
        pygame.draw.circle(screen, (50, 50, 50), self.position, self.sensor_range, 1)

        for i in range(self.num_sectors):
            # Draw the faint lines that define the edges of each sensor sector
            sector_start_angle_world = self.heading - math.pi + i * self.sector_angle
            line_end = self.position + pygame.math.Vector2(self.sensor_range, 0).rotate_rad(sector_start_angle_world)
            pygame.draw.line(screen, (40, 40, 40), self.position, line_end)

            dist = self.closest_in_sector[i]
            # If an obstacle is detected in this sector, draw the new visualization
            if dist < self.sensor_range and dist > 0:
                
                # Calculate the angle for the middle of the current sector
                middle_angle = sector_start_angle_world + (self.sector_angle / 2)
                
                # Calculate the endpoint of the line based on the detected distance
                detection_point = self.position + pygame.math.Vector2(dist, 0).rotate_rad(middle_angle)
                
                # Draw a thick, bright line from the robot's center to the detection point
                pygame.draw.line(screen, (255, 165, 0), self.position, detection_point, 3)
