# Import necessary libraries
import pygame

# goal.py
# This module defines the Goal class, which represents a goal in a simulation.
# It includes methods for initializing the goal and drawing it on the screen.   
class Goal:
    
    # The goal is defined by its position, radius, and color.
    def __init__(self, x, y, radius=15, color=(0, 255, 0)):
        # Input:
        #   x (int): The x-coordinate of the goal.
        #   y (int): The y-coordinate of the goal. 
        #   radius (int): The radius of the goal circle (default is 15).
        #   color (tuple): The color of the goal, represented as an RGB tuple (default is green).
        # Output:
        #   None: This method initializes the goal's properties.
        
        # We use pygame's Vector2 for position handling.
        self.position = pygame.math.Vector2(x, y)
        self.radius = radius
        self.color = color

    # Method to draw the goal on the screen.
    # It uses pygame's draw.circle method to render the goal.
    def draw(self, screen):
        # Input:
        #   screen: The Pygame surface where the goal will be drawn. 
        # Output:
        #   None: This method does not return anything; it simply draws the goal on the screen.

        pygame.draw.circle(screen, self.color, (int(self.position.x), int(self.position.y)), self.radius)

