# Description: This module defines the Obstacle class for a simulation using PyGame.
# The obstacle.py module is part of a simple robot simulation using Pygame.
# Although the obstacled doesn't do much, the class can easily be extended to include more complex behaviors or properties.
# obstacle.py
# --------------------------------------------------------------------------------------------
# The only library we need for this module is Pygame, which is used for creating games and simulations in Python.
import pygame

# --------------------------------------------------------------------------------------------
# The Obstacle class represents an obstacle in the simulation. It has properties like position, radius, and color.
class Obstacle:
    # The __init__ method initializes the obstacle with a position (x, y), a radius, and a color.
    # The position is a vector that represents the center of the obstacle, and the radius defines how large the obstacle is.
    # Note that in the __init__ method we define all of the various properties of the class. This is a common practice in Python to ensure that each instance of the class has its own unique properties.
    def __init__(self, x, y, radius=30, color=(255, 0, 0)):
        # Input:
        #   x: The x-coordinate of the obstacle's position. 
        #   y: The y-coordinate of the obstacle's position.
        #   radius: The radius of the obstacle (default is 30).
        #   color: The color of the obstacle (default is red, represented as an RGB tuple).
        # Output:
        #   None: This method does not return anything; it initializes the obstacle's properties.

        # Instead of storing x and y as separate variables, we use a Pygame Vector2 object to represent the position.
        # This allows us to easily perform vector operations like distance calculations.
        self.position = pygame.math.Vector2(x, y)
        self.radius = radius
        self.color = color

    # The draw method is responsible for rendering the obstacle on the screen.
    # It uses Pygame's draw.circle function to draw a circle at the obstacle's position with the specified radius and color.
    def draw(self, screen):
        # Input:
        #   screen: The Pygame surface where the obstacle will be drawn.
        # Output:
        #   None: This method does not return anything; it simply draws the obstacle on the screen.

        # Draw the obstacle as a circle on the given screen.
        pygame.draw.circle(screen, self.color, (int(self.position.x), int(self.position.y)), self.radius)