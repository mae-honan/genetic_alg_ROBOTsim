# Import necessary modules
from genetic_algorithm import GeneticAlgorithm
from gensimulation import Simulation
import matplotlib.pyplot as plt
import time

# --- Constants ---
# This defines the screen dimensions and frame rate for the simulation. Modify as needed.
SCREEN_WIDTH = 800
SCREEN_HEIGHT = 600
FPS = 60

# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# The number of sectors for the robot's sensors. This can be adjusted based on the robot's design.
# Each sector will have its own set of parameters for the robot's behavior.
#  Adjust this value to see how changing the number of sectors affects the robot's performance.
NUM_SECTORS = 8 # The number of sensor wedges for the robot

# --- Genetic Algorithm Configuration ---
#  Adjust these parameters to see how they affect the evolution process.
POPULATION_SIZE = 20
NUM_GENERATIONS = 10 # Keep low for testing, increase for better results (e.g., 50)
NUM_EVAL_RUNS = 20   # How many random maps to test each robot on (e.g., 100)
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

# --- Calculated Parameters (Do not change) ---
# For each sector: A, B, C, D params. Plus 2 for goal seeking.
vars_per_sector = 4
vars_for_goal_seeking = 2
NUM_PARAMS = (NUM_SECTORS * vars_per_sector) + vars_for_goal_seeking

# --- Functions ---
# This function plots the performance of the genetic algorithm over generations.
# It takes a list of average fitness scores for each generation and creates a line plot.
def plot_performance(generation_fitness):
    # Input:
    #   generation_fitness: A list of average fitness scores for each generation.
    # Output:
    #   None: This function does not return anything; it simply displays a plot.

    # Plots the best fitness score for each generation.
    plt.figure(figsize=(12, 6))
    plt.plot(range(1, len(generation_fitness) + 1), generation_fitness, marker='o', linestyle='-')
    plt.title('Genetic Algorithm Performance Over Generations')
    plt.xlabel('Generation')
    plt.ylabel('Average Fitness Score')
    plt.grid(True)
    plt.xticks(range(1, len(generation_fitness) + 1))
    plt.show()

# This is the main function that runs the genetic algorithm, evaluates the best chromosome, and visualizes the results.
# It initializes the GeneticAlgorithm class, runs the evolution process, and plots the performance.
def main():
    # Input:
    #   None: This function does not take any input parameters.
    # Output:
    #   None: This function does not return anything; it runs the genetic algorithm and visualizes the results.

    # Print initial configuration details for the user.    
    print("--- Starting Genetic Algorithm Evolution ---")
    print(f"Robot Sectors: {NUM_SECTORS}")
    print(f"Genes per Robot: {NUM_PARAMS}")

    ## Start the timer to measure the evolution time.
    start_time = time.time()
    
    # Initialize the GeneticAlgorithm with the specified parameters.
    print("Initializing Genetic Algorithm...")
    # This will create a population of robots with random chromosomes.
    ga = GeneticAlgorithm(
        population_size=POPULATION_SIZE,
        num_params=NUM_PARAMS,
        num_sectors=NUM_SECTORS,
        screen_dims=(SCREEN_WIDTH, SCREEN_HEIGHT)
    )
    
    # Run the evolution process for a specified number of generations and evaluation runs.
    print(f"Running evolution for {NUM_GENERATIONS} generations with {NUM_EVAL_RUNS} evaluation runs per generation...")
    # This will evolve the population of robots, evaluate their performance, and return the best chromosome
    best_chromosome, fitness_history = ga.run_evolution(
        generations=NUM_GENERATIONS, 
        num_eval_runs=NUM_EVAL_RUNS
    )
    
    # Calculate the total time taken for the evolution process.
    # This will give us an idea of how long the evolution took to complete.
    evolution_time = time.time() - start_time
    print(f"\n--- Evolution Complete in {evolution_time:.2f} seconds ---")
    
    # Print the best overall fitness score and the corresponding chromosome.
    print(f"\nBest Overall Average Fitness: {max(fitness_history):.2f}")
    print("Best Evolved Parameters (Chromosome):")
    print(best_chromosome)

    # Run a final simulation with the best evolved robot to visualize its performance.
    print("\n--- Running Final Simulation with Best Evolved Robot ---")
    print("\n--- Displaying Performance Plot ---")
    plot_performance(fitness_history)

    # Create a simulation instance to visualize the best evolved robot's performance.
    print("\n--- Running final simulation with the overall best evolved robot ---")
    final_sim = Simulation(SCREEN_WIDTH, SCREEN_HEIGHT, visualize=True)
    final_sim.run_sim_with_chromosome(best_chromosome, NUM_SECTORS, max_time_s=60)


# This is the entry point of the script.
# When the script is run directly, it will execute the main function.
if __name__ == "__main__":
    main()