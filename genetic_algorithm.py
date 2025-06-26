# genetic_algorithm.py
# Description: This module implements a genetic algorithm to evolve robot behaviors in a simulation.

# import necessary libraries
import random
from gensimulation import Simulation
import pygame



# We define a class for the genetic algorithm that will manage the evolution process.
# This class will handle the population of robots, their chromosomes, and the evolution process.
# We use a class to encapsulate the genetic algorithm logic, making it easier to manage and extend in the future.
class GeneticAlgorithm:

    # ----------------------------------------------------------------------------------------
    # The constructor initializes the genetic algorithm with parameters like population size, number of parameters per robot
    def __init__(self, population_size, num_params, num_sectors, screen_dims):
        # Input:
        #   population_size (int): The number of robots in the simulation.
        #   num_params (int): The number of parameters (genes) each robot will have in its chromosome.
        #   num_sectors (int): The number of sectors for obstacle avoidance.
        #   screen_dims (tuple): The dimensions of the simulation area (width, height).
        # Output:
        #   None: This method initializes the genetic algorithm with the given parameters.

        # The Population size determines how many robots will be in the simulation.
        self.population_size = population_size
        # The number of parameters defines how many genes each robot will have in its chromosome.
        # Each gene corresponds to a specific behavior or characteristic of the robot.
        self.num_params = num_params
        # The number of sectors defines how many sectors the robot will use for obstacle avoidance.
        # Each sector will have its own set of parameters for obstacle avoidance.
        self.num_sectors = num_sectors
        # The screen dimensions define the size of the simulation area.
        # This will be used to create the simulation environment.
        self.screen_dims = screen_dims

        # We define the parameter ranges for each gene in the chromosome.
        # This will be used to generate random chromosomes for the initial population.
        self._define_param_ranges()

        # Create the initial population of robots with random chromosomes.
        self.population = self._create_initial_population()

    # ----------------------------------------------------------------------------------------
    # This method defines the parameter ranges for each gene in the chromosome.
    # Each robot's chromosome consists of parameters that control its behavior in the simulation.
    def _define_param_ranges(self):
        # Input:
        #   None: This method initializes the parameter ranges for the robot's chromosomes.
        # Output:
        #   None: This method does not return anything; it sets up the parameter ranges for the robot's chromosomes.

        self.param_ranges = []
        # Add ranges for each obstacle avoidance sector (A, B, C, D)
        for i in range(self.num_sectors):
            # A: Forward influence (usually negative to slow down)
            self.param_ranges.append((-200.0, 20.0))
            # B: Forward exponent
            self.param_ranges.append((0.5, 2.0))
            # C: Rotational influence
            self.param_ranges.append((-200.0, 200.0))
            # D: Rotational exponent
            self.param_ranges.append((0.5, 2.0))
        
        # Add ranges for goal-seeking behavior at the end
        # Goal rotation scale
        self.param_ranges.append((0.5, 5.0))
        # Base forward speed
        self.param_ranges.append((50.0, 150.0))

    # ----------------------------------------------------------------------------------------
    # This method creates the initial population of robots with random chromosomes.
    # Each chromosome is a list of parameters within the defined ranges.
    def _create_initial_population(self):
        # Input:
        #   None: This method generates the initial population of robots.
        # Output:
        # population (list): A list of chromosomes, where each chromosome is a list of parameters.

        # Create an empty list to hold the population of chromosomes.
        # Each chromosome will be a list of parameters, randomly initialized within the defined ranges.
        population = []

        # Generate random chromosomes for the initial population.
        for _ in range(self.population_size):
            # Create a chromosome by sampling random values from the defined parameter ranges.
            # Each chromosome will have a length equal to the number of parameters defined.
            chromosome = [random.uniform(low, high) for low, high in self.param_ranges]
            # Append the generated chromosome to the population list.
            # This ensures that each robot in the population has a unique set of parameters.
            population.append(chromosome)
        return population

    # ----------------------------------------------------------------------------------------
    # This method runs the evolution process for a specified number of generations.
    # It evaluates the fitness of each robot in the population, selects the best ones, and
    # generates the next generation through crossover and mutation.
    # The evolution process is repeated for a specified number of generations.
    def run_evolution(self, generations, num_eval_runs, mutation_rate=0.15, crossover_rate=0.8):
        # Input:
        #   generations (int): The number of generations to evolve the population.
        #   num_eval_runs (int): The number of evaluation runs for each robot to get a reliable fitness score.
        #   mutation_rate (float): The probability of mutating a gene in the chromosome (default is 0.15).
        #   crossover_rate (float): The probability of performing crossover between two chromosomes (default is 0.8).
        # Output:
        #   best_chromosome_overall (list): The best chromosome found across all generations.

        # Create a headless simulation instance for fitness evaluation.
        # "Headless" means it runs without rendering the graphical interface, which speeds up the evaluation process since we don't need to visualize every run.
        # The simulation will be used to evaluate the fitness of each robot's chromosome.
        sim_headless = Simulation(self.screen_dims[0], self.screen_dims[1], visualize=False)

        # Initialize variables to track the best chromosome and fitness over generations.
        # fitness_history will store the best fitness score for each generation.
        fitness_history = []

        # best_chromosome_overall will hold the best chromosome found across all generations.
        # best_fitness_overall will track the highest fitness score achieved.
        best_chromosome_overall = None

        # Initialize the best fitness score to a very low value.
        # This ensures that any valid fitness score will be higher than this initial value.
        best_fitness_overall = -1

        # Loop through the specified number of generations to evolve the population.
        # In each generation, we evaluate the fitness of each robot, select the best ones, 
        # and generate the next generation through crossover and mutation.
        print(f"Starting evolution for {generations} generations with {num_eval_runs} evaluation runs per robot.")
        for gen in range(generations):
            print(f"\n--- Evaluating Generation {gen+1}/{generations} ---")
            
            # Create a list to hold the fitness scores for each robot in the current generation.
            # This will be used to determine which robots performed best in the simulation.
            fitness_scores = []
            
            # Evaluate each robot in the population by running the simulation multiple times.
            # This minimizes the impact of random variations in the simulation environment and makes the fitness evaluation more robust.
            for i, chromosome in enumerate(self.population):
                print(f"  - Evaluating robot {i+1}/{self.population_size}...", end='\r')
                
                # We will sum the fitness scores from multiple runs to get a more reliable average fitness score.
                # To get an average, we need a variable to hold the total fitness score.
                total_fitness = 0
                for _ in range(num_eval_runs):
                    # Generate new obstacles for each evaluation run.
                    # This ensures that each run has a fresh set of obstacles, making the evaluation more challenging and realistic.
                    sim_headless.generate_new_obstacles()
                    # Run the simulation with the current chromosome and accumulate the fitness score.
                    total_fitness += sim_headless.run_sim_with_chromosome(chromosome, self.num_sectors)
                
                # Calculate the average fitness score for the current robot.
                average_fitness = total_fitness / num_eval_runs
                # Append the average fitness score to the fitness_scores list.
                # This will be used to rank the robots based on their performance.
                fitness_scores.append(average_fitness)
            
            # Sort the population based on fitness scores in descending order.
            # This will help us identify the best-performing robots in the current generation.
            pop_with_fitness = sorted(zip(self.population, fitness_scores), key=lambda x: x[1], reverse=True)
            
            # The best chromosome and fitness for the current generation is the first element in the sorted list.
            # This is because we sorted the population based on fitness scores, so the first element has the highest fitness.
            best_chromosome_gen, best_fitness_gen = pop_with_fitness[0]
            fitness_history.append(best_fitness_gen)

            # Check if the best fitness of this generation is better than the overall best fitness found so far.
            # If it is, update the overall best fitness and the corresponding chromosome.
            if best_fitness_gen > best_fitness_overall:
                best_fitness_overall = best_fitness_gen
                best_chromosome_overall = best_chromosome_gen

            # Print the best fitness of the current generation and visualize the best robot.
            # This is not necessary for the algorithm to function, but it provides useful feedback during the evolution process and it looks nice.
            print(f"Gen {gen+1} Best Avg Fitness: {best_fitness_gen:.2f}         ")
            print(f"Visualizing best of generation...")

            sim_visual = Simulation(self.screen_dims[0], self.screen_dims[1], visualize=True)
            sim_visual.run_sim_with_chromosome(best_chromosome_gen, self.num_sectors)
            try:
                pygame.quit()
            except pygame.error:
                pass

            # Prepare for the next generation by selecting the best chromosomes and applying crossover and mutation.
            # We will keep the top 10% of the population as elites, and then perform crossover and mutation to generate the rest of the next generation.
            print("Generating next generation...")
            # Start with an empty list for the next generation.
            # This will hold the chromosomes for the next generation of robots.
            next_generation = []

            # Select the top N% of the population as elites.
            N = 0.1  # Top 10% of the population
            elite_count = int(self.population_size * N)
            elites = [p[0] for p in pop_with_fitness[:elite_count]]
            next_generation.extend(elites)

            # Fill the rest of the next generation using tournament selection, crossover, and mutation.
            # We will use tournament selection to choose pairs of parents, perform crossover to create offspring,
            # and apply mutation to introduce genetic diversity.
            while len(next_generation) < self.population_size:
                # Select two parents using tournament selection.
                # Tournament selection randomly selects a few individuals from the population and chooses the best one among them
                p1 = self._tournament_selection(pop_with_fitness)
                p2 = self._tournament_selection(pop_with_fitness)

                # Perform crossover with a certain probability.
                # Crossover combines the genetic information of two parents to create offspring.
                # If crossover occurs, we create two new chromosomes; otherwise, we keep the parents unchanged
                if random.random() < crossover_rate:
                    c1, c2 = self._crossover(p1, p2)
                else:
                    c1, c2 = p1[:], p2[:]

                # Mutate the offspring chromosomes with a certain mutation rate.
                # Mutation introduces random changes to the chromosomes, which helps maintain genetic diversity in the population.
                # This is important to prevent the population from converging too quickly to a suboptimal solution.
                # Note: This is as good a time as any to discuss deep versus shallow copies...
                self._mutate(c1, mutation_rate)
                self._mutate(c2, mutation_rate)

                # Add the mutated offspring to the next generation.
                # We ensure that we do not exceed the population size by checking the length of next_generation.
                # If we have space, we add both offspring; otherwise, we only add one.
                next_generation.append(c1)
                if len(next_generation) < self.population_size:
                    next_generation.append(c2)

            # Now that the next generation is filled, we update the population for the next iteration.
            self.population = next_generation
        
        # After all generations are complete, we return the best chromosome found and the fitness history.
        return best_chromosome_overall, fitness_history
    
    # ----------------------------------------------------------------------------------------
    # --- Helper methods (Tournament, Crossover, Mutate) remain the same ---
    # These methods are used to perform tournament selection, crossover, and mutation on the chromosomes.
    # Tournament selection randomly selects a few individuals from the population and returns the best one.
    def _tournament_selection(self, pop_with_fitness, k=5):
        tournament = random.sample(pop_with_fitness, k)
        return sorted(tournament, key=lambda x: x[1], reverse=True)[0][0]

    # ---------------------------------------------------------------------------------------
    # Crossover combines two parent chromosomes to create two offspring chromosomes.
    # It randomly selects a crossover point and exchanges genes between the parents.
    # This method takes two parent chromosomes and returns two new chromosomes created by crossover.
    def _crossover(self, p1, p2):
        # Inputs:
        # p1: First parent chromosome (list of parameters)
        # p2: Second parent chromosome (list of parameters)
        # Outputs:
        # c1: First offspring chromosome (list of parameters)
        # c2: Second offspring chromosome (list of parameters)

        # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
       
        # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


        # The crossover point is randomly chosen, and the genes are exchanged between the parents to create two new offspring.
        # t create a variable called "point" that is a random integer between 1 and self.num_params - 1.
        # This will be the point at which the genes are exchanged between the parents.
        point = random.randint(1,self.num_params-1)

        # Use the crossover point to create two new chromosomes.
        # The first part of the first parent is combined with the second part of the second parent, then the same is done in reverse.
        # This creates two new chromosomes that inherit traits from both parents.
        #  create c1 and c2 from p1 and p2 using the crossover point.
        c1 = p1[:point]+p2[point:]
        c2 = p2[:point]+p1[point:]

        # In python we can return multiple values like this
        return c1, c2

    # ---------------------------------------------------------------------------------------
    # This method applies mutation to a chromosome by randomly changing its genes. 
    def _mutate(self, chromosome, mutation_rate):
        # Inputs: 
        # chromosome: The chromosome to mutate (list of parameters)
        # mutation_rate: The probability of each gene being mutated (float between 0 and 1)
        # Outputs:
        # None

        # For each gene in the chromosome, we check if it should be mutated based on the mutation rate.
        # If a random number is less than the mutation rate, we apply a small random change
        for i in range(self.num_params):
            # Only mutate with some probability
            if random.random() < mutation_rate:
                # Apply a small random change to the gene based on its range.
                # This ensures that the mutation is small enough to not drastically change the gene's value.
                range_width = self.param_ranges[i][1] - self.param_ranges[i][0]
                # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
                # How to formulate this change? (hint: use range_width as a scaling factor for some random number)
                change = random.normalvariate(0,range_width * 0.1)
                # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
                chromosome[i] += change
                low, high = self.param_ranges[i]
                chromosome[i] = max(low, min(high, chromosome[i])) # This line of code bounds the mutated gene within its defined range.
