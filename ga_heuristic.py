import numpy as np # Used for mathematical calculations such as square roots.
import random # Used to make random choices while creating and changing routes.

class GA_Heuristic:# Groups together the functions used to optimise a sensor visiting route.
    def __init__(self, env): # Receive the environment containing sensor and base-station information.
        self.env = env# Save the environment so all functions can use it.
        
   '''This function creates multiple candidate routes.
    - self: Refers to the current GA object.
    - sensors: The sensors that this mobile sink must visit.
    - pop_size=50: The function creates 50 candidate routes by default.'''     
    def generate_initial_population(self, sensors, pop_size=50):
        # 1. Polar Angle Evaluation (done in env, but let's sort them if needed)
        # 2. Random Partition
        
        population = []#Creates an empty list to store all the candidate routes.
        for _ in range(pop_size):#loop repeats the process 50 times without creating a loop variable<-- the meaning of the underscore.
            s_out = []#Creates an empty list for sensors in the outbound group.
            s_in = []#Creates an empty list for sensors in the inbound group.
            
            for s in sensors:# Randomly decide which part of the route will contain this sensor.
                if random.random() < 0.5:
                    s_out.append(s)# Randomly Put 50% of the sensor in the outbound group.
                else:
                    s_in.append(s) # Randomly put the sensor in the inbound group.
                    
            # 3. Monotonic Sorting (The Heuristic)
            # Sort S_out strictly ascending order of radial distance
            s_out.sort(key=lambda x: self.env.get_radial_distance(x)# Sort outbound sensors from nearest to farthest from the base station.
            
            # Sort S_in strictly descending order of radial distance
            s_in.sort(key=lambda x: self.env.get_radial_distance(x), reverse=True)# Sort inbound sensors from farthest to nearest from the base station.
            
            # 4. Chromosome Assembly
            # pi = (S_0, pi_out, pi_in, S_0)
            chromosome = s_out + s_in# Join the outbound and inbound groups to create one candidate route.
            population.append(chromosome)# Save this candidate route in the population.
            
        return population
        
    def fitness(self, chromosome):
        # Fitness(pi) = L_euclid(pi) + m(pi) * pi * R
        # L_euclid:
        l_euclid = 0# Initialize the route length
        
        # S_0 to first
        if len(chromosome) > 0:#checks whether the route contains any sensors, then calculate the distance from the base station to the first sensor.
            l_euclid += np.sqrt((chromosome[0].x - self.env.base_station[0])**2 + (chromosome[0].y - self.env.base_station[1])**2)
            
            for i in range(len(chromosome) - 1):# Calculate the straight-line distance between every pair of
                                                # consecutive sensors and add it to the total route length.
                l_euclid += np.sqrt((chromosome[i+1].x - chromosome[i].x)**2 + (chromosome[i+1].y - chromosome[i].y)**2)
                
            # last to S_0# Add the distance from the last sensor back to the base station.
            l_euclid += np.sqrt((chromosome[-1].x - self.env.base_station[0])**2 + (chromosome[-1].y - self.env.base_station[1])**2)
            
        # m(pi) is the number of radial maxima.
        # Since we strictly sort S_out ascending and S_in descending, the sequence reaches a maximum distance exactly once.
        # This mathematically guarantees m(pi) = 1 for the INITIAL population.
        # However, GA operations (crossover/mutation) might change it, so we need a function to count m(pi).
        # Count the radial peaks in the route.
        # More peaks mean a larger penalty in the fitness score.
        m_pi = self.count_radial_maxima(chromosome)
        # Add a penalty for each radial peak and return the total score.
        # The genetic algorithm prefers routes with lower fitness values.
        return l_euclid + m_pi * np.pi * self.env.R
        
    def count_radial_maxima(self, chromosome):#This function estimates how many peaks exist in the sequence of radial distances.
        if len(chromosome) <= 1:# If there are zero or one sensors, return one peak by convention.
            return 1
        # Get the radial distance of each sensor in the visiting order. get_radial_distance() is obtained from environment.py file    
        distances = [self.env.get_radial_distance(s) for s in chromosome]
        maxima_count = 0# Initially, no radial peaks have been counted.
        
        # Check peaks in the sequence (including bounds conceptually if they drop to 0 at BS)
        # BS distance is 0. 
        # Add the base station's radial distance (zero) at the start and end.
        # This helps us evaluate the route as a complete round trip.
        full_dists = [0] + distances + [0]
        for i in range(1, len(full_dists) - 1):
            # Check each sensor to see whether it is farther from the base station
            # than both the sensor before it and the sensor after it.
            # example: Distances: Base Station (0) -> A (2) -> B (5) -> C (9) -> D (4) -> Base Station (0).
            # Sensor C is the only peak because its distance (9) is greater than both neighbours (5 and 4).
            if full_dists[i] > full_dists[i-1] and full_dists[i] > full_dists[i+1]:
                maxima_count += 1
                
        # If it's a plateau, it's slightly more complex, but let's assume strictly greater for peak.
        #example [2, 5, 5, 2]The two adjacent values of 5 form a flat peak. Neither one is strictly greater than the other, so the earlier test might not count any peak.
        #This fallback says: if the route contains sensors but no strict peak was found, count one peak anyway.
        if maxima_count == 0 and len(distances) > 0:
            maxima_count = 1 # at least one peak if there are sensors
            
        return maxima_count#Returns the number of peaks to the fitness function.

    def optimize(self, sensors, pop_size=50, generations=100):
        # If no sensors are provided, there is no route to optimise
        if not sensors:
            return []
        # Create the initial set of candidate routes.     
        population = self.generate_initial_population(sensors, pop_size)

        # Repeat the optimisation process for the specified number of generations.
        for gen in range(generations):
            # Evaluate fitness
            # Sort all routes from best (lowest fitness) to worst (highest fitness).
            population.sort(key=lambda ind: self.fitness(ind))
            
            # Select top 50%
            next_gen = population[:pop_size//2]
            
            # Crossover and Mutation to fill the rest
            # Continue creating new routes until the next generation is full.
            while len(next_gen) < pop_size:
                # Randomly select two parent routes from the best half.
                p1, p2 = random.sample(population[:pop_size//2], 2)
                
                # Simple ordered crossover
                #The program randomly chooses a position in the first parent route. If the parent contains only one sensor, it uses cut = 1
                cut = random.randint(1, len(p1)-1) if len(p1) > 1 else 1
                #p1[:cut] takes the first part of parent 1 and stores it in child.
                child = p1[:cut]

                #exampleParent 1: [A, B, C, D, E]
                #Cut:       3
                #Child:    [A, B, C]

                # Add the sensors from parent 2 that are not already in the child.
                # This helps ensure that a sensor is not visited twice.
                for gene in p2:
                    if gene not in child:
                        child.append(gene)
                        
                # Mutation (swap)
                # With a 10% probability, make a small random change to the child.
                if random.random() < 0.1 and len(child) > 1:
                     # Choose two different positions in the route.
                    i, j = random.sample(range(len(child)), 2)
                      # Swap the sensors at these positions.
                    child[i], child[j] = child[j], child[i]
                    
                next_gen.append(child)# Add the newly created route to the next generation.
                
            population = next_gen# Replace the old population with the new generation.
            
        population.sort(key=lambda ind: self.fitness(ind))# Sort the final population from best to worst route.
        return population[0] # Best sequence# Return the route with the lowest fitness score.
