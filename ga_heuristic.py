import numpy as np
import random

class GA_Heuristic:
    def __init__(self, env):
        self.env = env
        
    def generate_initial_population(self, sensors, pop_size=50):
        # 1. Polar Angle Evaluation (done in env, but let's sort them if needed)
        # 2. Random Partition
        population = []
        for _ in range(pop_size):
            s_out = []
            s_in = []
            for s in sensors:
                if random.random() < 0.5:
                    s_out.append(s)
                else:
                    s_in.append(s)
                    
            # 3. Monotonic Sorting (The Heuristic)
            # Sort S_out strictly ascending order of radial distance
            s_out.sort(key=lambda x: self.env.get_radial_distance(x))
            
            # Sort S_in strictly descending order of radial distance
            s_in.sort(key=lambda x: self.env.get_radial_distance(x), reverse=True)
            
            # 4. Chromosome Assembly
            # pi = (S_0, pi_out, pi_in, S_0)
            chromosome = s_out + s_in
            population.append(chromosome)
            
        return population
        
    def fitness(self, chromosome):
        # Fitness(pi) = L_euclid(pi) + m(pi) * pi * R
        # L_euclid:
        l_euclid = 0
        
        # S_0 to first
        if len(chromosome) > 0:
            l_euclid += np.sqrt((chromosome[0].x - self.env.base_station[0])**2 + (chromosome[0].y - self.env.base_station[1])**2)
            
            for i in range(len(chromosome) - 1):
                l_euclid += np.sqrt((chromosome[i+1].x - chromosome[i].x)**2 + (chromosome[i+1].y - chromosome[i].y)**2)
                
            # last to S_0
            l_euclid += np.sqrt((chromosome[-1].x - self.env.base_station[0])**2 + (chromosome[-1].y - self.env.base_station[1])**2)
            
        # m(pi) is the number of radial maxima.
        # Since we strictly sort S_out ascending and S_in descending, the sequence reaches a maximum distance exactly once.
        # This mathematically guarantees m(pi) = 1 for the INITIAL population.
        # However, GA operations (crossover/mutation) might change it, so we need a function to count m(pi).
        m_pi = self.count_radial_maxima(chromosome)
        
        return l_euclid + m_pi * np.pi * self.env.R
        
    def count_radial_maxima(self, chromosome):
        if len(chromosome) <= 1:
            return 1
            
        distances = [self.env.get_radial_distance(s) for s in chromosome]
        maxima_count = 0
        
        # Check peaks in the sequence (including bounds conceptually if they drop to 0 at BS)
        # BS distance is 0. 
        full_dists = [0] + distances + [0]
        for i in range(1, len(full_dists) - 1):
            if full_dists[i] > full_dists[i-1] and full_dists[i] > full_dists[i+1]:
                maxima_count += 1
                
        # If it's a plateau, it's slightly more complex, but let's assume strictly greater for peak.
        if maxima_count == 0 and len(distances) > 0:
            maxima_count = 1 # at least one peak if there are sensors
            
        return maxima_count

    def optimize(self, sensors, pop_size=50, generations=100):
        if not sensors:
            return []
            
        population = self.generate_initial_population(sensors, pop_size)
        
        for gen in range(generations):
            # Evaluate fitness
            population.sort(key=lambda ind: self.fitness(ind))
            
            # Select top 50%
            next_gen = population[:pop_size//2]
            
            # Crossover and Mutation to fill the rest
            while len(next_gen) < pop_size:
                p1, p2 = random.sample(population[:pop_size//2], 2)
                
                # Simple ordered crossover
                cut = random.randint(1, len(p1)-1) if len(p1) > 1 else 1
                child = p1[:cut]
                for gene in p2:
                    if gene not in child:
                        child.append(gene)
                        
                # Mutation (swap)
                if random.random() < 0.1 and len(child) > 1:
                    i, j = random.sample(range(len(child)), 2)
                    child[i], child[j] = child[j], child[i]
                    
                next_gen.append(child)
                
            population = next_gen
            
        population.sort(key=lambda ind: self.fitness(ind))
        return population[0] # Best sequence
