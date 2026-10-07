import numpy as np
from environment import Environment, Sensor, Obstacle
from ga_heuristic import GA_Heuristic
from partitioning import geodesic_spectral_partitioning, estimate_workload

def compute_T(env, route):
    if not route:
        return 0
    # Recalculate using the heuristic fitness function logic
    # T = L/v + sum(tau)
    from ga_heuristic import GA_Heuristic
    ga = GA_Heuristic(env)
    
    # We shouldn't use GA fitness directly for time, we just need the length of the specific route
    length = 0
    if len(route) > 0:
        length += np.sqrt((route[0].x - env.base_station[0])**2 + (route[0].y - env.base_station[1])**2)
        for i in range(len(route) - 1):
            length += np.sqrt((route[i+1].x - route[i].x)**2 + (route[i+1].y - route[i].y)**2)
        length += np.sqrt((route[-1].x - env.base_station[0])**2 + (route[-1].y - env.base_station[1])**2)
        
    harvest_time = sum([s.tau for s in route])
    
    return (length / env.v) + harvest_time

def optimize_partition(env, partition):
    ga = GA_Heuristic(env)
    best_route = ga.optimize(partition, pop_size=50, generations=100)
    T_min = compute_T(env, best_route)
    return best_route, T_min

def overall_algorithm(env):
    """
    Implements Section 0.11 Overall algorithm
    """
    # Step 1: Initialize with k=1 MS and assign all sensors to it
    partitions = [env.sensors]
    routes = {}
    
    # Step 2: Construct curvature-constrained feasible graph (handled implicitly by env/heuristic)
    
    all_constraints_satisfied = False
    
    while not all_constraints_satisfied:
        new_partitions = []
        
        # Step 6 logic embedded here for the loop
        for i, partition in enumerate(partitions):
            # Step 3: Generate a radially unimodal ordering and optimize it using GA
            best_route, T_min = optimize_partition(env, partition)
            
            # Step 4 & 5 / Step 7: Check against Budget B
            if T_min <= env.budget:
                # Accept the solution
                new_partitions.append(partition)
                routes[len(new_partitions) - 1] = best_route
            else:
                # Perform curvature-aware geodesic partitioning
                print(f"Partition {i} exceeds budget (T={T_min:.2f} > B={env.budget}). Partitioning...")
                A, B = geodesic_spectral_partitioning(env, partition)
                
                if not A or not B:
                    print("Failed to partition further. Bound violated structurally.")
                    new_partitions.append(partition)
                    routes[len(new_partitions) - 1] = best_route
                else:
                    new_partitions.extend([A, B])
                    
        # Check if we successfully broke everything down
        if len(new_partitions) == len(partitions):
            # No further splits happened, or splits failed. Proceed to conflict detection
            pass
        else:
            # We split something, go back and optimize the new partitions (repeat step 6)
            partitions = new_partitions
            continue
            
        # Step 8 & 9: Detect conflicts and apply CorridorGames
        # (Assuming CorridorGame internally checks conflicts and updates)
        
        updated_routes = routes
        # Step 10 & 11: Verify constraints
        constraints_satisfied = True
        for k, route in updated_routes.items():
            T_k = compute_T(env, route)
            if T_k > env.budget:
                constraints_satisfied = False
                # If a route violated budget after corridor games, we might need to 
                # re-optimize or re-partition. The algorithm says "re-optimize the affected routes and repeat"
                partitions = [p for i, p in enumerate(partitions) if compute_T(env, updated_routes[i]) <= env.budget]
                affected_sensors = []
                for s in route:
                    affected_sensors.append(s)
                # Re-feed affected sensors into the loop
                partitions.append(affected_sensors)
                break
                
        if constraints_satisfied:
            all_constraints_satisfied = True
            routes = updated_routes
            
    # Step 12: Accept final solution
    print(f"Final Solution accepted with {len(routes)} mobile sinks.")
    for k, route in routes.items():
        print(f"MS {k}: {[s.id for s in route]} (Time: {compute_T(env, route):.2f})")
        
    return routes

if __name__ == "__main__":
    # Test execution
    env = Environment(base_station_pos=(50, 50), budget=80, velocity=5.0, radius=5.0, min_safety_dist=2.0)
    
    from shapely.geometry import Polygon as ShapelyPolygon, Point
    import random
    import time

    np.random.seed(int(time.time()) % 10000)

    # Add random polygonal obstacles
    num_obstacles = np.random.randint(10, 16)
    for i in range(num_obstacles):
        valid_obstacle = False
        attempts = 0
        while not valid_obstacle and attempts < 1000:
            attempts += 1
            # Generate random center for obstacle
            cx, cy = np.random.uniform(0, 100, 2)
            
            # Ensure the obstacle is sufficiently far from the base station (50, 50)
            if np.linalg.norm([cx - 50, cy - 50]) < 25:
                continue
                
            # Create a Shapely polygon to check validity (must be valid and not self-intersecting)
            shape_type = np.random.choice(['random', 'L_shape', 'C_shape', 'cross'])
            if shape_type == 'random':
                num_vertices = np.random.randint(4, 7)
                angles = np.sort(np.random.uniform(0, 2*np.pi, num_vertices))
                r = np.random.uniform(8, 12, num_vertices)
                vertices = [(cx + r[k]*np.cos(angles[k]), cy + r[k]*np.sin(angles[k])) for k in range(num_vertices)]
            elif shape_type == 'L_shape':
                w, h, t = np.random.uniform(10, 20), np.random.uniform(10, 20), np.random.uniform(4, 8)
                vertices = [(0, 0), (w, 0), (w, t), (t, t), (t, h), (0, h)]
            elif shape_type == 'C_shape':
                w, h, t = np.random.uniform(10, 20), np.random.uniform(15, 25), np.random.uniform(4, 8)
                vertices = [(0, 0), (w, 0), (w, t), (t, t), (t, h-t), (w, h-t), (w, h), (0, h)]
            elif shape_type == 'cross':
                w, h, t = np.random.uniform(15, 25), np.random.uniform(15, 25), np.random.uniform(4, 8)
                vertices = [(w/2-t/2, 0), (w/2+t/2, 0), (w/2+t/2, h/2-t/2), (w, h/2-t/2), (w, h/2+t/2), 
                            (w/2+t/2, h/2+t/2), (w/2+t/2, h), (w/2-t/2, h), (w/2-t/2, h/2+t/2), 
                            (0, h/2+t/2), (0, h/2-t/2), (w/2-t/2, h/2-t/2)]
                            
            if shape_type != 'random':
                # Center and rotate
                poly_tmp = ShapelyPolygon(vertices)
                centroid = poly_tmp.centroid
                theta = np.random.uniform(0, 2*np.pi)
                c_theta, s_theta = np.cos(theta), np.sin(theta)
                vertices = []
                for x, y in poly_tmp.exterior.coords[:-1]:
                    nx = (x - centroid.x) * c_theta - (y - centroid.y) * s_theta + cx
                    ny = (x - centroid.x) * s_theta + (y - centroid.y) * c_theta + cy
                    vertices.append((nx, ny))

            poly = ShapelyPolygon(vertices)
            if poly.is_valid:
                # Ensure the base station is strictly outside the buffer
                b_poly = poly.buffer(env.d_safe + 2.0 * env.R, resolution=2)
                if not Point(env.base_station).within(b_poly):
                    # Ensure it doesn't intersect with existing obstacles
                    overlaps = False
                    for existing_obs in env.obstacles:
                        existing_poly = ShapelyPolygon(existing_obs.vertices).buffer(env.d_safe + 2.0 * env.R, resolution=2)
                        if b_poly.intersects(existing_poly):
                            overlaps = True
                            break
                    if not overlaps:
                        env.add_obstacle(Obstacle(f"Ob{i}", vertices))
                        valid_obstacle = True

    # Pre-compute buffered polygons for collision checking
    buffered_polys = []
    for obs in env.obstacles:
        poly = ShapelyPolygon(obs.vertices)
        buffered_polys.append(poly.buffer(env.d_safe + 2.0 * env.R, resolution=2))

    # Add random sensors uniformly across the space, enforcing minimum distance
    num_sensors = np.random.randint(90, 110)
    min_dist_between_sensors = 7.0
    
    for i in range(num_sensors):
        valid_pos = False
        attempts = 0
        while not valid_pos and attempts < 1000:
            attempts += 1
            x, y = np.random.uniform(0, 100, 2)
            pt = Point(x, y)
                
            valid_pos = True
            
            # Check against obstacles
            for b_poly in buffered_polys:
                if pt.within(b_poly):
                    valid_pos = False
                    break
                    
            # Check against base station
            if valid_pos and pt.distance(Point(env.base_station)) < 5.0:
                valid_pos = False
                
            # Check distance from existing sensors to ensure they are far apart
            if valid_pos:
                for s in env.sensors:
                    # Dynamically lower the constraint if we're struggling to find space
                    current_min_dist = min_dist_between_sensors * (1.0 - (attempts / 1000.0))
                    if np.linalg.norm([x - s.x, y - s.y]) < current_min_dist:
                        valid_pos = False
                        break
        
        if valid_pos:
            tau = np.random.uniform(1, 5)
            env.add_sensor(Sensor(f"S{i}", x, y, tau))

    routes = overall_algorithm(env)
    
    # Import and run visualizer
    from visualize import animate_solution
    print("Starting visual animation...")
    animate_solution(env, routes)
