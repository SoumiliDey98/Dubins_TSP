import numpy as np
from scipy.linalg import eigh

def geodesic_distance(env, s1, s2):
    # This should be obstacle-respecting shortest path.
    # For now, approximate with Euclidean distance as a placeholder.
    # In a full implementation, you would use visibility graphs or A* around obstacles.
    return np.sqrt((s1.x - s2.x)**2 + (s1.y - s2.y)**2)

def estimate_workload(env, partition):
    # T_hat(A_t) estimated workload.
    # We can estimate it by the Euclidean TSP length of the partition + stationary times.
    if not partition:
        return 0
    # Simple estimation: 
    # Workload = sum of stationary times + (sum of distances to base station + inter-sensor) / v
    # A better heuristic is the fitness of the Radially Unimodal Heuristic divided by v.
    
    # We use a quick proxy for workload estimation:
    # 2 * max(dist to BS) + sum(dist between consecutive sorted by angle) + sum(tau)
    # The paper's GA heuristic fitness is also a good proxy.
    
    # Let's use the GA heuristic fitness directly as an estimate of travel length
    from ga_heuristic import GA_Heuristic
    ga = GA_Heuristic(env)
    
    # Just sort by polar angle to get a quick valid TSP-like tour length
    partition_sorted = sorted(partition, key=lambda s: env.get_polar_angle(s))
    
    length = 0
    if len(partition_sorted) > 0:
        length += np.sqrt((partition_sorted[0].x - env.base_station[0])**2 + (partition_sorted[0].y - env.base_station[1])**2)
        for i in range(len(partition_sorted) - 1):
            length += np.sqrt((partition_sorted[i+1].x - partition_sorted[i].x)**2 + (partition_sorted[i+1].y - partition_sorted[i].y)**2)
        length += np.sqrt((partition_sorted[-1].x - env.base_station[0])**2 + (partition_sorted[-1].y - env.base_station[1])**2)
        
    harvest_time = sum([s.tau for s in partition])
    
    return (length / env.v) + harvest_time

def geodesic_spectral_partitioning(env, sensors, sigma=1.0):
    n = len(sensors)
    if n <= 1:
        return [sensors], []
        
    # 1. Geodesic Distance Matrix
    D_geo = np.zeros((n, n))
    for i in range(n):
        for j in range(n):
            if i != j:
                D_geo[i, j] = geodesic_distance(env, sensors[i], sensors[j])
                
    # 2. Affinity Matrix
    W = np.exp(-(D_geo**2) / (sigma**2))
    np.fill_diagonal(W, 0)
    
    # 3. Degree Matrix
    d = np.sum(W, axis=1)
    # Handle isolated nodes (d=0) to prevent division by zero
    d[d == 0] = 1e-10
    
    # 4. Normalized Laplacian
    D_inv_sqrt = np.diag(1.0 / np.sqrt(d))
    L_norm = np.eye(n) - D_inv_sqrt @ W @ D_inv_sqrt
    
    # 5 & 6. Eigenvalues and Eigenvectors -> Fiedler Vector
    # eigh returns sorted eigenvalues and corresponding eigenvectors
    eigenvalues, eigenvectors = eigh(L_norm)
    fiedler_vector = eigenvectors[:, 1] # Second smallest eigenvalue
    
    # 7. Sensor Partitioning
    best_t = None
    min_max_workload = float('inf')
    best_A, best_B = [], []
    
    # Candidate thresholds can be the values in the Fiedler vector itself
    thresholds = np.unique(fiedler_vector)
    
    for t in thresholds:
        A_t = [sensors[i] for i in range(n) if fiedler_vector[i] <= t]
        B_t = [sensors[i] for i in range(n) if fiedler_vector[i] > t]
        
        if not A_t or not B_t:
            continue
            
        workload_A = estimate_workload(env, A_t)
        workload_B = estimate_workload(env, B_t)
        
        max_workload = max(workload_A, workload_B)
        
        if max_workload < min_max_workload:
            min_max_workload = max_workload
            best_t = t
            best_A = A_t
            best_B = B_t
            
    # If partitioning failed (e.g. all same value), just split in half
    if not best_A or not best_B:
        mid = n // 2
        best_A = sensors[:mid]
        best_B = sensors[mid:]
        
    return best_A, best_B
