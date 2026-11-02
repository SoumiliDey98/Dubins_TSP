import numpy as np

def resolve_collisions(env, all_trajectories, safe_dist=3.0):
    """
    Simulates the paths frame-by-frame. If a lower-priority robot gets within 
    `safe_dist` of a higher-priority robot, it yields by pausing for one frame.
    To prevent infinite deadlocks in narrow corridors, it forces a move after 50 yields.
    """
    print("Resolving inter-robot collisions...")
    keys = sorted(list(all_trajectories.keys()))
    
    frame = 1
    wait_counts = {k: 0 for k in keys}
    
    while True:
        active = [k for k in keys if frame < len(all_trajectories[k])]
        if len(active) < 2:
            break
            
        for i in range(len(keys)):
            k1 = keys[i]
            if frame >= len(all_trajectories[k1]):
                continue
                
            p1 = np.array(all_trajectories[k1][frame])
            collision = False
            
            for j in range(i):
                k2 = keys[j]
                # If the other robot has finished its trajectory, ignore it
                if frame >= len(all_trajectories[k2]):
                    continue
                    
                p2 = np.array(all_trajectories[k2][frame])
                
                # Check collision (ignore if both are near the base station)
                if np.linalg.norm(p1 - p2) < safe_dist:
                    if np.linalg.norm(p1 - env.base_station) > 5.0 or np.linalg.norm(p2 - env.base_station) > 5.0:
                        collision = True
                        break
                        
            if collision and wait_counts[k1] < 50: # Avoid infinite deadlock
                # Yield to higher-priority robot by waiting
                all_trajectories[k1].insert(frame, all_trajectories[k1][frame-1])
                wait_counts[k1] += 1
            else:
                wait_counts[k1] = 0
                
        frame += 1
        
    return all_trajectories

