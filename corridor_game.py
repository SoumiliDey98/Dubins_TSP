import numpy as np

def game_theory_resolve(env, all_trajectories, safe_dist=3.0):
    """
    Resolves collisions using a Game Theory approach (Nash Equilibrium).
    When two robots arrive at a bottleneck (distance < safe_dist), they play a game.
    Strategies: {Go, Yield}
    Cost of Yielding (w) is proportional to how close the robot is to its maximum time budget.
    The Nash Equilibrium that minimizes social cost dictates that the robot with the 
    lower urgency (shorter overall path) Yields, while the more urgent robot Goes.
    """
    print("Resolving collisions using Game Theory (Nash Equilibrium)...")
    keys = sorted(list(all_trajectories.keys()))
    
    frame = 1
    wait_counts = {k: 0 for k in keys}
    
    while True:
        if frame % 500 == 0:
            print(f"Game Theory: Processing frame {frame}...", end='\r')
            
        active = [k for k in keys if frame < len(all_trajectories[k])]
        if len(active) < 2:
            break
            
        # We need to find all colliding pairs at this frame
        # To handle multiple simultaneous, we collect intended actions
        strategies = {k: "Go" for k in active}
        
        for i in range(len(keys)):
            k1 = keys[i]
            if frame >= len(all_trajectories[k1]):
                continue
                
            p1 = np.array(all_trajectories[k1][frame])
            
            for j in range(i + 1, len(keys)):
                k2 = keys[j]
                if frame >= len(all_trajectories[k2]):
                    continue
                    
                p2 = np.array(all_trajectories[k2][frame])
                
                # Check collision (ignore if both are near the base station)
                if np.linalg.norm(p1 - p2) < safe_dist:
                    if np.linalg.norm(p1 - env.base_station) > 5.0 or np.linalg.norm(p2 - env.base_station) > 5.0:
                        
                        # GAME THEORY MECHANICS:
                        # Define cost of yielding based on urgency.
                        # Urgency = current total length of trajectory (approximates time spent).
                        len1 = len(all_trajectories[k1])
                        len2 = len(all_trajectories[k2])
                        
                        # If a robot has waited too long, its cost of yielding approaches infinity
                        if wait_counts[k1] > 50:
                            w1 = float('inf')
                        else:
                            w1 = len1 + wait_counts[k1] * 10
                            
                        if wait_counts[k2] > 50:
                            w2 = float('inf')
                        else:
                            w2 = len2 + wait_counts[k2] * 10
                            
                        # Find Socially Optimal Nash Equilibrium
                        # The robot with lower cost of yielding should yield.
                        if w1 < w2:
                            strategies[k1] = "Yield"
                            strategies[k2] = "Go"
                        elif w2 < w1:
                            strategies[k2] = "Yield"
                            strategies[k1] = "Go"
                        else:
                            # Tie-breaker (arbitrary convention to break symmetry)
                            strategies[k1] = "Yield"
                            strategies[k2] = "Go"

        # Apply Strategies
        for k in active:
            if strategies[k] == "Yield":
                # Duplicate the previous frame's position to simulate waiting
                all_trajectories[k].insert(frame, all_trajectories[k][frame-1])
                wait_counts[k] += 1
            else:
                wait_counts[k] = 0
                
        frame += 1
        
    return all_trajectories

