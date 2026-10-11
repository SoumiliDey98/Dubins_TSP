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
    keys = sorted(list(all_trajectories.keys()))#sort all MS ids MS0, MS1, MS2,..
    
    frame = 1#starts with frame 1
    wait_counts = {k: 0 for k in keys}#The counter records how many times the code has made a robot wait. initially its 0 for all MS 
    
    while True:
        if frame % 500 == 0:
            print(f"Game Theory: Processing frame {frame}...", end='\r')#The program prints a progress message every 500 frames
            
        active = [k for k in keys if frame < len(all_trajectories[k])]#creates a list of robots that are active, if frame is <length of trajectory of MS the that MS is active
                                                                    #An MS is considered active if the current frame index is smaller than the number of positions in its trajectory.
        if len(active) < 2:#if only <2 MS are active then break from the loop
            break
            
        # We need to find all colliding pairs at this frame
        # To handle multiple simultaneous, we collect intended actions
        strategies = {k: "Go" for k in active}#This creates a dictionary assigning the initial action "Go" to every active robot.
        
        for i in range(len(keys)):#number of time loop runs=number of robots; that is no of keys
            k1 = keys[i]#the first outer-loop iteration selects Robot A.
            if frame >= len(all_trajectories[k1]):#Robot A has no position at the current frame, skip it.
                continue
                
            p1 = np.array(all_trajectories[k1][frame])#This retrieves the first robot's position at the current frame and converts it into a NumPy array.
            
            for j in range(i + 1, len(keys)):#checking array from i+1th position till the end, why i+1: To avoid checking the same pair twice
                k2 = keys[j]
                if frame >= len(all_trajectories[k2]):If Robot B has no position at this frame, skip it
                    continue
                    
                p2 = np.array(all_trajectories[k2][frame])#This retrieves the second robot's position at the current frame and converts it into a NumPy array.
                
                # Check collision (ignore if both are near the base station)
                if np.linalg.norm(p1 - p2) < safe_dist:#Checking whether the robots are too close by euclidian distance<safe_distance=3
                    if np.linalg.norm(p1 - env.base_station) > 5.0 or np.linalg.norm(p2 - env.base_station) > 5.0:#checking if none of the robots are at a distance of 5units from base station
                        
                        # GAME THEORY MECHANICS:
                        # Define cost of yielding based on urgency.
                        # Urgency = current total length of trajectory (approximates time spent).
                        #These lines count how many positions are currently stored in each robot's trajectory.These numbers are not the actual geometric lengths of the paths.
                        #They count the list entries, which can include repeated positions inserted when a robot waits.
                        len1 = len(all_trajectories[k1])
                        len2 = len(all_trajectories[k2])
                        
                        # If a robot has waited too long, its cost of yielding approaches infinity
                        #If Robot A has waited more than 50 times, its score becomes infinity.
                        #Otherwise, its score is its trajectory-list length plus 10 for every recorded wait.
                        if wait_counts[k1] > 50:
                            w1 = float('inf')
                        else:
                            w1 = len1 + wait_counts[k1] * 10
                        #same calculation is performed for Robot B    
                        if wait_counts[k2] > 50:
                            w2 = float('inf')
                        else:
                            w2 = len2 + wait_counts[k2] * 10
                            
                        # Find Socially Optimal Nash Equilibrium
                        # The robot with lower cost of yielding should yield.
                        #If Robot A's score is smaller than Robot B's score, Robot A waits and Robot B goes
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
        for k in active:#This loop goes through every robot that is active at the current frame.
            if strategies[k] == "Yield":#The next line checks whether a robot was told to wait:
                # Duplicate the previous frame's position to simulate waiting
                all_trajectories[k].insert(frame, all_trajectories[k][frame-1])#Suppose Robot A's original path is:[(0, 0), (1, 0), (2, 0), (3, 0)]f Robot A must wait at frame 2, this instruction inserts the previous position at index 2.
                                                                                #The updated path becomes:[(0, 0), (1, 0), (1, 0), (2, 0), (3, 0)]
                                                                                #The robot does not physically move backwards. The program adds a duplicate position to delay the rest of its recorded path
                wait_counts[k] += 1#This increases the robot's waiting counter by one.
            else:
                wait_counts[k] = 0
                
        frame += 1#Moving to the next frame
        
    return all_trajectories#Once the loop ends, the function returns the updated trajectories.
#The returned paths may contain extra repeated positions where the robots were made to wait.

