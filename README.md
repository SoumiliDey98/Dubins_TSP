# Multi-Agent Dubins TSP with Obstacle Avoidance & Collision Resolution

## Project Overview

This project simulates a fleet of Mobile Sinks (robots or drones) assigned to collect data from a large number of static sensors (e.g., 100 sensors) scattered in an environment filled with complex polygonal obstacles. 

The primary goal of the system is to calculate optimal, collision-free paths for all mobile sinks such that they start from a base station, visit their assigned sensors, and return to the base station. The paths must obey the kinematic constraints of the mobile sinks (they can't make sharp turns, so we use **Dubins curves**). Furthermore, each robot has a strict time budget ($B$) to complete its tour. 

When the environment requires multiple robots, they might cross paths. This project elegantly resolves multi-robot collisions using **Game Theory (Nash Equilibrium)**, specifically treating narrow passages between obstacles as a "Corridor Game".

---

## Architecture in Simple Terms

Imagine you are managing a fleet of delivery drones:
1. **The Map & Obstacles**: First, we create the map (`Environment`), which places random obstacles (like buildings) and sensors (delivery targets) across a grid.
2. **Assigning Targets (Partitioning)**: If one drone cannot visit all targets within its battery/time budget, we split the map using a mathematical technique called "Geodesic Spectral Partitioning." This groups sensors that are close to each other into distinct clusters, assigning each cluster to a different drone.
3. **Ordering the Targets (TSP + Heuristic)**: For each drone, we must find the shortest sequence to visit its targets. We apply an intelligent heuristic that forces drones to fly radially outward, hit the farthest target, and come radially back. We then refine this order using a Genetic Algorithm.
4. **Drawing the Path (Dubins + A*)**: A straight line might cut through a building. So, we use the A* algorithm (a pathfinding method) to find the shortest path *around* the buildings. Because the drones can't turn on a dime, we smooth these corners using Dubins curves (curves that obey the drone's turning radius).
5. **Avoiding Each Other (Game Theory)**: Finally, if two drones are about to crash into each other, they play a mathematical "game". Each drone evaluates its "Urgency" (how long its current route is, plus how long it has been waiting). To achieve a Socially Optimal Nash Equilibrium, the drone that is in less of a hurry will "Yield" (wait in place), while the more urgent drone gets to "Go".

---

## Explanation of Every Code File

### 1. `main.py`
**Purpose**: The entry point of the project.
**Working**: It initializes the map, randomly generates 100+ static sensors (spaced uniformly apart) and 10-15 complex convex/concave obstacles. It orchestrates the overall algorithm: partitioning the sensors, validating time budgets, and calling the visualizer to animate the result.

### 2. `environment.py`
**Purpose**: Defines the physical space.
**Working**: Contains the `Environment`, `Sensor`, and `Obstacle` classes. It manages the mathematical bounds of the obstacles, checks for overlapping geometries, and defines properties like the Base Station location, the robots' velocity, and turning radius.

### 3. `partitioning.py`
**Purpose**: Divides the workload among multiple robots.
**Working**: If a route exceeds the robot's time budget, this file splits the sensors into two smaller groups. It calculates the physical (geodesic) distances between sensors and uses a technique called Spectral Partitioning to intelligently split the group in half, minimizing the distance between targets in the new groups.

### 4. `ga_heuristic.py`
**Purpose**: Determines the order in which a robot visits its sensors (The Traveling Salesperson Problem).
**Working**: First, it applies a Unimodal Radial Heuristic: sort the outbound path by ascending distance from the base, go to the absolute farthest sensor (apex), and sort the inbound path by descending distance. It then uses a Genetic Algorithm (simulating evolution) to swap and mutate this sequence over multiple generations to find an even faster order.

### 5. `dubins.py`
**Purpose**: Handles physical drone movement (A* Pathfinding + Dubins Curves).
**Working**: It calculates how the robot actually flies from Sensor A to Sensor B. It builds a "Visibility Graph" (lines connecting the corners of obstacles) and runs A* search to find the shortest path around the obstacles. It then wraps these straight lines with Dubins curves so the robot makes smooth, realistic turns.

### 6. `corridor_game.py`
**Purpose**: Resolves multi-robot collisions dynamically.
**Working**: Instead of just using a priority list, it treats collisions as a Game Theory problem (Nash Equilibrium). When two robots come within a unsafe distance, it calculates their "Urgency" (total path length + penalty for waiting). The robot with the lower urgency mathematically "Yields" (pauses), and the robot with higher urgency "Goes", preventing deadlocks and minimizing the overall societal delay.

### 7. `collision_resolution.py`
**Purpose**: Legacy collision avoidance logic (Kept for historical reference).
**Working**: The original collision resolver that simply forced higher-ID robots to yield to lower-ID robots. It has since been superseded by the superior `corridor_game.py`.

### 8. `visualize.py`
**Purpose**: Renders the simulation to a GIF.
**Working**: Takes all the calculated trajectories, collision wait-states, and obstacle geometries, and uses `matplotlib` to draw a frame-by-frame animation (`dubins_tsp_animation.gif`). It visually demonstrates the A* path hugging, the Dubins smooth curves, and the robots pausing to yield for one another.
