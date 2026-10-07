import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.patches import Polygon
from shapely.geometry import Polygon as ShapelyPolygon
from shapely.geometry import LineString, Point
import networkx as nx

from dubins import generate_dubins_path

def build_visibility_graph(env):
    vertices = [env.base_station]
    for s in env.sensors:
        vertices.append((s.x, s.y))
        
    polygons = []
    for obs in env.obstacles:
        poly = ShapelyPolygon(obs.vertices)
        buffered = poly.buffer(env.d_safe + 2.0 * env.R, resolution=2)
        polygons.append(buffered)
        if hasattr(buffered, 'exterior'):
            for coord in list(buffered.exterior.coords)[:-1]:
                vertices.append(coord)
                
    G = nx.Graph()
    for i, v1 in enumerate(vertices):
        G.add_node(i, pos=v1)
        
    for i in range(len(vertices)):
        for j in range(i + 1, len(vertices)):
            v1, v2 = vertices[i], vertices[j]
            line = LineString([v1, v2])
            
            intersects = False
            for poly in polygons:
                if line.crosses(poly) or line.within(poly):
                    intersects = True
                    break
                    
            if not intersects:
                dist = np.linalg.norm(np.array(v1) - np.array(v2))
                G.add_edge(i, j, weight=dist)
                
    return G, vertices

def shortest_path_avoiding_obstacles(G, vertices, p1, p2):
    n1 = min(G.nodes, key=lambda n: np.linalg.norm(np.array(G.nodes[n]['pos']) - np.array(p1)))
    n2 = min(G.nodes, key=lambda n: np.linalg.norm(np.array(G.nodes[n]['pos']) - np.array(p2)))
    
    def heuristic(n, target):
        pos1 = np.array(G.nodes[n]['pos'])
        pos2 = np.array(G.nodes[target]['pos'])
        return np.linalg.norm(pos1 - pos2)
    
    try:
        path_indices = nx.astar_path(G, source=n1, target=n2, heuristic=heuristic, weight='weight')
        return [G.nodes[i]['pos'] for i in path_indices]
    except nx.NetworkXNoPath:
        return [p1, p2]


def interpolate_constant_velocity(points, v, dt):
    if len(points) < 2:
        return points
    
    interpolated = [points[0]]
    step = v * dt
    
    curr_idx = 0
    curr_pos = np.array(points[0])
    
    while curr_idx < len(points) - 1:
        next_pos = np.array(points[curr_idx + 1])
        dist = np.linalg.norm(next_pos - curr_pos)
        
        if dist < step:
            step -= dist
            curr_pos = next_pos
            curr_idx += 1
        else:
            direction = (next_pos - curr_pos) / dist
            curr_pos = curr_pos + direction * step
            interpolated.append((curr_pos[0], curr_pos[1]))
            step = v * dt
            
    interpolated.append(points[-1])
    return interpolated


def generate_interpolated_trajectory(env, route, dt, G, G_vertices):

    trajectory = []
    if not route:
        return trajectory
        
    waypoints = [env.base_station]
    dwells = [0.0]
    
    for s in route:
        waypoints.append((s.x, s.y))
        dwells.append(s.tau)
        
    waypoints.append(env.base_station)
    dwells.append(0.0)
    
    current_heading = 0.0
    
    for i in range(len(waypoints) - 1):
        # 1. Dwell time
        dwell_time = dwells[i]
        num_dwell_steps = int(dwell_time / dt)
        if len(trajectory) > 0:
            last_pos = trajectory[-1]
            for _ in range(num_dwell_steps):
                trajectory.append(last_pos)
        else:
            for _ in range(num_dwell_steps):
                trajectory.append(waypoints[i])
                
        # 2. Path to next waypoint avoiding obstacles
        start_wp = waypoints[i]
        target_wp = waypoints[i+1]
        
        obstacle_free_path = shortest_path_avoiding_obstacles(G, G_vertices, start_wp, target_wp)
        
        # Simplify the path slightly by removing very close intermediate points to prevent Dubins backflips
        simplified_path = [obstacle_free_path[0]]
        for pt in obstacle_free_path[1:-1]:
            if np.linalg.norm(np.array(pt) - np.array(simplified_path[-1])) > env.R:
                simplified_path.append(pt)
        if np.linalg.norm(np.array(obstacle_free_path[-1]) - np.array(simplified_path[-1])) > 1e-3:
            simplified_path.append(obstacle_free_path[-1])
            
        headings = []
        for j in range(len(simplified_path)):
            if j == 0:
                headings.append(current_heading)
            elif j == len(simplified_path) - 1:
                # Heading to reach the target depends on the last segment
                p_prev = simplified_path[j-1]
                p_curr = simplified_path[j]
                headings.append(np.arctan2(p_curr[1] - p_prev[1], p_curr[0] - p_prev[0]))
            else:
                # Intermediate heading is the angle of the sum of incoming and outgoing vectors
                p_prev = np.array(simplified_path[j-1])
                p_curr = np.array(simplified_path[j])
                p_next = np.array(simplified_path[j+1])
                
                v_in = p_curr - p_prev
                v_out = p_next - p_curr
                n_in = np.linalg.norm(v_in)
                n_out = np.linalg.norm(v_out)
                
                if n_in > 0 and n_out > 0:
                    v_bisect = (v_in / n_in) + (v_out / n_out)
                    headings.append(np.arctan2(v_bisect[1], v_bisect[0]))
                else:
                    headings.append(headings[-1])
        
        for j in range(len(simplified_path) - 1):
            p_start = simplified_path[j]
            p_end = simplified_path[j+1]
            
            q0 = (p_start[0], p_start[1], headings[j])
            q1 = (p_end[0], p_end[1], headings[j+1])
            
            dubins_points = generate_dubins_path(q0, q1, env.R)

            smooth_points = interpolate_constant_velocity(dubins_points, env.v, dt)
            trajectory.extend(smooth_points)
            
            current_heading = headings[j+1]
            
    return trajectory

def animate_solution(env, routes):
    print("Building visibility graph for obstacle avoidance...")
    G, G_vertices = build_visibility_graph(env)
    
    fig, ax = plt.subplots(figsize=(10, 10))
    ax.set_title("Dubins Trajectory with Obstacle Avoidance")
    ax.set_xlim(-10, 110)
    ax.set_ylim(-10, 110)
    
    ax.plot(env.base_station[0], env.base_station[1], 'r^', markersize=15, label='Base Station', zorder=10)
    
    for obs in env.obstacles:
        p = Polygon(obs.vertices, closed=True, fill=True, color='gray', alpha=0.7)
        ax.add_patch(p)
        
        # Draw safety boundary
        poly_safe = ShapelyPolygon(obs.vertices).buffer(env.d_safe, resolution=2)
        if hasattr(poly_safe, 'exterior'):
            x_safe, y_safe = poly_safe.exterior.xy
            label = 'Safety Boundary' if obs == env.obstacles[0] else ""
            ax.plot(x_safe, y_safe, color='red', linestyle='--', linewidth=1.5, alpha=0.7, label=label)
        
    colors = ['blue', 'green', 'orange', 'purple', 'cyan', 'magenta', 'yellow', 'brown']
    
    all_trajectories = {}
    dt = 0.5 
    max_steps = 0
    
    print("Computing exact Dubins paths for visualization...")
    for k, route in routes.items():
        c = colors[k % len(colors)]
        
        xs = [s.x for s in route]
        ys = [s.y for s in route]
        ax.scatter(xs, ys, color=c, s=50, label=f'Partition {k}', zorder=5)
        for s in route:
            ax.annotate(s.id, (s.x+1, s.y+1), fontsize=8, color='black', zorder=6)
            
        traj = generate_interpolated_trajectory(env, route, dt, G, G_vertices)
        
        if len(traj) > 0:
            tx = [p[0] for p in traj]
            ty = [p[1] for p in traj]
            ax.plot(tx, ty, color=c, linestyle='-', alpha=0.4)
            
        all_trajectories[k] = traj
        
    # Resolve Inter-Robot Collisions using Game Theory
    from corridor_game import game_theory_resolve
    all_trajectories = game_theory_resolve(env, all_trajectories, safe_dist=3.0)

    max_steps = max((len(t) for t in all_trajectories.values()), default=0)
    for k in all_trajectories:
        traj = all_trajectories[k]
        if len(traj) < max_steps:
            last_pos = traj[-1] if traj else env.base_station
            padding = [last_pos] * (max_steps - len(traj))
            all_trajectories[k].extend(padding)

    # Dynamic Bounds Calculation
    all_x = [env.base_station[0]]
    all_y = [env.base_station[1]]
    for obs in env.obstacles:
        for x, y in obs.vertices:
            all_x.append(x)
            all_y.append(y)
    for traj in all_trajectories.values():
        for x, y in traj:
            all_x.append(x)
            all_y.append(y)
            
    min_x, max_x = min(all_x) - 5, max(all_x) + 5
    min_y, max_y = min(all_y) - 5, max(all_y) + 5
    ax.set_xlim(min_x, max_x)
    ax.set_ylim(min_y, max_y)

    ax.legend(loc='upper right')
    ax.grid(True)
    
    ms_plots = {}
    for k in routes.keys():
        c = colors[k % len(colors)]
        ms_plots[k], = ax.plot([], [], marker='o', color=c, markersize=12, markeredgecolor='black', zorder=15)

    time_text = ax.text(0.02, 0.95, '', transform=ax.transAxes, fontsize=12, bbox=dict(facecolor='white', alpha=0.8), zorder=20)

    def init():
        for k in ms_plots:
            ms_plots[k].set_data([], [])
        time_text.set_text('')
        return list(ms_plots.values()) + [time_text]

    def update(frame):
        for k in ms_plots:
            if frame < len(all_trajectories[k]):
                x, y = all_trajectories[k][frame]
                ms_plots[k].set_data([x], [y])
            
        current_time = frame * dt
        time_text.set_text(f'Simulation Time: {current_time:.1f}s')
        return list(ms_plots.values()) + [time_text]

    ani = animation.FuncAnimation(fig, update, frames=max_steps, init_func=init, 
                                  blit=True, interval=50, repeat=False)
                                  
    print("Saving animation to dubins_tsp_animation.gif... (This may take 1-3 minutes)")
    try:
        ani.save('dubins_tsp_animation.gif', writer='pillow', fps=20,
                 progress_callback=lambda i, n: print(f'Saving frame {i}/{n}...', end='\r'))
        print("\nAnimation saved successfully.")
    except Exception as e:
        print(f"Error saving animation: {e}")
