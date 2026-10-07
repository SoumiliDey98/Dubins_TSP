import numpy as np
import math

def returnDubinsPath(x1, y1, psi1, x2, y2, psi2, R):
    best_dist = float('inf')
    best_path = (None, None, None)

    for turnType in ["LSL", "RSR", "LSR", "RSL"]:
        if turnType == "LSL":
            sigma1, sigma2 = 1, 1
        elif turnType == "RSR":
            sigma1, sigma2 = -1, -1
        elif turnType == "LSR":
            sigma1, sigma2 = 1, -1
        elif turnType == "RSL":
            sigma1, sigma2 = -1, 1
            
        xc1, yc1 = returnCircleCenter(x1, y1, psi1, sigma1, R)
        xc2, yc2 = returnCircleCenter(x2, y2, psi2, sigma2, R)
        d = distance(xc1, xc2, yc1, yc2)
        
        if turnType in ["LSR", "RSL"] and d < 2*R:
            continue # Invalid
            
        xt, yt, dist = calculateCSCTrajectory(x1, x2, xc1, xc2, y1, y2, yc1, yc2, psi1, psi2, R, turnType, sigma1, sigma2)
        if dist < best_dist:
            best_dist = dist
            best_path = (xt, yt, dist)
            
    for turnType in ["LRL", "RLR"]:
        if turnType == "LRL":
            sigma1, sigma2 = 1, 1
        elif turnType == "RLR":
            sigma1, sigma2 = -1, -1
            
        xc1, yc1 = returnCircleCenter(x1, y1, psi1, sigma1, R)
        xc2, yc2 = returnCircleCenter(x2, y2, psi2, sigma2, R)
        d = distance(xc1, xc2, yc1, yc2)
        
        if d > 4*R:
            continue # Invalid
            
        xt, yt, dist = calculateCCCTrajectory(x1, x2, xc1, xc2, y1, y2, yc1, yc2, psi1, psi2, R, sigma1, sigma2)
        if dist < best_dist:
            best_dist = dist
            best_path = (xt, yt, dist)

    return best_path


def findCSCTurnType(cp1, cp2):
    if cp1 <= 0 and cp2 >= 0:
        turn = "RSL"
        sigma1 = -1
        sigma2 = 1
    elif cp1 <= 0 and cp2 < 0:
        turn = "RSR"
        sigma1 = -1
        sigma2 = -1
    elif cp1 > 0 and cp2 < 0:
        turn = "LSR"
        sigma1 = 1
        sigma2 = -1
    else: # cp1 > 0 and cp2 >= 0
        turn = "LSL"
        sigma1 = 1
        sigma2 = 1
    return sigma1, sigma2, turn


def findCCCTurnType(turnOld):
    proceed = False
    sigma1 = 0
    sigma2 = 0
    turn = turnOld
    if turnOld[0] == turnOld[2]:
        if turnOld[0] == "R":
            turn = "RLR"
            sigma1 = -1
            sigma2 = -1
        elif turnOld[0] == "L":
            turn = "LRL"
            sigma1 = 1
            sigma2 = 1
        proceed = True
    return sigma1, sigma2, turn, proceed


def returnCircleCenter(x, y, psi, sigma, R):
    xc = x + R*np.cos(psi + sigma*np.pi/2)
    yc = y + R*np.sin(psi + sigma*np.pi/2)
    return xc, yc


def distance(x1, x2, y1, y2):
    return np.hypot(x2-x1, y2-y1)


def generate_arc_points(xc, yc, R, start_angle, end_angle, sigma, step_size=0.1):
    delta = (end_angle - start_angle) % (2 * np.pi)
    if sigma == -1 and delta != 0:
        delta -= 2 * np.pi
    
    num_steps = int(np.ceil(abs(delta) / step_size))
    if num_steps == 0:
        return [xc + R * np.cos(start_angle)], [yc + R * np.sin(start_angle)], 0.0
    
    angles = start_angle + np.linspace(0, delta, num_steps + 1)
    xs = xc + R * np.cos(angles)
    ys = yc + R * np.sin(angles)
    distance = abs(delta) * R
    return xs.tolist(), ys.tolist(), distance


def generate_straight_points(x1, y1, x2, y2, step_size=0.5):
    dist = np.hypot(x2 - x1, y2 - y1)
    num_steps = int(np.ceil(dist / step_size))
    if num_steps == 0:
        return [x1], [y1], 0.0
    xs = np.linspace(x1, x2, num_steps + 1)
    ys = np.linspace(y1, y2, num_steps + 1)
    return xs.tolist(), ys.tolist(), dist


def calculateCSCTrajectory(x1, x2, xc1, xc2, y1, y2, yc1, yc2, psi1, psi2, R, turnType, sigma1, sigma2):
    d = distance(xc1, xc2, yc1, yc2)
    theta = np.arctan2(yc2 - yc1, xc2 - xc1)

    if turnType == "LSL":
        a1 = theta - np.pi/2
        a2 = theta - np.pi/2
    elif turnType == "RSR":
        a1 = theta + np.pi/2
        a2 = theta + np.pi/2
    elif turnType == "RSL":
        beta = math.acos(max(-1.0, min(1.0, 2*R / d)))
        a1 = theta + beta
        a2 = theta + beta - np.pi
    elif turnType == "LSR":
        beta = math.acos(max(-1.0, min(1.0, 2*R / d)))
        a1 = theta - beta
        a2 = theta - beta + np.pi

    xco = xc1 + R * np.cos(a1)
    yco = yc1 + R * np.sin(a1)
    xci = xc2 + R * np.cos(a2)
    yci = yc2 + R * np.sin(a2)

    phi1 = np.arctan2(y1 - yc1, x1 - xc1)
    phi2 = np.arctan2(y2 - yc2, x2 - xc2)

    xs1, ys1, dist1 = generate_arc_points(xc1, yc1, R, phi1, a1, sigma1, step_size=0.1)
    xs_s, ys_s, dist_s = generate_straight_points(xco, yco, xci, yci, step_size=R*0.1)
    xs2, ys2, dist2 = generate_arc_points(xc2, yc2, R, a2, phi2, sigma2, step_size=0.1)

    xtrajectory = xs1 + xs_s[1:] + xs2[1:]
    ytrajectory = ys1 + ys_s[1:] + ys2[1:]
    pathDistance = dist1 + dist_s + dist2

    return xtrajectory, ytrajectory, pathDistance


def calculateCCCTrajectory(x1, x2, xc1, xc2, y1, y2, yc1, yc2, psi1, psi2, R, sigma1, sigma2):
    d = distance(xc1, xc2, yc1, yc2)
    u_vec = np.array([xc2 - xc1, yc2 - yc1])
    u_vec = u_vec / np.linalg.norm(u_vec)

    theta = math.acos(max(-1.0, min(1.0, d / (4*R))))
    Mco = np.array([[np.cos(theta), -sigma1*np.sin(theta)], [sigma1*np.sin(theta), np.cos(theta)]])
    
    C3 = 2*R*(Mco @ u_vec) + np.array([xc1, yc1])
    xc3, yc3 = C3[0], C3[1]
    
    P13_x, P13_y = (xc1 + xc3)/2, (yc1 + yc3)/2
    P32_x, P32_y = (xc3 + xc2)/2, (yc3 + yc2)/2
    
    a13 = np.arctan2(P13_y - yc1, P13_x - xc1)
    a31 = np.arctan2(P13_y - yc3, P13_x - xc3)
    a32 = np.arctan2(P32_y - yc3, P32_x - xc3)
    a23 = np.arctan2(P32_y - yc2, P32_x - xc2)

    phi1 = np.arctan2(y1 - yc1, x1 - xc1)
    phi2 = np.arctan2(y2 - yc2, x2 - xc2)
    
    sigma3 = -sigma1

    xs1, ys1, dist1 = generate_arc_points(xc1, yc1, R, phi1, a13, sigma1, step_size=0.1)
    xs3, ys3, dist3 = generate_arc_points(xc3, yc3, R, a31, a32, sigma3, step_size=0.1)
    xs2, ys2, dist2 = generate_arc_points(xc2, yc2, R, a23, phi2, sigma2, step_size=0.1)

    xtrajectory = xs1 + xs3[1:] + xs2[1:]
    ytrajectory = ys1 + ys3[1:] + ys2[1:]
    pathDistance = dist1 + dist3 + dist2

    return xtrajectory, ytrajectory, pathDistance


def dubins_path_length(p1, p2, R):
    x1, y1, psi1 = p1
    x2, y2, psi2 = p2
    xt, yt, dist = returnDubinsPath(x1, y1, psi1, x2, y2, psi2, R)
    if dist is not None:
        return dist
    return distance(x1, x2, y1, y2)


def generate_dubins_path(p1, p2, R):
    x1, y1, psi1 = p1
    x2, y2, psi2 = p2
    xt, yt, dist = returnDubinsPath(x1, y1, psi1, x2, y2, psi2, R)
    
    if xt is not None and yt is not None:
        return list(zip(xt, yt))
    else:
        # Fallback to straight line if dubins path generation fails (e.g., edge cases)
        return [(x1, y1), (x2, y2)]
