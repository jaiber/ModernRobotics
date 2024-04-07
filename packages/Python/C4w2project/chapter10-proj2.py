#!/usr/bin/env python3

import sys
import csv
import json
import math
import scipy
import random
import numpy as np
import matplotlib.pyplot as plt


def read_obstacles(obs_file):
    """Reads obstacles.csv file into list"""
    lines = csv.reader(open(obs_file))
    obstacles = []
    for l in lines:
        if l[0][0] != "#":
            obstacles.append([float(l[0]), float(l[1]), float(l[2])])
    return obstacles


def random_samples(parameters, obstacles):
    
    #fig, ax = plt.subplots()
    """Generate random samples"""
    
    x = np.random.uniform(
        low=parameters["left"], high=parameters["right"], size=parameters["num_samples"]
    )
    x = np.insert(x, 0, -0.5)
    x = np.append(x, 0.5)
    
    y = np.random.uniform(
        low=parameters["bottom"], high=parameters["top"], size=parameters["num_samples"]
    )
    y = np.insert(y, 0, -0.5)
    y = np.append(y, 0.5)
    
    #ax.set_xlim([-0.5, 0.5])
    #ax.set_ylim([-0.5, 0.5])

    #plt.scatter(x, y)
    
    samples = list(zip(x, y))

    i = 0
    new_samples = []
    for s in samples:
        add = True
        for o in obstacles:
            # Calculate distance of point from circle center
            dist = math.dist(s, [o[0], o[1]])
            if dist < o[2]/2:
                #print("dist: ", dist)
                #print("radius: ", o[2]/2)
                #samples.remove(s)
                #samples.pop(i)
                add = False
                #print("Removing: ", i)
                break
        
        if add:
            new_samples.append([s[0], s[1]])
        i = i + 1
    # Filter out samples lying within obstacles
    #sys.exit(0)
    #plt.close()
    #for i in range(len(samples)):
    #    plt.text(samples[i][0], samples[i][1], str(i))
    #plt.show()
    #return samples
    #print(new_samples)
    #sys.exit(0)
    return new_samples
    
def sign(x):
    return -1 if x < 0 else 1

def check_collision(obstacles, adj_matrix, l1, l2):
    #print("Checking line from: ", l1, " to ", l2)
    i = 0
    for o in obstacles:
        #print("\tChecking obstacle: ", i)
        i = i + 1

        x1 = l1[0] - o[0]
        y1 = l1[1] - o[1]
        x2 = l2[0] - o[0]
        y2 = l2[1] - o[1]
        dx = x2 - x1
        dy = y2 - y1
        dr = math.sqrt(dx * dx + dy * dy)
        #print("dr: ", dr)
        D = x1 * y2 - x2 * y1
        r = o[2]/2
        discriminant = r * r * dr * dr - D * D
        #print("\tdiscriminant: ", discriminant)
        
        if discriminant < 0: # No intersection
            continue

        if discriminant == 0:
            xa = (D * dy ) /  (dr * dr)
            ya = (-D * dx ) /  (dr * dr)
            ta = (xa-x1)*dx/dr + (ya-y1)*dy/dr
            if 0 < ta < dr:
                return True
        
        xa = (D * dy + sign(dy) * dx * math.sqrt(discriminant)) / (dr * dr)
        ya = (-D * dx + abs(dy) * math.sqrt(discriminant)) / (dr * dr)
        ta = (xa-x1)*dx/dr + (ya-y1)*dy/dr
        
        xb = (D * dy - sign(dy) * dx * math.sqrt(discriminant)) / (dr * dr) 
        yb = (-D * dx - abs(dy) * math.sqrt(discriminant)) / (dr * dr)
        tb = (xb-x1)*dx/dr + (yb-y1)*dy/dr

        if 0 < ta < dr or 0 < tb < dr:
            return True

    return False


def PRM(obstacles, samples, parameters, plt):
    fig, ax = plt.subplots()
    ax.set_xlim([-0.5, 0.5])
    ax.set_ylim([-0.5, 0.5])

    i = 0
    for circle in obstacles:
        ax.add_patch(plt.Circle((circle[0], circle[1]), circle[2]/2))
        plt.text(circle[0], circle[1], "c" + str(i))
        i = i + 1

    # Build KD Tree
    kdtree = scipy.spatial.KDTree(samples)

    # Declare the Graph
    #sz = parameters["num_samples"]
    sz = len(samples)
    adj_matrix = np.zeros(shape=(sz, sz))
    k = parameters["nearest"]

    j = 0
    # Initialize the Graph with the top 'k' nearest neighbours
    for s in samples:
        dd, ii = kdtree.query(s, k=k + 1)
        #print(dd, ii)
        
        plt.text(s[0], s[1], str(j))
        j = j + 1
        #plt.scatter(s[0], s[1])
        for i in range(1, k + 1):
            start_idx = ii[0]
            end_idx = ii[i]
            #print(start_idx, " -> ", end_idx, " = ", dd[i])
            e = samples[end_idx]
            #print("plotting from: ", s, " to ", e)
            #ax.plot([s[0], e[0]], [s[1], e[1]], marker='o')

            # Check the edge for collision
            result = check_collision(obstacles, adj_matrix, samples[start_idx], samples[end_idx])
            if not result:
                #print("Adding path")
                ax.plot([s[0], e[0]], [s[1], e[1]], marker='o')
                #print(f"\tadj_matrix{start_idx}{end_idx} = {dd[i]}")
                adj_matrix[start_idx][end_idx] = dd[i]

    #plt.show()
    #print(adj_matrix)
    #for row in adj_matrix:
    #    print(*row, sep="\t\t")
    return adj_matrix

def plot(obstacles, samples, plt):
    fig, ax = plt.subplots()
   
    for circle in obstacles:
        ax.add_patch(plt.Circle((circle[0], circle[1]), circle[2]))

    for s in samples:
        plt.plot(s[0], s[1])

    #plt.show()
    return

def save_nodes(nodes, csv_file):
    new_nods = []
    with open(csv_file, "w") as f:
        print("# no,x,y,heuristic_distance", file=f)
        count = 1
        
        for n in nodes:
            dist = math.dist([0.5, 0.5], [n[0], n[1]])
            #print(">>>>>>>: ", n)
            print(f"{count},{n[0]},{n[1]},{dist}", file=f)
            new_nods.append([n[0], n[1], dist])
            count = count + 1

    return new_nods

def save_edges(adj_matrix, csv_file):
    uniqsets = {}
    with open(csv_file, "w") as f:
        print("# ID1,ID2,cost", file=f)
        for i in range(len(adj_matrix)):
            uniqsets[i] = set()
            for j in range(len(adj_matrix[0])):
                #print("i = ", i, " j = ", j)
                val = adj_matrix[i][j]
                #print("\t val: ", val)
                if val == 0:
                    continue
                #print(f"\tchecking if {j} exists")
                if j in uniqsets:
                    #print(f"\t-- checking if {i} exists in {j}'s set")
                    if i not in uniqsets[j]:
                        #print(f"\t\t path from {i} to {j}")
                        print(f"{i+1},{j+1},{val}", file=f)
                
    return

# Path planning program to find optimal path using A* algorithm
# Chapter 10 project - Following the algorithm shown in Graph Search 10.2.4 lecture
INFINITY = 10000 # Define infinity as a high number
#start = 1 # Define the start and end nodes
#goal = 12

# A* Algorithm implementation - returns the optimal parent list
def A_STAR(samples, adj_matrix, num_samples):

    global INFINITY #, start, goal
    start = 0
    goal = num_samples - 1

    # open_list is a list of tuples (node_id, cost)
    open_list = [(0, 0)]
    # closed_list is a simple list of node_ids
    closed_list = []
    # initialize the past_cost list with the first node, followed by infinite for other nodes
    past_cost = [INFINITY] * num_samples
    past_cost[0] = 0
    # Dictionary of parent nodes to return
    parent = {}

    # While there are elements left in the open_list
    while open_list:
        # Sort the tuples by the 2nd item which is the node cost
        open_list.sort(key=lambda tup: tup[1])
        
        # Pop the first item in the list, with lowest cost
        (current, _) = open_list[0]
        del open_list[0] # Remove it from the list
        closed_list.append(current) # Add current to closed_list
        #print("Current: ", current)

        # Check if the goal is reached or not
        if (current == goal):
            print("Goal reached!")
            # Find best path
            optimal_path = []
            p = goal
            # Trace the path from end to start
            total_cost = 0
            while (True):
                optimal_path.append(p + 1)
                #print("\t p: ", p, " start: ", start)
                #print("\t optimal path: ", optimal_path)
                if (p == start):
                    break
                total_cost = total_cost + adj_matrix[p][parent[p]]
                p = parent[p]

            # Reverse the list and return
            optimal_path.reverse()
            print("total cost: ", total_cost)
            print("Optimal path: ", optimal_path)
            return optimal_path
        
        # Process each neighbour of the current node
        for nbr in range(num_samples):
            
            # Get neighbour from the adjacency matrix
            #print(f"\tcurrent: {current}, nbr: {nbr}")
            cost = adj_matrix[current][nbr]
         
            #print("\tChecking: ", nbr, end="")
            #print("\tcost: ", cost)
            # Skip if its in closed_list
            if (cost == 0) or (nbr in closed_list):
                continue          

            tentative_past_cost = past_cost[current] + cost
            #print("\ttentative_past_cost: ", tentative_past_cost, " past_cost: ", past_cost[nbr])

            if (tentative_past_cost < past_cost[nbr]):
                past_cost[nbr] = tentative_past_cost
                parent[nbr] = current

                # Calculate the estimated total cost
                est_total_cost = past_cost[nbr] + samples[nbr][2] #cost

                #print("\t\tInserting: ", nbr, " est cost: ", est_total_cost)
                # Just append it to open list, it'll be sorted always
                open_list.append((nbr, est_total_cost))

    print ("No solution found!", goal)
    return []

parameters_json = "parameters.json"

##################### MAIN ##########################
if __name__ == "__main__":
    parameters = json.load(open(parameters_json))
    print(parameters)

    obstacles = read_obstacles(obs_file="obstacles.csv")
    print(obstacles)

    samples = random_samples(parameters, obstacles)
    num_samples = len(samples)
    """
    for s in samples:
        print(s)

    fig, ax = plt.subplots()
    ax.set_aspect("equal")

    # Set the limits of the plot
    ax.set_xlim([-0.5, 0.5])
    ax.set_ylim([-0.5, 0.5])

    # Label the axes
    ax.set_xlabel("X")
    ax.set_ylabel("Y")

    # Title the plot
    ax.set_title("Circles")
    """
    #plot(obstacles, samples, plt)
    adj_matrix = PRM(obstacles, samples, parameters, plt)

    nodes = save_nodes(samples, "nodes.csv")
    save_edges(adj_matrix, "edges.csv")

    path = A_STAR(nodes, adj_matrix, num_samples)
    with open("path.csv", "w") as f:
        csvwriter = csv.writer(f)
        csvwriter.writerow(path)
    plt.show()

    lines = csv.reader(open("nodes.csv"))
    for l in lines:
        print(l)

    lines = csv.reader(open("edges.csv"))
    for l in lines:
        print(l)