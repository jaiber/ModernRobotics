#!/usr/bin/env python3

import csv
import numpy as np

# Path planning program to find optimal path using A* algorithm
# Chapter 10 project - Following the algorithm shown in Graph Search 10.2.4 lecture
INFINITY = 10000 # Define infinity as a high number
start = 1 # Define the start and end nodes
goal = 12

# A* Algorithm implementation - returns the optimal parent list
def A_STAR(nodes_dict, adj_matrix):

    global INFINITY, start, goal

    # open_list is a list of tuples (node_id, cost)
    open_list = [(1, 0)]
    # closed_list is a simple list of node_ids
    closed_list = []
    # initialize the past_cost list with the first node, followed by infinite for other nodes
    past_cost = [0, 1] + [INFINITY] * 11
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

        # Check if the goal is reached or not
        if (current == goal):
            print("Goal reached!")
            # Find best path
            optimal_path = []
            p = goal
            # Trace the path from end to start
            while (p):
                optimal_path.append(p)
                if (p == start):
                    break
                p = parent[p]

            # Reverse the list and return
            optimal_path.reverse()
            #print(past_cost)
            print("Optimal path: ", optimal_path)
            return optimal_path
        
        # Process each neighbour of the current node
        for nbr in range(1, 13):
            # Get neighbour from the adjacency matrix
            cost = adj_matrix[current][nbr]

            # Skip if its in closed_list
            if (cost == 0) or (nbr in closed_list):
                continue

            tentative_past_cost = past_cost[current] + cost

            if (tentative_past_cost < past_cost[nbr]):
                past_cost[nbr] = tentative_past_cost
                parent[nbr] = current

                # Calculate the estimated total cost
                est_total_cost = past_cost[nbr] + cost

                # Just append it to open list, it'll be sorted always
                open_list.append((nbr, est_total_cost))

    print ("No solution found!")
    return []

# MAIN program
if __name__ == '__main__':

    # Read 'nodes.csv' into a dictionary
    nodes = csv.reader(open("nodes.csv"))
    nodes_dict = {}
    for k in nodes:
        id = k[0]
        if (id[0] != '#'):
            vals = {}
            vals["x"] = float(k[1])
            vals["y"] = float(k[2])
            vals["h"] = float(k[3])
            nodes_dict[int(id)] = vals

    print(nodes_dict)

    # Read edges.csv, build adjacency matrix
    adj_matrix = np.zeros(shape=(13,13))
    edges = csv.reader(open("edges.csv"))
    for e in edges:
        id1 = e[0]
        if (id1[0] == '#'):
            continue
        id1 = int(id1)
        id2 = int(e[1])
        cost = float(e[2])
        adj_matrix[id1][id2] = cost
        adj_matrix[id2][id1] = cost
    
    print(adj_matrix)

    # Call A_STAR function, it returns array of optimal path nodes
    optimal_path = A_STAR(nodes_dict, adj_matrix)

    # Write out the result in path.csv
    with open("path.csv", "w") as f:
        csvwriter = csv.writer(f)
        csvwriter.writerow(optimal_path)