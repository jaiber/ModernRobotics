#!/usr/bin/env python3

import csv
import numpy as np

# Path planning program to find optimal path using A* algorithm
# Chapter 10 project - Following the algorithm shown in Graph Search 10.2.4 lecture
INFINITY = 10000
start = 1
goal = 12

# A* Algorithm implementation - returns the optimal parent list
def A_STAR(nodes_dict, adj_matrix):

    global INFINITY, start, goal

    open_list = [(1, 0)] # List of tuples
    closed_list = [] # List
    past_cost = [0, 1] + [INFINITY] * 11 # Since it's 0 based
    parent = {} # Dict

    while open_list:
        open_list.sort(key=lambda tup: tup[1]) # Sort tuples by cost - 2nd item
        
        (current, _) = open_list[0] # First of the list
        #print("\tcurrent: ", current)
        del open_list[0] # Remove it from the list
        closed_list.append(current) # Add current to closed_list

        if (current == goal):
            print("Goal reached!")
            # Find best path
            optimal_path = []
            p = goal
            while (p):
                #print(p, " -> ", end="")
                optimal_path.append(p)
                if (p == start):
                    break
                p = parent[p]

            optimal_path.reverse()
            #print(past_cost)
            print("Optimal path: ", optimal_path)
            return optimal_path
        
        # For each neighbor of current
        for nbr in range(1, 13):
            cost = adj_matrix[current][nbr]
            # Skip if its in closed_list
            if (cost == 0) or (nbr in closed_list):
                #print("skip nbr: ", nbr)
                continue
            #print(current, " -> ", nbr, " : ", cost)

            # tentative_past_cost <- past_cost[current] + cost[current][nbr]
            tentative_past_cost = past_cost[current] + cost

            if (tentative_past_cost < past_cost[nbr]):
                past_cost[nbr] = tentative_past_cost
                parent[nbr] = current
                #print("parent of ", nbr, " = ", current)

                est_total_cost = past_cost[nbr] + cost
                # Just append it to open list, it'll be sorted always
                open_list.append((nbr, est_total_cost))

    print ("No solution found!")
    return []

# MAIN program
if __name__ == '__main__':

    # Read 'nodes.csv'
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