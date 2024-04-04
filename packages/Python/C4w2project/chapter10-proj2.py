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
    
    fig, ax = plt.subplots()
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
    
    ax.set_xlim([-0.5, 0.5])
    ax.set_ylim([-0.5, 0.5])

    #plt.scatter(x, y)
    #for i in range(len(x)):
    #    plt.text(x[i], y[i], str(i))
    samples = list(zip(x, y))

    i = 0
    for s in samples:
        for o in obstacles:
            # Calculate distance of point from circle center
            dist = math.dist(s, [o[0], o[1]])
            if dist < o[2]/2:
                #print("dist: ", dist)
                #print("radius: ", o[2]/2)
                samples.remove(s)
                #samples.pop(i)
                #print("Removing: ", i)
                break
        i = i + 1
    # Filter out samples lying within obstacles
    #sys.exit(0)
    return samples
    
    #rand_arr = np.random.rand(parameters["num_samples"], 2)
    #print(rand_arr)
    #rand_arr = rand_arr - 0.5
    #plt.scatter(rand_arr[:,0], rand_arr[:,1])
    #plt.show()
    #return rand_arr

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
            #return [(xa + cpt[0], ya + cpt[1])] if 0 < ta < dr else []
        
        xa = (D * dy + sign(dy) * dx * math.sqrt(discriminant)) / (dr * dr)
        ya = (-D * dx + abs(dy) * math.sqrt(discriminant)) / (dr * dr)
        ta = (xa-x1)*dx/dr + (ya-y1)*dy/dr
        #xpt = [(xa + cpt[0], ya + cpt[1])] if 0 < ta < dr else []
        
        xb = (D * dy - sign(dy) * dx * math.sqrt(discriminant)) / (dr * dr) 
        yb = (-D * dx - abs(dy) * math.sqrt(discriminant)) / (dr * dr)
        tb = (xb-x1)*dx/dr + (yb-y1)*dy/dr
        #xpt += [(xb + cpt[0], yb + cpt[1])] if 0 < tb < dr else []

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
    sz = parameters["num_samples"]
    adj_matrix = np.zeros(shape=(sz + 1, sz + 1))
    k = parameters["nearest"]

    j = 0
    # Initialize the Graph with the top 'k' nearest neighbours
    for s in samples:
        dd, ii = kdtree.query(s, k=k + 1)
        # print(dd, ii)
        
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
            #if not check_collision(obstacles, samples[start_idx], samples[end_idx]):
            #    adj_matrix[start_idx][end_idx] = dd[i]
            result = check_collision(obstacles, adj_matrix, samples[start_idx], samples[end_idx])
            if not result:
                #print("Adding path")
                ax.plot([s[0], e[0]], [s[1], e[1]], marker='o')
                adj_matrix[start_idx][end_idx] = dd[i]

    plt.show()
    print(adj_matrix)
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
    with open(csv_file, "w") as f:
        print("# no, x, y, heuristic_distance", file=f)
        count = 1
        
        for n in nodes:
            dist = math.dist([-0.5, -0.5], [n[0], n[1]])
            #print(">>>>>>>: ", n)
            print(f"{count}, {n[0]}, {n[1]}, {dist}", file=f)

    return

def save_edges(adj_matrix, csv_file):
    uniqsets = {}
    with open(csv_file, "w") as f:
        print("# ID1, ID2, cost", file=f)
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
                        print(f"\t\t path from {i} to {j}")
                        print(f"{i}, {j}, {val}", file=f)
                
    return

parameters_json = "parameters.json"

##################### MAIN ##########################
if __name__ == "__main__":
    parameters = json.load(open(parameters_json))
    print(parameters)

    obstacles = read_obstacles(obs_file="obstacles.csv")
    print(obstacles)

    samples = random_samples(parameters, obstacles)
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

    save_nodes(samples, "nodes.csv")
    save_edges(adj_matrix, "edges.csv")
