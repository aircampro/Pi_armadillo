#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# In robotics,
# Nearest point search is a commonly used calculation.
#
# For example, for collision detection and search for correspondence points for sensor information.
#
# In the example below, we are creating a kd-tree from a two-dimensional point sequence.
#
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial import cKDTree
points = np.random.random((20, 2))
kd_tree = KDTree(points)

print(f"{kd_tree.m=}")
print(f"{kd_tree.n=}")
print(f"{kd_tree.leafsize=}")
print(f"{kd_tree.size=}")
print(f"{kd_tree.maxes=}")
print(f"{kd_tree.mins=}")
# kd_tree.m=2
# kd_tree.n=100
# kd_tree.leafsize=16
# kd_tree.size=15
# kd_tree.maxes=array([0.99660085, 0.98942514])
# kd_tree.mins=array([0.01764816, 0.00777515]

# In this case, we will set k=2 and explore the closest points to the first and second.
#
np.random.seed(20)
targets = np.random.random((3, 2))
points = np.random.random((100, 2))
kd_tree = KDTree(points)
dists, indexes = kd_tree.query(targets, k=2)
plt.plot(targets[:, 0], targets[:, 1], "xb")
plt.plot(points[:, 0], points[:, 1], "og")
for i in range(len(indexes)):
    for j in indexes[i]:
        plt.plot([points[j, 0], targets[i, 0]],[points[j, 1], targets[i, 1]], "-r")
plt.show()

# This time, I searched for points within r=0.2. query_ball_point
#
np.random.seed(20)
targets = np.random.random((3, 2))
points = np.random.random((100, 2))
kd_tree = KDTree(points)
indexes = kd_tree.query_ball_point(targets, 0.2)
plt.plot(targets[:, 0], targets[:, 1], "xb")
plt.plot(points[:, 0], points[:, 1], "og")
for i in range(len(indexes)):
    for j in indexes[i]:
        plt.plot([points[j, 0], targets[i, 0]], [points[j, 1], targets[i, 1]], "-r")
plt.show()

# In the example below, we are exploring the point where the points of two kd-trees are r=0.2 or less. query_ball_tree
#
np.random.seed(21701)
points1 = np.random.random((15, 2))
points2 = np.random.random((15, 2))
plt.figure(figsize=(6, 6))
plt.plot(points1[:, 0], points1[:, 1], "xk")
plt.plot(points2[:, 0], points2[:, 1], "og")
kd_tree1 = KDTree(points1)
kd_tree2 = KDTree(points2)
indexes = kd_tree1.query_ball_tree(kd_tree2, r=0.2)
for i in range(len(indexes)):
    for j in indexes[i]:
        plt.plot([points1[i, 0], points2[j, 0]],
                 [points1[i, 1], points2[j, 1]], "-r")
plt.show()

# If you want to search for a point within a certain range in a single kd-tree,
# query_pairs is used.
#
np.random.seed(21701)
points = np.random.random((20, 2))
plt.figure(figsize=(6, 6))
plt.plot(points[:, 0], points[:, 1], "xk")
kd_tree = KDTree(points)
pairs = kd_tree.query_pairs(r=0.2)
for (i, j) in pairs:
    plt.plot([points[i, 0], points[j, 0]], [points[i, 1], points[j, 1]], "-r")
plt.show()

# If you want to calculate the distance of a point below the distance with two kd-trees, you can use the
# Use sparse_distance_matrix.
p.random.seed(21701)
points1 = np.random.random((5, 2))
points2 = np.random.random((5, 2))
kd_tree1 = KDTree(points1)
kd_tree2 = KDTree(points2)
indexes = kd_tree1.query_ball_tree(kd_tree2, r=0.2)
sparse_distance_matrix = kd_tree1.sparse_distance_matrix(kd_tree2, 0.2, output_type="ndarray")
print(sparse_distance_matrix)




