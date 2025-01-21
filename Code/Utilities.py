import numpy as np
import time
import math
import re
import copy


def euclidean_distance(coord1, coord2):
    return math.sqrt((coord1[0] - coord2[0]) ** 2 + (coord1[1] - coord2[1]) ** 2)


# Helper function to calculate route cost
def calculate_route_cost(route, distance_matrix, depot):
    if route == []:
        cost = 0
        return cost
    else:
        cost = distance_matrix[depot][route[0]]
    for i in range(len(route) - 1):
        cost += distance_matrix[route[i]][route[i + 1]]
    cost += distance_matrix[route[len(route)-1]][depot]
    return cost

def calculate_route_improvement(old_route, new_route, distance_matrix):
    improvement = calculate_route_cost(new_route, distance_matrix) - calculate_route_cost(old_route, distance_matrix)
    if improvement > 0:
        return improvement
    else:
        improvement = 0
        return improvement

def calculate_solution_cost(solution, distance_matrix, depot):
    cost = 0
    for route in solution:
        cost += calculate_route_cost(route, distance_matrix, depot)
    return cost

def calculate_vehicle_capacity(route, demands):
    capacity = 0
    for i in route:
        capacity += demands[i - 1]
    return capacity

def print_matrix(matrix, title="Matrix"):
    """
    Prints a 2D matrix in a readable format.

    Parameters:
    - matrix (2D list or numpy array): The matrix to be printed.
    - title (str): Title to display above the matrix (optional).
    """
    print(f"{title}:")
    for row in matrix:
        print(" ".join(f"{value:8.2f}" if isinstance(value, float) else f"{value:8}" for value in row))
    print()