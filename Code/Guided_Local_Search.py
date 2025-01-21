import numpy as np
import time
import math
import re
import copy
from Utilities import calculate_solution_cost
from Local_Search import local_search


def create_initial_solution(Instance):
    #solution = [[22, 32, 20, 18, 14, 8, 27],[13, 2, 17, 31],[28, 25],[30, 19, 9, 10, 23, 16, 11, 26, 6, 21],[15, 29, 12, 5, 24, 4, 3, 7]]
    solution = [[] for _ in range(Instance["num_vehicles"])]
    remaining_customers = list(range(2, Instance["dimension"] + 1))
    Id = 1
    #return solution
    for customer in remaining_customers:
        demand = Instance["demands"][Id]
        Id += 1
        for vehicle in range(Instance["num_vehicles"]):
            if Instance["vehicle_capacity"][vehicle] + demand <= Instance["capacity"]:
                solution[vehicle].append(customer)
                Instance["vehicle_capacity"][vehicle] += demand
                break
    return solution

def choose_penalty_features(solution, penalty_matrix, cost_matrix):
    """
    Selects the features to penalize based on the GLS strategy.

    Parameters:
    - solution (list of lists): Current solution, where each route is a list of nodes.
    - penalty_matrix (2D array): Matrix of penalties for each feature (i, j).
    - cost_matrix (2D array): Matrix of costs (e.g., distances) for each feature (i, j).

    Returns:
    - features_to_penalize (list of tuples): List of features (i, j) to penalize.
    """
    max_ratio = -1
    features_to_penalize = []

    # Iterate through the solution to evaluate features
    for route in solution:
        for i in range(len(route) - 1):
            u, v = route[i], route[i + 1]
            # Compute the GLS ratio: cost / (penalty + 1)
            ratio = cost_matrix[u][v] / (penalty_matrix[u][v] + 1)
            
            # Update the maximum ratio and selected features
            if ratio > max_ratio:
                max_ratio = ratio
                features_to_penalize = [(u, v)]
            elif ratio == max_ratio:
                features_to_penalize.append((u, v))
    return features_to_penalize


def apply_gls_penalty(distance_matrix, penalty_matrix, lambda_value):
    """
    Applies the GLS penalty to the distance matrix.

    Parameters:
    - distance_matrix (2D array): Original distance matrix.
    - penalty_matrix (2D array): Penalty matrix.
    - lambda_value (float): Weight of the penalty.

    Returns:
    - new_distance_matrix (2D array): Modified distance matrix with penalties applied.
    """
    new_distance_matrix = np.copy(distance_matrix)
    for i in range(len(distance_matrix)):
        for j in range(len(distance_matrix[i])):
            new_distance_matrix[i][j] = distance_matrix[i][j] + lambda_value * penalty_matrix[i][j]
    return new_distance_matrix

# Guided Local Search method
def guided_local_search(Instance, distance_matrix, LAMBDA, time_limit=300):
    Iterations = 0
    """Perform Guided Local Search (GLS) to solve the VRP.""" 

    penalyzed_distance_matrix = np.copy(distance_matrix)

    penalties = np.zeros_like(penalyzed_distance_matrix)

    # Create the initial solution
    initial_solution = create_initial_solution(Instance)
    #return initial_solution
    solution = local_search(initial_solution, penalyzed_distance_matrix, Instance)
    Best_Cost = calculate_solution_cost(solution, distance_matrix, Instance["depot"])

    # Start timer
    start_time = time.time()

    # Run GLS until the time limit is reached
    while time.time() - start_time < time_limit:
        Iterations += 1
        # Penalize features
        features = choose_penalty_features(solution, penalties, penalyzed_distance_matrix)  
        for (i, j) in features:
            penalties[i][j] += 1

        # Update the distance matrix with penalties
        penalyzed_distance_matrix = apply_gls_penalty(penalyzed_distance_matrix, penalties, LAMBDA)

        # Perform local search
        solution = local_search(solution, penalyzed_distance_matrix, Instance)
        new_cost = calculate_solution_cost(solution, distance_matrix, Instance["depot"])
        # Update the best solution
        if new_cost < Best_Cost:
            Best_Cost = new_cost
            Best_Solution = copy.deepcopy(solution)
        # Perform a final local search on the best solution
    Best_Solution = local_search(Best_Solution, distance_matrix, Instance)

    return Best_Solution