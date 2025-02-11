import copy
from Utilities import calculate_route_cost, calculate_vehicle_capacity


# Relocate operator
def relocate(solution, from_vehicle, from_index, to_vehicle, to_index):
    """Move a customer from one route to another."""
    customer = solution[from_vehicle].pop(from_index)
    solution[to_vehicle].insert(to_index, customer)
    return solution

# Exchange operator
def exchange(solution, vehicle1, index1, vehicle2, index2):
    """Swap two customers between different routes."""
    solution[vehicle1][index1], solution[vehicle2][index2] = solution[vehicle2][index2], solution[vehicle1][index1]
    return solution

# Cross operator
def cross(solution, vehicle1, index1, vehicle2, index2):
    """Swap the end portions of two vehicle routes."""
    new_route1 = solution[vehicle1][:index1 + 1] + solution[vehicle2][index2 + 1:]
    new_route2 = solution[vehicle2][:index2 + 1] + solution[vehicle1][index1 + 1:]
    solution[vehicle1] = new_route1
    solution[vehicle2] = new_route2
    return solution

# Local search method
# Optimized Two-opt procedure
def two_opt(route, distance_matrix, depot):
    """Perform the two-opt operation on a single vehicle route with fewer redundant computations."""
    best_route = route[:]
    best_cost = calculate_route_cost(route, distance_matrix, depot)
    n = len(route)

    for i in range(1, n - 2):
        for j in range(i + 1, n - 1):
            # Reverse the segment
            new_route = route[:i] + route[i:j + 1][::-1] + route[j + 1:]
            new_cost = calculate_route_cost(new_route, distance_matrix, depot)

            # Update best if improved
            if new_cost < best_cost:
                best_route = new_route
                best_cost = new_cost

    return best_route

# Optimized Local Search
def local_search(solution, distance_matrix, instance):
    """Greedy local search with best acceptance scheme and optimized performance."""
    depot = instance["depot"]
    demands = instance["demands"]
    capacity = instance["capacity"]
    vehicle_capacities = [calculate_vehicle_capacity(route, demands) for route in solution]

    while True:
        best_improvement = 0
        best_solution = None

        for vehicle in range(len(solution)):
            current_route = solution[vehicle]
            current_cost = calculate_route_cost(current_route, distance_matrix, depot)

            # Two-opt improvement
            new_route = two_opt(current_route, distance_matrix, depot)
            new_cost = calculate_route_cost(new_route, distance_matrix, depot)
            improvement = current_cost - new_cost

            if improvement > best_improvement:
                best_improvement = improvement
                best_solution = copy.deepcopy(solution[:])
                best_solution[vehicle] = new_route

            # Relocation improvement
            for other_vehicle in range(len(solution)):
                if other_vehicle == vehicle:
                    continue

                for from_index, customer in enumerate(current_route):
                    if vehicle_capacities[other_vehicle] + demands[customer - 1] <= capacity:
                        temp_solution = [route[:] for route in solution]
                        relocate(temp_solution, vehicle, from_index, other_vehicle, 0)

                        # Compute improvement
                        new_cost_vehicle = calculate_route_cost(temp_solution[vehicle], distance_matrix, depot)
                        new_cost_other = calculate_route_cost(temp_solution[other_vehicle], distance_matrix, depot)
                        total_new_cost = new_cost_vehicle + new_cost_other

                        improvement = (current_cost + calculate_route_cost(solution[other_vehicle], distance_matrix, depot)) - total_new_cost

                        if improvement > best_improvement and calculate_vehicle_capacity(temp_solution[vehicle], demands) <= capacity and  calculate_vehicle_capacity(temp_solution[other_vehicle], demands) <= capacity:
                            best_improvement = improvement
                            best_solution = copy.deepcopy(temp_solution)

            # Exchange and Cross improvements (merged for efficiency)
            for other_vehicle in range(len(solution)):
                if other_vehicle == vehicle:
                    continue

                for from_index, customer_1 in enumerate(current_route):
                    for to_index, customer_2 in enumerate(solution[other_vehicle]):
                        # Exchange
                        temp_solution = [route[:] for route in solution]
                        exchange(temp_solution, vehicle, from_index, other_vehicle, to_index)

                        # Compute improvement
                        new_cost_vehicle = calculate_route_cost(temp_solution[vehicle], distance_matrix, depot)
                        new_cost_other = calculate_route_cost(temp_solution[other_vehicle], distance_matrix, depot)
                        total_new_cost = new_cost_vehicle + new_cost_other

                        improvement = (current_cost + calculate_route_cost(solution[other_vehicle], distance_matrix, depot)) - total_new_cost

                        if improvement > best_improvement and calculate_vehicle_capacity(temp_solution[vehicle], demands) <= capacity and  calculate_vehicle_capacity(temp_solution[other_vehicle], demands) <= capacity:
                            best_improvement = improvement
                            best_solution = copy.deepcopy(temp_solution)


                        # Cross
                        temp_solution = [route[:] for route in solution]
                        cross(temp_solution, vehicle, from_index, other_vehicle, to_index)

                        # Compute improvement
                        new_cost_vehicle = calculate_route_cost(temp_solution[vehicle], distance_matrix, depot)
                        new_cost_other = calculate_route_cost(temp_solution[other_vehicle], distance_matrix, depot)
                        total_new_cost = new_cost_vehicle + new_cost_other

                        improvement = (current_cost + calculate_route_cost(solution[other_vehicle], distance_matrix, depot)) - total_new_cost

                        if improvement > best_improvement and calculate_vehicle_capacity(temp_solution[vehicle], demands) <= capacity and  calculate_vehicle_capacity(temp_solution[other_vehicle], demands) <= capacity:
                            best_improvement = improvement
                            best_solution = copy.deepcopy(temp_solution)


        # Stop if no improvements
        if best_improvement <= 0:
            break

        solution = copy.deepcopy(best_solution)

    return solution