import math



def euclidean_distance(coord1, coord2):
    """
    Computes the Euclidean distance between two coordinate points.

    Parameters:
    - coord1 (tuple): The (x, y) coordinates of the first point.
    - coord2 (tuple): The (x, y) coordinates of the second point.

    Returns:
    - float: The Euclidean distance between the two points.
    """
    return math.sqrt((coord1[0] - coord2[0]) ** 2 + (coord1[1] - coord2[1]) ** 2)


# Helper function to calculate route cost
def calculate_route_cost(route, distance_matrix, depot):
    """
    Computes the total cost (distance) of a given route.

    Parameters:
    - route (list): A list of node indices representing a vehicle route.
    - distance_matrix (2D list): A matrix containing pairwise distances between nodes.
    - depot (int): The index of the depot (starting and ending location).

    Returns:
    - float: The total cost of the given route.
    """
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
    """
    Computes the improvement (or deterioration) in cost when switching from an old route to a new one.

    Parameters:
    - old_route (list): The original route.
    - new_route (list): The modified route.
    - distance_matrix (2D list): A matrix containing pairwise distances between nodes.

    Returns:
    - float: The improvement in cost (positive means a worse route, zero means no change).
    """
    improvement = calculate_route_cost(new_route, distance_matrix) - calculate_route_cost(old_route, distance_matrix)
    if improvement > 0:
        return improvement
    else:
        improvement = 0
        return improvement

def calculate_solution_cost(solution, distance_matrix, depot):
    """
    Computes the total cost of a solution consisting of multiple vehicle routes.

    Parameters:
    - solution (list of lists): A list of routes, where each route is a list of node indices.
    - distance_matrix (2D list): A matrix containing pairwise distances between nodes.
    - depot (int): The index of the depot (starting and ending location).

    Returns:
    - float: The total cost of the solution.
    """
    cost = 0
    for route in solution:
        cost += calculate_route_cost(route, distance_matrix, depot)
    return cost

def calculate_custumer_number(solution):
    """
    Computes the total number of customers visited in a given solution.

    Parameters:
    - solution (list of lists): A list of routes, where each route is a list of node indices.

    Returns:
    - int: The total number of customers visited.
    """
    N = 1
    for route in solution:
        for i in range(len(route)):
            N += 1
    return N

def calculate_vehicle_capacity(route, demands):
    """
    Computes the total demand served by a vehicle on a given route.

    Parameters:
    - route (list): A list of node indices representing a vehicle route.
    - demands (list): A list where demands[i] is the demand of customer i.

    Returns:
    - int: The total demand carried by the vehicle on this route.
    """
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