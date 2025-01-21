import numpy as np
import time
import math
import re
import copy
from Utilities import euclidean_distance, calculate_solution_cost
from Guided_Local_Search import guided_local_search

def parse_vrp_file(file_path):
    """
    Parse a VRP file and create an instance of the problem.
    
    Parameters:
    - file_path (str): Path to the VRP file.
    
    Returns:
    - instance (dict): A dictionary containing the parsed problem parameters.
    """
    instance = {
        "name": None,
        "num_vehicles": None,
        "optimal_value": None,
        "type": None,
        "dimension": None,
        "capacity": None,
        "vehicle_capacity": None,
        "coordinates": [],
        "demands": [],
        "depot": None
    }

    with open(file_path, "r") as file:
        lines = file.readlines()
        
        section = None
        for line in lines:
            line = line.strip()
            
            if line.startswith("NAME"):
                instance["name"] = line.split(":")[1].strip()
            elif line.startswith("COMMENT"):
                # Extracting number of vehicles and optimal value from the comment line
                match = re.search(r"\((.*?)\)", line)
                if match:
                    comment_content = match.group(1)
                    # Extract number of vehicles and optimal value from the comment
                    parts = comment_content.split(", ")
                    for part in parts:
                        if "No of trucks" in part:
                            instance["num_vehicles"] = int(re.search(r"\d+", part).group())
                        elif "Optimal value" in part:
                            instance["optimal_value"] = int(re.search(r"\d+", part).group())
            elif line.startswith("TYPE"):
                instance["type"] = line.split(":")[1].strip()
            elif line.startswith("DIMENSION"):
                instance["dimension"] = int(line.split(":")[1].strip())
            elif line.startswith("CAPACITY"):
                instance["capacity"] = int(line.split(":")[1].strip())
            elif line.startswith("NODE_COORD_SECTION"):
                section = "coordinates"
            elif line.startswith("DEMAND_SECTION"):
                section = "demands"
            elif line.startswith("DEPOT_SECTION"):
                section = "depot"
            elif line.startswith("EOF"):
                break
            elif section == "coordinates":
                parts = line.split()
                if len(parts) == 3:
                    instance["coordinates"].append((int(parts[1]), int(parts[2])))
            elif section == "demands":
                parts = line.split()
                if len(parts) == 2:
                    instance["demands"].append(int(parts[1]))
            elif section == "depot":
                parts = line.split()
                if len(parts) == 1 and parts[0] != "-1":
                    instance["depot"] = int(parts[0])
        instance["vehicle_capacity"] = [0] * instance["num_vehicles"]
    return instance

def main_function(Instance, Lambda):

    distance_matrix = np.zeros((Instance["dimension"] + 1, Instance["dimension"] + 1), dtype=int)
    for i in range(Instance["dimension"]):
        for j in range(Instance["dimension"]):
            distance_matrix[i+1][j+1] = euclidean_distance(Instance["coordinates"][i], Instance["coordinates"][j])

    # Run Guided Local Search
    final_solution = guided_local_search(Instance, distance_matrix, Lambda)


    # Print final solution
    best_cost = Instance["optimal_value"]
    print(Instance["name"])
    for v, route in enumerate(final_solution):
        route_cost = 0
        for i in range(len(route) - 1):
            route_cost += distance_matrix[route[i]][route[i + 1]]
        #print(f"Vehicle {v + 1}: Route = {route}, Cost = {route_cost}")
    print(f"Total Cost = {calculate_solution_cost(final_solution, distance_matrix, 1)}   --  Best Cost = {best_cost}")



vrp_file_path1 = "Set A\A\A-n32-k5.vrp"
vrp_file_path2 = "Set A\A\A-n46-k7.vrp"
vrp_file_path3 = "Set A\A\A-n80-k10.vrp"
vrp_file_path4 = "Set B\B\B-n50-k8.vrp"
vrp_file_path5 = "Set B\B\B-n78-k10.vrp"
instance1 = parse_vrp_file(vrp_file_path1)
instance2 = parse_vrp_file(vrp_file_path2)
instance3 = parse_vrp_file(vrp_file_path3)
instance4 = parse_vrp_file(vrp_file_path4)
instance5 = parse_vrp_file(vrp_file_path5)



print("-----------------------------------------------------------------------------------")
print("Lambda : 0.35")
main_function(instance1, 0.25)
main_function(instance2, 0.25)
main_function(instance3, 0.25)
main_function(instance4, 0.25)
main_function(instance5, 0.25)

instance1 = parse_vrp_file(vrp_file_path1)
instance2 = parse_vrp_file(vrp_file_path2)
instance3 = parse_vrp_file(vrp_file_path3)
instance4 = parse_vrp_file(vrp_file_path4)
instance5 = parse_vrp_file(vrp_file_path5)

print("-----------------------------------------------------------------------------------")
print("Lambda : 0.55")
main_function(instance1, 0.55)
main_function(instance2, 0.55)
main_function(instance3, 0.55)
main_function(instance4, 0.55)
main_function(instance5, 0.55)

instance1 = parse_vrp_file(vrp_file_path1)
instance2 = parse_vrp_file(vrp_file_path2)
instance3 = parse_vrp_file(vrp_file_path3)
instance4 = parse_vrp_file(vrp_file_path4)
instance5 = parse_vrp_file(vrp_file_path5)

print("-----------------------------------------------------------------------------------")
print("Lambda : 0.95")
main_function(instance1, 0.95)
main_function(instance2, 0.95)
main_function(instance3, 0.95)
main_function(instance4, 0.95)
main_function(instance5, 0.95)
print("-----------------------------------------------------------------------------------")



