import numpy as np
import re
from Utilities import euclidean_distance, calculate_solution_cost
from Guided_Local_Search import guided_local_search
from Grafh import plot_grapfh

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
                match = re.search(r"\((.*?)\)", line)
                if match:
                    comment_content = match.group(1)
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
                    instance["coordinates"].append((float(parts[1]), float(parts[2])))
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

def get_path_instance(file_path, lam, Penalidade, iterations=1):
    """
    Processes a TXT file and create the instance obect and runs multiple iterations of the main function.

    Parameters:
    - file_path (str): Path to the file containing the VRP problem instances.
    - lam (float): Adjustment parameter used in the main function.
    - Penalidade (float): Penalty applied during the algorithm execution.
    - iterations (int, optional): Number of iterations to run for each instance (default: 1).

    This function reads a path of files of instances line by line, processes each instance, 
    by calling the function of hte cration of the instace object, 
    executes the main function multiple times, and logs the results to a file.
    """
    with open(file_path, "r") as file:
        lines = file.readlines()
        for line in lines:
                All_Iteration = 0
                All_Time = 0
                All_Cost = 0
                Best_Iteration = 100000000
                Best_Time = 10000000000000
                Best_Cost = 10000000000000
                for i in range(iterations):
                    line = line.strip()
                    instance = parse_vrp_file(line)
                    print(instance)
                    Cost,Time,Iteration = main_function(instance, lam, Penalidade)
                    name = instance["name"]
                    Benchmark = instance["optimal_value"]
                    All_Iteration += Iteration
                    All_Time += Time
                    All_Cost += Cost
                    if Cost < Best_Cost:
                        Best_Cost = Cost
                    if Time < Best_Time:
                        Best_Time = Time
                    if Iteration < Best_Iteration:
                        Best_Iteration = Iteration
                with open("..\Resultados.txt", 'a', encoding='utf-8') as Output:
                    Output.write("*************************************************************************************************************\n")
                    Output.write(f" Nome da Instancia: {name}\n")
                    Output.write(f"[Melhor Custo: {Best_Cost:.2f} | Media do Custo: {All_Cost/5:.2f} | Melhor Custo Possivel: {Benchmark:.2f}]\n")
                    Output.write(f"[Melhor Tempo: {Best_Time:.2f} | Media de Tempo: {All_Time/5:.2f}]\n")
                    Output.write(f"[Melhor Numero de Iterações: {Best_Iteration:.2f} | Media do Numero de Iterações: {All_Iteration/5:.2f}]\n")
                    Output.write(f"[Melhor Gap: {(Best_Cost - Benchmark) / Benchmark:.5f} | Media da Gap: {((All_Cost / 5) - Benchmark) / Benchmark:.5f}]\n")
                    Output.write("*************************************************************************************************************\n")

                    print(name)

def main_function(Instance, Lambda, Penalidade):

    distance_matrix = np.zeros((Instance["dimension"] + 1, Instance["dimension"] + 1), dtype=float)
    for i in range(Instance["dimension"]):
        for j in range(Instance["dimension"]):
            distance_matrix[i+1][j+1] = euclidean_distance(Instance["coordinates"][i], Instance["coordinates"][j])      
    distance_matrix = np.round(distance_matrix, 0)
    final_solution, time, iteration = guided_local_search(Instance, distance_matrix, Lambda, Penalidade)
    # plot_grapfh(Instance, distance_matrix, Lambda, Penalidade)
    Total_Cost = calculate_solution_cost(final_solution, distance_matrix, 1)
    return Total_Cost, time, iteration 


vrp_file_path = "..\Lista de Execucao.txt"
get_path_instance(vrp_file_path,0.3,1)





