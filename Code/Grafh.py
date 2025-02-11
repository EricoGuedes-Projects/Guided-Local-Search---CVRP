import numpy as np
import matplotlib.pyplot as plt
import copy
import time
from Guided_Local_Search import  create_random_initial_solution, apply_gls_penalty, choose_penalty_features
from Local_Search import local_search
from Utilities import calculate_solution_cost


def plot_grapfh(Instance, distance_matrix, LAMBDA, Penalidade, time_limit=300):
    Iterations = 0
    penalyzed_distance_matrix = np.copy(distance_matrix)
    penalties = np.zeros_like(penalyzed_distance_matrix)
    initial_solution = create_random_initial_solution(Instance)
    solution = local_search(initial_solution, penalyzed_distance_matrix, Instance)
    Best_Cost = calculate_solution_cost(solution, distance_matrix, Instance["depot"])
    Best_Solution = copy.deepcopy(solution)
    iteration_list = [0]
    best_cost_values = [Best_Cost]
    new_cost_values = [Best_Cost]
    start_time = time.time()
    while time.time() - start_time < time_limit:
        Iterations += 1
        features = choose_penalty_features(solution, penalties, penalyzed_distance_matrix)  
        for (i, j) in features:
            penalties[i][j] += 1
        penalyzed_distance_matrix = apply_gls_penalty(distance_matrix, penalties, LAMBDA, Penalidade)
        solution = local_search(solution, penalyzed_distance_matrix, Instance)
        new_cost = calculate_solution_cost(solution, distance_matrix, Instance["depot"])
        if new_cost < Best_Cost:
            Best_Cost = new_cost
            Best_Solution = copy.deepcopy(solution)
        iteration_list.append(Iterations)
        best_cost_values.append(Best_Cost)
        new_cost_values.append(new_cost)
    Best_Solution = local_search(Best_Solution, distance_matrix, Instance)
    optiml_value = Instance["optimal_value"]
    plt.figure(figsize=(10, 6))

    # Plota o melhor valor encontrado até agora
    plt.plot(iteration_list, best_cost_values, label="Melhor custo até agora", color='b', linewidth=2)

    # Plota o valor da solução gerada a cada iteração
    plt.plot(iteration_list, new_cost_values, label="Custo da solução na iteração", color='g', linestyle='dashed', linewidth=2)

    # Linha horizontal representando o valor ótimo
    plt.axhline(y=optiml_value, color='r', linestyle='--', label=f"Ótimo ({optiml_value:.2f})")

    # Adicionando rótulos e título
    plt.xlabel("Iterações")
    plt.ylabel("Valor da Função Objetivo")
    plt.title("Evolução da Solução - Instância A-n32-k5")
    plt.legend()
    plt.grid(True)

    # Exibir o gráfico
    plt.show()