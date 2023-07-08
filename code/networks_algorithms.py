import numpy as np


# get_manhattan_distance function is used to find the Manhattan distance between two vertices
def get_manhattan_distance(first_vertex, second_vertex):
    return abs(first_vertex[0] - second_vertex[0]) + abs(first_vertex[1] - second_vertex[1])

# sort_vertices_by_manhattan_distance function is used to sort a list of vertices by their Manhattan distance from a target vertex
def sort_vertices_by_manhattan_distance(vertices, target_vertex):
    vertices_to_sort = sorted(vertices, key = lambda vertex: vertex[0])
    return sorted(vertices_to_sort, key = lambda vertex: get_manhattan_distance(vertex, target_vertex))

# create_adjacency_matrix function is used to create an adjacency matrix from a graph represented by a list of vertices,
# a list of edges and a list of edges directionalities
def create_adjacency_matrix(graph_vertices, graph_edges, graph_edges_directionalities):
    adjacent_matrix = np.zeros((len(graph_vertices), len(graph_vertices)), dtype = int)
    for edge_number in range(len(graph_edges)):  # go through all the edges of the graph
        left_vertex = graph_vertices.index(graph_edges[edge_number][0])  # one of the vertices of the edge
        right_vertex = graph_vertices.index(graph_edges[edge_number][1])  # the other vertex of the edge
        edge_connection = graph_edges_directionalities[edge_number]  # the directionality of the edge
        if edge_connection == "←":  # there is an edge from right_vertex to left_vertex
            adjacent_matrix[right_vertex][left_vertex] = 1
        if edge_connection == "→":  # there is an edge from left_vertex to right_vertex
            adjacent_matrix[left_vertex][right_vertex] = 1
        if edge_connection == "↔":  # there is an edge from left_vertex to right_vertex and from right_vertex to left_vertex
            adjacent_matrix[left_vertex][right_vertex] = 1
            adjacent_matrix[right_vertex][left_vertex] = 1
    return(adjacent_matrix)

# create_adjacency_list function is used to create an adjacency list from a graph represented by a list of vertices,
# a list of edges and a list of edges directionalities
def create_adjacency_list(graph_vertices, graph_edges, graph_edges_directionalities):
    adjacency_list = {key: [] for key in range(1, len(graph_vertices) + 1)}
    for edge_number in range(len(graph_edges)):  # go through all the edges of the graph
        left_vertex = graph_vertices.index(graph_edges[edge_number][0]) + 1  # one of the vertices of the edge
        right_vertex = graph_vertices.index(graph_edges[edge_number][1]) + 1  # the other vertex of the edge
        edge_connection = graph_edges_directionalities[edge_number]  # the directionality of the edge
        if edge_connection == "←":  # there is an edge from right_vertex to left_vertex
            adjacency_list[right_vertex].append(left_vertex)
        elif edge_connection == "→":  # there is an edge from left_vertex to right_vertex
            adjacency_list[left_vertex].append(right_vertex)
        elif edge_connection == "↔":  # there is an edge from left_vertex to right_vertex and from right_vertex to left_vertex
            adjacency_list[right_vertex].append(left_vertex)
            adjacency_list[left_vertex].append(right_vertex)
    return(adjacency_list)

# check_graph_connectivity function is used to check if a graph represented by an adjacency list is fully connected
def check_graph_connectivity(graph):  # graph is an adjacency list
    visited_vertices_total = []  # list of sets of visited vertices
    not_visited_vertices_total = []  # list of lists of not visited vertices
    for vertex in range(1, len(graph) + 1):  # go through all the vertices of the graph
        visited_vertices = set()  # set of visited vertices
        dfs_for_connectivity(graph, vertex, visited_vertices)  # perform a depth first search on the vertex
        visited_vertices_total.append(visited_vertices)  # add the set of visited vertices to the list of visited vertices
    is_graph_connected = True  # suppose the graph is connected
    for k in range(len(visited_vertices_total)):  # go through all the sets of visited vertices
        # add the list of not visited vertices to the list of not visited vertices
        not_visited_vertices_total.append([vertex for vertex in range(1, len(graph) + 1) if vertex not in visited_vertices_total[k]])
        if not_visited_vertices_total[k] != []:  # if there are not visited vertices
            is_graph_connected = False  # the graph is not connected
    return not_visited_vertices_total, is_graph_connected  # return the list of not visited vertices

# depth_first_search function is used to perform a depth first search on a graph represented by an adjacency list
def dfs_for_connectivity(graph, vertex, visited_vertices):
    visited_vertices.add(vertex)  # visited vertices are added to the set of visited_vertices
    for neighbor in graph[vertex]:  # go through all the neighbors of the vertex
        if neighbor not in visited_vertices:  # if the neighbor has not been visited yet
            dfs_for_connectivity(graph, neighbor, visited_vertices)  # perform a depth first search on the neighbor

# find_all_circles function is used to find all circles in a graph represented by an adjacency list
def find_all_circles(graph):
    circles = []
    visited = set()
    for node in graph:
        if node not in visited:
            dfs_for_circles(graph, node, visited, [node], circles)
    return circles

# dfs_for_circles function is used to perform a depth first search on a graph represented by an adjacency list
# and record all circles found in the graph
def dfs_for_circles(graph, current_node, visited_vertices, path, circles):
    visited_vertices.add(current_node)
    for neighbor in graph[current_node]:
        if neighbor not in visited_vertices:
            dfs_for_circles(graph, neighbor, visited_vertices, path + [neighbor], circles)
        elif neighbor == path[0]:
            circles.append(path + [neighbor])

# the heuristic function used for the A* algorithm
def heuristic_function(node, goal_node):
    return get_manhattan_distance(node, goal_node)  # Manhattan distance as heuristic_function

# the implementation of the A* algorithm
def A_star(adjacency_list, costs_list, start_node, goal_node):
    open_list = [start_node]  # start node first in the open list
    parents = {}  # it contains the parent nodes for each node
    parents[start_node] = start_node  # the start node has itself as parent
    g_scores = {node: float('inf') for node in adjacency_list}  # g-score (cost from start node to current node)
    g_scores[start_node] = 0
    f_scores = {node: float('inf') for node in adjacency_list}  # f-score (g-score + heuristic)
    f_scores[start_node] = heuristic_function(start_node, goal_node)  # set the f-score of the start node to the heuristic function value
    while len(open_list) != 0:  # while the open list is not empty
        for node in open_list:  # go through all the nodes in the open list
            f_scores[node] = g_scores[node] + heuristic_function(node, goal_node)  # calculate the f-score for each node in the open list
        current_node = min(open_list, key = lambda x: f_scores[x])  # get the node with the minimum f-score
        if current_node == goal_node:  # if goal node is reached, return the path
            path = []  # list of nodes in the path
            while parents[current_node] != current_node:  # go through the parents of the nodes in the path
                path.append(current_node)  # add the current node to the path
                current_node = parents[current_node]  # go to the parent of the current node
            path.append(start_node)  # add the start node to the path
            path.reverse()  # reverse the path
            return path  # return the path
        open_list.remove(current_node)  # remove the current node from the open list
        for neighbor_node in adjacency_list[current_node]:  # go through the neighbors of the current node
            nodes_pair = tuple(sort_vertices_by_manhattan_distance([current_node, neighbor_node], (0, 0)))
            tentative_gScore  = g_scores[current_node] + costs_list[nodes_pair]  # calculate the tentative g-score
            # if tentative_gScore is more than the g-score of the neighbor node
            if tentative_gScore < g_scores[neighbor_node]:
                parents[neighbor_node] = current_node  # set the parent of the neighbor node to the current node
                g_scores[neighbor_node] = tentative_gScore  # set the g-score of the neighbor node to the calculated tentative_gScore
                # set the f-score of the neighbor node to the calculated f-score
                f_scores[neighbor_node] = g_scores[neighbor_node] + heuristic_function(neighbor_node, goal_node)
                if neighbor_node not in open_list:  # if the neighbor node is not in the open list
                    open_list.append(neighbor_node)  # add the neighbor node to the open list
    return None  # if the goal node is not reached, return None
