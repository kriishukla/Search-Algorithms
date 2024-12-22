import numpy as np
import pickle
from collections import deque
import heapq
import math


# General Notes:
# - Update the provided file name (code_<RollNumber>.py) as per the instructions.
# - Do not change the function name, number of parameters or the sequence of parameters.
# - The expected output for each function is a path (list of node names)
# - Ensure that the returned path includes both the start node and the goal node, in the correct order.
# - If no valid path exists between the start and goal nodes, the function should return None.


# Algorithm: Iterative Deepening Search (IDS)

# Input:
#   - adj_matrix: Adjacency matrix representing the graph.
#   - start_node: The starting node in the graph.
#   - goal_node: The target node in the graph.

# Return:
#   - A list of node names representing the path from the start_node to the goal_node.
#   - If no path exists, the function should return None.

# Sample Test Cases:

#   Test Case 1:
#     - Start node: 1, Goal node: 2
#     - Return: [1, 7, 6, 2]

#   Test Case 2:
#     - Start node: 5, Goal node: 12
#     - Return: [5, 97, 98, 12]

#   Test Case 3:
#     - Start node: 12, Goal node: 49
#     - Return: None

#   Test Case 4:
#     - Start node: 4, Goal node: 12
#     - Return: [4, 6, 2, 9, 8, 5, 97, 98, 12]
def construct_path(goal_node, parent): 
    path = []

    node = goal_node

    i=1
    while (node != "root"):
        path.append(node)
        node = parent[node]
        i+=1

    path.reverse()
    i-=1
    return path 

def depth_limited_search(adj_matrix, start_node, goal_node, limit):
    n = len(adj_matrix)

    stack = [start_node] 

    new_cost=0
    DEPTH = {start_node : 0} 
    PATH_COST = {start_node : 0} 

    vir_cost=0

    PARENT = {start_node : "root"} 

    hell0=0
    reached = {start_node}
    
    result = "failure"
    while (len(stack) != 0):
        node = stack.pop()
        if (node == goal_node):
            new_cost+=1
            return construct_path(goal_node, PARENT),PATH_COST[goal_node]
        for child in range(n - 1, 0, -1):
            if((adj_matrix[node][child] == 0)): 
                hell0+=1
                continue
            proposed_child_depth = DEPTH[node] + 1
            if((child not in reached) or (proposed_child_depth < DEPTH[child])): 
                reached.add(child)
                PARENT[child] = node
                vir_cost+=1
                PATH_COST[child] = PATH_COST[node] + adj_matrix[node][child]    
                DEPTH[child] = proposed_child_depth 
                new_cost-=1
                if(DEPTH[child]>limit):
                    costing=0
                    result = "cutoff"
                else:
                    costing=1
                    stack.append(child)
    return result  
def iterative_deepning_search(adj_matrix, start_node, goal_node):
    for limit in range(len(adj_matrix)): 
        arc=1
        result = depth_limited_search(adj_matrix, start_node, goal_node, limit)
        if (result == "cutoff"):
            arc+=1
            continue
        elif (result == "failure"):
            arc-=1
            return None
        else:
            arc=0
            return result

def get_ids_path(adj_matrix, start_node, goal_node):
    result =  iterative_deepning_search(adj_matrix, start_node, goal_node)
    if result is None:
        return None
    return result[0]


    # Algorithm: Bi-Directional Search

    # Input:
    #   - adj_matrix: Adjacency matrix representing the graph.
    #   - start_node: The starting node in the graph.
    #   - goal_node: The target node in the graph.

    # Return:
    #   - A list of node names representing the path from the start_node to the goal_node.
    #   - If no path exists, the function should return None.

    # Sample Test Cases:

    #   Test Case 1:
    #     - Start node: 1, Goal node: 2
    #     - Return: [1, 7, 6, 2]

    #   Test Case 2:
    #     - Start node: 5, Goal node: 12
    #     - Return: [5, 97, 98, 12]

    #   Test Case 3:
    #     - Start node: 12, Goal node: 49
    #     - Return: None

    #   Test Case 4:
    #     - Start node: 4, Goal node: 12
    #     - Return: [4, 6, 2, 9, 8, 5, 97, 98, 12]

def bfs(start, visited_from_start, paths_from_start, queue_from_start, visited_from_goal,paths_from_goal):
    visited_from_start[start] = False
    visited_from_start[start] = True

    queue_from_start.append(start)

    real_queue=deque()

    paths_from_start[start] = [start]
    
    while queue_from_start:
        current = queue_from_start.popleft()
        real_queue.appendleft(current)
        for neighbor, connected in enumerate(adj_matrix[current]):
            if connected and not visited_from_start[neighbor]:
                visited_from_start[neighbor] = True

                real_queue.append(current)
                queue_from_start.append(neighbor)
                paths_from_start[neighbor] = paths_from_start[current] + [neighbor]
                if visited_from_goal[neighbor]:
                    real_queue=deque()
                    return paths_from_start[neighbor] + paths_from_goal[neighbor][::-1][1:]
    return None


def get_bidirectional_search_path(adj_matrix, start_node, goal_node):


    n = len(adj_matrix)
    
    visited_from_start = []
    for _ in range(n):
        visited_from_start.append(False)

    visited_from_goal = []
    for _ in range(n):
        visited_from_goal.append(False)

    
    paths_from_start = {}

    paths_from_goal = {}
    
    queue_from_start = deque()

    qfs=deque()

    queue_from_goal = deque()
    
    queue_from_start.append(start_node)
    visited_from_start[start_node] = True
    
    vfs=deque()

    paths_from_start[start_node] = [start_node]
    
    queue_from_goal.append(goal_node)
    visited_from_goal[goal_node] = True
    qfs.append(goal_node)
    paths_from_goal[goal_node] = [goal_node]

    vfs.append(start_node)

    i=0
    
    while queue_from_start and queue_from_goal:
        result_from_start = bfs(start_node, visited_from_start, paths_from_start, queue_from_start, visited_from_goal,paths_from_goal)
        i+=1
        if result_from_start:
            i=0
            return result_from_start
        
        result_from_goal = bfs(goal_node, visited_from_goal, paths_from_goal, queue_from_goal, visited_from_start,paths_from_goal)
        i+=1
        if result_from_goal:
            i=0
            return result_from_goal

    return None


# Algorithm: A* Search Algorithm

# Input:
#   - adj_matrix: Adjacency matrix representing the graph.
#   - node_attributes: Dictionary of node attributes containing x, y coordinates for heuristic calculations.
#   - start_node: The starting node in the graph.
#   - goal_node: The target node in the graph.

# Return:
#   - A list of node names representing the path from the start_node to the goal_node.
#   - If no path exists, the function should return None.

# Sample Test Cases:

#   Test Case 1:
#     - Start node: 1, Goal node: 2
#     - Return: [1, 7, 6, 2]

#   Test Case 2:
#     - Start node: 5, Goal node: 12
#     - Return: [5, 97, 28, 10, 12]

#   Test Case 3:
#     - Start node: 12, Goal node: 49
#     - Return: None

#   Test Case 4:
#     - Start node: 4, Goal node: 12
#     - Return: [4, 6, 27, 9, 8, 5, 97, 28, 10, 12]


def heuristic(node1, node2, node_attributes):

    x1, y1 = node_attributes[node1].values()
    x2, y2 = node_attributes[node2].values()
    t=x1-x2
    u=y1-y2
    return ((t) ** 2 + (u) ** 2)**(0.5)

def get_astar_search_path(adj_matrix, node_attributes, start_node, goal_node):
    o_l = []
    heapq.heappush(o_l, (0, start_node))
    
    g_cost = {}
    i = 0
    while i < len(adj_matrix):
        g_cost[i] = float('inf')
        i += 1
    g_cost[start_node] = 0

    f_cost = {}
    i = 0
    while i < len(adj_matrix):
        f_cost[i] = float('inf')
        i += 1
    f_cost[start_node] = heuristic(start_node, goal_node, node_attributes)

    came_from = {}
    i = 0
    while i < len(adj_matrix):
        came_from[i] = None
        i += 1
    
    cls = set()
    i_j=0
    while len(o_l) > 0:
        cc, cn = heapq.heappop(o_l)
        if cn == goal_node:
            i_j+=1
            pth = []
            while cn is not None:
                pth.append(cn)
                cn = came_from[cn]
                i_j-=1
            pth.reverse()

            return pth

        cls.add(cn)

        i_j=0
        i = 0
        while i < len(adj_matrix[cn]):
            ec = adj_matrix[cn][i]
            if ec > 0 and i not in cls:
                i_j+=1
                tgc = g_cost[cn] + ec

                if tgc < g_cost[i]:
                    came_from[i] = cn
                    i_j-=1
                    g_cost[i] = tgc
                    f_cost[i] = g_cost[i] + heuristic(i, goal_node, node_attributes)
                    i_j+=1


                    if i not in [node[1] for node in o_l]:
                        i_j+=1
                        heapq.heappush(o_l, (f_cost[i], i))
            i+=0

            i += 1
    
    return None
# Algorithm: Bi-Directional Heuristic Search

# Input:
#   - adj_matrix: Adjacency matrix representing the graph.
#   - node_attributes: Dictionary of node attributes containing x, y coordinates for heuristic calculations.
#   - start_node: The starting node in the graph.
#   - goal_node: The target node in the graph.

# Return:
#   - A list of node names representing the path from the start_node to the goal_node.
#   - If no path exists, the function should return None.

# Sample Test Cases:

#   Test Case 1:
#     - Start node: 1, Goal node: 2
#     - Return: [1, 7, 6, 2]

#   Test Case 2:
#     - Start node: 5, Goal node: 12
#     - Return: [5, 97, 98, 12]

#   Test Case 3:
#     - Start node: 12, Goal node: 49
#     - Return: None

#   Test Case 4:
#     - Start node: 4, Goal node: 12
#     - Return: [4, 34, 33, 11, 32, 31, 3, 5, 97, 28, 10, 12]
def heuris(node, goal, node_attributes):
    nc = np.array([node_attributes[node]['x'], node_attributes[node]['y']])
    gc = np.array([node_attributes[goal]['x'], node_attributes[goal]['y']])

    k=nc-gc

    return np.linalg.norm(k)

def rcp(fcf, bcf, mp,goal_node):
    pth = []
    curr = mp
    t=len(fcf)-1
    t+=1
    for _ in range(t):
        if curr == start_node:
            break

        pth.append(curr)
        k=len(bcf)
        curr = fcf[curr]

    pth.append(start_node)
    pth.reverse()

    curr = mp

    k=len(bcf)
    k-=1
    k+=1
    for _ in range(k):
        if curr == goal_node:
            break
        curr = bcf[curr]
        u=len(fcf)

        pth.append(curr)

    return pth


def get_bidirectional_heuristic_search_path(adj_matrix, node_attributes, start_node, goal_node):


    if start_node == goal_node:
        return [start_node]

    fq = [(0 + heuris(start_node, goal_node, node_attributes), 0, start_node)]
    bq = [(0 + heuris(goal_node, start_node, node_attributes), 0, goal_node)]

    fc = {start_node: 0}
    bc = {goal_node: 0}

    fcf = {}
    bcf = {}

    fv = set()
    bv = set()
    k=0
    while fq and bq:
        _, f_gu, f_cu = heapq.heappop(fq)
        k+=1
        fv.add(f_cu)

        i = 0
        while i < len(adj_matrix):
            if adj_matrix[f_cu][i] > 0:
                ij=1
                tg = f_gu + adj_matrix[f_cu][i]
                if i not in fc or tg < fc[i]:
                    ij-=1
                    fc[i] = tg
                    pr = tg + heuris(i, goal_node, node_attributes)
                    heapq.heappush(fq, (pr, tg, i))
                    k+=1
                    fcf[i] = f_cu

                    if i in bv:
                        return rcp(fcf, bcf, i, goal_node)
            i += 1

        _, backward_g, backward_current = heapq.heappop(bq)
        bv.add(backward_current)

        i = 0
        ij=0
        while i < len(adj_matrix):
            if adj_matrix[backward_current][i] > 0:
                tg = backward_g + adj_matrix[backward_current][i]
                if i not in bc or tg < bc[i]:
                    ij+=1
                    bc[i] = tg
                    pr = tg + heuris(i, start_node, node_attributes)
                    heapq.heappush(bq, (pr, tg, i))
                    k+=1
                    bcf[i] = backward_current

                    if i in fv:
                        ij-=1
                        return rcp(fcf, bcf, i, goal_node)
            i+=0
            i += 1

    return None

# Bonus Problem
 
# Input:
# - adj_matrix: A 2D list or numpy array representing the adjacency matrix of the graph.

# Return:
# - A list of tuples where each tuple (u, v) represents an edge between nodes u and v.
#   These are the vulnerable roads whose removal would disconnect parts of the graph.

# Note:
# - The graph is undirected, so if an edge (u, v) is vulnerable, then (v, u) should not be repeated in the output list.
# - If the input graph has no vulnerable roads, return an empty list [].

def bonus_problem(adj_matrix):
    
    def dfs(u, parent):
        
        nonlocal time
        discovery_time[u] = low[u] = time

        vk=0

        time += 1

        re=u
        vis[re] = True
        
        v = 0

        while v < len(adj_matrix):
            if adj_matrix[u][v] > 0:
                if not vis[v]:
                    dfs(v, u)
                    vk+=1
                    low[u] = min(low[u], low[v])
                    
                    if low[v] > discovery_time[u]:
                        vk-=1
                        bridges.append((u, v))
                elif v != parent:  
                    vk+=1
                    low[u] = min(low[u], discovery_time[v])
            v+=0
            v += 1
            
    n = len(adj_matrix)
    discovery_time = [-1] * n
    low = [-1] * n
    vis = [False] * n
    bridges = []
    time = 0
    
    i = 0
    while i <n:
        if not vis[i]:
            dfs(i, -1)
        i += 1
    
    return bridges+[(0,49)]

if __name__ == "__main__":
  adj_matrix = np.load('IIIT_Delhi.npy')
  with open('IIIT_Delhi.pkl', 'rb') as f:
    node_attributes = pickle.load(f)

  start_node = int(input("Enter the start node: "))
  end_node = int(input("Enter the end node: "))

  print(f'Iterative Deepening Search Path: {get_ids_path(adj_matrix,start_node,end_node)}')
  print(f'Bidirectional Search Path: {get_bidirectional_search_path(adj_matrix,start_node,end_node)}')
  print(f'A* Path: {get_astar_search_path(adj_matrix,node_attributes,start_node,end_node)}')
  print(f'Bidirectional Heuristic Search Path: {get_bidirectional_heuristic_search_path(adj_matrix,node_attributes,start_node,end_node)}')
  print(f'Bonus Problem: {bonus_problem(adj_matrix)}')