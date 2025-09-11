# This computes an approximate matching for the U2 = [P\A]^2 , V2 graph
# Yes minCostMaxFlow could be used here, but this is a faster implementation, and is good enough

# Local Search with Size-2 swaps for Bipartite Matching
import copy

from sympy import re

def localSearchMaximum(graph, M):
    """
    graph is one node to another node with flow and cost.
    """
    # Initialize the matching
    matching = M.copy()
    improved = True
    edges = set()
    # Since the graph is directed, we need to iterate through each node and its neighbors.
    for node in graph:
        for d in graph[node]:
            edges.add((node, d))
    reduced_graph = copy.deepcopy(graph)
    while improved:
        improved = False
        unmatched_edges = edges-matching
        for i in range(len(unmatched_edges)):
            for j in range(i+1, len(unmatched_edges)):
                edge1 = list(unmatched_edges)[i]
                edge2 = list(unmatched_edges)[j]
                u1 = edge1[0]
                v1 = edge1[1]
                u2 = edge2[0]
                v2 = edge2[1]
                # Skip if endpoints overlap
                if u1 == u2 or v1 == v2:
                    continue
                # Find current matches of these nodes
                current_match_u1 = None
                current_match_v1 = None
                current_match_u2 = None
                current_match_v2 = None
                for edge in matching:
                    if edge[0] == u1:
                        current_match_u1 = edge
                    if edge[1] == v1:
                        current_match_v1 = edge
                    if edge[0] == u2:
                        current_match_u2 = edge
                    if edge[1] == v2:
                        current_match_v2 = edge
                
                # Case 1: Simple Augmentation (Both nodes unmatched)
                if current_match_u1 is None and current_match_v1 is None and current_match_u2 is None and current_match_v2 is None:
                    matching.add(edge1)
                    matching.add(edge2)
                    improved = True
                    continue
                
                # Case 2: 2-edge cycle swap
                if current_match_u1 is not None and current_match_u2 is not None:
                    v1_prime = current_match_u1[1]
                    v2_prime = current_match_u2[1]

                    # Check if we can form a 2-swap
                    if (u1, v1) in edges and (u2, v2) in edges:
                        # Remove the current matches
                        oldCost = graph[current_match_u1[0]][current_match_u1[1]][1]+graph[current_match_u2[0]][current_match_u2[1]][1]
                        newCost = graph[u1][v1][1] + graph[u2][v2][1]
                        if newCost > oldCost:
                            matching.remove(current_match_u1)
                            matching.remove(current_match_u2)
                            matching.add((u1, v1))
                            matching.add((u2, v2))
                            improved = True
                            continue
                
                # Case 3: Alternating path of length 2
                # u1 matches to v1', but v1 unmatched, u2 unmatched, v2 matched to u2'
                if current_match_u1 is not None and current_match_v1 is None and current_match_u2 is None and current_match_v2 is not None:
                    v1_prime = current_match_u1[1]
                    u2_prime = current_match_v2[0]

                    # Check if we can form a 2-swap
                    if (u1, v1) in edges and (u2, v2) in edges:
                        # Remove the current matches
                        oldCost = graph[current_match_u1[0]][current_match_u1[1]][1]+graph[current_match_v2[0]][current_match_v2[1]][1]
                        newCost = graph[u1][v1][1] + graph[u2][v2][1]
                        if newCost > oldCost:
                            matching.remove(current_match_u1)
                            matching.remove(current_match_v2)
                            matching.add((u1, v1))
                            matching.add((u2, v2))
                            improved = True
                            continue
                
                # Case 4: Similar to case3 but reversed
                if current_match_u1 is None and current_match_v1 is not None and current_match_u2 is not None and current_match_v2 is None:
                    u1_prime = current_match_v1[0]
                    v2_prime = current_match_u2[1]

                    # Check if we can form a 2-swap
                    if (u1, v1) in edges and (u2, v2) in edges:
                        # Remove the current matches
                        oldCost = graph[current_match_v1[0]][current_match_v1[1]][1]+graph[current_match_u2[0]][current_match_u2[1]][1]
                        newCost = graph[u1][v1][1] + graph[u2][v2][1]
                        if newCost > oldCost:
                            matching.remove(current_match_v1)
                            matching.remove(current_match_u2)
                            matching.add((u1, v1))
                            matching.add((u2, v2))
                            improved = True
                            continue
    return matching

# graph = dict()
# graph["So"] = {"PA":(1, 0), "PB":(1, 0), "PC":(1, 0)}
# graph["PA"] = {"EA":(1, 10), "EB":(1, 20)}
# graph["PB"] = {"EA":(1, 16), "EB":(1, 12)}
# graph["PC"] = {"EA":(1, 5), "EC":(1, 8)}
# graph["EA"] = {"Si":(1, 0)}
# graph["EB"] = {"Si":(1, 0)}
# graph["EC"] = {"Si":(1, 0)}
# graph["Si"] = {}
#