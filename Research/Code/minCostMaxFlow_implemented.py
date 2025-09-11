import heapq

def SuccessiveShortestPath(graph, source, sink):
    """
    Implements the Successive Shortest Path algorithm for finding the minimum cost maximum flow in a flow network.
    
    Parameters:
    graph: A directed graph represented as an adjacency list where each edge has a capacity and a cost.
    source: The source node in the flow network.
    sink: The sink node in the flow network.
    

    Returns:
    A tuple containing the maximum flow and the minimum cost.
    """
    # nodes: A list of nodes in the graph.
    nodes = list(graph.keys())

    # edges: A list of edges in the graph, where each edge is represented as a tuple (u, v, capacity, cost).
    edges = []
    # Since the graph is directed, we need to iterate through each node and its neighbors.
    for node in graph:
        for d in graph[node]:
            edges.append((node, d, graph[node][d][0], graph[node][d][1]))

    # Initialize flow and cost
    flow = dict(((edge[0], edge[1]), 0) for edge in edges) # Flow for each edge is 0
    potentials = dict((node, 0) for node in nodes) # Cost for each node is 0
    residual_graph = graph.copy() # Create a residual graph from the original graph
    # while there is a shortest path from source to sink in the residual graph
    while True:
        # Find the shortest path in the reduced costs(take into account the potentials)
        dist, prevArc = Dijkstra_wPotentials(residual_graph, edges, potentials, source, sink)

        # No shortest path was found
        if prevArc[sink] is None:
            break

        for x in nodes:
            # if dist[x] < float('inf'):
            if dist[x] != -1:
                # Update the potentials for each node
                potentials[x] = potentials[x] + dist[x]

        # Add a flow of 1 along the path given by prevArcs and generate a residual graph.
        augment_flow(prevArc, 1, flow, residual_graph, source, sink)
    
    return flow

def augment_flow(prevArc, flow_value, flow, graph, source, sink):
    """
    Augments the flow along the path given by prevArc.
    
    Parameters:
    prevArc: A dictionary mapping each node to its previous arc in the path.
    flow_value: The value of flow to be augmented.
    flow: The current flow in the network.
    graph: The residual graph where the flow is being augmented.
    
    Returns:
    None. The function modifies the flow and graph in place.
    """
    current = sink
    while current != source:
        prev = prevArc[current]
        if prev is None:
            break
        # Update the flow on the edge
        flow[prev] += flow_value
        # Update the residual graph
        graph[prev[0]][prev[1]] = (graph[prev[0]][prev[1]][0] - flow_value, graph[prev[0]][prev[1]][1])
        current = prev[0]

def Dijkstra_wPotentials(residual_graph, edges, potentials, source, sink):
    # Implements Dijkstra's algorithm with potentials to find the shortest path in a residual graph.
    nodes = list(residual_graph.keys())
    dist = dict()
    prevArc = dict()
    for node in nodes:
        # dist[node] = float('inf')
        dist[node] = -1
        prevArc[node] = None
    dist[source] = 0
    Q = []
    visited = set()
    heapq.heappush(Q, (source, 0)) # Only has a minimum heap implementation
    
    while len(Q) > 0:
        (u, du) = heapq.heappop(Q)

        # if du > dist[u]:
        if dist[u] != -1 and du > dist[u]:
            continue
        
        if u in visited:
            continue
        visited.add(u)

        if u == sink:
            break

        for v, (capacity, cost) in residual_graph[u].items():
            if capacity <= 0:
                continue
            reducedCost = cost - potentials[u] + potentials[v]
            if dist[u] + reducedCost > dist[v]:
                dist[v] = dist[u] + reducedCost
                prevArc[v] = (u, v)
                for i in range(len(Q)):
                    if Q[i][0] == v:
                        Q[i] = (v, dist[v])
                        break
                else:
                    heapq.heappush(Q, (v, dist[v]))
                    
    return dist, prevArc

graph = dict()
# graph["So"] = {"PA":(1, 0), "PB":(1, 0), "PC":(1, 0)}
# graph["PA"] = {"EA":(1, 10), "EB":(1, 20)}
# graph["PB"] = {"EA":(1, 16), "EB":(1, 12)}
# graph["PC"] = {"EA":(1, 5), "EC":(1, 8)}
# graph["EA"] = {"Si":(1, 0)}
# graph["EB"] = {"Si":(1, 0)}
# graph["EC"] = {"Si":(1, 0)}
# graph["Si"] = {}
# graph = {'So': {(0,): (1, 0), (1,): (1, 0)}, (0,): { "E": (1, 5)}, (1,): {"E": (1, -8)},
#   "E": {'Si': (1, 0)}, 'Si': {}}
# print(SuccessiveShortestPath(graph, "So", "Si"))