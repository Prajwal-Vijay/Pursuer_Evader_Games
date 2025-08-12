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
    for node in graph:
        for d in graph[node]:
            edges.append((node, d[0], d[1], d[2]))

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
            potentials[x] = potentials[x] + dist[x]
        # Add a flow of 1 along the path given by prevArcs and generate a residual graph.
        augment_flow(prevArc, 1)
    
    return flow, cost

def augment_flow(prevArc, flow_value, flow, graph):
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
        graph[prev[0]][prev[1]] -= flow_value
        current = prev[0]
