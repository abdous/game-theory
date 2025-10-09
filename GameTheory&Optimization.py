import networkx as nx
import matplotlib.pyplot as plt
import matplotlib.lines as mlines


def min_planes_via_flow(a, b, r):
    """
    Solve the minimum number of planes problem using
    a max-flow / min-cut reduction.

    Parameters
    ----------
    a[i], b[i]: start and finish time of flight i
    r[i][j]: reposition time from end of flight i to start of flight j

    Returns
    -------
    min_planes : int
        Minimum number of planes required.
    flow_value : int
        Maximum flow value (number of feasible connections used).
    flow_dict : dict
        Flow dictionary from networkx (shows used edges).
    G : nx.DiGraph
        The constructed flow network.
    source, sink : str
        Source and sink node names.
    """
    p = len(a)
    G = nx.DiGraph()
    source, sink = "s", "t"

    # Left (L1..Lp) = flight start nodes
    # Right (R1..Rp) = flight end nodes
    left_nodes = [f"L{i+1}" for i in range(p)]
    right_nodes = [f"R{i+1}" for i in range(p)]

    # Connect source → left, right → sink
    for u in left_nodes:
        G.add_edge(source, u, capacity=1)
    for v in right_nodes:
        G.add_edge(v, sink, capacity=1)

    # Feasible transitions: flight i can be followed by flight j
    for i in range(p):
        for j in range(p):
            if i != j and b[i] + r[i][j] <= a[j]:
                G.add_edge(f"L{i+1}", f"R{j+1}", capacity=1)

    # Compute maximum flow
    flow_value, flow_dict = nx.maximum_flow(G, source, sink)
    min_planes = p - flow_value

    return min_planes, flow_value, flow_dict, G, source, sink
