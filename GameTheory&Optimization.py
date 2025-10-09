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


def plot_flow_network(a, b, r, title="Flight Flow Network", save_fig=False):
    """
    Calls the algorithm and plots the flow network.
    Colors:
      Red = transition used in flow
      Green = feasible but unused
      Black = source/sink edges
    """
    min_planes, flow_value, flow_dict, G, source, sink = min_planes_via_flow(a, b, r)
    p = len(a)

    left_nodes = [f"L{i+1}" for i in range(p)]
    right_nodes = [f"R{i+1}" for i in range(p)]

    # --- Layout positions ---
    pos = {source: (-1, 0), sink: (3, 0)}
    for i, L in enumerate(left_nodes):
        pos[L] = (0, p - i - 0.5)
    for i, R in enumerate(right_nodes):
        pos[R] = (2, p - i - 0.5)

    # Edge colors
    edge_colors = []
    for u, v in G.edges():
        if u == source or v == sink:
            color = "black"
        elif u.startswith("L") and v.startswith("R"):
            color = "red" if flow_dict.get(u, {}).get(v, 0) > 0 else "green"
        else:
            color = "gray"
        edge_colors.append(color)

    # Plotting the network
    fig, ax = plt.subplots(figsize=(10, 6))
    nx.draw(
        G,
        pos,
        with_labels=True,
        node_color="white",
        edgecolors="black",
        node_size=1800,
        arrowsize=15,
        width=2,
        edge_color=edge_colors,
        font_size=10,
        ax=ax,
    )

    plt.axis("off")
    plt.title(
        f"{title}\n(Max Flow = {flow_value}, Min Planes = {min_planes})",
        fontsize=12,
    )

    # Legend
    red_line = mlines.Line2D(
        [], [], color="red", marker=">", markersize=10, label="Used transition"
    )
    green_line = mlines.Line2D(
        [], [], color="green", marker=">", markersize=10, label="Feasible (unused)"
    )
    ax.legend(handles=[red_line, green_line], loc="upper right")

    plt.show()

    # Text summary
    print("\nFeasible transitions:")
    for i in range(p):
        for j in range(p):
            if i != j and b[i] + r[i][j] <= a[j]:
                used = flow_dict.get(f"L{i+1}", {}).get(f"R{j+1}", 0)
                mark = "🟥 Used" if used else "🟩 Not used"
                print(f"  L{i+1:<2} → R{j+1:<2}   {mark}")

    print(f"\n✅ Maximum flow = {flow_value}")
    print(f"✈️  Minimum planes = {min_planes}")
