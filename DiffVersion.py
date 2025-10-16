import networkx as nx
import matplotlib.pyplot as plt
import matplotlib.lines as mlines


def reconstruct_schedules(flow_dict, p):
    """
    Reconstruct the actual flight sequences for each plane from the flow solution.

    Parameters
    ----------
    flow_dict : dict
        Flow dictionary from networkx showing used edges.
    p : int
        Number of flights.

    Returns
    -------
    schedules : list of list
        Each sublist contains the flight sequence for one plane.
    """
    # Build adjacency from flow (only edges with flow > 0)
    # Edge from Li to Rj means flight i is followed by flight j
    adjacency = {}
    in_degree = {i: 0 for i in range(p)}

    for i in range(p):
        Li = f"L{i+1}"
        if Li in flow_dict:
            for Rj, flow_val in flow_dict[Li].items():
                if Rj.startswith("R") and flow_val > 0:
                    j = int(Rj[1:]) - 1
                    adjacency[i] = j
                    in_degree[j] += 1

    # Find starting flights (no incoming edges in flow)
    schedules = []
    visited = set()

    for start_flight in range(p):
        if start_flight not in visited and in_degree[start_flight] == 0:
            # Build chain starting from this flight
            chain = []
            current = start_flight

            while current is not None and current not in visited:
                visited.add(current)
                chain.append(current + 1)  # Convert to 1-indexed
                current = adjacency.get(current)

            if chain:
                schedules.append(chain)

    return schedules


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
    schedules : list of list
        Flight sequences for each plane.
    """
    p = len(a)
    G = nx.DiGraph()
    source, sink = "s", "t"

    # Left (L1..Lp) = flight start nodes
    # Right (R1..Rp) = flight end nodes
    left_nodes = [f"L{i+1}" for i in range(p)]
    right_nodes = [f"R{i+1}" for i in range(p)]

    # Connect source -> left, right -> sink
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

    # Reconstruct schedules
    schedules = reconstruct_schedules(flow_dict, p)

    return min_planes, flow_value, flow_dict, G, source, sink, schedules


def plot_flow_network(a, b, r, title="Flight Flow Network"):
    """
    Calls the algorithm and plots the flow network.
    Colors:
      Red = transition used in flow
      Green = feasible but unused
      Black = source/sink edges
    """
    min_planes, flow_value, flow_dict, G, source, sink, schedules = min_planes_via_flow(
        a, b, r
    )
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

    # Adding labels for edges (flow, capacity)
    edge_labels = {}
    for u, v in G.edges():
        flow_val = flow_dict.get(u, {}).get(v, 0)
        cap = G[u][v].get("capacity", 0)
        if u.startswith("L") and v.startswith("R"):
            edge_labels[(u, v)] = f"({flow_val},{cap})"
        elif u == source or v == sink:
            edge_labels[(u, v)] = f"({flow_val},{cap})"
        else:
            edge_labels[(u, v)] = ""

    nx.draw_networkx_edge_labels(
        G,
        pos,
        edge_labels=edge_labels,
        label_pos=0.3,
        font_color="blue",
        ax=ax,
    )

    plt.axis("off")
    plt.title(
        f"{title}\n(Max Flow = {flow_value}, Min Planes = {min_planes})",
        fontsize=12,
    )

    # Arrow Legend
    red_line = mlines.Line2D(
        [], [], color="red", marker=">", markersize=10, label="Used transition"
    )
    green_line = mlines.Line2D(
        [], [], color="green", marker=">", markersize=10, label="Feasible (unused)"
    )
    ax.legend(handles=[red_line, green_line], loc="upper right")

    plt.show()

    # Text summary
    print("\n" + "=" * 60)
    print("FLIGHT SCHEDULES PER PLANE")
    print("=" * 60)
    for plane_idx, schedule in enumerate(schedules, 1):
        print(f"\nPlane {plane_idx}: Flights {schedule}")
        for seq_idx, flight_num in enumerate(schedule):
            flight_idx = flight_num - 1
            start_time = a[flight_idx]
            end_time = b[flight_idx]

            if seq_idx == 0:
                print(f"  Flight {flight_num}: {start_time:.1f} -> {end_time:.1f}")
            else:
                prev_flight_num = schedule[seq_idx - 1]
                prev_idx = prev_flight_num - 1
                turnaround = r[prev_idx][flight_idx]
                arrival_at_origin = b[prev_idx] + turnaround
                wait_time = start_time - arrival_at_origin
                print(
                    f"  Flight {flight_num}: {start_time:.1f} -> {end_time:.1f} "
                    f"(turnaround: {turnaround:.1f}h, wait: {wait_time:.1f}h)"
                )

    print("\n" + "=" * 60)
    print("FEASIBLE TRANSITIONS")
    print("=" * 60)
    for i in range(p):
        for j in range(p):
            if i != j and b[i] + r[i][j] <= a[j]:
                used = flow_dict.get(f"L{i+1}", {}).get(f"R{j+1}", 0)
                mark = "[USED]" if used else "[NOT USED]"
                print(f"  L{i+1:<2} -> R{j+1:<2}   {mark}")

    print(f"\n{'='*60}")
    print(f"[+] Maximum flow = {flow_value}")
    print(f"[*] Minimum planes = {min_planes}")
    print(f"{'='*60}\n")


# --- Example ---
a = [8.0, 11.0, 12.0]
b = [10.0, 13.0, 15.0]
r = [[0, 1, 2], [1, 0, 2], [2, 1, 0]]

plot_flow_network(a, b, r)
