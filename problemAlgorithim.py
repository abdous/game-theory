"""Edmonds–Karp (manual) implementation for the Minimum Planes problem.

This module implements the Edmonds–Karp max-flow algorithm from scratch:
- BFS to find shortest augmenting paths (shortest in number of edges)
- explicit residual updates by modifying flow[(u, v)] and flow[(v, u)]
Time complexity: O(V * E^2) worst-case.
"""

from collections import deque, defaultdict
import matplotlib.pyplot as plt
import matplotlib.lines as mlines


def bfs_augmenting_path(capacity, flow, source, sink, adj):
    """Finding the shortest augmenting path using BFS."""
    parent = {source: None}
    queue = deque([source])

    while queue:
        u = queue.popleft()
        for v in adj[u]:
            if v not in parent and capacity[(u, v)] - flow[(u, v)] > 0:
                parent[v] = u
                if v == sink:
                    return parent  # Found path
                queue.append(v)
    return None  # No path found


def edmonds_karp_concept(capacity, adj, source, sink):
    """the max flow computation using Edmonds-Karp."""
    flow = defaultdict(int)
    max_flow = 0

    while True:
        parent = bfs_augmenting_path(capacity, flow, source, sink, adj)
        if not parent:
            break

        # Finding the minimum residual capacity along the path(bottleneck)
        v = sink
        bottleneck = float("inf")
        while v != source:
            u = parent[v]
            bottleneck = min(bottleneck, capacity[(u, v)] - flow[(u, v)])
            v = u

        # Augmenting flow along the path
        v = sink
        while v != source:
            u = parent[v]
            flow[(u, v)] += bottleneck
            flow[(v, u)] -= bottleneck
            v = u

        max_flow += bottleneck

    return max_flow, flow


def flight_network(a, b, r):
    """
    Building the flight network connection.
    a[i], b[i] = start and end times of flight i
    r[i][j] = reposition time between flights i and j
    """
    p = len(a)
    source, sink = "s", "t"
    adj = defaultdict(list)
    capacity = defaultdict(int)

    left = [f"L{i+1}" for i in range(p)]
    right = [f"R{i+1}" for i in range(p)]

    # connecting source to left and right to sink with capacity 1
    for L in left:
        adj[source].append(L)
        adj[L].append(source)
        capacity[(source, L)] = 1
        capacity[(L, source)] = 0

    for R in right:
        adj[R].append(sink)
        adj[sink].append(R)
        capacity[(R, sink)] = 1
        capacity[(sink, R)] = 0

    # feasible transitions(flight i to possible flight j)
    for i in range(p):
        for j in range(p):
            if i != j and b[i] + r[i][j] <= a[j]:
                Li, Rj = f"L{i+1}", f"R{j+1}"
                adj[Li].append(Rj)
                adj[Rj].append(Li)
                capacity[(Li, Rj)] = 1
                capacity[(Rj, Li)] = 0

    return source, sink, capacity, adj, left, right


def minimum_number_of_planes(a, b, r):
    """Finding the minimum number of planes required."""
    source, sink, capacity, adj, left, right = flight_network(a, b, r)
    max_flow, flow = edmonds_karp_concept(capacity, adj, source, sink)
    min_planes = len(a) - max_flow

    return min_planes, max_flow, flow, capacity, left, right, source, sink


# Plane Schedules
def planes_Schedules(flow, left, right):
    """flight Schedulling to each planes."""
    planes = []
    next_flight = {}

    # Adjacent mapping from Li to Rj for used transitions
    for L in left:
        for R in right:
            if flow.get((L, R), 0) > 0:
                i = int(L[1:])
                j = int(R[1:])
                next_flight[i] = j

    # Find starting flights (those not reached from any R)
    reachable_from = {j for j in next_flight.values()}
    start_nodes = [i for i in range(1, len(left) + 1) if i not in reachable_from]

    # Plane route
    for start in sorted(start_nodes):
        chain = [start]
        cur = start
        while cur in next_flight:
            nxt = next_flight[cur]
            chain.append(nxt)
            cur = nxt
        planes.append(chain)

    return planes


def get_flow_data(a, b, r):
    """Return the data flow for automated checks."""
    min_planes, max_flow, flow, capacity, left, right, source, sink = (
        minimum_number_of_planes(a, b, r)
    )
    # convert defaultdicts to plain dicts (ints) for easy assertions / JSON
    flow_dict = {k: int(v) for k, v in flow.items()}
    capacity_dict = {k: int(v) for k, v in capacity.items()}
    planes = planes_Schedules(flow, left, right)

    return {
        "min_planes": int(min_planes),
        "max_flow": int(max_flow),
        "flow": flow_dict,
        "capacity": capacity_dict,
        "left": list(left),
        "right": list(right),
        "source": source,
        "sink": sink,
        "planes": planes,
    }


def flow_plotting(a, b, r):
    """Plotting the flow network."""
    min_planes, max_flow, flow, capacity, left, right, source, sink = (
        minimum_number_of_planes(a, b, r)
    )
    p = len(a)

    # Positions definition for the visualization
    pos = {source: (-1, 0), sink: (3, 0)}
    for i, L in enumerate(left):
        pos[L] = (0, p - i)
    for i, R in enumerate(right):
        pos[R] = (2, p - i)

    # Edge listing and colors
    edges = []
    edge_colors = []
    edge_labels = {}

    for u, v in capacity:
        if capacity[(u, v)] == 0:
            continue
        edges.append((u, v))
        f = flow.get((u, v), 0)
        c = capacity[(u, v)]
        edge_labels[(u, v)] = f"({f},{c})"
        if u == source or v == sink:
            color = "black"
        elif u.startswith("L") and v.startswith("R"):
            color = "red" if f > 0 else "green"
        else:
            color = "gray"
        edge_colors.append(color)

    # Graph plotting
    fig, ax = plt.subplots(figsize=(10, 6))
    for (u, v), color in zip(edges, edge_colors):
        x1, y1 = pos[u]
        x2, y2 = pos[v]
        ax.annotate(
            "",
            xy=(x2, y2),
            xytext=(x1, y1),
            arrowprops=dict(arrowstyle="->", color=color, lw=2, shrinkA=10, shrinkB=10),
        )
        # adding labels
        if (u, v) in edge_labels:
            xm, ym = (x1 + x2) / 2, (y1 + y2) / 2
            ax.text(
                xm,
                ym + 0.15,
                edge_labels[(u, v)],
                color="blue",
                fontsize=9,
                ha="center",
            )

    for n, (x, y) in pos.items():
        # Draw nodes
        ax.scatter(x, y, s=1000, c="white", edgecolors="black", zorder=3)
        ax.text(x, y, n, fontsize=10, ha="center", va="center")

    plt.axis("off")
    plt.title(
        f"Max-Flow Network\n(Max Flow = {max_flow}, Min Planes = {min_planes})",
        fontsize=12,
    )

    # Legend
    red = mlines.Line2D([], [], color="red", marker=">", label="Used")
    green = mlines.Line2D([], [], color="green", marker=">", label="Feasible unused")
    ax.legend(handles=[red, green], loc="upper right")
    plt.show()

    # console detailled output
    print("\nFeasible transitions:")
    for L in left:
        for R in right:
            if (L, R) in capacity:
                f = flow.get((L, R), 0)
                mark = "Red Used" if f > 0 else "Green Not used"
                print(f"  {L} → {R}: {mark}")


# small test case, can also randomly generate more complex cases
a = [8.0, 11.0, 12.0]
b = [10.0, 13.0, 15.0]
r = [[0, 1, 2], [1, 0, 2], [2, 1, 0]]

if __name__ == "__main__":

    min_planes, max_flow, flow, capacity, left, right, source, sink = (
        minimum_number_of_planes(a, b, r)
    )
    planes = planes_Schedules(flow, left, right)

    print(f"Maximum Flow = {max_flow}")
    print(f"Minimum Planes = {min_planes}")
    print("\nplane schedules:")
    for i, chain in enumerate(planes, start=1):
        print(f"  Plane {i}: " + " → ".join(f"Flight {f}" for f in chain))

    from pprint import pprint

    data = get_flow_data(a, b, r)
    print("\nFlow data:")
    pprint(data)

    flow_plotting(a, b, r)
