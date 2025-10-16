# 1. Start with 0 flow on all edges
# 2. While there exists a path from source (s) to sink (t) with available capacity:
#       - Find the path using BFS
#       - Compute the minimum residual capacity (bottleneck)
#       - Augment flow along the path by that amount
# 3. When no path remains, the current flow is the maximum flow.

from collections import deque, defaultdict


def shortest_path(capacity, flow, source, sink, adj):
    """Finding the shortest augmenting path using BFS."""
    parent = {source: None}
    queue = deque([source])

    while queue:
        u = queue.popleft()
        for v in adj[u]:
            if v not in parent and capacity[(u, v)] - flow[(u, v)] > 0:
                parent[v] = u
                if v == sink:
                    return parent
                queue.append(v)
    return None


def edmonds_karp_concept(capacity, adj, source, sink):
    """Compute max flow using Edmonds-Karp."""
    flow = defaultdict(int)
    max_flow = 0

    while True:
        parent = shortest_path(capacity, flow, source, sink, adj)
        if not parent:
            break  # if no more augmenting path

        # Finding the minimum residual capacity along the path
        v = sink
        bottleneck = float("inf")
        while v != source:
            u = parent[v]
            bottleneck = min(bottleneck, capacity[(u, v)] - flow[(u, v)])
            v = u

        # Augment the flow along the path
        v = sink
        while v != source:
            u = parent[v]
            flow[(u, v)] += bottleneck
            flow[(v, u)] -= bottleneck
            v = u

        max_flow += bottleneck

    return max_flow, flow


def flight_network(a, b, r):
    p = len(a)
    source, sink = "s", "t"
    adj = defaultdict(list)
    capacity = defaultdict(int)

    # Left (L1..Lp) and Right (R1..Rp)
    left = [f"L{i+1}" for i in range(p)]
    right = [f"R{i+1}" for i in range(p)]

    # source → left, right → sink
    for L in left:
        adj[source].append(L)
        adj[L].append(source)
        capacity[(source, L)] = 1

    for R in right:
        adj[R].append(sink)
        adj[sink].append(R)
        capacity[(R, sink)] = 1

    # feasible transitions
    for i in range(p):
        for j in range(p):
            if i != j and b[i] + r[i][j] <= a[j]:
                Li, Rj = f"L{i+1}", f"R{j+1}"
                adj[Li].append(Rj)
                adj[Rj].append(Li)
                capacity[(Li, Rj)] = 1

    return source, sink, capacity, adj, left, right


# running the algorithm
def minimum_number_of_planes(a, b, r):
    source, sink, capacity, adj, left, right = flight_network(a, b, r)
    max_flow, flow = edmonds_karp_concept(capacity, adj, source, sink)
    min_planes = len(a) - max_flow
    return min_planes, max_flow, flow
