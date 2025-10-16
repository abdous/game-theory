# 1. Start with 0 flow on all edges
# 2. While there exists a path from source (s) to sink (t) with available capacity:
#       - Find the path using BFS
#       - Compute the minimum residual capacity (bottleneck)
#       - Augment flow along the path by that amount
# 3. When no path remains, the current flow is the maximum flow.

from collections import deque, defaultdict


def shortest_path(capacity, flow, source, sink, adj):
    """we trying to find the Find shortest augmenting path using BFS."""
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
