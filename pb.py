"""Edmonds–Karp (manual) implementation for the Minimum Planes problem.

This module implements the Edmonds–Karp max-flow algorithm from scratch:
- BFS to find shortest augmenting paths (shortest in number of edges)
- explicit residual updates by modifying flow[(u, v)] and flow[(v, u)]
Time complexity: O(V * E^2) worst-case.
"""

from collections import deque, defaultdict
import matplotlib.pyplot as plt
import matplotlib.lines as mlines
from datetime import datetime, timedelta


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


#  Alllows for user input 

class Flight:
    """Represents a flight leg with origin, destination, and times."""
    def __init__(self, flight_id, origin, destination, departure_time, arrival_time):
        self.id = flight_id
        self.origin = origin
        self.destination = destination
        self.departure_time = departure_time  # Absolute time (can be > 24)
        self.arrival_time = arrival_time  # Absolute time (can be > 24)
        self.duration = arrival_time - departure_time
    
    def get_display_time(self, time_value):
        """Convert absolute time to day and time format for display."""
        day = int(time_value // 24)
        hour = time_value % 24
        if day == 0:
            return f"{hour:.2f}"
        else:
            return f"Day {day+1} {hour:.2f}"
    
    def __repr__(self):
        dep_str = self.get_display_time(self.departure_time)
        arr_str = self.get_display_time(self.arrival_time)
        return f"Flight {self.id}: {self.origin} → {self.destination} ({dep_str} - {arr_str})"


def get_flight_times():
    """Store flight times between city pairs based on user input."""
    flight_times = {}
    return flight_times


def calculate_reposition_time(flight_i, flight_j, flight_times):
    """
    Calculate the repositioning time from the destination of flight_i 
    to the origin of flight_j.
    """
    # Check if we need to reposition (different cities)
    if flight_i.destination == flight_j.origin:
        # Same city, minimal reposition time (e.g., 0 or small buffer)
        return 0
    
    # Need to fly from flight_i.destination to flight_j.origin
    city_pair = (flight_i.destination, flight_j.origin)
    reverse_pair = (flight_j.origin, flight_i.destination)
    
    # Check if we have a direct flight time for this route
    if city_pair in flight_times:
        return flight_times[city_pair]
    elif reverse_pair in flight_times:
        # Use the reverse flight time (assuming symmetric travel time)
        return flight_times[reverse_pair]
    else:
        # If no direct flight time exists, return infinity (not feasible)
        return float('inf')


def input_flights_interactive():
    """Interactive input method for entering flight data."""
    flights = []
    flight_times = {}
    current_day = 0  # Track current day for absolute time calculation
    
    print("\n=== Flight Scheduling System ===")
    print("Enter flight legs to schedule. The system will determine the minimum number of planes needed.")
    print("\nFirst, let's define the flight routes and their durations.")
    print("\nNOTE: Times are in 24-hour format (0-24). Flights can span multiple days.")
    print("      The system will track days automatically.")
    
    # Get unique routes
    routes_defined = set()
    
    while True:
        print("\n--- Enter Flight Leg ---")
        try:
            flight_id = len(flights) + 1
            origin = input(f"Flight {flight_id} - Origin city: ").strip().upper()
            if not origin:
                break
                
            destination = input(f"Flight {flight_id} - Destination city: ").strip().upper()
            
            # Store flight time if this route hasn't been defined yet
            route = (origin, destination)
            reverse_route = (destination, origin)
            
            if route not in flight_times and reverse_route not in flight_times:
                while True:
                    try:
                        flight_duration = float(input(f"Flight duration from {origin} to {destination} (in hours): "))
                        if flight_duration <= 0:
                            print("Duration must be positive!")
                            continue
                        break
                    except ValueError:
                        print("Please enter a valid number!")
                
                flight_times[route] = flight_duration
                flight_times[reverse_route] = flight_duration  # Symmetric assumption
            
            # Get departure day and time
            while True:
                try:
                    day_input = input(f"Departure day (1, 2, 3, etc., or press Enter for day 1): ").strip()
                    if day_input == "":
                        dep_day = 0
                    else:
                        dep_day = int(day_input) - 1
                        if dep_day < 0:
                            print("Day must be 1 or greater!")
                            continue
                    
                    dep_hour = float(input(f"Departure time on day {dep_day + 1} (0-24, e.g., 8.30 for 8:30 AM): "))
                    if dep_hour < 0 or dep_hour >= 24:
                        print("Time must be between 0 and 24!")
                        continue
                    
                    departure = dep_day * 24 + dep_hour  # Convert to absolute time
                    break
                except ValueError:
                    print("Please enter valid numbers!")
            
            # Calculate arrival time
            if route in flight_times:
                duration = flight_times[route]
            else:
                duration = flight_times[reverse_route]
            
            arrival = departure + duration
            
            flight = Flight(flight_id, origin, destination, departure, arrival)
            flights.append(flight)
            
            print(f"Added: {flight}")
            
            another = input("\nAdd another flight? (y/n): ").strip().lower()
            if another != 'y':
                break
                
        except KeyboardInterrupt:
            print("\nInput cancelled.")
            break
    
    return flights, flight_times




def convert_flights_to_parameters(flights, flight_times):
    """Convert Flight objects to a, b, r parameters for the algorithm."""
    n = len(flights)
    a = [f.departure_time for f in flights]
    b = [f.arrival_time for f in flights]
    
    # Build repositioning time matrix
    r = [[0] * n for _ in range(n)]
    
    for i in range(n):
        for j in range(n):
            if i != j:
                r[i][j] = calculate_reposition_time(flights[i], flights[j], flight_times)
    
    return a, b, r


def display_results_with_flight_info(flights, planes):
    """Display results with actual flight information."""
    print("\n=== SCHEDULING RESULTS ===")
    print(f"Minimum number of planes required: {len(planes)}")
    print("\nDetailed plane schedules:")
    
    for i, chain in enumerate(planes, start=1):
        print(f"\n  Plane {i}:")
        for j, flight_idx in enumerate(chain):
            flight = flights[flight_idx - 1]  # Convert to 0-based index
            dep_str = flight.get_display_time(flight.departure_time)
            arr_str = flight.get_display_time(flight.arrival_time)
            print(f"    Leg {j+1}: Flight {flight.id} - {flight.origin} → {flight.destination} "
                  f"(Depart: {dep_str}, Arrive: {arr_str})")


def main_with_input():
    """Main function with user input capability."""
    print("Choose input method:")
    print("1. Interactive (step-by-step)")
    print("2. Use example data")
    
    choice = input("\nEnter choice (1/2): ").strip()
    
    if choice == '1':
        flights, flight_times = input_flights_interactive()

    elif choice == '2':
        # Example data with multi-day flights
        flights = [
            Flight(1, "London", "Paris", 8.0, 13.0),   # Day 1: 08:00 - 13:00
            Flight(2, "Paris", "London", 23.0, 28.0),  # Day 1: 23:00 - Day 2: 04:00
            Flight(3, "London", "Sofia", 30.0, 33.0),  # Day 2: 06:00 - 09:00
            Flight(4, "Sofia", "London", 36.0, 39.0)   # Day 2: 12:00 - 15:00
        ]
        flight_times = {
            ("London", "Paris"): 5.0,
            ("Paris", "London"): 5.0,
            ("London", "Sofia"): 3.0,
            ("Sofia", "London"): 3.0,
            ("Paris", "Sofia"): 4.0,
            ("Sofia", "Paris"): 4.0
        }
        print("\nUsing example flights (including overnight flights):")
        for flight in flights:
            print(f"  {flight}")
    else:
        print("Invalid choice!")
        return
    
    if not flights:
        print("No flights entered!")
        return
    
    # Convert to algorithm parameters
    a, b, r = convert_flights_to_parameters(flights, flight_times)
    
    print("\n--- Running optimization algorithm ---")
    
    # Run the algorithm
    min_planes, max_flow, flow, capacity, left, right, source, sink = (
        minimum_number_of_planes(a, b, r)
    )
    planes = planes_Schedules(flow, left, right)
    
    # Display results
    print(f"\nMaximum Flow = {max_flow}")
    print(f"Minimum Planes = {min_planes}")
    
    # Display with flight information
    display_results_with_flight_info(flights, planes)
    
    # Ask if user wants to see the visualization
    show_plot = input("\nShow flow network visualization? (y/n): ").strip().lower()
    if show_plot == 'y':
        flow_plotting(a, b, r)
    
    return flights, flight_times, min_planes, planes


# small test case, can also randomly generate more complex cases
a = [8.0, 11.0, 12.0]
b = [10.0, 13.0, 15.0]
r = [[0, 1, 2], [1, 0, 2], [2, 1, 0]]

if __name__ == "__main__":
    # Run the new interactive version
    main_with_input()
    
    # Uncomment below to run the original test case
    # min_planes, max_flow, flow, capacity, left, right, source, sink = (
    #     minimum_number_of_planes(a, b, r)
    # )
    # planes = planes_Schedules(flow, left, right)
    # 
    # print(f"Maximum Flow = {max_flow}")
    # print(f"Minimum Planes = {min_planes}")
    # print("\nplane schedules:")
    # for i, chain in enumerate(planes, start=1):
    #     print(f"  Plane {i}: " + " → ".join(f"Flight {f}" for f in chain))
    # 
    # from pprint import pprint
    # data = get_flow_data(a, b, r)
    # print("\nFlow data:")
    # pprint(data)
    # 
    # flow_plotting(a, b, r)
