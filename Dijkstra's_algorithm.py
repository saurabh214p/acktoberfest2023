import sys

# Create a function to find the node with the minimum distance value
def min_distance(distances, visited):
    min_dist = sys.maxsize
    min_node = None

    for node in range(len(distances)):
        if distances[node] < min_dist and not visited[node]:
            min_dist = distances[node]
            min_node = node

    return min_node

# Implement Dijkstra's algorithm
def dijkstra(graph, start):
    num_nodes = len(graph)
    distances = [sys.maxsize] * num_nodes
    visited = [False] * num_nodes

    distances[start] = 0

    for _ in range(num_nodes):
        u = min_distance(distances, visited)
        visited[u] = True

        for v in range(num_nodes):
            if not visited[v] and graph[u][v] > 0:
                if distances[u] + graph[u][v] < distances[v]:
                    distances[v] = distances[u] + graph[u][v]

    return distances

# Example usage
if __name__ == "__main__":
    # Example graph represented as an adjacency matrix
    graph = [
        [0, 4, 2, 0, 0, 0],
        [0, 0, 5, 10, 0, 0],
        [0, 0, 0, 3, 0, 0],
        [0, 0, 0, 0, 7, 0],
        [0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 2, 0]
    ]

    start_node = 0
    distances = dijkstra(graph, start_node)

    for i, distance in enumerate(distances):
        print(f"Shortest distance from node {start_node} to node {i} is {distance}")
