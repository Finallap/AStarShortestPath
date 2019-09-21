import heapq
import sys
from scipy.io import loadmat


class Graph:
    def __init__(self):
        self.vertices = {}

    def add_vertex(self, name, edges):
        self.vertices[name] = edges

    def shortest_path(self, start, finish):
        distances = {}  # Distance from start to node
        previous = {}  # Previous node in optimal path from source
        nodes = []  # Priority queue of all nodes in Graph

        for vertex in self.vertices:
            if vertex == start:  # Set root node as distance of 0
                distances[vertex] = 0
                heapq.heappush(nodes, [0, vertex])
            else:
                distances[vertex] = sys.maxsize
                heapq.heappush(nodes, [sys.maxsize, vertex])
            previous[vertex] = None

        while nodes:
            smallest = heapq.heappop(nodes)[1]  # Vertex in nodes with smallest distance in distances
            if smallest == finish:  # If the closest node is our target we're done so print the path
                path = []
                while previous[smallest]:  # Traverse through nodes til we reach the root which is 0
                    path.append(smallest)
                    smallest = previous[smallest]
                return path
            if distances[smallest] == sys.maxsize:  # All remaining vertices are inaccessible from source
                break

            for neighbor in self.vertices[smallest]:  # Look at all the nodes that this vertex is attached to
                alt = distances[smallest] + self.vertices[smallest][neighbor]  # Alternative path distance
                if alt < distances[neighbor]:  # If there is a new shortest path update our priority queue (relax)
                    distances[neighbor] = alt
                    previous[neighbor] = smallest
                    for n in nodes:
                        if n[1] == neighbor:
                            n[0] = alt
                            break
                    heapq.heapify(nodes)
        return distances

    def __str__(self):
        return str(self.vertices)


if __name__ == '__main__':
    g = Graph()
    mat_path = "G:/研究生课程相关/研究生数学建模/2019年中国研究生数学建模竞赛F题/pre_data.mat"
    mat_data = loadmat(mat_path)
    for i in range(612):
        query_bool = mat_data['pre_data'][:, 0] == i
        row_query = mat_data['pre_data'][query_bool]
        row_dict = {}
        for j in range(len(row_query)):
            key = int(row_query[j,1])
            value = row_query[j,8]
            # row_dict.update(key=value)
            row_dict[key] = value
        g.add_vertex(i, row_dict)
    print(g.shortest_path(0,612))

    g1 = Graph()
    g1.add_vertex('A', {'B': 7, 'C': 8})
    g1.add_vertex('B', {'A': 7, 'F': 2})
    g1.add_vertex('C', {'A': 8, 'F': 6, 'G': 4})
    g1.add_vertex('D', {'F': 8})
    g1.add_vertex('E', {'H': 1})
    g1.add_vertex('F', {'B': 2, 'C': 6, 'D': 8, 'G': 9, 'H': 3})
    g1.add_vertex('G', {'C': 4, 'F': 9})
    g1.add_vertex('H', {'E': 1, 'F': 3})
    print(g1.shortest_path('A', 'H'))