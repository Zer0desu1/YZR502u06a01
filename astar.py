import numpy as np
import heapq
import time

class AStar:

    def __init__(self, grid, allow_diagonal=True):

        self.grid = grid
        self.rows, self.cols = grid.shape
        self.allow_diagonal = allow_diagonal

        if allow_diagonal:
            self.movements = [
                (0, 1, 1.0), (1, 0, 1.0), (0, -1, 1.0), (-1, 0, 1.0),
                (1, 1, np.sqrt(2)), (1, -1, np.sqrt(2)),
                (-1, 1, np.sqrt(2)), (-1, -1, np.sqrt(2))
            ]
        else:
            self.movements = [
                (0, 1, 1.0), (1, 0, 1.0), (0, -1, 1.0), (-1, 0, 1.0)
            ]

    def heuristic(self, a, b):

        return np.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)

    def is_valid(self, x, y):

        return 0 <= x < self.rows and 0 <= y < self.cols and self.grid[x, y] == 0

    def plan(self, start, goal):

        start_time = time.time()

        if not self.is_valid(*start) or not self.is_valid(*goal):
            return None, {"planning_time": 0, "nodes_expanded": 0, "path_length": 0}
        open_set = []
        heapq.heappush(open_set, (0 + self.heuristic(start, goal), 0, start))

        came_from = {}
        g_score = {start: 0}
        closed_set = set()
        nodes_expanded = 0

        while open_set:
            f_current, g_current, current = heapq.heappop(open_set)

            if current == goal:
                path = []
                node = goal
                while node in came_from:
                    path.append(node)
                    node = came_from[node]
                path.append(start)
                path.reverse()

                planning_time = time.time() - start_time
                path_length = self._calculate_path_length(path)

                metrics = {
                    "planning_time": planning_time,
                    "nodes_expanded": nodes_expanded,
                    "path_length": path_length,
                    "path_points": len(path)
                }
                return path, metrics

            if current in closed_set:
                continue
            closed_set.add(current)
            nodes_expanded += 1

            for dx, dy, cost in self.movements:
                neighbor = (current[0] + dx, current[1] + dy)

                if not self.is_valid(*neighbor) or neighbor in closed_set:
                    continue

                tentative_g = g_score[current] + cost

                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    g_score[neighbor] = tentative_g
                    f_score = tentative_g + self.heuristic(neighbor, goal)
                    came_from[neighbor] = current
                    heapq.heappush(open_set, (f_score, tentative_g, neighbor))

        planning_time = time.time() - start_time
        return None, {"planning_time": planning_time, "nodes_expanded": nodes_expanded, "path_length": 0}

    def _calculate_path_length(self, path):

        length = 0
        for i in range(1, len(path)):
            dx = path[i][0] - path[i-1][0]
            dy = path[i][1] - path[i-1][1]
            length += np.sqrt(dx**2 + dy**2)
        return length

if __name__ == "__main__":
    from map_loader import create_sample_map

    grid, resolution, origin = create_sample_map()
    planner = AStar(grid)

    start = (10, 10)
    goal = (180, 180)

    path, metrics = planner.plan(start, goal)
    if path:
        print(f"Yol bulundu!")
        print(f"  Yol uzunlugu (piksel): {metrics['path_length']:.2f}")
        print(f"  Yol uzunlugu (metre): {metrics['path_length'] * resolution:.2f}")
        print(f"  Planlama suresi: {metrics['planning_time']*1000:.2f} ms")
        print(f"  Genislenen dugum sayisi: {metrics['nodes_expanded']}")
    else:
        print("Yol bulunamadi!")
