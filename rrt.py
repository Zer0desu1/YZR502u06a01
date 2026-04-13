import numpy as np
import time

class RRTNode:

    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None
        self.cost = 0.0

class RRT:

    def __init__(self, grid, step_size=5, max_iter=10000, goal_sample_rate=0.1):

        self.grid = grid
        self.rows, self.cols = grid.shape
        self.step_size = step_size
        self.max_iter = max_iter
        self.goal_sample_rate = goal_sample_rate

    def plan(self, start, goal, goal_threshold=5):

        start_time = time.time()

        start_node = RRTNode(start[0], start[1])
        goal_node = RRTNode(goal[0], goal[1])

        node_list = [start_node]
        nodes_expanded = 0

        for i in range(self.max_iter):
            if np.random.random() < self.goal_sample_rate:
                random_point = (goal[0], goal[1])
            else:
                random_point = (
                    np.random.randint(0, self.rows),
                    np.random.randint(0, self.cols)
                )
            nearest_node = self._get_nearest(node_list, random_point)
            nodes_expanded += 1
            new_node = self._steer(nearest_node, random_point)
            if self._collision_free(nearest_node, new_node):
                new_node.parent = nearest_node
                new_node.cost = nearest_node.cost + self._distance(nearest_node, new_node)
                node_list.append(new_node)
                dist_to_goal = self._distance(new_node, goal_node)
                if dist_to_goal <= goal_threshold:
                    if self._collision_free(new_node, goal_node):
                        goal_node.parent = new_node
                        goal_node.cost = new_node.cost + dist_to_goal
                        node_list.append(goal_node)

                        path = self._extract_path(goal_node)
                        planning_time = time.time() - start_time
                        path_length = self._calculate_path_length(path)

                        metrics = {
                            "planning_time": planning_time,
                            "nodes_expanded": nodes_expanded,
                            "path_length": path_length,
                            "path_points": len(path),
                            "tree_size": len(node_list),
                            "iterations": i + 1
                        }
                        return path, metrics

        planning_time = time.time() - start_time
        return None, {
            "planning_time": planning_time,
            "nodes_expanded": nodes_expanded,
            "path_length": 0,
            "tree_size": len(node_list),
            "iterations": self.max_iter
        }

    def _get_nearest(self, node_list, point):

        distances = [
            (n.x - point[0])**2 + (n.y - point[1])**2
            for n in node_list
        ]
        return node_list[np.argmin(distances)]

    def _steer(self, from_node, to_point):

        dx = to_point[0] - from_node.x
        dy = to_point[1] - from_node.y
        dist = np.sqrt(dx**2 + dy**2)

        if dist < self.step_size:
            new_x, new_y = to_point[0], to_point[1]
        else:
            theta = np.arctan2(dy, dx)
            new_x = from_node.x + int(self.step_size * np.cos(theta))
            new_y = from_node.y + int(self.step_size * np.sin(theta))

        new_x = max(0, min(self.rows - 1, new_x))
        new_y = max(0, min(self.cols - 1, new_y))

        return RRTNode(new_x, new_y)

    def _collision_free(self, from_node, to_node):

        x0, y0 = int(from_node.x), int(from_node.y)
        x1, y1 = int(to_node.x), int(to_node.y)

        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        steps = max(dx, dy)

        if steps == 0:
            return self.grid[x0, y0] == 0

        for i in range(steps + 1):
            t = i / steps
            x = int(x0 + t * (x1 - x0))
            y = int(y0 + t * (y1 - y0))

            if not (0 <= x < self.rows and 0 <= y < self.cols):
                return False
            if self.grid[x, y] == 1:
                return False

        return True

    def _distance(self, node1, node2):

        return np.sqrt((node1.x - node2.x)**2 + (node1.y - node2.y)**2)

    def _extract_path(self, goal_node):

        path = []
        node = goal_node
        while node is not None:
            path.append((node.x, node.y))
            node = node.parent
        path.reverse()
        return path

    def _calculate_path_length(self, path):

        length = 0
        for i in range(1, len(path)):
            dx = path[i][0] - path[i-1][0]
            dy = path[i][1] - path[i-1][1]
            length += np.sqrt(dx**2 + dy**2)
        return length

class RRTStar(RRT):

    def __init__(self, grid, step_size=5, max_iter=10000,
                 goal_sample_rate=0.1, search_radius=15):
        super().__init__(grid, step_size, max_iter, goal_sample_rate)
        self.search_radius = search_radius

    def plan(self, start, goal, goal_threshold=5):
        start_time = time.time()

        start_node = RRTNode(start[0], start[1])
        goal_node = RRTNode(goal[0], goal[1])

        node_list = [start_node]
        nodes_expanded = 0
        best_goal_node = None
        best_cost = float('inf')

        for i in range(self.max_iter):
            if np.random.random() < self.goal_sample_rate:
                random_point = (goal[0], goal[1])
            else:
                random_point = (
                    np.random.randint(0, self.rows),
                    np.random.randint(0, self.cols)
                )

            nearest_node = self._get_nearest(node_list, random_point)
            new_node = self._steer(nearest_node, random_point)
            nodes_expanded += 1

            if not self._collision_free(nearest_node, new_node):
                continue
            near_nodes = self._get_near_nodes(node_list, new_node)
            new_node.parent = nearest_node
            new_node.cost = nearest_node.cost + self._distance(nearest_node, new_node)

            for near_node in near_nodes:
                new_cost = near_node.cost + self._distance(near_node, new_node)
                if new_cost < new_node.cost and self._collision_free(near_node, new_node):
                    new_node.parent = near_node
                    new_node.cost = new_cost

            node_list.append(new_node)
            for near_node in near_nodes:
                new_cost = new_node.cost + self._distance(new_node, near_node)
                if new_cost < near_node.cost and self._collision_free(new_node, near_node):
                    near_node.parent = new_node
                    near_node.cost = new_cost
            dist_to_goal = self._distance(new_node, goal_node)
            if dist_to_goal <= goal_threshold:
                total_cost = new_node.cost + dist_to_goal
                if total_cost < best_cost and self._collision_free(new_node, goal_node):
                    best_goal = RRTNode(goal[0], goal[1])
                    best_goal.parent = new_node
                    best_goal.cost = total_cost
                    best_goal_node = best_goal
                    best_cost = total_cost

        if best_goal_node is not None:
            path = self._extract_path(best_goal_node)
            planning_time = time.time() - start_time
            path_length = self._calculate_path_length(path)

            metrics = {
                "planning_time": planning_time,
                "nodes_expanded": nodes_expanded,
                "path_length": path_length,
                "path_points": len(path),
                "tree_size": len(node_list),
                "iterations": self.max_iter
            }
            return path, metrics

        planning_time = time.time() - start_time
        return None, {
            "planning_time": planning_time,
            "nodes_expanded": nodes_expanded,
            "path_length": 0,
            "tree_size": len(node_list),
            "iterations": self.max_iter
        }

    def _get_near_nodes(self, node_list, target_node):

        near_nodes = []
        for node in node_list:
            if self._distance(node, target_node) <= self.search_radius:
                near_nodes.append(node)
        return near_nodes

if __name__ == "__main__":
    from map_loader import create_sample_map

    grid, resolution, origin = create_sample_map()

    print("=== RRT ===")
    rrt = RRT(grid, step_size=5, max_iter=10000, goal_sample_rate=0.1)
    path, metrics = rrt.plan((10, 10), (180, 180))
    if path:
        print(f"  Yol uzunlugu (piksel): {metrics['path_length']:.2f}")
        print(f"  Yol uzunlugu (metre): {metrics['path_length'] * resolution:.2f}")
        print(f"  Planlama suresi: {metrics['planning_time']*1000:.2f} ms")
        print(f"  Agac boyutu: {metrics['tree_size']}")

    print("\n=== RRT* ===")
    rrt_star = RRTStar(grid, step_size=5, max_iter=5000, goal_sample_rate=0.1)
    path, metrics = rrt_star.plan((10, 10), (180, 180))
    if path:
        print(f"  Yol uzunlugu (piksel): {metrics['path_length']:.2f}")
        print(f"  Yol uzunlugu (metre): {metrics['path_length'] * resolution:.2f}")
        print(f"  Planlama suresi: {metrics['planning_time']*1000:.2f} ms")
        print(f"  Agac boyutu: {metrics['tree_size']}")
