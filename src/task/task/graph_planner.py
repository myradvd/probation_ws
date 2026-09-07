#!/usr/bin/env python3
import math
import heapq


# Extracts yaw (heading) in radians from a quaternion, assuming rotation is about the z-axis
def yaw_from_quaternion(qz: float, qw: float) -> float:
    return 2.0 * math.atan2(qz, qw)


# Returns the shortest distance from a point to a 2D line segment (p1 to p2)
def point_to_segment_distance_2d(p1, p2, point):
    ux = p2[0] - p1[0]
    uy = p2[1] - p1[1]
    len_sq = ux * ux + uy * uy
    if len_sq < 1e-9:
        return math.hypot(point[0] - p1[0], point[1] - p1[1])
    t = ((point[0] - p1[0]) * ux + (point[1] - p1[1]) * uy) / len_sq
    t = max(0.0, min(1.0, t))
    closest_x = p1[0] + t * ux
    closest_y = p1[1] + t * uy
    return math.hypot(point[0] - closest_x, point[1] - closest_y)


# Generates the 6 vertices of a hexagon centered on an obstacle; these are the safe waypoints to pass next to
def hexagon_vertices(center, radius):
    vertices = []
    for i in range(6):
        angle = math.pi / 3.0 * i  # 60 degrees between each vertex
        vx = center[0] + radius * math.cos(angle)
        vy = center[1] + radius * math.sin(angle)
        vertices.append((vx, vy))
    return vertices


class GraphPlanner:
    # Stores the safety radius (hexagon circumradius) and how far past the gate the through-point sits
    def __init__(self, safety_radius: float, through_distance: float = 2.0):
        self.safety_radius = safety_radius
        self.through_distance = through_distance

    # Computes the through-point: a fixed distance PAST the gate, along its facing direction --
    # aiming here (instead of stopping in front) makes the planned path actually cross the gate
    def compute_through_point(self, gate_position, gate_yaw):
        fx = math.cos(gate_yaw)
        fy = math.sin(gate_yaw)
        gx, gy = gate_position[0], gate_position[1]
        return (gx + self.through_distance * fx, gy + self.through_distance * fy)

    # Checks whether a straight segment between two points stays outside every obstacle's safety radius
    def is_segment_clear(self, p1, p2, obstacle_positions):
        for obs in obstacle_positions:
            dist = point_to_segment_distance_2d(p1, p2, (obs[0], obs[1]))
            if dist < self.safety_radius:
                return False
        return True

    # Builds the visibility graph: nodes are start, goal, and every obstacle's hexagon vertices
    def build_graph(self, start, goal, obstacle_positions):
        nodes = [start, goal]
        for obs in obstacle_positions:
            nodes.extend(hexagon_vertices((obs[0], obs[1]), self.safety_radius))

        # An edge exists between two nodes only if the straight line between them is obstacle-free
        edges = {i: [] for i in range(len(nodes))}
        for i in range(len(nodes)):
            for j in range(i + 1, len(nodes)):
                if self.is_segment_clear(nodes[i], nodes[j], obstacle_positions):
                    dist = math.hypot(nodes[i][0] - nodes[j][0], nodes[i][1] - nodes[j][1])
                    edges[i].append((j, dist))
                    edges[j].append((i, dist))
        return nodes, edges

    # Runs A* search over the visibility graph from node 0 (start) to node 1 (goal)
    def a_star(self, nodes, edges):
        start_idx, goal_idx = 0, 1

        def heuristic(i):
            return math.hypot(nodes[i][0] - nodes[goal_idx][0], nodes[i][1] - nodes[goal_idx][1])

        open_set = [(heuristic(start_idx), start_idx)]
        came_from = {}
        g_score = {start_idx: 0.0}

        while open_set:
            _, current = heapq.heappop(open_set)
            if current == goal_idx:
                return self.reconstruct_path(came_from, current, nodes)

            for neighbor, edge_cost in edges[current]:
                tentative_g = g_score[current] + edge_cost
                if tentative_g < g_score.get(neighbor, float('inf')):
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score = tentative_g + heuristic(neighbor)
                    heapq.heappush(open_set, (f_score, neighbor))

        return None  # no path found

    # Walks the came_from chain backward to build the final ordered list of waypoint coordinates
    def reconstruct_path(self, came_from, current, nodes):
        path = [nodes[current]]
        while current in came_from:
            current = came_from[current]
            path.append(nodes[current])
        path.reverse()
        return path

    # Top-level planning call: computes the through-point, builds the graph, and returns a safe waypoint path
    def plan(self, auv_position, gate_position, gate_yaw, obstacle_positions):
        start = (auv_position[0], auv_position[1])
        through_point = self.compute_through_point(gate_position, gate_yaw)
        nodes, edges = self.build_graph(start, through_point, obstacle_positions)
        path = self.a_star(nodes, edges)
        return path