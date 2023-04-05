import rospy
import math
import heapq
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped


class ShortestPathPlanner:
    def __init__(self):
        self.start = None
        self.goal = None
        self.path_pub = rospy.Publisher('/path', Path, queue_size=10)

    def set_start(self, x, y):
        self.start = (x, y)

    def set_goal(self, x, y):
        self.goal = (x, y)

    def calc_heuristic(self, node):
        return math.sqrt((self.goal[0] - node[0])**2 + (self.goal[1] - node[1])**2)

    def calc_path(self):
        if self.start is None or self.goal is None:
            rospy.logwarn('Start or goal coordinates not set.')
            return

        frontier = [(0, self.start)]
        came_from = {self.start: None}
        cost_so_far = {self.start: 0}

        while frontier:
            current = heapq.heappop(frontier)[1]

            if current == self.goal:
                break

            for next_node in self.get_neighbors(current):
                new_cost = cost_so_far[current] + self.calc_cost(current, next_node)
                if next_node not in cost_so_far or new_cost < cost_so_far[next_node]:
                    cost_so_far[next_node] = new_cost
                    priority = new_cost + self.calc_heuristic(next_node)
                    heapq.heappush(frontier, (priority, next_node))
                    came_from[next_node] = current

        path = []
        current = self.goal
        while current != self.start:
            path.append(current)
            current = came_from[current]
        path.append(self.start)
        path.reverse()

        path_msg = Path()
        path_msg.header.stamp = rospy.Time.now()
        path_msg.header.frame_id = 'map'

        for node in path:
            pose = PoseStamped()
            pose.header.stamp = rospy.Time.now()
            pose.header.frame_id = 'map'
            pose.pose.position.x = node[0]
            pose.pose.position.y = node[1]
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)

        self.path_pub.publish(path_msg)

    def get_neighbors(self, node):
        x, y = node
        neighbors = []
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                if dx == 0 and dy == 0:
                    continue
                neighbor = (x + dx, y + dy)
                if self.is_valid(neighbor):
                    neighbors.append(neighbor)
        return neighbors

    def is_valid(self, node):
        # Implement your own validity check here
        return True

    def calc_cost(self, node1, node2):
        # Implement your own cost calculation here
        return math.sqrt((node1[0] - node2[0])**2 + (node1[1] - node2[1])**2)

if __name__ == '__main__':
    rospy.init_node('shortest_path_planner')
    planner = ShortestPathPlanner()
    planner.set_start(0, 0)
    planner.set_goal(5, 5)
    planner.calc_path()
    rospy.spin()