import rospy
import osmnx as ox
import networkx as nx
from shapely.geometry import Point
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

# define the origin and destination points as Point objects
origin_point = Point(40.74457, -73.98795) # latitude, longitude of origin
destination_point = Point(40.73060, -73.93520) # latitude, longitude of destination

# download the street network data for the area surrounding the origin and destination points
graph = ox.graph_from_point(origin_point, distance=500, network_type='drive')

# compute the shortest path between the origin and destination points based on the length of the edges
origin_node = ox.get_nearest_node(graph, (origin_point.y, origin_point.x))
destination_node = ox.get_nearest_node(graph, (destination_point.y, destination_point.x))
shortest_path = nx.shortest_path(graph, origin_node, destination_node, weight='length')

# create a new path message
path_msg = Path()

# set the header of the path message
path_msg.header.frame_id = 'map'

# iterate over the nodes in the shortest path and add them to the path message
for node in shortest_path:
    # create a new pose stamped message for the current node
    pose_stamped = PoseStamped()
    pose_stamped.header.frame_id = 'map'
    pose_stamped.pose.position.x = graph.nodes[node]['x']
    pose_stamped.pose.position.y = graph.nodes[node]['y']
    pose_stamped.pose.orientation.w = 1.0

    # add the pose stamped message to the path message
    path_msg.poses.append(pose_stamped)

# publish the path message to a ROS topic
rospy.init_node('shortest_path_publisher')
path_publisher = rospy.Publisher('/shortest_path', Path, queue_size=10)
path_publisher.publish(path_msg)