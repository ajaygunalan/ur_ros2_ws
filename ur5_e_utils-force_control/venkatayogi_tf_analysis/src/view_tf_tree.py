import os
import rosbag
import tf
import pydot  # Library to create the graph
from collections import defaultdict

def create_tf_tree_graph(bag_path):
    # Dictionary to store parent-child relationships for frames
    frame_tree = defaultdict(set)

    # Read the TF data from the bag file
    with rosbag.Bag(bag_path, 'r') as bag:
        for topic, msg, t in bag.read_messages(topics=['/tf', '/tf_static']):
            for transform in msg.transforms:
                parent_frame = transform.header.frame_id
                child_frame = transform.child_frame_id

                # Store the relationship in the dictionary
                frame_tree[parent_frame].add(child_frame)

    # Create the TF tree graph using pydot
    graph = pydot.Dot(graph_type='digraph')

    for parent, children in frame_tree.items():
        for child in children:
            edge = pydot.Edge(parent, child)
            graph.add_edge(edge)

    # Save the graph as a PDF or PNG
    graph.write_pdf("tf_tree.pdf")
    print("TF tree saved as tf_tree.pdf")

# Example usage
if __name__ == '__main__':
    bag_path = os.path.join('/home/mardava/Projects/robots/ur5_e_ws/bag/nethra_bag/fixed/2024-09-25-14-35-34.bag')
    # bag_path = '/data/mardava/Projects/robots/ur5_e_ws/bag/20241106_animal_study/fixed/sys_a_2024-11-06-11-59-57.bag'

    create_tf_tree_graph(bag_path)

