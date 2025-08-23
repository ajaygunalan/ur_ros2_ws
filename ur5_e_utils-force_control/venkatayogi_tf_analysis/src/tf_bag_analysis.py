#!/usr/bin/env python

import rosbag
import pandas as pd
from tf_bag import BagTfTransformer
from rospy import Time
import os

bag_dir = '/home/mardava/Projects/robots/ur5_e_ws/bag/nethra/fixed'

frame1_id1 = 'lap_tool_link2'
frame1_id2 = 'wrist_3_link'
frame2_id = 'base_link'

bag_dir = '/home/mardava/Projects/robots/ur5_e_ws/bag/nethra/fixed'

frame1_id1 = 'lap_tool_link2'
frame1_id2 = 'wrist_3_link'
frame2_id = 'base_link'

for bag_file in os.listdir(bag_dir):
    if bag_file.endswith('.bag'):
        bag_file_path = os.path.join(bag_dir, bag_file)

        bag = rosbag.Bag(bag_file_path)
        bag_transformer = BagTfTransformer(bag)
        
        bag_name = bag_file
        data1 = []
        data2 = []

        for topic, msg, t in bag.read_messages(topics=['/verasonics/channel_data']):
            try:
                translation1, quaternion1 = bag_transformer.lookupTransform(frame2_id, frame1_id1, t)
                data1.append([t.to_sec(), translation1[0], translation1[1], translation1[2],
                              quaternion1[0], quaternion1[1], quaternion1[2], quaternion1[3]])
            except:
                pass

            try:
                translation2, quaternion2 = bag_transformer.lookupTransform(frame2_id, frame1_id2, t)
                data2.append([t.to_sec(), translation2[0], translation2[1], translation2[2],
                              quaternion2[0], quaternion2[1], quaternion2[2], quaternion2[3]])
            except:
                pass
        df1 = pd.DataFrame(data1, columns=['time', 'tx', 'ty', 'tz', 'qx', 'qy', 'qz', 'qw'])
        df2 = pd.DataFrame(data2, columns=['time', 'tx', 'ty', 'tz', 'qx', 'qy', 'qz', 'qw'])

        df1.to_csv(f'transformations_{bag_name}_{frame2_id}_to_{frame1_id1}.csv', index=False)
        df2.to_csv(f'transformations_{bag_name}_{frame2_id}_to_{frame1_id2}.csv', index=False)

        print(f'Transformations for {bag_name} saved to separate files')
