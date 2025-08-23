#!/usr/bin/env python
import argparse
import h5py
import numpy as np
import os
import sys

import rosbag
import rospy

from datetime import datetime
from scipy.spatial.transform import Rotation
from tqdm import tqdm

from tf_bag import BagTfTransformer

__all__ = ['RobotPoseExtractor']

class RobotPoseExtractor:
	def __init__(self, bag_file_name, conf_score_thresh):
		'''
		Instantiate an object of the type `BagFileParser`.

		Args:
			bag_file_name: The name of the ROS bag file to be processed
		'''
		self.bag_file_name = bag_file_name
		self.bag_fh = None
		self.bag_tf = None

		self.base_frame_name = 'base_link'
		self.probe_frame_name = 'p42v_link1'

		self.conf_score_thresh = conf_score_thresh
		self.src_class_id = 0

		self.generateOutputDirName()
		self.generateRobotPoseFileName()

	def closeBagFile(self):
		'''
		Close the bag file handler if it exists
		'''
		if self.bag_fh is not None:
			self.bag_fh.close()

	def extractPointSourceEstimatesBaseFrame(self):
		'''
		Extract point source estimates, transform them to the robot base frame, and write them to file.
		'''
		ft_probe_topic_name = '/netft/proc_probe'
		lkf_topic_name = '/visual_servoing/multi_track_lkf'
		pt_src_det_topic_name = '/pt_src_loc/detections'
		ur_hw_topic_name = '/ur_hardware_interface/robot_program_running'

		bag_info = self.bag_fh.get_type_and_topic_info()
		bag_start_time = self.bag_fh.get_start_time()
		num_detections = bag_info.topics[pt_src_det_topic_name].message_count
		num_ft_msgs = bag_info.topics[ft_probe_topic_name].message_count
		num_lkf_msgs = bag_info.topics[lkf_topic_name].message_count

		pt_src_time_vec = np.zeros((num_detections, 1), dtype=np.float64)

		ur_hw_if_robot_program_running_vec = np.zeros(num_detections)
		robot_program_running = False

		# TODO: Determine if these are required.
		pt_src_conf_score_list = np.zeros((num_detections, 1), dtype=np.float64)
		pt_src_num_det_vec = np.zeros((num_detections, 2), dtype=np.float64)

		pt_src_loc_probe_list = np.zeros((num_detections, 3), dtype=np.float64)
		pt_src_loc_base_list = np.zeros((num_detections, 3), dtype=np.float64)

		tf_time_list = np.zeros((num_detections, 1), dtype=np.float64)
		tf_trans_list = np.zeros((num_detections, 3), dtype=np.float64)
		tf_rot_list = np.zeros((num_detections, 4), dtype=np.float64)

		ft_time_list = np.zeros((num_ft_msgs, 1), dtype=np.float64)
		ft_probe_list = np.zeros((num_ft_msgs, 6), dtype=np.float64)

		lkf_time_vec = np.zeros((num_lkf_msgs, 1), dtype=np.float64)
		lkf_num_filters_vec = np.zeros((num_lkf_msgs, 1), dtype=np.float64)
		lkf_validity_vec = np.zeros((num_lkf_msgs, 1), dtype=np.float64)
		lkf_state_dict = {\
				'LKF_TRACKING': 0, 'LKF_MISSED_ONE': 1, 'LKF_MISSED_TWO': 2, 'LKF_INITIALIZE': 3, 'LKF_DELETE': 4}

		det_msg_id = 0
		ft_msg_id = 0
		lkf_msg_id = 0

		print('Opening file \'{:s}\' for writing...'.format(self.robot_pose_h5_file_name))

		with tqdm(total=num_detections, desc='Processing detections', unit='img') as progress_bar:
			for topic, msg, t in self.bag_fh.read_messages(\
					topics=[ft_probe_topic_name, lkf_topic_name, pt_src_det_topic_name, ur_hw_topic_name]):
				if topic == ft_probe_topic_name:
					ft_time_list[ft_msg_id] = t.to_sec()
					ft_probe_list[ft_msg_id, 0] = msg.wrench.force.x
					ft_probe_list[ft_msg_id, 1] = msg.wrench.force.y
					ft_probe_list[ft_msg_id, 2] = msg.wrench.force.z
					ft_probe_list[ft_msg_id, 3] = msg.wrench.torque.x
					ft_probe_list[ft_msg_id, 4] = msg.wrench.torque.y
					ft_probe_list[ft_msg_id, 5] = msg.wrench.torque.z
					ft_msg_id += 1
				elif topic == lkf_topic_name:
					lkf_time_vec[lkf_msg_id] = t.to_sec()
					lkf_num_filters_vec[lkf_msg_id] = len(msg.filters)
					lkf_validity_vec[lkf_msg_id] = len(lkf_state_dict)

					if len(msg.filters) > 0:
						for i0 in range(len(msg.filters)):
							if lkf_state_dict[msg.filters[i0].fsm_state] < lkf_validity_vec[lkf_msg_id]:
								lkf_validity_vec[lkf_msg_id] = lkf_state_dict[msg.filters[i0].fsm_state]

					lkf_msg_id += 1
				elif topic == pt_src_det_topic_name:
					# Get the time of the current message.
					t_sec = t.to_sec()

					# TODO: Get the current transform from the probe to the robot base.
					probe_pose_base_mat = np.zeros((4, 4), dtype=np.float64)
					probe_pose_base_mat[:3, 3], rot_q = self.bag_tf.lookupTransform(\
							self.base_frame_name, self.probe_frame_name, t)
					probe_pose_base_mat[:3, :3] = Rotation.from_quat(rot_q).as_matrix()
					probe_pose_base_mat[3, 3] = 1.0

					pt_src_pos_probe_mat = []
					pt_src_conf_score_vec = []
					pt_src_bbox_mat = []

					# TODO: Get each source detection exceeding the confidence score and extract the point source
					# location to a list.
					if len(msg.segmentations) > 0:
						for i0 in range(len(msg.segmentations)):
							if(\
									(len(msg.segmentations[i0].results) > 0) \
									and (msg.segmentations[i0].results[0].id == self.src_class_id) \
									and (msg.segmentations[i0].results[0].score >= self.conf_score_thresh)):
								pt_src_pos_probe_mat.append([\
										msg.segmentations[i0].results[0].pose.pose.position.x, \
										msg.segmentations[i0].results[0].pose.pose.position.y, \
										msg.segmentations[i0].results[0].pose.pose.position.z])
								pt_src_conf_score_vec.append(msg.segmentations[i0].results[0].score)
								pt_src_bbox_mat.append([\
										msg.segmentations[i0].bbox.center.x, \
										msg.segmentations[i0].bbox.center.y, \
										msg.segmentations[i0].bbox.size_x, \
										msg.segmentations[i0].bbox.size_y])
								pt_src_num_det_vec[det_msg_id] += 1

								output_h5_file_name = os.path.join(self.output_dir_name, '{:06d}.h5'.format(det_msg_id))

								with h5py.File(output_h5_file_name, 'w') as h5_fh:
									h5_fh.create_dataset('/tf/p42v_link1', data=probe_pose_base_mat)
									h5_fh.create_dataset('/pt_src/p42v_link1', data=np.array(pt_src_pos_probe_mat))
									h5_fh.create_dataset('/pt_src/conf_score', data=np.array(pt_src_conf_score_vec))
									h5_fh.create_dataset('/pt_src/bbox', data=np.array(pt_src_bbox_mat))
									h5_fh.create_dataset('/pt_src/time', data=np.array([t.to_sec() - bag_start_time]))
									h5_fh.create_dataset(ur_hw_topic_name, data=np.array([robot_program_running]))

					# TODO: Write the data to an HDF5 file corresponding to the current message (use the index).
					pt_src_time_vec[det_msg_id] = t_sec
					ur_hw_if_robot_program_running_vec[det_msg_id] = robot_program_running

					tf_time_list[det_msg_id] = t_sec
					tf_trans_list[det_msg_id, :] = probe_pose_base_mat[:3, 3]
					tf_rot_list[det_msg_id, :] = rot_q

					det_msg_id += 1
					progress_bar.update(1)
				elif topic == ur_hw_topic_name:
					robot_program_running = msg.data
				else:
					raise ValueError('Unknown ROS topic \'{:s}\'.'.format(topic))

		print('Processed {:d} FT readings'.format(ft_msg_id))
		print('Writing file \'{:s}\'...'.format(self.robot_pose_h5_file_name))

		with h5py.File(self.robot_pose_h5_file_name, 'w') as robot_pose_fh:
			# pt_src_conf_score_dataset_name = '/pt_src/conf_score'
			# robot_pose_fh.create_dataset(pt_src_conf_score_dataset_name, data=pt_src_conf_score_list[:det_msg_id])

			# probe_dataset_name = '/pt_src/p42v_link1'
			# base_dataset_name = '/pt_src/base_link'
			# robot_pose_fh.create_dataset(probe_dataset_name, data=pt_src_loc_probe_list[:det_msg_id, :])
			# robot_pose_fh.create_dataset(base_dataset_name, data=pt_src_loc_base_list[:det_msg_id, :])

			robot_pose_fh.create_dataset('/multi_track_lkf/time', data=lkf_time_vec)
			robot_pose_fh.create_dataset('/multi_track_lkf/num_filters', data=lkf_num_filters_vec)
			robot_pose_fh.create_dataset('/multi_track_lkf/validity', data=lkf_validity_vec)

			robot_pose_fh.create_dataset('/netft/time', data=ft_time_list[:ft_msg_id])
			robot_pose_fh.create_dataset('/netft/proc_probe', data=ft_probe_list[:ft_msg_id, :])

			robot_pose_fh.create_dataset('/pt_src/time', data=pt_src_time_vec[:det_msg_id])
			robot_pose_fh.create_dataset('/pt_src/num_detections', data=pt_src_num_det_vec[:det_msg_id, :])
			robot_pose_fh.create_dataset(\
					'/pt_src/robot_program_running', data=ur_hw_if_robot_program_running_vec)

			robot_pose_fh.create_dataset('/tf/p42v_link1/time', data=tf_time_list[:det_msg_id])
			robot_pose_fh.create_dataset('/tf/p42v_link1/trans', data=tf_trans_list[:det_msg_id, :])
			robot_pose_fh.create_dataset('/tf/p42v_link1/rot', data=tf_rot_list[:det_msg_id, :])

		print('File \'{:s}\' successfully written.'.format(self.robot_pose_h5_file_name))

	def generateOutputDirName(self):
		'''
		Generate the name of the directory to which the output HDF5 file is to be written.
		'''
		self.output_dir_name = os.path.join(\
				self.bag_file_name.replace('.bag', ''), \
				'detections_{:.3f}'.format(self.conf_score_thresh).replace('.', 'p'))

		if not os.path.isdir(self.output_dir_name):
			print('Creating directory \'{:s}\'...'.format(self.output_dir_name))
			os.makedirs(self.output_dir_name)

	def generateRobotPoseFileName(self):
		'''
		Generate the name of the HDF5 file to which the data is to be written
		'''
		self.robot_pose_h5_file_name = os.path.join(self.output_dir_name, 'robot_poses.h5')

	def openBagFile(self):
		'''
		Open the bag file if it has not already been opened.
		'''
		if self.bag_fh is None:
			print('Opening file \'{:s}\' (read-only)...'.format(self.bag_file_name))
			self.bag_fh = rosbag.Bag(self.bag_file_name, 'r')

			print('Instantiating TF-Bag object...')
			self.bag_tf = BagTfTransformer(self.bag_fh)


def main():
	parser = argparse.ArgumentParser(description='Extract required information from ROS1 bag files')
	parser.add_argument('-i', '--input-file-name', type=str, required=True, help='Name of bag file to be parsed')
	parser.add_argument(\
			'-c', '--conf-score-thresh', type=float, default=0.0, help='Confidence score threshold for source class')
	args = parser.parse_args()
	robot_pose_extractor = RobotPoseExtractor(args.input_file_name, args.conf_score_thresh)
	robot_pose_extractor.openBagFile()
	robot_pose_extractor.extractPointSourceEstimatesBaseFrame()
	robot_pose_extractor.closeBagFile()

if __name__ == '__main__':
	main()
