#!/usr/bin/env python
import argparse
import copy
import cv2 as cv
import h5py
import numpy as np
import os
import pycocotools.mask as pm
import rosbag
import rospy
import sys
import time
import torch
import yaml

import detectron2.utils.comm as comm

from datetime import datetime
from scipy.spatial.transform import Rotation
from tqdm import tqdm

from tf_bag import BagTfTransformer

from collections import OrderedDict
from cv_bridge import CvBridge, CvBridgeError
from datetime import datetime
from fvcore.common.config import CfgNode as CN
from torch.nn.parallel import DistributedDataParallel
from tqdm import tqdm

from detectron2.checkpoint import DetectionCheckpointer
from detectron2.config import get_cfg
from detectron2.data.datasets.coco import register_coco_instances
from detectron2.data.datasets import load_coco_json
from detectron2.data import MetadataCatalog
from detectron2.engine import default_argument_parser, default_setup
from detectron2.engine import DefaultPredictor
from detectron2.modeling import build_model
from detectron2_ros.base import Detectron2RosBase

from geometry_msgs.msg import PoseStamped, PoseWithCovariance, Pose2D
from pulse_vs_msgs.msg import \
		ObjectHypothesisWithPoseSosRad, Segmentation2DArray, Segmentation2D, Segmentation2DSosRad, \
		Segmentation2DSosRadArray
from sensor_msgs.msg import Image
from std_msgs.msg import String, Float64, Float64MultiArray, MultiArrayDimension
from vision_msgs.msg import Detection2DArray, Detection2D, BoundingBox2D, ObjectHypothesisWithPose

# Shape estimation imports
sys.path.append(os.path.join(\
		os.path.dirname(__file__), os.pardir, os.pardir, os.pardir, os.pardir, os.pardir, os.pardir, os.pardir, \
		'deep_learning_projects'))
import point_source_localization.theory_3d.utils.grad_desc_config as psl_gn_cfg
import point_source_localization.theory_3d.data as psl_data
import point_source_localization.theory_3d.models as psl_models
import point_source_localization.theory_3d.losses as psl_losses
import point_source_localization.theory_3d.least_squares as psl_least_sq
import point_source_localization.theory_3d.eval_metrics as psl_eval

from point_source_localization.utils import checkpoint_utils as cu

__all__ = ['RobotPoseExtractorSystemC']

class RobotPoseExtractorSystemC:
	def __init__(self, grad_desc_cfg_file_name, bag_file_name, trial_id, conf_score_thresh):
		self.bag_file_name = bag_file_name
		self.bag_fh = None
		self.bag_tf = None

		self.base_frame_name = 'base_link'
		self.probe_frame_name = 'p42v_link1'

		self.conf_score_thresh = conf_score_thresh

		self.generateOutputDirName(trial_id)
		self.generateRobotPoseFileName()

		# Fetch the default Detectron2 configuration structure which is modified later for our specific model.
		detectron2_cfg = get_cfg()
		detectron2_rel_path = os.path.join(\
				os.path.dirname(__file__), '..', '..', '..', '..', '..', '..', '..', 'deep_learning_projects', \
				'detectron2')
		detectron2_cfg_rel_path = os.path.join(\
				'configs', 'p42v_12cm_norm', 'mask_rcnn_fpn', '500um', 'chsnr_n0', 'theory_seg_masks_1r', '080000', \
				'sim_test', 'test_0079999.yaml')
		dataset_rel_path = os.path.join(\
				'datasets', 'paimdb', 'images', 'p42v_12cm_norm', 'p42v_12cm_1s1r_500um_20241017_bp_256x926_n0_nm')
		coco_json_rel_path = os.path.join(\
				'Annotations', 'theory_seg_masks_1r', 'Annotations', 'CocoTheory', 'test.json')
		detectron2_cfg_file_name = os.path.join(detectron2_rel_path, detectron2_cfg_rel_path)
		detectron2_cfg.merge_from_file(detectron2_cfg_file_name)
		detectron2_cfg.MODEL.ROI_HEADS.SCORE_THRESH_TEST = 0.5 # set threshold for this model

		# Fetch and load the details of the COCO-formatted dataset.
		coco_dataset_name = detectron2_cfg.DATASETS.TEST[0]
		dataset_dir_name = os.path.join(detectron2_rel_path, dataset_rel_path)
		coco_json_file_name = os.path.join(dataset_dir_name, coco_json_rel_path)
		coco_img_dir_name = os.path.join(dataset_dir_name, 'Images')

		try:
			register_coco_instances(coco_dataset_name, {}, coco_json_file_name, coco_img_dir_name)
		except Exception as e:
			print(e)

		load_coco_json(coco_json_file_name, coco_img_dir_name, dataset_name=coco_dataset_name)
		self._class_names = MetadataCatalog.get(coco_dataset_name).get("thing_classes", None)
		self.src_class_id = -1

		for i0 in range(len(self._class_names)):
			if self._class_names[i0] in ['src', 'source']:
				self.src_class_id = i0
				break

		# Build the model and load the desired checkpoint file containing the model weights.
		detectron2_cfg.MODEL.WEIGHTS = os.path.join(detectron2_rel_path, 'tools', detectron2_cfg.MODEL.WEIGHTS)
		print(\
				'Building default detectron2_predictor with fine-tuned weights from\n\'{}\'...'.format(\
				detectron2_cfg.MODEL.WEIGHTS))
		self.detectron2_predictor = DefaultPredictor(detectron2_cfg)

		# Initialize the variables required to convert the detected instances to point source location estimates.
		self.img_width_px = 256
		self.img_height_px = 926
		self.img_width = 0.0192
		self.img_height = 0.12
		self.zero_pad_width_px = 0
		self.lat_upsamp = 4
		self.trans_num_elem = 64
		self.img_px_to_m = np.array([self.img_width / self.img_width_px, self.img_height / self.img_height_px])
		self.pt_src_offset_m = np.array([self.img_width / 2.0, 0.0])

		print(\
				'Expecting images of:\n\tdimensions {:s},{:s},\n\tresolution {:s},\n\tand offset {:s}...'.format(\
				'({:d}, {:d}) px'.format(self.img_width_px, self.img_height_px), \
				'({:.2f}, {:.2f}) mm'.format(1e3 * self.img_width, 1e3 * self.img_height), \
				'({:.2f}, {:.2f}) um'.format(1e6 * self.img_px_to_m[0], 1e6 * self.img_px_to_m[1]), \
				'({:.2f}, {:.2f}) mm'.format(1e3 * self.pt_src_offset_m[0], 1e3 * self.pt_src_offset_m[1])))

		# Initialize zero-padding arrays.
		self.left_zero_pad = np.zeros((self.img_height_px, self.zero_pad_width_px), dtype=np.uint8)
		self.right_zero_pad = np.zeros(\
				(self.img_height_px, \
				self.img_width_px - (self.lat_upsamp * self.trans_num_elem) - self.zero_pad_width_px), \
				dtype=np.uint8)

		self.match_histograms = False
		self.normalize_img = True

		self.grad_desc_cfg_file_name = grad_desc_cfg_file_name

		self.grad_desc_cfg = psl_gn_cfg.get_cfg_defaults()
		self.grad_desc_cfg.merge_from_file(self.grad_desc_cfg_file_name)

		if self.grad_desc_cfg.GRAD_DESC.COST_FUNCTION == 'ax_shifts':
			self.gauss_newton_opt = psl_least_sq.NewtonLocSosRadEstimator(self.grad_desc_cfg)
		else:
			raise RuntimeError('Unknown cost function \'{:s}\'.'.format(\
					self.grad_desc_cfg.GRAD_DESC.COST_FUNCTION))

		if torch.cuda.is_available():
			self.device = 'cuda:0'
		else:
			self.device = 'cpu'

		self.bridge = CvBridge()

	def closeBagFile(self):
		'''
		Close the bag file handler if it exists
		'''
		if self.bag_fh is not None:
			self.bag_fh.close()

	def extractPointSourceEstimatesBaseFrame(self, t_start, t_end):
		'''
		Extract point source estimates, transform them to the robot base frame, and write them to file.
		'''
		print('Opening file \'{:s}\' for writing...'.format(self.robot_pose_h5_file_name))

		bag_info = self.bag_fh.get_type_and_topic_info()
		bag_start_time = self.bag_fh.get_start_time()

		ft_probe_topic_name = '/netft/proc_probe'
		chdat_topic_name = '/verasonics/channel_data'
		lkf_topic_name = '/visual_servoing/multi_track_lkf'
		pt_src_det_topic_name = '/pt_src_loc/detections'
		ur_hw_topic_name = '/ur_hardware_interface/robot_program_running'

		num_detections = bag_info.topics[pt_src_det_topic_name].message_count
		num_ft_msgs = bag_info.topics[ft_probe_topic_name].message_count
		num_img = bag_info.topics[chdat_topic_name].message_count
		num_lkf_msgs = bag_info.topics[lkf_topic_name].message_count

		sys_ab_time_vec = np.zeros((num_detections, 1), dtype=np.float64)
		sys_ab_num_det_mat = np.zeros((num_detections, 2), dtype=np.float64)
		sys_ab_robot_running_vec = np.zeros((num_detections, 1), dtype=np.float64)
		sys_ab_tf_trans_mat = np.zeros((num_detections, 3), dtype=np.float64)
		sys_ab_tf_rot_mat = np.zeros((num_detections, 4), dtype=np.float64)

		sys_c_time_vec = np.zeros((num_img, 1), dtype=np.float64)
		sys_c_exe_time_vec = np.zeros((num_img, 1), dtype=np.float64)
		sys_c_num_det_mat = np.zeros((num_img, 2), dtype=np.float64)
		sys_c_robot_running_vec = np.zeros((num_img, 1), dtype=np.float64)
		sys_c_tf_trans_mat = np.zeros((num_img, 3), dtype=np.float64)
		sys_c_tf_rot_mat = np.zeros((num_img, 4), dtype=np.float64)

		ft_time_vec = np.zeros((num_ft_msgs, 1), dtype=np.float64)
		ft_probe_mat = np.zeros((num_ft_msgs, 6), dtype=np.float64)

		lkf_time_vec = np.zeros((num_lkf_msgs, 1), dtype=np.float64)
		lkf_num_filters_vec = np.zeros((num_lkf_msgs, 1), dtype=np.float64)
		lkf_validity_vec = np.zeros((num_lkf_msgs, 1), dtype=np.float64)
		lkf_state_dict = {\
				'LKF_TRACKING': 0, 'LKF_MISSED_ONE': 1, 'LKF_MISSED_TWO': 2, 'LKF_INITIALIZE': 3, 'LKF_DELETE': 4}

		robot_program_running = False

		det_msg_id = 0
		ft_msg_id = 0
		img_msg_id = 0
		lkf_msg_id = 0

		with tqdm(total=num_img, desc='Processing images', unit='img') as progress_bar:
			for topic, msg, t in self.bag_fh.read_messages(\
					topics=[ft_probe_topic_name, chdat_topic_name, lkf_topic_name, pt_src_det_topic_name, \
					ur_hw_topic_name]):
				# Get the time of the current message, typically required by each data storage process.
				t_sec = t.to_sec() - bag_start_time

				if topic == ft_probe_topic_name:
					# Extract and store the FT readings.
					ft_time_vec[ft_msg_id] = t_sec
					ft_probe_mat[ft_msg_id, 0] = msg.wrench.force.x
					ft_probe_mat[ft_msg_id, 1] = msg.wrench.force.y
					ft_probe_mat[ft_msg_id, 2] = msg.wrench.force.z
					ft_probe_mat[ft_msg_id, 3] = msg.wrench.torque.x
					ft_probe_mat[ft_msg_id, 4] = msg.wrench.torque.y
					ft_probe_mat[ft_msg_id, 5] = msg.wrench.torque.z
					ft_msg_id += 1
				elif topic == chdat_topic_name:
					sys_c_time_vec[img_msg_id] = t_sec
					sys_c_robot_running_vec[img_msg_id] = robot_program_running

					# Get the current transform from the probe to the robot base.
					probe_pose_base_mat = np.zeros((4, 4), dtype=np.float64)
					probe_pose_base_mat[:3, 3], rot_q = self.bag_tf.lookupTransform(\
							self.base_frame_name, self.probe_frame_name, t)
					probe_pose_base_mat[:3, :3] = Rotation.from_quat(rot_q).as_matrix()
					probe_pose_base_mat[3, 3] = 1.0

					sys_c_tf_trans_mat[img_msg_id, :] = probe_pose_base_mat[:3, 3]
					sys_c_tf_rot_mat[img_msg_id, :] = rot_q

					inference_start = time.time()

					pred_input = self.rosToCvImg(msg)
					pred_output = self.detectron2_predictor(pred_input)
					num_predictions = len(pred_output['instances'])
					src_id_list = []
					max_conf_src_id = -1
					max_conf_score = 0.0
					pt_src_conf_score_vec = []

					for i0 in range(num_predictions):
						if pred_output['instances'][i0].pred_classes < sys_c_num_det_mat.shape[1]:
							sys_c_num_det_mat[img_msg_id, pred_output['instances'][i0].pred_classes] += 1

							if pred_output['instances'][i0].pred_classes == self.src_class_id:
								src_id_list.append(i0)
								pt_src_conf_score_vec.append(pred_output['instances'][i0].scores.item())

								if(\
										(max_conf_src_id < 0) \
										or (pred_output['instances'][i0].scores.item() > max_conf_score)):
									max_conf_src_id = i0
									max_conf_score = pred_output['instances'][i0].scores.item()

					num_src = len(src_id_list)

					pt_src_pos_probe_mat = np.zeros(\
							(num_src, self.grad_desc_cfg.GRAD_DESC.NUM_ITERATIONS + 1, 3), dtype=np.float64)
					pt_src_sos_mat = np.zeros(\
							(num_src, self.grad_desc_cfg.GRAD_DESC.NUM_ITERATIONS + 1), dtype=np.float64)
					pt_src_rad_mat = np.zeros(\
							(num_src, self.grad_desc_cfg.GRAD_DESC.NUM_ITERATIONS + 1), dtype=np.float64)
					pt_src_bbox_mat = np.zeros((num_src, 4), dtype=np.float64)

					for i0 in range(len(src_id_list)):
						# Process each detection to get an estimate of the corresponding point source location.
						dt_src_loc_sos_mat = \
								torch.zeros(self.grad_desc_cfg.GRAD_DESC.NUM_ITERATIONS + 1, 5, 1).to(self.device)

						# Extract the predicted segmentation mask regardless of the corresponding predicted class.
						pred_mask = pred_output['instances'][src_id_list[i0]].pred_masks.to(torch.float32)

						# Construct matrix containing lower and upper bounds of region corresponding to point source.
						non_zero_seg_ch = (pred_mask[0] > 0.0).max(dim=0)

						if non_zero_seg_ch.values.sum() > 0:
							dt_point_mat = torch.zeros(4, non_zero_seg_ch.values.sum()).to(self.device)
							dt_point_mat[0, :] = torch.Tensor(\
									[i1 for i1 in range(pred_mask.shape[-1]) if non_zero_seg_ch.values[i1] > 0])
							dt_point_mat[2, :] = dt_point_mat[0, :]
							dt_point_mat[1, :] = non_zero_seg_ch.indices[[\
									i1 for i1 in range(pred_mask.shape[-1]) if non_zero_seg_ch.values[i1] > 0]]
							dt_point_mat[3, :] = msg.height - 1 - (pred_mask[0] > 0.0).flip(0).max(dim=0).indices[[\
									i1 for i1 in range(pred_mask.shape[-1]) if non_zero_seg_ch.values[i1] > 0]]

							# Use the shape estimation output as the initial guess for the Gauss Newton algorithm.
							# TODO: Figure out dimensions of `dt_src_loc_sos_mat` here.
							dt_src_loc_sos_mat[0] = self.gauss_newton_opt.generateInitialEstimate(dt_point_mat)

							for i1 in range(self.grad_desc_cfg.GRAD_DESC.NUM_ITERATIONS):
								dt_src_loc_sos_mat[i1 + 1, :3], dt_src_loc_sos_mat[i1 + 1, 3], \
										dt_src_loc_sos_mat[i1 + 1, 4] = self.gauss_newton_opt(\
										dt_point_mat, dt_src_loc_sos_mat[i1, :3], dt_src_loc_sos_mat[i1, 3], \
										dt_src_loc_sos_mat[i1, 4])

							dt_src_loc_sos_mat[-1, :3], dt_src_loc_sos_mat[-1, 3], dt_src_loc_sos_mat[-1, 4] = \
									self.gauss_newton_opt.limitEstimate(\
									dt_src_loc_sos_mat[-1, :3], dt_src_loc_sos_mat[-1, 3], dt_src_loc_sos_mat[-1, 4])
							print(\
									'(x_s, c) = [{:.2f}, {:.2f}, {:.2f}] mm, {:.2f} m/s, {:.2f} um ({:d} iter)'.format(\
									1.0e3 * dt_src_loc_sos_mat[-1, 0].item(), 1.0e3 * dt_src_loc_sos_mat[-1, 1].item(), \
									1.0e3 * dt_src_loc_sos_mat[-1, 2].item(), dt_src_loc_sos_mat[-1, 3].item(), \
									1.0e6 * dt_src_loc_sos_mat[-1, 4].item(), \
									self.grad_desc_cfg.GRAD_DESC.NUM_ITERATIONS))

							pt_src_pos_probe_mat[i0] = dt_src_loc_sos_mat[:, :3, 0].detach().cpu().numpy()
							pt_src_sos_mat[i0] = dt_src_loc_sos_mat[:, 3, 0].detach().cpu().numpy()
							pt_src_rad_mat[i0] = dt_src_loc_sos_mat[:, 4, 0].detach().cpu().numpy()

					inference_end = time.time()
					sys_c_exe_time_vec[img_msg_id] = inference_end - inference_start

					if len(src_id_list) > 0:
						output_h5_file_name = os.path.join(\
								self.output_dir_name, 'sys_c_{:06d}.h5'.format(img_msg_id))

						with h5py.File(output_h5_file_name, 'w') as h5_fh:
							h5_fh.create_dataset('/pt_src/conf_score', data=np.array(pt_src_conf_score_vec))
							h5_fh.create_dataset('/pt_src/p42v_link1', data=pt_src_pos_probe_mat)
							h5_fh.create_dataset('/pt_src/sound_speed', data=pt_src_sos_mat)
							h5_fh.create_dataset('/pt_src/src_radius', data=pt_src_rad_mat)
							h5_fh.create_dataset('/pt_src/time', data=np.array([t_sec]))
							h5_fh.create_dataset('/tf/p42v_link1', data=probe_pose_base_mat)
							# h5_fh.create_dataset('/pt_src/bbox', data=np.array(pt_src_bbox_mat))
							h5_fh.create_dataset(ur_hw_topic_name, data=np.array([robot_program_running]))

						output_mask_file_name = os.path.join(\
								self.output_dir_name, 'seg_mask_c_{:06d}.h5'.format(img_msg_id))

						with h5py.File(output_mask_file_name, 'w') as mask_fh:
							for i0 in range(len(src_id_list)):
								pred_mask = pred_output['instances'][src_id_list[i0]].pred_masks.to(torch.float32)
								mask_fh.create_dataset(\
										'/sys_c/src_mask_{:02d}'.format(i0 + 1), data=pred_mask.detach().cpu().numpy())

					output_img_file_name = os.path.join(self.output_dir_name, 'chdat_{:06d}.h5'.format(img_msg_id))

					with h5py.File(output_img_file_name, 'w') as img_fh:
						img_fh.create_dataset('/chdat', data=pred_input)

					img_msg_id += 1
					progress_bar.update(1)
				elif topic == lkf_topic_name:
					lkf_time_vec[lkf_msg_id] = t_sec
					lkf_num_filters_vec[lkf_msg_id] = len(msg.filters)
					lkf_validity_vec[lkf_msg_id] = len(lkf_state_dict)

					if len(msg.filters) > 0:
						for i0 in range(len(msg.filters)):
							if lkf_state_dict[msg.filters[i0].fsm_state] < lkf_validity_vec[lkf_msg_id]:
								lkf_validity_vec[lkf_msg_id] = lkf_state_dict[msg.filters[i0].fsm_state]

					lkf_msg_id += 1
				elif topic == pt_src_det_topic_name:
					sys_ab_time_vec[det_msg_id] = t_sec
					sys_ab_robot_running_vec[det_msg_id] = robot_program_running

					# Get the current transform from the probe to the robot base.
					probe_pose_base_mat = np.zeros((4, 4), dtype=np.float64)
					probe_pose_base_mat[:3, 3], rot_q = self.bag_tf.lookupTransform(\
							self.base_frame_name, self.probe_frame_name, t)
					probe_pose_base_mat[:3, :3] = Rotation.from_quat(rot_q).as_matrix()
					probe_pose_base_mat[3, 3] = 1.0

					sys_ab_tf_trans_mat[det_msg_id, :] = probe_pose_base_mat[:3, 3]
					sys_ab_tf_rot_mat[det_msg_id, :] = rot_q

					pt_src_pos_probe_mat = []
					pt_src_conf_score_vec = []
					pt_src_bbox_mat = []

					# Get each source detection exceeding the confidence score and extract the point source location to
					# a list.
					if len(msg.segmentations) > 0:
						for i0 in range(len(msg.segmentations)):
							if(\
									(len(msg.segmentations[i0].results) > 0) \
									and (msg.segmentations[i0].results[0].id == self.src_class_id) \
									and (msg.segmentations[i0].results[0].score >= self.conf_score_thresh)):
								sys_ab_num_det_mat[det_msg_id, msg.segmentations[i0].results[0].id] += 1

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

								output_h5_file_name = os.path.join(\
										self.output_dir_name, 'sys_ab_{:06d}.h5'.format(det_msg_id))

								with h5py.File(output_h5_file_name, 'w') as h5_fh:
									h5_fh.create_dataset('/tf/p42v_link1', data=probe_pose_base_mat)
									h5_fh.create_dataset('/pt_src/p42v_link1', data=np.array(pt_src_pos_probe_mat))
									h5_fh.create_dataset('/pt_src/conf_score', data=np.array(pt_src_conf_score_vec))
									h5_fh.create_dataset('/pt_src/bbox', data=np.array(pt_src_bbox_mat))
									h5_fh.create_dataset('/pt_src/time', data=np.array([t_sec]))
									h5_fh.create_dataset(ur_hw_topic_name, data=np.array([robot_program_running]))

					det_msg_id += 1
				elif topic == ur_hw_topic_name:
					robot_program_running = msg.data
				else:
					raise ValueError('Unknown ROS topic \'{:s}\'.'.format(topic))

		with h5py.File(self.robot_pose_h5_file_name, 'w') as robot_pose_fh:
			# Write the fetched force readings and timestamps to the corresponding datasets in the output HDF5 file.
			robot_pose_fh.create_dataset('/netft/time', data=ft_time_vec[:ft_msg_id])
			robot_pose_fh.create_dataset('/netft/proc_probe', data=ft_probe_mat[:ft_msg_id, :])

			robot_pose_fh.create_dataset('/multi_track_lkf/time', data=lkf_time_vec[:lkf_msg_id])
			robot_pose_fh.create_dataset('/multi_track_lkf/num_filters', data=lkf_num_filters_vec[:lkf_msg_id])
			robot_pose_fh.create_dataset('/multi_track_lkf/validity', data=lkf_validity_vec[:lkf_msg_id])

			robot_pose_fh.create_dataset('/sys_ab/time', data=sys_ab_time_vec[:det_msg_id])
			robot_pose_fh.create_dataset('/sys_ab/num_detections', data=sys_ab_num_det_mat[:det_msg_id, :])
			robot_pose_fh.create_dataset('/sys_ab/robot_program_running', data=sys_ab_robot_running_vec[:det_msg_id, :])
			robot_pose_fh.create_dataset('/sys_ab/tf/p42v_link1/trans', data=sys_ab_tf_trans_mat[:det_msg_id, :])
			robot_pose_fh.create_dataset('/sys_ab/tf/p42v_link1/rot', data=sys_ab_tf_rot_mat[:det_msg_id, :])

			robot_pose_fh.create_dataset('/sys_c/time', data=sys_c_time_vec[:img_msg_id])
			robot_pose_fh.create_dataset('/sys_c/exe_time', data=sys_c_exe_time_vec[:img_msg_id])
			robot_pose_fh.create_dataset('/sys_c/num_detections', data=sys_c_num_det_mat[:img_msg_id, :])
			robot_pose_fh.create_dataset('/sys_c/robot_program_running', data=sys_c_robot_running_vec[:img_msg_id, :])
			robot_pose_fh.create_dataset('/sys_c/tf/p42v_link1/trans', data=sys_c_tf_trans_mat[:img_msg_id, :])
			robot_pose_fh.create_dataset('/sys_c/tf/p42v_link1/rot', data=sys_c_tf_rot_mat[:img_msg_id, :])

		print('Closed file \'{:s}\'.'.format(self.robot_pose_h5_file_name))

	def generateOutputDirName(self, trial_id):
		'''
		Generate the name of the directory to which the output HDF5 file is to be written.
		'''
		self.output_dir_name = self.bag_file_name.replace('.bag', '_{:02d}'.format(trial_id))

		if not os.path.isdir(self.output_dir_name):
			print('Creating directory \'{:s}\'...'.format(self.output_dir_name))
			os.makedirs(self.output_dir_name)

	def generateRobotPoseFileName(self):
		'''
		Generate the name of the HDF5 file to which the data is to be written
		'''
		self.robot_pose_h5_file_name = os.path.join(self.output_dir_name, 'robot_poses.h5')

	def normalizeImg(self, input_img):
		img_height, img_width, img_channels = input_img.shape

		# Normalize image by subtracting mean and rescaling peak amplitude.
		src_img = copy.deepcopy(input_img)
		src_img = src_img.astype(np.float32)
		src_mean = np.mean(src_img)
		src_img = src_img - src_mean
		src_abs_max = np.max(np.absolute(src_img))
		src_img = src_img / src_abs_max
		src_img = ((src_img + 1.0)) * (255.0 / 2.0)

		output_img = copy.deepcopy(src_img)
		output_img = output_img.astype(np.uint8)

		return output_img

	def openBagFile(self):
		'''
		Open the bag file if it has not already been opened.
		'''
		if self.bag_fh is None:
			print('Opening file \'{:s}\' (read-only)...'.format(self.bag_file_name))
			self.bag_fh = rosbag.Bag(self.bag_file_name, 'r')

			print('Instantiating TF-Bag object...')
			self.bag_tf = BagTfTransformer(self.bag_fh)

	def rosToCvImg(self, img_msg):
		if img_msg is None:
			return None

		encoding = None

		if img_msg.encoding.lower() in ['rgb8', 'bgr8']:
			encoding = np.uint8
			channels = 3
		elif img_msg.encoding.lower() == 'mono8':
			encoding = np.uint8
			channels = 1
		elif img_msg.encoding.lower() == '32fc1':
			encoding = np.float32
			channels = 1

		cv_img = np.ndarray(shape=(img_msg.height, img_msg.width, channels), dtype=encoding, buffer=img_msg.data)
		print('Detectron2: Received image shape = {}'.format(cv_img.shape))
		print('Detectron2: Received image: {:d} x {:d} px, encoding: \'{:s}\''.format(\
				cv_img.shape[0], cv_img.shape[1], img_msg.encoding.lower()))

		if self.normalize_img:
			cv_img = self.normalizeImg(cv_img)
			print('Detectron2: Normalized image to shift mean.')

		# Resize takes input dimensions in reversed order.
		cv_img = cv.resize(cv_img, (self.trans_num_elem * self.lat_upsamp, self.img_height_px))
		print('Detectron2: Resized image: {:d} x {:d} px'.format(cv_img.shape[0], cv_img.shape[1]))

		# Zero-pad as required.
		cv_img = np.concatenate((self.left_zero_pad, cv_img, self.right_zero_pad), 1)
		print('Detectron2: Zero-padded image: {:d} x {:d} px'.format(cv_img.shape[0], cv_img.shape[1]))

		if img_msg.encoding.lower() == 'mono8':
			# The Detectron2 predictor requires a 3-channel image.
			cv_img = cv.cvtColor(cv_img, cv.COLOR_GRAY2RGB)
		else:
			# Our images are all grayscale, so the conversion from RGB to BGR below is not necessary.
			# cv_img = cv.cvtColor(cv_img, cv.COLOR_RGB2BGR)
			pass

		return cv_img

def main():
	parser = argparse.ArgumentParser(description='Extract required information from ROS1 bag files')
	parser.add_argument('-i', '--input-file-name', type=str, required=True, help='Name of bag file to be parsed')
	parser.add_argument(\
			'-s', '--t-start', type=float, required=True, help='Relative start time of current trial to be processed')
	parser.add_argument(\
			'-e', '--t-end', type=float, required=True, help='Relative end time of current trial to be processed')
	parser.add_argument(\
			'-t', '--trial-id', type=int, required=True, help='Index of current trial within input bag file')
	parser.add_argument(\
			'-g', '--grad-desc-cfg-file-name', \
			type=str, \
			default=os.path.join(os.path.dirname(__file__), os.pardir, os.pardir, os.pardir, os.pardir, os.pardir, \
			os.pardir, os.pardir, 'deep_learning_projects', 'point_source_localization', 'theory_3d', 'configs', \
			'newton', 'reparameterize', 'default.yaml'), \
			help='Name of configuration file to be used for gradient descent')
	parser.add_argument(\
			'-c', '--conf-score-thresh', type=float, default=0.0, \
			help='Confidence score threshold to be applied to detections')
	args = parser.parse_args()
	robot_pose_extractor = RobotPoseExtractorSystemC(\
			args.grad_desc_cfg_file_name, args.input_file_name, args.trial_id, args.conf_score_thresh)
	robot_pose_extractor.openBagFile()
	robot_pose_extractor.extractPointSourceEstimatesBaseFrame(args.t_start, args.t_end)
	robot_pose_extractor.closeBagFile()

if __name__ == '__main__':
	main()
