function fastImgPublishFunction()
	persistent kwave_metadata ...
		img_width img_height img_frame_id ...
		min_pt_src_pos_probe max_pt_src_pos_probe ...
		sound_speed src_rad sensor_data_dir_name ...
		ros_img_topic_name ros_img_pub ros_tf_tree

	% Time the execution of this command to determine how quickly this loop may be run. Ideally, we want to get to
	% 10 Hz.
	perf_check = tic;

	% Load the desired k-Wave metadata file corresponding to the 3D simulated outputs.
	if(...
			(isempty(kwave_metadata)) ...
			|| (isempty(img_width)) || (isempty(img_height)) || (isempty(img_frame_id)) ...
			|| (isempty(min_pt_src_pos_probe)) || (isempty(max_pt_src_pos_probe)) ...
			|| (isempty(sound_speed)) || (isempty(src_rad)) || (isempty(sensor_data_dir_name)))
		kwave_metadata_file_name = fullfile(...
			'..', '..', '..', '..', '..', '..', '..', 'simulations', 'pa_point_sources', 'datasets', 'paimdb', ...
			'sources', 'p42v_3d_12cm_20230627_162555', 'images', 'rt_01s_20240620_182240', 'metadata.mat');
		sensor_data_dir_name = fullfile(fileparts(kwave_metadata_file_name), 'chdat');
		fprintf(['Loading file ''', kwave_metadata_file_name, '''...\n']);
		kwave_metadata = load(kwave_metadata_file_name);

		% TODO: Fix these hard-coded values. Get them from the image metadata file, not the source metadata file above.
		img_width = 64;
		img_height = 926;
		img_frame_id = 0;

		% TODO: If using an image metadata file, these might need to be changed.
		min_pt_src_pos_probe = [...
			-max(kwave_metadata.sim.src.pos.x), -max(kwave_metadata.sim.src.pos.y), min(kwave_metadata.sim.src.pos.z)]';
		max_pt_src_pos_probe = [...
			max(kwave_metadata.sim.src.pos.x), max(kwave_metadata.sim.src.pos.y), max(kwave_metadata.sim.src.pos.z)]';

		% Set the default speed of sound [m/s] and source radius [m] to be used in this emulation.
		sound_speed = 1542;
		src_rad = 1e-4;
	end

	% Initialize the ROS image publisher. This publisher is meant to emulate the Verasonics scanner, so it will publish
	% channel data frames of dimensions 64 x 926 pixels on the channel `/verasonics/channel_data` at a fixed rate. That
	% rate is currently set to a lower value to allow for the images to be generated from the k-Wave simulation outputs.
	if isempty(ros_img_topic_name) || isempty(ros_img_pub)
		ros_img_topic_name = '/verasonics/channel_data';
		ros_img_pub = rospublisher(ros_img_topic_name, 'sensor_msgs/Image', 'DataFormat', 'struct');
	end

	% Initialize the ROS TF tree to fetch the instantaneous position of the point source relative to the transducer.
	if isempty(ros_tf_tree)
		ros_tf_tree = rostf;
	end

	% Get the current position of the point source in the probe frame.
	pt_src_pos_probe = getTransform(ros_tf_tree, 'probe', 'pt_src');

	if ~isempty(pt_src_pos_probe)
		pt_src_pos_probe = [...
			pt_src_pos_probe.Transform.Translation.X, pt_src_pos_probe.Transform.Translation.Y, ...
			pt_src_pos_probe.Transform.Translation.Z]';

		% Generate a channel data frame corresponding to the given source and reflection artifact.
		% Check the source position to see if it is within the range of simulated source positions.
		if((~any(pt_src_pos_probe < min_pt_src_pos_probe)) && (~any(pt_src_pos_probe > max_pt_src_pos_probe)))
			% Fetch the filename of the k-Wave simulation outputs corresponding to the given source position.
			pt_src_pos_probe_x_id = getLateralIndex(...
				pt_src_pos_probe(1), kwave_metadata.trans.elements.pitch, kwave_metadata.sim.grid.res(1));
			[src_file_name, src_file_id] = getIntegratedSensorDataFileName(...
				kwave_metadata, sensor_data_dir_name, ...
				pt_src_pos_probe_x_id, abs(pt_src_pos_probe(2)), pt_src_pos_probe(3));
			fprintf(['Source file name: ''', src_file_name, '''\n']);
			fprintf(['Input : [%.2f, %.2f, %.2f] mm\n'], 1e3 * pt_src_pos_probe);
			fprintf(...
				'Output: [%.2f, %.2f, %.2f] mm\n', 1e3 * squeeze(kwave_metadata.img.src.pos.vec(src_file_id, :, :)));
			fprintf(...
				'Others: %.2f um, %.1f m/s\n', ...
				1e6 * kwave_metadata.img.src.rad.vec(src_file_id), kwave_metadata.img.sos.vec(src_file_id));

			% Slice the simulation outputs to form the desired source waveform. Unlike the PAIMDB simulated datasets,
			% the elevation displacement may be negative here, so use the magnitude to accommodate elevation symmetry.
			% TODO: Image resolution and transducer pitch are not always the same. Fix if performing lateral resampling.
			pt_src_pos_probe_x_offset = kwave_metadata.img.src.pos.vec(pt_src_pos_probe_x_id, 1, 1);
			src_waveform = sliceIntegratedChannelDataImage(...
				src_file_name, kwave_metadata, pt_src_pos_probe(1), pt_src_pos_probe_x_offset, ...
				kwave_metadata.trans.elements.pitch);

			% Generate a random reflector position within a fixed maximum distance of the point source.
			ref_pos_pt_src = 1e-3 .* ((rand(3, 1) .* [10, 2, 10]') - [5, 1, 5]');
			fprintf(...
				'Generating reflection artifact located at [%.2f, %.2f, %.2f] mm relative to point source...\n', ...
				1e3 .* ref_pos_pt_src);

			% Compute the position of the reflector relative to the probe.
			ref_pos_probe = ref_pos_pt_src + pt_src_pos_probe;

			% Check the reflector position to see if it is within the range of simulated source positions.
			if((~any(ref_pos_probe < min_pt_src_pos_probe)) && (~any(ref_pos_probe > max_pt_src_pos_probe)))
				% Fetch the filename of the simulation output corresponding to the given reflector position.
				ref_pos_probe_x_id = getLateralIndex(...
					ref_pos_probe(1), kwave_metadata.trans.elements.pitch, kwave_metadata.sim.grid.res(1));
				ref_file_name = getIntegratedSensorDataFileName(...
					kwave_metadata, sensor_data_dir_name, ref_pos_probe_x_id, abs(ref_pos_probe(2)), ref_pos_probe(3));

				% Compute the Euclidean distance between source and artifact to perform the required axial shift
				% while generating the channel data frame.
				ref_dist = norm(ref_pos_pt_src);
				ref_depth_shift_px = round((ref_dist / sound_speed) * kwave_metadata.trans.samp_freq);

				% Slice the simulation outputs to form the desired artifact waveform, downshifted by the Euclidean
				% distance between source and reflector. Similar to the source waveform, use the magnitude to
				% accommodate elevation symmetry.
				% TODO: Image resolution and transducer pitch are not always the same. Fix if performing lateral
				% resampling.
				ref_pos_probe_x_offset = kwave_metadata.img.src.pos.vec(ref_pos_probe_x_id, 1, 1);
				ref_waveform = sliceIntegratedChannelDataImage(...
					ref_file_name, kwave_metadata, ref_pos_probe(1), ref_pos_probe_x_offset, ...
					kwave_metadata.trans.elements.pitch, 'depthShiftPx', ref_depth_shift_px);
			else
				ref_waveform = zeros(img_height, img_width);
			end
		else
			src_waveform = zeros(img_height, img_width);
			ref_waveform = zeros(img_height, img_width);
		end

		% Add the source and artifact waveforms together to form the desired channel data frame.
		% TODO: Incorporate randomized amplitude scaling factors prior to summation.
		ch_data_img = src_waveform + 0.1 * ref_waveform;
		ch_data_img = imresize(ch_data_img, [img_height, img_width]);
		ch_data_img = ch_data_img - min(ch_data_img(:));
		ch_data_img = ch_data_img .* (255.0 / max(ch_data_img(:)));
		ch_data_img = uint8(ch_data_img);

		% Publish the generated channel data frame as a ROS message on the pre-determined ROS topic.
		ros_img_msg = rosmessage('sensor_msgs/Image', 'DataFormat', 'struct');
		ros_img_msg.Header.Seq = uint32(img_frame_id);
		img_frame_id = img_frame_id + 1;
		ros_img_msg.Header.FrameId = 'ch_data';

		ros_img_msg = rosWriteImage(ros_img_msg, ch_data_img, 'Encoding', 'mono8');
		send(ros_img_pub, ros_img_msg);

		% Time the execution of this function.
		perf_check = toc(perf_check);
		fprintf(['Published image on topic ''', ros_img_topic_name, ''' in %.1f ms.\n'], 1e3 * perf_check);
	else
		fprintf('Waiting for valid transform from point source frame to transducer frame...\n');
	end
end
