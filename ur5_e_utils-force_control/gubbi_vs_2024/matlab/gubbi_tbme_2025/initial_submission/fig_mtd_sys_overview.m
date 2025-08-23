addpath(genpath(fullfile(...
	'..', '..', '..', '..', '..', '..', '..', 'beamforming_code', 'image_quality_metrics', 'utils')));

set(...
	0, ...
	'defaultlinelinewidth', 3, ...
	'defaultaxesfontsize', 14, ...
	'defaultlinemarkersize', 7, ...
	'defaulterrorbarlinewidth', 3);

img_id = 19;
img_id_str = sprintf('%06d', img_id);

input_img_dir_name = fullfile(...
	'..', 'datasets', 'paimdb', 'images', 'p42v_12cm_spheres', 'p42v_12cm_1s1r_500um_20240903_bp_109p4', 'Images');
elev_coco_bbox_anno_dir_name = fullfile(...
	'..', 'datasets', 'paimdb', 'images', 'p42v_12cm_spheres', 'p42v_12cm_1s1r_500um_20240903_bp_109p4_zp', ...
	'Annotations', 'elev_class_1mm_64x25', 'Annotations', 'ElevClassBbox');
coco_seg_mask_anno_dir_name = fullfile(...
	'..', 'datasets', 'paimdb', 'images', 'p42v_12cm_spheres', 'p42v_12cm_1s1r_500um_20240903_bp_109p4_256x926', ...
	'Annotations', 'theory_seg_masks_1r', 'Annotations', 'Theory');

input_img_file_name = fullfile(input_img_dir_name, [img_id_str, '.jpg']);
fprintf(['Reading file ''', input_img_file_name, '''...\n']);
input_img = imread(input_img_file_name);

imagesc(input_img);
colormap gray
yticks([200, 400, 600, 800]);
xlabel('Lateral [pixels]');
ylabel('Axial [pixels]');

output_img_dir_name = fullfile('.', 'img', 'mtd_sys_overview');
createMissingDirectories(output_img_dir_name);
output_img_file_name = fullfile(output_img_dir_name, 'input_img.png');
fprintf(['Saving file ''', output_img_file_name, '''...\n']);
saveas(gcf, output_img_file_name);

zero_pad_region = zeros(size(input_img, 1), 251, size(input_img, 3), 'uint8');
obj_det_img = [zero_pad_region, input_img, zero_pad_region];
imagesc(obj_det_img);
colormap gray
hold on

% TODO: Load elevation-encoded bounding box annotations.
elev_coco_bbox_anno_file_name = fullfile(elev_coco_bbox_anno_dir_name, [img_id_str, '.txt']);
fprintf(['Reading file ''', elev_coco_bbox_anno_file_name, '''...\n']);
elev_coco_bbox_fh = fopen(elev_coco_bbox_anno_file_name, 'r');
elev_coco_bbox_lines = textscan(elev_coco_bbox_fh, '%s', 'Delimiter', '\n');
fclose(elev_coco_bbox_fh);

for i0 = 1:length(elev_coco_bbox_lines)
	for i1 = 1:length(elev_coco_bbox_lines{i0})
		text_seg = textscan(elev_coco_bbox_lines{i0}{i1}, '%s %f %f %f %f');

		if startsWith(text_seg{1}, 'src')
			fprintf([elev_coco_bbox_lines{i0}{i1}, '\n']);
			bbox_coord = [...
				text_seg{2}, text_seg{4}, text_seg{4}, text_seg{2}, text_seg{2}; ...
				text_seg{3}, text_seg{3}, text_seg{5}, text_seg{5}, text_seg{3}];
			plot(bbox_coord(1, :), bbox_coord(2, :), 'r');
		else
			fprintf('Ignoring reflection artifact...\n');
		end
	end
end

hold off
yticks([200, 400, 600, 800]);
xlabel('Lateral [pixels]');
ylabel('Axial [pixels]');
output_img_file_name = fullfile(output_img_dir_name, 'obj_det.png');
fprintf(['Saving file ''', output_img_file_name, '''...\n']);
saveas(gcf, output_img_file_name);

clf(gcf);

seg_img = imresize(input_img, [926, 256]);
coco_seg_mask_anno_file_name = fullfile(coco_seg_mask_anno_dir_name, [img_id_str, '.mat']);
fprintf(['Loading file ''', coco_seg_mask_anno_file_name, '''...\n']);
coco_seg_mask_data = load(coco_seg_mask_anno_file_name);

x_vec = [...
	squeeze(coco_seg_mask_data.ch_data.hyp_point_mat(1, :, 1)), ...
	flip(squeeze(coco_seg_mask_data.ch_data.hyp_point_mat(1, :, 3)))];
y_vec = [...
	squeeze(coco_seg_mask_data.ch_data.hyp_point_mat(1, :, 2)), ...
	flip(squeeze(coco_seg_mask_data.ch_data.hyp_point_mat(1, :, 4)))];

imagesc(seg_img);
colormap gray
hold on
fill(x_vec, y_vec, 'r', 'LineWidth', 3);
hold off
yticks([200, 400, 600, 800]);
xlabel('Lateral [pixels]');
ylabel('Axial [pixels]');
output_img_file_name = fullfile(output_img_dir_name, 'inst_segm.png');
fprintf(['Saving file ''', output_img_file_name, '''...\n']);
saveas(gcf, output_img_file_name);
