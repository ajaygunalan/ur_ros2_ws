input_vid_file_name = 'input_vid.avi';
input_vid_fh = VideoReader(input_vid_file_name);

output_vid_file_name = 'output_vid.avi';
output_vid_fh = VideoWriter(output_vid_file_name);

fprintf(['Opening file ''', output_vid_file_name, ''' for writing...\n']);
open(output_vid_fh);

frame_counter = 0;

while hasFrame(input_vid_fh)
	input_frame = readFrame(input_vid_fh);
	frame_counter = frame_counter + 1;

	if mod(frame_counter, 4) == 0
		writeVideo(output_vid_fh, input_frame);
	end
end

fprintf(['Closing file ''', output_vid_file_name, '''...\n']);
close(output_vid_fh);
