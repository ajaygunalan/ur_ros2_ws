clear all
clc
% Choose the folder that contains the SWEI images and the image coordinates
folder = uigetdir;
cd(folder);
param = load('parameters.txt');
ic = load('image_coordinates.txt');
numscans = param(1);
numframes = param(3);
% make sure ic is a matrix of the form 'p' 'q' 'imnum' 'scannum' 'tstamp'
% first row will be of the form 'scannum' 'scansep' 'imnum'
images = dir('*.png');
% make sure ic_p is of the form ('x' 'y' 'z' by 'imnum' by 'scannum')
%img = imread('SWEimage2017_09_27_21_57_19_528321.png');

for i = 1:numscans
    for j = 1:numframes
        % Read in the images and clean them up a bit
        %img = rgb2gray(imread(images(j + (i-1)*numframes).name));
        img = imread(images(j + (i-1)*numframes).name);
        img(400:444, 1:40) = 0;
        imgdata(i,j,:,:) = img(2:end, 2:end);

        % Format the coordinate data in a more intuitive way
        ic_p(i,j,:) = ic(j + (i-1)*numframes, 1:3);
        ic_p2(j + (i-1)*numframes,:) = ic(j + (i-1)*numframes, 1:3);
    end
end
[y, x] = size(img);

ky = 0.0676; % [mm / px]
kx = 0.069307; % [mm / px]
xs = ic_p2(:,2);
ys = ic_p2(:,1);
zs = ic_p2(:,3);


ymax = max(ys);
ymin = min(ys);
ydifference = ymax - ymin;
ypixeldifference = floor((ydifference / ky) * 1000); % Distance in voxels between highest and
                                    % lowest image collected

max_distance = 0;
for i = 1:numframes
    x1 = ic_p(1, i, 2);
    z1 = ic_p(1, i, 3);
    for j = 2:numscans
        x2 = ic_p(j, i, 2);
        z2 = ic_p(j, i, 3);
        distance = sqrt((x1-x2)^2 + (z1-z2)^2);
        if distance > max_distance
            max_distance = distance;
        end
    end
end
xpixeldifference = floor((max_distance / kx) * 1000); % Distance in voxels between the
                                            % most far apart images collected

img_height = y;
img_width = x;
z_size = numframes;
y_size = img_height + ypixeldifference + 1;
x_size = img_width + xpixeldifference + 1;
v = zeros(y_size,x_size,z_size);
% Empty volume has been sized and initialized
size(imgdata)
for i = 1:numscans
    for j = 1:numframes
       % Calculate where to start placing the pixels from the current 2D
       % image within the 3D volume
       ystart = floor(((ymax - ys((i-1)*numframes + j))/ky)*1000) + 1;
       xstart = 1;
       if i > 1
           x1 = ic_p(1, i, 2);
           z1 = ic_p(1, i, 3);

           x2 = ic_p(i, i, 2);
           z2 = ic_p(i, i, 3);

           distance = sqrt((x1-x2)^2 + (z1-z2)^2);
           xstart = floor((distance / kx) * 1000);
       end
       % Iterate through our current image, and place it into the volume
       for k = 1:img_height-1
           for l = 1:img_width-1
               % If the voxel is unoccupied, place in the image value
               if v(ystart+k-1, xstart+l-1, j) == 0
                   v(ystart+k-1, xstart+l-1, j)=imgdata(i,j,k,l);
               % I am not sure if these elseifs are necessary
               elseif v(ystart+k-1, xstart+l-1, j) > 255
                   v(ystart+k-1, xstart+l-1, j) = 255;
               elseif v(ystart+k-1, xstart+l-1, j) < 0
                   v(ystart+k-1, xstart+l-1, j) = 0;
               % If the voxel is occupied, average the new and old values
               else
                   current_value = v(ystart+k-1, xstart+l-1, j);
                   new_value = imgdata(i,j,k,l);
                   average_value = (current_value + new_value) / 2;
                   v(ystart+k-1, xstart+l-1, j)=average_value;
               end
           end
       end
       %v(ystart:ystart+img_height-1, xstart:xstart+img_width-1, j)=imgdata(i,j,:,:);
    end
end
%v = imgaussfilt3(v,1);

for i=1:51

    %v(:,:,i) = imgaussfilt(v(:,:,i),1.25);
    %imshow(squeeze(v(:,:,i)),[0 255])

    slice(v, [], [], i)
    %shading interp
    xlim([0 811])
    ylim([0 818])
    zlim([0 51])
    drawnow
    %drawnow
end

save('volume.mat','v')
%{
[x y z] = ind2sub(size(v), find(v));
plot3(x, y, z, 'k.');
[x y z] = size(v)

v = reshape(v,[x y 1 z]);
delay = 0.003;
imwrite( v, 'volume.gif', 'DelayTime', delay)
%}

