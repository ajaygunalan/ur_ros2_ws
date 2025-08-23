%% Script to ptip segmentation based on C++ node's segmented image
% Author: Jessica Su jsu30@jhu.edu

data_path = "/home/pulser/Jessica/data";
files = dir(data_path);
files = files(~ismember({files.name},{'.','..','debug_data.txt'}));

for i=1:size(files, 1)
    img = imread(strcat(files(i).folder, filesep, files(i).name));
    % img = img ./ 255;
    m00 = calculateMoment(img, 0, 0);
    if (m00 > 0)
        % Non-empty image
        pcenter = calculatePCenter(img);
        [eigVal, eigVec] = calculateEig(img, pcenter);
        % Show image
        figure;
        imagesc(img, [0 1]);
        colormap('gray');
        hold on;
        
        quiver(pcenter(1), pcenter(2), ...
            2 * sqrt(abs(eigVal(1))) * sign(eigVal(1)) * eigVec(1, 1), ...
            2 * sqrt(abs(eigVal(1))) * sign(eigVal(1)) * eigVec(2, 1), ...
            '-g', 'AutoScale', 'off', 'LineWidth', 2);
        quiver(pcenter(2), pcenter(1), ...
            2 * sqrt(abs(eigVal(2))) * sign(eigVal(2)) * eigVec(1, 2), ...
            2 * sqrt(abs(eigVal(2))) * sign(eigVal(2)) * eigVec(2, 2), ...
            '-r', 'AutoScale', 'off', 'LineWidth', 2);

        plot(pcenter(1), pcenter(2),'b.');
        hold off;
        xlabel('x');
        ylabel('z');
        title(files(i).name)
        
        pause;
    end
end

function [eigVal, eigVec] = calculateEig(img, pcenter)
covI = calculateCovI(img, pcenter);
[eigVec, eigVal] = eig(covI);

% Sort eigenvalues from small to large
if (eigVal(1, 1) > eigVal(2, 2))
    [eigVal(2, 2), eigVal(1, 1)] = deal(eigVal(1, 1), eigVal(2, 2));
    eigVec = eigVec(:,[2 1]);
end

end

function covI = calculateCovI(img, pcenter)
covI = zeros(2, 2);
m00 = calculateMoment(img, 0, 0);
m20 = calculateMoment(img, 2, 0);
m11 = calculateMoment(img, 1, 1);
m02 = calculateMoment(img, 0, 2);
xc = pcenter(1);
zc = pcenter(2);
covI(1, 1) = m20/m00 - xc^2;
covI(1, 2) = m11/m00 - xc*zc;
covI(2, 1) = m11/m00 - xc*zc;
covI(2, 2) = m02/m00 - zc^2;
end

function M = calculateMoment(img, p, q)
M = 0;
for i=1:size(img, 2)
    for j=1:size(img, 1)
        M = M + (((i-1) ^ p) * ((j-1) ^ q) * double(img(j, i)));
    end
end
end

function pcenter = calculatePCenter(img)
m10 = calculateMoment(img, 1, 0);
m00 = calculateMoment(img, 0, 0);
m01 = calculateMoment(img, 0, 1);
xc = m10 / m00;
zc = m01 / m00;
pcenter = [xc; zc];
end