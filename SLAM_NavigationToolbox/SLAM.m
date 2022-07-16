% 2D Laser SLAM using MATLAB  (scans collected using RPLIDAR)
clc; clear; close all;

savePlot = false; 

% Load one of the 2D lidar scans
addpath("../data/")
fileName = 'scans_a.mat'; % {scans_a.mat, scans_b.mat}
load(fileName);

% Downsample scans for efficiency
[nSamples, ~] = size(ranges); % get  number of scan samples
startIdx      = 1;        % start index
stepSize      = 3;        % step size
stopIdx       = nSamples; % stop index
sampleIdx     = startIdx:stepSize:stopIdx;
ranges        = ranges(sampleIdx,:);
[nSamples, ~] = size(ranges); % recalculate number of scans after down sampling

% Generate down sampled scans
samples = cell(1, nSamples);
for k = 1:nSamples
    samples{k} = lidarScan(ranges(k,:), angles);
end
clear k

% Create slamAlg Object
maxLidarRange                   = 6;    % meters
mapResolution                   = 20;   % cells/meters
SlamAlg                         = lidarSLAM(mapResolution, maxLidarRange);
SlamAlg.LoopClosureThreshold    = 210;
SlamAlg.LoopClosureSearchRadius = 6;

% For recording run-time
toc_per_sample  = zeros(1,nSamples);
toc_total       = 0;
GridMap  = cell(1,nSamples);

 f = figure('rend','painters','pos',[100 100 800 600]); clf;
for i=1:nSamples
    oneTic = tic;
    [isScanAccepted, loopClosureInfo, optimizationInfo] = addScan(SlamAlg, samples{i});
    [scans, poses]  = scansAndPoses(SlamAlg);
    % Build occupancy grid map
    GridMap{i} = buildMap(scans, poses, mapResolution, maxLidarRange);
    % Calculate run-time
    toc_per_sample(i) = toc(oneTic);
    toc_total         = toc(oneTic)+ toc_total;

    % Plot figures
    show(GridMap{i}); hold on;
    show(SlamAlg.PoseGraph, 'IDs', 'off');
    title('Occupancy Grid Map')
    drawnow
    if savePlot 
        save_gif(f, i)
    end 
    if isScanAccepted
        fprintf('Added scan %d / %d \n', i, nSamples);
    else
        continue;
    end
end
clear i

function save_gif(fig, i)
frame = getframe(fig); % capture frame for file-writing
im = frame2im(frame);
[imind,cm] = rgb2ind(im,256);
if i==1
    imwrite(imind,cm,'anime.gif','gif', 'Loopcount',inf, 'DelayTime', .05);
else
    imwrite(imind,cm,'anime.gif','gif','WriteMode','append', 'DelayTime', .05);
end
end

