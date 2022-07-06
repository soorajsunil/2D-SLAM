% 2D Laser SLAM using MATLAB  (real data collected using Quanser Qcar) 

clc; clear; close all; 

% Load one of the 2D lidar scans from {scans_a.mat, scans_b.mat}
load('scans_a.mat'); 

% get  number of scans and scan length of the data 
[nSamples, scanLength] = size(ranges); 

% Downsample scans for efficiency 
startIdx  = 1;     % start index 
stepSize  = 3;      % step size 
stopIdx   = nSamples; % stop index 
sampleIdx = startIdx:stepSize:stopIdx; 

% Generate down sampled scans 
nSamples    = length(sampleIdx); 
samples     = cell(1, nSamples); 
samples{1}  = lidarScan(ranges(startIdx,:),angles);

for k = 2:nSamples
    samples{k} = lidarScan(ranges(startIdx+(stepSize*(k-1)),:), angles);    
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
map_per_sample  = cell(1,nSamples); 

figure;
for i=1:nSamples
   
    oneTic = tic; 
    
    [isScanAccepted, loopClosureInfo, optimizationInfo] = ...
                                               addScan(SlamAlg, samples{i});
                                           
    [scans, optimizedPoses]  = scansAndPoses(SlamAlg);
    
    % Build occupancy grid map 
    GridMap = buildMap(scans, optimizedPoses,mapResolution, maxLidarRange);
    
    % Calculate run-time
    toc_per_sample(i) = toc(oneTic);  
    toc_total         = toc(oneTic)+ toc_total; 
    
    % Plot figures 
    show(GridMap); hold on; 
    show(SlamAlg.PoseGraph, 'IDs', 'off');
    title('Occupancy Grid Map')
    hold off; drawnow

    if isScanAccepted
       fprintf('Added scan %d \n', i);
    else
        continue;
    end

    map_per_sample{i} = GridMap; % save maps for external purposes 
        
end
clear i 

% Save data for external purposes 

% SlamInfo.Ranges             = Ranges; 
% SlamInfo.Angles             = Angles; 
% SlamInfo.downSampleStepSize = stepSize; 
% SlamInfo.samplesAdded       = length(sampleIdx);  
% SlamInfo.slamObj            = SlamAlg; 
% SlamInfo.sampleIndex        = sampleIdx; 
% SlamInfo.time_per_sample    = toc_per_sample; 
% SlamInfo.totalTime          = toc_total;
% SlamInfo.Map_per_sample     = map_per_sample; 


