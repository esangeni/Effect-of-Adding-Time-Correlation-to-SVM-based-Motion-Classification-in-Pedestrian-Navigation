clc
clear
filpath = pwd; % main working folder storing all navigation information
savepath = [filpath, '\dataset','\2022_02_24'];

% savepath = [filpath '\']; %Saved Directory
mode = '_SC_v3_GPS.mat';
n = dir([savepath '\*' mode]); % Change me
g  = 9.817269086191379;
r2d = 180/pi;

ind_shift = 3;

for j =1:size(n)
    filename = n(j).name;
    load([savepath '\' n(j).name])
    N = min(find(isnan(dataStreamArray(1,:))))-1;
    u = zeros(15,N);
    u(11,:) = dataStreamArray(1+ind_shift,1:N);
    u(8,:) = cumsum(dataStreamArray(1+ind_shift,1:N));
    u(1,:) = dataStreamArray(2+ind_shift,1:N)/g;
    u(2,:) = dataStreamArray(3+ind_shift,1:N)/g;
    u(3,:) = dataStreamArray(4+ind_shift,1:N)/g;
    u(4,:) = dataStreamArray(5+ind_shift,1:N)*r2d;
    u(5,:) = dataStreamArray(6+ind_shift,1:N)*r2d;
    u(6,:) = dataStreamArray(7+ind_shift,1:N)*r2d;
    
    est_GT.orientation = dataStreamArray((8:10)+ind_shift,1:N);
    est_GT.velocity = dataStreamArray((11:13)+ind_shift,1:N);
    est_GT.position = dataStreamArray((14:16)+ind_shift,1:N);
    est_GT.LLA = dataStreamArray((19:21)+ind_shift,1:N);
    ZUPT_Detection_GT.statistics = dataStreamArray(17+ind_shift,1:N);
    ZUPT_Detection_GT.state = dataStreamArray(18+ind_shift,1:N);
    
    GNSS.LLA = dataStreamArray(1:2,1:N)/1e7;
    GNSS.hor_acc = dataStreamArray(3,1:N);
    
    save([savepath '\' n(j).name(1:end-length(mode)) '.mat'],'u')
    save([savepath '\' n(j).name(1:end-length(mode)) '_GT.mat'],'est_GT','ZUPT_Detection_GT')
    save([savepath '\' n(j).name(1:end-length(mode)) '_GNSS.mat'],'GNSS')
    
end