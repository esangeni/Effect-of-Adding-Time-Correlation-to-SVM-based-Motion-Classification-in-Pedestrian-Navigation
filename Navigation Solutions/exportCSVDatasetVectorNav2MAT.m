clc; clear all; close all;

datasetFolder = [pwd '\dataset'];
fileName = 'exp1200';
dataFolder = '\2023_08_30';
filpath  = [datasetFolder dataFolder];
savepath = [datasetFolder dataFolder '\']; %Saved Directory

mode = '.csv'; 
n = dir([savepath fileName mode]); % Change me
for j =1:size(n)
    cur_file_path = split(n(j).folder,'\');
    save_fname = cur_file_path{end};
    %     if strcmp(fname,'exp6300')
    %     if 1
    fname = n(j).name;
    filename = [n(j).folder '\' fname ];
    if ~isfile([save_fname '_VN.mat']) || 1
        
        g  = 9.817269086191379;
        %f0 = 492; %in Hz used by ADIS % Change me
        %f0 = 472; %in Hz used by ADIS
        f0 = 800;%1/2e-3; %in Hz used in Flex platform
        
        data = readtable(filename);
        time = 0:1/f0:1/f0*(length(data.UncompAccX)-1);
        
        u(1,:) = data.UncompAccX/g; % X acc, in g [m/s^2]
        u(2,:) = data.UncompAccY/g; % Y acc
        u(3,:) = data.UncompAccZ/g; % Z acc
        u(4,:) = data.UncompGyroX*180/pi; % X gyr, in deg/sec
        u(5,:) = data.UncompGyroY*180/pi; % Y gyr
        u(6,:) = data.UncompGyroZ*180/pi; % Z gyr
        u(7,:) = u(1,:)*0; % flag gyr
        u(8,:) = time;
        u(9,:) = u(1,:)*0; % Debug var        
        u(10,:) = u(1,:)*0; % pressure sensor, Pa
        u(11,:) = 1/f0; % sampling period, s 
        u(12,:) = data.UncompMagX; % mag x, uT
        u(13,:) = data.UncompMagY; % mag y, uT
        u(14,:) = data.UncompMagZ; % mag z, uT
        u(15,:) = data.Temperature; % temperature, degree C
        
        if exist('u2','var')
            fname_sav = [savepath fname '_VN_L.mat'];
            u = u1;
            save(fname_sav,'u')  % Normal recording.
            disp(fname_sav)
            fname_sav = [savepath fname '_VN_R.mat'];
            u = u2;
            save(fname_sav,'u')  % Normal recording.
            disp(fname_sav)
            clear u u1 u2
        else
            %         fname_sav = [savepath fname '_VN.mat'];
            fname_sav = [savepath fileName '_VN.mat'];
            save(fname_sav,'u')  % Normal recording.
            disp(fname_sav)
            clear u1
        end
        clear u
    end
end