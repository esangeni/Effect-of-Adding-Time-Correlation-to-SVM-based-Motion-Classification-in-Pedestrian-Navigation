clc; clear; close all;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% File Header
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Authors:
%   Chico Jao: chishihj@uci.edu
% 
%   Description:
%   This file converts the vectornav data file to a matlab array
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Initial Parameters
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

datasetFolder = [pwd '\dataset'];
dataFolder = '\2022_06_02';

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Start of the code
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

FIGURES = findall(0,'type','figure'); % For clearing all figures.
for i=1:length(FIGURES)
    figure(i)
    clf(FIGURES(i));
    %hold on
end
filpath  = [datasetFolder dataFolder];
savepath = [datasetFolder dataFolder '\']; %Saved Directory
mode = 'dat'; %'bin' 'dat' 'csv'
n = dir([savepath '**\*.' mode]); % Change me
for j =1:size(n)
    cur_file_path = split(n(j).folder,'\');
    save_fname = cur_file_path{end};

    fname = n(j).name;
    filename = [n(j).folder '\' fname ];
    if ~isfile([save_fname '_VN.mat']) || 1
        
        g  = 9.817269086191379;
        f0 = 800; % Hz

        data = readtable(filename);
        time = 0:1/f0:1/f0*(length(data.UnCompAccX)-1);
        
        u(1,:) = data.UnCompAccX/g; % X acc, in g [m/s^2]
        u(2,:) = data.UnCompAccY/g; % Y acc
        u(3,:) = data.UnCompAccZ/g; % Z acc
        u(4,:) = data.UnCompGyroX*180/pi; % X gyr, in deg/sec
        u(5,:) = data.UnCompGyroY*180/pi; % Y gyr
        u(6,:) = data.UnCompGyroZ*180/pi; % Z gyr
        u(7,:) = u(1,:)*0; % flag gyr
        u(8,:) = time;
        u(9,:) = u(1,:)*0; % Debug var        
        u(10,:) = u(1,:)*0; % pressure sensor, Pa
        u(11,:) = 1/f0; % sampling period, s    
        u(12,:) = data.UnCompMagX; % mag x, uT
        u(13,:) = data.UnCompMagY; % mag y, uT
        u(14,:) = data.UnCompMagZ; % mag z, uT
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
            fname_sav = [savepath save_fname '_VN.mat'];
            save(fname_sav,'u')  % Normal recording.
            disp(fname_sav)
            clear u1
        end
        clear u
    end
end