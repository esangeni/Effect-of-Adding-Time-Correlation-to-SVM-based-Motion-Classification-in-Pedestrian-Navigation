%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%  funtion [zupt] = zero_velocity_detector_2(u, X) 
%
%>
%> @brief Wrapper function for running the zero-velocity detection 
%> algorithms. 
%>
%> @details A wrapper function that runs the zero-velocity detection 
%> algorithm that was specified in the file \a setting.m. 
%>
%> @param[out]  zupt       Vector with the detector decsions. [ true = zero velocity, false = moving]    
%> @param[in]   u          The IMU data vector
%>
%> Note: This function is the same as zero_velocity_detector but also
%outputs statistics information
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [zupt,total,total_x,total_y,total_z,total_shoe] = downward_ultrasnoic_aided_ZUPT_detector(u,shoe_height)

% Global struct holding the simulation settings
global simdata;
% true_shoe_height = 0.04; % meters
true_shoe_height = simdata.factorShoeHeight; % meters

% true_shoe_height = 0.06; % meters
% true_shoe_height = 0.10; % meters

% Run the desired detector type. Each detector return a vector with their 
% calculated test statistics T. 


W=simdata.Window_size;
sigma2_a = (simdata.accel/simdata.Ts)^2;
sigma2_g = (simdata.gyro/simdata.Ts)^2;
% sigma2_shoe = 0.0001;
% sigma2_shoe = 0.000000000000000000000001;
% sigma2_shoe = 0.00000000000000000000000000000001;
% sigma2_shoe = 0.000000000000000000000000000000000000001;
sigma2_shoe = 1e-38;
% sigma2_shoe = 0.000000000001;

tmp = var(u, 0, 2);

total = 0;
total_x = tmp(1)/sigma2_a+tmp(4)/sigma2_g;
total_y = tmp(2)/sigma2_a+tmp(5)/sigma2_g;
total_z = tmp(3)/sigma2_a+tmp(6)/sigma2_g;

for i = 1:3
    total = total + tmp(i)/sigma2_a;
    total = total + tmp(i+3)/sigma2_g;
end

% total_DVS = (mean(fire_rate))^5/sigma2_DVS;
% shoe_height = log(shoe_height);

total_shoe = sum(abs(shoe_height-true_shoe_height).^2)/sigma2_shoe+1;
% total_shoe = sum(abs(shoe_height-log(true_shoe_height)).^2)/sigma2_shoe+1;


total = total + total_shoe;

total = total/7;

if(total < simdata.factorDownwardSONAR)
    zupt = ones(1, simdata.Window_size);
else
    zupt = zeros(1, simdata.Window_size);
end





end

