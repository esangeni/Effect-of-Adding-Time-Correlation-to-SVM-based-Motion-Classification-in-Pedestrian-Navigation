% -------------------------------------------------------------------------
% This function calculated adaptive threshld for Acceleration-Magnitude Variance detector
% Input: u:     IMU readouts at current time step
%        xi:    Uncertainty of estimated velocity
%        dt:    Time differnce beween last time ZUPT is on and current step
%        ZUPT:  ZUPT state of previous time step
%        thre1: Threshold of previous time step
%        shock: Maximum shock level of last step
% Output: zupt:  Detected ZUPT state. 1 is stance and 0 is swing
%         LR:    Likelihood ratio
%         thre2: Actual adaptive threshold
% -------------------------------------------------------------------------
function [zupt,total,total_x,total_y,total_z] = AccelMov_detector(u)


% Global struct holding the simulation settings
global simdata;


% Run the desired detector type. Each detector return a vector with their 
% calculated test statistics T. 


W=simdata.Window_size;
sigma2_a = (simdata.accel/simdata.Ts)^2;

tmp = var(u, 0, 2);

total = 0;
total_x = tmp(1)/sigma2_a;
total_y = tmp(2)/sigma2_a;
total_z = tmp(3)/sigma2_a;

for i = 1:3
    total = total + tmp(i)/sigma2_a;
end

total = total/3;

if(total < simdata.factor)
    zupt = ones(1, simdata.Window_size);
else
    zupt = zeros(1, simdata.Window_size);
end

end

