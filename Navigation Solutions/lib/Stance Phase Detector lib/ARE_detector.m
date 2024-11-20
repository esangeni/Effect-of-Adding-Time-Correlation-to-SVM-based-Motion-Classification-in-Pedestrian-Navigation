function [zupt,total,total_x,total_y,total_z] = ARE_detector(u)
% -------------------------------------------------------------------------
% This function calculated adaptive threshld for Angular Rate Energy detector
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
global simdata;

total_x = 1;
total_y = 1;
total_z = 1;

[r, c] = size(u);
W=simdata.Window_size;
sigma2_g = (simdata.sigma_g/simdata.Ts)^2;

total = sum(sum(u(4:6,:).^2))/sigma2_g;
total = total/c;

if(total < simdata.factor)
    zupt = ones(1, simdata.Window_size);
else
    zupt = zeros(1, simdata.Window_size);
end
end

