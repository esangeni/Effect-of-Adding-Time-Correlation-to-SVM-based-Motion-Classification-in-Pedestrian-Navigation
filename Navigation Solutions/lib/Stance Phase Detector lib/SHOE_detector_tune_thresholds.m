function [zupt,total,total_x,total_y,total_z] = SHOE_detector_tune_thresholds(u,threshold)
% -------------------------------------------------------------------------
% This function calculated adaptive threshld for ZUPT detector
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
sigma2_a = (simdata.sigma_a/simdata.Ts)^2;
sigma2_g = (simdata.sigma_g/simdata.Ts)^2;
g = 9.796;
u_n = mean(u(1:3, :), 2);
u_n = u_n / norm(u_n);  % Unit vector along the specific force
for i = 1:3
    u(i, :) = u(i, :) - g*u_n(i);
end
total_x = sum(sum(u(1,:).^2))/sigma2_a+sum(sum(u(4,:).^2))/sigma2_g;
total_y = sum(sum(u(2,:).^2))/sigma2_a+sum(sum(u(5,:).^2))/sigma2_g;
total_z = sum(sum(u(3,:).^2))/sigma2_a+sum(sum(u(6,:).^2))/sigma2_g;

total = sum(sum(u(1:3,:).^2))/sigma2_a+sum(sum(u(4:6,:).^2))/sigma2_g;
total = total/c;

if(total < threshold)
    zupt = ones(1, simdata.Window_size);
else
    zupt = zeros(1, simdata.Window_size);
end
end

