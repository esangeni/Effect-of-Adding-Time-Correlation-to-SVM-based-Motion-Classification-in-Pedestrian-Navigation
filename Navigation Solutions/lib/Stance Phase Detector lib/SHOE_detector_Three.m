function [zupt,total,total_x,total_y,total_z] = SHOE_detector_Three(u)
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
global simdata_Three;

total_x = 1;
total_y = 1;
total_z = 1;

[r, c] = size(u);
W=simdata_Three.Window_size;
sigma2_a = (simdata_Three.sigma_a/simdata_Three.Ts)^2;
sigma2_g = (simdata_Three.sigma_g/simdata_Three.Ts)^2;
g = 9.796;
u_n = mean(u(1:3, :), 2);
u_n = u_n / norm(u_n);  % Unit vector along the specific force
for i = 1:3
    u(i, :) = u(i, :) - g*u_n(i);
end
total = sum(sum(u(1:3,:).^2))/sigma2_a+sum(sum(u(4:6,:).^2))/sigma2_g;
total = total/c;

if(total < simdata_Three.factor)
    zupt = ones(1, simdata_Three.Window_size);
else
    zupt = zeros(1, simdata_Three.Window_size);
end
end

