function [zupt,LR,thre2] = adaptive_detector_w_DVS(u, xi, dt, ZUPT, thre1, shock, firing_rate)
% -------------------------------------------------------------------------
% This function calculated adaptive threshld for ZUPT detector
% Input: u:     IMU readouts at current time step
%        xi:    Uncertainty of estimated velocity
%        dt:    Time differnce beween last time ZUPT is on and current step
%        ZUPT:  ZUPT state of previous time step
%        thre1: Threshold of previous time step
%        shock: Maximum shock level of last step
%        firing_rate: readouts from side-facing shoe-mounted DVS128
% Output: zupt:  Detected ZUPT state. 1 is stance and 0 is swing
%         LR:    Likelihood ratio
%         thre2: Actual adaptive threshold
% -------------------------------------------------------------------------
    global simdata;
    alpha = simdata.alpha;
    theta = simdata.theta;
    beta = simdata.beta*0;
    p = [0.0364 7.9276];  % Derived from experimental results
    p = [0.032 7.9276];  % Adjusted according to differnet subjects
%     p = [0.0307 8.6348];  % According to the paper
%     p = [0.0364 7.7276];  % According to the paper
    temp = exp(p(1)*shock + p(2));
    [r, c] = size(u);
    W=simdata.Window_size;
    sigma2_a = (simdata.sigma_a/simdata.Ts)^2;
    sigma2_g = (simdata.sigma_g/simdata.Ts)^2;
    sigma2_DVS = 1;
    g = 9.796;
    u_n = mean(u(1:3, :), 2);
    u_n = u_n / norm(u_n);  % Unit vector along the specific force
    for i = 1:3
        u(i, :) = u(i, :) - g*u_n(i);
    end
    total = sum(sum(u(1:3,:).^2))/sigma2_a+sum(sum(u(4:6,:).^2))/sigma2_g + sum(firing_rate.^2)/sigma2_DVS;
    total = total/c;
%     thre2 = alpha + theta*temp*log(dt/0.005) - beta*xi;
    thre2 = alpha + theta*temp*log(dt/0.0083) - beta*xi;
    % Hold the threshold during the stance phase
    if (ZUPT == 0)
%         thre2 = alpha + theta*temp*log(dt/0.005) - beta*xi;
        thre2 = alpha + theta*temp*log(dt/0.0083) - beta*xi;
    elseif ZUPT ==1
        thre2 = thre1;
    end
    if(thre2 < 1000)
        thre2 = 1000;
    end
    if(total < thre2)
        zupt = 1;
    else
        zupt = 0;
    end
    LR = total;
end

