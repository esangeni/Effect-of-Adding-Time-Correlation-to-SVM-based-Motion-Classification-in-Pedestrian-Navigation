function [sigma_zupt, threshold_zupt] = STD_ZUPTwithMotionClassification(classification_results,w_cfm)

%% STOCHASTIC THRESHOLD DETRMINATION ZUPT:

%                thres, sigma
% thr_sigma_zupt = [3.00, 0.005;...  % Stationary        - act 1
%     3.00, 0.020;...  % Walking           - act 2
%     4.19, 0.030;...  % Fast walking      - act 3
%     6.89, 0.050;...  % Jogging           - act 4
%     7.80, 0.100;...  % Running           - act 5
%     8.70, 0.100;...  % Sprinting         - act 6
%     3.00, 0.020;...  % walking backward  - act 7
%     5.40, 0.020;...  % running backward  - act 8
%     5.70, 0.050;...  % Side stepping     - act 9
%     5.70, 0.050];    % Side stepping     - act 10

thr_sigma_zupt = [3.00, 0.005;...  % Stationary        - act 1
    4.80, 0.020;...  % Walking           - act 2
    5.2, 0.030;...  % Fast walking      - act 3
    6.89, 0.050;...  % Jogging           - act 4
    7.8, 0.100;...  % Running           - act 5
    8.7, 0.100;...  % Sprinting         - act 6
    4.5, 0.020;...  % walking backward  - act 7
    6.0, 0.020;...  % running backward  - act 8
    6.0, 0.050;...  % Side stepping     - act 9
    6.0, 0.050];    % Side stepping     - act 10

% Weights CFM:
% Rows 1-10 are the zupt activities;
% Rows 11-19 are the motion activities
%     w_cfm = load(['2022_04_25_cfm_rbf_C100_6sig_df_Chico_v2.csv']);


if classification_results == 0 % Stationary
    act = 1;
    sigma_zupt = thr_sigma_zupt(act,2);
    threshold_zupt = STD_thr(act,thr_sigma_zupt,w_cfm);
elseif classification_results == 1 || classification_results == 1+9 % Walking
    act = 2;
    sigma_zupt = thr_sigma_zupt(act,2);
    threshold_zupt = STD_thr(act,thr_sigma_zupt,w_cfm);
elseif classification_results == 2 || classification_results == 2+9 % Fast walking
    act = 3;
    sigma_zupt = thr_sigma_zupt(act,2);
    threshold_zupt = STD_thr(act,thr_sigma_zupt,w_cfm);
elseif classification_results == 3 || classification_results == 3+9 % Jogging
    act = 4;
    sigma_zupt = thr_sigma_zupt(act,2);
    threshold_zupt = STD_thr(act,thr_sigma_zupt,w_cfm);
elseif classification_results == 4 || classification_results == 4+9 % Running
    act = 5;
    sigma_zupt = thr_sigma_zupt(act,2);
    threshold_zupt = STD_thr(act,thr_sigma_zupt,w_cfm);
elseif classification_results == 5 || classification_results == 5+9 % Sprinting
    act = 6;
    sigma_zupt = thr_sigma_zupt(act,2);
    threshold_zupt = STD_thr(act,thr_sigma_zupt,w_cfm);
elseif classification_results == 6 || classification_results == 6+9 % walking backward
    act = 7;
    sigma_zupt = thr_sigma_zupt(act,2);
    threshold_zupt = STD_thr(act,thr_sigma_zupt,w_cfm);
elseif classification_results == 7 || classification_results == 7+9 % running backward
    act = 8;
    sigma_zupt = thr_sigma_zupt(act,2);
    threshold_zupt = STD_thr(act,thr_sigma_zupt,w_cfm);
elseif classification_results == 8 || classification_results == 8+9 % Side stepping
    act = 9;
    sigma_zupt = thr_sigma_zupt(act,2);
    threshold_zupt = STD_thr(act,thr_sigma_zupt,w_cfm);
elseif classification_results == 9 || classification_results == 9+9 % Side stepping
    act = 10;
    sigma_zupt = thr_sigma_zupt(act,2);
    threshold_zupt = STD_thr(act,thr_sigma_zupt,w_cfm);
end

end

function threshold_zupt = STD_thr(act,thr_sigma_zupt,w_cfm)
%     if act == 1
%         threshold_zupt = thr_sigma_zupt(act,1)*w_cfm(1,act)+...
%                          thr_sigma_zupt(act,1)*w_cfm(11,act)+...
%                          thr_sigma_zupt(act,1)*w_cfm(12,act)+...
%                          thr_sigma_zupt(act,1)*w_cfm(13,act)+...
%                          thr_sigma_zupt(act,1)*w_cfm(14,act)+...
%                          thr_sigma_zupt(act,1)*w_cfm(15,act)+...
%                          thr_sigma_zupt(act,1)*w_cfm(16,act)+...
%                          thr_sigma_zupt(act,1)*w_cfm(17,act)+...
%                          thr_sigma_zupt(act,1)*w_cfm(18,act)+...
%                          thr_sigma_zupt(act,1)*w_cfm(19,act);
%     else
%         threshold_zupt = thr_sigma_zupt(act,1)*w_cfm(1,act+9)+...
%                          thr_sigma_zupt(act,1)*w_cfm(11,act+9)+...
%                          thr_sigma_zupt(act,1)*w_cfm(12,act+9)+...
%                          thr_sigma_zupt(act,1)*w_cfm(13,act+9)+...
%                          thr_sigma_zupt(act,1)*w_cfm(14,act+9)+...
%                          thr_sigma_zupt(act,1)*w_cfm(15,act+9)+...
%                          thr_sigma_zupt(act,1)*w_cfm(16,act+9)+...
%                          thr_sigma_zupt(act,1)*w_cfm(17,act+9)+...
%                          thr_sigma_zupt(act,1)*w_cfm(18,act+9)+...
%                          thr_sigma_zupt(act,1)*w_cfm(19,act+9);
%     end


threshold_zupt = thr_sigma_zupt(act,1)*w_cfm(1,act)+...
    thr_sigma_zupt(act,1)*(w_cfm(2,act)+w_cfm(2+9,act))+...
    thr_sigma_zupt(act,1)*(w_cfm(3,act)+w_cfm(3+9,act))+...
    thr_sigma_zupt(act,1)*(w_cfm(4,act)+w_cfm(4+9,act))+...
    thr_sigma_zupt(act,1)*(w_cfm(5,act)+w_cfm(5+9,act))+...
    thr_sigma_zupt(act,1)*(w_cfm(6,act)+w_cfm(6+9,act))+...
    thr_sigma_zupt(act,1)*(w_cfm(7,act)+w_cfm(7+9,act))+...
    thr_sigma_zupt(act,1)*(w_cfm(8,act)+w_cfm(8+9,act))+...
    thr_sigma_zupt(act,1)*(w_cfm(9,act)+w_cfm(9+9,act))+...
    thr_sigma_zupt(act,1)*(w_cfm(10,act)+w_cfm(10+9,act));

end