close all; clear all; clc; warning off;
set(0,'DefaultTextInterpreter', 'latex');
set(0,'DefaultAxesTickLabelInterpreter','latex');
set(0,'DefaultLegendInterpreter','latex');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% File Header
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Authors:
%   Eudald Sangenis: esangeni@uci.edu
%   Chico Jao: chishihj@uci.edu
% 
%   Description:
%   This file implements three types of algorithms described on the Journal
%   paper: ZUPT-SHOE, ZUPT-SVM, ZUPT-SHOE-SVM.
%
%   Algorithms involved:
%
%   1. Inertial Navigation System for one foot
%   2. Zero Velocity Update
%   3. SVM classification
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Initial Parameters
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Parameters to select
subject = 'III'; % I, II, III
data_filename = 'exp1200'; % exp100, exp200, exp300, ... exp1000
typeClassifier = 'timeSeriesSVM'; % standardSVM or timeSeriesSVM

zupt = 0; % ZUPT switch 1 is on, 0 Inertial Navigation (0-1)
standaloneSVMdetector_switch = 0; % Switch to use ZUPT-SVM (0-1), zupt has to be 1
svm_SHOEdetector_switch = 0; % Swtich to use ZUPT-SVM-SHOE (0-1), zupt has to be 1
% If zupt = 1 && both switches == 0 -> ZUPT-SHOE algorithm

% Remember to Navigate to the:
% "lib\Activities_Classification_lib\ZUPTwithMotionClassification"
% file in order to change the theresholds and uncertainty for the ZUPT 
% activities while ZUPT-SVM-SHOE algorithm is selected.
% Inside the function there are four if statments three of them, contribute
% the optimal threshold parmeters for each subject I, II, III; and the last
% one is for the case that we select the same threshold for the three
% subjects.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Start of the code
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
progress_bar = waitbar(0,'Initialization...');

yaw_Deg =  0;
currDir = pwd;
addpath([currDir, '\lib\INS lib']);
addpath([currDir, '\lib\Stance Phase Detector lib']);

% Load Datasets: Journal Paper Datasets
if subject == "I"
    datasetDir = [currDir, '\dataset','\2023_09_07']; % Subject I (Chico)
end
if subject == "II"
    datasetDir = [currDir, '\dataset','\2023_08_23']; % Subject II (Eudald)
end
if subject == "III"
    datasetDir = [currDir, '\dataset','\2023_08_30']; % Subject III (Austin)
end
addpath(datasetDir);
load([data_filename '_VN.mat']);

% Load SVM predictions depending on the type of classifier:
if typeClassifier == "standardSVM" || zupt == 1
    if subject == "I" && (data_filename == "exp100" || data_filename == "exp200" || data_filename == "exp300" || data_filename == "exp400" || data_filename == "exp500")
        load([data_filename '_SVM.mat']); % For Journal Paper simple SVM clf
    else
        data = load([data_filename '_SVM.mat']); % For Journal Paper time series SVM clf
        svm_results = data.([data_filename '_SVM']);
    end

    svm_window = 1; % Simple SVM
end

if typeClassifier == "timeSeriesSVM" || zupt == 1
    if subject == "I" && (data_filename == "exp100" || data_filename == "exp200" || data_filename == "exp300" || data_filename == "exp400" || data_filename == "exp500")
        load([data_filename '_SVM_HD400.mat']); % For Journal Paper time series SVM clf just for the first 3 datasets of the 2023_09_07 folder
    else
        data = load([data_filename '_SVM_HD400.mat']); % For Journal Paper time series SVM clf
        svm_results = data.([data_filename '_SVM']);
    end
%     load([data_filename '_SVM_HD400.mat']); % For Journal Paper time series SVM clf just for the first 3 datasets of the 2023_09_07 folder

    svm_window = 400; % Window Size: 400 HD
end

if zupt == 0
    zupt = zupt + 20; % higher than the number of classes is ok, just to not interfere with the rest of the code.
    if subject == "I" && (data_filename == "exp100" || data_filename == "exp200" || data_filename == "exp300" || data_filename == "exp400" || data_filename == "exp500")
        load([data_filename '_SVM.mat']); % For Journal Paper simple SVM clf
    else
        data = load([data_filename '_SVM.mat']); % For Journal Paper time series SVM clf
        svm_results = data.([data_filename '_SVM']);
    end

    svm_window = 1; % Simple SVM
end

if svm_window == 400
    u = u(:,svm_window+1-50:end-50);
end
if svm_window == 1
    u = u;
end

figDspl = [];
dspl.enblCov = 1;
s=settings_constructed_data();
global simdata;
d2r = pi/180;
r2d = 180/pi;
g = 9.817269086191379;

IMU_dt = floor(mean(1./u(11,100:200)));

acc_thre = [16 - 0.1,-(16 - 0.1);16 - 0.1,-(16 - 0.1);16 - 0.45,-16+0.45];

u_raw = u;

if_recontructed = 0;
if_variance = if_recontructed;

% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Navigation Solution
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%------------------------------------------------------------------------

m = 1;  % Number of averaging of raw data to adjust sampling rate.
cal = 3*IMU_dt; % Number of initial time steps for calibration

% Averaging the data
for i = 1:m:length(u)-m
    for j = 1:6
        v(j, (i-1)/m+1) = mean(u(j, i:i+m-1));
    end
end

% gravity
a     = 6378137;               % m ( semi-major axis)
e2    = 6.694380004260835e-3;  % EarthEccentricitySq
cLat = cos(simdata.latitude);
sLat = sin(simdata.latitude);
ax   = 1-e2*sLat^2;
R_N  = a/sqrt(ax);
R_M  = R_N*(1-e2)/ax;
h    = simdata.altitude;
r_e_n          = [-R_N*e2*sLat*cLat; 0; -R_N*ax - h];
g_n            = gravityModel(simdata.latitude, norm(r_e_n), a, e2);
g = norm(g_n);

% converting accelerometer readouts to m/s^2 and gyroscope to rad/s
v(1:3, :) = v(1:3, :) * g;    % convert to m/s^2
v(4:6, :) = v(4:6, :) * d2r;  % convert to rad/s

v = v';

N = length(v);

% Calculating initial orientation

f_u=mean(v(2:cal,1));
f_v=mean(v(2:cal,2));
f_w=mean(v(2:cal,3));

roll=atan2(-f_v,-f_w);
pitch=atan2(f_u,sqrt(f_v^2+f_w^2));


roll_Deg = roll * r2d;
pitch_Deg = pitch * r2d;

q_b2n_0  = Imu2YPRdeg_to_qb2n(roll_Deg,pitch_Deg,yaw_Deg);

% calculating initial states for the EKF
latDeg = simdata.latitude * r2d;
lonDeg = simdata.longitude * r2d;
h = simdata.altitude;

V_0 = zeros(3, 1);
dt = simdata.Ts;
true.t = cumsum(u(11,2:end));

q_e2N0_l =  lonLatDegTo_q_e2N(lonDeg, latDeg);
% Quaternion of navigation to earth frame combined with transport rate
q_e2N_l  = integVel_into_q_e2N(q_e2N0_l, h, V_0, dt);

LLA_0 = [simdata.longitude; simdata.latitude; h];

input.q_b2n          = q_b2n_0;
input.q_e2n          = q_e2N_l;
input.LLA            = LLA_0;
input.v_nWrtE_n      = V_0;
input.delta_b_Prev   = zeros(3,1);
input.delta_n_Prev   = zeros(3,1);
input.delta_n2e_Prev = zeros(3,1);

sensor.dt = dt;
true.dt = dt;

est.t = true.t;
n = length(est.t);
est.q_b2n_l     = [input.q_b2n(:,1), zeros(4,n-1)];
est.q_e2n_l     = [input.q_e2n(:,1), zeros(4,n-1)];
est.LLA_l       = [input.LLA(:,1),   zeros(3,n-1)];
est.v_nWrtE_n_l = [input.v_nWrtE_n(:,1),    zeros(3,n-1)];

% set gyro noise characteristics
gyro.sigma_AWN = 2e-4*d2r*0;               % rad
gyro.sigma_ARW = simdata.sigma_g/sqrt(simdata.Ts) * 1;           % rad/sqrt(s)
gyro.sigma_RRW = simdata.gyro_bias_driving_noise/sqrt(simdata.Ts) * 1;   %  (rad/s)/sqrt(s)
gyro.Bias      = simdata.sigma_initial_gyro_bias';         % rad/s
%
% set accel noise characteristics
accl.sigma_VWN   = 4.5e-4*0;    % m/s
accl.sigma_VRW   = simdata.sigma_a/sqrt(simdata.Ts) *1;        % m/s/sqrt(s)
accl.sigma_AcRW  = simdata.acc_bias_driving_noise/sqrt(simdata.Ts) * 1;        % m/s^2/sqrt(s)
accl.Bias      = simdata.sigma_initial_acc_bias';  % m/s^2

disp(['Accelerometer: VRW = ',num2str(accl.sigma_VRW),' m/s/sqrt(s), ','AcRW = ',num2str(accl.sigma_AcRW), 'm/s^2/sqrt(s)']);
disp(['Gyroscope: ARW = ',num2str(gyro.sigma_ARW),' rad/sqrt(s), ','RRW = ',num2str(gyro.sigma_RRW), '(rad/s)/sqrt(s)']);

w_e2i_n_l        = earthRateInBody(0,0,0, LLA_0(2,1)*180/pi);

% att,vel,pos,gB,aB
e3 = [1,1,1];
Q_diag = [gyro.sigma_ARW^2*e3, accl.sigma_VRW^2*e3, e3*0, ...
    gyro.sigma_RRW^2*e3, accl.sigma_AcRW^2*e3 ]*true.dt; % noise matrix in the EKF
x = [simdata.sigma_initial_att, simdata.sigma_initial_vel, simdata.sigma_initial_pos,...
    simdata.sigma_initial_gyro_bias/3, simdata.sigma_initial_acc_bias/3] * 1; % initialize state in the EKF

P = diag(x.^2); % initialize state covariance matrix in the EKF

H_ZUPT = zeros(3,size(P,1)); % measurement matrix
H_ZUPT(:,4:6) = eye(3,3);
H_ZART = zeros(3,size(P,1)); % measurement matrix
H_ZART(:,10:12) = eye(3,3);

R_ZUPT = diag(simdata.sigma_vel.^2); % measurement noise matrix
R_ZART = diag([0.1,0.1,0.000001].^2); % measurement noise matrix
stance_velocity = [0.0;0;0];

aB_l = zeros(3,1);
gB_l = mean(v(cal-400+1:cal, 4:6))';

dx_l = zeros(size(P,1),1);
Id = eye(size(P));

zupt_l = zeros(1,n);  % ZUPTing marker
zupt_l(1) = 1;  % ZUPTing marker
T = zeros(1,n);   % Test statistics of ZUPTing detector
W = simdata.Window_size;  % ZUPTing window size

O33 = zeros(3,3);
I33 = eye(3,3);
A11 = -skew(w_e2i_n_l);

dx_hist = [];

input_cal = input;

sum_IMU  = [];
sum_IMU_x  = [];
sum_IMU_y  = [];
sum_IMU_z  = [];
innovation_seq = [];
innovation_cov = [];
tic
zupt_varinace_hist = [];
zupt_threshold_hist = [];

for i=2:cal
    if mod(i,10000) == 0
        waitbar(floor(i/length(u)*100)/100,progress_bar,['Estimating navigation solution (initial)... ',num2str(floor(i/length(u)*100)),'%']);        
    end    
    
    sensor.w_b2i_b = v(i, 4:6)' - gB_l;
    sensor.f_b     = v(i, 1:3)'  - aB_l;

    if i < n - W + 2
        [sigma_zupt, threshold_zupt] = ZUPTwithMotionClassification(svm_results(9,i));    
        zupt_varinace_hist(end+1) = sigma_zupt;
        zupt_threshold_hist(end+1) = threshold_zupt;        
        [zupt_l(i:i+W-1),sum_IMU(end+1),sum_IMU_x(end+1),sum_IMU_y(end+1),sum_IMU_z(end+1)]  = SHOE_detector(v(i:i+W-1, :)');
    end

    sensor.dt = u(11,i);
    input_cal = navSLN_ZUPT(sensor, input_cal);
    
    A14 = -quat2dcos(input_cal.q_b2n);
    A21 =  skew(-A14*sensor.f_b);
    A = [A11 O33 O33 A14  O33
        A21 O33 O33 O33 -A14
        O33 I33 O33 O33  O33
        O33 O33 O33 O33  O33
        O33 O33 O33 O33  O33];
    F = expm(A*sensor.dt);
    
    P = F*P*F' + diag(Q_diag);

    z = [];
    H = [];
    R = [];
    if 1
        z = [z;input_cal.v_nWrtE_n];
        H = [H;H_ZUPT];
        R = diag([diag(R);diag(R_ZUPT)]);
    end

    if ~isempty(z)
        
        S = (H*P*H'+R);        
        K  = (P*H')/(H*P*H'+R);
        P  = (Id-K*H)*P;

        dx_l = K*z;
        gB_l = gB_l + dx_l(10:12);
        aB_l = aB_l + dx_l(13:15);

        dx_hist(:,end+1) = dx_l;
        
        input_cal.LLA(2)     = input_cal.LLA(2) - dx_l(7)/simdata.a;
        input_cal.LLA(1)     = input_cal.LLA(1) - dx_l(8)/simdata.a/cos(input_cal.LLA(2));
        input_cal.LLA(3)     = input_cal.LLA(3) + dx_l(9);
        input_cal.q_e2n      = lonLatDegTo_q_e2N(input_cal.LLA(1)*r2d, input_cal.LLA(2)*r2d);
        input_cal.v_nWrtE_n  = input_cal.v_nWrtE_n - dx_l(4:6);
        input_cal.q_b2n      = q_mult(input_cal.q_b2n, theta2quat(-dx_l(1:3)));
        
        innovation_seq(:,end+1) = z(1:3);
        innovation_cov(:,end+1) = diag(S(1:3,1:3));        
    else
        innovation_seq(:,end+1) = [nan;nan;nan];   
        innovation_cov(:,end+1) = [nan;nan;nan];                
    end
    P  = (P+P')/2;
    est.q_b2n_l(:,i)     = input_cal.q_b2n;
    est.q_e2n_l(:,i)     = input_cal.q_e2n;
    est.LLA_l(:,i)       = input_cal.LLA;
    est.v_nWrtE_n_l(:,i) = input_cal.v_nWrtE_n;
    kf.dx_l(:,i)     = dx_l;
    kf.gB_l(:,i)     = gB_l;
    kf.aB_l(:,i)     = aB_l;
    kf.diagP_l(:,i)  = diag(P);
end

for i=cal+1:length(est.t)
    if mod(i,10000) == 0
        waitbar(floor(i/length(u)*100)/100,progress_bar,['Estimating navigation solution... (Main)',num2str(floor(i/length(u)*100)),'%']);        
    end        
    
    sensor.w_b2i_b = v(i, 4:6)' - gB_l;
    sensor.f_b     = v(i, 1:3)'  - aB_l;

    if i < n - W + 2
        [sigma_zupt, threshold_zupt] = ZUPTwithMotionClassification(svm_results(9,i)); 

        zupt_varinace_hist(end+1) = sigma_zupt;
        zupt_threshold_hist(end+1) = threshold_zupt;        

        if svm_SHOEdetector_switch == 1
            [zupt_l(i:i+W-1),sum_IMU(end+1),sum_IMU_x(end+1),sum_IMU_y(end+1),sum_IMU_z(end+1)]  = SVM_SHOE_detector(v(i:i+W-1, :)', exp(threshold_zupt));
        else
            [zupt_l(i:i+W-1),sum_IMU(end+1),sum_IMU_x(end+1),sum_IMU_y(end+1),sum_IMU_z(end+1)]  = SHOE_detector(v(i:i+W-1, :)');
        end
    end

    sensor.dt = u(11,i);
    input = navSLN_ZUPT(sensor, input);
    
    A14 = -quat2dcos(input.q_b2n);
    A21 =  skew(-A14*sensor.f_b);
    A = [A11 O33 O33 A14  O33
        A21 O33 O33 O33 -A14
        O33 I33 O33 O33  O33
        O33 O33 O33 O33  O33
        O33 O33 O33 O33  O33];
    F = expm(A*sensor.dt);
    
    P = F*P*F' + diag(Q_diag);
    
    z = [];
    H = [];
    R = [];

    if ((zupt_l(i) == zupt) && standaloneSVMdetector_switch~=1 ...
            || standaloneSVMdetector_switch == 1 && (svm_results(9,i) <= 9)) % SVM detection results
        
        z = [z;input.v_nWrtE_n - stance_velocity];
        H = [H;H_ZUPT];
        R = diag([diag(R);(sigma_zupt)^2*[1;1;1]]);        
    end

    if ~isempty(z)    
     
        S = (H*P*H'+R);
        K  = (P*H')/(H*P*H'+R);
        P  = (Id-K*H)*P;
        
        dx_l = K*z;
        gB_l = gB_l + dx_l(10:12);
        aB_l = aB_l + dx_l(13:15);

        dx_hist(:,end+1) = dx_l;
        
        input.LLA(2)     = input.LLA(2) - dx_l(7)/simdata.a;
        input.LLA(1)     = input.LLA(1) - dx_l(8)/simdata.a/cos(input.LLA(2));
        input.LLA(3)     = input.LLA(3) + dx_l(9);
        input.q_e2n      = lonLatDegTo_q_e2N(input.LLA(1)*r2d, input.LLA(2)*r2d);
        input.v_nWrtE_n  = input.v_nWrtE_n - dx_l(4:6);
        input.q_b2n      = q_mult(input.q_b2n, theta2quat(-dx_l(1:3)));
        
        innovation_seq(:,end+1) = z;
        innovation_cov(:,end+1) = diag(S);        
    else
        innovation_seq(:,end+1) = [nan;nan;nan];   
        innovation_cov(:,end+1) = [nan;nan;nan];                
    end
    P  = (P+P')/2;
    est.q_b2n_l(:,i)     = input.q_b2n;
    est.q_e2n_l(:,i)     = input.q_e2n;
    est.LLA_l(:,i)       = input.LLA;
    est.v_nWrtE_n_l(:,i) = input.v_nWrtE_n;
    kf.dx_l(:,i)     = dx_l;
    kf.gB_l(:,i)     = gB_l;
    kf.aB_l(:,i)     = aB_l;
    kf.diagP_l(:,i)  = diag(P);
end

% -------------------------------------------------------------------------
est.Northing_l =  (est.LLA_l(2,:)-LLA_0(2,1))*simdata.a;
est.Easting_l  =  (est.LLA_l(1,:)-LLA_0(1,1)).*cos(est.LLA_l(2,:))*simdata.a;
est.Down_l     = -(est.LLA_l(3,:)-LLA_0(3,1));

[roll_deg_l, pitch_deg_l, yaw_deg_l] = qb2nImu2YPRdeg(est.q_b2n_l);
est.rpyDeg_l = [roll_deg_l; pitch_deg_l; yaw_deg_l];

est.rpyDeg_l(1,:) = wrapTo180(est.rpyDeg_l(1,:));
est.rpyDeg_l(2,:) = wrapTo180(est.rpyDeg_l(2,:));
est.rpyDeg_l(3,:) = wrapTo180(est.rpyDeg_l(3,:));

waitbar(1,progress_bar,'Ploting...');        

disp(['Computation Time: ' num2str(toc) ' s'])

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Plotting Results
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if (1)
    % Accelerometer readouts
    figure
    subplot(2,1,1)
    plot( est.t(cal+1:length(est.t)),  v(cal+1:length(est.t), 1)); grid; hold on;
    plot( est.t(cal+1:length(est.t)),  v(cal+1:length(est.t), 2));
    plot( est.t(cal+1:length(est.t)),  v(cal+1:length(est.t), 3));
    ttlMsg = 'Accelerometer readouts';
    title(ttlMsg);
    xlabel('Time, s');
    ylabel('Acceleration, $m/s^2$');
    legend('1', '2', '3');
    figDspl = [figDspl, gcf];

    % Gyroscope readouts
    subplot(2,1,2)
    plot( est.t(cal+1:length(est.t)),  v(cal+1:length(est.t), 4)*r2d); grid; hold on;
    plot( est.t(cal+1:length(est.t)),  v(cal+1:length(est.t), 5)*r2d);
    plot( est.t(cal+1:length(est.t)),  v(cal+1:length(est.t), 6)*r2d);
    ttlMsg = 'Gyroscope readouts';
    title(ttlMsg);
    xlabel('Time, s');
    ylabel('Angular rate, deg/s');
    legend('1', '2', '3');
    figDspl = [figDspl, gcf];
    set( figDspl(end), 'Name', 'Accelerometer and Gyrocope readouts');

    % Adjust layout
    sgtitle('Accelerometer and Gyrocope readouts');

    % -------------------------------------------------------------------------
    % Accelerometer vs Classification Results
    figure
    subplot(2,1,1)
    plot( est.t(cal+1:length(est.t)),  v(cal+1:length(est.t), 1)); grid; hold on;
    plot( est.t(cal+1:length(est.t)),  v(cal+1:length(est.t), 2));
    plot( est.t(cal+1:length(est.t)),  v(cal+1:length(est.t), 3));
    ttlMsg = 'Accelerometer readouts vs Classification results';
    title(ttlMsg);
    xlabel('Time, s');
    ylabel('Acceleration, m/s^2');
    legend('x','y','z')

    subplot(2,1,2)
    plot( est.t(1:length(est.t)),  svm_results(8,1:length(est.t))); grid; hold on;
    plot( est.t(1:length(est.t)),  svm_results(9,1:length(est.t))); grid; hold on;
    area( est.t(1:length(est.t)),  9*ones(1,length(est.t))); grid; hold on;
    alpha(0.2)
    yticks([0:18])
    yticklabels({'ZUPT_{Still}','ZUPT_{Walk}','ZUPT_{Walk Fast}','ZUPT_{Jog}','ZUPT_{Run}',...
        'ZUPT_{Sprint}','ZUPT_{Walk Backward}','ZUPT_{Jog Backward}','ZUPT_{Side Step Right}',...
        'ZUPT_{Side Step Left}','Walk','Walk Fast','Jog','Run',...
        'Sprint','Walk Backward','Jog Backward','Side Step Right',...
        'Side Step Left'})
    legend('true', 'classified');
    ylim([0 18])
    figDspl = [figDspl, gcf];
    set( figDspl(end), 'Name', ttlMsg);

    % -------------------------------------------------------------------------
    % Classification Histogram
    figure
    histogram(svm_results(9,:))
    xticks([0:18])
    xticklabels({'ZUPT Still','ZUPT Walk','ZUPT Walk Fast','ZUPT Jog','ZUPT Run',...
        'ZUPT Sprint','ZUPT Walk Backward','ZUPT Jog Backward','ZUPT Side Step Right',...
        'ZUPT Side Step Left','Walk','Walk Fast','Jog','Run',...
        'Sprint','Walk Backward','Jog Backward','Side Step Right',...
        'Side Step Left'})
    xlabel('Activities');
    ylabel('Detected Samples');
    ttlMsg = 'Classification Histogram';
    title(ttlMsg);
    set(gcf, 'Name', ttlMsg);

    % -------------------------------------------------------------------------
    % Comparison of Accelerometer and Gyroscope with SHOE-SVM-ZUPT and ZUPT-SVM
    figure;
    % Top-left: Accelerometer vs SHOE-SVM-ZUPT
    subplot(2, 2, 1);
    plot(est.t(cal+1:end), v(cal+1:end, 1)); grid on; hold on;
    plot(est.t(cal+1:end), v(cal+1:end, 2));
    plot(est.t(cal+1:end), v(cal+1:end, 3));
    area(est.t(cal+1:end), (zupt_l(cal+1:end)) * 20, 'FaceAlpha', 0.3); 
    title('Accelerometer vs SHOE-SVM-ZUPT');
    xlabel('Time, s');
    ylabel('Normalized data, m/s^2');
    legend('accel x', 'accel y', 'accel z', 'SHOE-SVM-ZUPT flags');
    
    % Top-right: Gyroscope vs SHOE-SVM-ZUPT
    subplot(2, 2, 2);
    plot(est.t(cal+1:end), v(cal+1:end, 4)); grid on; hold on;
    plot(est.t(cal+1:end), v(cal+1:end, 5));
    plot(est.t(cal+1:end), v(cal+1:end, 6));
    area(est.t(cal+1:end), (zupt_l(cal+1:end)) * 5, 'FaceAlpha', 0.3); 
    title('Gyroscope vs SHOE-SVM-ZUPT');
    xlabel('Time, s');
    ylabel('Normalized data, dps');
    legend('gyro x', 'gyro y', 'gyro z', 'SHOE-SVM-ZUPT flags');
    
    % Bottom-left: Accelerometer vs ZUPT-SVM
    subplot(2, 2, 3);
    plot( est.t(cal+1:length(est.t)),  v(cal+1:length(est.t), 1)); grid; hold on;
    plot( est.t(cal+1:length(est.t)),  v(cal+1:length(est.t), 2));
    plot( est.t(cal+1:length(est.t)),  v(cal+1:length(est.t), 3));
    area( est.t(cal+1:length(est.t)),  (svm_results(9,cal+1:length(est.t))<=9)*20, 'FaceAlpha', 0.3); grid; hold on; 
    title('Accelerometer vs ZUPT-SVM');
    xlabel('Time, s');
    ylabel('Normalized data, m/s^2');
    legend('accel x', 'accel y', 'accel z', 'ZUPT-SVM flags');
    
    % Bottom-right: Gyroscope vs ZUPT-SVM
    subplot(2, 2, 4);
    plot( est.t(cal+1:length(est.t)),  v(cal+1:length(est.t), 4)); grid; hold on;
    plot( est.t(cal+1:length(est.t)),  v(cal+1:length(est.t), 5));
    plot( est.t(cal+1:length(est.t)),  v(cal+1:length(est.t), 6));
    area( est.t(cal+1:length(est.t)),  (svm_results(9,cal+1:length(est.t))<=9)*5, 'FaceAlpha', 0.3); grid; hold on; 
    title('Gyroscope vs ZUPT-SVM');
    xlabel('Time, s');
    ylabel('Normalized data, dps');
    legend('gyro x', 'gyro y', 'gyro z', 'ZUPT-SVM flags');
    
    % Adjust layout
    set(gcf, 'Name', 'Comparison of Accelerometer and Gyroscope with SHOE-SVM-ZUPT and ZUPT-SVM');
    sgtitle('Comparison of Accelerometer and Gyroscope with SHOE-SVM-ZUPT and ZUPT-SVM');
    
    % -------------------------------------------------------------------------
    % Roll, Pitch, Azimuth Estimates
    yL = {'Roll', 'Pitch', 'Azimuth'};
    figure
    for i = 1:3
        subplot(3, 1, i);
        plot(est.t(cal+1:end), est.rpyDeg_l(i, cal+1:end), 'b'); grid; hold on;
        if i == 1
            title('Roll, Pitch, Azimuth, deg.');
        end
        ylabel(yL{i});
    end
    xlabel('Time, s');
    figDspl = [figDspl, gcf];
    set(figDspl(end), 'Name', 'Roll, Pitch, Azimuth');

    % -------------------------------------------------------------------------
    % Roll, Pitch, Azimuth Errors
    yL = {'roll','pitch','azim.'};
    figure
    for i=1:3
        subplot(3,1,i);
        grid; hold on
        if dspl.enblCov
            plot(est.t(cal+1:length(est.t)),  sqrt(kf.diagP_l(i, cal+1:length(est.t)))*r2d*3, 'b');
            plot(est.t(cal+1:length(est.t)), -sqrt(kf.diagP_l(i, cal+1:length(est.t)))*r2d*3, 'b');
        end
        if i==1
            title('Error: Roll, Pitch, Azimuth, deg.');
        end
        ylabel(yL{i});
    end
    xlabel('Time, s');
    figDspl = [figDspl, gcf];
    set(figDspl(end), 'Name', 'Error: Roll, Pitch, Azimuth');

    % -------------------------------------------------------------------------
    % ZUPT State and Velocity NED
    yL = {'(1)', '(2)', '(3)'};
    figure
    subplot(4, 1, 1);
    plot(true.t(cal+1:end), zupt_l(cal+1:end), 'r'); hold on;
    
    title('ZUPT State');
    for i = 1:3
        subplot(4, 1, i+1);
        plot(est.t(cal+1:end), est.v_nWrtE_n_l(i, cal+1:end), 'b'); grid; hold on;
        if i == 1
            legend('Est left');
            title('Velocity NED, m/s');
        end
        ylabel(yL{i});
    end
    xlabel('Time, s');
    figDspl = [figDspl, gcf];
    set(figDspl(end), 'Name', 'ZUPT State and Velocity NED');

    % -------------------------------------------------------------------------
    % Velocity Error NED
    yL = {'(1)', '(2)', '(3)'};
    figure
    for i = 1:3
        subplot(3, 1, i);
        grid; hold on;
        if dspl.enblCov
            plot(est.t(cal+1:end), 3*sqrt(kf.diagP_l(i+3, cal+1:end)), 'b');
            plot(est.t(cal+1:end), -3*sqrt(kf.diagP_l(i+3, cal+1:end)), 'b');
        end
        if i == 1
            title('Error: Velocity NED, m/s');
        end
        ylabel(yL{i});
    end
    xlabel('Time, s');
    figDspl = [figDspl, gcf];
    set(figDspl(end), 'Name', 'Error: Velocity NED');

    % -------------------------------------------------------------------------
    % Velocity Correction NED
    yL = {'(1)', '(2)', '(3)'};
    figure
    for i = 1:3
        subplot(3, 1, i);
        plot(est.t(cal+1:end), kf.dx_l(3+i, cal+1:end), 'r'); grid;
        if i == 1
            title('Velocity Correction NED, m/s');
        end
        ylabel(yL{i});
    end
    xlabel('Time, s');
    figDspl = [figDspl, gcf];
    set(figDspl(end), 'Name', 'Velocity Correction NED');

    % -------------------------------------------------------------------------
    figure
    % left foot
    plot( est.Easting_l(cal+1:length(est.t)),  -est.Northing_l(cal+1:length(est.t)),'b','LineWidth',2); grid; hold on;
    hold on
    plot(est.Easting_l(cal+1), -est.Northing_l(cal+1), 's','LineWidth',2);hold on
    plot(est.Easting_l(length(est.t)), -est.Northing_l(length(est.t)), '^','LineWidth',2);hold on
    hold on
    xlabel('Easting, m');
    ylabel('Northing, m');
    legend('Path','Start','End')
    ttlMsg = 'Estimated Path, Northing-Easting, m';
    title(ttlMsg);
    figDspl = [figDspl, gcf];
    set( figDspl(end), 'Name', ttlMsg);

    % -------------------------------------------------------------------------
    figure
    plot3( est.Easting_l(cal+1:length(est.t)),  est.Northing_l(cal+1:length(est.t)),est.LLA_l(3,cal+1:length(est.t)), 'b'); grid; hold on;
    axis equal
    hold on
    plot3(est.Easting_l(cal+1), est.Northing_l(cal+1),est.LLA_l(3,cal+1), 's');
    plot3(est.Easting_l(length(est.t)), est.Northing_l(length(est.t)),est.LLA_l(3,length(est.t)), '*');
    
    xlabel('Easting, m');
    ylabel('Northing, m');
    zlabel('Down, m')
    ttlMsg = 'Estimated and True Path, Northing-Easting-Down, m';
    title(ttlMsg);
    figDspl = [figDspl, gcf];
    set( figDspl(end), 'Name', ttlMsg);
    saveas(gcf,[datasetDir '\' data_filename '_3d_path.fig'])    

    % -------------------------------------------------------------------------
    figure
    pos_temp = rotz(atan2(est.Easting_l(10*IMU_dt),est.Northing_l(10*IMU_dt))*r2d)*[est.Easting_l(1:length(est.t));est.Northing_l(1:length(est.t));est.LLA_l(3,1:length(est.t)) ];
    hold on
    plot3(pos_temp(1,cal+1), pos_temp(2,cal+1),pos_temp(3,cal+1), 's');
    plot3(0, 87.8,0, 'r^'); 
    plot3( pos_temp(1,:),  pos_temp(2,:) , pos_temp(3,:), 'b'); hold on;
    plot3(pos_temp(1,length(est.t)), pos_temp(2,length(est.t)),pos_temp(3,length(est.t)), '*');  
    xlabel('Easting, m');
    ylabel('Northing, m');
    zlabel('Down, m')
    xlim([-2 2])
    legend('Start','Reference End','Path','End')
    ttlMsg = 'Estimated and True Path Aligned w/ North, Northing-Easting-Down, m';
    figDspl = [figDspl, gcf];
    set( figDspl(end), 'Name', ttlMsg);
    saveas(gcf,[datasetDir '\' data_filename '_2d_path_north_aligned.fig']) 
    disp(['Displacement Error at Destination: ', num2str(norm(pos_temp(:,end) - [0;42.6;0]))])

    % -------------------------------------------------------------------------
    yL = {'(1)','(2)','(3)'};
    figure
    subplot(4,1,1);
    plot( true.t(cal+1:length(est.t)),  zupt_l(cal+1:length(est.t)), 'r'); hold on; grid on;
    
    ttlMsg = 'ZUPT state';
    title(ttlMsg);
    for i=1:3
        subplot(4,1,i+1);
        if ( dspl.enblCov )
            plot( est.t(cal+1:length(est.t)),  3*sqrt(kf.diagP_l(i+6, cal+1:length(est.t))), 'b');
            grid on;
            hold on;
            plot( est.t(cal+1:length(est.t)), -3*sqrt(kf.diagP_l(i+6, cal+1:length(est.t))), 'b');
            
        end
        if (i==1)
            ttlMsg = 'Error: Position NED, m.';
            title(ttlMsg);
        end
        ylabel(yL{i});
    end
    xlabel('time, sec.')
    figDspl = [figDspl, gcf];
    set(figDspl(end), 'Name', 'Error: Position NED');

    % -------------------------------------------------------------------------
    % Gyro Bias Error
    yL = {'1','2','3'};
    figure
    for i=1:3
        subplot(3,1,i);
        plot( est.t,  3*sqrt(kf.diagP_l(i+9, 1:length(est.t))), 'r');grid;hold on
        plot( est.t,  -3*sqrt(kf.diagP_l(i+9, 1:length(est.t))), 'r');grid;hold on        
        if (i==1)
            ttlMsg = 'Error: Gyro bias, deg/h';
            title(ttlMsg);
        end
        ylabel(yL{i});
    end
    xlabel('Time, s')
    figDspl = [figDspl, gcf];
    set( figDspl(end), 'Name', ttlMsg);

    % -------------------------------------------------------------------------
    figure
    subplot(3,1,1)
    plot(est.t(1:end-W),log(abs(sum_IMU)));hold on
    plot(est.t(1:end-W),log(simdata.factor)*ones(1,length(est.t(1:end-W))),'Color',[0.5 0 0]);
    plot(est.t(1:end-W),zupt_threshold_hist,'Color',[0 0.5 0]);    
    ttlMsg = 'Stance phase Detection: statistic vs threshold';
    title(ttlMsg);
    legend('Log-Likelihood, Fixed threshold', 'SVM-Threshold')
    xlabel('time, s')
    ylabel('Log-Likelihood')
    subplot(3,1,2)
    area(est.t(1:end), zupt_l(1:length(est.t)));hold on
    area(est.t(1:end), svm_results(9,1:length(est.t))<=9);
    legend('SVM-Aided SHOE','Standalone SVM')
    alpha(0.2)
    title('ZUPT flags')
    xlabel('time, s')
    ylabel('flags')
    ylim([0 2])
    xlim([0 est.t(end)])
    subplot(3,1,3)
    plot(est.t(1:end-W), zupt_varinace_hist, 'r');
    title('ZUPT Measurements Variance')
    xlabel('time, s')
    ylabel('flags')
    ylim([0 0.1])
    figDspl = [figDspl, gcf];
    set( figDspl(end), 'Name', ttlMsg);       

    % -------------------------------------------------------------------------
    figure
    subplot(2,1,1)
    plot(est.t(1:end-W),log(abs(sum_IMU)));
    ttlMsg = 'Log-Likelihood stadistics vs Classification results';
    title(ttlMsg);
    xlabel('Time, s');
    ylabel('Acceleration, m/s^2');
    legend('x','y','z')
    subplot(2,1,2)
    plot( est.t(1:length(est.t)),  svm_results(8,1:length(est.t))); grid; hold on;
    plot( est.t(1:length(est.t)),  svm_results(9,1:length(est.t))); grid; hold on;
    area( est.t(1:length(est.t)),  9*ones(1,length(est.t))); grid; hold on;
    alpha(0.2)
    yticks([0:18])
    yticklabels({'ZUPT_{Still}','ZUPT_{Walk}','ZUPT_{Walk Fast}','ZUPT_{Jog}','ZUPT_{Run}',...
        'ZUPT_{Sprint}','ZUPT_{Walk Backward}','ZUPT_{Jog Backward}','ZUPT_{Side Step Right}',...
        'ZUPT_{Side Step Left}','Walk','Walk Fast','Jog','Run',...
        'Sprint','Walk Backward','Jog Backward','Side Step Right',...
        'Side Step Left'})
    legend('true', 'classified');
    ylim([0 18])
    figDspl = [figDspl, gcf];
    set(figDspl(end), 'Name', ttlMsg);

    % -------------------------------------------------------------------------
    figure
    plot(est.t(1:end-W),log(abs(sum_IMU)));hold on
    plot(est.t(1:end-W),log(simdata.factor)*ones(1,length(est.t(1:end-W))),'Color',[0.5 0 0]);
    plot(est.t(1:end-W),zupt_threshold_hist,'Color',[0 0.5 0]);    
    ttlMsg = 'ZUPT-SVM-SHOE Stance phase Detection: variance of thresholds';
    title(ttlMsg);
    xlabel('time, s')
    ylabel('Log-Likelihood')
    area(est.t(1:end), 8*zupt_l(1:length(est.t))); hold on
    alpha(0.2)
    legend('Log-Likelihood', 'Fixed threshold', 'SVM-Threshold','SVM-Aided SHOE')
    figDspl = [figDspl, gcf];
    set(figDspl(end), 'Name', 'ZUPT-SVM-SHOE thresholds and events');

    % -------------------------------------------------------------------------
    figure, 
    yyaxis left
    plot(est.t(1:end-W),log(abs(sum_IMU)),'linewidth',1.25, 'Color',[0.2 0.5 0.9 0.8]);
    ttlMsg = 'Log-Likelihood stadistics vs Classification results';
    title(ttlMsg);
    xlabel('Time, s');
    ylabel('Acceleration, m/s^2');

    yyaxis right
    plot( est.t(1:length(est.t)),  svm_results(8,1:length(est.t)), '-k'); grid; hold on;
    plot( est.t(1:length(est.t)),  svm_results(9,1:length(est.t)), '-k'); grid; hold on;
    area( est.t(1:length(est.t)),  9.5*ones(1,length(est.t))); grid; hold on;
    yline(1,'-.','Color','#DCDCDC')
    yline(2,'-.','Color','#DCDCDC')
    yline(3,'-.','Color','#DCDCDC')
    yline(4,'-.','Color','#DCDCDC')
    yline(5,'-.','Color','#DCDCDC')
    yline(6,'-.','Color','#DCDCDC')
    yline(7,'-.','Color','#DCDCDC')
    yline(8,'-.','Color','#DCDCDC')
    yline(9,'-.','Color','#DCDCDC')
    yline(10,'-.','Color','#DCDCDC')
    yline(11,'-.','Color','#DCDCDC')
    yline(12,'-.','Color','#DCDCDC')
    yline(13,'-.','Color','#DCDCDC')
    yline(14,'-.','Color','#DCDCDC')
    yline(15,'-.','Color','#DCDCDC')
    yline(16,'-.','Color','#DCDCDC')
    yline(17,'-.','Color','#DCDCDC')
    yline(18,'-.','Color','#DCDCDC')
    yline(19,'-.','Color','#DCDCDC')
    alpha(0.1)
    yticks([0:18])
    yticklabels({'ZUPT_{Still}','ZUPT_{Walk}','ZUPT_{Walk Fast}','ZUPT_{Jog}','ZUPT_{Run}',...
        'ZUPT_{Sprint}','ZUPT_{Walk Backward}','ZUPT_{Jog Backward}','ZUPT_{Side Step Right}',...
        'ZUPT_{Side Step Left}','Walk','Walk Fast','Jog','Run',...
        'Sprint','Walk Backward','Jog Backward','Side Step Right',...
        'Side Step Left'})
    ax = gca; % Get the current axis handle
    ax.YAxis(2).Color = 'k'; % Set the color to black
    legend('Log-Likelihood', 'Prediction', 'location','northwest');
    ylim([0 18])
    figDspl = [figDspl, gcf];
    set( figDspl(end), 'Name', ttlMsg, 'position',[10,10,1900,500]);
end

% display navigation results
disp(['Final position: (',num2str(est.Easting_l(length(est.t))),',',num2str(est.Northing_l(length(est.t))),',',...
    num2str(est.LLA_l(3,length(est.t))),')']);
deviat_d = sqrt(est.Easting_l(length(est.t))^2+est.Northing_l(length(est.t))^2+(est.LLA_l(3,length(est.t))-est.LLA_l(3,cal))^2);
disp(['Deviated distance: ',num2str(deviat_d)])

deviat_d_horizontal = sqrt((0-est.Easting_l(length(est.t)))^2+(87.6-est.Northing_l(length(est.t)))^2);
disp(['Deviated horizontal distance: ',num2str(deviat_d_horizontal)])
deviat_d_vertical = sqrt((est.LLA_l(3,length(est.t))-est.LLA_l(3,cal))^2);
disp(['Deviated vertical distance: ',num2str(deviat_d_vertical)])
disp(['3D final error: ',num2str(norm([est.Easting_l(length(est.t)) est.Northing_l(length(est.t)) est.LLA_l(3,length(est.t))]-[0 87.8 0]))])

% save navigation results
INS_info.IMU_readouts = u(1:6,2:end);
INS_info.ZUPT_flags = zupt_l;
INS_info.velocity = est.v_nWrtE_n_l;
INS_info.trajectory = [est.Northing_l;est.Easting_l;est.Down_l];
INS_info.heading = est.rpyDeg_l; % roll pitch yaw
INS_info.timestamp = est.t;
INS_info.covariance_mtx = kf.diagP_l;

save([datasetDir '\INS_info_' data_filename '.mat'],'INS_info');

close(progress_bar)