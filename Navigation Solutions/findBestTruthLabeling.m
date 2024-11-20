clear all
error_array = [];
% data_filename = 100:200:1700;
% GT_parameter = 1.2:0.2:10;
data_filename = 100;
GT_parameter = 1.2;

cut_time_cal = 1;
time_cal = 2;

% Chico's parameters (calibration) Per each experiment 
% cut_time_cal = [1016, 1, 744, 1, 220, 1, 1, 1, 1];
% time_cal = [3.5, 0.5, 1, 1, 1.25, 2, 2, 0.8 , 1];

% Austin's parameters (calibration) Per each experiment 
% cut_time_cal = [1, 1, 800, 2552, 1, 1, 1, 800, 1440];
% time_cal = [3, 2, 2.5, 2, 4.5, 4, 4, 5, 3];

% Eudald's parameters (calibration) Per each experiment 
% cut_time_cal = [1016, 1, 744, 1, 220, 1, 1, 1, 1];
% time_cal = [3.5, 0.5, 1, 1, 1.25, 2, 2, 0.8 , 1];

tic
for jjj = 1:length(data_filename)
for kkk = 1:length(GT_parameter)
% File Header
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Description:
%   This file implements ZUPT-aided Inertial Navigation Systems with
%   conventional ZUPT detection method
%
%   Algorithm used:
%
%   1. Inertial Navigation System for one foot
%   2. Zero Velocity Update
%   3. SVM classification
%
%   NOTES:
%   The following summarizes some stuffs to check before implementing this
%   code
%   1. Check the file name of left IMU and right IMU. Some IMU files in the
%   dataset folder has the wrong labels.
%   2. set the number of data for initial calibration in the variable 'cal'
%   3. Check the sampling rate of the system and time instances in
%   'u(11,:)' and 'true.t'
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Start of the code
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clc;
clearvars -except error_array GT_parameter data_filename kkk jjj cut_time_cal time_cal;
close all;

progress_bar = waitbar(0,['Dataset: ' num2str(data_filename(jjj)) ', parameter: ' num2str(GT_parameter(kkk))]);


currDir = pwd;

% datasetDir = [currDir, '\dataset','\2022_02_28'];
% datasetDir = [currDir, '\dataset','\2022_03_17'];
% datasetDir = [currDir, '\dataset','\2022_04_25'];
% datasetDir = [currDir, '\dataset','\2022_05_20'];
% datasetDir = [currDir, '\dataset','\2022_06_02'];   % IEEE Conference Paper

% Training SVM datasets:
datasetDir = [currDir, '\dataset','\2022_03_17'];       % Chico
% datasetDir = [currDir, '\dataset','\2022_07_27'];     % Austin
% datasetDir = [currDir, '\dataset','\2022_08_01'];     % Eudald

% datasetDir = [currDir, '\dataset','\2022_09_02'];     % HD


% data_filename(jjj) = '100'; % without .mat %203

yaw_Deg =  0;

addpath([currDir, '\lib\google map lib']);
addpath([currDir, '\lib\INS lib']);
addpath([currDir, '\lib\Stance Phase Detector lib']);
addpath([currDir, '\lib\Activities_Classification_lib']);

addpath(datasetDir);
load(['exp' num2str(data_filename(jjj)) '_VN.mat']);
% load([data_filename(jjj) '_SVM.mat']);
% load([data_filename(jjj) '_SVM_HD50.mat']);
% load([data_filename(jjj) '_SVM_HD100.mat']);
% load([data_filename(jjj) '_SVM_HD200.mat']);
% load(['exp' data_filename(jjj) '_SVM_HD400.mat']);

tableTemp = readtable(['y_' num2str(data_filename(jjj)) '_threshold_' num2str(GT_parameter(kkk)) '_df.csv']);
svm_results = [];
svm_results(9,:) = tableTemp.label';


svm_window = 400; % if the 50,100,200,400 - HD dataset is used, 0 otherwise (HD datasets align 20*IMU, if 0 - align 15*IMU)

% u = u(:,svm_window+1:end);

% % TODO add pedestrian activity fucntion
% pyversion;
% clf_filename = 'lib\Activities_Classification_lib\2022_03_17_&_04_25_clf_rbf_100_6sig.sav';
% % Filename is the name of the file.
% svm_classifier = py.open(clf_filename,'rb');
% svm_classifier = py.pickle.load(svm_classifier);

figDspl = [];
dspl.enblCov = 1;
s=settings_constructed_data();
global simdata;
d2r = pi/180;
r2d = 180/pi;

u = u(:,cut_time_cal(jjj)+50:end);
svm_results = svm_results(:,cut_time_cal(jjj):end);

IMU_dt = floor(mean(1./u(11,100:200)));

%% -------------------------------------------------------------------------
% -------------------------------------------------------------------------
%
%  Estimated Navigation Solution
%
% -------------------------------------------------------------------------
% -------------------------------------------------------------------------
m = 1;  % Number of averaging of raw data to adjust sampling rate.
cal = time_cal(jjj)*IMU_dt; % Number of initial time steps for calibration

standaloneSVMdetector_switch = 1; % switch to use ZUPT event determined by svm
svm_SHOEdetector_switch = 10; % swtich to use SVM-aided SHOE detector
stdsvm_SHOEdetector_switch = 10; % swtich to use STDSVM-aided SHOE detector

if stdsvm_SHOEdetector_switch==1
    w_cfm = load(['2022_04_25_cfm_rbf_C100_6sig_df_Chico_v2.csv']);
end

zupt = 1; % ZUPT switch 1 is on, 10 is off

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
% g = 9.817269086191379;
% g = 9.79607;

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
% yaw_Deg = simdata.init_heading * r2d;

q_b2n_0  = Imu2YPRdeg_to_qb2n(roll_Deg,pitch_Deg,yaw_Deg);

% calculating initial states for the EKF
latDeg = simdata.latitude * r2d;
lonDeg = simdata.longitude * r2d;
h = simdata.altitude;

V_0 = zeros(3, 1);
dt = simdata.Ts;
true.t = cumsum(u(11,2:end));
% true.t = u(8,2:end); % for VectorNav


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
%
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

% H_ZUPT = zeros(3,size(P,1)); % measurement matrix
% H_ZUPT(:,4:6) = eye(3,3);
H_ZUPT = zeros(3,size(P,1)); % measurement matrix
H_ZUPT(:,4:6) = eye(3,3);
H_ZART = zeros(3,size(P,1)); % measurement matrix
H_ZART(:,10:12) = eye(3,3);
%
R_ZUPT = diag(simdata.sigma_vel.^2); % measurement noise matrix
R_ZART = diag([0.1,0.1,0.000001].^2); % measurement noise matrix
stance_velocity = [0.0;0;0];

aB_l = zeros(3,1);
gB_l = mean(v(cal-200+1:cal, 4:6))';


dx_l = zeros(size(P,1),1);
Id = eye(size(P));
%
zupt_SHOE = zeros(1,n);  % ZUPTing marker predicted by SHOE detector
zupt_SHOE(1) = 1;  % ZUPTing marker
zupt_SVM = zeros(1,n);  % ZUPTing marker

T = zeros(1,n);   % Test statistics of ZUPTing detector
W = simdata.Window_size;  % ZUPTing window size
%
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
% tic
zupt_varinace_hist = [];
zupt_threshold_hist = [];

for i=2:cal
    %     i

    if mod(i,10000) == 0
        waitbar(floor(i/length(u)*100)/100,progress_bar,['Dataset: ' num2str(data_filename(jjj)) ', parameter: ' num2str(GT_parameter(kkk))]);
    end

    sensor.w_b2i_b = v(i, 4:6)' - gB_l;
    sensor.f_b     = v(i, 1:3)' - aB_l;

    if i < n - W + 2
        [sigma_zupt, threshold_zupt] = ZUPTwithMotionClassification(svm_results(9,i));
        zupt_varinace_hist(end+1) = sigma_zupt;
        zupt_threshold_hist(end+1) = threshold_zupt;
        %      [zupt(i:i+W-1)] = zero_velocity_detector(u(:, i:i+W-1), X(:, i));
        [zupt_SHOE(i:i+W-1),sum_IMU(end+1),sum_IMU_x(end+1),sum_IMU_y(end+1),sum_IMU_z(end+1)]  = SHOE_detector(v(i:i+W-1, :)');
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

    %     if (zupt_l(i) == zupt)

    z = [];
    H = [];
    R = [];
    if 1
        z = [z;input_cal.v_nWrtE_n];
        H = [H;H_ZUPT];
        R = diag([diag(R);diag(R_ZUPT)]);
    end

    % Zero Angular Rate update
    %     if 1
    %         z = [z;sensor.w_b2i_b];
    %         H = [H;H_ZART];
    %         R = diag([diag(R);diag(R_ZART)]);
    %     end
    if ~isempty(z)
        %
        S = (H*P*H'+R);
        K  = (P*H')/(H*P*H'+R);
        P  = (Id-K*H)*P;
        %
        dx_l = K*z;
        gB_l = gB_l + dx_l(10:12);
        aB_l = aB_l + dx_l(13:15);
        %
        %dx(7:9) = z;
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

% P = diag(x.^2);

for i=cal+1:length(est.t)
    %     i

    if mod(i,10000) == 0
        waitbar(floor(i/length(u)*100)/100,progress_bar,['Dataset: ' num2str(data_filename(jjj)) ', parameter: ' num2str(GT_parameter(kkk))]);
    end

    sensor.w_b2i_b = v(i, 4:6)' - gB_l;
    sensor.f_b     = v(i, 1:3)'  - aB_l;

    if i < n - W + 2
        if stdsvm_SHOEdetector_switch == 1
            [sigma_zupt, threshold_zupt] = STD_ZUPTwithMotionClassification(svm_results(9,i),w_cfm);
        else
            [sigma_zupt, threshold_zupt] = ZUPTwithMotionClassification(svm_results(9,i));
        end
        zupt_varinace_hist(end+1) = sigma_zupt;
        zupt_threshold_hist(end+1) = threshold_zupt;
        %      [zupt(i:i+W-1)] = zero_velocity_detector(u(:, i:i+W-1), X(:, i));
        if svm_SHOEdetector_switch == 1
            [zupt_SHOE(i:i+W-1),sum_IMU(end+1),sum_IMU_x(end+1),sum_IMU_y(end+1),sum_IMU_z(end+1)]  = SVM_SHOE_detector(v(i:i+W-1, :)',exp(threshold_zupt));
        else
            [zupt_SHOE(i:i+W-1),sum_IMU(end+1),sum_IMU_x(end+1),sum_IMU_y(end+1),sum_IMU_z(end+1)]  = SHOE_detector(v(i:i+W-1, :)');
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

%     zupt_SVM(i) = sum(svm_results(9,i:min(i+20,length(est.t))) <= 9)==21;
    zupt_SVM(i) =svm_results(9,i) <= 9;

    z = [];
    H = [];
    R = [];
    %     if ((zupt_l(i) == zupt)&&standaloneSVMdetector_switch~=1 ... % regular ZUPT condition
    %             || standaloneSVMdetector_switch==1 && (svm_results(9,i) <= 9)) % SHOE detection results
    if ((zupt_SHOE(i) == zupt)&&standaloneSVMdetector_switch~=1 ...
            || standaloneSVMdetector_switch==1 && zupt_SVM(i)) % SHOE detection results
        %     if (svm_results(9,i) <= 9) % SVM results
        %     if (svm_results(8,i) <= 9) % SVM true labeling

        z = [z;input.v_nWrtE_n - stance_velocity];
        H = [H;H_ZUPT];
        %         R = diag([diag(R);diag(R_ZUPT)]);
        R = diag([diag(R);(sigma_zupt)^2*[1;1;1]]);
    end
    if ~isempty(z)
        %
        S = (H*P*H'+R);
        K  = (P*H')/(H*P*H'+R);
        P  = (Id-K*H)*P;
        %
        dx_l = K*z;
        gB_l = gB_l + dx_l(10:12);
        aB_l = aB_l + dx_l(13:15);
        %
        %dx(7:9) = z;
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
%
[roll_deg_l, pitch_deg_l, yaw_deg_l] = qb2nImu2YPRdeg(est.q_b2n_l);
est.rpyDeg_l = [roll_deg_l; pitch_deg_l; yaw_deg_l];
%
est.rpyDeg_l(1,:) = wrapTo180(est.rpyDeg_l(1,:));
est.rpyDeg_l(2,:) = wrapTo180(est.rpyDeg_l(2,:));
est.rpyDeg_l(3,:) = wrapTo180(est.rpyDeg_l(3,:));
%
waitbar(1,progress_bar,'Ploting...');

%
% disp(['Computation Time: ' num2str(toc) ' s'])

%% -------------------------------------------------------------------------
% -------------------------------------------------------------------------
%
%  Results Plotting
%
% -------------------------------------------------------------------------
% -------------------------------------------------------------------------
if (0)
    figure
    plot( est.t(1:length(est.t)),  v(1:length(est.t), 1)); grid; hold on;
    plot( est.t(1:length(est.t)),  v(1:length(est.t), 2));
    plot( est.t(1:length(est.t)),  v(1:length(est.t), 3));
    ttlMsg = 'Accelerometer readouts';
    title(ttlMsg);
    xlabel('Time, s');
    ylabel('Acceleration, m/s^2');
    legend('1', '2', '3');
    figDspl = [figDspl, gcf];
    set( figDspl(end), 'Name', ttlMsg);
end
% -------------------------------------------------------------------------
if (0)
    figure
    plot( est.t(cal+1:length(est.t)),  v(cal+1:length(est.t), 4)*r2d); grid; hold on;
    plot( est.t(cal+1:length(est.t)),  v(cal+1:length(est.t), 5)*r2d);
    plot( est.t(cal+1:length(est.t)),  v(cal+1:length(est.t), 6)*r2d);
    ttlMsg = 'Gyroscope readouts';
    title(ttlMsg);
    xlabel('Time, s');
    ylabel('Angular rate, deg/s');
    legend('1', '2', '3');
    figDspl = [figDspl, gcf];
    set( figDspl(end), 'Name', ttlMsg);
end
% -------------------------------------------------------------------------
if (0)
    figure
    plot( est.t(cal+1:length(est.t)-1),  diff(v(cal+1:length(est.t), 1))); grid; hold on;
    plot( est.t(cal+1:length(est.t)-1),  diff(v(cal+1:length(est.t), 2)));
    plot( est.t(cal+1:length(est.t)-1),  diff(v(cal+1:length(est.t), 3)));
    ttlMsg = 'Accelerometer Diff. readouts';
    title(ttlMsg);
    xlabel('Time, s');
    ylabel('Acceleration, m/s^2');
    legend('1', '2', '3');
    figDspl = [figDspl, gcf];
    set( figDspl(end), 'Name', ttlMsg);
end
% -------------------------------------------------------------------------
if (0)
    figure
    plot( est.t(cal+1:length(est.t)-1),  diff(v(cal+1:length(est.t), 4)*r2d)); grid; hold on;
    plot( est.t(cal+1:length(est.t)-1),  diff(v(cal+1:length(est.t), 5)*r2d));
    plot( est.t(cal+1:length(est.t)-1),  diff(v(cal+1:length(est.t), 6)*r2d));
    ttlMsg = 'Gyroscope Diff. readouts';
    title(ttlMsg);
    xlabel('Time, s');
    ylabel('Angular rate, deg/s');
    legend('1', '2', '3');
    figDspl = [figDspl, gcf];
    set( figDspl(end), 'Name', ttlMsg);
end
% -------------------------------------------------------------------------
if (0)
    figure
    plot( est.t(cal+1:length(est.t)),  (v(cal+1:length(est.t), 4)-mean(v(cal+1:cal+400,4)))*r2d); grid; hold on;
    plot( est.t(cal+1:length(est.t)),  (v(cal+1:length(est.t), 5)-mean(v(cal+1:cal+400,5)))*r2d); grid; hold on;
    plot( est.t(cal+1:length(est.t)),  (v(cal+1:length(est.t), 6)-mean(v(cal+1:cal+400,6)))*r2d); grid; hold on;
    ttlMsg = 'Gyroscope readouts (removed initial biased)';
    title(ttlMsg);
    xlabel('Time, s');
    ylabel('Angular rate, deg/s');
    legend('1', '2', '3');
    figDspl = [figDspl, gcf];
    set( figDspl(end), 'Name', ttlMsg);
end
% -------------------------------------------------------------------------
if (0)
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
    %     yyaxis right
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
end
% -------------------------------------------------------------------------
if (0)
    figure;
    histogram(svm_results(9,:))
    xticks([0:18])
    xticklabels({'ZUPT_{Still}','ZUPT_{Walk}','ZUPT_{Walk Fast}','ZUPT_{Jog}','ZUPT_{Run}',...
        'ZUPT_{Sprint}','ZUPT_{Walk Backward}','ZUPT_{Jog Backward}','ZUPT_{Side Step Right}',...
        'ZUPT_{Side Step Left}','Walk','Walk Fast','Jog','Run',...
        'Sprint','Walk Backward','Jog Backward','Side Step Right',...
        'Side Step Left'})
    xlabel('Activties')
    ylabel('Detected Samples')
    ttlMsg = ['Calssification Histogram'];
    xlim([0 18])
    title(ttlMsg)
    figDspl = [figDspl, gcf];
    set( figDspl(end), 'Name', ttlMsg);
end
% -------------------------------------------------------------------------
if (0)
    figure
    plot( est.t(cal+1:length(est.t)),  v(cal+1:length(est.t), 1)); grid; hold on;
    plot( est.t(cal+1:length(est.t)),  v(cal+1:length(est.t), 2));
    plot( est.t(cal+1:length(est.t)),  v(cal+1:length(est.t), 3));
    plot( est.t(cal+1:length(est.t)),  vecnorm(v(cal+1:length(est.t), 1:3)'));

    ttlMsg = 'Accelerometer readouts';
    title(ttlMsg);
    xlabel('Time, s');
    ylabel('Acceleration, m/s^2');
    legend('x', 'y', 'z', 'magnitude');
    figDspl = [figDspl, gcf];
    set( figDspl(end), 'Name', ttlMsg);
end
% -------------------------------------------------------------------------
if (0)
    figure
    plot( est.t(cal+1:length(est.t)),  v(cal+1:length(est.t), 4)*r2d); grid; hold on;
    plot( est.t(cal+1:length(est.t)),  v(cal+1:length(est.t), 5)*r2d);
    plot( est.t(cal+1:length(est.t)),  v(cal+1:length(est.t), 6)*r2d);
    plot( est.t(cal+1:length(est.t)),  vecnorm(v(cal+1:length(est.t), 4:6)')*r2d);

    ttlMsg = 'Gyroscope readouts';
    title(ttlMsg);
    xlabel('Time, s');
    ylabel('Angular rate, deg/s');
    legend('x', 'y', 'z', 'magnitude');
    figDspl = [figDspl, gcf];
    set( figDspl(end), 'Name', ttlMsg);
end
% -------------------------------------------------------------------------
% plot accel and gyro together
if (0)
    figure
    yyaxis left
    plot( est.t(cal+1:length(est.t)),  v(cal+1:length(est.t), 1),'LineWidth',2); grid; hold on;
    plot( est.t(cal+1:length(est.t)),  v(cal+1:length(est.t), 2),'LineWidth',2);
    plot( est.t(cal+1:length(est.t)),  v(cal+1:length(est.t), 3),'LineWidth',2);

    xlabel('Time, s');
    ylabel('Acceleration, m/s^2');
    %     ylim([-100 40])
    yyaxis right
    plot( est.t(cal+1:length(est.t)),  v(cal+1:length(est.t), 4)*r2d,'LineWidth',2); grid; hold on;
    plot( est.t(cal+1:length(est.t)),  v(cal+1:length(est.t), 5)*r2d,'LineWidth',2);
    plot( est.t(cal+1:length(est.t)),  v(cal+1:length(est.t), 6)*r2d,'LineWidth',2);

    ttlMsg = 'Accelerometer and Gyroscope readouts';
    ylabel('Angular rate, deg/s');
    %     ylim([-250 500])

    legend('Accel x', 'Accel y', 'Accel z','Gyro x', 'Gyro y', 'Gyro z');
    title(ttlMsg);
    figDspl = [figDspl, gcf];
    set( figDspl(end), 'Name', ttlMsg);
end
% -------------------------------------------------------------------------
if (0)
    figure
    plot( est.t(cal+1:length(est.t)),  v(cal+1:length(est.t), 1)); grid; hold on;
    plot( est.t(cal+1:length(est.t)),  v(cal+1:length(est.t), 2));
    plot( est.t(cal+1:length(est.t)),  v(cal+1:length(est.t), 3));
    area( est.t(cal+1:length(est.t)),  (zupt_SHOE(cal+1:length(est.t)))*20); grid; hold on;
    alpha(.3)
    ttlMsg = 'Accelerometer vs SHOE-ZUPT';
    title(ttlMsg);
    xlabel('Time, s');
    ylabel('normalized data, m/s^2');
    legend('accel x', 'accel y', 'accel z', 'SHOE-ZUPT flags');
    figDspl = [figDspl, gcf];
    set( figDspl(end), 'Name', ttlMsg);
end
% -------------------------------------------------------------------------
% -------------------------------------------------------------------------
if (0)
    figure
    plot( est.t(cal+1:length(est.t)),  v(cal+1:length(est.t), 4)); grid; hold on;
    plot( est.t(cal+1:length(est.t)),  v(cal+1:length(est.t), 5));
    plot( est.t(cal+1:length(est.t)),  v(cal+1:length(est.t), 6));
    area( est.t(cal+1:length(est.t)),  (zupt_SHOE(cal+1:length(est.t)))*5); grid; hold on;
    alpha(.3)
    ttlMsg = 'Gyroscope vs SHOE-SVM-ZUPT';
    title(ttlMsg);
    xlabel('Time, s');
    ylabel('normalized data, dps');
    legend('gyro x', 'gyro y', 'gyro z', 'SHOE-SVM-ZUPT flags');
    figDspl = [figDspl, gcf];
    set( figDspl(end), 'Name', ttlMsg);
end
% -------------------------------------------------------------------------
if (0)
    figure
    plot( est.t(cal+1:length(est.t)),  v(cal+1:length(est.t), 1)); grid; hold on;
    plot( est.t(cal+1:length(est.t)),  v(cal+1:length(est.t), 2));
    plot( est.t(cal+1:length(est.t)),  v(cal+1:length(est.t), 3));
    area( est.t(cal+1:length(est.t)),  (svm_results(9,cal+1:length(est.t))<=9)*20); grid; hold on;
    alpha(.3)
    ttlMsg = 'Accelerometer vs SVM-ZUPT';
    title(ttlMsg);
    xlabel('Time, s');
    ylabel('normalized data, m/s^2');
    legend('accel x', 'accel y', 'accel z', 'SVM-ZUPT flags');
    figDspl = [figDspl, gcf];
    set( figDspl(end), 'Name', ttlMsg);
end
% -------------------------------------------------------------------------
if (0)
    figure
    plot( est.t(cal+1:length(est.t)),  v(cal+1:length(est.t), 4)); grid; hold on;
    plot( est.t(cal+1:length(est.t)),  v(cal+1:length(est.t), 5));
    plot( est.t(cal+1:length(est.t)),  v(cal+1:length(est.t), 6));
    area( est.t(cal+1:length(est.t)),  (svm_results(9,cal+1:length(est.t))<=9)*5); grid; hold on;
    alpha(.3)
    ttlMsg = 'Gyroscope vs SVM-ZUPT';
    title(ttlMsg);
    xlabel('Time, s');
    ylabel('normalized data, dps');
    legend('gyro x', 'gyro y', 'gyro z', 'SVM-ZUPT flags');
    figDspl = [figDspl, gcf];
    set( figDspl(end), 'Name', ttlMsg);
end
% -------------------------------------------------------------------------
if (0)
    yL = {'roll','pitch','azim.'};
    figure
    for i=1:3
        subplot(3,1,i);
        %        plot( est.t,  err.dthDeg_l(i,:), 'r');
        grid; hold on
        if ( dspl.enblCov )
            plot(est.t(cal+1:length(est.t)),  sqrt(kf.diagP_l(i, cal+1:length(est.t)))*r2d*3, 'b');
            plot(est.t(cal+1:length(est.t)), -sqrt(kf.diagP_l(i, cal+1:length(est.t)))*r2d*3, 'b');

        end
        if (i==1)
            ttlMsg = 'Error: Roll,Pitch,Azimuth, deg.';
            title(ttlMsg);
        end
        ylabel(yL{i});
    end
    xlabel('time, sec.')
    figDspl = [figDspl, gcf];
    set( figDspl(end), 'Name', ttlMsg);
end
% -------------------------------------------------------------------------
if (0)
    yL = {'roll','pitch','azim.'};
    figure
    for i=1:3
        subplot(3,1,i);
        %        plot( est.t,  err.dthDeg_l(i,:), 'r');
        grid; hold on
        if ( dspl.enblCov )
            plot(est.t(cal+1:length(est.t)),  sqrt(kf.diagP_l(i, cal+1:length(est.t)))*r2d*3, 'b');
            plot(est.t(cal+1:length(est.t)), -sqrt(kf.diagP_l(i, cal+1:length(est.t)))*r2d*3, 'b');

        end
        if (i==1)
            ttlMsg = 'Error: Roll,Pitch,Azimuth, deg.';
            title(ttlMsg);
        end
        ylabel(yL{i});
    end
    xlabel('time, sec.')
    figDspl = [figDspl, gcf];
    set( figDspl(end), 'Name', ttlMsg);
end
% -------------------------------------------------------------------------
if (0) % turn this on later
    yL = {'roll','pitch','azim.'};
    figure
    for i=1:3
        subplot(3,1,i);
        plot( est.t(cal+1:length(est.t)),   est.rpyDeg_l(i,cal+1:length(est.t)),  'b');grid; hold on;
        inBetween = [est.rpyDeg_l(i,cal+1:length(est.t)) + 3*sqrt(kf.diagP_l(i, cal+1:length(est.t))),...
            fliplr(est.rpyDeg_l(i,cal+1:length(est.t)) -3*sqrt(kf.diagP_l(i, cal+1:length(est.t))))];
        fill([est.t(cal+1:length(est.t)), fliplr(est.t(cal+1:length(est.t)))], inBetween, 'b--');
        alpha(0.2)
        %        plot( pred.t,  pred.rpyDeg_l(i,:), 'r--');

        if (i==1)
            %            legend('est','true');
            ttlMsg = 'Roll,Pitch,Azim., deg.';
            title(ttlMsg);
        end
        ylabel(yL{i});
        legend('Mean','3\sigma')
    end
    xlabel('time, sec.')
    figDspl = [figDspl, gcf];
    set( figDspl(end), 'Name', ttlMsg);
end
% -------------------------------------------------------------------------
if (0)
    yL = {'roll','pitch','azim.'};
    figure
    for i=1:3
        subplot(3,1,i);
        plot( est.t(cal+1:length(est.t)),   est.rpyDeg_l(i,cal+1:length(est.t)),  'b');grid; hold on;

        %        plot( pred.t,  pred.rpyDeg_l(i,:), 'r--');

        if (i==1)
            %            legend('est','true');
            ttlMsg = 'Roll,Pitch,Azim., deg.';
            title(ttlMsg);
        end
        ylabel(yL{i});
    end
    xlabel('time, sec.')
    figDspl = [figDspl, gcf];
    set( figDspl(end), 'Name', ttlMsg);
end
% -------------------------------------------------------------------------
if (0)
    yL = {'(1)','(2)','(3)'};
    figure
    subplot(3,1,1);
    %    plot( true.t,  zupt);
    %    ttlMsg = 'ZUPT state';
    %    title(ttlMsg);
    for i=1:3
        subplot(3,1,i);
        plot( est.t,    err.Vn_l(i,:),            'r');grid; hold on

        if ( dspl.enblCov )
            plot( est.t,  3*sqrt(kf.diagP_l(i+3, :)), 'b');
            plot( est.t, -3*sqrt(kf.diagP_l(i+3, :)), 'b');

        end
        if (i==1)
            ttlMsg = 'Error: Velocity NED, m/s.';
            title(ttlMsg);
        end
        ylabel(yL{i});
    end
    xlabel('time, sec.')
    figDspl = [figDspl, gcf];
    set( figDspl(end), 'Name', ttlMsg);
end
% -------------------------------------------------------------------------
if (0)
    yL = {'(1)','(2)','(3)'};
    figure
    subplot(4,1,1);
    plot( true.t(cal+1:length(est.t)),  zupt_SHOE(cal+1:length(est.t)), 'r'); hold on;

    ttlMsg = 'ZUPT state';
    title(ttlMsg);
    for i=1:3
        subplot(4,1,i+1);
        plot( est.t(cal+1:length(est.t)),   est.v_nWrtE_n_l(i,cal+1:length(est.t)),  'b');grid; hold on;


        if (i==1)
            legend('est left');
            ttlMsg = 'Velocity NED, m/s.';
            title(ttlMsg);
        end
        ylabel(yL{i});
    end
    xlabel('time, sec.')
    figDspl = [figDspl, gcf];
    set( figDspl(end), 'Name', ttlMsg);
end
% -------------------------------------------------------------------------
if (0) % turn this on later
    yL = {'(1)','(2)','(3)'};
    figure
    subplot(4,1,1);
    plot( true.t(cal+1:length(est.t)),  zupt_SHOE(cal+1:length(est.t)), 'r'); hold on;

    ttlMsg = 'ZUPT state';
    title(ttlMsg);
    for i=1:3
        subplot(4,1,i+1);
        plot( est.t(cal+1:length(est.t)),   est.v_nWrtE_n_l(i,cal+1:length(est.t)),  'b');grid; hold on;
        inBetween = [est.v_nWrtE_n_l(i,cal+1:length(est.t)) + 3*sqrt(kf.diagP_l(i+3, cal+1:length(est.t))),...
            fliplr(est.v_nWrtE_n_l(i,cal+1:length(est.t)) -3*sqrt(kf.diagP_l(i+3, cal+1:length(est.t))))];
        fill([est.t(cal+1:length(est.t)), fliplr(est.t(cal+1:length(est.t)))], inBetween, 'b--');
        alpha(0.2)

        if (i==1)
            legend('est left');
            ttlMsg = 'Velocity NED, m/s.';
            title(ttlMsg);
        end
        ylabel(yL{i});
        legend('Mean','3\sigma')
    end
    xlabel('time, sec.')
    figDspl = [figDspl, gcf];
    set( figDspl(end), 'Name', ttlMsg);
end
% -------------------------------------------------------------------------
if (0)
    yL = {'(1)','(2)','(3)'};
    figure
    for i=1:3
        subplot(3,1,i);
        plot( est.t(cal+1:length(est.t)), kf.dx_l(3+i,cal+1:length(est.t)), 'r');grid;

        if (i==1)
            ttlMsg = 'Velocity Correction NED, m/s.';
            title(ttlMsg);
        end
        ylabel(yL{i});
    end
    xlabel('time, sec.')
    figDspl = [figDspl, gcf];
    set( figDspl(end), 'Name', ttlMsg);
end
% -------------------------------------------------------------------------
if (0)
    yL = {'(1)','(2)','(3)'};
    figure
    subplot(3,1,1);
    %    plot( true.t,  zupt);
    %    ttlMsg = 'ZUPT state';
    %    title(ttlMsg);
    for i=1:3
        subplot(3,1,i);
        %        plot( est.t,    err.Vn_l(i,:),            'r');
        %        plot( est.t,    err.Vn_r(i,:),            'm');
        grid; hold on;
        if ( dspl.enblCov )
            plot( est.t(cal+1:length(est.t)),  3*sqrt(kf.diagP_l(i+3, cal+1:length(est.t))), 'b');
            plot( est.t(cal+1:length(est.t)), -3*sqrt(kf.diagP_l(i+3, cal+1:length(est.t))), 'b');
        end
        if (i==1)
            ttlMsg = 'Error: Velocity NED, m/s.';
            title(ttlMsg);
        end
        ylabel(yL{i});
    end
    xlabel('time, sec.')
    figDspl = [figDspl, gcf];
    set( figDspl(end), 'Name', ttlMsg);
end
%% -------------------------------------------------------------------------
if (0) % turn this on later
    figure

    %     plot( est.Easting_l(cal+1:length(est.t)) + 3*sqrt(kf.diagP_l(1+6, cal+1:length(est.t))), est.Northing_l(cal+1:length(est.t)) + 3*sqrt(kf.diagP_l(2+6, cal+1:length(est.t))), 'k--');
    %     plot( est.Easting_l(cal+1:length(est.t)) - 3*sqrt(kf.diagP_l(1+6, cal+1:length(est.t))), est.Northing_l(cal+1:length(est.t)) - 3*sqrt(kf.diagP_l(2+6, cal+1:length(est.t))), 'k--');
    x_cir_set = [];
    y_cir_set = [];
    for iii = cal+1:50:length(est.t)
        %         [x_cir,y_cir] = circle(est.Easting_l(iii),est.Northing_l(iii),3*sqrt(kf.diagP_l(1+6, iii)),30,0);
        [ell_x,ell_y] = ellipsePoints(3*sqrt(kf.diagP_l(2+6, iii)),3*sqrt(kf.diagP_l(1+6, iii)),0,est.Easting_l(iii),est.Northing_l(iii));
        %         if ~isempty(y_cir_set)
        %         in_minus = inpolygon(x_cir,y_cir,x_cir_set',y_cir_set');
        %         in_plus = inpolygon(x_cir_set,y_cir_set,x_cir',y_cir');
        %
        x_cir_set = [x_cir_set ell_x];
        y_cir_set = [y_cir_set ell_y];
        %         x_cir_set = [x_cir_set(~in_plus) x_cir(~in_minus)];
        %         y_cir_set = [y_cir_set(~in_plus) y_cir(~in_minus)];
        %         else
        %         x_cir_set = [x_cir];
        %         y_cir_set = [y_cir];
        %         end

        %         fill(ell_x, ell_y, 'b','EdgeColor','None');hold on;
        %         alpha(.1)
    end
    %     alpha(0.1);
    b_point_ind = boundary(x_cir_set',y_cir_set',1);
    fill(x_cir_set(b_point_ind), y_cir_set(b_point_ind), 'b--');hold on
    alpha(.1)

    plot( est.Easting_l(cal+1:length(est.t)),  est.Northing_l(cal+1:length(est.t)), 'b','Linewidth',2); grid; hold on;
    axis equal
    hold on
    plot(est.Easting_l(cal+1), est.Northing_l(cal+1), 's');
    plot(est.Easting_l(length(est.t)), est.Northing_l(length(est.t)), '*');

    %     b_point_ind = boundary(x_cir_set',y_cir_set',1);
    %         fill(x_cir_set(b_point_ind), y_cir_set(b_point_ind), 'b--');
    %     [cir_v,cir_i] = sort(x_cir_set);
    %         fill(x_cir_set(cir_i), y_cir_set(cir_i), 'b.');
    %
    %         alpha(.05)
    %     inBetween_x = [est.Easting_l(cal+1:length(est.t)) + 3*sqrt(kf.diagP_l(1+6, cal+1:length(est.t))),...
    %         fliplr(est.Easting_l(cal+1:length(est.t)) -3*sqrt(kf.diagP_l(1+6, cal+1:length(est.t))))];
    %     inBetween_y = [est.Northing_l(cal+1:length(est.t)) + 3*sqrt(kf.diagP_l(2+6, cal+1:length(est.t))),...
    %         fliplr(est.Northing_l(cal+1:length(est.t)) -3*sqrt(kf.diagP_l(2+6, cal+1:length(est.t))))];
    %     fill(inBetween_x, inBetween_y, 'b--');
    %     alpha(.05)
    xlabel('Easting, m');
    ylabel('Northing, m');
    legend('3\sigma','Path','Start','End')
    ttlMsg = 'Estimated and True Path, Northing-Easting, m';
    title(ttlMsg);
    figDspl = [figDspl, gcf];
    set( figDspl(end), 'Name', ttlMsg);
    saveas(gcf,[datasetDir '\' data_filename(jjj) '_2d_path.fig'])
end
%% -------------------------------------------------------------------------
if (0)
    yL = {'(1)','(2)','(3)'};
    figure
    subplot(4,1,1);
    plot( true.t(cal+1:length(est.t)),  zupt_SHOE(cal+1:length(est.t)), 'r'); hold on; grid on;

    ttlMsg = 'ZUPT state';
    title(ttlMsg);
    for i=1:3
        subplot(4,1,i+1);
        %        plot( est.t,    err.P_l(i,:),            'r');grid; hold on
        %        plot( est.t,    err.P_r(i,:),            'm');
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
    set( figDspl(end), 'Name', ttlMsg);
end
%% -------------------------------------------------------------------------
if (0)
    % zero_crossing = [0 diff(zupt_l)] == 1;
    zero_crossing = [0 diff(zupt_SHOE)] == -1;
    % zero_crossing = zupt_l==1;

    % [x_cir,y_cir] = circle(-0.205,-0.007,0.205,37,2);
    % -------------------------------------------------------------------------

    figure
    x_cir_set = [];
    y_cir_set = [];
    for iii = cal+1:50:length(est.t)
        %         [x_cir,y_cir] = circle(est.Easting_l(iii),est.Northing_l(iii),3*sqrt(kf.diagP_l(1+6, iii)),30,0);
        [ell_x,ell_y] = ellipsePoints(3*sqrt(kf.diagP_l(2+6, iii)),3*sqrt(kf.diagP_l(1+6, iii)),0,est.Easting_l(iii),est.Northing_l(iii));
        %         if ~isempty(y_cir_set)
        %         in_minus = inpolygon(x_cir,y_cir,x_cir_set',y_cir_set');
        %         in_plus = inpolygon(x_cir_set,y_cir_set,x_cir',y_cir');
        %
        x_cir_set = [x_cir_set ell_x];
        y_cir_set = [y_cir_set ell_y];
        %         x_cir_set = [x_cir_set(~in_plus) x_cir(~in_minus)];
        %         y_cir_set = [y_cir_set(~in_plus) y_cir(~in_minus)];
        %         else
        %         x_cir_set = [x_cir];
        %         y_cir_set = [y_cir];
        %         end

        %         fill(ell_x, ell_y, 'b','EdgeColor','None');hold on;
        %         alpha(.1)
    end
    %     alpha(0.1);
    b_point_ind = boundary(x_cir_set',y_cir_set',1);
    fill(x_cir_set(b_point_ind), y_cir_set(b_point_ind), 'b--');hold on
    alpha(.1)
    plot( est.Easting_l(cal+1:length(est.t)),  est.Northing_l(cal+1:length(est.t)), 'b'); grid; hold on;
    axis equal
    hold on
    plot(est.Easting_l(cal+1), est.Northing_l(cal+1), 's');
    plot(est.Easting_l(length(est.t)), est.Northing_l(length(est.t)), '*');
    plot( est.Easting_l(zero_crossing),  est.Northing_l(zero_crossing), 'rx','LineWidth',2); grid; hold on;
    %     plot(y_cir,x_cir,'gx','LineWidth',2)

    xlabel('Easting, m');
    ylabel('Northing, m');
    legend('3\sigma','Path','Start','End','Step')
    ttlMsg = 'Estimated and True Path w/Steps, Northing-Easting, m';
    title(ttlMsg);
    figDspl = [figDspl, gcf];
    set( figDspl(end), 'Name', ttlMsg);
    %     saveas(gcf,[datasetDir '\' data_filename(jjj) '_2d_path.fig'])

    %     step_error = vecnorm([[est.Easting_l(zero_crossing);est.Northing_l(zero_crossing)]-[y_cir(1:end-1);x_cir(1:end-1)]]);

    %     figure
    %     bar(step_error*100)
    %     xlabel('step #')
    %     ylabel('cm')
    %     title('Error Growth')
end

%% -------------------------------------------------------------------------
if (0)
    figure
    if 0
        %     I = imread('EG_flooplan.JPG');
        I = imread('EG_outside.JPG');
        %     xImg = linspace(min(est.Easting_l(cal+1:length(est.t))), max(est.Easting_l(cal+1:length(est.t))), size(I, 2));
        xImg = [-1.10881407928765,-1.03664501012641,-0.964475940965169,-0.892306871803930,-0.820137802642692,-0.747968733481454,-0.675799664320216,-0.603630595158977,-0.531461525997739,-0.459292456836501,-0.387123387675262,-0.314954318514024,-0.242785249352786,-0.170616180191548,-0.0984471110303093,-0.0262780418690711,0.0458910272921673,0.118060096453406,0.190229165614644,0.262398234775882,0.334567303937120,0.406736373098359,0.478905442259597,0.551074511420835,0.623243580582074,0.695412649743312,0.767581718904550,0.839750788065788,0.911919857227027,0.984088926388265,1.05625799554950,1.12842706471074,1.20059613387198,1.27276520303322,1.34493427219446,1.41710334135569,1.48927241051693,1.56144147967817,1.63361054883941,1.70577961800065,1.77794868716189,1.85011775632312,1.92228682548436,1.99445589464560,2.06662496380684,2.13879403296808,2.21096310212932,2.28313217129055,2.35530124045179,2.42747030961303,2.49963937877427,2.57180844793551,2.64397751709675,2.71614658625798,2.78831565541922,2.86048472458046,2.93265379374170,3.00482286290294,3.07699193206417,3.14916100122541,3.22133007038665,3.29349913954789,3.36566820870913,3.43783727787037,3.51000634703161,3.58217541619284,3.65434448535408,3.72651355451532,3.79868262367656,3.87085169283780,3.94302076199903,4.01518983116027,4.08735890032151,4.15952796948275,4.23169703864399,4.30386610780523,4.37603517696646,4.44820424612770,4.52037331528894,4.59254238445018,4.66471145361142,4.73688052277266,4.80904959193389,4.88121866109513,4.95338773025637,5.02555679941761,5.09772586857885,5.16989493774009,5.24206400690132,5.31423307606256,5.38640214522380,5.45857121438504,5.53074028354628,5.60290935270752,5.67507842186875,5.74724749102999,5.81941656019123,5.89158562935247,5.96375469851371,6.03592376767494,6.10809283683618,6.18026190599742,6.25243097515866,6.32460004431990,6.39676911348114,6.46893818264238,6.54110725180361,6.61327632096485,6.68544539012609,6.75761445928733,6.82978352844857,6.90195259760981,6.97412166677104,7.04629073593228,7.11845980509352,7.19062887425476,7.26279794341600,7.33496701257723,7.40713608173847,7.47930515089971,7.55147422006095,7.62364328922219,7.69581235838343,7.76798142754466,7.84015049670590,7.91231956586714,7.98448863502838,8.05665770418962,8.12882677335086,8.20099584251209,8.27316491167333,8.34533398083457,8.41750304999581,8.48967211915705,8.56184118831829,8.63401025747952,8.70617932664076,8.77834839580200,8.85051746496324,8.92268653412448,8.99485560328571,9.06702467244695,9.13919374160819,9.21136281076943,9.28353187993067,9.35570094909191,9.42787001825314,9.50003908741438,9.57220815657562,9.64437722573686,9.71654629489810,9.78871536405934,9.86088443322058,9.93305350238181,10.0052225715431,10.0773916407043,10.1495607098655,10.2217297790268,10.2938988481880,10.3660679173492,10.4382369865105,10.5104060556717,10.5825751248330,10.6547441939942,10.7269132631554,10.7990823323167,10.8712514014779,10.9434204706391,11.0155895398004,11.0877586089616,11.1599276781229,11.2320967472841,11.3042658164453,11.3764348856066,11.4486039547678,11.5207730239291,11.5929420930903,11.6651111622515,11.7372802314128,11.8094493005740,11.8816183697352,11.9537874388965,12.0259565080577,12.0981255772190,12.1702946463802,12.2424637155414,12.3146327847027,12.3868018538639,12.4589709230252,12.5311399921864,12.6033090613476,12.6754781305089,12.7476471996701,12.8198162688313,12.8919853379926,12.9641544071538,13.0363234763151,13.1084925454763,13.1806616146375,13.2528306837988,13.3249997529600,13.3971688221213,13.4693378912825,13.5415069604437,13.6136760296050,13.6858450987662,13.7580141679274,13.8301832370887,13.9023523062499,13.9745213754112,14.0466904445724,14.1188595137336,14.1910285828949,14.2631976520561,14.3353667212173,14.4075357903786,14.4797048595398,14.5518739287011,14.6240429978623,14.6962120670235,14.7683811361848,14.8405502053460,14.9127192745073,14.9848883436685,15.0570574128297,15.1292264819910,15.2013955511522,15.2735646203134,15.3457336894747,15.4179027586359,15.4900718277972,15.5622408969584];
        %
        %     yImg = linspace(min(est.Northing_l(cal+1:length(est.t))), max(est.Northing_l(cal+1:length(est.t))), size(I, 1));
        yImg = [-1.77807790791018e-06,0.0769139955922056,0.153829769262319,0.230745542932433,0.307661316602546,0.384577090272660,0.461492863942773,0.538408637612887,0.615324411283000,0.692240184953114,0.769155958623227,0.846071732293341,0.922987505963454,0.999903279633568,1.07681905330368,1.15373482697379,1.23065060064391,1.30756637431402,1.38448214798414,1.46139792165425,1.53831369532436,1.61522946899448,1.69214524266459,1.76906101633470,1.84597679000482,1.92289256367493,1.99980833734504,2.07672411101516,2.15363988468527,2.23055565835538,2.30747143202550,2.38438720569561,2.46130297936572,2.53821875303584,2.61513452670595,2.69205030037606,2.76896607404618,2.84588184771629,2.92279762138641,2.99971339505652,3.07662916872663,3.15354494239675,3.23046071606686,3.30737648973697,3.38429226340709,3.46120803707720,3.53812381074731,3.61503958441743,3.69195535808754,3.76887113175765,3.84578690542777,3.92270267909788,3.99961845276799,4.07653422643811,4.15345000010822,4.23036577377834,4.30728154744845,4.38419732111856,4.46111309478868,4.53802886845879,4.61494464212890,4.69186041579902,4.76877618946913,4.84569196313924,4.92260773680936,4.99952351047947,5.07643928414958,5.15335505781970,5.23027083148981,5.30718660515993,5.38410237883004,5.46101815250015,5.53793392617026,5.61484969984038,5.69176547351049,5.76868124718061,5.84559702085072,5.92251279452083,5.99942856819095,6.07634434186106,6.15326011553117,6.23017588920129,6.30709166287140,6.38400743654151,6.46092321021163,6.53783898388174,6.61475475755185,6.69167053122197,6.76858630489208,6.84550207856220,6.92241785223231,6.99933362590242,7.07624939957253,7.15316517324265,7.23008094691276,7.30699672058288,7.38391249425299,7.46082826792310,7.53774404159322,7.61465981526333,7.69157558893344,7.76849136260356,7.84540713627367,7.92232290994378,7.99923868361390,8.07615445728401,8.15307023095413,8.22998600462424,8.30690177829435,8.38381755196447,8.46073332563458,8.53764909930469,8.61456487297481,8.69148064664492,8.76839642031503,8.84531219398515,8.92222796765526,8.99914374132537,9.07605951499549,9.15297528866560,9.22989106233571,9.30680683600583,9.38372260967594,9.46063838334606,9.53755415701617,9.61446993068628,9.69138570435639,9.76830147802651,9.84521725169662,9.92213302536674,9.99904879903685,10.0759645727070,10.1528803463771,10.2297961200472,10.3067118937173,10.3836276673874,10.4605434410575,10.5374592147276,10.6143749883978,10.6912907620679,10.7682065357380,10.8451223094081,10.9220380830782,10.9989538567483,11.0758696304184,11.1527854040886,11.2297011777587,11.3066169514288,11.3835327250989,11.4604484987690,11.5373642724391,11.6142800461092,11.6911958197793,11.7681115934495,11.8450273671196,11.9219431407897,11.9988589144598,12.0757746881299,12.1526904618000,12.2296062354701,12.3065220091403,12.3834377828104,12.4603535564805,12.5372693301506,12.6141851038207,12.6911008774908,12.7680166511609,12.8449324248310,12.9218481985012,12.9987639721713,13.0756797458414,13.1525955195115,13.2295112931816,13.3064270668517,13.3833428405218,13.4602586141920,13.5371743878621,13.6140901615322,13.6910059352023,13.7679217088724,13.8448374825425,13.9217532562126,13.9986690298828,14.0755848035529,14.1525005772230,14.2294163508931,14.3063321245632,14.3832478982333,14.4601636719034,14.5370794455735,14.6139952192437,14.6909109929138,14.7678267665839,14.8447425402540,14.9216583139241,14.9985740875942,15.0754898612643,15.1524056349345,15.2293214086046,15.3062371822747,15.3831529559448,15.4600687296149,15.5369845032850,15.6139002769551,15.6908160506252,15.7677318242954,15.8446475979655,15.9215633716356,15.9984791453057,16.0753949189758,16.1523106926459,16.2292264663160,16.3061422399862,16.3830580136563,16.4599737873264,16.5368895609965,16.6138053346666,16.6907211083367,16.7676368820068,16.8445526556770,16.9214684293471,16.9983842030172,17.0752999766873,17.1522157503574,17.2291315240275,17.3060472976976,17.3829630713677,17.4598788450379,17.5367946187080,17.6137103923781,17.6906261660482,17.7675419397183,17.8444577133884,17.9213734870585,17.9982892607287,18.0752050343988,18.1521208080689,18.2290365817390,18.3059523554091,18.3828681290792,18.4597839027493,18.5366996764194,18.6136154500896,18.6905312237597,18.7674469974298,18.8443627710999,18.9212785447700,18.9981943184401,19.0751100921102,19.1520258657804,19.2289416394505,19.3058574131206,19.3827731867907,19.4596889604608,19.5366047341309,19.6135205078010,19.6904362814712,19.7673520551413,19.8442678288114,19.9211836024815,19.9980993761516,20.0750151498217,20.1519309234918,20.2288466971619,20.3057624708321,20.3826782445022,20.4595940181723,20.5365097918424,20.6134255655125,20.6903413391826,20.7672571128527,20.8441728865229,20.9210886601930,20.9980044338631,21.0749202075332,21.1518359812033,21.2287517548734,21.3056675285435,21.3825833022136,21.4594990758838,21.5364148495539,21.6133306232240,21.6902463968941,21.7671621705642,21.8440779442343,21.9209937179044,21.9979094915746,22.0748252652447,22.1517410389148,22.2286568125849,22.3055725862550,22.3824883599251,22.4594041335952,22.5363199072653,22.6132356809355,22.6901514546056,22.7670672282757,22.8439830019458,22.9208987756159,22.9978145492860,23.0747303229561,23.1516460966263,23.2285618702964,23.3054776439665,23.3823934176366,23.4593091913067,23.5362249649768,23.6131407386469,23.6900565123171,23.7669722859872,23.8438880596573,23.9208038333274,23.9977196069975,24.0746353806676,24.1515511543377,24.2284669280078,24.3053827016780,24.3822984753481,24.4592142490182,24.5361300226883,24.6130457963584,24.6899615700285,24.7668773436986,24.8437931173688,24.9207088910389,24.9976246647090,25.0745404383791,25.1514562120492,25.2283719857193,25.3052877593894,25.3822035330596,25.4591193067297,25.5360350803998,25.6129508540699,25.6898666277400,25.7667824014101,25.8436981750802,25.9206139487503,25.9975297224205,26.0744454960906,26.1513612697607,26.2282770434308,26.3051928171009,26.3821085907710,26.4590243644411,26.5359401381113,26.6128559117814,26.6897716854515,26.7666874591216,26.8436032327917,26.9205190064618,26.9974347801319,27.0743505538020,27.1512663274722,27.2281821011423,27.3050978748124,27.3820136484825,27.4589294221526,27.5358451958227,27.6127609694928,27.6896767431630,27.7665925168331,27.8435082905032,27.9204240641733,27.9973398378434,28.0742556115135,28.1511713851836,28.2280871588538,28.3050029325239,28.3819187061940,28.4588344798641,28.5357502535342,28.6126660272043,28.6895818008744,28.7664975745445,28.8434133482147,28.9203291218848,28.9972448955549,29.0741606692250,29.1510764428951,29.2279922165652,29.3049079902353,29.3818237639055,29.4587395375756,29.5356553112457,29.6125710849158,29.6894868585859,29.7664026322560,29.8433184059261,29.9202341795962,29.9971499532664,30.0740657269365,30.1509815006066,30.2278972742767,30.3048130479468,30.3817288216169,30.4586445952870,30.5355603689572,30.6124761426273,30.6893919162974,30.7663076899675,30.8432234636376,30.9201392373077,30.9970550109778,31.0739707846480,31.1508865583181,31.2278023319882,31.3047181056583,31.3816338793284,31.4585496529985,31.5354654266686,31.6123812003387,31.6892969740089,31.7662127476790,31.8431285213491,31.9200442950192,31.9969600686893,32.0738758423594,32.1507916160295,32.2277073896996,32.3046231633698,32.3815389370399,32.4584547107100,32.5353704843801,32.6122862580502,32.6892020317203,32.7661178053904,32.8430335790606,32.9199493527307,32.9968651264008,33.0737809000709,33.1506966737410,33.2276124474111,33.3045282210812,33.3814439947514,33.4583597684215];
        %     image(xImg*2.3-20-5+1.8, yImg*2.3-26.2-13.5+5-0.2, I, 'CDataMapping', 'scaled'); % for Oct16th data set zupt = exp(7.2)
        image(xImg*5-21, yImg*5-50.5, I, 'CDataMapping', 'scaled'); % for Oct16th data set zupt = exp(7.2)
        %     image(xImg*2-20-5+1.8+3.5, yImg*2-26.2-13.5+5-0.2+29, I, 'CDataMapping', 'scaled'); % for Oct16th data set
        hold on;
    end
    % left foot
    plot( est.Easting_l(cal+1:length(est.t)),  -est.Northing_l(cal+1:length(est.t)),'b','LineWidth',2); grid; hold on;
    %     axis equal
    hold on
    plot(est.Easting_l(cal+1), -est.Northing_l(cal+1), 's','LineWidth',2);hold on
    plot(est.Easting_l(length(est.t)), -est.Northing_l(length(est.t)), '^','LineWidth',2);hold on
    %     axis equal
    hold on
    %     legend('Left','Right')
    %     legend('Path(R)','Path(L)','Start(R)','End(R)','Start(L)','End(L)')
    xlabel('Easting, m');
    ylabel('Northing, m');
    %     ttlMsg = 'Estimated and True Path, Northing-Easting, m';
    legend('Path','Start','End')
    ttlMsg = 'Estimated Path, Northing-Easting, m';
    title(ttlMsg);
    figDspl = [figDspl, gcf];
    set( figDspl(end), 'Name', ttlMsg);
    %     xlim([-25 15])
    %     ylim([-5 60])
    %     xlim([-25 15])
    %     ylim([-5 40])
end
% -------------------------------------------------------------------------
if(0)
    [lon_axis, lat_axis, Map, MapColor]  = get_google_map_LLA(simdata.latitude*r2d, simdata.longitude*r2d);
    % GoogleMapAPIMap = 'AIzaSyAtMjz5aWdPdiE0Q9-_LVWTEEi3YdObyO8';
    % [lon_axis, lat_axis, Map] = plot_google_map('Scale',2,'Resize',2,'MapType','roadmap','Alpha',1,'APIKey',GoogleMapAPIMap)
    % [lon_XX, lat_YY, M, Mcolor] = get_google_map_LLA(lat, lon, varargin)
    ColorMap = [];

    for ii = 1:3
        for kk = 1:length(Map)
            for jj = 1:length(Map)
                ColorMap(jj,kk,ii) = MapColor(Map(jj,kk)+1,ii);
            end
        end
    end


    % s = surface(XX,YY,M,ColorMap)
    % s.EdgeColor = 'none';
    % s = image(XX,YY,ColorMap)
    figure
    s = image(lon_axis,lat_axis,ColorMap);hold on
    xlabel('Longitude (^o)')
    ylabel('Lattitude (^o)')
    plot(est.LLA_l(1,:)*r2d,est.LLA_l(2,:)*r2d,'r');
    ttlMsg = 'Estimated Path, LLA, with Map m';
    title(ttlMsg);
    figDspl = [figDspl, gcf];
    set( figDspl(end), 'Name', ttlMsg);
end

%%
% -------------------------------------------------------------------------
if (0)
    figure
    plot3( est.Easting_l(cal+1:length(est.t)),  est.Northing_l(cal+1:length(est.t)),est.LLA_l(3,cal+1:length(est.t)), 'b'); grid; hold on;
    axis equal
    hold on
    plot3(est.Easting_l(cal+1), est.Northing_l(cal+1),est.LLA_l(3,cal+1), 's');
    plot3(est.Easting_l(length(est.t)), est.Northing_l(length(est.t)),est.LLA_l(3,length(est.t)), '*');

    xlabel('Easting, m');
    ylabel('Northing, m');
    zlabel('Down, m')
    %     zlim([5 20])
    ttlMsg = 'Estimated and True Path, Northing-Easting-Down, m';
    title(ttlMsg);
    figDspl = [figDspl, gcf];
    set( figDspl(end), 'Name', ttlMsg);
    saveas(gcf,[datasetDir '\' data_filename(jjj) '_3d_path.fig'])
end
% -------------------------------------------------------------------------
if (0) % anamation
    figure
    cur_plot = plot3( est.Easting_l(cal+1),  est.Northing_l(cal+1),est.LLA_l(3,cal+1), 'b'); grid; hold on;
    axis equal
    hold on
    cur_point = plot3(est.Easting_l(cal+1), est.Northing_l(cal+1),est.LLA_l(3,cal+1), 's');
    xlabel('Easting, m');
    ylabel('Northing, m');
    zlabel('Down, m')
    %     zlim([5 20])
    ttlMsg = 'Estimated and True Path, Northing-Easting-Down, m';
    title(ttlMsg);

    xlim([min(est.Easting_l) max(est.Easting_l)]);
    ylim([min(est.Northing_l) max(est.Northing_l)]);
    zlim([min(est.LLA_l(3,:)) max(est.LLA_l(3,:))]);
    for jj = cal+2:10:length(est.Easting_l)
        delete(cur_point);
        delete(cur_plot);
        cur_point = plot3(est.Easting_l(jj), est.Northing_l(jj),est.LLA_l(3,jj), 's');
        cur_plot = plot3( est.Easting_l(cal+1:jj),  est.Northing_l(cal+1:jj),est.LLA_l(3,cal+1:jj), 'b');
        drawnow;
    end
    figDspl = [figDspl, gcf];
    set( figDspl(end), 'Name', ttlMsg);
end
%% -------------------------------------------------------------------------
if (1)
    if svm_window ~= 0 % Initialization
%         pos_temp = rotz(atan2(est.Easting_l(20*IMU_dt),est.Northing_l(20*IMU_dt))*r2d)*[est.Easting_l(1:length(est.t));est.Northing_l(1:length(est.t));est.LLA_l(3,1:length(est.t)) ];
        pos_temp = rotz(atan2(est.Easting_l(floor(length(u)/2)),est.Northing_l(floor(length(u)/2)))*r2d)*[est.Easting_l(1:length(est.t));est.Northing_l(1:length(est.t));est.LLA_l(3,1:length(est.t)) ];        
    else
%         pos_temp = rotz(atan2(est.Easting_l(15*IMU_dt),est.Northing_l(15*IMU_dt))*r2d)*[est.Easting_l(1:length(est.t));est.Northing_l(1:length(est.t));est.LLA_l(3,1:length(est.t)) ];
        pos_temp = rotz(atan2(est.Easting_l(floor(length(u)/2)),est.Northing_l(floor(length(u)/2)))*r2d)*[est.Easting_l(1:length(est.t));est.Northing_l(1:length(est.t));est.LLA_l(3,1:length(est.t)) ];
    end
    disp(['Aligned Final Pos Error: ' num2str(norm(pos_temp(:,end) - [0 42.6 0]')) ' m']);
    error_array(jjj,kkk) = norm(pos_temp(:,end) - [0 42.6 0]');
    %     pos_temp = rotz(atan2(est.Easting_l(400),est.Northing_l(400))*r2d)*[est.Easting_l(1:length(est.t));est.Northing_l(1:length(est.t));est.LLA_l(3,1:length(est.t)) ];

    figure
    %     plot3( pos_temp(1,:),  pos_temp(2,:) , pos_temp(3,:), 'b'); hold on;
    %     axis equal
    hold on
    plot3(pos_temp(1,cal+1), pos_temp(2,cal+1),pos_temp(3,cal+1), 's');
    %     plot3(pos_temp(1,length(est.t)), pos_temp(2,length(est.t)),pos_temp(3,length(est.t)), '*');
    plot3(0, 42.6,0, 'r^');
    %
    plot3( pos_temp(1,:),  pos_temp(2,:) , pos_temp(3,:), 'b'); hold on;
    plot3(pos_temp(1,length(est.t)), pos_temp(2,length(est.t)),pos_temp(3,length(est.t)), '*');
    xlabel('Easting, m');
    ylabel('Northing, m');
    zlabel('Down, m')
    %         zlim([5 20])
    xlim([-2 2])
    %     legend('Path','Start','End','Reference End')
    legend('Start','Reference End','Path','End')
    ttlMsg = 'Estimated and True Path Aligned w/ North, Northing-Easting-Down, m';
    %     title(ttlMsg);
    figDspl = [figDspl, gcf];
    set( figDspl(end), 'Name', ttlMsg);
    saveas(gcf,[datasetDir '\' data_filename(jjj) '_2d_path_north_aligned.fig'])
    disp(['Displacement Error at Destination: ', num2str(norm(pos_temp(:,end) - [0;42.6;0]))])
end
%% -------------------------------------------------------------------------
if (0)
    pos_temp = rotz(atan2(est.Easting_l(20*IMU_dt),est.Northing_l(20*IMU_dt))*r2d)*[est.Easting_l(1:length(est.t));est.Northing_l(1:length(est.t));est.LLA_l(3,1:length(est.t)) ];
    %     pos_temp = rotz(atan2(est.Easting_l(400),est.Northing_l(400))*r2d)*[est.Easting_l(1:length(est.t));est.Northing_l(1:length(est.t));est.LLA_l(3,1:length(est.t)) ];

    figure
    plot( pos_temp(2,:),  pos_temp(1,:), 'b'); grid; hold on;
    axis equal
    hold on
    plot(pos_temp(2,cal+1), pos_temp(1,cal+1), 's');
    plot(pos_temp(2,length(est.t)), pos_temp(1,length(est.t)), '*');

    xlabel('Easting, m');
    ylabel('Northing, m');
    %     zlim([5 20])
    ttlMsg = 'Estimated and True Path Aligned w/ North, Northing-Easting, m';
    title(ttlMsg);
    figDspl = [figDspl, gcf];
    set( figDspl(end), 'Name', ttlMsg);
    saveas(gcf,[datasetDir '\' data_filename(jjj) '_2d_path_north_aligned.fig'])
end
% -------------------------------------------------------------------------
if (0)
    figure
    plot( est.t(cal+1:length(est.t)),  est.Northing_l(cal+1:length(est.t)), 'b'); grid; hold on;
    plot( est.t(cal+1:length(est.t)),  est.Easting_l(cal+1:length(est.t)), 'r');
    ttlMsg = 'Displacement along east and north';
    title(ttlMsg);
    xlabel('Time, s');
    ylabel('Displacement, m');
    legend('Northing', 'Easting');
end
% -------------------------------------------------------------------------
if (0)
    yL = {'(1)','(2)','(3)'};
    figure
    subplot(4,1,1);
    plot( true.t(cal+1:length(est.t)),  zupt_SHOE(cal+1:length(est.t)), 'r'); hold on; grid on;

    ttlMsg = 'ZUPT state';
    title(ttlMsg);
    for i=1:3
        subplot(4,1,i+1);
        %        plot( est.t,    err.P_l(i,:),            'r');grid; hold on
        %        plot( est.t,    err.P_r(i,:),            'm');
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
    set( figDspl(end), 'Name', ttlMsg);
end
% -------------------------------------------------------------------------
if (0)
    yL = {'(1)','(2)'};
    figure
    subplot(2,1,1);
    for i=1:2
        subplot(2,1,i);
        %        plot( est.t,    err.P_l(i,:),            'r');grid; hold on
        %        plot( est.t,    err.P_r(i,:),            'm');
        if ( dspl.enblCov )
            plot( est.t(cal+1:length(est.t))-12,  3*sqrt(kf.diagP_l(i+6, cal+1:length(est.t))), 'b');
            grid on;
            hold on;
            plot( est.t(cal+1:length(est.t))-12, -3*sqrt(kf.diagP_l(i+6, cal+1:length(est.t))), 'b');

        end
        if (i==1)
            ttlMsg = 'Error: Position NED, m.';
            title(ttlMsg);
        end
        ylabel(yL{i});
    end
    xlabel('time, sec.')
    figDspl = [figDspl, gcf];
    set( figDspl(end), 'Name', ttlMsg);
end
%% -------------------------------------------------------------------------
if (0)
    figure
    plot( est.t(cal+1:length(est.t)),  est.Northing_l(cal+1:length(est.t)), 'b','Linewidth',2); grid; hold on;
    %     x2 = [x, fliplr(x)];
    inBetween = [est.Northing_l(cal+1:length(est.t)) + 3*sqrt(kf.diagP_l(1+6, cal+1:length(est.t))),...
        fliplr(est.Northing_l(cal+1:length(est.t)) -3*sqrt(kf.diagP_l(1+6, cal+1:length(est.t))))];
    fill([est.t(cal+1:length(est.t)), fliplr(est.t(cal+1:length(est.t)))], inBetween, 'b--');
    %     area( est.t(cal+1:length(est.t)),  est.Northing_l(cal+1:length(est.t)) + 3*sqrt(kf.diagP_l(1+6, cal+1:length(est.t))),'FaceColor','r');
    %     area( est.t(cal+1:length(est.t)),  est.Northing_l(cal+1:length(est.t)) -3*sqrt(kf.diagP_l(1+6, cal+1:length(est.t))),'FaceColor','r');
    area( est.t(cal+1:length(est.t)),  (zupt_SHOE(cal+1:length(est.t)))*0.4); grid; hold on;
    alpha(.3)

    xlabel('time, sec');
    ylabel('North, m');
    ttlMsg = 'Estimated and True North motion, m';
    title(ttlMsg);
    figDspl = [figDspl, gcf];
    set( figDspl(end), 'Name', ttlMsg);
end
% -------------------------------------------------------------------------
if (0)
    figure
    plot( est.t(cal+1:length(est.t)),  est.Easting_l(cal+1:length(est.t)), 'b'); grid; hold on;
    inBetween = [est.Easting_l(cal+1:length(est.t)) + 3*sqrt(kf.diagP_l(2+6, cal+1:length(est.t))),...
        fliplr(est.Easting_l(cal+1:length(est.t)) -3*sqrt(kf.diagP_l(2+6, cal+1:length(est.t))))];
    fill([est.t(cal+1:length(est.t)), fliplr(est.t(cal+1:length(est.t)))], inBetween, 'b--');
    area( est.t(cal+1:length(est.t)),  (zupt_SHOE(cal+1:length(est.t)))*0.4); grid; hold on;
    alpha(.3)

    xlabel('time, sec');
    ylabel('East, m');
    ttlMsg = 'Estimated and True East motion, m';
    title(ttlMsg);
    figDspl = [figDspl, gcf];
    set( figDspl(end), 'Name', ttlMsg);
end
% -------------------------------------------------------------------------
if (0)
    figure
    plot( est.t(cal+1:length(est.t)),  est.LLA_l(3,cal+1:length(est.t))-simdata.altitude, 'b'); grid; hold on;
    inBetween = [est.LLA_l(3,cal+1:length(est.t))-simdata.altitude + 3*sqrt(kf.diagP_l(3+6, cal+1:length(est.t))),...
        fliplr(est.LLA_l(3,cal+1:length(est.t))-simdata.altitude -3*sqrt(kf.diagP_l(3+6, cal+1:length(est.t))))];
    fill([est.t(cal+1:length(est.t)), fliplr(est.t(cal+1:length(est.t)))], inBetween, 'b--');
    area( est.t(cal+1:length(est.t)),  (zupt_SHOE(cal+1:length(est.t)))*0.1); grid; hold on;
    alpha(.3)

    xlabel('time, sec');
    ylabel('Down, m');
    ttlMsg = 'Estimated and True Altitude, m';
    title(ttlMsg);
    figDspl = [figDspl, gcf];
    set( figDspl(end), 'Name', ttlMsg);
end
%% -------------------------------------------------------------------------
if (0)
    yL = {'N','E','D'};
    figure
    for i=1:3
        subplot(3,1,i);
        plot( est.t,  err.P_l(i,:), 'r');grid;hold on

        if ( dspl.enblCov )
            plot(est.t,  sqrt(kf.diagP_l(i+6, :))*3, 'b');
            plot(est.t, -sqrt(kf.diagP_l(i+6, :))*3, 'b');

        end
        if (i==1)
            ttlMsg = 'Error: Pos. NED, m.';
            title(ttlMsg);
        end
        ylabel(yL{i});
    end
    xlabel('time, sec.')
    figDspl = [figDspl, gcf];
    set( figDspl(end), 'Name', ttlMsg);
end
% -------------------------------------------------------------------------
if (0)
    yL = {'1','2','3'};
    figure
    for i=1:3
        subplot(3,1,i);
        plot( est.t,  kf.aB_l(i,:), 'r');grid;hold on

        if (i==1)
            ttlMsg = 'Accel bias, m/s^2';
            title(ttlMsg);
        end
        ylabel(yL{i});
    end
    xlabel('time, sec.')
    figDspl = [figDspl, gcf];
    set( figDspl(end), 'Name', ttlMsg);
end
% -------------------------------------------------------------------------
if (0) % turn this on later
    yL = {'1','2','3'};
    figure
    for i=1:3
        subplot(3,1,i);
        plot( est.t,  kf.aB_l(i,:), 'r');grid;hold on
        inBetween = [kf.aB_l(i,cal+1:length(est.t)) + 3*sqrt(kf.diagP_l(i+12, cal+1:length(est.t))),...
            fliplr(kf.aB_l(i,cal+1:length(est.t)) -3*sqrt(kf.diagP_l(i+12, cal+1:length(est.t))))];
        fill([est.t(cal+1:length(est.t)), fliplr(est.t(cal+1:length(est.t)))], inBetween, 'b--');
        alpha(0.2)
        if (i==1)
            ttlMsg = 'Accel bias, m/s^2';
            title(ttlMsg);
        end
        ylabel(yL{i});
    end
    xlabel('time, sec.')
    figDspl = [figDspl, gcf];
    set( figDspl(end), 'Name', ttlMsg);
end
% -------------------------------------------------------------------------
if (0) % turn this on later
    yL = {'1','2','3'};
    figure
    for i=1:3
        subplot(3,1,i);
        plot( est.t,  kf.gB_l(i,:) *180/pi*3600, 'r');grid;hold on
        inBetween = [kf.gB_l(i,cal+1:length(est.t))*180/pi*3600 + 3*sqrt(kf.diagP_l(i+9, cal+1:length(est.t))),...
            fliplr(kf.gB_l(i,cal+1:length(est.t))*180/pi*3600 -3*sqrt(kf.diagP_l(i+9, cal+1:length(est.t))))];
        fill([est.t(cal+1:length(est.t)), fliplr(est.t(cal+1:length(est.t)))], inBetween, 'b--');
        alpha(0.2)
        hold on;

        if (i==1)
            ttlMsg = 'Error: Gyro bias, deg/h';
            title(ttlMsg);
        end
        ylabel(yL{i});
    end
    xlabel('time, sec.')
    figDspl = [figDspl, gcf];
    set( figDspl(end), 'Name', ttlMsg);
end
% -------------------------------------------------------------------------
if (0) % turn this on later
    yL = {'1','2','3'};
    figure
    for i=1:3
        subplot(3,1,i);
        plot( est.t,  3*sqrt(kf.diagP_l(i+9, 1:length(est.t))), 'r');grid;hold on
        plot( est.t,  -3*sqrt(kf.diagP_l(i+9, 1:length(est.t))), 'r');grid;hold on

        %         inBetween = [kf.gB_l(i,cal+1:length(est.t))*180/pi*3600 + 3*sqrt(kf.diagP_l(i+9, cal+1:length(est.t))),...
        %         fliplr(kf.gB_l(i,cal+1:length(est.t))*180/pi*3600 -3*sqrt(kf.diagP_l(i+9, cal+1:length(est.t))))];
        %         fill([est.t(cal+1:length(est.t)), fliplr(est.t(cal+1:length(est.t)))], inBetween, 'b--');
        %         alpha(0.2)
        %         hold on;

        if (i==1)
            ttlMsg = 'Error: Gyro bias, deg/h';
            title(ttlMsg);
        end
        ylabel(yL{i});
    end
    xlabel('time, sec.')
    figDspl = [figDspl, gcf];
    set( figDspl(end), 'Name', ttlMsg);
end
% -------------------------------------------------------------------------
if (0)
    yL = {'1','2','3'};
    figure
    for i=1:3
        subplot(3,1,i);
        plot( est.t(2:end),  kf.gB_l(i,2:end) *180/pi*3600, 'r');grid;
        hold on;

        if (i==1)
            ttlMsg = 'Error: Gyro bias, deg/h';
            title(ttlMsg);
        end
        ylabel(yL{i});
    end
    xlabel('time, sec.')
    figDspl = [figDspl, gcf];
    set( figDspl(end), 'Name', ttlMsg);
end
% -------------------------------------------------------------------------
if (0)
    yL = {'(1)','(2)','(3)'};
    figure
    subplot(3,1,1);
    for i=1:3
        subplot(3,1,i);
        plot( pred.t,  gyro.Bias(i)*ones(1,length(pred.t))*180/pi*3600, 'r--');grid; hold on;
        plot( est.t,  kf.gB_l(i,:) *180/pi*3600, 'b');

        if (i==1)
            legend('true','est left');
            ttlMsg = 'Estimated Gyro bias, deg/h.';
            title(ttlMsg);
        end
        ylabel(yL{i});
    end
    xlabel('time, sec.')
    figDspl = [figDspl, gcf];
    set( figDspl(end), 'Name', ttlMsg);
end
% -------------------------------------------------------------------------
if (0)
    yL = {'(1)','(2)','(3)'};
    figure
    subplot(3,1,1);
    for i=1:3
        subplot(3,1,i);
        plot( pred.t,  accl.Bias(i)*ones(1,length(pred.t)), 'r--');grid; hold on;
        plot( est.t,  kf.aB_l(i,:), 'b');

        if (i==1)
            legend('true','est left');
            ttlMsg = 'Estimated Accel. bias, m/s^2';
            title(ttlMsg);
        end
        ylabel(yL{i});
    end
    xlabel('time, sec.')
    figDspl = [figDspl, gcf];
    set( figDspl(end), 'Name', ttlMsg);
end
% -------------------------------------------------------------------------
if (0)
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
    area(est.t(1:end), zupt_SHOE(1:length(est.t)));hold on
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
end

if 1
    figure;
    plot(est.t(1:end-W),log(abs(sum_IMU)));hold on
    % area(est.t(1:end), 10*(svm_results(9,1:length(est.t))<=9));
    area(est.t(1:end), 10*(zupt_SVM));
    legend('LR Stats','SVM','Smoothed SVM')
    alpha(0.2)
end
%
if 0
    figure
    plot(est.t(1:end-W),log(abs(sum_IMU)));hold on
    plot(est.t(1:end-W),log(simdata.factor)*ones(1,length(est.t(1:end-W))),'Color',[0.5 0 0]);
    plot(est.t(1:end-W),zupt_threshold_hist,'Color',[0 0.5 0]);
    ttlMsg = 'Stance phase Detection: statistic vs threshold';
    title(ttlMsg);
    xlabel('time, s')
    ylabel('Log-Likelihood')
    area(est.t(1:end), 8*zupt_SHOE(1:length(est.t)));hold on
    %     area(est.t(1:end), 8*(svm_results(9,1:length(est.t))<=9));
    alpha(0.2)
    legend('Log-Likelihood', 'Fixed threshold', 'SVM-Threshold','SVM-Aided SHOE')
    %     legend('Log-Likelihood', 'Fixed threshold', 'SVM-Threshold','SVM-Aided SHOE','Standalone SVM')
end

if 0

    % display navigation results
    disp(['Final position: (',num2str(est.Easting_l(length(est.t))),',',num2str(est.Northing_l(length(est.t))),',',...
        num2str(est.LLA_l(3,length(est.t))),')']);
    deviat_d = sqrt(est.Easting_l(length(est.t))^2+est.Northing_l(length(est.t))^2+(est.LLA_l(3,length(est.t))-est.LLA_l(3,cal))^2);
    disp(['Deivated distance: ',num2str(deviat_d)])

    deviat_d_horizontal = sqrt(est.Easting_l(length(est.t))^2+est.Northing_l(length(est.t))^2);
    disp(['Deivated horizontal distance: ',num2str(deviat_d_horizontal)])
    deviat_d_vertical = sqrt((est.LLA_l(3,length(est.t))-est.LLA_l(3,cal))^2);
    disp(['Deivated vertical distance: ',num2str(deviat_d_vertical)])
    disp(['3D final error: ',num2str(norm([est.Easting_l(length(est.t)) est.Northing_l(length(est.t)) est.LLA_l(3,length(est.t))]-[0 87.8 0]))])
end
% if UWB is included
if 0
    %     inter_agent_range = u_l(17,:);
    %     inter_agent_power = u_l(18,:);
    INS_info.inter_agent_range = -1000*ones(1,length(u(1,:)));
    INS_info.inter_agent_power = -1000*ones(1,length(u(1,:)));
end

if 0

    % save navigation results
    INS_info.IMU_readouts = u(1:6,2:end);
    INS_info.ZUPT_flags = zupt_SHOE;
    INS_info.velocity = est.v_nWrtE_n_l;
    INS_info.trajectory = [est.Northing_l;est.Easting_l;est.Down_l];
    INS_info.heading = est.rpyDeg_l; % roll pitch yaw
    INS_info.timestamp = est.t;
    INS_info.covariance_mtx = kf.diagP_l;

    save([datasetDir '\INS_info_' data_filename(jjj) '.mat'],'INS_info');
end
close(progress_bar)
end
end
disp(['Computation Time: ' num2str(toc) ' s'])

for hhh = 1:length(data_filename)
figure;plot(GT_parameter,error_array(hhh,:))
title(['exp' num2str(hhh)])
xlabel('GT Parameter')
ylabel('3D Error')
end