%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%> @file settings.m
%>
%> @brief Functions for setting all the parameters in the zero-velocity
%> aided inertial navigation system framework, as well as loading the IMU
%> data.
%>
%> @authors Isaac Skog, John-Olof Nilsson
%> @copyright Copyright (c) 2011 OpenShoe, ISC License (open source)
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%% MAIN FUNCTION


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%  funtion u=settings()
%
%> @brief Function that controls all the settings for the zero-velocity
%> aided inertial navigation system.
%>
%> @param[out]  u      Matrix with IMU data. Each column corresponds to one
%> sample instant. The data in each column is arranged as x, y, and z axis
%> specfic force components; x, y, and z axis angular rates.
%>
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function s=settings_constructed_data

s=0;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%               IMU Selection             %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% selecting what IMU is chosen
% 0 = MPU9250, 1 = VN-200, 2 = ADIS16485, 3 = ADIS16497-3, 4 = SmartBug
% 5 = simulated comsumer-grade device, 6 = simulated industrial-grade
% device, 7 = simulated tactical-grade device, 8 = simulated
% navigation-grade device, 9 = VN-100, 10 = ADIS16497-3 with ADIS_EVAL,
% 11 = Lab-On-Shoe 2.0, 12 = Sugar-Cube Lab (ICM-20948), 13 = Open Shoe (4xMPU-9150)
% other number = simulated device,
imu_id = 1;
if imu_id == 0
    disp('----MPU9250----')
elseif imu_id == 1
    disp('----VN-200----')
elseif imu_id == 2
    disp('----ADIS16485----')
elseif imu_id == 3
    disp('----ADIS16497-3 w/ Lab-On-Shoe----')
elseif imu_id == 4
    disp('----SmartBug----')
elseif imu_id == 5
    disp('----Simulated comsumer-grade device----')
elseif imu_id == 6
    disp('----Simulated industrial-grade device----')
elseif imu_id == 7
    disp('----Simulated tactical-grade device----')
elseif imu_id == 8
    disp('----Simulated navigation-grade device----')
elseif imu_id == 9
    disp('----VN-100----')
elseif imu_id == 10
    disp('----ADIS16497-3 w/ ADIS_EVAL----')
elseif imu_id == 11
    disp('----ADIS16497-3 w/ Lab-On-Shoe 2.0----')
elseif imu_id == 12
    disp('----Sugar-Cube Lab (ICM-20948)----')
elseif imu_id == 13
    disp('----Open Shoe (4xMPU-9150)----')    
else
    disp('----Simulated device----')
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%              GENERAL PARAMETERS         %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

global simdata;

simdata.c = 299792458; % speed of light, m/s

% Total time span [s]
simdata.timespan=3600;

% Initial altitude [m]
% simdata.altitude=17;
% simdata.altitude=41;
simdata.altitude=0; % Ali's Lab

% Initial latitude [rad]
% simdata.latitude=33.68*pi/180;
% simdata.latitude=33.6434*pi/180; % EG2110
% simdata.latitude=33.64299*pi/180; % Ali's Lab
% simdata.latitude=33.643241000000000*pi/180; % EG2110
% simdata.latitude=33.643394760082770*pi/180; % out side EG2110
% simdata.latitude=33.643622392103940*pi/180; % out side EG2110
% simdata.latitude = 33.643226147785670*pi/180;
simdata.latitude = 33.642576800000000*pi/180;
% simdata.latitude = 33.643444600000000*pi/180;

% Initial longitude [rad]
% simdata.longitude=-117.83*pi/180;
% simdata.longitude=-117.84*pi/180; % EG 2110
% simdata.longitude=-117.8396*pi/180; % Ali's Lab
% simdata.longitude=-1.178401030000000e+02*pi/180; % EG 2110
% simdata.longitude=-1.178404656391065e+02*pi/180; % outside EG 2110
% simdata.longitude=-1.178402183709588e+02*pi/180; % outside EG 2110
% simdata.longitude = -1.178401898366121e+02*pi/180;
simdata.longitude = -1.178447567000000e+02*pi/180;
% simdata.longitude = -1.178405249000000e+02*pi/180;


% Sampling period [s]
if imu_id == 0
    simdata.Ts=1/100; % for MPU-9250
elseif imu_id == 1
    simdata.Ts=1/800; % for VN-200
elseif imu_id == 2
    simdata.Ts=1/120; % for ADIS16485
elseif imu_id == 3
    simdata.Ts=1/120; % for Lab-On-Shoe
    %     simdata.Ts=1/1000; % for Lab-On-Shoe 2.0
elseif imu_id == 4
    %     simdata.Ts=1/2000; % for SmartBug
    simdata.Ts=1/100; % for SmartBug
    %     simdata.Ts=1/370; % for SmartBug
elseif imu_id == 5
    simdata.Ts=1/200; % for comsumer
elseif imu_id == 6
    simdata.Ts=1/200; % for industrial
elseif imu_id == 7
    simdata.Ts=1/200; % for tactical
elseif imu_id == 8
    simdata.Ts=1/200; % for navigation
elseif imu_id == 9
    simdata.Ts=1/400; % for VN-100
elseif imu_id == 10
    %     simdata.Ts=1/4250; % for ADIS_EVAL
    %     simdata.Ts=1/(4250/5); % for ADIS_EVAL
    simdata.Ts=1/(4250/5)*1; % for ADIS_EVAL
    
    %     simdata.Ts=1/(4250/4); % for ADIS_EVAL
    %         simdata.Ts=1/(4250/2); % for ADIS_EVAL
elseif imu_id == 11
    simdata.Ts=1/1000; % for Lab-On-Shoe 2.0
elseif imu_id == 12
    simdata.Ts=1/370; % for Sugar-Cube
%     simdata.Ts=1/90; % for Sugar-Cube    
elseif imu_id == 13
    simdata.Ts=1/300; % for OpenShoe
else
    %     simdata.Ts=1/200; % for Simulated device
    simdata.Ts = 1/800; % for Simulated device
end

disp(['|Sampling Freq = ',num2str(round(1/simdata.Ts)),'Hz |']);

% Number of iteration for DCM update
simdata.M=10;

% Time steps
simdata.N = round(simdata.timespan/simdata.Ts)+1;

% Initial orientatoin [rad] (North is zero degrees)
simdata.init_heading= 0*pi/180;

% Initial velocity (u, v, w)-axis [m/s]
simdata.init_vel=[0 0 0]';

% Earth rotation rate [rad/s]
simdata.earthrate=7.2921150e-5;

% Earth radius [m]
simdata.a=6378137;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%              ZUPTING SETUPS             %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Standard deviation of the acceleromter noise [m/s^2]. This is used to
% control the zero-velocity detectors trust in the accelerometer data.

if imu_id == 0
    simdata.sigma_a=300e-6*9.81*sqrt(simdata.Ts); % 300ug/sqrt(Hz) for MPU-9250
elseif imu_id == 1
    simdata.sigma_a=0.14e-3*9.81*sqrt(simdata.Ts) * 10; % 0.14mg/sqrt(Hz) for VN-200
elseif imu_id == 2
    simdata.sigma_a=0.023/60*sqrt(simdata.Ts) * 1; % 0.023m/s/rt(hr) for ADIS16485
elseif imu_id == 3
    simdata.sigma_a=0.04/60*sqrt(simdata.Ts) * 1; % 0.04m/s/rt(hr) ADIS16497 for Lab-On-Shoe
elseif imu_id == 4
    simdata.sigma_a=70e-6*9.81*sqrt(simdata.Ts) * 500; % 70ug/sqrt(Hz) for SmartBug
    %     simdata.sigma_a=0.04/60*sqrt(simdata.Ts) * 1; % 0.04m/s/rt(hr) ADIS16497 for Lab-On-Shoe
elseif imu_id == 5
    simdata.sigma_a=70e-6*9.81*sqrt(simdata.Ts) * 1; % for comsumer
elseif imu_id == 6
    simdata.sigma_a=7e-6*9.81*sqrt(simdata.Ts) * 1; % for industrial
elseif imu_id == 7
    simdata.sigma_a=0.7e-6*sqrt(simdata.Ts) * 1; % for tactical
elseif imu_id == 8
    simdata.sigma_a=0.07e-6*sqrt(simdata.Ts) * 1; % for navigation
elseif imu_id == 9
    simdata.sigma_a=0.14e-3*9.81*sqrt(simdata.Ts) * 1; % for VN-100
elseif imu_id == 10
    simdata.sigma_a=0.04/60*sqrt(simdata.Ts) * 50; % 0.04m/s/rt(hr) ADIS16497 for ADIS_EVAL
elseif imu_id == 11
    simdata.sigma_a=0.04/60*sqrt(simdata.Ts) * 50; % 0.04m/s/rt(hr) ADIS16497 for Lab-On-Shoe 2.0
elseif imu_id == 12
    simdata.sigma_a=0.04/60*sqrt(simdata.Ts) * 50; % 0.04m/s/rt(hr) for ICM-20948
elseif imu_id == 13
    simdata.sigma_a=0.14e-3*9.81*sqrt(simdata.Ts) * 500; % Open Shoe
else
    simdata.sigma_a=0.04/60*sqrt(simdata.Ts) * 1; % for Simulated device
end

% Standard deviation of the gyroscope noise [rad/s]. This is used to
% control the zero-velocity detectors trust in the gyroscope data.

if imu_id == 0
    simdata.sigma_g=0.01*pi/180*sqrt(simdata.Ts); % 0.01deg/s/sqrt(h) for MPU-9250
elseif imu_id == 1
    simdata.sigma_g=0.21 *pi/180/60*sqrt(simdata.Ts)*10; % 0.21deg/sqrt(h) for VN-200
elseif imu_id == 2
    simdata.sigma_g=0.3 *pi/180/60*sqrt(simdata.Ts)*1; % 0.3deg/sqrt(h)for ADIS16485
elseif imu_id == 3
    simdata.sigma_g=0.18*pi/180/60*sqrt(simdata.Ts)*1; % 0.18deg/sqrt(h) ADIS16497 for Lab-On-Shoe
elseif imu_id == 4
    simdata.sigma_g=0.0028*pi/180*sqrt(simdata.Ts)*10; % 0.0028deg/s/sqrt(h) for SmartBug
    %     simdata.sigma_g=0.18*pi/180/60*sqrt(simdata.Ts)*1; % 0.18deg/sqrt(h) ADIS16497 for Lab-On-Shoe
elseif imu_id == 5
    simdata.sigma_g=0.5*pi/180/60*sqrt(simdata.Ts); % > 0.5 degree/sqrt(h) for comsumer
elseif imu_id == 6
    simdata.sigma_g=0.5*pi/180/60*sqrt(simdata.Ts)*1; % 0.5 degree/sqrt(h) for industrial
elseif imu_id == 7
    simdata.sigma_g=0.05 *pi/180/60*sqrt(simdata.Ts)*1; % 0.5~0.05 degree/sqrt(h) for tactical
elseif imu_id == 8
    simdata.sigma_g=0.001*pi/180/60*sqrt(simdata.Ts)*1; % 0.001 degree/sqrt(h) for navigation
elseif imu_id == 9
    simdata.sigma_g=0.21 *pi/180/60*sqrt(simdata.Ts)*1; % 0.21deg/sqrt(h) for VN-100
elseif imu_id == 10
    simdata.sigma_g=0.18*pi/180/60*sqrt(simdata.Ts)*10; % 0.04m/s/rt(hr) ADIS16497 for ADIS_EVAL
elseif imu_id == 11
    simdata.sigma_g=0.18*pi/180/60*sqrt(simdata.Ts)*10; % 0.18deg/sqrt(h) ADIS16497 for Lab-On-Shoe 2.0
elseif imu_id == 12
    simdata.sigma_g=0.18*pi/180/60*sqrt(simdata.Ts)*10; % 0.18deg/sqrt(h) for ICM-20948
elseif imu_id == 13
    simdata.sigma_g=0.21 *pi/180/60*sqrt(simdata.Ts)*500; % Open Shoe 
else
    simdata.sigma_g=10/60*pi/180*sqrt(simdata.Ts)*1;  % 10 deg/sqrt(h) for Simulated device
end

% For ZUPT detector

simdata.gyro = simdata.sigma_g; % ARW
simdata.accel = simdata.sigma_a; % VRW

% Window size of the zero-velocity detector [samples]
% Sampling period [s]
zupt_window = 0.05;%seconds
% zupt_window = 0.5;%seconds for Knee mounted IMU

if imu_id == 0
    %     simdata.Window_size=5; % for MPU-9250
    simdata.Window_size=round(zupt_window/simdata.Ts);
    simdata.Window_size_for_step_detector = simdata.Window_size*5;
elseif imu_id == 1
    %     simdata.Window_size=5*20; % for VN-200
    %     simdata.Window_size=round(zupt_window/simdata.Ts);
    simdata.Window_size=round(zupt_window/simdata.Ts/1)*1;
    
    simdata.Window_size_for_step_detector = simdata.Window_size*5;
elseif imu_id == 2
    %     simdata.Window_size=5; % for ADIS16485
    simdata.Window_size=round(zupt_window/simdata.Ts);
    simdata.Window_size_for_step_detector = simdata.Window_size*5;
elseif imu_id == 3
    %     simdata.Window_size=5; % for Lab-On-Shoe
    simdata.Window_size=round(zupt_window/simdata.Ts);
    simdata.Window_size_for_step_detector = simdata.Window_size*5;
elseif imu_id == 4
    %     simdata.Window_size=5*16; % for SmartBug
    %         simdata.Window_size=20; % for SmartBug when FS = 100Hz
    %     simdata.Window_size=5; % for SmartBug when FS = 100Hz
    simdata.Window_size=round(zupt_window/simdata.Ts)*2;
    simdata.Window_size_for_step_detector = simdata.Window_size*5;
elseif imu_id == 5
    %     simdata.Window_size=5; % for comsumer
    %     simdata.Window_size=5*2; % for comsumer
    simdata.Window_size=round(zupt_window/simdata.Ts);
    simdata.Window_size_for_step_detector = simdata.Window_size*5;
elseif imu_id == 6
    %     simdata.Window_size=5*2; % for industrial
    simdata.Window_size=round(zupt_window/simdata.Ts);
    simdata.Window_size_for_step_detector = simdata.Window_size*5;
elseif imu_id == 7
    %     simdata.Window_size=5; % for tactical
    simdata.Window_size=round(zupt_window/simdata.Ts);
    simdata.Window_size_for_step_detector = simdata.Window_size*5;
elseif imu_id == 8
    %     simdata.Window_size=5; % for navigation
    simdata.Window_size=round(zupt_window/simdata.Ts);
    simdata.Window_size_for_step_detector = simdata.Window_size*5;
elseif imu_id == 9
    %     simdata.Window_size=5*4; % for VN-100
    simdata.Window_size=round(zupt_window/simdata.Ts);
    simdata.Window_size_for_step_detector = simdata.Window_size*5;
elseif imu_id == 10
    %     simdata.Window_size=5*40; % for ADIS_EVAL
    simdata.Window_size=round(zupt_window/simdata.Ts/1);
    %     simdata.Window_size=round(zupt_window/simdata.Ts/8);
    simdata.Window_size_for_step_detector = simdata.Window_size*5;
elseif imu_id == 11
    %     simdata.Window_size=5*40; % for ADIS_EVAL
    simdata.Window_size=round(zupt_window/simdata.Ts/1);
    %     simdata.Window_size=round(zupt_window/simdata.Ts/8);
    simdata.Window_size_for_step_detector = simdata.Window_size*5;
elseif imu_id == 12
    %     simdata.Window_size=5*40; % for ADIS_EVAL
    simdata.Window_size=round(zupt_window/simdata.Ts/1);
    %     simdata.Window_size=round(zupt_window/simdata.Ts/8);
    simdata.Window_size_for_step_detector = simdata.Window_size*5;
elseif imu_id == 12
    %     simdata.Window_size=5*40; % Open Shoe
    simdata.Window_size=round(zupt_window/simdata.Ts/1);
    %     simdata.Window_size=round(zupt_window/simdata.Ts/8);
    simdata.Window_size_for_step_detector = simdata.Window_size*5;    
else
    %     simdata.Window_size=5*10; % for Simulated device
    simdata.Window_size=round(zupt_window/simdata.Ts/2);
    simdata.Window_size_for_step_detector = simdata.Window_size*5;
end

simdata.Window_size_for_step_detector = simdata.Window_size*5;
simdata.Window_size_for_dynamic_covariance = simdata.Window_size/5;


% Threshold used in the zero-velocity detector. If the test statistics are
% below this value the zero-velocity hypothesis is chosen.
simdata.gamma=0.3e5;
simdata.gamma=3e5;


if imu_id == 0
    simdata.factor = exp(11); % for MPU-9250
elseif imu_id == 1
    %     threshold = 9.5;
%         threshold = 4.87;
%         threshold = 3.6;
%         threshold = 4;
%         threshold = 0.7;
%     threshold = 7;
%     threshold = 3; % walking
%     threshold = 4.19; % walking fast
    threshold = 5.4; % jogging backward
%     threshold = 6.89; % jogging    
%     threshold = 7.8; % running    
%     threshold = 8.7; % sprinting    

%     threshold = 8.8;
%         threshold = 3.5;
%         threshold = 6.5;
%         threshold = 5;
%         threshold = 1.5;
%         threshold = 0.08;
%         threshold = 0.8;
%         threshold = -0.25;
%         threshold = -0.03;



    
    simdata.factor = exp(threshold); % for VN-200
    simdata.factor_step = exp(6.3);
elseif imu_id == 2
    threshold = 11;
    simdata.factor = exp(threshold); % for ADIS16485
    simdata.factor_step = exp(11);
elseif imu_id == 3
%     threshold = 12;
    threshold = 4.6;
    simdata.factor = exp(threshold); % for Lab-On-Shoe
    simdata.factor_step = exp(14);
elseif imu_id == 4
    %     threshold = 10.5;
    %     threshold = 15;
    %     threshold = 14;
    %     threshold = 10;
%     threshold = 12.2;
    threshold = 1.4;
    
    %     simdata.factor = exp(10); % for SmartBug
    simdata.factor = exp(threshold); % for SmartBug
    simdata.factor_step = exp(14);
elseif imu_id == 5
    threshold = 6.2;
    simdata.factor = exp(threshold);  % for comsumer
    simdata.factor_step = exp(11);
elseif imu_id == 6
    threshold = 9.1;
    simdata.factor = exp(threshold); % for industrial
    simdata.factor_step = exp(11);
elseif imu_id == 7
    threshold = 16.8;
    simdata.factor = exp(threshold); % for tactical
    simdata.factor_step = exp(11);
elseif imu_id == 8
    threshold = 8;
    simdata.factor = exp(threshold); % for navigation
    simdata.factor_step = exp(11);
elseif imu_id == 9
    threshold = 13;
    simdata.factor = exp(threshold); % for VN-100
    simdata.factor_step = exp(11);
elseif imu_id == 10
    %     threshold = 10.5;
    threshold = 5.5;
    %     threshold = 12.5;
    %     threshold = 4;
    %     threshold = 8;
    %     threshold = 3.5;
    
    simdata.factor = exp(threshold); % for ADIS_EVAL
    simdata.factor_step = exp(12);
elseif imu_id == 11
%     threshold = 4;
    threshold = 2.0;
    simdata.factor = exp(threshold); % for Lab-On-Shoe 2.0
    simdata.factor_step = exp(5);
elseif imu_id == 12
%     threshold = 6;
    threshold = 6;

    simdata.factor = exp(threshold); % for Sugar-Cube Lab 20948
    simdata.factor_step = exp(14);
elseif imu_id == 13
%     threshold = -2; % for walking
%     threshold = 0.7; % for jogging
%     threshold = 2.3; % for sprinting 
    threshold = -7.4; % for stationary case
    simdata.factor = exp(threshold); % for OpenShoe
    simdata.factor_step = exp(14);    
else
    threshold = 7;
    simdata.factor = exp(threshold); % for Simulated device
    simdata.factor_step = exp(10.5);
end

disp(['ZUPT thresholds = ',num2str(threshold) ])

% Thresholds when other sensors are included
simdata.factor2 = exp(6);
simdata.factorDVS = exp(14); % DVS-aided SHOE Detector

if imu_id == 0
    simdata.factor = exp(11); % for MPU-9250
elseif imu_id == 1
    simdata.factorDownwardSONAR = exp(12); % UA-SHOE Detector
    simdata.factorShoeHeight = 0.045; % USPD Detector
elseif imu_id == 2
    simdata.factorDownwardSONAR = exp(12); % UA-SHOE Detector
    simdata.factorShoeHeight = 0.045; % USPD Detector
elseif imu_id == 3
    simdata.factorDownwardSONAR = exp(12); % UA-SHOE Detector
    simdata.factorShoeHeight = 0.045; % USPD Detector
elseif imu_id == 4
    simdata.factorDownwardSONAR = exp(12); % UA-SHOE Detector
    simdata.factorShoeHeight = 0.045; % USPD Detector
elseif imu_id == 5
    simdata.factorDownwardSONAR = exp(12); % UA-SHOE Detector
    simdata.factorShoeHeight = 0.045; % USPD Detector
elseif imu_id == 6
    simdata.factorDownwardSONAR = exp(12); % UA-SHOE Detector
    simdata.factorShoeHeight = 0.045; % USPD Detector
elseif imu_id == 7
    simdata.factorDownwardSONAR = exp(12); % UA-SHOE Detector
    simdata.factorShoeHeight = 0.045; % USPD Detector
elseif imu_id == 8
    simdata.factorDownwardSONAR = exp(12); % UA-SHOE Detector
    simdata.factorShoeHeight = 0.045; % USPD Detector
elseif imu_id == 9
    simdata.factorDownwardSONAR = exp(12); % UA-SHOE Detector
    simdata.factorShoeHeight = 0.045; % USPD Detector
elseif imu_id == 10
    simdata.factorDownwardSONAR = exp(12); % UA-SHOE Detector
    simdata.factorShoeHeight = 0.045; % USPD Detector
elseif imu_id == 11
    simdata.factorDownwardSONAR = exp(1.2); % UA-SHOE Detector
    simdata.factorShoeHeight = 0.09; % USPD Detector
elseif imu_id == 12
    simdata.factorDownwardSONAR = exp(12); % UA-SHOE Detector
    simdata.factorShoeHeight = 0.045; % USPD Detector
elseif imu_id == 13
    simdata.factorDownwardSONAR = exp(12); % UA-SHOE Detector
    simdata.factorShoeHeight = 0.045; % USPD Detector    
else
    simdata.factorDownwardSONAR = exp(12); % UA-SHOE Detector
    simdata.factorShoeHeight = 0.045; % USPD Detector
end

% simdata.factorDownwardSONAR = exp(12); % UA-SHOE Detector
% simdata.factorShoeHeight = 0.045; % USPD Detector


% Diagonal elements of the initial state covariance matrix (P).
% MPU9250: Turn on bias: 5 deg/s  60 mg for x&y  80 mg for z  60 mg = 0.59 m/s^2
simdata.sigma_initial_acc_bias=1e-4*ones(1,3)*10;           % Accelerometer biases (x,y,z platform coordinate axis)[m/s^2]
simdata.sigma_initial_gyro_bias=5*pi/180/3600*ones(1,3);   % 5 deg/h Gyroscope biases (x,y,z platform coordinate axis) [rad/s]
simdata.sigma_initial_pos=1e-3*ones(1,3)*0.1;               % Position (x,y,z navigation coordinate axis) [m]
simdata.sigma_initial_vel=1e-3*ones(1,3)*1;               % Velocity (x,y,z navigation coordinate axis) [m/s]
simdata.sigma_initial_att=(2* pi/180*ones(1,3))*0.1;            % 2 deg Attitude (roll,pitch,heading) [rad]
simdata.sigma_initial_acc_scale=0.0001*ones(1,3);       % Accelerometer scale factors (x,y,z platform coordinate axis)
simdata.sigma_initial_gyro_scale=0.00001*ones(1,3);     % Gyroscope scale factors (x,y,z platform coordinate axis)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%      For initial state matrix Q        %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% For consumer grade IMU:
%        Gyro:  ARW = 10 deg/sqrt(h)  RRW = 2e-3 deg/s/sqrt(s)
%        Accel:


% Process noise for modeling the drift in accelerometer biases (x,y,z
% platform coordinate axis) [m/s^2]. (In-Run Bias Stability)

if imu_id == 0
    simdata.acc_bias_driving_noise=8e-3*9.81*sqrt(simdata.Ts); % 8mg no data for MPU-9250
elseif imu_id == 1
    simdata.acc_bias_driving_noise=40e-6*9.81*sqrt(simdata.Ts)*1; % 0.04mg VN-200
%     simdata.acc_bias_driving_noise=40e-6*9.81*sqrt(simdata.Ts)*0.001; % 0.04mg VN-200    
%     simdata.acc_bias_driving_noise=40e-6*9.81*sqrt(simdata.Ts)*100; % 0.04mg VN-200
elseif imu_id == 2
    simdata.acc_bias_driving_noise=32e-6*9.81*sqrt(simdata.Ts); % 32ug for ADIS16485
elseif imu_id == 3
    simdata.acc_bias_driving_noise=13e-6*9.81*sqrt(simdata.Ts)*1; % 13ug for Lab-On-Shoe
    %         simdata.acc_bias_driving_noise=13e-6*9.81*sqrt(simdata.Ts)*1; % 13ug for Lab-On-Shoe
    %     simdata.acc_bias_driving_noise=13e-6*9.81*sqrt(simdata.Ts)*0.1; % 13ug for Lab-On-Shoe
    %     simdata.acc_bias_driving_noise=13e-6*9.81*sqrt(simdata.Ts)*1; % 13ug for Lab-On-Shoe
    %     simdata.acc_bias_driving_noise=4.9e-6*9.81*sqrt(simdata.Ts)*0.1; % 4.9ug for Simulated device
elseif imu_id == 4
    simdata.acc_bias_driving_noise=700e-6*9.81*sqrt(simdata.Ts); % 0.70mg for SmartBug
    %     simdata.acc_bias_driving_noise=700e-6*9.81*sqrt(simdata.Ts)*0.5; % 0.70mg for SmartBug
    %     simdata.acc_bias_driving_noise=700e-6*9.81*sqrt(simdata.Ts)*5; % 0.70mg for SmartBug incorrect for experiment
    %     simdata.acc_bias_driving_noise=4.9e-6*9.81*sqrt(simdata.Ts)*0.1; % 4.9ug for Simulated device
elseif imu_id == 5
    simdata.acc_bias_driving_noise=50e-3*9.81*sqrt(simdata.Ts); % >50mg for comsumer-grade device
    %     simdata.acc_bias_driving_noise=700e-6*9.81*sqrt(simdata.Ts); % 0.70mg for SmartBug
elseif imu_id == 6
    simdata.acc_bias_driving_noise=1e-3*9.81*sqrt(simdata.Ts); % 1~50mg for industrial-grade device
elseif imu_id == 7
    simdata.acc_bias_driving_noise=0.05e-3*9.81*sqrt(simdata.Ts); % 0.05~1mg for tactical-grade device
elseif imu_id == 8
    simdata.acc_bias_driving_noise=0.001e-3*9.81*sqrt(simdata.Ts); % <0.05mg for navigation-grade device
elseif imu_id == 9
    simdata.acc_bias_driving_noise=40e-6*9.81*sqrt(simdata.Ts)*100; % 0.04mg VN-100
elseif imu_id == 10
    %     simdata.acc_bias_driving_noise=13e-6*9.81*sqrt(simdata.Ts)*1; % 13ug for ADIS_EVAL
    simdata.acc_bias_driving_noise=13e-6*9.81*sqrt(simdata.Ts)*0.1; % 13ug for ADIS_EVAL
    %     simdata.acc_bias_driving_noise=13e-6*9.81*sqrt(simdata.Ts)*1; % 13ug for ADIS_EVAL
    %     simdata.acc_bias_driving_noise=4.9e-6*9.81*sqrt(simdata.Ts)*0.1; % 4.9ug for Simulated device
elseif imu_id == 11
    simdata.acc_bias_driving_noise=13e-6*9.81*sqrt(simdata.Ts)*1; % 13ug for Lab-On-Shoe
    %         simdata.acc_bias_driving_noise=13e-6*9.81*sqrt(simdata.Ts)*1; % 13ug for Lab-On-Shoe
    %     simdata.acc_bias_driving_noise=13e-6*9.81*sqrt(simdata.Ts)*0.1; % 13ug for Lab-On-Shoe
    %     simdata.acc_bias_driving_noise=13e-6*9.81*sqrt(simdata.Ts)*1; % 13ug for Lab-On-Shoe
    %     simdata.acc_bias_driving_noise=4.9e-6*9.81*sqrt(simdata.Ts)*0.1; % 4.9ug for Simulated device
elseif imu_id == 12
    simdata.acc_bias_driving_noise=13e-6*9.81*sqrt(simdata.Ts)*1; % Sugar-Cube Lab ICM-20948
elseif imu_id == 13
    simdata.acc_bias_driving_noise=40e-6*9.81*sqrt(simdata.Ts)*1; % Open Shoe
else
    simdata.acc_bias_driving_noise=4.9e-6*9.81*sqrt(simdata.Ts)*0.1; % 4.9ug for Simulated device
    %     simdata.acc_bias_driving_noise=50e-3*9.81*sqrt(simdata.Ts); % >50mg for comsumer-grade device
    %     simdata.acc_bias_driving_noise=25e-3*9.81*sqrt(simdata.Ts); % 1~50mg for industrial-grade device
    %     simdata.acc_bias_driving_noise=1e-3*9.81*sqrt(simdata.Ts); % 0.05~1mg for tactical-grade device
    %     simdata.acc_bias_driving_noise=0.05e-3*9.81*sqrt(simdata.Ts); % <0.05mg for navigation-grade device
end

% Process noise for modeling the drift in gyroscope biases (x,y,z platform
% coordinate axis) [rad/s]. (In-Run Bias Stability)
if imu_id == 0
    simdata.gyro_bias_driving_noise= 0.1 *pi/180*sqrt(simdata.Ts); % 0.1 degree/s for MPU-9250
elseif imu_id == 1
    simdata.gyro_bias_driving_noise= 10/3600 *pi/180*sqrt(simdata.Ts)*1; % 10 degree/hr for VN-200
%     simdata.gyro_bias_driving_noise= 10/3600 *pi/180*sqrt(simdata.Ts)*0.01; % 10 degree/hr for VN-200    
%     simdata.gyro_bias_driving_noise= 10/3600 *pi/180*sqrt(simdata.Ts)*100; % 10 degree/hr for VN-200    
elseif imu_id == 2
    simdata.gyro_bias_driving_noise= 6.25/3600 *pi/180*sqrt(simdata.Ts); % 6.25 degree/hr for ADIS16485
elseif imu_id == 3
    %     simdata.gyro_bias_driving_noise= 3.3/3600 *pi/180*sqrt(simdata.Ts)*0.1; % 3.3 degree/hr for ADIS16497-3 for Left Lab-On-Shoe
    simdata.gyro_bias_driving_noise= 3.3/3600 *pi/180*sqrt(simdata.Ts)*0.35; % 3.3 degree/hr for ADIS16497-3 for Right Lab-On-Shoe
    %     simdata.gyro_bias_driving_noise= 8e-4 *pi/180*sqrt(simdata.Ts)*0.1; % for Simulated device
elseif imu_id == 4
    simdata.gyro_bias_driving_noise= 0.028 *pi/180*sqrt(simdata.Ts)*1; % 0.028 degree/s for SmartBug
    %     simdata.gyro_bias_driving_noise= 0.028 *pi/180*sqrt(simdata.Ts)*0.5; % 0.028 degree/s for SmartBug
    %     simdata.gyro_bias_driving_noise= 0.028 *pi/180*sqrt(simdata.Ts)*5; % 0.028 degree/s for SmartBug, incorrect
    %     simdata.gyro_bias_driving_noise= 8e-4 *pi/180*sqrt(simdata.Ts)*0.1; % for Simulated device
elseif imu_id == 5
    simdata.gyro_bias_driving_noise= 100/3600 *pi/180*sqrt(simdata.Ts)*10; % > 100 degree/h for comsumer-grade device
elseif imu_id == 6
    simdata.gyro_bias_driving_noise= 10/3600 *pi/180*sqrt(simdata.Ts)*1; % 10~100 degree/h for industrial-grade device
elseif imu_id == 7
    simdata.gyro_bias_driving_noise= 0.1/3600 *pi/180*sqrt(simdata.Ts); % 0.01~10 degree/h for tactical-grade device
elseif imu_id == 8
    simdata.gyro_bias_driving_noise= 0.001/3600 *pi/180*sqrt(simdata.Ts); % <0.01 degree/h for navigation-grade device
elseif imu_id == 9
    simdata.gyro_bias_driving_noise= 10/3600 *pi/180*sqrt(simdata.Ts)*0.1; % 10 degree/hr for VN-100
elseif imu_id == 10
    simdata.gyro_bias_driving_noise= 3.3/3600 *pi/180*sqrt(simdata.Ts)*0.1; % 3.3 degree/hr for ADIS16497-3 for Left Lab-On-Shoe
    %     simdata.gyro_bias_driving_noise= 3.3/3600 *pi/180*sqrt(simdata.Ts)*0.35; % 3.3 degree/hr for ADIS16497-3 for Right Lab-On-Shoe
    %     simdata.gyro_bias_driving_noise= 8e-4 *pi/180*sqrt(simdata.Ts)*0.1; % for Simulated device
elseif imu_id == 11
    %     simdata.gyro_bias_driving_noise= 3.3/3600 *pi/180*sqrt(simdata.Ts)*0.1; % 3.3 degree/hr for ADIS16497-3 for Left Lab-On-Shoe
    simdata.gyro_bias_driving_noise= 3.3/3600 *pi/180*sqrt(simdata.Ts)*0.35; % 3.3 degree/hr for ADIS16497-3 for Right Lab-On-Shoe
    %     simdata.gyro_bias_driving_noise= 8e-4 *pi/180*sqrt(simdata.Ts)*0.1; % for Simulated device
elseif imu_id == 12
    simdata.gyro_bias_driving_noise= 3.3/3600 *pi/180*sqrt(simdata.Ts)*0.35; % Sugar-Cube Lab ICM-20948
elseif imu_id == 13
    simdata.gyro_bias_driving_noise= 10/3600 *pi/180*sqrt(simdata.Ts)*1; % Open Shoe
else
    simdata.gyro_bias_driving_noise= 8e-4 *pi/180*sqrt(simdata.Ts)*0.1; % for Simulated device
    %     simdata.gyro_bias_driving_noise= 100/3600 *pi/180*sqrt(simdata.Ts); % > 100 degree/h for comsumer-grade device
    %     simdata.gyro_bias_driving_noise= 10/3600 *pi/180*sqrt(simdata.Ts); % 10~100 degree/h for industrial-grade device
    %     simdata.gyro_bias_driving_noise= 0.1/3600 *pi/180*sqrt(simdata.Ts); % 0.01~10 degree/h for tactical-grade device
    %     simdata.gyro_bias_driving_noise= 0.01/3600 *pi/180*sqrt(simdata.Ts); % <0.01 100 degree/h for navigation-grade device
end


% Process noise for modeling the scale factor errors in accelerometers
simdata.acc_SF_driving_noise = 1e-10;

% Process noise for modeling the scale factor errors in gyroscopes
simdata.gyro_SF_driving_noise = 1e-10;

% Process noise for modeling the non-orthogonality in accelerometers
simdata.acc_ortho_driving_noise = 1e-8;

% Process noise for modeling the rotation misalignment in gyroscopes
simdata.gyro_rot_driving_noise = 1e-8;

% Process noise for modeling the non-orthogonality in gyroscopes
simdata.gyro_ortho_driving_noise = 1e-8;

% Pseudo zero-velocity update measurement noise covariance (R). The
% covariance matrix is assumed diagonal.
% Errors in the velocity measurement

if imu_id == 0 % for MPU-9250
    simdata.sigma_vel=[1 1 1]*0.001;      %[m/s]
elseif imu_id == 1 % VN-200
%         simdata.sigma_vel=[1 1 1]*0.005;      %[m/s]
%     simdata.sigma_vel=[1 1 1]*0.02;      %[m/s]
    simdata.sigma_vel=[1 1 1]*0.05;      %[m/s]
%     simdata.sigma_vel=[1 1 1]*0.1;      %[m/s]

elseif imu_id == 2 % for ADIS16485
    simdata.sigma_vel=[1 1 1]*0.001;      %[m/s]
elseif imu_id == 3 % for Lab-On-Shoe
    %     simdata.sigma_vel=[1 1 1]*0.001;      %[m/s]
    %     simdata.sigma_vel=[1 1 1]*0.005;      %[m/s]
    simdata.sigma_vel=[1 1 1]*0.02;      %[m/s]
    %     simdata.sigma_vel=[1 1 1]*0.001;      %[m/s]
elseif imu_id == 4 % for SmartBug
    simdata.sigma_vel=[1 1 1]*0.02;      %[m/s]
    %     simdata.sigma_vel=[1 1 1]*0.003;      %[m/s]
    %     simdata.sigma_vel=[1 1 1]*0.001;      %[m/s]
elseif imu_id == 5 % for Comsumer-grade
    simdata.sigma_vel=[1 1 1]*0.003;      %[m/s]
elseif imu_id == 6 % for industrial-grade
    simdata.sigma_vel=[1 1 1]*0.02;      %[m/s]
elseif imu_id == 7 % for tactical-grade
    simdata.sigma_vel=[1 1 1]*0.02;      %[m/s]
elseif imu_id == 8 % for navigation-grade
    simdata.sigma_vel=[1 1 1]*0.001;      %[m/s]
elseif imu_id == 9 % VN-100
    simdata.sigma_vel=[1 1 1]*0.005;      %[m/s]
elseif imu_id == 10 % for ADIS_EVAL
    %     simdata.sigma_vel=[1 1 1]*0.02;      %[m/s]
    simdata.sigma_vel=[1 1 1]*0.02;      %[m/s]
elseif imu_id == 11 % for Lab-On-Shoe
    %     simdata.sigma_vel=[1 1 1]*0.001;      %[m/s]
    %     simdata.sigma_vel=[1 1 1]*0.005;      %[m/s]
%     simdata.sigma_vel=[1 1 1]*0.005;      %[m/s]
        simdata.sigma_vel=[1 1 1]*0.02;      %[m/s]
elseif imu_id == 12 % for Sugar-Cube Lab
    %     simdata.sigma_vel=[1 1 1]*0.001;      %[m/s]
    %     simdata.sigma_vel=[1 1 1]*0.005;      %[m/s]
    simdata.sigma_vel=[1 1 1]*0.02;      %[m/s]
    %     simdata.sigma_vel=[1 1 1]*0.001;      %[m/s]
elseif imu_id == 13 % VN-200
    %     simdata.sigma_vel=[1 1 1]*0.005;      %[m/s]
%     simdata.sigma_vel=[1 1 1]*0.02;      %[m/s]    
    simdata.sigma_vel=[1 1 1]*0.001;      %[m/s]    
    
else
    simdata.sigma_vel=[1 1 1]*0.02;      %[m/s]
end

disp(['ZUPT uncertainty = ',num2str(simdata.sigma_vel(1)) ])


% simdata.sigma_vel=[1 1 1]*sqrt(simdata.factor)*simdata.sigma_a*sqrt(simdata.Window_size);

% Errors in the displacement measurement in ranging sensor
% simdata.sigma_dis=0.1;  %[m] for ultrasonic
simdata.sigma_dis=1;  %[m] for UWB

% Errors in the magnetic field measurement in magnetometer
simdata.sigma_mag=10;  %[nT]

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%             Altimeter SETUPS            %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if imu_id == 0
    % How many IMU data per Altimeter data
    simdata.ALT_rate = 20;
    % altimeter resolution
    simdata.alt_resolution = 0.1; % meters
elseif imu_id == 1
    % How many IMU data per Altimeter data
    simdata.ALT_rate = 20;
    % altimeter resolution
    simdata.alt_resolution = 0.1; % meters
elseif imu_id == 2
    % How many IMU data per Altimeter data
    simdata.ALT_rate = 20;
    % altimeter resolution
    simdata.alt_resolution = 0.1; % meters
elseif imu_id == 3
    % How many IMU data per Altimeter data
    simdata.ALT_rate = 20;
    % altimeter resolution
    simdata.alt_resolution = 0.1; % meters
elseif imu_id == 4
    % How many IMU data per Altimeter data
    simdata.ALT_rate = 20;
    % altimeter resolution
    simdata.alt_resolution = 0.1; % meters
elseif imu_id == 5
    % How many IMU data per Altimeter data
    simdata.ALT_rate = 20;
    % altimeter resolution
    simdata.alt_resolution = 0.1; % meters
elseif imu_id == 6
    % How many IMU data per Altimeter data
    simdata.ALT_rate = 20;
    % altimeter resolution
    simdata.alt_resolution = 0.1; % meters
elseif imu_id == 7
    % How many IMU data per Altimeter data
    simdata.ALT_rate = 20;
    % altimeter resolution
    simdata.alt_resolution = 0.1; % meters
elseif imu_id == 8
    % How many IMU data per Altimeter data
    simdata.ALT_rate = 20;
    % altimeter resolution
    simdata.alt_resolution = 0.1; % meters
elseif imu_id == 9
    % How many IMU data per Altimeter data
    simdata.ALT_rate = 20;
    % altimeter resolution
    simdata.alt_resolution = 0.1; % meters
elseif imu_id == 10
    % How many IMU data per Altimeter data
    simdata.ALT_rate = 20;
    % altimeter resolution
    simdata.alt_resolution = 0.1; % meters
elseif imu_id == 11
    % How many IMU data per Altimeter data
    simdata.ALT_rate = 20;
    % altimeter resolution
    simdata.alt_resolution = 100; % meters
elseif imu_id == 12
    % How many IMU data per Altimeter data
    simdata.ALT_rate = 20;
    % altimeter resolution
    simdata.alt_resolution = 0.1; % meters
elseif imu_id == 13
    % How many IMU data per Altimeter data
    simdata.ALT_rate = 20;
    % altimeter resolution
    simdata.alt_resolution = 0.1; % meters    
else
    % How many IMU data per Altimeter data
    simdata.ALT_rate = 20;
    % altimeter resolution
    simdata.alt_resolution = 0.1; % meters
end
% How many IMU data per Altimeter data
% simdata.ALT_rate = 20;

% altimeter resolution
% simdata.alt_resolution = 0.1; % meters

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%         relative position SETUPS        %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
simdata.cam_dis_std = 1.5;
simdata.cam_dis_vx = 0.001;
simdata.cam_dis_vy = 0.001;
simdata.cam_dis_vz = 0.001;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%      relative orientation SETUPS        %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
simdata.cam_roll_std = 0.01;
simdata.cam_pitch_std = 0.01;
simdata.cam_yaw_std = 0.01;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%            Adaptive ZUPT setup          %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% simdata.alpha = -exp(9)^1;
% simdata.theta = 35;
simdata.alpha = -exp(9)^1;
simdata.theta = 800.5;
simdata.beta = 0;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%               UWB setup                 %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
simdata.Window_size_nlos=50; % for MPU-9250
threshold_nlos = 0.000000001;
simdata.factor_nlos = exp(threshold_nlos); % for Lab-On-Shoe
simdata.UWB_std =1.5;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%               Dynamic Covariance                 %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% for simulation data
% dynamic_variance_hyper_para_alpha = [6 6 2]-1;
% dynamic_variance_hyper_para_gamma = 1;
% dynamic_variance_hyper_para_beta = [1 1 1]*0.02;
% dynamic_variance_hyper_para_psi = [-1,-1,-1]*6;

% for Sugar-Cube
simdata.dynamic_variance_hyper_para_alpha = [0 0 0]+(exp(-4.7));
simdata.dynamic_variance_hyper_para_gamma = 1.8;
simdata.dynamic_variance_hyper_para_beta = [1 1 1]*0.02;
simdata.dynamic_variance_hyper_para_psi = [-1,-1,-1]*6;

end





















