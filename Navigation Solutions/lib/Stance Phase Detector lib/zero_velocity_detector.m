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
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function zupt =zero_velocity_detector(u)


% Global struct holding the simulation settings
global simdata;


% Run the desired detector type. Each detector return a vector with their 
% calculated test statistics T. 


W=simdata.Window_size;
sigma2_a = (simdata.accel/simdata.Ts)^2;
sigma2_g = (simdata.gyro/simdata.Ts)^2;

tmp = var(u, 0, 2);

total = 0;

for i = 1:3
    total = total + tmp(i)/sigma2_a;
    total = total + tmp(i+3)/sigma2_g;
end

total = total/6;

if(total < simdata.factor)
    zupt = ones(1, simdata.Window_size);
else
    zupt = zeros(1, simdata.Window_size);
end





end

