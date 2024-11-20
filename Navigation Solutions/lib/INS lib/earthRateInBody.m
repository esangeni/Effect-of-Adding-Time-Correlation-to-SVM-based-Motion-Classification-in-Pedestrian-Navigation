function [ w_e2i_b ] = earthRateInBody(roll,pitch,yaw,latitude)

Omega = 7.292115060085166e-005;

scl = pi/180;

lat_rad   = latitude*scl;
roll_rad  = roll*scl;
pitch_rad = pitch*scl;
yaw_rad   = yaw*scl;

w_e2i_n = [cos(lat_rad);0;-sin(lat_rad)] * Omega;


C_roll  = trueCosine([roll_rad;0;0]);
C_pitch = trueCosine([0;pitch_rad;0]);
C_yaw   = trueCosine([0;0;yaw_rad]);

C_n2b = C_roll * C_pitch * C_yaw;

w_e2i_b = C_n2b * w_e2i_n;