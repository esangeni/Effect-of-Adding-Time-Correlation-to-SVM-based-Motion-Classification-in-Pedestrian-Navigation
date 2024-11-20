function [roll_deg, pitch_deg, yaw_deg] = b321tang(b)
% [roll,pitch,yaw] = b321tang(b)
% Input: 
%        b    =  coordinate transformation matrix obtained by 321 rotation 
%                sequence, that is first rotate about 3-axis by 'yaw' angle,
%                next rotate aboue 2-axis by 'pitch' angle, and finally
%                rotate about 1-axis by 'roll' angle.
% Output:
%        roll =  'roll' angle about 1-axis (x-axis),  deg, 
%        pitch = 'pitch' angle about 2-axis (y-axis), deg, 
%        yaw  =  'yaw' angle about 3-axis (z-axis), deg,     
%
%  Note that there is an ambiguity in roll and yaw when pitch is +/- 90 deg.

r2d = 180/pi;

roll_deg  = atan2(b(2,3), b(3,3)) * r2d;   
aux = sqrt(1.0 - b(1,3)^2 );
pitch_deg = atan2(-b(1,3), aux) * r2d; 
yaw_deg   = atan2(b(1,2), b(1,1)) * r2d; 