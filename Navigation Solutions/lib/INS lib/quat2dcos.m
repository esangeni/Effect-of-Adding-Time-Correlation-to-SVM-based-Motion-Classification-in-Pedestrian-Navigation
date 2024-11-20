function [aa]	= quat2dcos(q)
% compute direction cosine matrix from quaternion
%--------------------------------------------------
q1	= q(1);
q2	= q(2);
q3	= q(3);
q4	= q(4);

a11	= q1*q1 - q2*q2 - q3*q3 + q4*q4;
a12	= 2*(q1*q2 + q3*q4);
a13	= 2*(q1*q3 - q2*q4);
a21	= 2*(q1*q2 - q3*q4);
a22	= -q1*q1 + q2*q2 - q3*q3 + q4*q4;
a23	= 2*(q2*q3 + q1*q4);
a31	= 2*(q1*q3 + q2*q4);
a32	= 2*(q2*q3 - q1*q4);
a33	= -q1*q1 - q2*q2 + q3*q3 + q4*q4;

aa	= [a11 a12 a13
	   a21 a22 a23
	   a31 a32 a33 ];
