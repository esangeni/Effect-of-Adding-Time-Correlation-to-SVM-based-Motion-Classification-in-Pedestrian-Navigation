function [ q_b2n ] = Imu2YPRdeg_to_qb2n(roll_deg, pitch_deg, yaw_deg)
%
% Rotation 3-2-1 from nav. to b-frame 
% Yaw (Heading)  -> Pitch -> Roll

d2r = pi/180;

N1 = length(roll_deg);
N2 = length(pitch_deg);
N3 = length(yaw_deg);

N = max([N1,N2,N3]);
en = ones(1,N);

if (N1==1)
    roll_deg  = roll_deg*en;
end
if (N2==1)
    pitch_deg = pitch_deg*en;
end
if (N3==1)
    yaw_deg = yaw_deg*en;
end

q_b2n = zeros(4,N);

for i=1:N
    Cz = euler1(3,yaw_deg(i)*d2r);   
    Cy = euler1(2,pitch_deg(i)*d2r);   
    Cx = euler1(1,roll_deg(i)*d2r); 
    C_n2b = Cx*Cy*Cz;
    q = dcos2quat( C_n2b');
    q_b2n(:,i) =  q_canonicalize(q);
end



