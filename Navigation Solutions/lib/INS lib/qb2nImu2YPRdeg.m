function [roll_deg, pitch_deg, yaw_deg ] = qb2nImu2YPRdeg(q_b2n, C_bRef2b)
%
% Rotation 3-2-1 from nav. to b-frame 
% Yaw (Heading)  -> Pitch -> Roll
% If specified C_bRef2b then 
% Rotation 3-2-1 from nav. to bRef-frame  where
%  C_bRef2b is constant offset from b-frame

if (nargin==1)
   C_bRef2b = eye(3,3); 
end

[n1,n2] = size(q_b2n);

N = n2;

if (0)
if n1>n2
   q_b2n = q_b2n';
   N = n1;
end
end


roll_deg  = zeros(1,N); 
pitch_deg = roll_deg; 
yaw_deg   = roll_deg; 


for i=1:N
    nq = norm(q_b2n(:,i));
    q_b2n_use = q_b2n(:,i);
    if (nq>0.0)
       q_b2n_use = q_b2n_use/nq;
    end
    C_b2n = quat2dcos( q_b2n_use );
    C_bRef2n = C_b2n * C_bRef2b; 
    [roll_deg(i), pitch_deg(i), yaw_deg(i)] = b321tang( C_bRef2n' );
end



