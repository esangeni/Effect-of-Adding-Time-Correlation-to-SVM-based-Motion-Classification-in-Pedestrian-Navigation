function [q,a1,a2,a3,a4,b,ei] = dcos2quat( d, enblNormalization )
% =========================================================================
% convert 3x3 direction cosine matrix d to 4-vector quaternion q.
% If q maps frame A to frame B, then d maps vectors in
% frame A to frame B. q & d represent the orientation of B with
% respect to A.
%   NOTE: In theory, the resulting quaternion should be a unit
%         quaternion. But normalization is used.
%         
% d = 3x3 direction cosine matrix mapping frame A to frame B.
%
% q = quaternion state vector (4 elements), representing the
%     orientation of frame B with respect to frame A. In terms of
%     the Euler rotation of theta radians about the unit vector e
%     which maps frame A to frame B:
%
%       q(1:3) = e(1:3)*sin(theta/2) = V = vector part of q
%       q(4)   =        cos(theta/2) = S = scalar part of q
%
% any orientation has two associated quaternions, one of which is -1*the
% other. The canonical one is the one for which q(4) is non-negative,
% and corresponds to an Euler angle in the range 0 - pi.
% the variable "fix", below, is used to negate the quaternion on
% the fly, if necessary.
%
% Reference: eq 12-14, p. 415, Wertz, "Spacecraft Attitude
%                                       Determination and Control"
%
% =========================================================================
if (nargin == 1)
    enblNormalization = 1;
end

q = zeros(4,1);
% select combination of diagonal dir cos elements that give largest 
% denominator in initial quaternion equations to avoid numerical problems
        a1 = d(1,1) + d(2,2) + d(3,3);
        a2 = d(1,1) - d(2,2) - d(3,3);
        a3 = -d(1,1) + d(2,2) - d(3,3);
        a4 = -d(1,1) - d(2,2) + d(3,3);

        b  = max( [a1 a2 a3 a4 ]' );

        ei = 0.5*sqrt(1 + b);  % this is equation 12-14a from Wertz p. 415
        inv_4ei = 0.25/ei;
% now compute initial quaternions using formulas which are appropriate given
% the denominator element just selected
if b == a1
        q(1) = (d(2,3) - d(3,2))*inv_4ei;
        q(2) = (d(3,1) - d(1,3))*inv_4ei;
        q(3) = (d(1,2) - d(2,1))*inv_4ei;
        q(4) = ei;
elseif b == a2
        q(1) = ei;
        q(2) = (d(1,2) + d(2,1))*inv_4ei;
        q(3) = (d(3,1) + d(1,3))*inv_4ei;
        q(4) = (d(2,3) - d(3,2))*inv_4ei;
elseif b == a3
        q(1) = (d(1,2) + d(2,1))*inv_4ei;
        q(2) = ei;
        q(3) = (d(2,3) + d(3,2))*inv_4ei;
        q(4) = (d(3,1) - d(1,3))*inv_4ei;
else
        q(1) = (d(3,1) + d(1,3))*inv_4ei;
        q(2) = (d(3,2) + d(2,3))*inv_4ei;
        q(3) = ei;
        q(4) = (d(1,2) - d(2,1))*inv_4ei;
end
% ensure q is canonical
fix = sign(q(4));
if fix==0,
   fix=1;
end

q(1) = fix*q(1);
q(2) = fix*q(2);
q(3) = fix*q(3);
q(4) = fix*q(4);


if (enblNormalization) 
    %disp('normalization')
    if (abs(norm(q)) > 1e-40)
       q = q/norm(q); 
    else
       q = [0;0;0;1]; 
    end    
end    
    

