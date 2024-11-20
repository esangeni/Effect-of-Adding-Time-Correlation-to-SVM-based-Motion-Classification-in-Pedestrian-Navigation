
% ========================================================================
function [q_o, delta] = qintegrator(q, dTheta, deltaPrev, flag) 
% --------------------------------------------------------
% Input :
% q         (4 x 1) initial quaternion 
% dTheta    (3 x 1) current angle increment
% deltaPrev (3 x 1) increment used in propagating quaternion
% Output :
% q_o       (4 x 1) propagated quaternion
% delta     (3 x 1) increment for the next cycle
% --------------------------------------------------------

if (nargin < 4)
   flag = 4;
end

delta = zeros(3,1);
 
 
% -------------------------------------------------------------------
% Simple Liniear Approximations 
%keyboard
switch (flag) 
    case 0
         nrm_dth = norm(dTheta);
         if (nrm_dth < 1e-40) 
             e = zeros(3,1);
         else
             e = dTheta/nrm_dth;
         end
         dq0 = [e(:)*sin(nrm_dth/2); cos(nrm_dth/2)];
         q0 = q_mult(q, dq0);
         q_o = q0;
    case 1
         nrm_dth = norm(dTheta);
         if (nrm_dth < 1e-40) 
             e = zeros(3,1);
         else
             e = dTheta/nrm_dth;
         end
         E = [0     e(3)  -e(2)  e(1); 
             -e(3)   0     e(1)  e(2); 
              e(2) -e(1)    0    e(3);
             -e(1) -e(2)  -e(3)   0];
         dTh = E*nrm_dth;
         q1 = q + 0.5*dTh*q;
         q_o = q1;
    case 2 
         nrm_dth = norm(dTheta);
         if (nrm_dth < 1e-40) 
             e = zeros(3,1);
         else
             e = dTheta/nrm_dth;
         end
         E = [0     e(3)  -e(2)  e(1); 
             -e(3)   0     e(1)  e(2); 
              e(2) -e(1)    0    e(3);
             -e(1) -e(2)  -e(3)   0];
         q2 = (eye(4,4)*cos(nrm_dth/2)+sin(nrm_dth/2)*E)*q;
         q_o = q2;
end
 % -------------------------------------------------------------------

 q_o = q_o * (1.5 - 0.5 * q_o'*q_o );
 
% ========================================================================	   

 
        



