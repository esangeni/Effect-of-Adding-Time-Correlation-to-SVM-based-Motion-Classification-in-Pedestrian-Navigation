function [ W ] = quatRot(q, V)                                     
%     This function performs:   W = q * V   where
%     q is a quaternions (4, N)  (actully matrix)                          
%     V is vector  (3 x N)
%     W is matrix (3, N);
%     The i-th column in W(:,i) is rotated by q(:,i) vector v  
%! ===========================================================================
q0 = q(4,:);
q1 = q(1,:);
q2 = q(2,:);
q3 = q(3,:);

ax0  = 2*q0.^2;
ax12 = 2*q1.*q2;
ax30 = 2*q3.*q0;
ax23 = 2*q2.*q3;
ax20 = 2*q2.*q0;
ax13 = 2*q1.*q3;
ax10 = 2*q1.*q0;

Q11 = ax0-1+ 2*q1.^2;
Q12 = ax12 + ax30;
Q13 = ax13 - ax20;

Q21 = ax12 - ax30;
Q22 = ax0 - 1 + 2*q2.^2;
Q23 = ax23 + ax10;

Q31 = ax13 + ax20;
Q32 = ax23 - ax10;
Q33 = ax0 - 1 + 2*q3.^2;

%w1 = [Q11(:) Q12(:) Q13(:)] * v;
%w2 = [Q21(:) Q22(:) Q23(:)] * v;
%w3 = [Q31(:) Q32(:) Q33(:)] * v;

X = V';
w1 = Q11(:).* X(:,1) + Q12(:).* X(:,2) + Q13(:).*X(:,3);
w2 = Q21(:).* X(:,1) + Q22(:).* X(:,2) + Q23(:).*X(:,3);
w3 = Q31(:).* X(:,1) + Q32(:).* X(:,2) + Q33(:).*X(:,3);

W = [w1(:)'; w2(:)'; w3(:)'];
