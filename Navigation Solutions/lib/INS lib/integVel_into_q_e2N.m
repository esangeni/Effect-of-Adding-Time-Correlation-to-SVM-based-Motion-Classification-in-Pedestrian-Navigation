%
%
function [ q_e2N ] = integVel_into_q_e2N(q_e2N0, h, Vn, dt)

a = 6378137.0 + h;
dth = [Vn(2,:)./a; -Vn(1,:)./a;  zeros(1,length(h))]*dt;  % cos(Lat)?
dth = (dth + [dth(:,2:end), dth(:,end)])/2;
dq = theta2quat(dth);
q_e2N = zeros(4,length(h));
q_e2N(:,1) = q_e2N0;
for i=2:length(h)
    q_e2N(:,i) = q_mult(q_e2N(:,i-1), dq(:,i-1));
end