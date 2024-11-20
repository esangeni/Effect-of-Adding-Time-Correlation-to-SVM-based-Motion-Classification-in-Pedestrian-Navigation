function [lonDeg, latDeg] = q_e2N_toLonLatDeg(q_e2N)

Npnt = size(q_e2N,2);

lonDeg = zeros(1,Npnt);
latDeg = zeros(1,Npnt);

for i=1:Npnt
    C_e2n = quat2dcos(q_e2N(:,i));
    [lonDeg(i), latDeg(i)] = Ce2n_toLLA(C_e2n);
end

