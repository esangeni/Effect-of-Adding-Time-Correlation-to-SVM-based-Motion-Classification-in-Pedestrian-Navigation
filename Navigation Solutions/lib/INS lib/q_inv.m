function  q_out = q_inv( q_in )

q_out = q_in;
q_out(1:3,:) = -1 * q_out(1:3,:);
