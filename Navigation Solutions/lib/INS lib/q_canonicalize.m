function  q_out = q_canonicalize( q_in )

q_out = q_in;
k = find(q_in(4,:) < 0);
if isempty(k) 
    return
else
    q_out(:,k) = -1 * q_out(:,k);
end

