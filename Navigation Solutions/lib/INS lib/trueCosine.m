function C = trueCosine(v)
% -------------------------------------------------------------------------
v = v(:);
I3  = eye(3,3);
norm_v = norm(v);
if norm_v < 1e-15
   C = I3;
   return
end    
e_th = v/norm_v;
EEt  = (e_th*e_th'); 
Ex   = skew(e_th);
cosTh = cos(norm_v);
sinTh = sin(norm_v);
% -------------------------------------------------------------------------
C = cosTh*I3 + (1-cosTh)*(e_th*e_th') - sinTh*skew(e_th);
% -------------------------------------------------------------------------    