function m = euler1(i,a)
% function m=euler1(i,a)
% Calculates elementary rotation matrix corresponding to 
% rotation thru angle a (in radians) about axis i.
% Copied from John L. Junkins presentation AAS 89-060
% m (3,3) maps the vector in original coordinates to new coordinates
%         i.e, Vnew = m * Voriginal
ca=cos(a);
sa=sin(a);
if i==1, % rotation about x-axis
   m=[1,  0, 0
      0, ca, sa
      0,-sa, ca];
end;
if i==2, % rotation about y-axis
   m=[ca, 0,-sa
       0, 1,  0
      sa, 0, ca];
end;
if i==3, % rotation about z-axis
   m=[ ca, sa, 0
      -sa, ca, 0
        0,  0, 1];
end;
if i>3 | i<1 , 
  error('euler called with bad number for axis');
end;
return;