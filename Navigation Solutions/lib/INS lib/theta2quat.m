function [ q ] = theta2quat( theta )                                     
% theta in rad.
% =========================================================================
x = theta;
xNorm = sqrt(x(1,:).^2+x(2,:).^2+x(3,:).^2);
k=find(xNorm>0);
rotAxs = zeros(size(x));
if ~isempty(k)
   rotAxs(:,k) = [ x(1,k)./xNorm(k); x(2,k)./xNorm(k); x(3,k)./xNorm(k)];
end
sAnglDiv2 = sin(xNorm/2);
cAnglDiv2 = cos(xNorm/2);
q = [ rotAxs(1,:).* sAnglDiv2; rotAxs(2,:).* sAnglDiv2; 
      rotAxs(3,:).* sAnglDiv2; cAnglDiv2];
 