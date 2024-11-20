function thetaArray = quatToAngl(qArray)

% test
%if (1)
%   angl1 = 25*pi/180;
%   x1 = [1;2;-5];
%   x1 = x/norm(x);
%   q = [x1*sin(angl/2);cos(angl/2)];
%   theta1 = x1*angl;
%end

N = size(qArray,2);

thetaArray = zeros(3,N);

if N<2
    qArray = qArray(:);
end

for i=1:N
q = qArray(:,i);    
x = q(1:3);
xNorm = norm(x);
theta = zeros(3,1);
if xNorm>0
  angl = atan2(xNorm, q(4))*2;
  theta = x/xNorm*angl;
end 
thetaArray(:,i) = theta(:);
end