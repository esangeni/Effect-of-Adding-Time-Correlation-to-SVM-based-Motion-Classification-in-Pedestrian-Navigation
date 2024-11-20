function p = cubic_interpolation( d1, d2 )
% This subfunciton is to calculate the coefficient of the cubic
% interpolation, given the value and slope of both ends

% Input:  d1&d2  1*3 array [position value derivative]
% Output: p      1*4 array [a b c d]

x1 = d1(1);
x2 = d2(1);
y1 = d1(2);
y2 = d2(2);
y1_d = d1(3);
y2_d = d2(3);

A = [x1^3 x1^2 x1 1; x2^3 x2^2 x2 1; 3*x1^2 2*x1 1 0; 3*x2^2 2*x2 1 0];

b = [y1; y2; y1_d; y2_d];

p = A\b;

p = p';

end

