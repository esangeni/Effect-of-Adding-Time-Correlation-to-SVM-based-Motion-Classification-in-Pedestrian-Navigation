function [ g ] = gravityModel(Lat, P, a, e2);

GM  = 3.986005e14; % m^3/s^2 GravitationalConstant

c20 = -sqrt(5)*4.8416685e-4; 
Latc = atan((1-e2)*tan(Lat));

% geocentric
P2 = P*P;
a2 = a*a;
ax = 3*c20*a2/P2;
GMoverP2 = GM/P2; 
sLatc = sin(Latc);
cLatc = cos(Latc);
g_n = -GMoverP2 * ax*sLatc*cLatc; 
g_d =  GMoverP2 * (1 + ax/2*(3*sLatc^2-1));

g_n = -1.0 * g_n; % match Savage's model ?

% geodetic
Alpha = Lat - Latc;
cAlpha = cos(Alpha);
sAlpha = sin(Alpha);
g_N =  g_n*cAlpha + g_d*sAlpha;
g_D = -g_n*sAlpha + g_d*cAlpha;

g = [g_N; 0; g_D];

%keyboard


return


% gravity model
Omega    = 7.292115e-5; % rad/s (earth's rate) 
a = 6378137;     % m ( semi-major axis)
b = 6356752.3142;
f = (a-b)/a;
e2 = f*(2-f);
GM = 3.986005e14;  % m^3/s^2


P = a;

Latc = (0:100)/100*pi/2;
%Latc = pi/4;

%add = (0:100)/100*100; 
%P = a + add;

% -------------------------------
sLatc = sin(Latc);
cLatc = cos(Latc);




b2 = b*b;



% affect of centropital accel.
w_e2i_n = [ Omega*cos(Lat); 
            zeros(size(Lat));
           -Omega*sin(Lat)];

ax = 1-e2*sin(Lat).^2;
R_N = a./sqrt(ax);
h = 0;

% ---------------------------------
g = GM/a2;

(gD(end)-gD(1))/g * 1e3

close all

figure
plot(Latc*180/pi, gN/g*1e3,'b');  hold
plot(Latc*180/pi, gD/g*1e3,'r');

plot(Latc*180/pi, dg_cross(1,:)/g*1e3,'k--');
plot(Latc*180/pi, dg_cross(2,:)/g*1e3,'g--');
plot(Latc*180/pi, dg_cross(3,:)/g*1e3,'m--');

title(' Geodetic, deltas  g(North) &  g(Down),  [ mg ]')


dgT = dg_cross;
dgT(1,:) = dgT(1,:)+gN;
dgT(3,:) = dgT(3,:)+gD;

( dgT(3,end)- dgT(3,1))/g * 1e3


figure
plot(Latc*180/pi, dgT(1,:)/g*1e3,'b');  hold
plot(Latc*180/pi, dgT(2,:)/g*1e3,'g');
plot(Latc*180/pi, dgT(3,:)/g*1e3,'r');

title('Geodetic, total deltas g(N)[blue],g(E)[green],g(D)[red], [mg]')



