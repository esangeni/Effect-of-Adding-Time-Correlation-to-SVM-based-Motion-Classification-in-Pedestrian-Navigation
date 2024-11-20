% Navigation Solution Determination (nsd)
% outside need to initialize "input" and then "input = prev."output";
function [ output ] = navSLN_ZUPT( sensor, input )
%                        
global simdata;

w_b2i_b    = sensor.w_b2i_b;
f_b        = sensor.f_b;
dt         = sensor.dt;
%
q_b2n          = input.q_b2n;
q_e2n          = input.q_e2n;
LLA            = input.LLA;  % [lonRad; latRad; alt]
v_nWrtE_n      = input.v_nWrtE_n;
% No use
delta_b_Prev   = input.delta_b_Prev;
delta_n_Prev   = input.delta_n_Prev;
delta_n2e_Prev = input.delta_n2e_Prev;
% -------------------------------------------------------------------------
Omega = simdata.earthrate;           % rad/s (earth's rate) 
a     = 6378137;               % m ( semi-major axis)
e2    = 6.694380004260835e-3;  % EarthEccentricitySq
% -------------------------------------------------------------------------
cLat = cos(LLA(2));
sLat = sin(LLA(2));
w_e2i_n = Omega*[cLat; 0; -sLat];
ax   = 1-e2*sLat^2;
R_N  = a./sqrt(ax);
R_M  = R_N*(1-e2)/ax;
h    = LLA(3);
%
% dotLLA(1) =  v_nWrtE_n(2) /(R_N + h) * 1/cLat;
% dotLLA(2) =  v_nWrtE_n(1) /(R_M + h);
dotLLA(3) = -v_nWrtE_n(3);
% w_n2e_n = [ dotLLA(1) * cLat;  -dotLLA(2);  (-dotLLA(1) * sLat)*0];
w_n2e_n(1,:) =  v_nWrtE_n(2)/a;
w_n2e_n(2,:) = -v_nWrtE_n(1)/a;
w_n2e_n(3,:) =   0;

% =========================================================================
% Attitude
% =========================================================================
dTheta_b      = w_b2i_b * dt;
[q, delta_b_Prev] = qintegrator(q_inv(q_b2n), dTheta_b, delta_b_Prev,0);             
q_b2nNm_wErth = q_inv(q);               
w_n2i_n       = w_e2i_n + w_n2e_n;
dTheta_n      = w_n2i_n * dt; 
[q_b2n, delta_n_Prev] = qintegrator(q_b2nNm_wErth, dTheta_n, delta_n_Prev,0);
%==========================================================================
% Vel.& Pos.
% =========================================================================
% gravity
r_e_n          = [-R_N*e2*sLat*cLat; 0; -R_N*ax - h];
g_n            = gravityModel(LLA(2), norm(r_e_n), a, e2);       
dg_centropital = cross(w_e2i_n, cross(w_e2i_n, r_e_n));  
gl_n           = g_n - dg_centropital;
% =========================================================================
w_aux     = 2 * w_e2i_n  +  w_n2e_n;
f_n       = quatRot(q_b2n, f_b);
v_nWrtE_n = v_nWrtE_n  + (f_n - cross(w_aux, v_nWrtE_n) + gl_n) * dt ;
% =========================================================================
% dotLLA(1) =  v_nWrtE_n(2) /(R_N + h) * 1/cLat;
% dotLLA(2) =  v_nWrtE_n(1) /(R_M + h);
dotLLA(3) = -v_nWrtE_n(3);
% w_n2e_n = [ dotLLA(1) * cLat;  -dotLLA(2);  (-dotLLA(1) * sLat)*0];
w_n2e_n(1,:) =  v_nWrtE_n(2)/a;
w_n2e_n(2,:) = -v_nWrtE_n(1)/a;
w_n2e_n(3,:) =   0;
% =========================================================================
dTheta_n2e_n = w_n2e_n * dt;
[q_e2n, delta_n2e_Prev] = qintegrator(q_e2n, dTheta_n2e_n, delta_n2e_Prev,0);
%C_e2n  = quat2dcos(q_e2n);
[lonDeg, latDeg] = q_e2N_toLonLatDeg(q_e2n);
LLA(1) = lonDeg*pi/180;
LLA(2) = latDeg*pi/180;
LLA(3) = LLA(3) +  dotLLA(3) * dt;                   
% =========================================================================        
output.q_b2n          = q_b2n;
output.q_e2n          = q_e2n;
output.LLA            = LLA;
output.v_nWrtE_n      = v_nWrtE_n;
output.delta_b_Prev   = delta_b_Prev;
output.delta_n_Prev   = delta_n_Prev;
output.delta_n2e_Prev = delta_n2e_Prev;
% =========================================================================
    