function r_wrtE_e = LLA2rWrtEinE(LLA, ...
                    EarthSemiMajorAxis, EarthEccentricitySq )

                               
% --------------------------------------------------------------------
% --------------------------------------------------------------------
 
if (nargin == 1)               
   EarthSemiMajorAxis  = 6378137.0;               
   EarthEccentricitySq = 6.694380004260835e-003;   
end
 
cLon  = cos(LLA(1,:));
sLon  = sin(LLA(1,:)); 
cLat  = cos(LLA(2,:));
sLat  = sin(LLA(2,:)); 

R_N  = EarthSemiMajorAxis./sqrt(1-EarthEccentricitySq*sLat.^2);

h = LLA(3,:); 

r_wrtE_e = [(R_N + h).*cLat.*cLon; 
            (R_N + h).*cLat.*sLon;
            (R_N*(1-EarthEccentricitySq) + h).*sLat];
       
        
 if (0)     
 % Check with somebody else script
 % --------------------------------------------------------------------
 % --------------------------------------------------------------------
 % --------------------------------------------------------------------
 
 lon = LLA(1);
 lat = LLA(2);
 alt = LLA(3);
 
% LLA2ECEF - convert latitude, longitude, and altitude to
%            earth-centered, earth-fixed (ECEF) cartesian
% 
% USAGE:
% [x,y,z] = lla2ecef(lat,lon,alt)
% 
% x = ECEF X-coordinate (m)
% y = ECEF Y-coordinate (m)
% z = ECEF Z-coordinate (m)
% lat = geodetic latitude (radians)
% lon = longitude (radians)
% alt = height above WGS84 ellipsoid (m)
% 
% Notes: This function assumes the WGS84 model.
%        Latitude is customary geodetic (not geocentric).
% 
% Source: "Department of Defense World Geodetic System 1984"
%         Page 4-4
%         National Imagery and Mapping Agency
%         Last updated June, 2004
%         NIMA TR8350.2
% 
% Michael Kleder, July 2005

% function [x,y,z]=lla2ecef(lat,lon,alt)

% WGS84 ellipsoid constants:
a = 6378137;
e = 8.1819190842622e-2;

% intermediate calculation
% (prime vertical radius of curvature)
N = a ./ sqrt(1 - e^2 .* sin(lat).^2);

% results:
x = (N+alt) .* cos(lat) .* cos(lon);
y = (N+alt) .* cos(lat) .* sin(lon);
z = ((1-e^2) .* N + alt) .* sin(lat);
r_e = [x;y;z];

dr_mm = (r_wrtE_e(:) - r_e(:))*1e3;

end

        