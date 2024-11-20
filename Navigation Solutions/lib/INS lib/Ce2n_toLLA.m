function [Lon_deg, Lat_deg] = Ce2n_toLLA(C_e2n)


D13 = C_e2n(3,1);
D23 = C_e2n(3,2);

Lon_deg = atan2(-D23,-D13)*180/pi; 

D31 = C_e2n(1,3);
D32 = C_e2n(2,3);
D33 = C_e2n(3,3);

aux = sqrt(D31*D31 + D32*D32); 

Lat_deg = atan2(-D33, aux)*180/pi;


%C_e2n = [-sLat*cLon  -sLat*sLon  cLat;
%         -sLon        cLon       0;
%         -cLon*cLat  -cLat*sLon  -sLat];