function [ q_e2N ] = lonLatDegTo_q_e2N(lonDeg, latDeg)

q_e2N = zeros(4,length(lonDeg));

for i=1:length(lonDeg)
    
    cLon = cosd( lonDeg(i) ); sLon = sind( lonDeg(i) );
    cLat = cosd( latDeg(i) ); sLat = sind( latDeg(i) );
    
    C_e2N = [-sLat*cLon  -sLat*sLon  cLat;
             -sLon        cLon       0;
             -cLon*cLat  -cLat*sLon  -sLat];
         
    q_e2N = dcos2quat(C_e2N);
end