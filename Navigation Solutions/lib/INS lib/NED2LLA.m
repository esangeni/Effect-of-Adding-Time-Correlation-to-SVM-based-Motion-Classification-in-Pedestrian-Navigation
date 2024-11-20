function LLA = NED2LLA(NED, LLA0)

north = NED(1,:);
east = NED(2,:);
down = NED(3,:);

lat = north./6378137 + LLA0(2,1);
lon = east./(cos(lat)*6378137)+LLA0(1,1);
alt = -down +LLA0(3,1);

LLA = [lon;lat;alt];