function NED = LLA2NED(LLA, LLA0)

north =  (LLA(2,:)-LLA0(2,1))*6378137;
east  =  (LLA(1,:)-LLA0(1,1)).*cos(LLA(2,:))*6378137;
down    = -(LLA(3,:)-LLA0(3,1));

NED = [north;east;down];