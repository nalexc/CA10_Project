clear all; close all;

StationLocation = [0; 6371];
t = 0:1:5400;
orbitOmega = -2*pi/5400;
SatelliteLocation = [sin(orbitOmega*t); cos(orbitOmega*t)]*6971;

displacement = SatelliteLocation - StationLocation;

orientation = zeros(length(t),1);

for i = 1:length(t)
    orientation(i) = atan2(displacement(2,i),displacement(1,i)) + pi;    
end

swap = t >= 360;
orientation = orientation + 2*pi*swap';
% orientation = [orientation; orientation; orientation]


omega = diff(orientation);
omega = [omega;omega;omega];

omega = omega(2700:end);

figure
plot(omega)
title("Angular velocity of Earth station pointing satellite")
xlabel("Time (sec)")
ylabel("Angular velocity (rad/sec)")

figure
plot(diff(omega))
title("Angular acceleration of Earth station pointing satellite")
xlabel("Time (sec)")
ylabel("Angular acceleration (rad/sec^2)")

Ix = 0.0017;
Iy = 0.0022;
Iz = Iy;

I = [Ix Iy Iz];
torqueMax = max(I) * max(diff(omega))