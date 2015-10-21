function [FN,S1,S2,FMX,FMY,n,u_0,FF] = parameters()
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
%Set physical valves for system
R = 0.01 %radius in meters
u = (8.90*10^(-4));  %dynamic viscosity
%u = 0.5
W = 0.001 %weight of ball in kg
uk = 0.16 %kenitic coff of friction
FN = 9.81*W
S1 = (-6*3.14*u*R)/W;
S2 = S1;
FMX = (0.035*1.33)/W;
FMY = FMX;
FF = (uk*FN)/W;
u_0 = 4*3.14*10^(-7)
n = 512;

end

