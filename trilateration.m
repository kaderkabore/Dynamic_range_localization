function [x,y] = trilateration(pos1, zA,pos2, zB, pos3, zC)
%TRILATERATION Summary of this function goes here
%   Detailed explanation goes here
%--------------------------------------------------------------------------
%
% DESCRIPTION: Estimates the actual position based on previously estimated
%              distances
%
%      INPUTS: Pi - position coordinate of the Reference i 
%              zi - range distance to the Reference i

%
%     OUTPUTS: Estimated position (x, y) 
%
%--------------------------------------------------------------------------

xA = pos1(1);
yA = pos1(2);
xB = pos2(1);
yB = pos2(2);
xC = pos3(1);
yC = pos3(2);

P1 = [xA; yA; zA];
P2 = [xB; yB; zB];
P3 = [xC; yC; zC];

%% Transformation
ex = (P2 - P1) / (norm(P2 - P1));
i  = dot(ex, (P3 - P1));
ey = (P3 - P1 - i*ex) / (norm(P3 - P1 - i*ex));
ez = cross(ex, ey);
d  = norm(P2 - P1);
j  = dot(ey, (P3 - P1));

%% Estimation
x = ((zA^2) - (zB^2) + (d^2))/(2*d);
y = (((zA^2) - (zC^2) + (i^2) + (j^2))/(2*j)) - ((i/j)*x);


end

