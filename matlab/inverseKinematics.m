function [pp, pn, np, nn] = inverseKinematics(r, p)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
r1 = r(1);
r2 = r(2);
r3 = r(3);
x = p(1);
y = p(2);

a1 = r1^2 + y^2 + (x + r3)^2 - r2^2 + 2 * (x + r3) * r1;
b1 = -4 * y * r1;
c1 = r1^2 + y^2 + (x + r3)^2 - r2^2 - 2 *(x + r3) * r1;
a2 = r1^2 + y^2 + (x - r3)^2 - r2^2 + 2 * (x - r3) * r1;
b2 = b1;
c2 = r1^2 + y^2 + (x - r3)^2 - r2^2 - 2 *(x - r3) * r1;

det1 = b1^2 - 4 * a1 * c1;
det2 = b2^2 - 4 * a2 * c2;
if ( det1 < -1E-6 || det2 < -1E-6)
    pp = [];
    pn = [];
    np = [];
    nn = [];
    return;
end

if (det1 < 1E-6)
    det1 = 0;
end

if (det2 < 1E-6)
    det2 = 0;
end

pp = [0; 0];
pn = [0; 0];
np = [0; 0];
nn = [0; 0];

%sigma1 = 1
Y = -b1 + sqrt(det1);
X = 2 * a1;
theta1 = 2 * atan2d(Y, X);
pp(1) = theta1;
pn(1) = theta1;

%sigma1 = -1
Y = -b1 - sqrt(det1);
X = 2 * a1;
theta1 = 2 * atan2d(Y, X);
np(1) = theta1;
nn(1) = theta1;

%sigma2 = 1
Y = -b2 + sqrt(det2);
X = 2 * a2;
theta2 = 2 * atan2d(Y, X);
pp(2) = theta2;
np(2) = theta2;

%sigma2 = -1
Y = -b2 - sqrt(det2);
X = 2 * a2;
theta2 = 2 * atan2d(Y, X);
pn(2) = theta2;
nn(2) = theta2;

return;
end

