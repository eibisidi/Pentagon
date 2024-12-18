function [nSol, up, down] = forwardKinematics(r,theta)
%forward kinematics
%input arguments: 
%               r: [r1; r2; r3] column vector
%               theta: [theta1; theta2] column vector
%return value:
%           nSol: number of solutions.  0/1/2
%           up: up-configuration
%           down: down-configuration
s1 = sind(theta(1));
c1 = cosd(theta(1));
s2 = sind(theta(2));
c2 = cosd(theta(2));

e = r(1) * (s1 - s2)/ (2 * r(3) + r(1) * c2 - r(1) * c1);
f = r(1) * r(3) * (c2 + c1) / (2 * r(3) + r(1)*c2 - r(1) * c1);
d = 1 + e^2;
g = 2 * (e * f - e * r(1)*c1 + e * r(3) - r(1) * s1);
h = f^2 - 2*f*(r(1)*c1 - r(3)) - 2 * r(1) * r(3) * c1 + r(3)^2 + r(1)^2 - r(2)^2;

det = g^2 - 4*d*h;

if (det < -1E-6)
    nSol = 0; 
    up = [];
    down = [];
    return;
end

if (det < 1E-6)
    y = -g / 2 / d;
    x = e * y + f;
    nSol = 1;
    up = [x; y];
    down = [x; y];
    return;
end

y1 = (-g + sqrt(det)) / 2 /d;
x1 = e * y1 + f;

y2 = (-g - sqrt(det)) / 2 /d;
x2 = e * y2 + f;

nSol = 2;
up = [x1; y1];
down = [x2; y2];
end

