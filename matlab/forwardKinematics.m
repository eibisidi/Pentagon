function [nSol, up, down] = forwardKinematics(r,theta)
%forward kinematics
%input arguments: 
%               r: [r1; r2; r3] column vector
%               theta: [theta1; theta2] column vector
%return value:
%           nSol: number of solutions.  
%           	-1: infinitely many solutions, B1 and B2 coincides
%               0: no solution 1/2
%               1: up and down is the same point
%               2: up and down is different
%           up: up-configuration
%           down: down-configuration
s1 = sind(theta(1));
c1 = cosd(theta(1));
s2 = sind(theta(2));
c2 = cosd(theta(2));

denominator =  (2 * r(3) + r(1) * c2 - r(1) * c1);
if (abs(denominator) < 1E-6)
    %infinitely many solutions
    nSol = -1; 
    up = [];
    down = [];
    return;
end

e = r(1) * (s1 - s2)/ denominator;
f = r(1) * r(3) * (c2 + c1) / denominator;
d = 1 + e^2;
g = 2 * (e * f - e * r(1)*c1 + e * r(3) - r(1) * s1);
h = f^2 - 2*f*(r(1)*c1 - r(3)) - 2 * r(1) * r(3) * c1 + r(3)^2 + r(1)^2 - r(2)^2;

det = g^2 - 4*d*h;

if (det < -1E-6)
    %no solution
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

