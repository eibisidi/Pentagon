function phi = ik_sym( w, pE_Base)
% syms L11 L12 L21 L22 D L23 GAMA DELTA1 DELTA2 real;

L11 = w(1);
L12 = w(2);
L21 = w(3);
L22 = w(4);
D = w(5);
L23 = L22;
GAMA = 0;
DELTA1 = w(8);
DELTA2 = w(9);
% X0 = w(10); 
% Y0 = w(11);
% ALPHA = w(12);
% pTmp = pE_WF - [X0; Y0];
% rotation = [cos(ALPHA) sin(ALPHA); -sin(ALPHA) cos(ALPHA)];
% rOE = rotation' * pTmp;

rOE = pE_Base;

rA2E = rOE - [D/2; 0];
dA2E = sqrt(rA2E' * rA2E);
a2 = (L21^2 - L23^2 + dA2E^2) / (2*dA2E);
h2 = sqrt(L21^2 - a2^2);
rA2B2 = a2 / dA2E * rA2E + h2 / dA2E * [0 1; -1 0] * rA2E;
rOB2 = [D / 2; 0] + rA2B2;
rOC = rOB2 + L22 / L23 * [cos(GAMA) sin(GAMA); -sin(GAMA) cos(GAMA)] * (rOE - rOB2);
rA1C = rOC - [-D/2;0];
dA1C = sqrt(rA1C' * rA1C);
a1 = (L11^2 - L12^2 + dA1C^2) / (2 * dA1C);
h1 = sqrt(L11^2 - a1^2);
rA1B1 = a1 / dA1C * rA1C - h1 / dA1C * [0 1; -1 0] * rA1C;

xA1B1 = rA1B1(1);
yA1B1 = rA1B1(2);
t1 = wrapTo2Pi(atan2(yA1B1, xA1B1)) - DELTA1;

xA2B2 = rA2B2(1);
yA2B2 = rA2B2(2);
t2 = wrapToPi(atan2(yA2B2, xA2B2)) - DELTA2;

phi = [t1; t2];
end

