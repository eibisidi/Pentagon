function [xBF, yBF] = fk_sym(w, phi)
syms L11 L12 L21 L22 D L23 GAMA DELTA1 DELTA2 real;
syms T1 T2 real;
L11 = w(1);
L12 = w(2);
L21 = w(3);
L22 = w(4);
D = w(5);
L23 = w(6);
GAMA = w(7);
DELTA1 = w(8);
DELTA2 = w(9);

T1 = phi(1);
T2 = phi(2);
skew = [ 0 -1; 1 0];
rotation = [cos(GAMA) -sin(GAMA); sin(GAMA) cos(GAMA)];
rOB1 = [L11 * cos(T1 + DELTA1) - D / 2; L11 * sin(T1 + DELTA1)];
rOB2 = [L21 * cos(T2 + DELTA2) + D / 2; L21 * sin(T2 + DELTA2)];
rB1B2 = rOB2 - rOB1;
dB1B2 = sqrt(rB1B2' * rB1B2);
b1 = (L12^2 - L22^2 + dB1B2^2) / (2 * dB1B2);
h = sqrt(L12^2 - b1^2);
rOC = rOB1 + b1 / dB1B2 * rB1B2 + h / dB1B2 * skew * rB1B2;
rOE = rOB2 + L23/L22 * rotation * (rOC - rOB2);
xBF = rOE(1); %{BASE}
yBF = rOE(2);
end

