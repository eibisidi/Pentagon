syms L11 L12 L21 L22 D L23 GAMA DELTA1 DELTA2 X0 Y0 ALPHA;
syms T1 T2;
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
pE = [X0 + xBF * cos(ALPHA) - yBF * sin(ALPHA); Y0 + xBF * sin(ALPHA) + yBF * cos(ALPHA)];
xWF = pE(1);
yWF = pE(2);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                L11   L12  L21  L22   D   L23 GAMA DELTA1 DELTA2 X0 Y0 ALPHA
vnom = [270; 370; 270; 370; 200; 370; 0; 0; 0; 0; 0; 0];
%vactual = vnom;
vactual = vnom + [0.1; 0.07; 0.2; 0.11; 0.15; 3; deg2rad(4.1); deg2rad(1.5); deg2rad(2.7); 4; 7; deg2rad(3.5)];
%vactual = vnom + [0.1; 0.3; 0.2; 0.4; 0.15; 3; deg2rad(4.1); 0; 0; 0 ; 0; 0];
%vactual = vnom + [0.1; 0.3; 0.2; 0.4; 0.15; 3; deg2rad(4.1); deg2rad(1.5); deg2rad(2.7); 4; 7; deg2rad(3.5)];
% means = [60; 348.617;
%          40; 348.617;
%          20; 348.617;
%          0; 348.617;
%          -20; 348.617;
%          -40; 348.617;
%          -60; 348.617;];
means = [200; 348.617;
         -200; 348.617;
         310; 250
         -310; 250;
         150; 480;
         -150; 480;
         10; 470];
est = zeros(size(means));
n = size(means, 1) / 2;
m = size(vnom, 1);

phis = [];
for i = 1 : n
    position = [means(2*i - 1);means(2*i)];
    phi = ik_sym(position, vnom);
    phis = [phis, phi];
    [xtmp, ytmp] = fk_sym(vactual, [phi(1); phi(2)]);
    means(2*i-1) = xtmp;
    means(2*i) = ytmp;
end

Jk_formula = [diff(xWF, L11), diff(xWF, L12), diff(xWF, L21),  diff(xWF, L22), diff(xWF, D), diff(xWF, L23), diff(xWF, GAMA), diff(xWF, DELTA1), diff(xWF, DELTA2), diff(xWF, X0), diff(xWF, Y0), diff(xWF, ALPHA);
    diff(yWF, L11), diff(yWF, L12), diff(yWF, L21),  diff(yWF, L22), diff(yWF, D), diff(yWF, L23), diff(yWF, GAMA), diff(yWF, DELTA1), diff(yWF, DELTA2), diff(yWF, X0), diff(yWF, Y0), diff(yWF, ALPHA)];

vreal = vnom;
for i = 1:6
    J = zeros(2*n, m);
    for k = 1 : n %populate matrix J ; estimate est vector
        w_phi = [vreal; phis(1, k); phis(2, k)];
        Jk = subs(Jk_formula, [L11; L12; L21; L22; D; L23; GAMA; DELTA1; DELTA2; X0; Y0; ALPHA; T1; T2], w_phi);
        J(2*k-1,:) = Jk(1,:);
        J(2*k,:) = Jk(2,:);
        [xtmp, ytmp] = fk_sym(vreal, [phis(1, k); phis(2, k)]);
        est(2*k-1) = xtmp;
        est(2*k) = ytmp;
    end
    
    %do column scaling
    d = zeros(m, 1);
    for c = 1 : m
        d(c) = norm(J(:, c));
    end
    csMatrix = diag(d); %column scaling matrix
    Jnorm = J /(csMatrix); %Jnorm = J * inv(csMatrix);
    RJ = Jnorm' * Jnorm; %RJ is correlation matrix
    
    y = means - est;
    delta_norm = (RJ) \ (Jnorm' * y);
    delta = csMatrix \ delta_norm;
    vreal = vreal + delta;
    
    RMSE = sqrt((y' * y )/ (2*n));
    fprintf("[%d] RMSE=%f.\n", i, RMSE);
    if (RMSE < 1E-16)
        fprintf("converge criterion meets.i=%d.\n", i);
        break;
    end
end
disp(vactual');
disp(vreal');
disp((vreal - vactual)');
