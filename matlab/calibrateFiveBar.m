syms L11 L12 L21 L22 D L23 GAMA DELTA1 DELTA2 X0 Y0 ALPHA;
syms T1 T2;
simulation = 1;
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

means = [60; 348.617 + 10;
    40; 348.617 ;
    20; 348.617 + 10;
    0; 348.617; %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    -20; 348.617;
    -40; 348.617 + 10;
    -60; 348.617;
    -80; 348.617 + 10];

phis = [];
n = size(means, 1) / 2;
neutral = (n) / 2;
vnom = [270; 370; 270; 370; 200; 370; 0; 0; 0; 0; -348.617; 0];
u1 = [546.11,   1040.5,     548         1044.4  1048    549.17      1053.65     545.59];
u2 = [546.11,   1040.52,    547.98      1044.43 1047.96 549.2       1053.64     545.6];
v1 = [2122.87,  3124.33,    4123.45     5123.56 6124.74 7122.86     8125.27     9128.57];
v2 = [2122.86,  3124.32,    4123.5      5123.56 6124.78 7122.84     8125.29     9128.58];
u = (u1 + u2) /2;
v = (v1 + v2) /2;

for i = 1 : n
    pBase = [means(2*i - 1);means(2*i)];
    phi = ik_sym(pBase, vnom);
    phis = [phis, phi];
end

if simulation == 1
    %        L11  L12   L21 L22    D   L23      GAMA       DELTA1       DELTA2    X0 Y0     ALPHA;
    upper = [0.5; 0.5; 0.5; 0.5; 0.5; 3.0; deg2rad(3.0); deg2rad(2) ; deg2rad(2); 3; 10; deg2rad(3)];
    lower = -upper;
    random = lower + (upper - lower).*rand(12, 1);
    vactual = vnom + random;
    %vactual = vnom + [0.15; 0.21; 0.13; 0.23; 0.22; 7; deg2rad(4.1); deg2rad(6.5); deg2rad(-2.7); 43; 7; deg2rad(2.5)];
    
    for i = 1 : n
        [xtmp, ytmp] = fk_sym(vactual, [phis(1, i); phis(2, i)]);
        [xWorldTmp, yWorldTmp] = base2world(vactual, [xtmp, ytmp]);
%          means(2*i-1) = xWorldTmp;
%          means(2*i) = yWorldTmp;
        
        if (i ~= neutral)
            v(i) = v(neutral) - 50 * xWorldTmp;
            u(i) = u(neutral) - 50 * yWorldTmp;
        end
    end
end

%{u,v} -> means
counter = 0;
tmpphis = zeros(2, n - 1);
means = zeros(2*n - 2, 1);
 for i = 1 : n
     if (i ~= neutral)
        counter = counter + 1;
        xWorld = -(v(i) - v(neutral)) / 50;
        yWorld = -(u(i) - u(neutral)) / 50;
         means(2*counter-1) = xWorld;
         means(2*counter) = yWorld;

        tmpphis (2 *counter - 1) = phis(2 * i - 1);
        tmpphis(2*counter) = phis(2 * i);
     end
 end

phis = tmpphis;
n = size(means, 1) / 2;
est = zeros(size(means));
m = size(vnom, 1);

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
        [xWorldTmp, yWorldTmp] = base2world(vreal, [xtmp, ytmp]);
        est(2*k-1) = xWorldTmp;
        est(2*k) = yWorldTmp;
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
disp(vreal');

if simulation == 1
disp(vactual');
disp((vreal - vactual)');
end
