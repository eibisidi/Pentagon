syms L11 L12 L21 L22 D DELTA1 DELTA2 real;
syms T1 T2 real;
syms COSG G real;

D = 200;
skew = [ 0 -1; 1 0];

rOB1 = [L11 * cos(T1 + DELTA1) - D / 2; L11 * sin(T1 + DELTA1)];
rOB2 = [L21 * cos(T2 + DELTA2) + D / 2; L21 * sin(T2 + DELTA2)];
rB1B2 = rOB2 - rOB1;
dB1B2 = sqrt(rB1B2' * rB1B2);
b1 = (L12^2 - L22^2 + dB1B2^2) / (2 * dB1B2);
h = sqrt(L12^2 - b1^2);
rOC = rOB1 + b1 / dB1B2 * rB1B2 + h / dB1B2 * skew * rB1B2;
rB2C = rOC - rOB2;
rB2A2 = [D/2; 0]-rOB2;
COSG = rB2C'*rB2A2 / (L22 * L21);
G = acos(COSG);
Jk_formula = [diff(G, L11), diff(G, L12), diff(G, L21),  diff(G, L22), diff(G, DELTA1), diff(G, DELTA2)];


n = 13;
calib_points = zeros(2, n); %{BF}
measures = zeros(1, n);
thetas = zeros(2, n);
vnom =   [270; 370;  270;   370;  200;   370;     0;     0;     0;    0; 0; 0];
vdelta =  [0.4; -0.4; 0.37; 0.45; 0; 1; 0.04; 0.07; 0.037;    0; 0; 0];
vactual = vnom + vdelta;

vactual_less = [vactual(1); vactual(2); vactual(3); vactual(4); vactual(8); vactual(9);];
m = size(vactual_less, 1);
offset_error = 0.001;
angle_error = 0.00001;

xrange = linspace(-260, 260, n);
yrange = linspace(350, 350, n);
%calib_points = saved_points;
for i = 1 : n
    xtmp = xrange(i) + 10 * rand;
     ytmp = 348.617 - 5 +  10 * rand;
     xtmp = xrange(i);
     ytmp = yrange(i);
    calib_points(:, i) = [xtmp; ytmp];
    phitmp = ik_sym(vnom, calib_points(:,i));
    thetas(:, i) = phitmp;
    bf_tmp = fk_sym(vactual, phitmp);
    wf_tmp = base2world(vactual, bf_tmp);
    
    g = get_gama(vactual_less, phitmp);
    measures(i) = g - angle_error + 2 * angle_error * rand;
end

means = measures(1,:)';
estimate = zeros(n, 1);
J = zeros(n, m);

for k = 1 : n %populate matrix J ; estimate est vector
    g = get_gama(vactual_less, [thetas(1, k); thetas(2, k)]);
    estimate(k) = g;
end
y = means - estimate;
ACTUAL_RMSE = sqrt((y' * y )/ (n)); 

vreal = [vnom(1); vnom(2); vnom(3); vnom(4); vnom(8); vnom(9)];
for i = 1:6
    for k = 1 : n %populate matrix J ; estimate est vector
        w_phi = [vreal; thetas(1, k); thetas(2, k)];
        Jk = subs(Jk_formula, [L11; L12; L21; L22; DELTA1; DELTA2; T1; T2], w_phi);
        J(k,:) = Jk;
        g = get_gama(vreal, [thetas(1, k); thetas(2, k)]);
        estimate(k) = g;
    end
    
%     Jx = [J(:,1:4), J(:, 6:7)];
    

    %centralized matrix
    C = eye(n) - ones(n) / n;
    C = eye(n);
    X = C * J; %centralized matrix
    
    %do column scaling
    d = zeros(m, 1);
    for c = 1 : m
        d(c) = norm(X(:, c));
    end
    
    csMatrix = diag(d); %column scaling matrix
    Xnorm = X / (csMatrix);
    RX = Xnorm' * Xnorm;
    y = means - estimate;
    Cy = C * (means - estimate);
    delta_norm = (RX) \ (Xnorm' * Cy);
    delta = csMatrix \ delta_norm;
    
    vreal = vreal + delta;
   
    RMSE = sqrt((y' * y )/ (n));
    fprintf("[%d] RMSE=%f. cond(J)=%f, cond(RX)=%f. ACTUAL_RMSE=%f\n", i, RMSE, cond(J), cond(RX), ACTUAL_RMSE);

    if (RMSE < 1E-16)
        fprintf("converge criterion meets.i=%d.\n", i);
        break;
    end
end

disp((vreal - vactual_less)');