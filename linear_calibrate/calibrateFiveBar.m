syms L11 L12 L21 L22 D L23 GAMA DELTA1 DELTA2 X0 Y0 ALPHAA real;
syms T1 T2 real;

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
pE = [X0 + xBF * cos(ALPHAA) - yBF * sin(ALPHAA); Y0 + xBF * sin(ALPHAA) + yBF * cos(ALPHAA)];
xWF = pE(1);
yWF = pE(2);
% Jk_formula = [diff(xWF, L11), diff(xWF, L12), diff(xWF, L21),  diff(xWF, L22), diff(xWF, D), diff(xWF, L23), diff(xWF, GAMA), diff(xWF, DELTA1), diff(xWF, DELTA2), diff(xWF, ALPHAA), diff(xWF, X0), diff(xWF, Y0) ;
%               diff(yWF, L11), diff(yWF, L12), diff(yWF, L21),  diff(yWF, L22), diff(yWF, D), diff(yWF, L23), diff(yWF, GAMA), diff(yWF, DELTA1), diff(yWF, DELTA2), diff(yWF, ALPHAA), diff(yWF, X0), diff(yWF, Y0) ];
Jk_formula = [diff(xWF, L11), diff(xWF, L12), diff(xWF, L21),  diff(xWF, L22), diff(xWF, D), diff(xWF, L23), diff(xWF, GAMA), diff(xWF, DELTA1), diff(xWF, DELTA2)];
n = 30;
calib_points = zeros(2, n); %{BF}
measures = zeros(2, n);
thetas = zeros(2, n);
vnom =   [270; 370;  270;   370;  200;   370;     0;     0;     0;    0; 0; 0];
vdelta =  [0.2; -0.2; 0.3; 0.25; 0.31; 1; 0.04; 0.07; 0.037;    0; 0; 0];
vactual = vnom + vdelta;
m = 9;
offset_error = 0.0005;

xrange = linspace(-260, 260, n);
yrange = linspace(350, 360, n);
%calib_points = saved_points;
for i = 1 : n
    xtmp = xrange(i) + 10 * rand;
     ytmp = 348.617 - 5 +  10 * rand;
    calib_points(:, i) = [xtmp; ytmp];
    phitmp = ik_sym(vnom, calib_points(:,i));
    thetas(:, i) = phitmp;
    bf_tmp = fk_sym(vactual, phitmp);
    wf_tmp = base2world(vactual, bf_tmp);
    
    measures(:, i) = [0;0];
    for j = 1 :1
        mea = wf_tmp + (-offset_error + (offset_error + offset_error)*rand(2,1));
        measures(:, i) = measures(:, i) + mea;
    end
    measures(:, i) = measures(:, i) / 1;
end
saved_points = calib_points;
means = measures(1,:)';
estimate = zeros(n, 1);
J = zeros(n, m);
vreal = vnom;

for k = 1 : n %populate matrix J ; estimate est vector
    w_phi = [vreal; thetas(1, k); thetas(2, k)];
    Jk = subs(Jk_formula, [L11; L12; L21; L22; D; L23; GAMA; DELTA1; DELTA2; ALPHAA; X0; Y0; T1; T2], w_phi);
    
    bf = fk_sym(vactual, [thetas(1, k); thetas(2, k)]);
    %wf = base2world(vreal, bf);
    wf = bf;
    estimate(k) = wf(1);
end
y = means - estimate;
ACTUAL_RMSE = sqrt((y' * y )/ (n)); 

for i = 1:6
    for k = 1 : n %populate matrix J ; estimate est vector
        w_phi = [vreal; thetas(1, k); thetas(2, k)];
        Jk = subs(Jk_formula, [L11; L12; L21; L22; D; L23; GAMA; DELTA1; DELTA2; ALPHAA; X0; Y0; T1; T2], w_phi);
        J(k,:) = Jk;
       
        bf = fk_sym(vreal, [thetas(1, k); thetas(2, k)]);
        %wf = base2world(vreal, bf);
        wf = bf;
        estimate(k) = wf(1);
    end
    
    
    if i == 1
        sigmas = svd(J);
        product = prod(sigmas);
        O1 = (product ^(1/m)) / sqrt(n);
    end
    
    %do column scaling
    d = zeros(m, 1);
    for c = 1 : m
        d(c) = norm(J(:, c));
    end
    csMatrix = diag(d); %column scaling matrix
    Jnorm = J /(csMatrix); %Jnorm = J * inv(csMatrix);
    RJ = Jnorm' * Jnorm; %RJ is correlation matrix
    
    y = means - estimate;
    delta_norm = (RJ) \ (Jnorm' * y);
    delta = csMatrix \ delta_norm;
    vreal = vreal + [delta; 0; 0; 0];
    
    
    
    RMSE = sqrt((y' * y )/ (n));
    fprintf("[%d] RMSE=%f. cond(J)=%f, cond(RJ)=%f. ACTUAL_RMSE=%f\n", i, RMSE, cond(J), cond(RJ), ACTUAL_RMSE);

    if (RMSE < 1E-16)
        fprintf("converge criterion meets.i=%d.\n", i);
        break;
    end
end

disp((vreal - vactual)');