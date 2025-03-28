syms L11 L12 L21 L22 D DELTA1 DELTA2 real;
syms T1 T2 real;            %左右主动轴编码器读数
syms COSG1 COSG2 real;      %左右被动关节角余弦

skew = [ 0 -1; 1 0];

rOB1 = [L11 * cos(T1 + DELTA1) - D / 2; L11 * sin(T1 + DELTA1)];
rOB2 = [L21 * cos(T2 + DELTA2) + D / 2; L21 * sin(T2 + DELTA2)];
rB1B2 = rOB2 - rOB1;
dB1B2 = sqrt(rB1B2.' * rB1B2);
b1 = (L12^2 - L22^2 + dB1B2^2) / (2 * dB1B2);
h = sqrt(L12^2 - b1^2);
rOC = rOB1 + b1 / dB1B2 * rB1B2 + h / dB1B2 * skew * rB1B2;

rB2C = rOC - rOB2;
rB2A2 = [D/2; 0]-rOB2;
COSG2 = rB2C.'*rB2A2 / (L22 * L21);

rB1C = rOC - rOB1;
rB1A1 = [-D/2; 0]-rOB1;
COSG1 = rB1C.'*rB1A1 / (L11 * L12);

%L11 L12 L21 L22 D DELTA1 DELTA2
Jk_formula = [...
    diff(COSG1, L11), diff(COSG1, L12), diff(COSG1, L21), diff(COSG1, L22), diff(COSG1, D), diff(COSG1, DELTA1), diff(COSG1, DELTA2);...
    diff(COSG2, L11), diff(COSG2, L12), diff(COSG2, L21), diff(COSG2, L22), diff(COSG2, D), diff(COSG2, DELTA1), diff(COSG2, DELTA2)];

linear1 = simplify(-L11 / D * Jk_formula(1,1) - L12 / D * Jk_formula(1,2) - L21 / D * Jk_formula(1,3) - L22 / D * Jk_formula(1,4) - Jk_formula(1,5));
linear2 = simplify(-L11 / D * Jk_formula(2,1) - L12 / D * Jk_formula(2,2) - L21 / D * Jk_formula(2,3) - L22 / D * Jk_formula(2,4) - Jk_formula(2,5));

vnom   =  [270; 380; 280; 390; 200; 0; 0];
vdelta =  [0.3;0.2; 0.21; 0.25; 0.4; 0.02; 0.01];
vdelta = zeros(size(vnom, 1), 1);
vactual = vnom + vdelta;

bf_thetas_degs = [];

angle_step = 10;
for bf_t2_deg = 90 : -angle_step: 45
    for bf_t1_deg = 90 : angle_step : 135
        bf_thetas_degs = [bf_thetas_degs, [bf_t1_deg; bf_t2_deg]];
    end
end

n=size(bf_thetas_degs, 2);   %measure times
m = size(vactual, 1);        %运动学参数个数
measures = zeros(2*n,1);  %measured values = [cosg1; cosg2; ...]
actuals  = zeros(2*n,1);  %real values of [cosg1; cosg2; ...]
thetas   = zeros(2,n);  %measure values of encoders in radian
encoder_error = 0/3600; %in degrees

for i=1:n
    %关节角编码器测量值
    thetas(1,i) = deg2rad(bf_thetas_degs(1,i) - rad2deg(vactual(6)) - encoder_error + 2 * encoder_error * rand);
    thetas(2,i) = deg2rad(bf_thetas_degs(2,i) - rad2deg(vactual(7)) - encoder_error + 2 * encoder_error * rand);
    
    %计算真值
    actual_t1_deg = bf_thetas_degs(1,i) - rad2deg(vactual(6));
    actual_t2_deg = bf_thetas_degs(2,i) - rad2deg(vactual(7));
    actual_cosg1 = eval(subs(COSG1, [L11;L12; L21; L22; D; DELTA1; DELTA2; T1; T2], [vactual; deg2rad(actual_t1_deg); deg2rad(actual_t2_deg)]));
    actual_cosg2 = eval(subs(COSG2, [L11;L12; L21; L22; D; DELTA1; DELTA2; T1; T2], [vactual; deg2rad(actual_t1_deg); deg2rad(actual_t2_deg)]));
    actuals(2*i - 1) = actual_cosg1;
    actuals(2*i) = actual_cosg2;
    
    %计算被动角余弦测量值
    measure_g1d = acosd(actual_cosg1) - encoder_error + 2 * encoder_error * rand;
    measure_g2d = acosd(actual_cosg2) - encoder_error + 2 * encoder_error * rand;
    measure_cosg1 = cosd(measure_g1d);
    measure_cosg2 = cosd(measure_g2d);
    
    measures(2*i - 1) = measure_cosg1;
    measures(2*i) = measure_cosg2;
end

y = measures - actuals;
ACTUAL_RMSE = sqrt((y' * y )/ (2*n)); 

J = zeros(2 * n, m);
estimate = zeros(2*n,1);

vreal = vnom;
means = measures;
for i = 1:6
    for k = 1 : n %populate matrix J ; estimate using current phi
        w_phi = [vreal; thetas(1, k); thetas(2, k)]; %运动学参数；主动角测量值
        Jk = subs(Jk_formula, [L11;L12; L21; L22; D; DELTA1; DELTA2; T1; T2], w_phi);
        J(2*k-1,:) = Jk(1,:);
        J(2*k,:) = Jk(2,:);
        %使用当前运动学参数估计
        cosg1 = eval(subs(COSG1, [L11;L12; L21; L22; D; DELTA1; DELTA2; T1; T2], w_phi));
        cosg2 = eval(subs(COSG2, [L11;L12; L21; L22; D; DELTA1; DELTA2; T1; T2], w_phi));
        estimate(2*k-1) = cosg1;
        estimate(2*k) = cosg2;
    end
    
    %centralized matrix
    C = eye(n) - ones(n) / n;
    C = eye(2*n);
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
    RMSE = sqrt((y' * y )/ (2*n));
    fprintf("[%d] RMSE=%f. cond(J)=%f, cond(RX)=%f. ACTUAL_RMSE=%f\n", i, RMSE, cond(J), cond(RX), ACTUAL_RMSE);

    if (RMSE < 1E-8)
        fprintf("converge criterion meets.i=%d.\n", i);
        break;
    end
    vreal = vreal + delta;
   
   
end

disp((vreal - vactual)');