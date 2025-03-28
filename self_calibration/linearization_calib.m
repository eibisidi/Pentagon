syms L11 L12  L22 D B2X B2Y DELTA1 real;
syms COSG G real;
syms T1 T2 real;

skew = [ 0 -1; 1 0];

rOB1 = [L11 * cos(T1 + DELTA1) - D / 2; L11 * sin(T1 + DELTA1)];
rOB2 = [B2X; B2Y];
rB1B2 = rOB2 - rOB1;
dB1B2 = sqrt(rB1B2' * rB1B2);
b1 = (L12^2 - L22^2 + dB1B2^2) / (2 * dB1B2);
h = sqrt(L12^2 - b1^2);
rOC = rOB1 + b1 / dB1B2 * rB1B2 + h / dB1B2 * skew * rB1B2;
rB1C = rOC - rOB1;
rB1A1 = [-D/2; 0]-rOB1;
COSG = rB1C'*rB1A1 / (L11 * L12);

Jk_formula = [diff(COSG, L11), diff(COSG, L12), diff(COSG, L22), diff(COSG, D),...
    diff(COSG, B2X), diff(COSG, B2Y)...
    diff(COSG, DELTA1)];

vnom   =  [270; 370; 370;  200;  100; 270; 0;];
%vdelta =  [0.4; -0.4; 0.37; 0.45; 0; 1; 0.04; 0.07; 0.037;    0; 0; 0];
vdelta = zeros(size(vnom, 1), 1);
vactual = vnom + vdelta;

n=55;   %measure times
measures = zeros(n,1);  %measured values = cosG
actuals  = zeros(n,1);  %realvalues of cosG
t1       = zeros(n,1);  %measure values of t1
encoder_error = 0/3600; %in degrees
for i=1:n
    actual_t1_d = 89 + i;
    t1(i) = deg2rad(actual_t1_d - encoder_error + 2 * encoder_error * rand);
    
    actuals(i) =  eval(subs(COSG, [L11; L12; L22; D; B2X; B2Y; DELTA1;T1], [vactual; deg2rad(actual_t1_d)]));
    cosg = eval(subs(COSG, [L11; L12; L22; D; B2X; B2Y; DELTA1;T1], [vactual; t1(i)]));
    measure_g_d = acosd(cosg) - encoder_error + 2 * encoder_error * rand;
    measures(i) = cosd(measure_g_d);
end

m = size(vactual, 1);
y = measures - actuals;
ACTUAL_RMSE = sqrt((y' * y )/ (n)); 

J = zeros(n, m);
estimate = zeros(n,1);

vreal = vnom;
means = measures;
for i = 1:6
    for k = 1 : n %populate matrix J ; estimate using current phi
        w_phi = [vreal; t1(k)];
        Jk = subs(Jk_formula, [L11; L12; L22; D; B2X; B2Y; DELTA1;T1], w_phi);
        J(k,:) = Jk;
        cosg = eval(subs(COSG, [L11; L12; L22; D; B2X; B2Y; DELTA1;T1], [vreal; t1(k)]));
        estimate(k) = cosg;
    end
    
    %centralized matrix
    %C = eye(n) - ones(n) / n;
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