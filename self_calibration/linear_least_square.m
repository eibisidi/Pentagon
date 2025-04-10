syms L11 L12 L21 L22 D DELTA1 DELTA2 DELTA3 DELTA4 real;
syms T1 T2 real;            %左右主动轴编码器读数
syms COSG1 COSG2 real;      %左右被动关节角余弦
syms COSE1 COSE2 real;      %编码器角度对应余弦值

global measures_ag t1s t2s

encoders = 4;
do_linear_calibrate = 1;
do_nonlinear_calibrate = 0;
do_column_scaling = 1;
encoder_error = 0/360; %编码器测量误差
passive_encoder_error = 0/360;
vnom   =  [270; 370;  270;  370;    0;    0;   0;     0]; %运动学参数名义值
vdelta =  [0.5; 0.45; 0.55; 0.35; 0.03; 0.03; 0.04; 0.05]; %参数增量
%vdelta = zeros(size(vnom, 1), 1);
vactual = vnom + vdelta;
bf_thetas_degs = [];

angle_step = 5;
for bf_t2_deg = 90 : -angle_step: 25
    for bf_t1_deg = 90 : angle_step : 155
        bf_thetas_degs = [bf_thetas_degs, [bf_t1_deg; bf_t2_deg]];
    end
end

D = 200; %固定D求出杆长比例
skew = [ 0 -1; 1 0];
rOB1 = [L11 * cos(T1 + DELTA1) - D / 2; L11 * sin(T1 + DELTA1)];
rOB2 = [L21 * cos(T2 + DELTA2) + D / 2; L21 * sin(T2 + DELTA2)];
rB1B2 = rOB2 - rOB1;
dB1B2 = sqrt(rB1B2.' * rB1B2);
b1 = (L12^2 - L22^2 + dB1B2^2) / (2 * dB1B2);
h = sqrt(L12^2 - b1^2);
rOC = rOB1 + b1 / dB1B2 * rB1B2 + h / dB1B2 * skew * rB1B2;

%左侧被动角方程
rB1C = rOC - rOB1;
rB1A1 = [-D/2; 0]-rOB1;
COSG1 = rB1C.'*rB1A1 / (L11 * L12);
COSE1 = COSG1 * cos(DELTA3) + sqrt(1- COSG1*COSG1) * sin(DELTA3);

%右侧被动角方程
rB2C = rOC - rOB2;
rB2A2 = [D/2; 0]-rOB2;
COSG2 = rB2C.'*rB2A2 / (L22 * L21);
COSE2 = COSG2 * cos(DELTA4) + sqrt(1- COSG2*COSG2) * sin(DELTA4);

n=size(bf_thetas_degs, 2);   %measure times


if encoders == 4
    %Jacobians
    Jk_formula = [...
        diff(COSE1, L11), diff(COSE1, L12), diff(COSE1, L21), diff(COSE1, L22),  diff(COSE1, DELTA1), diff(COSE1, DELTA2), diff(COSE1, DELTA3), diff(COSE1, DELTA4);...
        diff(COSE2, L11), diff(COSE2, L12), diff(COSE2, L21), diff(COSE2, L22),  diff(COSE2, DELTA1), diff(COSE2, DELTA2), diff(COSE2, DELTA3), diff(COSE2, DELTA4)];
    m = size(vactual, 1);        %运动学参数个数
    measures = zeros(2*n,1);     %measured values = [cosg1; cosg2; ...]
    actuals  = zeros(2*n,1);     %real values of [cosg1; cosg2; ...]
    J = zeros(2 * n, m);
    estimate = zeros(2*n,1);
    measures_ag = zeros(2*n,1);  %measured values = [cosg1; cosg1; ...cosg2; cosg2]
else
    Jk_formula = [...
        diff(COSE1, L11), diff(COSE1, L12), diff(COSE1, L21), diff(COSE1, L22),  diff(COSE1, DELTA1), diff(COSE1, DELTA2), diff(COSE1, DELTA3)];
    vnom = vnom(1:size(vnom,1)-1, :);
    vactual = vactual(1:size(vactual,1)-1, :);
    m = size(vactual, 1);        %运动学参数个数
    measures = zeros(n,1);     %measured values = [cosg1; cosg2; ...]
    actuals  = zeros(n,1);     %real values of [cosg1; cosg2; ...]
    J = zeros(n, m);
    estimate = zeros(n,1);
end
thetas   = zeros(2,n);       %measure values of encoders in radian

%仿真数据生成
for i=1:n
    %关节角编码器测量值
    thetas(1,i) = deg2rad(bf_thetas_degs(1,i) - rad2deg(vactual(5)) - encoder_error + 2 * encoder_error * rand);
    thetas(2,i) = deg2rad(bf_thetas_degs(2,i) - rad2deg(vactual(6)) - encoder_error + 2 * encoder_error * rand);
    
    %计算真值
    actual_t1_deg = bf_thetas_degs(1,i) - rad2deg(vactual(5));
    actual_t2_deg = bf_thetas_degs(2,i) - rad2deg(vactual(6));
    
    if encoders == 4
        actual_cose1 = eval(subs(COSE1, [L11;L12; L21; L22; DELTA1; DELTA2; DELTA3; DELTA4; T1; T2], [vactual; deg2rad(actual_t1_deg); deg2rad(actual_t2_deg)]));
        actual_cose2 = eval(subs(COSE2, [L11;L12; L21; L22; DELTA1; DELTA2; DELTA3; DELTA4; T1; T2], [vactual; deg2rad(actual_t1_deg); deg2rad(actual_t2_deg)]));
        %计算被动编码器角余弦测量值
        measure_e1d = acosd(actual_cose1) - passive_encoder_error + 2 * passive_encoder_error * rand;
        measure_e2d = acosd(actual_cose2) - passive_encoder_error + 2 * passive_encoder_error * rand;
        actuals(2*i - 1) = actual_cose1;
        actuals(2*i)     = actual_cose2;
        measures(2*i - 1) = cosd(measure_e1d); %奇数行为左侧编码器角余弦
        measures(2*i)     = cosd(measure_e2d); %偶数行为右侧编码器角余弦
        measures_ag(i)    = measures(2*i - 1);
        measures_ag(n+i)  = measures(2*i);
    else
        actual_cose1 = eval(subs(COSE1, [L11;L12; L21; L22; DELTA1; DELTA2; DELTA3; T1; T2], [vactual; deg2rad(actual_t1_deg); deg2rad(actual_t2_deg)]));
        %计算被动编码器角余弦测量值
        measure_e1d = acosd(actual_cose1) - passive_encoder_error + 2 * passive_encoder_error * rand;
        actuals(i) = actual_cose1;
        measures(i) = cosd(measure_e1d);
    end
end

y = measures - actuals;
ACTUAL_RMSE = sqrt((y' * y )/ (size(y, 1)));

if do_linear_calibrate == 1
    vreal = vnom;
    means = measures;
    for i = 1:6
        for k = 1 : n %populate matrix J ; estimate using current phi
            w_phi = [vreal; thetas(1, k); thetas(2, k)]; %运动学参数；主动角测量值
            if encoders == 4
                Jk = subs(Jk_formula, [L11;L12; L21; L22; DELTA1; DELTA2; DELTA3; DELTA4; T1; T2], w_phi);
                %使用当前运动学参数估计
                cose1 = eval(subs(COSE1, [L11;L12; L21; L22; DELTA1; DELTA2; DELTA3; DELTA4; T1; T2], w_phi));
                cose2 = eval(subs(COSE2, [L11;L12; L21; L22; DELTA1; DELTA2; DELTA3; DELTA4; T1; T2], w_phi));
                J(2*k-1,:) = Jk(1,:);
                J(2*k,:) = Jk(2,:);
                estimate(2*k-1) = cose1;
                estimate(2*k)   = cose2;
            else
                Jk = subs(Jk_formula, [L11;L12; L21; L22; DELTA1; DELTA2; DELTA3; T1; T2], w_phi);
                %使用当前运动学参数估计
                cose1 = eval(subs(COSE1, [L11;L12; L21; L22; DELTA1; DELTA2; DELTA3; T1; T2], w_phi));
                J(k,:) = Jk(1,:);
                estimate(k) = cose1;
            end
        end
        if do_column_scaling == 1
            %centralized matrix
            C = eye(size(y, 1)) - ones(size(y, 1)) / (size(y, 1));
            %C = eye(size(y, 1));
            X = C * J; %centralized matrix
            
            %do column scaling
            d = zeros(m, 1);
            for c = 1 : m
                d(c) = norm(X(:, c));
            end
            
            csMatrix = diag(d);                 %column scaling matrix
            Xnorm = X / (csMatrix);             %X*inv(D)
            RX = Xnorm' * Xnorm;                %correlation coefficient matrix
            y = means - estimate;
            Cy = C * (means - estimate);
            delta_norm = Xnorm \ Cy;        %Cy = Xnorm * delta_norm
            delta = csMatrix \ delta_norm;  %delta_norm = D * delta
        else
            y = means - estimate;
            Xnorm = J;
            delta = Xnorm \ y;
        end
        RMSE = sqrt((y' * y )/ (size(y, 1)));
        fprintf("[%d] RMSE=%f. cond(J)=%f, cond(Jnorm)=%f. ACTUAL_RMSE=%f\n", i, RMSE, cond(J), cond(Xnorm), ACTUAL_RMSE);
        
        if (RMSE < 1E-16)
            fprintf("converge criterion meets.i=%d.\n", i);
            break;
        end
        vreal = vreal + delta;
        
    end
    
    disp((vreal - vactual)');
end

if do_nonlinear_calibrate == 1
    t1s = thetas(1, :)';
    t2s = thetas(2, :)';
    x0 = vnom;
    options = optimoptions(@lsqnonlin,'Algorithm','trust-region-reflective', 'OptimalityTolerance', 1.000000e-7);
    [x,resnorm,residual,exitflag,output] = lsqnonlin(@eag,x0,[],[],options);
    disp(x' - vactual');
end
