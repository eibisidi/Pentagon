%使用CMM测量末端位置标定运动学参数
clear;
syms L11 L12 L21 L22 D L23 GAMA DELTA1 DELTA2 X0 Y0 ALPHA real;
syms T1 T2 real;
global measures_ag t1s t2s; %非线性最小二乘需要使用的全局数据
global vactual;             %真实运动学参数
global vreal;               %标定结果
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
Jk_formula = [diff(xWF, L11), diff(xWF, L12), diff(xWF, L21),  diff(xWF, L22), diff(xWF, D), diff(xWF, L23), diff(xWF, GAMA), diff(xWF, DELTA1), diff(xWF, DELTA2), diff(xWF, X0), diff(xWF, Y0), diff(xWF, ALPHA);
              diff(yWF, L11), diff(yWF, L12), diff(yWF, L21),  diff(yWF, L22), diff(yWF, D), diff(yWF, L23), diff(yWF, GAMA), diff(yWF, DELTA1), diff(yWF, DELTA2), diff(yWF, X0), diff(yWF, Y0), diff(yWF, ALPHA)];

do_linear_calibrate = 0;
do_nonlinear_calibrate = 1;
do_column_scaling = 1;
encoder_error = 2/3600;             %编码器测量误差
cmm_error =0.02;                    %CMM坐标测量仪噪声
%          L11      L12     L21      L22        D     L23    GAMA DELTA1  DELTA2   X0    Y0   ALPHA
vnom   =  [270;     370;   270;      370;     200;   370;      0;     0;      0;    0;    0;     0]; %运动学参数名义值
vdelta =  [0.05;   0.04;   0.03;   -0.03;    0.16;   2.4;   0.04;  0.05;   0.02;    3;    3;  0.02]; %参数增量
%vdelta = zeros(size(vnom, 1), 1);
vactual = vnom + vdelta;
bf_thetas_degs = [];

%左右两个关节构成的Observation Strategies
angle_step = 3;
for bf_t2_deg = 90 : -angle_step: 25
    for bf_t1_deg = 90 : angle_step : 155
          bf_thetas_degs = [bf_thetas_degs, [bf_t1_deg; bf_t2_deg]];
    end
end
n=size(bf_thetas_degs, 2);   %measure times
m = size(vactual, 1);        %运动学参数个数
measures = zeros(2*n,1);     %measured values = [cosg1; cosg2; ...]
actuals  = zeros(2*n,1);     %real values of [cosg1; cosg2; ...]
J = zeros(2 * n, m);
estimate = zeros(2*n,1);
thetas   = zeros(2,n);       %measure values of encoders in radian
measures_ag = zeros(2*n,1);  %measured values = [xWF1; xWF2; ...yWF1; yWF2...]
%仿真数据生成
for i=1:n
    %关节角编码器测量值
    thetas(1,i) = deg2rad(bf_thetas_degs(1,i) - rad2deg(vactual(8)) - encoder_error + 2 * encoder_error * rand);
    thetas(2,i) = deg2rad(bf_thetas_degs(2,i) - rad2deg(vactual(9)) - encoder_error + 2 * encoder_error * rand);
    
    actual_t1_deg = bf_thetas_degs(1,i) - rad2deg(vactual(8));
    actual_t2_deg = bf_thetas_degs(2,i) - rad2deg(vactual(9));
    %计算真值
    actual_xWF =  eval(subs(xWF, [L11;L12;L21;L22;D;L23;GAMA;DELTA1;DELTA2;X0;Y0;ALPHA;T1;T2], [vactual; deg2rad(actual_t1_deg); deg2rad(actual_t2_deg)]));
    actual_yWF =  eval(subs(yWF, [L11;L12;L21;L22;D;L23;GAMA;DELTA1;DELTA2;X0;Y0;ALPHA;T1;T2], [vactual; deg2rad(actual_t1_deg); deg2rad(actual_t2_deg)]));
    
    actuals(2*i - 1) = actual_xWF;
    actuals(2*i)     = actual_yWF;
	measures(2*i - 1) = actual_xWF - cmm_error + 2 * cmm_error * rand; %奇数行为x坐标
    measures(2*i)     = actual_yWF - cmm_error + 2 * cmm_error * rand; %偶数行为y坐标
    measures_ag(i)    = measures(2*i - 1);
    measures_ag(n+i)  = measures(2*i);
end

y = measures - actuals;
ACTUAL_RMSE = sqrt((y' * y )/ (size(y, 1)));

if do_linear_calibrate == 1
    vreal = vnom;
    means = measures;
    for i = 1:6
        for k = 1 : n %populate matrix J ; estimate using current phi
            w_phi = [vreal; thetas(1, k); thetas(2, k)]; %运动学参数；主动角测量值
            Jk = subs(Jk_formula, [L11;L12;L21;L22;D;L23;GAMA;DELTA1;DELTA2;X0;Y0;ALPHA;T1;T2], w_phi);
            %使用当前运动学参数估计
            xwf = eval(subs(xWF, [L11;L12;L21;L22;D;L23;GAMA;DELTA1;DELTA2;X0;Y0;ALPHA;T1;T2], w_phi));
            ywf = eval(subs(yWF, [L11;L12;L21;L22;D;L23;GAMA;DELTA1;DELTA2;X0;Y0;ALPHA;T1;T2], w_phi));
            J(2*k-1,:) = Jk(1,:);
            J(2*k,:) = Jk(2,:);
            estimate(2*k-1) = xwf;
            estimate(2*k)   = ywf;
           
        end
        if do_column_scaling == 1
            %centralized matrix
            C = eye(size(y, 1)) - ones(size(y, 1)) / (size(y, 1));
            C = eye(size(y, 1));
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
            RX = Xnorm' * Xnorm;
            delta = Xnorm \ y;
        end
        ORX = (det(RX)^(1/(2*m)))/sqrt(n);       %use J or Xnorm to compute O1?
        O1 = (det(J'*J)^(1/(2*m)))/sqrt(n);   
        RMSE = sqrt((y' * y )/ (size(y, 1)));   %Observation Index 1
        fprintf("[%d] RMSE=%f. cond(J)=%f, cond(Jnorm)=%f, O1=%f.ORX=%f ACTUAL_RMSE=%f\n", i, RMSE, cond(J), cond(Xnorm), O1,ORX, ACTUAL_RMSE);
        
        if (RMSE < 1E-16)
            fprintf("converge criterion meets.i=%d.\n", i);
            break;
        end
        vreal = vreal + delta;
        
    end
    
    disp((vreal - vactual)');
    disp(norm(vreal - vactual));
end

if do_nonlinear_calibrate == 1
    t1s = thetas(1, :)';
    t2s = thetas(2, :)';
    x0 = vnom;  %非线性最小二乘法初始值
    options = optimoptions(@lsqnonlin,'Algorithm','trust-region-reflective', 'OptimalityTolerance', 1.000000e-6);
    [vreal,resnorm,residual,exitflag,output] = lsqnonlin(@eag_cmm,x0,[],[],options);
    diff = (vreal - vactual);
    disp(diff');
    fprintf("L11=%f, L12=%f, L21=%f, L22=%f, D=%f\n", diff(1), diff(2), diff(3), diff(4), diff(5));
    fprintf("L23=%f, GAMA=%f(deg)\n", diff(6), rad2deg(diff(7)));
    fprintf("DELTA1=%f(deg), DELTA2=%f(deg)\n", rad2deg(diff(8)), rad2deg(diff(9)));
    fprintf("X0=%f, Y0=%f, ALPHA=%f(deg)\n", diff(10), diff(11),  rad2deg(diff(12)));
end