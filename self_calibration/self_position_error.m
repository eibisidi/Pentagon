global vactual;   %真实运动学参数
global vreal;     %标定计算结果
%自标定的运动学参数与CMM格式不同
%self L11 L12 L21 L22 DELTA1 DELTA2 DELTA3 DELTA4
%cmm  L11 L12 L21 L22 D L23 GAMA DELTA1 DELTA2 X0 Y0 ALPHA
w_actual= [vactual(1); vactual(2); vactual(3); vactual(4); 200; vactual(4); 0; vactual(5); vactual(6); 0; 0; 0];
w_calib = [vreal(1);    vreal(2); vreal(3); vreal(4); 200; vreal(4); 0; vreal(5); vreal(6); 0; 0; 0];
%w = w_actual;
gearbox_error = deg2rad(0.00);
scale = 1;
x = scale*(-250:1:250);
y =  scale*(150:1:500);
[X,Y] = meshgrid(x,y);
Z_xBF = zeros(size(X));
Z_yBF = zeros(size(X));
Z_dist= zeros(size(X));             %距离误差
Z_gearbox_error = zeros(size(X));
for r=1:size(y,2)
    for c=1:size(x,2)
        xBF = X(r, c);
        yBF = Y(r, c);
        t1t2 = ik_sym(w_actual, [xBF;yBF]);  %使用真实值进行反解
        %不考虑减速器重复定位精度
        rOE_real = fk_sym(w_calib, t1t2);
        rOE_diff =  rOE_real - [xBF;yBF] ;
        Z_dist(r,c) = norm(rOE_diff);
        Z_xBF(r,c)   = rOE_diff(1);
        Z_yBF(r,c)   = rOE_diff(2);
        %叠加减速器重复定位误差
        if gearbox_error > 0
            t1load = t1t2(1) - gearbox_error + 2 * gearbox_error * rand;
            t2load = t1t2(2) - gearbox_error + 2 * gearbox_error * rand;
            rOE_real = fk_sym(w_calib, [t1load; t2load]);
            rOE_diff = [xBF;yBF] - rOE_real;
            Z_gearbox_error(r,c) = norm(rOE_diff);
        end
    end
end
figure('name', 'Position Error Length');
sLen = surfc(X,Y,Z_dist,'EdgeColor','none');
figure('name', 'Position Error in XY');
sX = surf(X,Y,Z_xBF,'EdgeColor','none');
hold on;
%figure('name', 'Position Error in Y');
sY = surf(X,Y,Z_yBF,'EdgeColor','none');
% figure('name', '减速器误差');
% s_gearbox_error = surfc(X,Y,Z_gearbox_error,'EdgeColor','none');
