global vactual;   %真实运动学参数
global vreal;     %标定计算结果

w = vreal;
%w = vactual;
gearbox_error = deg2rad(0.00);
scale = 1;
x = scale*(-250:1:250);
y =  scale*(150:1:500);
[X,Y] = meshgrid(x,y);
Z= zeros(size(X));
Z_gearbox_error = zeros(size(X));
for r=1:size(y,2)
    for c=1:size(x,2)
        xBF = X(r, c);
        yBF = Y(r, c);
        t1t2 = ik_sym(vactual, [xBF;yBF]);  %使用真实值进行反解
        %不考虑减速器重复定位精度
        rOE_real = fk_sym(w, t1t2);
        rOE_diff = [xBF;yBF] - rOE_real;
        Z(r,c) = norm(rOE_diff);
        %叠加减速器重复定位误差
        t1load = t1t2(1) - gearbox_error + 2 * gearbox_error * rand;
        t2load = t1t2(2) - gearbox_error + 2 * gearbox_error * rand;
        rOE_real = fk_sym(w, [t1load; t2load]);
        rOE_diff = [xBF;yBF] - rOE_real;
        Z_gearbox_error(r,c) = norm(rOE_diff);
    end
end
figure('name', '无误差');
s = surfc(X,Y,Z,'EdgeColor','none');
% figure('name', '减速器误差');
% s_gearbox_error = surfc(X,Y,Z_gearbox_error,'EdgeColor','none');
