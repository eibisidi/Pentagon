

% % ft = fittype( 'surfaceToFit(x, y, l11, l12, l21, l22, delta1, delta2, delta3)', ...
% %      'independent', {'x', 'y'}, 'dependent', 'z' );
% ft = fittype( 'sin(DELTA3)*(1 - (L11*cos(DELTA1 + x)*(((L21*cos(DELTA2 + y) - L11*cos(DELTA1 + x) + 200)*(L12^2 - L22^2 + (L21*cos(DELTA2 + y) - L11*cos(DELTA1 + x) + 200)^2 + (L11*sin(DELTA1 + x) - L21*sin(DELTA2 + y))^2))/(2*((L21*cos(DELTA2 + y) - L11*cos(DELTA1 + x) + 200)^2 + (L11*sin(DELTA1 + x) - L21*sin(DELTA2 + y))^2)) + ((L12^2 - (L12^2 - L22^2 + (L21*cos(DELTA2 + y) - L11*cos(DELTA1 + x) + 200)^2 + (L11*sin(DELTA1 + x) - L21*sin(DELTA2 + y))^2)^2/(4*((L21*cos(DELTA2 + y) - L11*cos(DELTA1 + x) + 200)^2 + (L11*sin(DELTA1 + x) - L21*sin(DELTA2 + y))^2)))^(1/2)*(L11*sin(DELTA1 + x) - L21*sin(DELTA2 + y)))/((L21*cos(DELTA2 + y) - L11*cos(DELTA1 + x) + 200)^2 + (L11*sin(DELTA1 + x) - L21*sin(DELTA2 + y))^2)^(1/2)) - L11*sin(DELTA1 + x)*(((L11*sin(DELTA1 + x) - L21*sin(DELTA2 + y))*(L12^2 - L22^2 + (L21*cos(DELTA2 + y) - L11*cos(DELTA1 + x) + 200)^2 + (L11*sin(DELTA1 + x) - L21*sin(DELTA2 + y))^2))/(2*((L21*cos(DELTA2 + y) - L11*cos(DELTA1 + x) + 200)^2 + (L11*sin(DELTA1 + x) - L21*sin(DELTA2 + y))^2)) - ((L12^2 - (L12^2 - L22^2 + (L21*cos(DELTA2 + y) - L11*cos(DELTA1 + x) + 200)^2 + (L11*sin(DELTA1 + x) - L21*sin(DELTA2 + y))^2)^2/(4*((L21*cos(DELTA2 + y) - L11*cos(DELTA1 + x) + 200)^2 + (L11*sin(DELTA1 + x) - L21*sin(DELTA2 + y))^2)))^(1/2)*(L21*cos(DELTA2 + y) - L11*cos(DELTA1 + x) + 200))/((L21*cos(DELTA2 + y) - L11*cos(DELTA1 + x) + 200)^2 + (L11*sin(DELTA1 + x) - L21*sin(DELTA2 + y))^2)^(1/2)))^2/(L11^2*L12^2))^(1/2) - (cos(DELTA3)*(L11*cos(DELTA1 + x)*(((L21*cos(DELTA2 + y) - L11*cos(DELTA1 + x) + 200)*(L12^2 - L22^2 + (L21*cos(DELTA2 + y) - L11*cos(DELTA1 + x) + 200)^2 + (L11*sin(DELTA1 + x) - L21*sin(DELTA2 + y))^2))/(2*((L21*cos(DELTA2 + y) - L11*cos(DELTA1 + x) + 200)^2 + (L11*sin(DELTA1 + x) - L21*sin(DELTA2 + y))^2)) + ((L12^2 - (L12^2 - L22^2 + (L21*cos(DELTA2 + y) - L11*cos(DELTA1 + x) + 200)^2 + (L11*sin(DELTA1 + x) - L21*sin(DELTA2 + y))^2)^2/(4*((L21*cos(DELTA2 + y) - L11*cos(DELTA1 + x) + 200)^2 + (L11*sin(DELTA1 + x) - L21*sin(DELTA2 + y))^2)))^(1/2)*(L11*sin(DELTA1 + x) - L21*sin(DELTA2 + y)))/((L21*cos(DELTA2 + y) - L11*cos(DELTA1 + x) + 200)^2 + (L11*sin(DELTA1 + x) - L21*sin(DELTA2 + y))^2)^(1/2)) - L11*sin(DELTA1 + x)*(((L11*sin(DELTA1 + x) - L21*sin(DELTA2 + y))*(L12^2 - L22^2 + (L21*cos(DELTA2 + y) - L11*cos(DELTA1 + x) + 200)^2 + (L11*sin(DELTA1 + x) - L21*sin(DELTA2 + y))^2))/(2*((L21*cos(DELTA2 + y) - L11*cos(DELTA1 + x) + 200)^2 + (L11*sin(DELTA1 + x) - L21*sin(DELTA2 + y))^2)) - ((L12^2 - (L12^2 - L22^2 + (L21*cos(DELTA2 + y) - L11*cos(DELTA1 + x) + 200)^2 + (L11*sin(DELTA1 + x) - L21*sin(DELTA2 + y))^2)^2/(4*((L21*cos(DELTA2 + y) - L11*cos(DELTA1 + x) + 200)^2 + (L11*sin(DELTA1 + x) - L21*sin(DELTA2 + y))^2)))^(1/2)*(L21*cos(DELTA2 + y) - L11*cos(DELTA1 + x) + 200))/((L21*cos(DELTA2 + y) - L11*cos(DELTA1 + x) + 200)^2 + (L11*sin(DELTA1 + x) - L21*sin(DELTA2 + y))^2)^(1/2))))/(L11*L12)', ...
%     'independent', {'x', 'y'}, 'dependent', 'z' )
% opts = fitoptions( ft );
% opts.Robust = 'LAR';
% opts.StartPoint = [0, 0, 0, 270, 370, 270, 370];      %coeffnames(ft) to see coeff orders
% 
% [fs, gof] = fit( thetas', measures, ft, opts);
% 
% plot(fs, thetas', measures, 'XLim', [pi/2, pi], 'YLim', [0, pi/2]);

% load franke
% sf = fit([x, y],z,'poly23')
% plot(sf)

% rng default % for reproducibility
% d = linspace(0,3);
% y = exp(-1.3*d) + 0.05*randn(size(d));
% fun = @(r)exp(-d*r)-y;
% x0 = 4;
% x = lsqnonlin(fun,x0);
% plot(d,y,'ko',d,exp(-x*d),'b-')
% legend('Data','Best fit')
% xlabel('t')
% ylabel('exp(-tx)')

% t = linspace(-4,4);
% y = 1/sqrt(2*pi)*exp(-t.^2/2);
% fun = @(x)x(1)*exp(-t).*exp(-exp(-(t-x(2)))) - y;
% lb = [1/2,-1];
% ub = [3/2,3];
% x0 = [1/2,0];
% x = lsqnonlin(fun,x0,lb,ub)
% plot(t,y,'r-',t,fun(x)+y,'b-')
% xlabel('t')
% legend('Normal density','Fitted function')

%x0 = vnom +[0.3; 0.2; 0.21; 0.25; 0.02; 0.01; 0.04; 0.02]; %参数增量;
x0 = vnom;
options = optimoptions(@lsqnonlin,'Algorithm','trust-region-reflective');
x = lsqnonlin(@eag,x0,[],[],options);