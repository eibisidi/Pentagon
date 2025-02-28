syms L11 L12 L21 L22 D L23 GAMA DELTA1 DELTA2 X0 Y0 ALPHA real;
syms T1 T2 real;
simulation = 1;
ybaseline = 200;
alphabase = 0;
y0 = 200;
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

samples=[];
counter = 0;
neutral = 0;
plotX = [];
plotY = [];
% for i = 260:-20:-260
%     counter = counter + 1;
%     ytmp = ybaseline + mod(counter, 6) * 50;
%     if (i == 0)
%         neutral = counter;
%         y0 = ytmp;
%     end
%     
%     plotX = [plotX; i];
%     plotY = [plotY; ytmp];
%     means = [means; i];
%     means = [means; ytmp];
% end

for i = 260:-20:-260
    counter = counter + 1;
    %ytmp = ybaseline + 200 - counter * 5 + mod(counter, 2) * 100;
    %ytmp = ybaseline + 200 - counter * 5 + mod(counter, 2) * 10;  %O1 = 0.3200
    %ytmp = ybaseline + 200 - counter * 5 + mod(counter, 4) * 40;  %O1 = 0.6984
    ytmp = ybaseline + 300 * rand;
    if (i == 0)
        neutral = counter;
        y0 = ytmp;
    end
    
    plotX = [plotX; i];
    plotY = [plotY; ytmp];
    samples = [samples; i];
    samples = [samples; ytmp];
end

samples = maxPoints;
neutral = 14;
y0 = samples(2 * neutral);

plot(plotX, plotY);

% means = [60; 348.617 + 10;
%     40; 348.617 ;
%     20; 348.617 + 10;
%     0; 348.617; %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%     -20; 348.617;
%     -40; 348.617 + 10;
%     -60; 348.617;
%     -80; 348.617 + 10];
% neutral = 4;
% y0 = 348.617;

phis = [];
n = size(samples, 1) / 2;
vnom = [270; 370; 270; 370; 200; 370; 0; 0; 0; 0; -y0; alphabase];
m = size(vnom, 1);

if simulation == 1
    u = zeros(1, n);
    v = zeros(1, n);
else
    u1 = [546.11,   1040.5,     548         1044.4  1048    549.17      1053.65     545.59];
    u2 = [546.11,   1040.52,    547.98      1044.43 1047.96 549.2       1053.64     545.6];
    v1 = [2122.87,  3124.33,    4123.45     5123.56 6124.74 7122.86     8125.27     9128.57];
    v2 = [2122.86,  3124.32,    4123.5      5123.56 6124.78 7122.84     8125.29     9128.58];
    u = (u1 + u2) /2;
    v = (v1 + v2) /2;
end

for i = 1 : n
    pBase = [samples(2*i - 1);samples(2*i)];
    phi = ik_sym(pBase, vnom);
    phis = [phis, phi];
end

times = 100;
max_error = 0;
max_difference = [];
max_vactual = [];
while times > 0
    times = times - 1;
    if simulation == 1
        %        L11  L12   L21 L22    D   L23      GAMA       DELTA1       DELTA2    X0 Y0     ALPHA;
        upper = [0.5; 0.5; 0.5; 0.5; 0.5; 3.0; deg2rad(3.0); deg2rad(2) ; deg2rad(2); 3; 10; deg2rad(3)];
        lower = -upper;
        random = lower + (upper - lower).*rand(12, 1);
        %random = [0.1868; -0.3165; -0.1315; 0.1256; 0.2802; -2.5132; 0.0450; 0.0192; -0.0009; -0.3848; -1.0643; -0.0203];
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
            if simulation == 1
                means(2*counter-1) = xWorld - 0.001 + 0.002 * rand;
                means(2*counter) = yWorld - 0.001 + 0.002 * rand;
            else
                means(2*counter-1) = xWorld;
                means(2*counter) = yWorld;
            end
            
            tmpphis(:, counter) = phis(:, i);
        end
    end
    
    N = n - 1;
    est = zeros(N, 1);
    
    
    vreal = vnom;
    for i = 1:6
        J = zeros(2*N, m);
        for k = 1 : N %populate matrix J ; estimate est vector
            w_phi = [vreal; tmpphis(1, k); tmpphis(2, k)];
            Jk = subs(Jk_formula, [L11; L12; L21; L22; D; L23; GAMA; DELTA1; DELTA2; X0; Y0; ALPHA; T1; T2], w_phi);
            J(2*k-1,:) = Jk(1,:);
            J(2*k,:) = Jk(2,:);
            [xtmp, ytmp] = fk_sym(vreal, [tmpphis(1, k); tmpphis(2, k)]);
            [xWorldTmp, yWorldTmp] = base2world(vreal, [xtmp, ytmp]);
            est(2*k-1) = xWorldTmp;
            est(2*k) = yWorldTmp;
        end
        
        if i == 1
            sigmas = svd(J);
            product = prod(sigmas);
            O1 = (product ^(1/m)) / sqrt(N);
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
        
        RMSE = sqrt((y' * y )/ (2*N));
        fprintf("[%d] RMSE=%f.\n", i, RMSE);
        if (RMSE < 1E-16)
            fprintf("converge criterion meets.i=%d.\n", i);
            break;
        end
    end
    %disp(vreal');
    
    if simulation == 1
       disp(vactual');
       disp((vreal - vactual)');
    end
    
    difference = vreal - vactual;
    diff_len = norm(difference);
    if (diff_len > max_error)
        max_error = diff_len;
        max_difference = difference;
        max_vactual = vactual;
    end
    
    fprintf("times = %d. max_difference:\n", times);
    disp(max_difference');
    disp(max_vactual');
    fprintf("max_error = %f. O1=%f:\n", max_error, O1);
end
