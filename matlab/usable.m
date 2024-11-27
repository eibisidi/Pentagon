axis equal
hold on
set(gca, 'XAxisLocation', 'origin');
set(gca, 'YAxisLocation', 'origin');

%Fig.19
r1 = 1.2;
r2 = 1.0;
r3 = 0.8;

%Workspace boundary
C1o = @(x,y) (x + r3).^2 + y.^2 - (r1 + r2)^2;   %Equation (13)
C1i = @(x,y) (x + r3).^2 + y.^2 - (r1 - r2)^2;   %Equation (14)
C2o = @(x,y) (x - r3).^2 + y.^2 - (r1 + r2)^2;   %Equation (15)
C2i = @(x,y) (x - r3).^2 + y.^2 - (r1 - r2)^2;   %Equation (16)
fimplicit(C1o, '-b')
fimplicit(C1i, '-b')
fimplicit(C2o, '-b')
fimplicit(C2i, '-b')


%Uncertainty Singularity
%Type a Equation (25) Fig.6
fiia1 = @(x,y) x.*x + (y-sqrt(r1*r1 - r3*r3)).^2 - r2 * r2;
fimplicit(fiia1, '-k')

fiia2 = @(x,y) x.*x + (y+sqrt(r1*r1 - r3*r3)).^2 - r2 * r2;
fimplicit(fiia2, '-k')

%Type b Equation (28) Fig.7
% fiib = @(x,y) ((x + 2 * r3).^2) .* ((x.^2 + 4 * r3 * x + 3 * r3* r3 + r2 * r2 - r1 * r1 + y.^2).^2) + (y.^2).*((x.^2 + 4 * r3*x + 5 * r3*r3 + r2*r2 - r1*r1 + y.^2).^2) - 4*r3*r3*r2*r2*(y.^2);
fiib = @(x,y) ((x).^2) .* ((x.^2 - r3 * r3 + r2 * r2 - r1 * r1 + y.^2).^2) + (y.^2).*((x.^2 + r3 * r3 + r2 * r2 - r1*r1 + y.^2).^2) - 4*r3*r3*r2*r2*(y.^2);
fimplicit(fiib, '-r')
