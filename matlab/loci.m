function [] = loci(ax, rs)
%loci Draw all singularity points curve
%   ax: axis to plot into
%   rs: [r1; r2; r3] vector
r1 = rs(1);
r2 = rs(2);
r3 = rs(3);

%axis equal
hold(ax,'on')
set(ax, 'XAxisLocation', 'origin');
set(ax, 'YAxisLocation', 'origin');

%title(sprintf('r1=%0.2f r2=%0.2f r3=%0.2f', r1, r2, r3))

%Workspace boundary
C1o = @(x,y) (x + r3).^2 + y.^2 - (r1 + r2)^2;   %Equation (13)
C1i = @(x,y) (x + r3).^2 + y.^2 - (r1 - r2)^2;   %Equation (14)
C2o = @(x,y) (x - r3).^2 + y.^2 - (r1 + r2)^2;   %Equation (15)
C2i = @(x,y) (x - r3).^2 + y.^2 - (r1 - r2)^2;   %Equation (16)
fimplicit(ax, C1o, '-b')
fimplicit(ax, C1i, '-b')
fimplicit(ax, C2o, '-b')
fimplicit(ax, C2i, '-b')


%Uncertainty Singularity
%Type a Equation (25) Fig.6
CCoin_U = @(x,y) x.*x + (y-sqrt(r1*r1 - r3*r3)).^2 - r2 * r2;
fimplicit(ax, CCoin_U, '-k')

CCoin_D = @(x,y) x.*x + (y+sqrt(r1*r1 - r3*r3)).^2 - r2 * r2;
fimplicit(ax, CCoin_D, '-k')

%Type b Equation (28) Fig.7
% fiib = @(x,y) ((x + 2 * r3).^2) .* ((x.^2 + 4 * r3 * x + 3 * r3* r3 + r2 * r2 - r1 * r1 + y.^2).^2) + (y.^2).*((x.^2 + 4 * r3*x + 5 * r3*r3 + r2*r2 - r1*r1 + y.^2).^2) - 4*r3*r3*r2*r2*(y.^2);
CCol = @(x,y) ((x).^2) .* ((x.^2 - r3 * r3 + r2 * r2 - r1 * r1 + y.^2).^2) + (y.^2).*((x.^2 + r3 * r3 + r2 * r2 - r1*r1 + y.^2).^2) - 4*r3*r3*r2*r2*(y.^2);
fimplicit(ax, CCol, '-r')

end

