function [fp_C1o, fp_C1i, fp_C2o, fp_C2i, fp_CCoin_U, fp_CCoin_D, fp_CCol] = loci(ax, rs)
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
fp_C1o = fimplicit(ax, C1o, '-b');
fp_C1i = fimplicit(ax, C1i, '-b');
fp_C2o = fimplicit(ax, C2o, '-b');
fp_C2i = fimplicit(ax, C2i, '-b');


%Uncertainty Singularity
%Type a Equation (25) Fig.6
CCoin_U = @(x,y) x.*x + (y-sqrt(r1*r1 - r3*r3)).^2 - r2 * r2;
fp_CCoin_U = fimplicit(ax, CCoin_U, '-k');

CCoin_D = @(x,y) x.*x + (y+sqrt(r1*r1 - r3*r3)).^2 - r2 * r2;
fp_CCoin_D = fimplicit(ax, CCoin_D, '-k');

%Type b Equation (28) Fig.7
% fiib = @(x,y) ((x + 2 * r3).^2) .* ((x.^2 + 4 * r3 * x + 3 * r3* r3 + r2 * r2 - r1 * r1 + y.^2).^2) + (y.^2).*((x.^2 + 4 * r3*x + 5 * r3*r3 + r2*r2 - r1*r1 + y.^2).^2) - 4*r3*r3*r2*r2*(y.^2);
CCol = @(x,y) ((x).^2) .* ((x.^2 - r3 * r3 + r2 * r2 - r1 * r1 + y.^2).^2) + (y.^2).*((x.^2 + r3 * r3 + r2 * r2 - r1*r1 + y.^2).^2) - 4*r3*r3*r2*r2*(y.^2);
fp_CCol = fimplicit(ax, CCol, '-r');

end

