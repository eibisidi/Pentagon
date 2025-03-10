function [xBase, yBase] = world2base(w,pE_WF)
syms X0 Y0 ;
sym ALPHA;

X0 = w(11); 
Y0 = w(12);
ALPHA = w(10);

pTmp = pE_WF - [X0; Y0];
rotation = [cos(ALPHA) sin(ALPHA); -sin(ALPHA) cos(ALPHA)];
basePoint = rotation * pTmp;
xBase = basePoint(1);
yBase = basePoint(2);
end

