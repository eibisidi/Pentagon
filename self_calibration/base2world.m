function [wf] = base2world(w, pE_Base)
syms X0 Y0 ;
sym ALPHA;

ALPHA = w(10);
X0 = w(11); 
Y0 = w(12);


xBF = pE_Base(1);
yBF = pE_Base(2);
pWF = [X0 + xBF * cos(ALPHA) - yBF * sin(ALPHA); Y0 + xBF * sin(ALPHA) + yBF * cos(ALPHA)];
xWorld = pWF(1);
yWorld = pWF(2);
wf = [xWorld; yWorld];
end

