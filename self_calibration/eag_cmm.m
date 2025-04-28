function Fx = eag_cmm(x)
%aggregated pose error vector
global measures_ag t1s t2s;
Fx = zeros(size(measures_ag));
samples = length(measures_ag) / 2;

%matlabFunction(xWF,'File','xWF_func');
Fx(1:samples) = measures_ag(1:samples) - xWF_func(x, t1s, t2s);
Fx(samples + 1:2*samples) = measures_ag(samples + 1:2*samples) - yWF_func(x, t1s, t2s);
end


