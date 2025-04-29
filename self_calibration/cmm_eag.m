function Fx = cmm_eag(x)
%aggregated pose error vector
global measures_ag t1s t2s;
Fx = zeros(size(measures_ag));
samples = length(measures_ag) / 2;

Fx(1:samples) = measures_ag(1:samples) - cmm_xWF_func(x, t1s, t2s);
Fx(samples + 1:2*samples) = measures_ag(samples + 1:2*samples) - cmm_yWF_func(x, t1s, t2s);
end


