function cose1 = self_cose1_func(x,T1,T2)
%D=200 matlabFunction(COSE1, 'File', 'self_cose1_func')����
L11 = x(1);
L12 = x(2);
L21 = x(3);
L22 = x(4);
DELTA1 = x(5);
DELTA2 = x(6);
DELTA3 = x(7);
DELTA4 = x(8);
%COSE1_FUNC
%    COSE1 = COSE1_FUNC(DELTA1,DELTA2,DELTA3,L11,L12,L21,L22,T1,T2)

%    This function was generated by the Symbolic Math Toolbox version 8.2.
%    03-Apr-2025 15:31:17

t2 = DELTA1+T1;
t3 = cos(t2);
t5 = DELTA2+T2;
t7 = L11.*t3;
t8 = cos(t5);
t9 = L21.*t8;
t4 = -t7+t9+2.0e2;
t11 = sin(t2);
t12 = L11.*t11;
t13 = sin(t5);
t14 = L21.*t13;
t6 = t12-t14;
t10 = t4.^2;
t15 = t6.^2;
t16 = t10+t15;
t17 = 1.0./t16;
t18 = L12.^2;
t19 = L22.^2;
t20 = t10+t15+t18-t19;
t21 = 1.0./sqrt(t16);
t22 = t20.^2;
t27 = (t17.*t22)./4.0;
t23 = t18-t27;
t24 = sqrt(t23);
t26 = (t4.*t17.*t20)./2.0;
t28 = t6.*t21.*t24;
t29 = t26+t28;
t30 = L11.*t3.*t29;
t31 = (t6.*t17.*t20)./2.0;
t32 = t4.*t21.*t24;
t33 = t31-t32;
t34 = L11.*t11.*t33;
t25 = t30-t34;
cose1 = sin(DELTA3).*sqrt(-1.0./L11.^2.*1.0./L12.^2.*t25.^2+1.0)-(t25.*cos(DELTA3))./(L11.*L12);
